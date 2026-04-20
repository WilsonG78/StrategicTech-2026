"""Phase 1 – lawnmower scout over the field with live CV detection.

Flight executor drives the pre-computed waypoint list while a daemon
thread consumes camera frames, runs HSV + contour filtering to spot
grey storage crates, and confirms a candidate only after it appears
in at least ``detection_consecutive_frames`` frames in a row.  Each
confirmed candidate is converted to a GPS fix and de-duplicated
against earlier hits.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None

from .camera_control import FrameGrabber
from .gps_utils import (
    GpsPoint,
    apply_drift_correction,
    ground_sample_distance_m_per_px,
    haversine_distance_m,
    lawnmower_waypoints_ned,
    ned_offset_to_gps,
    pixel_offset_to_ned_m,
)
from .mavsdk_controller import MavsdkController
from .ros_publisher import ConfirmedTarget, MissionContext

logger = logging.getLogger(__name__)


@dataclass
class Phase1Config:
    field_length_m: float
    field_width_m: float
    scan_altitude_m: float
    scan_speed_m_s: float
    scan_overlap_pct: float
    camera_fov_h_deg: float
    sensor_width_mm: float
    focal_length_mm: float
    image_width_px: int
    image_height_px: int
    hsv_lower: Tuple[int, int, int]
    hsv_upper: Tuple[int, int, int]
    min_contour_area: float
    max_contour_area: float
    aspect_min: float
    aspect_max: float
    consecutive_frames: int
    dedup_radius_m: float
    home_gps: GpsPoint
    drift_lat_deg: float
    drift_lon_deg: float
    waypoint_tolerance_m: float


@dataclass
class _Candidate:
    center_px: Tuple[int, int]
    streak: int
    last_seen_mono: float


class _CrateDetector:
    """HSV + contour filter with consecutive-frame confirmation."""

    def __init__(self, cfg: Phase1Config) -> None:
        self.cfg = cfg
        self._candidates: List[_Candidate] = []
        self._match_radius_px = max(cfg.image_width_px, cfg.image_height_px) * 0.08

    def process(
        self, frame_bgr: np.ndarray
    ) -> Tuple[List[Tuple[int, int, int, int, int, int]], np.ndarray]:
        """Return (confirmed_boxes, annotated_frame)."""
        if cv2 is None:
            return [], frame_bgr

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array(self.cfg.hsv_lower, dtype=np.uint8),
                           np.array(self.cfg.hsv_upper, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        annotated = frame_bgr.copy()
        matched_this_frame: List[Tuple[int, int, int, int]] = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.cfg.min_contour_area or area > self.cfg.max_contour_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if h == 0:
                continue
            aspect = w / float(h)
            if aspect < self.cfg.aspect_min or aspect > self.cfg.aspect_max:
                continue
            matched_this_frame.append((x, y, w, h))
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 255), 2)

        confirmed: List[Tuple[int, int, int, int, int, int]] = []
        now = time.monotonic()
        next_candidates: List[_Candidate] = []
        used_matches = set()

        for cand in self._candidates:
            best_idx = -1
            best_dist = self._match_radius_px
            for i, (x, y, w, h) in enumerate(matched_this_frame):
                if i in used_matches:
                    continue
                cx, cy = x + w // 2, y + h // 2
                dist = math.hypot(cx - cand.center_px[0], cy - cand.center_px[1])
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            if best_idx >= 0:
                used_matches.add(best_idx)
                x, y, w, h = matched_this_frame[best_idx]
                cx, cy = x + w // 2, y + h // 2
                cand.center_px = (cx, cy)
                cand.streak += 1
                cand.last_seen_mono = now
                next_candidates.append(cand)
                if cand.streak >= self.cfg.consecutive_frames:
                    confirmed.append((x, y, w, h, cx, cy))
                    cv2.rectangle(annotated, (x, y), (x + w, y + h),
                                  (0, 0, 255), 3)
                    cv2.putText(annotated, "CRATE", (x, max(0, y - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                if now - cand.last_seen_mono < 0.8:
                    next_candidates.append(cand)

        for i, (x, y, w, h) in enumerate(matched_this_frame):
            if i in used_matches:
                continue
            cx, cy = x + w // 2, y + h // 2
            next_candidates.append(
                _Candidate(center_px=(cx, cy), streak=1, last_seen_mono=now))

        self._candidates = next_candidates
        return confirmed, annotated


class Phase1Scan:
    """Fly lawnmower, detect crates, publish annotated frames."""

    def __init__(
        self,
        controller: MavsdkController,
        grabber: FrameGrabber,
        publisher,  # MissionPublisher
        cfg: Phase1Config,
        ctx: MissionContext,
    ) -> None:
        self.controller = controller
        self.grabber = grabber
        self.publisher = publisher
        self.cfg = cfg
        self.ctx = ctx

        self._targets: List[ConfirmedTarget] = []
        self._targets_lock = threading.Lock()
        self._cv_thread: Optional[threading.Thread] = None
        self._cv_running = threading.Event()
        self._latest_annotated: Optional[np.ndarray] = None
        self._annotated_lock = threading.Lock()

    @property
    def targets(self) -> List[ConfirmedTarget]:
        with self._targets_lock:
            return list(self._targets)

    def latest_annotated(self) -> Optional[np.ndarray]:
        with self._annotated_lock:
            return None if self._latest_annotated is None else self._latest_annotated.copy()

    async def execute(self) -> List[ConfirmedTarget]:
        waypoints = lawnmower_waypoints_ned(
            field_length_m=self.cfg.field_length_m,
            field_width_m=self.cfg.field_width_m,
            altitude_m=self.cfg.scan_altitude_m,
            fov_horizontal_deg=self.cfg.camera_fov_h_deg,
            overlap_pct=self.cfg.scan_overlap_pct,
        )
        logger.info("Phase 1 waypoints: %d", len(waypoints))

        self._cv_running.set()
        self._cv_thread = threading.Thread(
            target=self._cv_loop, name="phase1-cv", daemon=True)
        self._cv_thread.start()

        try:
            await self.controller.start_offboard()
            for i, (n, e, d) in enumerate(waypoints):
                logger.info("WP %d/%d -> N=%.1f E=%.1f D=%.1f",
                            i + 1, len(waypoints), n, e, d)
                await self.controller.goto_ned(
                    n, e, d,
                    yaw_deg=0.0,
                    tolerance_m=self.cfg.waypoint_tolerance_m,
                    timeout_s=90.0,
                )
                self.ctx.flight_path_ned.append(
                    (self.controller.state.north_m,
                     self.controller.state.east_m))
        finally:
            self._cv_running.clear()
            if self._cv_thread is not None:
                self._cv_thread.join(timeout=2.0)

        return self.targets

    # ── CV thread ────────────────────────────────────────────────────────────

    def _cv_loop(self) -> None:
        detector = _CrateDetector(self.cfg)
        while self._cv_running.is_set():
            frame = self.grabber.latest()
            if frame is None:
                time.sleep(0.05)
                continue

            confirmed, annotated = detector.process(frame)
            with self._annotated_lock:
                self._latest_annotated = annotated

            if not confirmed:
                time.sleep(0.02)
                continue

            alt = self.controller.state.altitude_rel_m
            if not math.isfinite(alt) or alt < 0.3:
                continue

            try:
                gsd = ground_sample_distance_m_per_px(
                    alt,
                    self.cfg.sensor_width_mm,
                    self.cfg.focal_length_mm,
                    self.cfg.image_width_px,
                )
            except ValueError:
                continue

            heading = self.controller.state.heading_deg or 0.0
            drone_ned_n = self.controller.state.north_m
            drone_ned_e = self.controller.state.east_m
            drone_gps = apply_drift_correction(
                ned_offset_to_gps(
                    self.cfg.home_gps, drone_ned_n, drone_ned_e),
                self.cfg.drift_lat_deg, self.cfg.drift_lon_deg)

            for (_x, _y, _w, _h, cx, cy) in confirmed:
                dx_px = cx - self.cfg.image_width_px / 2.0
                dy_px = cy - self.cfg.image_height_px / 2.0
                n_off, e_off = pixel_offset_to_ned_m(
                    dx_px, dy_px, alt, gsd, heading_deg=heading)
                target_gps = ned_offset_to_gps(
                    drone_gps, n_off, e_off)
                self._maybe_register_target(
                    target_gps,
                    ned_north=drone_ned_n + n_off,
                    ned_east=drone_ned_e + e_off,
                    annotated=annotated,
                )
            time.sleep(0.02)

    def _maybe_register_target(
        self,
        gps: GpsPoint,
        ned_north: float,
        ned_east: float,
        annotated: Optional[np.ndarray],
    ) -> None:
        with self._targets_lock:
            for existing in self._targets:
                if haversine_distance_m(existing.gps, gps) < self.cfg.dedup_radius_m:
                    return
            target = ConfirmedTarget(
                index=len(self._targets) + 1,
                gps=gps,
                ned_north_m=ned_north,
                ned_east_m=ned_east,
            )
            self._targets.append(target)
            self.ctx.targets.append(target)
            logger.info("Phase 1 target #%d locked at lat=%.6f lon=%.6f",
                        target.index, gps.latitude, gps.longitude)

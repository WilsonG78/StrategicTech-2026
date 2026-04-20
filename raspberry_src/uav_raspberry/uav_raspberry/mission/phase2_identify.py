"""Phase 2 – visit each confirmed target and identify it.

Per target:
  1. Descend to ``identify_altitude_m`` above the target GPS.
  2. Engage a camera lock: P-controller on pixel offset drives NED
     velocity setpoints, keeping the crate centred in the frame.
  3. Hold for ``hold_seconds`` once the crate is stable.
  4. Meanwhile: run YOLOv8n on the frame in a thread-pool executor and
     attempt QR decode with pyzbar.
  5. Publish a DetectionReport, Detection (organizer), annotated image,
     NavSatFix, and updated mission map.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None

try:
    from pyzbar.pyzbar import decode as pyzbar_decode  # type: ignore
except ImportError:  # pragma: no cover
    pyzbar_decode = None

try:
    from ultralytics import YOLO  # type: ignore
except ImportError:  # pragma: no cover
    YOLO = None

from .camera_control import FrameGrabber
from .mavsdk_controller import MavsdkController
from .ros_publisher import ConfirmedTarget, MissionContext, MissionPublisher

logger = logging.getLogger(__name__)


@dataclass
class Phase2Config:
    identify_altitude_m: float
    waypoint_tolerance_m: float
    lock_kp: float
    lock_max_velocity_m_s: float
    lock_center_tolerance_px: int
    lock_hold_seconds_min: float
    lock_hold_seconds_max: float
    lock_stable_frames: int
    image_width_px: int
    image_height_px: int
    yolo_model_path: str
    yolo_confidence_threshold: float
    yolo_imgsz: int
    qr_max_attempts: int


class Phase2Identify:
    def __init__(
        self,
        controller: MavsdkController,
        grabber: FrameGrabber,
        publisher: MissionPublisher,
        cfg: Phase2Config,
        ctx: MissionContext,
    ) -> None:
        self.controller = controller
        self.grabber = grabber
        self.publisher = publisher
        self.cfg = cfg
        self.ctx = ctx
        self._yolo = self._load_yolo()
        self._executor = ThreadPoolExecutor(max_workers=2)

    def _load_yolo(self):
        if YOLO is None:
            logger.warning("ultralytics not importable - YOLO disabled")
            return None
        import os
        path = os.path.expanduser(self.cfg.yolo_model_path)
        if not os.path.exists(path):
            logger.warning("YOLO model missing at %s - skipping classification", path)
            return None
        try:
            return YOLO(path)
        except Exception as exc:  # noqa: BLE001
            logger.error("YOLO load failed: %s", exc)
            return None

    # ── public ───────────────────────────────────────────────────────────────

    async def execute(self, targets: List[ConfirmedTarget]) -> None:
        if not targets:
            logger.warning("Phase 2 - no targets to visit")
            return
        for target in targets:
            logger.info("Phase 2 - visiting target #%d", target.index)
            try:
                await self._visit_target(target)
            except Exception as exc:  # noqa: BLE001
                logger.exception("Phase 2 target #%d failed: %s",
                                 target.index, exc)

    async def _visit_target(self, target: ConfirmedTarget) -> None:
        down_m = -self.cfg.identify_altitude_m
        yaw = self.controller.state.heading_deg or 0.0

        await self.controller.goto_ned(
            target.ned_north_m, target.ned_east_m, down_m,
            yaw_deg=yaw,
            tolerance_m=self.cfg.waypoint_tolerance_m,
            timeout_s=60.0,
        )

        stable_frames = 0
        lock_deadline = asyncio.get_event_loop().time() + 8.0
        while asyncio.get_event_loop().time() < lock_deadline:
            frame = self.grabber.latest()
            if frame is None:
                await asyncio.sleep(0.05)
                continue
            offset = self._locate_crate(frame)
            if offset is None:
                await self.controller.set_velocity_ned(0.0, 0.0, 0.0, yaw)
                await asyncio.sleep(0.05)
                continue
            dx_px, dy_px = offset
            if (abs(dx_px) <= self.cfg.lock_center_tolerance_px
                    and abs(dy_px) <= self.cfg.lock_center_tolerance_px):
                stable_frames += 1
                await self.controller.set_velocity_ned(0.0, 0.0, 0.0, yaw)
                if stable_frames >= self.cfg.lock_stable_frames:
                    break
            else:
                stable_frames = 0
                vn, ve = self._p_controller(dx_px, dy_px, yaw)
                await self.controller.set_velocity_ned(vn, ve, 0.0, yaw)
            await asyncio.sleep(0.05)

        hold_seconds = 0.5 * (self.cfg.lock_hold_seconds_min
                              + self.cfg.lock_hold_seconds_max)
        logger.info("Lock acquired - holding %.1fs for ID", hold_seconds)

        best_class = ""
        qr_data = ""
        best_conf = 0.0
        best_frame: Optional[np.ndarray] = None
        hold_deadline = asyncio.get_event_loop().time() + hold_seconds
        qr_attempts = 0

        while asyncio.get_event_loop().time() < hold_deadline:
            n, e, _d = (self.controller.state.north_m,
                        self.controller.state.east_m,
                        self.controller.state.down_m)
            await self.controller.set_position_ned(n, e, down_m, yaw)

            frame = self.grabber.latest()
            if frame is not None:
                if best_frame is None:
                    best_frame = frame

                if qr_attempts < self.cfg.qr_max_attempts and not qr_data:
                    qr_attempts += 1
                    qr_data = await asyncio.get_event_loop().run_in_executor(
                        self._executor, self._decode_qr, frame)

                if self._yolo is not None and not best_class:
                    cls, conf = await asyncio.get_event_loop().run_in_executor(
                        self._executor, self._classify, frame)
                    if cls and conf > best_conf:
                        best_class, best_conf = cls, conf
                        best_frame = frame
            await asyncio.sleep(0.1)

        target.qr_data = qr_data
        target.object_type = best_class or "unknown"
        annotated = self._annotate(best_frame if best_frame is not None
                                   else self.grabber.latest(),
                                   target)
        self.publisher.publish_detection(target, self.ctx, annotated)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _locate_crate(
        self, frame_bgr: np.ndarray
    ) -> Optional[Tuple[float, float]]:
        if cv2 is None:
            return None
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _ret, mask = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 500:
            return None
        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x + w // 2, y + h // 2
        return (cx - self.cfg.image_width_px / 2.0,
                cy - self.cfg.image_height_px / 2.0)

    def _p_controller(
        self, dx_px: float, dy_px: float, heading_deg: float
    ) -> Tuple[float, float]:
        vx_body = self.cfg.lock_kp * dx_px            # body east
        vy_body = -self.cfg.lock_kp * dy_px           # body north (image-up)
        hdg = math.radians(heading_deg)
        vn = vy_body * math.cos(hdg) - vx_body * math.sin(hdg)
        ve = vy_body * math.sin(hdg) + vx_body * math.cos(hdg)
        v_max = self.cfg.lock_max_velocity_m_s
        mag = math.hypot(vn, ve)
        if mag > v_max:
            scale = v_max / mag
            vn *= scale
            ve *= scale
        return vn, ve

    def _decode_qr(self, frame_bgr: np.ndarray) -> str:
        if pyzbar_decode is None:
            return ""
        try:
            results = pyzbar_decode(frame_bgr)
            for r in results:
                data = r.data.decode("utf-8", errors="replace").strip()
                if data:
                    return data
        except Exception as exc:  # noqa: BLE001
            logger.debug("pyzbar error: %s", exc)
        return ""

    def _classify(self, frame_bgr: np.ndarray) -> Tuple[str, float]:
        if self._yolo is None:
            return ("", 0.0)
        try:
            results = self._yolo.predict(
                source=frame_bgr,
                imgsz=self.cfg.yolo_imgsz,
                conf=self.cfg.yolo_confidence_threshold,
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug("YOLO predict failed: %s", exc)
            return ("", 0.0)
        if not results:
            return ("", 0.0)
        r0 = results[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return ("", 0.0)
        idx = int(r0.boxes.conf.argmax().item())
        cls_id = int(r0.boxes.cls[idx].item())
        conf = float(r0.boxes.conf[idx].item())
        name = r0.names.get(cls_id, str(cls_id))
        return (name, conf)

    def _annotate(
        self,
        frame: Optional[np.ndarray],
        target: ConfirmedTarget,
    ) -> Optional[np.ndarray]:
        if cv2 is None or frame is None:
            return frame
        out = frame.copy()
        lines = [
            f"type: {target.object_type}",
            f"qr:   {target.qr_data or '-'}",
            f"gps:  {target.gps.latitude:.6f}, {target.gps.longitude:.6f}",
        ]
        y = 25
        for line in lines:
            cv2.putText(out, line, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(out, line, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            y += 26
        return out

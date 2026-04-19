"""scout_mission.py — two-phase autonomous reconnaissance mission.

Phase 1 – FAST_SCOUT
    Fly the lawnmower grid at scout speed.  OpenCV first tries to detect ArUco
    markers directly in each frame.  When the marker is too small to decode at
    altitude, white-blob detection is used as a coarse fallback.  The drone
    does NOT stop — it only records the NED coordinates of every hit.

Phase 2 – INSPECT_TARGETS
    After completing the grid, sort the collected positions with a
    nearest-neighbour heuristic and visit each one.  At every stop the drone
    descends, centres itself over the ArUco marker, reads its ID, publishes a
    full DetectionReport (annotated photo + 2D map), then climbs and continues.

Safety
    A dedicated asyncio task streams flight_mode telemetry.  Any manual RC
    switch-over (mode not in _ALLOWED_MODES while on a mission) immediately
    enters STATE_MANUAL_OVERRIDE and freezes all offboard commands.

Architecture
    rclpy.spin()  →  main thread  (image cb, debug pub)
    asyncio loop  →  daemon thread (all MAVSDK / offboard commands)
    Shared state protected by threading.Lock.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
from mavsdk.telemetry import FlightMode

from uav_msgs.msg import DetectionReport, UavTelemetry

from .math_utils import (
    DEFAULT_ARUCO_DICT,
    NedPoint,
    annotate_detection_photo,
    detect_white_blobs,
    generate_lawnmower,
    generate_map_image,
    nearest_neighbor_sort,
    ned_to_global,
    pixel_to_ned_offset,
)

# ── Constants ─────────────────────────────────────────────────────────────────

_ALLOWED_MODES: frozenset = frozenset(
    {FlightMode.OFFBOARD, FlightMode.HOLD, FlightMode.TAKEOFF}
)

OBJECT_TYPE_PLACEHOLDER = "UNKNOWN_3D_MODEL"

# ── State Machine ─────────────────────────────────────────────────────────────


class State(Enum):
    TAKEOFF          = auto()
    FAST_SCOUT       = auto()   # Phase 1
    INSPECT_TARGETS  = auto()   # Phase 2 setup (sort targets)
    APPROACH         = auto()   # flying to next target
    DESCENDING       = auto()   # descending over target
    ALIGNING         = auto()   # centering on ArUco marker
    SCANNING         = auto()   # hovering, reading marker, publishing report
    RESUMING         = auto()   # climbing back to patrol altitude
    RETURNING        = auto()   # flying back to home position at patrol altitude
    LANDING          = auto()   # executing ArduPilot land at home
    MANUAL_OVERRIDE  = auto()   # RC took over — freeze all offboard
    DONE             = auto()   # landed, mission finished


_MISSION_STATES: frozenset = frozenset({
    State.TAKEOFF, State.FAST_SCOUT, State.INSPECT_TARGETS,
    State.APPROACH, State.DESCENDING, State.ALIGNING,
    State.SCANNING, State.RESUMING, State.RETURNING, State.LANDING,
})


# ── Node ──────────────────────────────────────────────────────────────────────


class ScoutMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("scout_mission_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("dry_run",             False)
        self.declare_parameter("mavsdk_address",      "udpin://127.0.0.1:14552")
        self.declare_parameter("patrol_alt_m",        3.0)
        self.declare_parameter("scan_alt_m",          2.5)
        self.declare_parameter("scout_speed_m_s",     1.5)
        self.declare_parameter("waypoint_tol_scout",  1.2)
        self.declare_parameter("waypoint_tol_inspect",0.6)
        self.declare_parameter("camera_fov_deg",      70.0)
        self.declare_parameter("p_gain",              0.3)
        self.declare_parameter("max_velocity",        0.5)
        self.declare_parameter("center_threshold_px", 15)
        self.declare_parameter("wait_time_sec",       5.0)
        self.declare_parameter("grid_length_m",       10.0)
        self.declare_parameter("grid_width_m",        10.0)
        self.declare_parameter("line_spacing_m",      2.0)
        self.declare_parameter("min_blob_area_px",    800.0)
        self.declare_parameter("blob_dedup_radius_m", 1.0)
        self.declare_parameter("aruco_dict_id",       int(DEFAULT_ARUCO_DICT))

        # ── ROS interfaces ─────────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub_img = self.create_subscription(
            Image, "/ugv_camera/image_raw/compressed", self._image_cb, qos_be
        )
        self._pub_qr     = self.create_publisher(String, "/qr/detected", 10)
        self._pub_debug  = self.create_publisher(
            CompressedImage, "/qr/debug_image", qos_be
        )
        self._pub_report = self.create_publisher(
            DetectionReport, "/detection_report", 10
        )

        self._bridge = CvBridge()

        # ArUco detector — dictionary and parameters created once, reused every frame
        _dict_id           = self.get_parameter("aruco_dict_id").value
        self._aruco_dict   = cv2.aruco.Dictionary_get(_dict_id)
        self._aruco_params = cv2.aruco.DetectorParameters_create()
        self.get_logger().info(f"ArUco detector ready (dict_id={_dict_id}).")

        # ── Shared state ───────────────────────────────────────────────────────
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._img_h: int = 480
        self._img_w: int = 640

        self._tel_lock  = threading.Lock()
        self._rel_alt   = 0.0
        self._north_m   = 0.0
        self._east_m    = 0.0
        self._lat       = 0.0
        self._lon       = 0.0

        self._sm_lock   = threading.Lock()
        self._state: State = State.TAKEOFF

        # ── Lawnmower ─────────────────────────────────────────────────────────
        self._waypoints: List[NedPoint] = generate_lawnmower(
            self.get_parameter("grid_length_m").value,
            self.get_parameter("grid_width_m").value,
            self.get_parameter("line_spacing_m").value,
        )
        self._wp_idx = 0
        self.get_logger().info(
            f"Lawnmower: {len(self._waypoints)} waypoints generated."
        )

        # ── Scout / inspect bookkeeping ────────────────────────────────────────
        self._detected_blobs: List[NedPoint] = []   # Phase 1 results
        self._inspect_queue:  List[NedPoint] = []   # Phase 2 sorted targets
        self._inspect_idx = 0
        self._current_target: Optional[NedPoint] = None

        # Home position — NED (0, 0) at launch; overwritten once offboard starts
        self._home: NedPoint = (0.0, 0.0)

        # ── Alignment bookkeeping ──────────────────────────────────────────────
        self._centered_since:  Optional[float] = None
        self._wait_started:    Optional[float] = None
        self._aruco_bbox:      Optional[list]  = None   # last detected corners
        self._last_marker_id:  str             = ""     # ArUco ID as string

        # ── asyncio (MAVSDK) in background thread ──────────────────────────────
        self._loop = asyncio.new_event_loop()
        self._bg   = threading.Thread(
            target=self._run_loop, name="mavsdk-scout", daemon=True
        )
        self._bg.start()

        self.get_logger().info("ScoutMissionNode ready.")

    # ── Image callback (ROS main thread) ──────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._latest_frame = frame
                self._img_h, self._img_w = frame.shape[:2]
        except Exception as exc:
            self.get_logger().error(
                f"cv_bridge: {exc}", throttle_duration_sec=5.0
            )

    # ── asyncio bootstrap ──────────────────────────────────────────────────────

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self) -> None:
        url = self.get_parameter("mavsdk_address").value
        self.get_logger().info(f"Connecting MAVSDK at {url} …")

        self._drone = System(port=50053)
        try:
            await self._drone.connect(system_address=url)
            async for cs in self._drone.core.connection_state():
                if cs.is_connected:
                    self.get_logger().info("MAVSDK connected.")
                    break
        except Exception as exc:
            self.get_logger().fatal(f"MAVSDK connect failed: {exc}")
            return

        await asyncio.gather(
            self._task_tel_pos(),
            self._task_tel_alt(),
            self._task_rc_monitor(),
            self._task_mission(),
        )

    # ── Telemetry streams ──────────────────────────────────────────────────────

    async def _task_tel_pos(self) -> None:
        while rclpy.ok():
            try:
                async for pv in self._drone.telemetry.position_velocity_ned():
                    with self._tel_lock:
                        self._north_m = pv.position.north_m
                        self._east_m  = pv.position.east_m
            except Exception as exc:
                self.get_logger().error(f"tel_pos error: {exc}")
            await asyncio.sleep(1.0)

    async def _task_tel_alt(self) -> None:
        while rclpy.ok():
            try:
                async for pos in self._drone.telemetry.position():
                    with self._tel_lock:
                        self._rel_alt = pos.relative_altitude_m
                        self._lat     = pos.latitude_deg
                        self._lon     = pos.longitude_deg
            except Exception as exc:
                self.get_logger().error(f"tel_alt error: {exc}")
            await asyncio.sleep(1.0)

    # ── RC override monitor ────────────────────────────────────────────────────

    async def _task_rc_monitor(self) -> None:
        while rclpy.ok():
            try:
                async for fm in self._drone.telemetry.flight_mode():
                    with self._sm_lock:
                        s = self._state
                    if s in _MISSION_STATES and fm not in _ALLOWED_MODES:
                        with self._sm_lock:
                            if self._state in _MISSION_STATES:
                                self.get_logger().warn(
                                    f"[RC OVERRIDE] Mode={fm.name}. "
                                    "All offboard commands frozen."
                                )
                                self._state = State.MANUAL_OVERRIDE
            except Exception as exc:
                self.get_logger().error(f"rc_monitor error: {exc}")
            await asyncio.sleep(1.0)

    # ── Mission task ───────────────────────────────────────────────────────────

    async def _task_mission(self) -> None:
        while rclpy.ok():
            with self._sm_lock:
                state = self._state

            try:
                if   state == State.TAKEOFF:         await self._do_takeoff()
                elif state == State.FAST_SCOUT:      await self._do_fast_scout()
                elif state == State.INSPECT_TARGETS: await self._do_inspect_setup()
                elif state == State.APPROACH:        await self._do_approach()
                elif state == State.DESCENDING:      await self._do_descending()
                elif state == State.ALIGNING:        await self._do_aligning()
                elif state == State.SCANNING:        await self._do_scanning()
                elif state == State.RESUMING:        await self._do_resuming()
                elif state == State.RETURNING:       await self._do_returning()
                elif state == State.LANDING:         await self._do_landing()
                elif state == State.MANUAL_OVERRIDE: self._publish_debug(state)
                elif state == State.DONE:
                    self.get_logger().info("Mission complete — landed.", once=True)
            except OffboardError as exc:
                self.get_logger().error(f"OffboardError in {state.name}: {exc}")
            except Exception as exc:
                self.get_logger().error(f"Mission error in {state.name}: {exc}")

            await asyncio.sleep(0.05)

    # ── State handlers ─────────────────────────────────────────────────────────

    async def _do_takeoff(self) -> None:
        if self._p("dry_run"):
            self.get_logger().warn(
                "DRY RUN – arm/takeoff blocked. Mission stays in TAKEOFF state."
            )
            await asyncio.sleep(2.0)
            return

        patrol_alt = self._p("patrol_alt_m")
        try:
            self.get_logger().info("Arming …")
            await self._drone.action.arm()
            await self._drone.action.takeoff()
        except Exception as exc:
            self.get_logger().error(f"Arm/takeoff: {exc}")
            await asyncio.sleep(2.0)
            return

        # Wait for patrol altitude
        while rclpy.ok():
            if self._get_state() == State.MANUAL_OVERRIDE:
                return
            if self._alt() >= patrol_alt * 0.90:
                break
            self.get_logger().info(
                f"Climbing … {self._alt():.1f}/{patrol_alt:.1f} m",
                throttle_duration_sec=1.0,
            )
            await asyncio.sleep(0.2)

        # Prime offboard (≥10 setpoints required by MAVSDK before start)
        try:
            for _ in range(10):
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.05)
            await self._drone.offboard.start()
            self.get_logger().info("Offboard started → FAST_SCOUT")
        except OffboardError as exc:
            self.get_logger().error(f"Offboard start: {exc}")
            return

        # Record launch point in NED frame (origin at takeoff = 0,0 always)
        with self._tel_lock:
            self._home = (self._north_m, self._east_m)
        self.get_logger().info(
            f"Home recorded: N={self._home[0]:.2f} E={self._home[1]:.2f}"
        )

        self._wp_idx = 0
        self._set_state(State.FAST_SCOUT)

    async def _do_fast_scout(self) -> None:
        """Phase 1: fly lawnmower at scout speed, record white-blob positions."""
        patrol_alt   = self._p("patrol_alt_m")
        scout_speed  = self._p("scout_speed_m_s")
        tol          = self._p("waypoint_tol_scout")

        if self._wp_idx >= len(self._waypoints):
            self.get_logger().info(
                f"FAST_SCOUT complete. {len(self._detected_blobs)} blobs found."
            )
            self._set_state(State.INSPECT_TARGETS)
            return

        wp_n, wp_e = self._waypoints[self._wp_idx]
        with self._tel_lock:
            n, e = self._north_m, self._east_m

        dist  = math.hypot(wp_n - n, wp_e - e)
        if dist > 0.1:
            vn = (wp_n - n) / dist * scout_speed
            ve = (wp_e - e) / dist * scout_speed
        else:
            vn = ve = 0.0

        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, 0.0, 0.0)
            )
        except OffboardError as exc:
            self.get_logger().error(f"FAST_SCOUT velocity: {exc}")
            return

        # ── Marker detection in current frame ─────────────────────────────
        frame = self._get_frame()
        if frame is not None:
            with self._tel_lock:
                cur_n, cur_e = self._north_m, self._east_m
                cur_alt      = self._rel_alt
            fov = self._p("camera_fov_deg")
            with self._frame_lock:
                iw, ih = self._img_w, self._img_h

            # Primary: ArUco (gives ID + precise centre)
            aruco_corners, aruco_ids, _ = cv2.aruco.detectMarkers(
                frame, self._aruco_dict, parameters=self._aruco_params
            )
            found_via_aruco = 0
            if aruco_ids is not None:
                for corners, mid in zip(aruco_corners, aruco_ids):
                    pts  = corners[0]
                    bx   = float(np.mean(pts[:, 0]))
                    by   = float(np.mean(pts[:, 1]))
                    dn, de = pixel_to_ned_offset(bx, by, iw, ih, cur_alt, fov)
                    blob_n, blob_e = cur_n + dn, cur_e + de
                    if not self._is_duplicate_blob(blob_n, blob_e):
                        self._detected_blobs.append((blob_n, blob_e))
                        self.get_logger().info(
                            f"ArUco ID={int(mid[0])} recorded at "
                            f"N={blob_n:.1f} E={blob_e:.1f}"
                        )
                    found_via_aruco += 1

            # Fallback: white-blob (marker too small to decode at altitude)
            if found_via_aruco == 0:
                blobs = detect_white_blobs(
                    frame, min_area_px=self._p("min_blob_area_px")
                )
                for bx, by, _ in blobs:
                    dn, de = pixel_to_ned_offset(bx, by, iw, ih, cur_alt, fov)
                    blob_n, blob_e = cur_n + dn, cur_e + de
                    if not self._is_duplicate_blob(blob_n, blob_e):
                        self._detected_blobs.append((blob_n, blob_e))
                        self.get_logger().info(
                            f"White-blob (fallback) recorded at "
                            f"N={blob_n:.1f} E={blob_e:.1f}"
                        )

            self._publish_debug(State.FAST_SCOUT)

        # ── Arrival check ──────────────────────────────────────────────────
        if math.hypot(wp_n - n, wp_e - e) < tol:
            self._wp_idx += 1

    async def _do_inspect_setup(self) -> None:
        """Sort detected blobs into nearest-neighbour order and start Phase 2."""
        with self._tel_lock:
            start = (self._north_m, self._east_m)

        if not self._detected_blobs:
            self.get_logger().warn("No blobs detected — returning to home.")
            self._set_state(State.RETURNING)
            return

        self._inspect_queue = nearest_neighbor_sort(self._detected_blobs, start)
        self._inspect_idx   = 0
        self.get_logger().info(
            f"INSPECT_TARGETS: {len(self._inspect_queue)} targets queued."
        )
        self._set_state(State.APPROACH)

    async def _do_approach(self) -> None:
        """Fly to the next inspection target at patrol altitude."""
        if self._inspect_idx >= len(self._inspect_queue):
            self.get_logger().info("All targets inspected → RETURNING")
            self._set_state(State.RETURNING)
            return

        tgt_n, tgt_e = self._inspect_queue[self._inspect_idx]
        self._current_target = (tgt_n, tgt_e)
        patrol_alt = self._p("patrol_alt_m")
        tol        = self._p("waypoint_tol_inspect")

        try:
            await self._drone.offboard.set_position_ned(
                PositionNedYaw(tgt_n, tgt_e, -patrol_alt, 0.0)
            )
        except OffboardError as exc:
            self.get_logger().error(f"APPROACH set_pos: {exc}")
            return

        with self._tel_lock:
            n, e = self._north_m, self._east_m
        if math.hypot(tgt_n - n, tgt_e - e) < tol:
            self.get_logger().info(
                f"Target {self._inspect_idx} reached → DESCENDING"
            )
            self._centered_since = None
            self._set_state(State.DESCENDING)

        frame = self._get_frame()
        if frame is not None:
            self._publish_debug(State.APPROACH)

    async def _do_descending(self) -> None:
        scan_alt = self._p("scan_alt_m")
        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.5, 0.0)   # vz > 0 → down in NED
            )
        except OffboardError as exc:
            self.get_logger().error(f"DESCEND: {exc}")
            return

        if self._alt() <= scan_alt:
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            self._centered_since = None
            self._set_state(State.ALIGNING)

        frame = self._get_frame()
        if frame is not None:
            self._publish_debug(State.DESCENDING)

    async def _do_aligning(self) -> None:
        """P-controller to centre drone over ArUco marker."""
        p_gain    = self._p("p_gain")
        max_vel   = self._p("max_velocity")
        threshold = self._p("center_threshold_px")
        fov_deg   = self._p("camera_fov_deg")

        frame = self._get_frame()
        if frame is None:
            return

        with self._frame_lock:
            iw, ih = self._img_w, self._img_h

        corners_list, ids, _ = cv2.aruco.detectMarkers(
            frame, self._aruco_dict, parameters=self._aruco_params
        )

        if ids is None or len(ids) == 0:
            # No marker visible — hover and wait for it to appear
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            self._centered_since = None
            self._publish_debug(State.ALIGNING)
            return

        # Pick the largest (nearest / most prominent) marker
        areas  = [cv2.contourArea(c[0]) for c in corners_list]
        best   = int(np.argmax(areas))
        pts    = corners_list[best][0]   # shape (4, 2)
        marker_id = int(ids[best][0])

        self._last_marker_id = str(marker_id)
        self._aruco_bbox     = pts.tolist()

        cx    = float(np.mean(pts[:, 0]))
        cy    = float(np.mean(pts[:, 1]))
        off_x = cx - iw / 2.0
        off_y = cy - ih / 2.0

        alt   = max(self._alt(), 0.5)
        fov_r = math.radians(fov_deg)
        scale = 2.0 * alt * math.tan(fov_r / 2.0) / iw

        vn = float(np.clip(-off_y * scale * p_gain, -max_vel, max_vel))
        ve = float(np.clip( off_x * scale * p_gain, -max_vel, max_vel))

        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, 0.0, 0.0)
            )
        except OffboardError as exc:
            self.get_logger().error(f"ALIGN vel: {exc}")

        self._publish_debug(State.ALIGNING, qr_bbox=self._aruco_bbox)
        self.get_logger().debug(
            f"ALIGN  marker={marker_id}  off=({off_x:.0f},{off_y:.0f})px  "
            f"vel=({vn:.2f},{ve:.2f})m/s",
            throttle_duration_sec=0.5,
        )

        if abs(off_x) < threshold and abs(off_y) < threshold:
            now = time.monotonic()
            if self._centered_since is None:
                self._centered_since = now
            elif now - self._centered_since >= 1.0:
                try:
                    await self._drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
                except OffboardError:
                    pass
                self._wait_started = time.monotonic()
                self._set_state(State.SCANNING)
        else:
            self._centered_since = None

    async def _do_scanning(self) -> None:
        """Hover, read ArUco ID, publish report, wait, then resume."""
        wait_sec = self._p("wait_time_sec")
        if self._wait_started is None:
            self._wait_started = time.monotonic()

        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
        except OffboardError:
            pass

        frame = self._get_frame()
        if frame is not None:
            self._publish_debug(State.SCANNING, qr_bbox=self._aruco_bbox)

            # Publish competition detection report once per target
            if self._last_marker_id:
                self._publish_detection_report(frame)
                msg = String(); msg.data = self._last_marker_id
                self._pub_qr.publish(msg)
                self._last_marker_id = ""  # prevent duplicate reports

        if time.monotonic() - self._wait_started >= wait_sec:
            self._inspect_idx += 1
            self._set_state(State.RESUMING)

    async def _do_resuming(self) -> None:
        patrol_alt = self._p("patrol_alt_m")
        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, -0.5, 0.0)  # vz < 0 → climb
            )
        except OffboardError as exc:
            self.get_logger().error(f"RESUME: {exc}")
            return

        if self._alt() >= patrol_alt * 0.97:
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            self._set_state(State.APPROACH)

        frame = self._get_frame()
        if frame is not None:
            self._publish_debug(State.RESUMING)

    async def _do_returning(self) -> None:
        """Fly back to the home (launch) position at patrol altitude."""
        patrol_alt = self._p("patrol_alt_m")
        tol        = self._p("waypoint_tol_inspect")
        home_n, home_e = self._home

        try:
            await self._drone.offboard.set_position_ned(
                PositionNedYaw(home_n, home_e, -patrol_alt, 0.0)
            )
        except OffboardError as exc:
            self.get_logger().error(f"RETURNING set_pos: {exc}")
            return

        with self._tel_lock:
            n, e = self._north_m, self._east_m

        dist = math.hypot(home_n - n, home_e - e)
        self.get_logger().info(
            f"Returning home … dist={dist:.1f} m", throttle_duration_sec=2.0
        )

        if dist < tol:
            self.get_logger().info("Home reached → LANDING")
            self._set_state(State.LANDING)

        frame = self._get_frame()
        if frame is not None:
            self._publish_debug(State.RETURNING)

    async def _do_landing(self) -> None:
        """Stop offboard mode and command ArduPilot to land."""
        try:
            await self._drone.offboard.stop()
        except Exception:
            pass  # offboard stop failure is non-fatal

        try:
            await self._drone.action.land()
            self.get_logger().info("Land command sent — waiting to touch down …")
        except Exception as exc:
            self.get_logger().error(f"Land command failed: {exc}")
            return

        # Wait until the drone is on the ground (relative altitude < 0.3 m)
        while rclpy.ok():
            if self._alt() < 0.3:
                break
            await asyncio.sleep(0.5)

        self.get_logger().info("Touchdown confirmed → DONE")
        self._set_state(State.DONE)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _alt(self) -> float:
        with self._tel_lock:
            return self._rel_alt

    def _get_state(self) -> State:
        with self._sm_lock:
            return self._state

    def _set_state(self, new: State) -> None:
        with self._sm_lock:
            old, self._state = self._state, new
        self.get_logger().info(f"State: {old.name} → {new.name}")

    def _get_frame(self) -> Optional[np.ndarray]:
        with self._frame_lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def _is_duplicate_blob(self, n: float, e: float) -> bool:
        r = self._p("blob_dedup_radius_m")
        return any(math.hypot(n - b[0], e - b[1]) < r for b in self._detected_blobs)

    def _publish_debug(
        self,
        state: State,
        qr_bbox: Optional[list] = None,
        blob_count: int = 0,
    ) -> None:
        frame = self._get_frame()
        if frame is None:
            return
        with self._frame_lock:
            iw, ih = self._img_w, self._img_h

        img = frame.copy()
        cx, cy = iw // 2, ih // 2

        # Crosshair
        cv2.line(img, (cx - 25, cy), (cx + 25, cy), (0, 255, 0), 2)
        cv2.line(img, (cx, cy - 25), (cx, cy + 25), (0, 255, 0), 2)

        # QR bbox
        if qr_bbox is not None:
            pts = np.array(qr_bbox, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(img, [pts], True, (0, 0, 255), 2)

        # State + stats
        label = state.name
        if state == State.FAST_SCOUT:
            label += f"  WP {self._wp_idx}/{len(self._waypoints)}  Blobs:{len(self._detected_blobs)}"
        elif state == State.APPROACH:
            label += f"  T {self._inspect_idx}/{len(self._inspect_queue)}"

        cv2.putText(img, label, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_frame"
        out.format          = "jpeg"
        out.data            = buf.tobytes()
        self._pub_debug.publish(out)

    def _publish_detection_report(self, frame: np.ndarray) -> None:
        with self._tel_lock:
            lat, lon, alt = self._lat, self._lon, self._rel_alt
            n, e          = self._north_m, self._east_m

        # Annotated photo
        photo_img = annotate_detection_photo(
            frame,
            np.array(self._aruco_bbox) if self._aruco_bbox else None,
            self._last_marker_id,
            OBJECT_TYPE_PLACEHOLDER,
            lat, lon,
        )

        # Map image
        all_visited = [
            self._inspect_queue[i] for i in range(self._inspect_idx + 1)
        ]
        map_img = generate_map_image(
            self._waypoints, all_visited, (n, e)
        )

        def encode_jpeg(arr: np.ndarray) -> CompressedImage:
            _, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            msg = CompressedImage()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.format          = "jpeg"
            msg.data            = buf.tobytes()
            return msg

        report = DetectionReport()
        report.header.stamp    = self.get_clock().now().to_msg()
        report.qr_data         = self._last_marker_id
        report.object_type     = OBJECT_TYPE_PLACEHOLDER
        report.latitude        = lat
        report.longitude       = lon
        report.altitude_m      = alt
        report.photo           = encode_jpeg(photo_img)
        report.map_image       = encode_jpeg(map_img)

        self._pub_report.publish(report)
        self.get_logger().info(
            f"DetectionReport published: ArUco ID='{report.qr_data}' "
            f"({lat:.6f}, {lon:.6f})"
        )


# ── Entry point ────────────────────────────────────────────────────────────────


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScoutMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

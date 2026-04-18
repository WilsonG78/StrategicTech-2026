"""
qr_lawnmower_node.py
====================
ArduCopter mission node – ROS 2 / MAVSDK-Python / OpenCV

Responsibilities
----------------
* Generates a lawnmower (boustrophedon) waypoint grid in local NED frame.
* Arms, takes off, and patrols the grid in OFFBOARD (GUIDED) mode.
* Detects QR codes via cv2.QRCodeDetector and centres the drone over them
  with a P-controller before descending for a closer scan.
* Publishes /qr/detected and a compressed debug image with bbox overlay.
* Monitors flight mode continuously; any manual RC switch-over immediately
  freezes all autopilot commands (STATE_MANUAL_OVERRIDE).

Architecture
------------
ROS 2 spin  →  main thread   (image subscription, timer, publishers)
MAVSDK loop →  daemon thread (asyncio event loop, all drone commands)
Shared state: threading.Lock-protected dicts/values
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from enum import Enum, auto
from typing import List, Optional, Tuple

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

# ── Flight modes that are controlled by this script ───────────────────────────
# When using MAVSDK Offboard API against ArduPilot, the FC enters GUIDED mode.
# MAVSDK may report it as OFFBOARD or HOLD depending on firmware/MAVLink mapping.
_ALLOWED_MODES: frozenset[FlightMode] = frozenset(
    {FlightMode.OFFBOARD, FlightMode.HOLD, FlightMode.TAKEOFF}
)

# ── State Machine ─────────────────────────────────────────────────────────────


class State(Enum):
    TAKEOFF = auto()
    PATROL = auto()
    ALIGNING = auto()
    DESCENDING = auto()
    WAITING = auto()
    RESUMING = auto()
    MANUAL_OVERRIDE = auto()


_MISSION_STATES: frozenset[State] = frozenset(
    {
        State.TAKEOFF,
        State.PATROL,
        State.ALIGNING,
        State.DESCENDING,
        State.WAITING,
        State.RESUMING,
    }
)

# ── Lawnmower route generator ─────────────────────────────────────────────────


def generate_lawnmower(
    length_m: float, width_m: float, spacing_m: float
) -> List[Tuple[float, float]]:
    """Return (north_m, east_m) waypoints for a boustrophedon grid.

    The grid starts at origin (0, 0) and sweeps along the North axis,
    stepping East by *spacing_m* on each return leg.
    """
    waypoints: List[Tuple[float, float]] = []
    east = 0.0
    go_north = True
    while east <= width_m + 1e-6:
        if go_north:
            waypoints.append((0.0, east))
            waypoints.append((length_m, east))
        else:
            waypoints.append((length_m, east))
            waypoints.append((0.0, east))
        east += spacing_m
        go_north = not go_north
    return waypoints


# ── Node ──────────────────────────────────────────────────────────────────────


class QRLawnmowerNode(Node):
    def __init__(self) -> None:
        super().__init__("qr_lawnmower_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("camera_fov_deg", 90.0)
        self.declare_parameter("p_gain", 0.3)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("center_threshold_px", 15)
        self.declare_parameter("patrol_alt_m", 3.0)
        self.declare_parameter("scan_alt_m", 2.5)
        self.declare_parameter("wait_time_sec", 5.0)
        self.declare_parameter("grid_length_m", 10.0)
        self.declare_parameter("grid_width_m", 10.0)
        self.declare_parameter("line_spacing_m", 2.0)
        self.declare_parameter("mavsdk_address", "udp://:14550")

        # ── ROS interfaces ────────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub_image = self.create_subscription(
            Image, "/ugv_camera/image_raw", self._image_cb, qos_be
        )
        self._pub_qr = self.create_publisher(String, "/qr/detected", 10)
        self._pub_debug = self.create_publisher(
            CompressedImage, "/qr/debug_image", qos_be
        )

        self._bridge = CvBridge()
        self._qr_detector = cv2.QRCodeDetector()

        # ── Shared telemetry (written by asyncio tasks, read by SM) ───────────
        self._tel_lock = threading.Lock()
        self._rel_alt: float = 0.0        # relative altitude [m]
        self._north_m: float = 0.0        # NED north [m]
        self._east_m: float = 0.0         # NED east  [m]
        self._flight_mode: Optional[FlightMode] = None

        # ── Shared camera frame ────────────────────────────────────────────────
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._img_h: int = 480
        self._img_w: int = 640

        # ── State machine ──────────────────────────────────────────────────────
        self._sm_lock = threading.Lock()
        self._state: State = State.TAKEOFF

        # ── Lawnmower route ────────────────────────────────────────────────────
        self._waypoints: List[Tuple[float, float]] = generate_lawnmower(
            self.get_parameter("grid_length_m").value,
            self.get_parameter("grid_width_m").value,
            self.get_parameter("line_spacing_m").value,
        )
        self._wp_idx: int = 0
        self.get_logger().info(
            f"Lawnmower route generated: {len(self._waypoints)} waypoints."
        )

        # ── Aligning bookkeeping ───────────────────────────────────────────────
        self._centered_since: Optional[float] = None
        self._wait_started: Optional[float] = None

        # ── MAVSDK asyncio in background thread ────────────────────────────────
        self._loop = asyncio.new_event_loop()
        self._bg_thread = threading.Thread(
            target=self._run_loop, name="mavsdk-asyncio", daemon=True
        )
        self._bg_thread.start()

        self.get_logger().info("QRLawnmowerNode initialised.")

    # ── Image subscription callback (ROS main thread) ─────────────────────────

    def _image_cb(self, msg: Image) -> None:
        """Store latest camera frame; never blocks."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._latest_frame = frame
                self._img_h, self._img_w = frame.shape[:2]
        except Exception as exc:
            self.get_logger().error(f"cv_bridge error: {exc}", throttle_duration_sec=5.0)

    # ── asyncio entry point ────────────────────────────────────────────────────

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_main())

    async def _async_main(self) -> None:
        address: str = self.get_parameter("mavsdk_address").value
        self.get_logger().info(f"Connecting to ArduPilot at {address} …")

        self._drone = System()
        try:
            await self._drone.connect(system_address=address)
            async for cs in self._drone.core.connection_state():
                if cs.is_connected:
                    self.get_logger().info("MAVLink connection established.")
                    break
        except Exception as exc:
            self.get_logger().fatal(f"Cannot connect to drone: {exc}")
            return

        await asyncio.gather(
            self._task_stream_pos_ned(),
            self._task_stream_rel_alt(),
            self._task_stream_flight_mode(),
            self._task_rc_monitor(),
            self._task_mission(),
        )

    # ── Telemetry streaming tasks ──────────────────────────────────────────────

    async def _task_stream_pos_ned(self) -> None:
        while rclpy.ok():
            try:
                async for pvn in self._drone.telemetry.position_velocity_ned():
                    with self._tel_lock:
                        self._north_m = pvn.position.north_m
                        self._east_m = pvn.position.east_m
            except Exception as exc:
                self.get_logger().error(f"pos_ned stream error: {exc}")
            await asyncio.sleep(1.0)

    async def _task_stream_rel_alt(self) -> None:
        while rclpy.ok():
            try:
                async for pos in self._drone.telemetry.position():
                    with self._tel_lock:
                        self._rel_alt = pos.relative_altitude_m
            except Exception as exc:
                self.get_logger().error(f"rel_alt stream error: {exc}")
            await asyncio.sleep(1.0)

    async def _task_stream_flight_mode(self) -> None:
        while rclpy.ok():
            try:
                async for fm in self._drone.telemetry.flight_mode():
                    with self._tel_lock:
                        self._flight_mode = fm
            except Exception as exc:
                self.get_logger().error(f"flight_mode stream error: {exc}")
            await asyncio.sleep(1.0)

    # ── RC / manual override monitor ──────────────────────────────────────────

    async def _task_rc_monitor(self) -> None:
        """Detect pilot RC switch-over and freeze autopilot commands."""
        while rclpy.ok():
            try:
                async for fm in self._drone.telemetry.flight_mode():
                    with self._sm_lock:
                        current_state = self._state
                    if current_state in _MISSION_STATES and fm not in _ALLOWED_MODES:
                        with self._sm_lock:
                            if self._state in _MISSION_STATES:
                                self.get_logger().warn(
                                    f"[RC OVERRIDE] Pilot switched to {fm.name}. "
                                    "Freezing autopilot – STATE_MANUAL_OVERRIDE active."
                                )
                                self._state = State.MANUAL_OVERRIDE
            except Exception as exc:
                self.get_logger().error(f"RC monitor error: {exc}")
            await asyncio.sleep(1.0)

    # ── Mission state-machine task ─────────────────────────────────────────────

    async def _task_mission(self) -> None:
        while rclpy.ok():
            with self._sm_lock:
                state = self._state

            if state == State.MANUAL_OVERRIDE:
                self._publish_debug(State.MANUAL_OVERRIDE)
                await asyncio.sleep(0.1)

            elif state == State.TAKEOFF:
                await self._state_takeoff()

            elif state == State.PATROL:
                await self._state_patrol()

            elif state == State.ALIGNING:
                await self._state_aligning()

            elif state == State.DESCENDING:
                await self._state_descending()

            elif state == State.WAITING:
                await self._state_waiting()

            elif state == State.RESUMING:
                await self._state_resuming()

            await asyncio.sleep(0.05)

    # ─────────────────────────────────────────────────────────────────────────
    # State handlers
    # ─────────────────────────────────────────────────────────────────────────

    async def _state_takeoff(self) -> None:
        patrol_alt: float = self.get_parameter("patrol_alt_m").value
        try:
            self.get_logger().info("Arming…")
            await self._drone.action.arm()
            self.get_logger().info("Taking off…")
            await self._drone.action.takeoff()
        except Exception as exc:
            self.get_logger().error(f"Arm/takeoff error: {exc}")
            await asyncio.sleep(2.0)
            return

        # Wait for patrol altitude
        while rclpy.ok():
            with self._sm_lock:
                if self._state == State.MANUAL_OVERRIDE:
                    return
            with self._tel_lock:
                alt = self._rel_alt
            if alt >= patrol_alt * 0.90:
                break
            self.get_logger().info(
                f"Climbing … {alt:.1f} / {patrol_alt:.1f} m", throttle_duration_sec=1.0
            )
            await asyncio.sleep(0.2)

        # Transition to Offboard: send ≥10 setpoints before start() — required by MAVSDK
        try:
            for _ in range(10):
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.05)
            await self._drone.offboard.start()
            self.get_logger().info("Offboard mode started.")
        except OffboardError as exc:
            self.get_logger().error(f"Offboard start failed: {exc}")
            return

        # Fly to first waypoint
        if self._waypoints:
            wp_n, wp_e = self._waypoints[0]
            try:
                await self._drone.offboard.set_position_ned(
                    PositionNedYaw(wp_n, wp_e, -patrol_alt, 0.0)
                )
            except OffboardError as exc:
                self.get_logger().error(f"set_position_ned error: {exc}")
                return

            while rclpy.ok():
                with self._sm_lock:
                    if self._state == State.MANUAL_OVERRIDE:
                        return
                with self._tel_lock:
                    n, e = self._north_m, self._east_m
                dist = math.hypot(n - wp_n, e - wp_e)
                if dist < 0.8:
                    break
                await asyncio.sleep(0.2)

        self.get_logger().info("First waypoint reached → STATE_PATROL")
        with self._sm_lock:
            self._state = State.PATROL
            self._wp_idx = 1

    async def _state_patrol(self) -> None:
        patrol_alt: float = self.get_parameter("patrol_alt_m").value

        if self._wp_idx >= len(self._waypoints):
            self.get_logger().info("All waypoints complete – hovering.")
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            await asyncio.sleep(1.0)
            return

        wp_n, wp_e = self._waypoints[self._wp_idx]
        try:
            await self._drone.offboard.set_position_ned(
                PositionNedYaw(wp_n, wp_e, -patrol_alt, 0.0)
            )
        except OffboardError as exc:
            self.get_logger().error(f"PATROL set_position_ned: {exc}")
            await asyncio.sleep(0.5)
            return

        # Fly toward waypoint, scanning for QR each iteration
        while rclpy.ok():
            with self._sm_lock:
                if self._state != State.PATROL:
                    return

            # ── QR detection ──────────────────────────────────────────────────
            qr = self._detect_qr()
            self._publish_debug(State.PATROL, qr_bbox=qr[1] if qr else None)
            if qr is not None:
                qr_text, bbox = qr
                msg = String()
                msg.data = qr_text
                self._pub_qr.publish(msg)
                self.get_logger().info(f"QR detected: '{qr_text}' → STATE_ALIGNING")
                with self._sm_lock:
                    self._state = State.ALIGNING
                self._centered_since = None
                return

            # ── Arrival check ─────────────────────────────────────────────────
            with self._tel_lock:
                n, e = self._north_m, self._east_m
            if math.hypot(n - wp_n, e - wp_e) < 0.8:
                self._wp_idx += 1
                return

            await asyncio.sleep(0.1)

    async def _state_aligning(self) -> None:
        p_gain: float = self.get_parameter("p_gain").value
        max_vel: float = self.get_parameter("max_velocity").value
        threshold: int = self.get_parameter("center_threshold_px").value
        fov_deg: float = self.get_parameter("camera_fov_deg").value
        patrol_alt: float = self.get_parameter("patrol_alt_m").value

        with self._frame_lock:
            frame = self._latest_frame
            img_w = self._img_w
            img_h = self._img_h

        if frame is None:
            # No frame yet – hover in place
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            await asyncio.sleep(0.05)
            return

        qr = self._detect_qr()
        self._publish_debug(State.ALIGNING, qr_bbox=qr[1] if qr else None)

        if qr is None:
            # Lost QR – hover and keep looking
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            self._centered_since = None
            await asyncio.sleep(0.05)
            return

        qr_text, bbox = qr
        msg = String()
        msg.data = qr_text
        self._pub_qr.publish(msg)

        # ── Compute pixel offset from image centre ────────────────────────────
        cx = sum(p[0] for p in bbox) / 4.0
        cy = sum(p[1] for p in bbox) / 4.0
        off_px_x = cx - img_w / 2.0   # + = right  = East
        off_px_y = cy - img_h / 2.0   # + = down   = South (NED: -North)

        # ── Convert offset to metres ──────────────────────────────────────────
        fov_rad = math.radians(fov_deg)
        scale_m_per_px = (2.0 * patrol_alt * math.tan(fov_rad / 2.0)) / img_w

        off_m_e = off_px_x * scale_m_per_px   # East
        off_m_s = off_px_y * scale_m_per_px   # South → North = -off_m_s

        # ── P-controller ──────────────────────────────────────────────────────
        vn = float(np.clip(-off_m_s * p_gain, -max_vel, max_vel))
        ve = float(np.clip(off_m_e * p_gain, -max_vel, max_vel))

        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, 0.0, 0.0)
            )
        except OffboardError as exc:
            self.get_logger().error(f"ALIGNING set_velocity: {exc}")

        # ── Centre check (must hold for 1 s) ──────────────────────────────────
        if abs(off_px_x) < threshold and abs(off_px_y) < threshold:
            now = time.monotonic()
            if self._centered_since is None:
                self._centered_since = now
            elif now - self._centered_since >= 1.0:
                self.get_logger().info("QR centred ≥ 1 s → STATE_DESCENDING")
                try:
                    await self._drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
                except OffboardError:
                    pass
                with self._sm_lock:
                    self._state = State.DESCENDING
        else:
            self._centered_since = None

        await asyncio.sleep(0.05)

    async def _state_descending(self) -> None:
        scan_alt: float = self.get_parameter("scan_alt_m").value
        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.5, 0.0)   # vz > 0 → descend in NED
            )
        except OffboardError as exc:
            self.get_logger().error(f"DESCENDING set_velocity: {exc}")
            await asyncio.sleep(0.2)
            return

        self._publish_debug(State.DESCENDING)

        with self._tel_lock:
            alt = self._rel_alt
        if alt <= scan_alt:
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            self.get_logger().info(
                f"Scan altitude {scan_alt:.1f} m reached → STATE_WAITING"
            )
            self._wait_started = time.monotonic()
            with self._sm_lock:
                self._state = State.WAITING

        await asyncio.sleep(0.05)

    async def _state_waiting(self) -> None:
        wait_time: float = self.get_parameter("wait_time_sec").value

        if self._wait_started is None:
            self._wait_started = time.monotonic()

        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
        except OffboardError:
            pass

        self._publish_debug(State.WAITING)

        if time.monotonic() - self._wait_started >= wait_time:
            self.get_logger().info("Wait complete → STATE_RESUMING")
            with self._sm_lock:
                self._state = State.RESUMING

        await asyncio.sleep(0.1)

    async def _state_resuming(self) -> None:
        patrol_alt: float = self.get_parameter("patrol_alt_m").value
        try:
            await self._drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, -0.5, 0.0)   # vz < 0 → climb in NED
            )
        except OffboardError as exc:
            self.get_logger().error(f"RESUMING set_velocity: {exc}")
            await asyncio.sleep(0.2)
            return

        self._publish_debug(State.RESUMING)

        with self._tel_lock:
            alt = self._rel_alt
        if alt >= patrol_alt * 0.97:
            try:
                await self._drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
            except OffboardError:
                pass
            self.get_logger().info(
                f"Back at patrol altitude {patrol_alt:.1f} m → STATE_PATROL"
            )
            with self._sm_lock:
                self._state = State.PATROL

        await asyncio.sleep(0.05)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _detect_qr(
        self,
    ) -> Optional[Tuple[str, List[List[float]]]]:
        """Run QR detection on the latest frame. Returns (text, bbox) or None."""
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            return None
        data, bbox, _ = self._qr_detector.detectAndDecode(frame)
        if data and bbox is not None:
            return data, bbox[0].tolist()   # bbox[0] shape: (4, 2)
        return None

    def _publish_debug(
        self,
        state: State,
        qr_bbox: Optional[List[List[float]]] = None,
    ) -> None:
        """Draw overlay on latest frame and publish as CompressedImage."""
        with self._frame_lock:
            frame = self._latest_frame
            img_h = self._img_h
            img_w = self._img_w

        if frame is None:
            return

        debug = frame.copy()

        # Crosshair at image centre
        cx, cy = img_w // 2, img_h // 2
        cv2.line(debug, (cx - 25, cy), (cx + 25, cy), (0, 255, 0), 2)
        cv2.line(debug, (cx, cy - 25), (cx, cy + 25), (0, 255, 0), 2)

        # QR bounding box
        if qr_bbox is not None:
            pts = np.array(qr_bbox, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug, [pts], isClosed=True, color=(0, 0, 255), thickness=2)
            qx = int(sum(p[0] for p in qr_bbox) / 4)
            qy = int(sum(p[1] for p in qr_bbox) / 4)
            cv2.circle(debug, (qx, qy), 5, (0, 0, 255), -1)

        # State label
        cv2.putText(
            debug,
            state.name,
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ok, buf = cv2.imencode(".jpg", debug, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self._pub_debug.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRLawnmowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
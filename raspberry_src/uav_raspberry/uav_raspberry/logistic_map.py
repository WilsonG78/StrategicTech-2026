"""
logistic_map_node.py
====================
ROS 2 node – logistic-map chaos visualisation published to Foxglove Studio.

The logistic map  x_{n+1} = r · x_n · (1 − x_n),  x_0 = 0.5
is iterated 500 steps every 200 ms.  The last 200 values are rendered
as coloured spheres in /logistic_map/scene (foxglove_msgs/SceneUpdate).

The bifurcation parameter r (2.5 – 4.0) is mapped linearly from RC
channel 2 raw PWM (988 – 2011 µs).  RC telemetry is polled in a
dedicated asyncio loop running in a daemon thread so it never blocks
the ROS executor.

Sphere layout (Foxglove "map" frame)
-------------------------------------
  X  =  iteration index × 0.05 m
  Y  =  x_n value × 2.0 m          (0 – 2 m range)
  Z  =  0

Colour  =  HSV hue driven by local density within 20 value bins
           dense (many points in same bin) → red  (H ≈ 0°)
           sparse                          → blue (H ≈ 240°)
"""

from __future__ import annotations

import asyncio
import colorsys
import threading
import time
from collections import Counter
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavsdk import System

# foxglove_msgs ROS 2 package  (sudo apt install ros-jazzy-foxglove-msgs)
from foxglove_msgs.msg import SceneUpdate, SceneEntity, SpherePrimitive

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from builtin_interfaces.msg import Duration

# foxglove_msgs/Color  (float64 r, g, b, a)
from foxglove_msgs.msg import Color as FgColor

# ── Constants ─────────────────────────────────────────────────────────────────

RC_MIN_US: float = 988.0
RC_MAX_US: float = 2011.0
R_MIN: float = 2.5
R_MAX: float = 4.0
R_DEFAULT: float = 3.57   # edge of chaos

ITER_STEPS: int = 500
DISPLAY_POINTS: int = 200
DENSITY_BINS: int = 20
PUBLISH_INTERVAL_S: float = 0.20

SPHERE_SCALE: float = 0.035   # metres
ITER_STEP_M: float = 0.05     # X spacing per iteration step
VALUE_SCALE_M: float = 2.0    # Y scale for x_n ∈ [0, 1]


# ── Node ──────────────────────────────────────────────────────────────────────


class LogisticMapNode(Node):
    def __init__(self) -> None:
        super().__init__("logistic_map_node")

        self.declare_parameter("mavsdk_address", "udpin://127.0.0.1:14551")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(SceneUpdate, "/logistic_map/scene", qos)

        # Shared r value – written by MAVSDK thread, read by compute thread
        self._r_lock = threading.Lock()
        self._r: float = R_DEFAULT

        # MAVSDK asyncio loop
        self._loop = asyncio.new_event_loop()
        self._mavsdk_thread = threading.Thread(
            target=self._run_mavsdk, name="mavsdk-logistic", daemon=True
        )
        self._mavsdk_thread.start()

        # Compute & publish loop (daemon thread – no rclpy involvement)
        self._compute_thread = threading.Thread(
            target=self._compute_loop, name="logistic-compute", daemon=True
        )
        self._compute_thread.start()

        self.get_logger().info("LogisticMapNode started.")

    # ── MAVSDK thread ─────────────────────────────────────────────────────────

    def _run_mavsdk(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._rc_monitor_loop())

    async def _rc_monitor_loop(self) -> None:
        address: str = self.get_parameter("mavsdk_address").value
        drone = System(port=50052)
        while rclpy.ok():
            try:
                await drone.connect(system_address=address)
                async for cs in drone.core.connection_state():
                    if cs.is_connected:
                        self.get_logger().info("LogisticMapNode: MAVLink connected.")
                        break
                async for rc in drone.telemetry.rc_channels():
                    # channels list: index 0 → ch1, index 1 → ch2
                    if len(rc.channels) >= 2:
                        raw_us: float = rc.channels[1]   # raw PWM µs from ArduPilot
                        t = (raw_us - RC_MIN_US) / (RC_MAX_US - RC_MIN_US)
                        t = max(0.0, min(1.0, t))
                        r_new = R_MIN + t * (R_MAX - R_MIN)
                        with self._r_lock:
                            self._r = r_new
            except Exception as exc:
                self.get_logger().warning(
                    f"RC monitor disconnected: {exc}. Retrying in 5 s.",
                    throttle_duration_sec=10.0,
                )
            await asyncio.sleep(5.0)

    # ── Compute & publish thread ───────────────────────────────────────────────

    def _compute_loop(self) -> None:
        while rclpy.ok():
            t0 = time.monotonic()

            with self._r_lock:
                r = self._r

            # ── Iterate logistic map ──────────────────────────────────────────
            x: float = 0.5
            series: List[float] = []
            for _ in range(ITER_STEPS):
                x = r * x * (1.0 - x)
                series.append(x)

            points = series[-DISPLAY_POINTS:]

            # ── Density → HSV colour ──────────────────────────────────────────
            bin_counts: Counter = Counter(
                int(v * DENSITY_BINS) for v in points
            )
            max_count: int = max(bin_counts.values()) if bin_counts else 1

            # ── Build SceneUpdate ─────────────────────────────────────────────
            scene_msg = SceneUpdate()

            entity = SceneEntity()
            entity.id = "logistic_map_entity"
            entity.frame_id = "map"
            entity.frame_locked = True
            lifetime = Duration()
            lifetime.sec = 1          # entity expires after 1 s if not refreshed
            lifetime.nanosec = 0
            entity.lifetime = lifetime

            for i, val in enumerate(points):
                sphere = SpherePrimitive()

                # Position
                pose = Pose()
                pose.position = Point(
                    x=float(i) * ITER_STEP_M,
                    y=float(val) * VALUE_SCALE_M,
                    z=0.0,
                )
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                sphere.pose = pose

                # Size (uniform sphere)
                sphere.size = Vector3(
                    x=SPHERE_SCALE, y=SPHERE_SCALE, z=SPHERE_SCALE
                )

                # HSV colour by density: dense → red, sparse → blue
                bin_key: int = int(val * DENSITY_BINS)
                density: float = bin_counts.get(bin_key, 0) / max_count
                hue: float = (1.0 - density) * 0.66   # 0.66 = blue, 0.0 = red
                rgb = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                sphere.color = FgColor(r=rgb[0], g=rgb[1], b=rgb[2], a=0.85)

                entity.spheres.append(sphere)

            scene_msg.entities.append(entity)

            try:
                self._pub.publish(scene_msg)
            except Exception as exc:
                self.get_logger().error(
                    f"publish error: {exc}", throttle_duration_sec=5.0
                )

            elapsed = time.monotonic() - t0
            sleep_s = max(0.0, PUBLISH_INTERVAL_S - elapsed)
            time.sleep(sleep_s)


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LogisticMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
"""UAV telemetry node – bridges MAVSDK/ArduPilot to ROS 2.

Publishes GPS position, GPS time, and barometer altitude on /data at >= 1 Hz.
The asyncio event loop runs in a dedicated background thread so it never
blocks the ROS 2 executor.
"""

import asyncio
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavsdk import System

from uav_msgs.msg import UavTelemetry

RECONNECT_DELAY_S = 5.0
WATCHDOG_TIMEOUT_S = 5.0
PUBLISH_HZ = 2.0


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_node")

        self.declare_parameter("connection_url", "serial:///dev/ttyAMA0:115200")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(UavTelemetry, "/data", qos)

        self._state = {
            "latitude": math.nan,
            "longitude": math.nan,
            "altitude_gps": math.nan,
            "altitude_baro": math.nan,
            "gps_unix_time": math.nan,
            "gps_satellites": 0,
        }
        self._lock = threading.Lock()

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop, name="mavsdk-asyncio", daemon=True
        )
        self._thread.start()

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self.get_logger().info("TelemetryNode started.")

    # ── asyncio thread ────────────────────────────────────────────────────────

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._connect_loop())

    async def _connect_loop(self) -> None:
        url: str = (
            self.get_parameter("connection_url").get_parameter_value().string_value
        )
        while rclpy.ok():
            self.get_logger().info(f"Connecting to ArduPilot at {url} …")
            system = System()
            try:
                await system.connect(system_address=url)

                async for state in system.core.connection_state():
                    if state.is_connected:
                        self.get_logger().info("Connected to ArduPilot.")
                        break

                await asyncio.gather(
                    self._stream_position(system),
                    self._stream_altitude(system),
                    self._stream_gps_info(system),
                    self._stream_unix_epoch(system),
                )
            except Exception as exc:
                self.get_logger().error(
                    f"MAVSDK error: {exc} – retrying in {RECONNECT_DELAY_S} s"
                )
            await asyncio.sleep(RECONNECT_DELAY_S)

    async def _stream_position(self, system: System) -> None:
        async for pos in system.telemetry.position():
            with self._lock:
                self._state["latitude"] = pos.latitude_deg
                self._state["longitude"] = pos.longitude_deg
                self._state["altitude_gps"] = pos.absolute_altitude_m

    async def _stream_altitude(self, system: System) -> None:
        async for alt in system.telemetry.altitude():
            with self._lock:
                self._state["altitude_baro"] = alt.altitude_amsl_m

    async def _stream_gps_info(self, system: System) -> None:
        async for info in system.telemetry.gps_info():
            with self._lock:
                self._state["gps_satellites"] = info.num_satellites

    async def _stream_unix_epoch(self, system: System) -> None:
        async for epoch in system.telemetry.unix_epoch_time():
            with self._lock:
                self._state["gps_unix_time"] = epoch.time_us / 1_000_000.0

    # ── ROS 2 timer ──────────────────────────────────────────────────────────

    def _publish(self) -> None:
        with self._lock:
            snap = dict(self._state)

        msg = UavTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.latitude = snap["latitude"]
        msg.longitude = snap["longitude"]
        msg.altitude_gps = snap["altitude_gps"]
        msg.altitude_baro = snap["altitude_baro"]
        msg.gps_unix_time = snap["gps_unix_time"]
        msg.gps_satellites = snap["gps_satellites"]

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Thin async wrapper around MAVSDK-Python.

Exposes the minimum surface the mission needs:
  * connect / arm / takeoff / land / return-to-launch
  * fly to a local NED waypoint (``goto_ned``)
  * send velocity setpoints in the body / NED frame
  * poll position, altitude (AMSL + AGL + baro), battery, GPS fix, heading

Every MAVLink/MAVSDK call is wrapped in try/except.  Errors raise
``MavsdkError`` so the caller can decide whether to abort or retry.
"""

from __future__ import annotations

import asyncio
import logging
import math
from dataclasses import dataclass, field
from typing import Optional

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

logger = logging.getLogger(__name__)


class MavsdkError(RuntimeError):
    """Raised when a MAVSDK call fails in a recoverable-or-not way."""


@dataclass
class DroneState:
    connected: bool = False
    armed: bool = False
    in_air: bool = False
    flight_mode: str = "UNKNOWN"
    latitude: float = math.nan
    longitude: float = math.nan
    altitude_amsl_m: float = math.nan
    altitude_rel_m: float = math.nan      # AGL from takeoff
    altitude_baro_m: float = math.nan
    heading_deg: float = math.nan
    battery_pct: float = math.nan
    battery_voltage_v: float = math.nan
    gps_fix: str = "NO_FIX"
    gps_satellites: int = 0
    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0
    last_update_monotonic: float = 0.0


@dataclass
class _Streams:
    tasks: list = field(default_factory=list)


class MavsdkController:
    """Async controller for a single vehicle."""

    def __init__(
        self,
        system_address: str,
        server_port: int = 50051,
        logger_: Optional[logging.Logger] = None,
    ) -> None:
        self._address = system_address
        self._server_port = server_port
        self._log = logger_ or logger
        self._system: Optional[System] = None
        self.state = DroneState()
        self._streams = _Streams()
        self._lock = asyncio.Lock()

    # ── connection lifecycle ─────────────────────────────────────────────────

    async def connect(self, timeout_s: float = 30.0) -> None:
        self._log.info("Connecting MAVSDK to %s", self._address)
        try:
            self._system = System(port=self._server_port)
            await self._system.connect(system_address=self._address)
        except Exception as exc:
            raise MavsdkError(f"mavsdk connect failed: {exc}") from exc

        try:
            async def _wait_connected() -> None:
                async for s in self._system.core.connection_state():
                    if s.is_connected:
                        self.state.connected = True
                        return

            await asyncio.wait_for(_wait_connected(), timeout=timeout_s)
        except asyncio.TimeoutError as exc:
            raise MavsdkError("timeout waiting for MAVLink heartbeat") from exc

        self._log.info("MAVLink heartbeat acquired.")
        self._start_streams()

    async def disconnect(self) -> None:
        for t in self._streams.tasks:
            t.cancel()
        self._streams.tasks.clear()
        self._system = None
        self.state.connected = False

    # ── telemetry streams ────────────────────────────────────────────────────

    def _start_streams(self) -> None:
        assert self._system is not None
        loop = asyncio.get_event_loop()
        self._streams.tasks = [
            loop.create_task(self._stream_position()),
            loop.create_task(self._stream_altitude()),
            loop.create_task(self._stream_battery()),
            loop.create_task(self._stream_armed()),
            loop.create_task(self._stream_in_air()),
            loop.create_task(self._stream_flight_mode()),
            loop.create_task(self._stream_gps_info()),
            loop.create_task(self._stream_heading()),
            loop.create_task(self._stream_position_velocity_ned()),
        ]

    async def _stream_position(self) -> None:
        try:
            async for p in self._system.telemetry.position():
                self.state.latitude = p.latitude_deg
                self.state.longitude = p.longitude_deg
                self.state.altitude_amsl_m = p.absolute_altitude_m
                self.state.altitude_rel_m = p.relative_altitude_m
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("position stream ended: %s", exc)

    async def _stream_altitude(self) -> None:
        try:
            async for a in self._system.telemetry.altitude():
                self.state.altitude_baro_m = a.altitude_amsl_m
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("altitude stream ended: %s", exc)

    async def _stream_battery(self) -> None:
        try:
            async for b in self._system.telemetry.battery():
                self.state.battery_pct = b.remaining_percent * 100.0
                self.state.battery_voltage_v = b.voltage_v
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("battery stream ended: %s", exc)

    async def _stream_armed(self) -> None:
        try:
            async for v in self._system.telemetry.armed():
                self.state.armed = bool(v)
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("armed stream ended: %s", exc)

    async def _stream_in_air(self) -> None:
        try:
            async for v in self._system.telemetry.in_air():
                self.state.in_air = bool(v)
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("in_air stream ended: %s", exc)

    async def _stream_flight_mode(self) -> None:
        try:
            async for m in self._system.telemetry.flight_mode():
                self.state.flight_mode = str(m)
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("flight_mode stream ended: %s", exc)

    async def _stream_gps_info(self) -> None:
        try:
            async for g in self._system.telemetry.gps_info():
                self.state.gps_fix = str(g.fix_type)
                self.state.gps_satellites = g.num_satellites
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("gps_info stream ended: %s", exc)

    async def _stream_heading(self) -> None:
        try:
            async for h in self._system.telemetry.heading():
                self.state.heading_deg = h.heading_deg
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("heading stream ended: %s", exc)

    async def _stream_position_velocity_ned(self) -> None:
        try:
            async for pv in self._system.telemetry.position_velocity_ned():
                self.state.north_m = pv.position.north_m
                self.state.east_m = pv.position.east_m
                self.state.down_m = pv.position.down_m
                self._touch()
        except Exception as exc:  # noqa: BLE001
            self._log.warning("position_velocity_ned stream ended: %s", exc)

    def _touch(self) -> None:
        self.state.last_update_monotonic = asyncio.get_event_loop().time()

    # ── flight control primitives ────────────────────────────────────────────

    async def arm(self) -> None:
        try:
            await self._system.action.arm()
        except Exception as exc:
            raise MavsdkError(f"arm failed: {exc}") from exc

    async def disarm(self) -> None:
        try:
            await self._system.action.disarm()
        except Exception as exc:
            raise MavsdkError(f"disarm failed: {exc}") from exc

    async def set_takeoff_altitude(self, altitude_m: float) -> None:
        try:
            await self._system.action.set_takeoff_altitude(altitude_m)
        except Exception as exc:
            raise MavsdkError(f"set_takeoff_altitude failed: {exc}") from exc

    async def takeoff(self, altitude_m: float, tolerance_m: float = 0.3) -> None:
        await self.set_takeoff_altitude(altitude_m)
        try:
            await self._system.action.takeoff()
        except Exception as exc:
            raise MavsdkError(f"takeoff failed: {exc}") from exc

        deadline = asyncio.get_event_loop().time() + 45.0
        while asyncio.get_event_loop().time() < deadline:
            if (self.state.in_air
                    and abs(self.state.altitude_rel_m - altitude_m)
                    <= tolerance_m):
                return
            await asyncio.sleep(0.2)
        raise MavsdkError("takeoff did not reach target altitude in time")

    async def land(self) -> None:
        try:
            await self._system.action.land()
        except Exception as exc:
            raise MavsdkError(f"land failed: {exc}") from exc

    async def return_to_launch(self) -> None:
        try:
            await self._system.action.return_to_launch()
        except Exception as exc:
            raise MavsdkError(f"RTL failed: {exc}") from exc

    # ── offboard (NED) ───────────────────────────────────────────────────────

    async def start_offboard(self) -> None:
        try:
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, self.state.heading_deg or 0.0))
            await self._system.offboard.start()
        except OffboardError as exc:
            raise MavsdkError(f"offboard start failed: {exc}") from exc
        except Exception as exc:
            raise MavsdkError(f"offboard start failed: {exc}") from exc

    async def stop_offboard(self) -> None:
        try:
            await self._system.offboard.stop()
        except OffboardError as exc:
            self._log.warning("offboard stop error: %s", exc)

    async def set_position_ned(
        self, north_m: float, east_m: float, down_m: float, yaw_deg: float
    ) -> None:
        try:
            await self._system.offboard.set_position_ned(
                PositionNedYaw(north_m, east_m, down_m, yaw_deg))
        except OffboardError as exc:
            raise MavsdkError(f"set_position_ned failed: {exc}") from exc

    async def set_velocity_ned(
        self, vn: float, ve: float, vd: float, yaw_deg: float
    ) -> None:
        try:
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vd, yaw_deg))
        except OffboardError as exc:
            raise MavsdkError(f"set_velocity_ned failed: {exc}") from exc

    async def goto_ned(
        self,
        north_m: float,
        east_m: float,
        down_m: float,
        yaw_deg: Optional[float] = None,
        tolerance_m: float = 1.0,
        timeout_s: float = 60.0,
    ) -> None:
        yaw = yaw_deg if yaw_deg is not None else (self.state.heading_deg or 0.0)
        await self.set_position_ned(north_m, east_m, down_m, yaw)

        deadline = asyncio.get_event_loop().time() + timeout_s
        while asyncio.get_event_loop().time() < deadline:
            dn = north_m - self.state.north_m
            de = east_m - self.state.east_m
            dd = down_m - self.state.down_m
            if math.sqrt(dn * dn + de * de + dd * dd) <= tolerance_m:
                return
            await self.set_position_ned(north_m, east_m, down_m, yaw)
            await asyncio.sleep(0.1)
        raise MavsdkError(
            f"goto_ned timeout - target=({north_m:.1f},{east_m:.1f},"
            f"{down_m:.1f}) current=({self.state.north_m:.1f},"
            f"{self.state.east_m:.1f},{self.state.down_m:.1f})")

    async def do_set_servo(self, servo_index: int, pwm_us: int) -> None:
        """Send MAVLink DO_SET_SERVO (used for the drop mechanism)."""
        try:
            await self._system.action.set_actuator(servo_index, float(pwm_us))
        except ActionError as exc:
            raise MavsdkError(f"do_set_servo failed: {exc}") from exc
        except Exception as exc:
            raise MavsdkError(f"do_set_servo failed: {exc}") from exc

    async def drop_payload(
        self,
        servo_index: int,
        open_us: int,
        close_us: int,
        hold_s: float,
    ) -> None:
        """Open drop servo, wait, then close it."""
        await self.do_set_servo(servo_index, open_us)
        await asyncio.sleep(hold_s)
        await self.do_set_servo(servo_index, close_us)

    async def hold_position(self, seconds: float) -> None:
        n, e, d = self.state.north_m, self.state.east_m, self.state.down_m
        yaw = self.state.heading_deg or 0.0
        deadline = asyncio.get_event_loop().time() + seconds
        while asyncio.get_event_loop().time() < deadline:
            await self.set_position_ned(n, e, d, yaw)
            await asyncio.sleep(0.1)

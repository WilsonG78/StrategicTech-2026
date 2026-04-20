"""Background battery watchdog.

Polls the MAVSDK battery stream (via ``MavsdkController.state``) at a
configurable interval and:
  * emits a one-shot WARNING when voltage falls below
    ``battery_voltage_warning_v`` (3.73 V/cell default),
  * triggers an auto-RTL + disarm when voltage falls below
    ``battery_voltage_critical_v`` (3.60 V/cell default).

Runs as an asyncio task owned by the mission orchestrator.
"""

from __future__ import annotations

import asyncio
import logging
import math
from typing import Awaitable, Callable, Optional

from .mavsdk_controller import MavsdkController

logger = logging.getLogger(__name__)

LogHook = Callable[[str, str], None]
AbortHook = Callable[[], Awaitable[None]]


class BatteryMonitor:
    def __init__(
        self,
        controller: MavsdkController,
        warning_v: float,
        critical_v: float,
        poll_interval_s: float = 2.0,
        log_hook: Optional[LogHook] = None,
        on_critical: Optional[AbortHook] = None,
    ) -> None:
        self.controller = controller
        self.warning_v = warning_v
        self.critical_v = critical_v
        self.poll_interval_s = max(0.5, poll_interval_s)
        self.log_hook = log_hook
        self.on_critical = on_critical

        self._warned = False
        self._triggered = False
        self._task: Optional[asyncio.Task] = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="battery-monitor")

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._task = None

    async def _run(self) -> None:
        try:
            while True:
                v = self.controller.state.battery_voltage_v
                if math.isfinite(v) and v > 0.0:
                    if not self._triggered and v < self.critical_v:
                        self._triggered = True
                        self._emit(
                            "ERROR",
                            f"Battery CRITICAL {v:.2f} V "
                            f"(< {self.critical_v:.2f} V) - auto-RTL")
                        if self.on_critical is not None:
                            try:
                                await self.on_critical()
                            except Exception as exc:  # noqa: BLE001
                                logger.exception(
                                    "auto-RTL hook failed: %s", exc)
                        return
                    if (not self._warned
                            and not self._triggered
                            and v < self.warning_v):
                        self._warned = True
                        self._emit(
                            "WARN",
                            f"Battery low {v:.2f} V (< {self.warning_v:.2f} V)")
                await asyncio.sleep(self.poll_interval_s)
        except asyncio.CancelledError:
            return

    def _emit(self, level: str, message: str) -> None:
        getattr(logger, level.lower(), logger.info)(message)
        if self.log_hook is not None:
            try:
                self.log_hook(level, message)
            except Exception:  # noqa: BLE001
                pass

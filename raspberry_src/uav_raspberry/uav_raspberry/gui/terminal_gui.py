"""Rich-based terminal GUI for the UAV mission.

Layout (see CLAUDE.md spec):
  ┌─────────────── header: name, elapsed, battery, GPS fix ─────────────────┐
  │ health checks      │      mission log (scrolling)     │  mission stats │
  └──────────── keybindings: [S] Start [A] Abort [Q] Quit [C] Checks ───────┘

Non-blocking key input is read from stdin (POSIX termios).  On Windows
the GUI still renders but key input falls back to line-buffered input.
"""

from __future__ import annotations

import asyncio
import collections
import importlib
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional

import rclpy

from rich.align import Align
from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from ..run_mission import MissionRunner


CRITICAL_CHECKS = {
    "MAVLink reachable",
    "MAVSDK importable",
    "rclpy running",
    "Camera topic reachable",
}


@dataclass
class HealthCheck:
    name: str
    status: str = "PENDING"   # PASS / WARN / FAIL / PENDING
    detail: str = ""


@dataclass
class GuiState:
    elapsed_s: float = 0.0
    mission_name: str = "StrategicTech Scout"
    checks: List[HealthCheck] = field(default_factory=list)
    log_lines: Deque[tuple] = field(
        default_factory=lambda: collections.deque(maxlen=200))
    last_key: str = ""


def _posix_key_reader(out_queue: "asyncio.Queue[str]",
                      loop: asyncio.AbstractEventLoop) -> None:
    try:
        import termios
        import tty
    except ImportError:
        # Windows: line-buffered fallback
        while True:
            line = sys.stdin.readline()
            if not line:
                return
            ch = line.strip()[:1].lower()
            if ch:
                asyncio.run_coroutine_threadsafe(out_queue.put(ch), loop)
        return

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            ch = os.read(fd, 1).decode("utf-8", errors="ignore").lower()
            if not ch:
                break
            asyncio.run_coroutine_threadsafe(out_queue.put(ch), loop)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class MissionGui:
    def __init__(self) -> None:
        self.console = Console()
        self.state = GuiState()
        self.runner: Optional[MissionRunner] = None
        self._mission_task: Optional[asyncio.Task] = None
        self._start_mono = time.monotonic()

    # ── health checks ────────────────────────────────────────────────────────

    def run_checks(self) -> None:
        checks: List[HealthCheck] = []

        checks.append(self._check_module("MAVSDK importable", "mavsdk"))
        checks.append(self._check_module("ultralytics YOLO", "ultralytics"))
        checks.append(self._check_module("pyzbar", "pyzbar.pyzbar"))
        checks.append(self._check_module("OpenCV", "cv2"))
        checks.append(self._check_module("rclpy", "rclpy"))
        checks.append(self._check_rclpy_running())
        checks.append(self._check_mavlink_reachable())
        checks.append(self._check_camera_topic())
        checks.append(self._check_yolo_model())
        checks.append(self._check_barometer())
        checks.append(self._check_gps_fix())

        self.state.checks = checks

    def _check_module(self, name: str, mod: str) -> HealthCheck:
        try:
            importlib.import_module(mod)
            return HealthCheck(name, "PASS")
        except ImportError as exc:
            critical = name in {"MAVSDK importable", "OpenCV", "rclpy"}
            return HealthCheck(name, "FAIL" if critical else "WARN", str(exc))

    def _check_rclpy_running(self) -> HealthCheck:
        if self.runner is not None and rclpy.ok():
            return HealthCheck("rclpy running", "PASS")
        return HealthCheck("rclpy running", "WARN", "ROS not yet started")

    def _check_mavlink_reachable(self) -> HealthCheck:
        url = "udp://:14540"
        if self.runner is not None and self.runner._params:
            url = str(self.runner._params.get("mavsdk_system_address", url))
        port = self._port_from_url(url)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", port))
            s.close()
            return HealthCheck("MAVLink reachable", "WARN",
                               f"no listener on udp {port} yet")
        except OSError:
            return HealthCheck("MAVLink reachable", "PASS",
                               f"udp {port} is bound")

    def _check_camera_topic(self) -> HealthCheck:
        if self.runner is None or self.runner.grabber is None:
            return HealthCheck("Camera topic reachable", "WARN",
                               "ROS not yet started")
        if self.runner.grabber.frame_count > 0:
            return HealthCheck("Camera topic reachable", "PASS",
                               f"{self.runner.grabber.frame_count} frames")
        return HealthCheck("Camera topic reachable", "WARN", "no frames yet")

    def _check_yolo_model(self) -> HealthCheck:
        path = "~/models/yolov8n.pt"
        if self.runner is not None and self.runner._params:
            path = str(self.runner._params.get("yolo_model_path", path))
        path = os.path.expanduser(path)
        if os.path.exists(path):
            return HealthCheck("YOLO model file", "PASS", path)
        return HealthCheck("YOLO model file", "WARN", f"missing: {path}")

    def _check_barometer(self) -> HealthCheck:
        ctrl = self.runner.controller if self.runner else None
        if ctrl is None:
            return HealthCheck("Barometer reading", "PENDING")
        alt = ctrl.state.altitude_baro_m
        if alt is not None and alt == alt:  # NaN check
            return HealthCheck("Barometer reading", "PASS", f"{alt:.1f} m")
        return HealthCheck("Barometer reading", "WARN", "no data")

    def _check_gps_fix(self) -> HealthCheck:
        ctrl = self.runner.controller if self.runner else None
        if ctrl is None:
            return HealthCheck("GPS fix", "PENDING")
        fix = ctrl.state.gps_fix
        sats = ctrl.state.gps_satellites
        if "3D" in fix or "RTK" in fix:
            return HealthCheck("GPS fix", "PASS", f"{fix} / {sats} sats")
        return HealthCheck("GPS fix", "WARN", f"{fix} / {sats} sats")

    @staticmethod
    def _port_from_url(url: str) -> int:
        if ":" not in url:
            return 14540
        try:
            return int(url.rsplit(":", 1)[-1])
        except ValueError:
            return 14540

    def all_critical_pass(self) -> bool:
        for c in self.state.checks:
            if c.name in CRITICAL_CHECKS and c.status != "PASS":
                return False
        return bool(self.state.checks)

    # ── rendering ────────────────────────────────────────────────────────────

    def render(self) -> Layout:
        layout = Layout()
        layout.split(
            Layout(name="header", size=3),
            Layout(name="body"),
            Layout(name="footer", size=3),
        )
        layout["body"].split_row(
            Layout(name="checks", ratio=2),
            Layout(name="log", ratio=4),
            Layout(name="stats", ratio=2),
        )

        layout["header"].update(self._header())
        layout["checks"].update(self._panel_checks())
        layout["log"].update(self._panel_log())
        layout["stats"].update(self._panel_stats())
        layout["footer"].update(self._footer())
        return layout

    def _header(self) -> Panel:
        ctrl = self.runner.controller if self.runner else None
        batt = f"{ctrl.state.battery_pct:.0f}%" if ctrl and ctrl.state.battery_pct == ctrl.state.battery_pct else "---"
        gps = f"{ctrl.state.gps_fix}/{ctrl.state.gps_satellites}sat" if ctrl else "---"
        mm, ss = divmod(int(self.state.elapsed_s), 60)
        text = Text.assemble(
            (f"  {self.state.mission_name}  ", "bold cyan"),
            (f" | elapsed {mm:02d}:{ss:02d} ", "white"),
            (f" | batt {batt} ", "yellow"),
            (f" | gps {gps} ", "green"),
        )
        return Panel(Align.center(text), border_style="cyan")

    def _panel_checks(self) -> Panel:
        table = Table.grid(padding=(0, 1))
        table.add_column(justify="left")
        table.add_column(justify="right")
        colour = {
            "PASS": "green", "FAIL": "red",
            "WARN": "yellow", "PENDING": "grey50"}
        for c in self.state.checks:
            status_text = Text(c.status, style=colour.get(c.status, "white"))
            table.add_row(c.name, status_text)
            if c.detail:
                table.add_row(f"  {c.detail}", "")
        return Panel(table, title="Health checks", border_style="blue")

    def _panel_log(self) -> Panel:
        table = Table.grid(padding=(0, 1))
        colour = {"INFO": "white", "WARN": "yellow",
                  "ERROR": "red", "DEBUG": "grey50"}
        for stamp, level, msg in list(self.state.log_lines)[-20:]:
            table.add_row(
                Text(stamp, style="grey50"),
                Text(level, style=colour.get(level, "white")),
                Text(msg))
        return Panel(table, title="Mission log", border_style="green")

    def _panel_stats(self) -> Panel:
        stats = self.runner.stats if self.runner else None
        lines = []
        if stats is None:
            lines.append(Text("mission not started", style="grey50"))
        else:
            lines.append(Text(f"phase   : {stats.phase}", style="bold"))
            lines.append(Text(
                f"waypoint: {stats.waypoint_index}/{stats.waypoint_total}"))
            lines.append(Text(
                f"found   : {stats.targets_found}"))
            lines.append(Text(
                f"identif.: {stats.targets_identified}"))
            if stats.aborted:
                lines.append(Text("ABORTED", style="bold red"))
        return Panel(Group(*lines), title="Mission stats", border_style="magenta")

    def _footer(self) -> Panel:
        can_start = self.all_critical_pass() and self._mission_task is None
        start_style = "bold green" if can_start else "dim"
        text = Text.assemble(
            (" [S] Start ", start_style),
            (" [A] Abort ", "bold red"),
            (" [Q] Quit ", "bold white"),
            (" [C] Checks ", "bold blue"),
            (f"   last key: {self.state.last_key} ", "grey50"),
        )
        return Panel(Align.center(text), border_style="white")

    # ── async main loop ──────────────────────────────────────────────────────

    async def main(self) -> None:
        keys: asyncio.Queue[str] = asyncio.Queue()
        loop = asyncio.get_event_loop()
        reader = threading.Thread(
            target=_posix_key_reader, args=(keys, loop),
            name="gui-keys", daemon=True)
        reader.start()

        self.runner = MissionRunner(log_hook=self._log_hook)
        self.runner.start_ros()
        self.run_checks()

        with Live(self.render(), console=self.console,
                  refresh_per_second=8, screen=True) as live:
            last_check = 0.0
            try:
                while True:
                    self.state.elapsed_s = time.monotonic() - self._start_mono

                    try:
                        ch = await asyncio.wait_for(keys.get(), timeout=0.1)
                    except asyncio.TimeoutError:
                        ch = ""

                    if ch:
                        self.state.last_key = ch
                    if ch == "q":
                        if self._mission_task is not None:
                            await self.runner.abort()
                            self._mission_task.cancel()
                        break
                    if ch == "c":
                        self.run_checks()
                    if ch == "a" and self._mission_task is not None:
                        await self.runner.abort()
                        self._mission_task.cancel()
                    if ch == "s" and self._mission_task is None and self.all_critical_pass():
                        self._log_hook("INFO", "Starting mission")
                        self._mission_task = asyncio.create_task(self.runner.run())

                    if time.monotonic() - last_check > 3.0:
                        self.run_checks()
                        last_check = time.monotonic()

                    live.update(self.render())

                    if self._mission_task is not None and self._mission_task.done():
                        exc = self._mission_task.exception()
                        if exc:
                            self._log_hook("ERROR", f"Mission task ended: {exc}")
                        self._mission_task = None
            finally:
                if self._mission_task is not None:
                    try:
                        await self.runner.abort()
                        self._mission_task.cancel()
                    except Exception:  # noqa: BLE001
                        pass
                self.runner.stop_ros()

    # ── mission <-> gui bridge ───────────────────────────────────────────────

    def _log_hook(self, level: str, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.state.log_lines.append((stamp, level.upper(), message))


def main() -> None:
    try:
        asyncio.run(MissionGui().main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

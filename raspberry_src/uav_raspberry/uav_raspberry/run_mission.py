"""Mission orchestrator - loads config, wires ROS + MAVSDK, runs phase 1 -> 2.

Entry point used by both the terminal GUI and the ROS launch file.
Run directly:
    ros2 run uav_raspberry run_mission --ros-args --params-file mission_params.yaml

The orchestrator is responsible for:
  * loading every parameter from mission_params.yaml (never hardcode),
  * starting rclpy on a dedicated spin thread,
  * connecting MAVSDK,
  * executing Phase 1 lawnmower + live CV detection,
  * executing Phase 2 target identification,
  * returning to home and landing,
  * tearing everything down cleanly on abort.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .mission.battery_monitor import BatteryMonitor
from .mission.camera_control import FrameGrabber
from .mission.gps_utils import GpsPoint, apply_drift_correction
from .mission.mavsdk_controller import MavsdkController, MavsdkError
from .mission.phase1_scan import Phase1Config, Phase1Scan
from .mission.phase2_identify import Phase2Config, Phase2Identify
from .mission.ros_publisher import MissionContext, MissionPublisher

LogHook = Callable[[str, str], None]   # (level, message) -> None


@dataclass
class MissionStats:
    phase: str = "IDLE"
    waypoint_index: int = 0
    waypoint_total: int = 0
    targets_found: int = 0
    targets_identified: int = 0
    started_at_mono: float = 0.0
    aborted: bool = False


class _ParamNode(Node):
    """Dedicated rclpy node just to pull declared parameters from a YAML load."""

    def __init__(self) -> None:
        super().__init__("run_mission")
        self._declare_defaults()

    def _declare_defaults(self) -> None:
        defaults: dict = {
            "mavsdk_system_address": "udp://:14540",
            "mavsdk_server_port": 50051,
            "field_length_m": 50.0,
            "field_width_m": 30.0,
            "scan_overlap_pct": 25.0,
            "home_latitude": 0.0,
            "home_longitude": 0.0,
            "home_altitude_amsl_m": 0.0,
            "drift_correction_lat_deg": 0.0,
            "drift_correction_lon_deg": 0.0,
            "scan_altitude_m": 3.8,
            "scan_speed_m_s": 2.2,
            "identify_altitude_m": 2.5,
            "identify_speed_m_s": 1.5,
            "takeoff_altitude_m": 3.8,
            "waypoint_tolerance_m": 1.0,
            "camera_fov_horizontal_deg": 70.0,
            "camera_fov_vertical_deg": 54.0,
            "camera_focal_length_mm": 4.74,
            "camera_sensor_width_mm": 6.287,
            "camera_sensor_height_mm": 4.712,
            "image_width_px": 1280,
            "image_height_px": 720,
            "camera_topic": "/ugv_camera/image_raw/compressed",
            "hsv_h_min": 0,  "hsv_h_max": 180,
            "hsv_s_min": 0,  "hsv_s_max": 60,
            "hsv_v_min": 60, "hsv_v_max": 200,
            "min_contour_area_px": 1500,
            "max_contour_area_px": 90000,
            "aspect_ratio_min": 0.5,
            "aspect_ratio_max": 2.0,
            "detection_consecutive_frames": 3,
            "target_dedup_radius_m": 1.2,
            "lock_kp": 0.004,
            "lock_max_velocity_m_s": 0.3,
            "lock_center_tolerance_px": 20,
            "lock_hold_seconds_min": 5.0,
            "lock_hold_seconds_max": 8.0,
            "lock_stable_frames": 10,
            "yolo_model_path": "~/models/yolov8n.pt",
            "yolo_confidence_threshold": 0.35,
            "yolo_imgsz": 640,
            "qr_max_attempts": 40,
            "battery_voltage_warning_v": 22.4,
            "battery_voltage_critical_v": 21.6,
            "battery_poll_interval_s": 2.0,
            "drop_servo_index": 9,
            "drop_servo_open_us": 2000,
            "drop_servo_close_us": 1000,
            "drop_servo_hold_s": 1.5,
            "topic_detection_report": "/mission/detection",
            "topic_detection_organizer": "/detection",
            "topic_mission_map": "/mission/map/compressed",
            "topic_annotated_image": "/mission/annotated/compressed",
            "topic_object_gps": "/mission/object_gps",
            "log_dir": "logs",
            "log_level": "INFO",
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

    def snapshot(self) -> dict:
        return {n: self.get_parameter(n).value for n in [
            "mavsdk_system_address", "mavsdk_server_port",
            "field_length_m", "field_width_m", "scan_overlap_pct",
            "home_latitude", "home_longitude", "home_altitude_amsl_m",
            "drift_correction_lat_deg", "drift_correction_lon_deg",
            "scan_altitude_m", "scan_speed_m_s",
            "identify_altitude_m", "identify_speed_m_s",
            "takeoff_altitude_m", "waypoint_tolerance_m",
            "camera_fov_horizontal_deg", "camera_fov_vertical_deg",
            "camera_focal_length_mm", "camera_sensor_width_mm",
            "camera_sensor_height_mm", "image_width_px", "image_height_px",
            "camera_topic",
            "hsv_h_min", "hsv_h_max", "hsv_s_min", "hsv_s_max",
            "hsv_v_min", "hsv_v_max",
            "min_contour_area_px", "max_contour_area_px",
            "aspect_ratio_min", "aspect_ratio_max",
            "detection_consecutive_frames", "target_dedup_radius_m",
            "lock_kp", "lock_max_velocity_m_s", "lock_center_tolerance_px",
            "lock_hold_seconds_min", "lock_hold_seconds_max",
            "lock_stable_frames",
            "yolo_model_path", "yolo_confidence_threshold", "yolo_imgsz",
            "qr_max_attempts",
            "battery_voltage_warning_v", "battery_voltage_critical_v",
            "battery_poll_interval_s",
            "drop_servo_index", "drop_servo_open_us",
            "drop_servo_close_us", "drop_servo_hold_s",
            "topic_detection_report", "topic_detection_organizer",
            "topic_mission_map", "topic_annotated_image", "topic_object_gps",
            "log_dir", "log_level",
        ]}


class MissionRunner:
    """Owns rclpy + MAVSDK + mission state.  Drive it from an async caller."""

    def __init__(self, log_hook: Optional[LogHook] = None) -> None:
        self.log_hook = log_hook
        self._log = logging.getLogger("mission")
        self._rclpy_started_here = False
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._param_node: Optional[_ParamNode] = None
        self._grabber: Optional[FrameGrabber] = None
        self._publisher: Optional[MissionPublisher] = None
        self._controller: Optional[MavsdkController] = None
        self._params: dict = {}
        self.stats = MissionStats()
        self._abort_event = asyncio.Event()

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start_ros(self) -> None:
        if not rclpy.ok():
            rclpy.init()
            self._rclpy_started_here = True
        self._param_node = _ParamNode()
        self._params = self._param_node.snapshot()
        self._configure_logging()

        self._grabber = FrameGrabber(self._params["camera_topic"])
        self._publisher = MissionPublisher(
            self._params["topic_detection_report"],
            self._params["topic_detection_organizer"],
            self._params["topic_annotated_image"],
            self._params["topic_object_gps"],
            self._params["topic_mission_map"],
        )
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self._param_node)
        self._executor.add_node(self._grabber)
        self._executor.add_node(self._publisher)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="rclpy-spin", daemon=True)
        self._spin_thread.start()

    def stop_ros(self) -> None:
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        for n in filter(None, (self._param_node, self._grabber, self._publisher)):
            n.destroy_node()
        self._param_node = None
        self._grabber = None
        self._publisher = None
        if self._rclpy_started_here and rclpy.ok():
            rclpy.shutdown()

    # ── mission flow ─────────────────────────────────────────────────────────

    async def run(self) -> None:
        assert self._grabber and self._publisher and self._params
        p = self._params
        home = GpsPoint(
            latitude=float(p["home_latitude"]),
            longitude=float(p["home_longitude"]),
            altitude_m=float(p["home_altitude_amsl_m"]),
        )
        home_corrected = apply_drift_correction(
            home,
            float(p["drift_correction_lat_deg"]),
            float(p["drift_correction_lon_deg"]),
        )

        ctx = MissionContext(
            field_length_m=float(p["field_length_m"]),
            field_width_m=float(p["field_width_m"]),
            home_gps=home_corrected,
        )

        phase1_cfg = Phase1Config(
            field_length_m=float(p["field_length_m"]),
            field_width_m=float(p["field_width_m"]),
            scan_altitude_m=float(p["scan_altitude_m"]),
            scan_speed_m_s=float(p["scan_speed_m_s"]),
            scan_overlap_pct=float(p["scan_overlap_pct"]),
            camera_fov_h_deg=float(p["camera_fov_horizontal_deg"]),
            sensor_width_mm=float(p["camera_sensor_width_mm"]),
            focal_length_mm=float(p["camera_focal_length_mm"]),
            image_width_px=int(p["image_width_px"]),
            image_height_px=int(p["image_height_px"]),
            hsv_lower=(int(p["hsv_h_min"]), int(p["hsv_s_min"]), int(p["hsv_v_min"])),
            hsv_upper=(int(p["hsv_h_max"]), int(p["hsv_s_max"]), int(p["hsv_v_max"])),
            min_contour_area=float(p["min_contour_area_px"]),
            max_contour_area=float(p["max_contour_area_px"]),
            aspect_min=float(p["aspect_ratio_min"]),
            aspect_max=float(p["aspect_ratio_max"]),
            consecutive_frames=int(p["detection_consecutive_frames"]),
            dedup_radius_m=float(p["target_dedup_radius_m"]),
            home_gps=home_corrected,
            drift_lat_deg=float(p["drift_correction_lat_deg"]),
            drift_lon_deg=float(p["drift_correction_lon_deg"]),
            waypoint_tolerance_m=float(p["waypoint_tolerance_m"]),
        )

        phase2_cfg = Phase2Config(
            identify_altitude_m=float(p["identify_altitude_m"]),
            waypoint_tolerance_m=float(p["waypoint_tolerance_m"]),
            lock_kp=float(p["lock_kp"]),
            lock_max_velocity_m_s=float(p["lock_max_velocity_m_s"]),
            lock_center_tolerance_px=int(p["lock_center_tolerance_px"]),
            lock_hold_seconds_min=float(p["lock_hold_seconds_min"]),
            lock_hold_seconds_max=float(p["lock_hold_seconds_max"]),
            lock_stable_frames=int(p["lock_stable_frames"]),
            image_width_px=int(p["image_width_px"]),
            image_height_px=int(p["image_height_px"]),
            yolo_model_path=str(p["yolo_model_path"]),
            yolo_confidence_threshold=float(p["yolo_confidence_threshold"]),
            yolo_imgsz=int(p["yolo_imgsz"]),
            qr_max_attempts=int(p["qr_max_attempts"]),
        )

        self._controller = MavsdkController(
            system_address=str(p["mavsdk_system_address"]),
            server_port=int(p["mavsdk_server_port"]),
            logger_=self._log,
        )
        self.stats.started_at_mono = time.monotonic()
        self._battery_monitor: Optional[BatteryMonitor] = None

        try:
            self.stats.phase = "CONNECT"
            self._emit("INFO", "Connecting MAVSDK ...")
            await self._controller.connect()

            self._battery_monitor = BatteryMonitor(
                controller=self._controller,
                warning_v=float(p["battery_voltage_warning_v"]),
                critical_v=float(p["battery_voltage_critical_v"]),
                poll_interval_s=float(p["battery_poll_interval_s"]),
                log_hook=self.log_hook,
                on_critical=self._safe_abort,
            )
            self._battery_monitor.start()

            self.stats.phase = "ARM"
            self._emit("INFO", "Arming ...")
            await self._controller.arm()

            self.stats.phase = "TAKEOFF"
            self._emit("INFO", f"Takeoff to {p['takeoff_altitude_m']:.1f} m")
            await self._controller.takeoff(float(p["takeoff_altitude_m"]))
            await self._controller.start_offboard()

            self.stats.phase = "PHASE1_SCAN"
            self._emit("INFO", "Phase 1 lawnmower scan started")
            p1 = Phase1Scan(self._controller, self._grabber,
                            self._publisher, phase1_cfg, ctx)
            targets = await p1.execute()
            self.stats.targets_found = len(targets)
            self._emit("INFO", f"Phase 1 done - {len(targets)} target(s)")

            self.stats.phase = "PHASE2_IDENTIFY"
            p2 = Phase2Identify(self._controller, self._grabber,
                                self._publisher, phase2_cfg, ctx)
            await p2.execute(targets)
            self.stats.targets_identified = sum(
                1 for t in targets if t.object_type and t.object_type != "unknown")

            self.stats.phase = "RTL"
            self._emit("INFO", "Returning to launch")
            try:
                await self._controller.stop_offboard()
            finally:
                await self._controller.return_to_launch()
            self.stats.phase = "DONE"
            self._emit("INFO", "Mission complete")
        except MavsdkError as exc:
            self._emit("ERROR", f"MAVSDK error: {exc}")
            self.stats.aborted = True
            await self._safe_abort()
        except asyncio.CancelledError:
            self._emit("WARN", "Mission cancelled by operator")
            self.stats.aborted = True
            await self._safe_abort()
            raise
        except Exception as exc:  # noqa: BLE001
            self._emit("ERROR", f"Mission failure: {exc}")
            self.stats.aborted = True
            await self._safe_abort()
        finally:
            if self._battery_monitor is not None:
                await self._battery_monitor.stop()
            if self._controller is not None:
                await self._controller.disconnect()

    async def abort(self) -> None:
        self._emit("WARN", "Abort requested - RTL + disarm")
        await self._safe_abort()

    async def _safe_abort(self) -> None:
        if self._controller is None:
            return
        try:
            await self._controller.stop_offboard()
        except Exception:  # noqa: BLE001
            pass
        try:
            await self._controller.return_to_launch()
        except Exception as exc:  # noqa: BLE001
            self._emit("ERROR", f"RTL failed: {exc}")
        try:
            await asyncio.sleep(2.0)
            await self._controller.disarm()
        except Exception:  # noqa: BLE001
            pass

    # ── helpers ──────────────────────────────────────────────────────────────

    def _configure_logging(self) -> None:
        log_dir = str(self._params.get("log_dir", "logs"))
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(log_dir, f"mission_{ts}.log")

        level = getattr(logging, str(self._params.get("log_level", "INFO")).upper(),
                        logging.INFO)
        fmt = logging.Formatter(
            "%(asctime)s  %(levelname)-7s %(name)s: %(message)s")
        root = logging.getLogger()
        root.setLevel(level)
        for h in list(root.handlers):
            root.removeHandler(h)
        fh = logging.FileHandler(log_path)
        fh.setFormatter(fmt)
        sh = logging.StreamHandler()
        sh.setFormatter(fmt)
        root.addHandler(fh)
        root.addHandler(sh)
        self._log.info("Logs -> %s", log_path)

    def _emit(self, level: str, message: str) -> None:
        getattr(self._log, level.lower(), self._log.info)(message)
        if self.log_hook is not None:
            try:
                self.log_hook(level, message)
            except Exception:  # noqa: BLE001
                pass

    # ── introspection helpers for the GUI ────────────────────────────────────

    @property
    def controller(self) -> Optional[MavsdkController]:
        return self._controller

    @property
    def grabber(self) -> Optional[FrameGrabber]:
        return self._grabber


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.parse_known_args(argv)

    runner = MissionRunner()
    runner.start_ros()
    try:
        asyncio.run(runner.run())
    except KeyboardInterrupt:
        asyncio.run(runner.abort())
    finally:
        runner.stop_ros()


if __name__ == "__main__":
    main()

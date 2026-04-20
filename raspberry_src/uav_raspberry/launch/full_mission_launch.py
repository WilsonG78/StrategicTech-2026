"""Launch: full UAV mission stack with dependency pre-flight checks.

Starts, in order:
  1. ``telemetry_node``           (MAVSDK -> /data @ 2 Hz)
  2. ``camera_streamer_focus``    (ugv_raspberry, GStreamer + V4L2 focus)
  3. ``run_mission``              (orchestrator, Phase 1 + Phase 2)

Before launching the nodes a small OpaqueFunction runs a preflight
check: mavlink-router reachable on the configured UDP ports, YOLO
model file present, required Python modules importable (mavsdk, cv2,
rich, matplotlib, pyzbar).  Failures are logged but do not prevent
launch (so you can still bring the stack up partially on a dev box) -
set ``strict_checks:=true`` to abort on any missing dependency.

``mavlink-router`` must already be forwarding the FC stream to
``mavsdk_system_address`` (default ``udp://:14541``) and to the
telemetry port ``udp://:14540``.  Launch it separately or via systemd
before this file - we do NOT start mavlink-router here because its
configuration is host-specific (UART path, baud rate).

Launch arguments:
  params_file         override mission_params.yaml path
  field_length_m      override field length (North axis)
  field_width_m       override field width  (East axis)
  home_latitude       override home GPS lat
  home_longitude      override home GPS lon
  scan_altitude_m     override Phase 1 scan altitude
  log_level           ROS 2 log level (debug/info/warn/error)
  strict_checks       abort launch if preflight fails
  test_mode           disable shutdown cascade
"""

import importlib
import os
import socket
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_REQUIRED_MODULES = ("mavsdk", "cv2", "numpy", "rich", "matplotlib", "pyzbar")


def _udp_port_open(host: str, port: int) -> bool:
    """Return True if *something* is bound to ``host:port`` (UDP)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        s.bind((host, port))
        s.close()
        return False  # we could bind -> nobody else is there
    except OSError:
        return True


def _read_yaml_value(path: Path, *keys: str, default=None):
    try:
        import yaml  # lazy - only if available
    except ImportError:
        return default
    try:
        data = yaml.safe_load(path.read_text())
    except Exception:
        return default
    node = data
    for k in keys:
        if not isinstance(node, dict) or k not in node:
            return default
        node = node[k]
    return node


def _preflight(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    strict = LaunchConfiguration("strict_checks").perform(context).lower() == "true"

    lines: list[str] = ["[preflight] ---- UAV mission dependency check ----"]
    failures: list[str] = []

    # 1. params file readable
    p = Path(params_file)
    if p.is_file():
        lines.append(f"[preflight] OK   params_file       {p}")
    else:
        msg = f"params_file not found: {p}"
        lines.append(f"[preflight] FAIL params_file       {msg}")
        failures.append(msg)

    # 2. YOLO weights
    yolo_path = _read_yaml_value(p, "run_mission", "ros__parameters",
                                 "yolo_model_path",
                                 default="~/models/yolov8n.pt")
    yolo_path = os.path.expanduser(str(yolo_path))
    if os.path.isfile(yolo_path):
        lines.append(f"[preflight] OK   yolo_model_path   {yolo_path}")
    else:
        msg = f"YOLO weights not found: {yolo_path}"
        lines.append(f"[preflight] WARN yolo_model_path   {msg}")
        failures.append(msg)

    # 3. MAVSDK UDP endpoints reachable (mavlink-router bound on them)
    for key, default in (("telemetry_node", "udp://:14540"),
                         ("run_mission", "udp://:14541")):
        url = _read_yaml_value(
            p, key, "ros__parameters",
            "connection_url" if key == "telemetry_node"
            else "mavsdk_system_address",
            default=default)
        try:
            port = int(str(url).rsplit(":", 1)[-1])
            bound = _udp_port_open("127.0.0.1", port)
        except Exception:
            bound = False
            port = -1
        if bound:
            lines.append(f"[preflight] OK   mavlink udp port  {port} ({key})")
        else:
            msg = (f"no listener on UDP {port} for {key} "
                   "- is mavlink-router running?")
            lines.append(f"[preflight] WARN mavlink udp port  {msg}")
            failures.append(msg)

    # 4. required python modules
    for mod in _REQUIRED_MODULES:
        try:
            importlib.import_module(mod)
            lines.append(f"[preflight] OK   import {mod}")
        except Exception as exc:  # noqa: BLE001
            msg = f"{mod}: {exc}"
            lines.append(f"[preflight] FAIL import {mod}    {msg}")
            failures.append(msg)

    lines.append(
        "[preflight] {}/{} checks passed"
        .format(len(lines) - 1 - len(failures), len(lines) - 1))

    actions = [LogInfo(msg=ln) for ln in lines]
    if failures and strict:
        actions.append(LogInfo(
            msg="[preflight] strict_checks=true - aborting launch."))
        actions.append(EmitEvent(event=Shutdown()))
    return actions


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("uav_raspberry")
    default_params = os.path.join(pkg_share, "config", "mission_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="mission_params.yaml override")
    field_length_arg = DeclareLaunchArgument(
        "field_length_m", default_value="",
        description="Override field length (North axis, m)")
    field_width_arg = DeclareLaunchArgument(
        "field_width_m", default_value="",
        description="Override field width (East axis, m)")
    home_lat_arg = DeclareLaunchArgument(
        "home_latitude", default_value="",
        description="Override home GPS latitude (deg)")
    home_lon_arg = DeclareLaunchArgument(
        "home_longitude", default_value="",
        description="Override home GPS longitude (deg)")
    scan_alt_arg = DeclareLaunchArgument(
        "scan_altitude_m", default_value="",
        description="Override Phase 1 scan altitude (m)")
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level: debug/info/warn/error")
    strict_arg = DeclareLaunchArgument(
        "strict_checks", default_value="false",
        description="Abort launch if any preflight check fails")
    test_mode_arg = DeclareLaunchArgument(
        "test_mode", default_value="false",
        description="Disable the shutdown cascade on node exit")

    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    test_mode = LaunchConfiguration("test_mode")

    # Mission-level overrides forwarded as ROS parameter overrides.
    def _maybe_param(arg_name: str, param_name: str):
        return {param_name: LaunchConfiguration(arg_name)}

    mission_overrides = {}
    for arg, pname in (
        ("field_length_m",  "field_length_m"),
        ("field_width_m",   "field_width_m"),
        ("home_latitude",   "home_latitude"),
        ("home_longitude",  "home_longitude"),
        ("scan_altitude_m", "scan_altitude_m"),
    ):
        mission_overrides.update(_maybe_param(arg, pname))

    telemetry = Node(
        package="uav_raspberry",
        executable="telemetry_node",
        name="telemetry_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file],
    )

    camera = Node(
        package="ugv_raspberry",
        executable="camera_streamer_focus",
        name="camera_streamer_focus",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file],
    )

    mission = Node(
        package="uav_raspberry",
        executable="run_mission",
        name="run_mission",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file, mission_overrides],
    )

    def _shutdown_on(node, tag):
        return RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    LogInfo(msg=f"[full_mission] {tag} exited - shutting down."),
                    EmitEvent(event=Shutdown()),
                ],
            ),
            condition=UnlessCondition(test_mode),
        )

    return LaunchDescription([
        params_arg,
        field_length_arg,
        field_width_arg,
        home_lat_arg,
        home_lon_arg,
        scan_alt_arg,
        log_level_arg,
        strict_arg,
        test_mode_arg,
        LogInfo(msg="[full_mission] starting UAV mission stack"),
        LogInfo(msg="[full_mission] TEST MODE - shutdown cascade disabled",
                condition=IfCondition(test_mode)),
        OpaqueFunction(function=_preflight),
        telemetry,
        camera,
        mission,
        _shutdown_on(telemetry, "telemetry_node"),
        _shutdown_on(camera, "camera_streamer_focus"),
        _shutdown_on(mission, "run_mission"),
    ])

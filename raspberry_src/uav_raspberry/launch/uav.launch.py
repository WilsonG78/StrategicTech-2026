"""
launch/uav.launch.py
====================
Launches the full ArduCopter drone system:

  1. qr_lawnmower_node  – mission management, vision, safety
  2. logistic_map_node  – chaos-map RC visualisation
  3. foxglove_bridge    – WebSocket bridge on port 8765

Parameters are loaded from  config/params.yaml  relative to the
*uav_system* package share directory.

Shutdown policy: if either mission-critical node dies, the entire
launch is terminated so the operator is immediately aware.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("uav_raspberry")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    # ── Launch arguments (CLI overrides) ──────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS 2 log level: debug / info / warn / error",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove WebSocket bridge port",
        ),
    ]

    log_level = LaunchConfiguration("log_level")
    foxglove_port = LaunchConfiguration("foxglove_port")

    # ── qr_lawnmower_node ─────────────────────────────────────────────────────
    qr_node = Node(
        package="uav_raspberry",
        executable="qr_lawnmower_node",
        name="qr_lawnmower_node",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file],
    )

    # ── logistic_map_node ─────────────────────────────────────────────────────
    logistic_node = Node(
        package="uav_raspberry",
        executable="logistic_map_node",
        name="logistic_map_node",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file],
    )

    # ── foxglove_bridge ───────────────────────────────────────────────────────
    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": foxglove_port,
                "address": "0.0.0.0",
                "tls": False,
                "topic_whitelist": [".*"],   # expose all topics to Studio
            }
        ],
    )

    # ── Shutdown cascade: if either mission node dies → kill launch ───────────
    shutdown_on_qr_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=qr_node,
            on_exit=[
                LogInfo(msg="[uav_system] qr_lawnmower_node exited – shutting down."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    shutdown_on_logistic_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=logistic_node,
            on_exit=[
                LogInfo(msg="[uav_system] logistic_map_node exited – shutting down."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )
    
    # ── ugv_raspberry: camera_streamer ────────────────────────────────────────
    camera_node = Node(
        package="ugv_raspberry",
        executable="camera_streamer",
        name="camera_streamer",
        output="screen",
        parameters=[params_file],
    )

    # ── ugv_raspberry: telemetry_node ─────────────────────────────────────────
    telemetry_node = Node(
        package="uav_raspberry",
        executable="telemetry_node",
        name="telemetry_node",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription(
        [
            *args,
            LogInfo(msg="[uav_system] Starting drone system…"),
            qr_node,
            logistic_node,
            foxglove_node,
            camera_node,
            telemetry_node,
            shutdown_on_qr_exit,
            shutdown_on_logistic_exit,
        ]
    )
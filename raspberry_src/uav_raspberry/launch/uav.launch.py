"""Launch: telemetry + camera (focus) + mission orchestrator.

Parameters are loaded from ``config/mission_params.yaml`` bundled with
the uav_raspberry package share directory.  Any critical node exiting
triggers a whole-launch shutdown (unless ``test_mode:=true``).
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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("uav_raspberry")
    params_file = os.path.join(pkg_share, "config", "mission_params.yaml")

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="ROS 2 log level: debug/info/warn/error")
    test_mode_arg = DeclareLaunchArgument(
        "test_mode", default_value="false",
        description="Disable shutdown cascade for on-bench testing")

    log_level = LaunchConfiguration("log_level")
    test_mode = LaunchConfiguration("test_mode")

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
        parameters=[params_file],
    )

    def _shutdown_on(node, tag):
        return RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    LogInfo(msg=f"[uav] {tag} exited - shutting down."),
                    EmitEvent(event=Shutdown()),
                ],
            ),
            condition=UnlessCondition(test_mode),
        )

    return LaunchDescription([
        log_level_arg,
        test_mode_arg,
        LogInfo(msg="[uav] bringing up mission stack"),
        LogInfo(msg="[uav] TEST MODE - shutdown cascade disabled",
                condition=IfCondition(test_mode)),
        telemetry,
        camera,
        mission,
        _shutdown_on(telemetry, "telemetry_node"),
        _shutdown_on(camera, "camera_streamer_focus"),
        _shutdown_on(mission, "run_mission"),
    ])

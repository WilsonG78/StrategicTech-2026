from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Log level: debug/info/warn/error"),
    ]

    motor_node = Node(
        package    = "ugv_raspberry",
        executable = "motor_controller",
        name       = "motor_controller",
        output     = "screen",
        arguments  = ["--ros-args", "--log-level",
                      LaunchConfiguration("log_level")],
    )

    shutdown_on_motor_exit = RegisterEventHandler(
        OnProcessExit(
            target_action = motor_node,
            on_exit       = [
                LogInfo(msg="[ugv] motor_controller exited – shutting down."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    return LaunchDescription([
        *args,
        LogInfo(msg="[ugv] Starting motor_controller..."),
        motor_node,
        shutdown_on_motor_exit,
    ])

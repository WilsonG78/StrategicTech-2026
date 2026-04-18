from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("cam_width",     default_value="640",
                              description="Image width [px]"),
        DeclareLaunchArgument("cam_height",    default_value="480",
                              description="Image height [px]"),
        DeclareLaunchArgument("fps",           default_value="30",
                              description="Camera FPS"),
        DeclareLaunchArgument("jpeg_quality",  default_value="50",
                              description="JPEG quality (0-100). Lower = less latency"),
        DeclareLaunchArgument("log_level",     default_value="info",
                              description="Log level: debug/info/warn/error"),
    ]

    motor_node = Node(
        package    = "robot_control",
        executable = "motor_controller",
        name       = "motor_controller",
        output     = "screen",
        arguments  = ["--ros-args", "--log-level",
                      LaunchConfiguration("log_level")],
    )

    # ── camera_streamer ───────────────────────────────────────────────────────
    camera_node = Node(
        package    = "robot_control",
        executable = "camera_streamer",
        name       = "camera_streamer",
        output     = "screen",
        arguments  = ["--ros-args", "--log-level",
                      LaunchConfiguration("log_level")],
        parameters = [{
            "width":        LaunchConfiguration("cam_width"),
            "height":       LaunchConfiguration("cam_height"),
            "fps":          LaunchConfiguration("fps"),
            "jpeg_quality": LaunchConfiguration("jpeg_quality"),
        }],
    )

    # ── Shutdown the entire launch if the camera exits ───────────────────────
    shutdown_on_camera_exit = RegisterEventHandler(
        OnProcessExit(
            target_action = camera_node,
            on_exit       = [
                LogInfo(msg="CAMERA exited – shutting down launch."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    # ── Shutdown the entire launch if the motor controller exits ──────────────
    shutdown_on_motor_exit = RegisterEventHandler(
        OnProcessExit(
            target_action = motor_node,
            on_exit       = [
                LogInfo(msg="MOTOR CONTROLLER exited – shutting down launch."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    return LaunchDescription([
        *args,
        LogInfo(msg="[robot.launch] Starting motor_controller + camera_streamer..."),
        motor_node,
        camera_node,
        shutdown_on_camera_exit,
        shutdown_on_motor_exit,
    ])

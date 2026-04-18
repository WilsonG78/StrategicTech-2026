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
        DeclareLaunchArgument(
            "connection_url",
            default_value="serial:///dev/ttyAMA0:115200",
            description="MAVSDK connection URL to ArduPilot",
        ),
        DeclareLaunchArgument(
            "cam_width",
            default_value="640",
            description="Camera image width [px]",
        ),
        DeclareLaunchArgument(
            "cam_height",
            default_value="480",
            description="Camera image height [px]",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="30",
            description="Camera FPS",
        ),
        DeclareLaunchArgument(
            "jpeg_quality",
            default_value="50",
            description="JPEG quality (0-100). Lower = less latency",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level: debug/info/warn/error",
        ),
    ]

    # ── Telemetry node (MAVSDK → /data) ──────────────────────────────────────
    telemetry_node = Node(
        package    = "uav_raspberry",
        executable = "telemetry_node",
        name       = "telemetry_node",
        output     = "screen",
        arguments  = ["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters = [{
            "connection_url": LaunchConfiguration("connection_url"),
        }],
    )

    # ── Camera streamer (libcamera + GStreamer → /camera/image/compressed) ───
    camera_node = Node(
        package    = "ugv_raspberry",
        executable = "camera_streamer",
        name       = "camera_streamer",
        output     = "screen",
        arguments  = ["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters = [{
            "width":        LaunchConfiguration("cam_width"),
            "height":       LaunchConfiguration("cam_height"),
            "fps":          LaunchConfiguration("fps"),
            "jpeg_quality": LaunchConfiguration("jpeg_quality"),
        }],
    )

    # ── Shutdown everything if either node dies ───────────────────────────────
    shutdown_on_telemetry_exit = RegisterEventHandler(
        OnProcessExit(
            target_action = telemetry_node,
            on_exit       = [
                LogInfo(msg="TELEMETRY NODE exited – shutting down launch."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    shutdown_on_camera_exit = RegisterEventHandler(
        OnProcessExit(
            target_action = camera_node,
            on_exit       = [
                LogInfo(msg="CAMERA STREAMER exited – shutting down launch."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    return LaunchDescription([
        *args,
        LogInfo(msg="[uav.launch] Starting telemetry_node + camera_streamer..."),
        telemetry_node,
        camera_node,
        shutdown_on_telemetry_exit,
        shutdown_on_camera_exit,
    ])

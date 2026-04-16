"""
robot.launch.py
Starts simultaneously:
  - motor_controller  -- motor control (C++, RT thread, HW PWM)
  - camera_streamer   -- camera stream from Camera Module 3 (C++, GStreamer, low latency)
  - detector_node     -- YOLO object detection, publishes Detection messages

Usage on RPi 5:
    ros2 launch robot_control robot.launch.py

Override parameters from CLI:
    ros2 launch robot_control robot.launch.py jpeg_quality:=40 fps:=24 width:=480 height:=360
"""

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

    # ── Configurable arguments from CLI ──────────────────────────────────────
    args = [
        DeclareLaunchArgument("cam_width",     default_value="640",
                              description="Image width [px]"),
        DeclareLaunchArgument("cam_height",    default_value="480",
                              description="Image height [px]"),
        DeclareLaunchArgument("fps",           default_value="30",
                              description="Camera FPS"),
        DeclareLaunchArgument("jpeg_quality",  default_value="50",
                              description="JPEG quality (0-100). Lower = lower latency"),
        DeclareLaunchArgument("log_level",     default_value="info",
                              description="Log level: debug/info/warn/error"),
        DeclareLaunchArgument("yolo_model",    default_value="best.pt",
                              description="Path to YOLO model (.pt or .onnx)"),
        DeclareLaunchArgument("yolo_conf",     default_value="0.5",
                              description="Detection confidence threshold (0.0-1.0)"),
        DeclareLaunchArgument("frames_to_skip", default_value="3",
                              description="Run inference every N-th frame (1 = every frame)"),
        DeclareLaunchArgument("publish_result", default_value="true",
                              description="Publish annotated image on /detection/image/compressed"),
    ]

    # ── motor_controller ──────────────────────────────────────────────────────
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

    # ── detector_node ─────────────────────────────────────────────────────────
    detector_node = Node(
        package    = "robot_control",
        executable = "detector_node",
        name       = "detector_node",
        output     = "screen",
        arguments  = ["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters = [{
            "yolo_model":     LaunchConfiguration("yolo_model"),
            "yolo_conf":      LaunchConfiguration("yolo_conf"),
            "frames_to_skip": LaunchConfiguration("frames_to_skip"),
            "publish_result": LaunchConfiguration("publish_result"),
        }],
    )

    def shutdown_on_exit(node, label):
        return RegisterEventHandler(
            OnProcessExit(
                target_action = node,
                on_exit = [
                    LogInfo(msg=f"{label} exited -- shutting down launch."),
                    EmitEvent(event=Shutdown()),
                ],
            )
        )

    return LaunchDescription([
        *args,
        LogInfo(msg="[robot.launch] Starting motor_controller + camera_streamer + detector_node..."),
        motor_node,
        camera_node,
        detector_node,
        shutdown_on_exit(motor_node,    "MOTOR CONTROLLER"),
        shutdown_on_exit(camera_node,   "CAMERA STREAMER"),
        shutdown_on_exit(detector_node, "DETECTOR"),
    ])

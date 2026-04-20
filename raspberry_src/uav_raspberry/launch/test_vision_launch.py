"""Launch: bench-test vision pipeline (no MAVLink, no drone).

Runs ``tools/test_vision.py`` as a subprocess inside a minimal rclpy
context so it can publish on the same topics the real mission uses
(``/mission/annotated/compressed``, ``/mission/detection``, ...).

Intended for lab checkout of:
  * GStreamer capture from the CSI camera
  * V4L2 focus_absolute mapping
  * YOLOv8 inference + pyzbar QR decoding
  * OpenCV grey-crate HSV detector

The C++ ``camera_streamer_focus`` node must NOT be running at the same
time - test_vision owns the camera device directly.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("uav_raspberry")
    default_config = os.path.join(pkg_share, "config", "mission_params.yaml")

    config_arg = DeclareLaunchArgument(
        "config", default_value=default_config,
        description="Path to mission_params.yaml")
    altitude_arg = DeclareLaunchArgument(
        "altitude", default_value="3.5",
        description="Simulated altitude [m] for focus mapping")
    model_arg = DeclareLaunchArgument(
        "model", default_value="",
        description="YOLO model path override (empty -> use config value)")
    res_arg = DeclareLaunchArgument(
        "res", default_value="1920x1080",
        description="Capture resolution WxH")
    fps_arg = DeclareLaunchArgument(
        "fps", default_value="30",
        description="Target capture fps")
    no_yolo_arg = DeclareLaunchArgument(
        "no_yolo", default_value="false",
        description="Skip YOLO inference entirely")
    no_qr_arg = DeclareLaunchArgument(
        "no_qr", default_value="false",
        description="Skip pyzbar QR decoding entirely")
    output_arg = DeclareLaunchArgument(
        "output", default_value="tools/test_output",
        description="Output directory for annotated JPEGs + summary")
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic", default_value="/ugv_camera/image_raw/compressed",
        description="Raw camera topic (matches ugv_raspberry camera_streamer_focus)")
    annotated_topic_arg = DeclareLaunchArgument(
        "annotated_topic", default_value="/mission/annotated/compressed",
        description="Annotated frame topic (matches run_mission topic_annotated_image)")
    raw_hz_arg = DeclareLaunchArgument(
        "raw_hz", default_value="5.0",
        description="Throttle rate [Hz] for raw camera publishing")
    backend_arg = DeclareLaunchArgument(
        "backend", default_value="auto",
        description="Capture backend: auto / gstreamer / picamera2")

    cmd = [
        "python3", "-m", "uav_raspberry.tools.test_vision",
        "--config", LaunchConfiguration("config"),
        "--altitude", LaunchConfiguration("altitude"),
        "--res", LaunchConfiguration("res"),
        "--fps", LaunchConfiguration("fps"),
        "--output", LaunchConfiguration("output"),
        "--ros-camera-topic", LaunchConfiguration("camera_topic"),
        "--ros-annotated-topic", LaunchConfiguration("annotated_topic"),
        "--ros-raw-hz", LaunchConfiguration("raw_hz"),
        "--backend", LaunchConfiguration("backend"),
    ]

    test_vision = ExecuteProcess(
        cmd=cmd,
        name="test_vision",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    topics = [
        "/mission/annotated/compressed  (sensor_msgs/CompressedImage)",
        "/mission/detection             (uav_msgs/DetectionReport)",
        "/detection                     (dualtech_msgs/Detection)",
        "/mission/object_gps            (sensor_msgs/NavSatFix)",
        "/mission/map/compressed        (sensor_msgs/CompressedImage)",
    ]
    banner = [LogInfo(msg="[test_vision] publishing topics:")]
    banner += [LogInfo(msg=f"  - {t}") for t in topics]
    banner.append(LogInfo(
        msg="[test_vision] ensure camera_streamer_focus is NOT running."))

    return LaunchDescription([
        config_arg,
        altitude_arg,
        model_arg,
        res_arg,
        fps_arg,
        no_yolo_arg,
        no_qr_arg,
        output_arg,
        camera_topic_arg,
        annotated_topic_arg,
        raw_hz_arg,
        backend_arg,
        *banner,
        test_vision,
    ])

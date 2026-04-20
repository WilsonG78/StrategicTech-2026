# uav_raspberry

Two-phase autonomous UAV reconnaissance mission for the Raspberry Pi 5 +
Speedybee F4V4 (ArduPilot) drone, published via ROS 2 Jazzy and driven by
MAVSDK-Python.

## Mission summary

1. **Phase 1 – lawnmower scan** at 3.5–4 m AGL, 2–2.5 m/s. HSV + contour
   filter looks for grey storage crates; a candidate is confirmed only
   after appearing in ≥ `detection_consecutive_frames` frames in a row.
   Confirmed GPS fixes are stored after de-duplication.
2. **Phase 2 – target identify**: visit each confirmed target at 2.5 m,
   engage a P-controller camera lock (NED velocity feedback from pixel
   offset, clamped to 0.3 m/s), hold 5–8 s. During hold a YOLOv8n
   inference runs in a thread-pool executor and QR codes are decoded
   with pyzbar. Per detection the node publishes:
   * `/mission/detection` – `uav_msgs/DetectionReport`
   * `/detection` – `dualtech_msgs/Detection` (organizer topic)
   * `/mission/annotated/compressed` – `sensor_msgs/CompressedImage`
   * `/mission/object_gps` – `sensor_msgs/NavSatFix`
   * `/mission/map/compressed` – top-down 2D mission map

Telemetry (`uav_msgs/UavTelemetry` on `/data`) is published at 2 Hz by
`telemetry_node`. The ugv_raspberry `camera_streamer_focus` node
subscribes to `/data` to drive focus via the altitude→focus linear map.

## Layout

```
uav_raspberry/
├── config/mission_params.yaml          ← single source of truth
├── launch/
│   ├── uav.launch.py                   ← telemetry + camera + mission
│   ├── full_mission_launch.py          ← full stack + preflight checks
│   └── test_vision_launch.py           ← bench vision test (no MAVLink)
├── uav_raspberry/
│   ├── telemetry_node.py               ← MAVSDK ↔ /data bridge
│   ├── run_mission.py                  ← orchestrator (rclpy + asyncio)
│   ├── mission/
│   │   ├── gps_utils.py
│   │   ├── mavsdk_controller.py
│   │   ├── camera_control.py
│   │   ├── ros_publisher.py
│   │   ├── phase1_scan.py
│   │   ├── phase2_identify.py
│   │   └── battery_monitor.py
│   ├── gui/terminal_gui.py             ← rich live GUI with health checks
│   └── tools/test_vision.py            ← standalone camera + YOLO + QR test
└── logs/                               ← mission_YYYYMMDD_HHMMSS.log
```

## Install

### System packages (Ubuntu 24.04 on RPi 5)

```bash
sudo apt update
sudo apt install -y \
    python3-opencv python3-numpy python3-matplotlib \
    python3-rich libzbar0 python3-pip
pip install --break-system-packages mavsdk ultralytics pyzbar
```

Place the YOLO weights at the path referenced by `yolo_model_path`
(`~/models/yolov8n.pt` by default):

```bash
mkdir -p ~/models
wget -O ~/models/yolov8n.pt \
    https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt
```

### ROS 2 build

```bash
cd ~/ros2_ws
colcon build --packages-select uav_msgs dualtech_msgs uav_raspberry ugv_raspberry \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## MAVLink fan-out

`telemetry_node` and `run_mission` both hold their own MAVSDK session,
so `mavlink-router` must forward the FC stream to two distinct local
UDP ports (defaults: 14540 + 14541). A minimal `/etc/mavlink-router/main.conf`:

```
[UartEndpoint uart]
Device = /dev/ttyAMA0
Baud   = 115200

[UdpEndpoint telemetry]
Mode     = Normal
Address  = 127.0.0.1
Port     = 14540

[UdpEndpoint mission]
Mode     = Normal
Address  = 127.0.0.1
Port     = 14541
```

## Configure

Edit `config/mission_params.yaml`. Every tuneable value lives there –
home GPS, drift correction, field dimensions, altitudes, speeds, HSV
thresholds, P-gain, YOLO path, topic names, etc. Nothing is hardcoded
in the Python code.

## Run

### Terminal GUI (recommended)

```bash
ros2 run uav_raspberry mission_gui
```

Keybindings:

```
[S] Start mission   [A] Abort (RTL + disarm)   [Q] Quit   [C] Re-run checks
```

`[S]` is only enabled when every critical check (MAVLink reachable,
MAVSDK importable, rclpy running, camera topic reachable) passes.
Abort cascades an RTL and then a disarm via MAVSDK.

### Full launch (no GUI)

```bash
# Minimal cascade (telemetry + camera + mission):
ros2 launch uav_raspberry uav.launch.py

# Full stack with preflight dependency checks:
ros2 launch uav_raspberry full_mission_launch.py \
    field_length_m:=50.0 field_width_m:=30.0 \
    home_latitude:=50.067100 home_longitude:=19.912800 \
    strict_checks:=false
```

`full_mission_launch.py` verifies mavlink-router UDP endpoints, YOLO
weights, and that `mavsdk / cv2 / rich / matplotlib / pyzbar` import
cleanly before starting any node. Pass `strict_checks:=true` to abort
launch on any failure.

### Individual nodes

```bash
ros2 run uav_raspberry telemetry_node \
    --ros-args -p connection_url:=udp://:14540
ros2 run uav_raspberry run_mission \
    --ros-args --params-file install/uav_raspberry/share/uav_raspberry/config/mission_params.yaml
```

## Bench test (no drone required)

`tools/test_vision.py` runs the full capture → detect → decode →
classify pipeline against the RPi Camera Module 3 **without** any
MAVLink or MAVSDK connection. It opens a GStreamer pipeline on the CSI
sensor, sets the V4L2 `focus_absolute` control from the configured
altitude, and displays a rich split-panel terminal UI with live stats
(fps, focus, YOLO top-class, last QR, HSV candidates) plus a scrolling
event log. Annotated JPEGs are saved on every QR decode, YOLO
detection above threshold, and confirmed crate. A
`session_summary.json` is written on exit.

If `rclpy` is importable, `test_vision` also acts as a ROS 2 bridge:
- `/ugv_camera/image_raw/compressed` — raw JPEGs throttled to
  `ros_raw_publish_hz` (default 5 Hz), same topic as
  `camera_streamer_focus`
- `/mission/annotated/compressed` — annotated frames on every crate
  confirmation, QR decode, or YOLO detection > 0.5, same topic as
  `run_mission`

This means you can `ros2 topic echo /mission/annotated/compressed` (or
`rviz2` it) while the drone sits on the bench. Disable with `--no-ros`
or override topics with `--ros-camera-topic` / `--ros-annotated-topic`.

**Important:** the C++ `camera_streamer_focus` node must **not** be
running at the same time - `test_vision.py` owns the camera device.
Since `test_vision` already republishes on `/ugv_camera/image_raw/compressed`,
consumers like `run_mission` downstream will not notice the difference.

```bash
# Direct CLI (after colcon build + source):
ros2 run uav_raspberry test_vision \
    --config install/uav_raspberry/share/uav_raspberry/config/mission_params.yaml \
    --altitude 3.5 --res 1920x1080 --fps 30

# Or as a launch file (publishes on the same topics as the real mission):
ros2 launch uav_raspberry test_vision_launch.py altitude:=3.5
```

Keybindings: `Q` quit+summary, `S` save frame, `+/-` focus ±0.5,
`F` freeze, `R` reset counters, `M` toggle YOLO, `H` HSV mask overlay.
Headless (no `$DISPLAY`) is detected automatically; the preview window
is skipped while the rich terminal UI stays fully functional.

## Logs

Mission runs write to `logs/mission_YYYYMMDD_HHMMSS.log`. The rich GUI
mirrors the same lines in the central log panel.

# StrategicTech-2026 — Dual Tech AGH 2026

Competition code for **Dual Tech AGH 2026**, covering two events:

- **UGV** – Ground Reconnaissance (Unmanned Ground Vehicle)
- **UAV** – Aerial Reconnaissance (Unmanned Aerial Vehicle)

---

## Competition Tasks

### UGV – Ground Reconnaissance

**Goal:** Drive through a designated area in FPV mode, recognize at least 10 objects (vehicles) using QR codes + AI, and transport items to marked locations using a gripper.

**Scoring:**
- Total mission time (max 20 minutes)
- Object recognition accuracy (ID + class)
- Correct item transport with gripper

**Technical requirements:**
- FPV-only control (operator sees only the onboard camera)
- Detected objects and their IDs transmitted in real time to the organizer's ROS topic
- Detection results overlaid on the live camera image

---

### UAV – Aerial Reconnaissance

**Goal:** Fly over a designated area, map it, recognize at least 10 objects (containers with QR codes + 3D models), record GPS positions, and deliver a payload (ball) to a designated location.

**Scoring:**
- Mission time (max 15 minutes) — mapping and recognition phase only
- Object recognition accuracy
- GPS coordinate accuracy
- Correct payload delivery (drop from min. 2 m)
- Bonus: fully autonomous execution

**Technical requirements:**
- Log every 1 s: GPS coordinates + altitude, barometer altitude, GPS timestamp
- Barometer altitude transmitted to organizers in real time
- After each object detection, publish to organizer's ROS topic:
  - Object GPS coordinates
  - Object type
  - Annotated image (type, coordinates, container ID)
  - Mission map with flight path and detected objects
- Flight altitude: 2–6 m AGL

---

## Repository Structure

```
StrategicTech-2026/
├── src/ros2/                           # Main project code (ROS 2)
│   ├── robot_control/                  # UGV: motors + camera + YOLO detection (RPi 5)
│   │   ├── robot_control/
│   │   │   ├── motor_controller.cpp    # Motor control (HW PWM, RT thread)
│   │   │   ├── camera_streamer.cpp     # Camera stream (GStreamer, low latency)
│   │   │   ├── detector.py             # YOLO detection + Detection publisher
│   │   │   └── take_photo.cpp          # On-demand photo capture
│   │   └── launch/robot.launch.py      # Launches motor + camera + detector
│   │
│   ├── UGV_RASPBERRY/                  # UGV Raspberry Pi
│   │   └── UGV_RASPBERRY/
│   │       ├── camera_node.py          # TCP camera → /camera/image/compressed
│   │       └── control_keys.py         # WASD keyboard → /cmd_vel
│   │
│   ├── UGV_STATION/                    # UGV Operator station
│   │   └── UGV_STATION/
│   │       └── control_listener.py     # /cmd_vel → gpiozero Robot (GPIO)
│   │
│   ├── UAV_RASPBERRY/                  # UAV Raspberry Pi (to be implemented)
│   ├── UAV_STATION/                    # UAV Operator station (to be implemented)
│   ├── dualtech/                       # Organizer package (detection publisher/subscriber)
│   └── dualtech_msgs/                  # Organizer message definition (Detection.msg)
│
├── dualtech-ros-main/                  # Original organizer packages (reference copy)
├── model/                              # AI models (YOLO .pt) — do not modify
├── external_recourses/                 # Competition documentation (PDFs, parts list)
└── .dockerfile                         # Project Docker image
```

---

## ROS 2 Data Flow

### UGV

```
[RPi Camera / TCP stream]
        |
/camera/image/compressed  (CompressedImage)
        |
   [detector.py / YOLO]
        |
/detection/image/compressed  (annotated image)
        |
  [Detection publisher]  <-- integrated in detector.py
        |
  topic: detection  (Detection: id + type + image)  -->  Organizer
```

```
[Operator station / WASD]
        |
   /cmd_vel  (Twist)
        |
 [control_listener]
        |
  GPIO --> motors L/R
```

### UAV (planned)

```
[UAV Camera]  [GPS]  [Barometer]
      |          |         |
/camera/... /gps/... /baro/...
          \    |    /
       [UAV detector]
              |
 topic: detection  (coords + type + image + map)  -->  Organizer
```

---

## Detection Message (dualtech_msgs)

```
uint32 object_id            # Object ID (e.g. QR code number)
string object_type          # Object class (e.g. "tank", "truck")
sensor_msgs/Image object_image  # Object image (with annotations)
```

---

## Getting Started

### Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### UGV — Raspberry Pi 5

```bash
# Start everything (motors + camera + YOLO detection)
ros2 launch robot_control robot.launch.py

# Lower resolution for reduced latency
ros2 launch robot_control robot.launch.py \
    cam_width:=320 cam_height:=240 fps:=30 jpeg_quality:=35
```

### UGV — Operator laptop

```bash
export ROS_DOMAIN_ID=42   # must match the RPi setting

# WASD keyboard control
ros2 run UGV_RASPBERRY control_keys

# Camera preview
ros2 run rqt_image_view rqt_image_view /camera/image/compressed

# Monitor detection output
ros2 topic echo /detection
```

---

## Dependencies

| Component | Technology |
|-----------|-----------|
| ROS 2 | Jazzy |
| Object detection | YOLOv8 (Ultralytics) |
| Camera | libcamera / GStreamer / OpenCV |
| UGV motors | gpiozero, HW PWM (RPi 5) |
| Image conversion | cv_bridge |
| Platform | Raspberry Pi 5, Ubuntu Server 24.04 |

See [`src/ros2/robot_control/README.md`](src/ros2/robot_control/README.md) for detailed installation instructions.

---

## Contribution Rules

- Test code before deploying to hardware
- Develop new features on separate branches
- Operator station code → `*_STATION/` directories
- Raspberry Pi code → `*_RASPBERRY/` directories
- Do not modify the `model/` folder without team agreement
- Keep the Docker image consistent with project dependencies

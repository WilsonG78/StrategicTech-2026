# Integration Plan: dualtech-ros-main into src/ros2

## Goal

Integrate the organizer's packages (`dualtech` + `dualtech_msgs`) into the project workspace so that YOLO detection results are automatically published to the `detection` topic (type `dualtech_msgs/Detection`) as required by the competition judges.

## Current State

### Organizer packages (`dualtech-ros-main/`)

| File | Role |
|------|------|
| `dualtech_msgs/msg/Detection.msg` | Message definition: `object_id` (uint32), `object_type` (string), `object_image` (sensor_msgs/Image) |
| `dualtech/dualtech/detection_publisher.py` | Example publisher — sends dummy data to the `detection` topic every second |
| `dualtech/dualtech/detection_subscriber.py` | Organizer's subscriber — runs on their side to verify results |

### Existing code in `src/ros2/`

| Package | Role |
|---------|------|
| `robot_control/robot_control/detector.py` | Subscribes to `/camera/image/compressed`, runs YOLO, logs detected classes, publishes annotated image |
| `robot_control/robot_control/camera_streamer.cpp` | Camera stream → `/camera/image/compressed` |
| `robot_control/robot_control/motor_controller.cpp` | Subscribes to `/cmd_vel` → GPIO motors |
| `UGV_RASPBERRY/camera_node.py` | Alternative TCP camera → `/camera/image/compressed` |
| `UGV_STATION/control_listener.py` | `/cmd_vel` → gpiozero Robot |
| `UAV_RASPBERRY/init_node.py` | Placeholder — not yet implemented |
| `UAV_STATION/init_node.py` | Placeholder — not yet implemented |

---

## Completed Steps

### Step 1 — Copy organizer packages into the workspace

```
src/ros2/dualtech/       <-- copied from dualtech-ros-main/dualtech/
src/ros2/dualtech_msgs/  <-- copied from dualtech-ros-main/dualtech_msgs/
```

### Step 2 — Add dependencies to `robot_control/package.xml`

Added:
```xml
<depend>dualtech_msgs</depend>
<depend>cv_bridge</depend>
```

### Step 3 — Integrate Detection publishing into `detector.py`

`detector.py` now:
1. Imports `dualtech_msgs.msg.Detection` and `cv_bridge`
2. Creates a publisher on the `detection` topic
3. For every detected object, crops the bounding box, converts it to `sensor_msgs/Image` via `cv_bridge`, and publishes a `Detection` message

### Step 4 — Build and verify

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Start UGV stack
ros2 launch robot_control robot.launch.py

# In a second terminal — verify the detection topic is active
ros2 topic echo /detection
```

---

## Remaining Work

### UAV — `UAV_RASPBERRY` (not yet implemented)

The `UAV_RASPBERRY` package is currently a placeholder. The following nodes need to be created according to the UAV task requirements:

| Node | Task |
|------|------|
| `camera_node.py` | Publish UAV camera feed |
| `gps_node.py` | Publish GPS coordinates + altitude every 1 s |
| `baro_node.py` | Publish barometer altitude in real time |
| `detector_uav.py` | YOLO detection + Detection message with GPS metadata |
| `mission_map_node.py` | Generate mission map with flight path and detected objects |

### GPS coordinates in Detection message

The organizer's `Detection.msg` does not include GPS fields. Options:

- **Option A (recommended):** Encode coordinates in `object_type` as a JSON string — no .msg changes needed, compatible with organizer infrastructure
- **Option B:** Create a custom `DetectionGPS.msg` extending Detection — requires agreement with organizers
- **Option C:** Overlay GPS text directly on `object_image` — required by competition rules regardless

Recommendation: use **Option A** for protocol compatibility, while also implementing **Option C** (GPS text overlay on the image).

### Required data on the UAV detection topic

After each detection:
1. `detection` topic → `Detection` message with type, GPS coords, and container ID annotated on the image
2. Mission map — format to be confirmed with organizers (likely a separate topic)

---

## Priority Summary

| # | Task | File(s) | Priority |
|---|------|---------|----------|
| 1 | Copy organizer packages | `src/ros2/dualtech*` | **DONE** |
| 2 | Add `dualtech_msgs` + `cv_bridge` to package.xml | `robot_control/package.xml` | **DONE** |
| 3 | Integrate Detection publisher in detector.py | `robot_control/robot_control/detector.py` | **DONE** |
| 4 | Test: `ros2 topic echo /detection` on RPi | — | Next |
| 5 | Implement UAV_RASPBERRY nodes | `UAV_RASPBERRY/` | Important |
| 6 | GPS + barometer integration for UAV | `UAV_RASPBERRY/gps_node.py` | Important |
| 7 | UAV mission map | `UAV_RASPBERRY/mission_map_node.py` | Bonus |

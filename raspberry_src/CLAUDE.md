# UAV Subsystem – Project Context

## Hardware
- Raspberry Pi 5 connected via UART to a Speedybee F4V4 flight controller running ArduPilot.

## Software
- ROS 2 Jazzy
- MAVSDK (Python bindings)

## Workspace layout
```
raspberry_src/
├── uav_raspberry/    ← UAV telemetry node (MAVSDK → ROS 2)
├── ugv_raspberry/    ← UGV motor + camera nodes
├── uav_msgs/         ← Custom message: UavTelemetry.msg
├── dualtech/         ← Organizer package (Detection publisher/subscriber)
└── dualtech_msgs/    ← Organizer message: Detection.msg
```

## Telemetry node (`uav_raspberry`)

**Entry point:** `ros2 run uav_raspberry telemetry_node`

**Published topic:** `/data` (`uav_msgs/UavTelemetry`) at 2 Hz

**Fields published:**
| Field | Source |
|---|---|
| `latitude`, `longitude`, `altitude_gps` | `telemetry.position()` |
| `altitude_baro` | `telemetry.altitude().altitude_amsl_m` |
| `gps_unix_time` | `telemetry.unix_epoch_time()` |
| `gps_satellites` | `telemetry.gps_info()` |

**Key parameter:**
```
connection_url  (default: serial:///dev/ttyAMA0:115200)
```

Override at launch:
```bash
ros2 run uav_raspberry telemetry_node \
  --ros-args -p connection_url:=udp://:14540
```

## Build order
```bash
cd ~/ros2_ws
colcon build --packages-select uav_msgs uav_raspberry --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

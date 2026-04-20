Use claude-opus-4-5 model for all agentic tasks.

Analyze ALL files in this repository first, understand the existing
structure, ROS topics, MAVLink setup, and ugv_raspberry camera package.
Then rewrite everything from scratch as a clean, working mission system.

## Mission overview

Two-phase autonomous UAV reconnaissance mission:

PHASE 1 – lawnmower scan at 3.5–4m altitude, 2–2.5 m/s, detect grey
storage crates using OpenCV HSV + contour filter (rectangular aspect
ratio 0.5–2.0, min area threshold). False positive rejection: candidate
must appear in at least 3 consecutive frames before being added to
target list. Each confirmed candidate is stored as GPS_drone +
pixel_offset × GSD (1 mm/px at 3.5m). Lawnmower pattern is
pre-computed from field dimensions (30×50m default, configurable) with
swath width derived from camera FOV 66° at flight altitude. Fly along
the 50m axis to minimise turns. Drone starts from a given GPS home
point with a pre-applied 1m GPS drift correction baked into waypoints.

PHASE 2 – visit each confirmed target point at 2.5m altitude. At each
point hold position using GPS + camera lock (keep crate centred in
frame using pixel offset feedback to MAVLink position setpoints). Hold
for 5–8s. During hold: run YOLOv8n inference (model path configurable)
to classify the 3D object inside the crate, simultaneously decode QR
code using pyzbar. Use ugv_raspberry camera package for focus control —
adjust focus based on current altitude (linear mapping, configurable
min/max). After identification publish result to ROS topic as required
by competition rules. Then move to next target. After all targets done,
return to home point and land.

## Stack

- MAVSDK-Python for all flight control (takeoff, waypoints, position
  setpoints, landing, return to home)
- MAVLink router already running — connect via UDP on port read from
  config
- ROS 2 for all data publishing (use rclpy)
- for camera use C++ Gstream libaries
- OpenCV for all CV processing on captured frames
- pyzbar for QR decoding
- ultralytics YOLOv8 for object classification
- Python 3.10+

## Hardware specifics (use these exact values in config/mission_params.yaml)

Flight controller: SpeedyBee F405 V4 Stack (BLS 55A ESC)
  - Flash ArduPilot Copter 4.x before running any mission scripts
  - MAVLink UART on UART1 or UART2 → MAVLink Router → UDP 14550

Compute: Raspberry Pi 5 8GB
  - Power via Pololu D24V50F5 step-down (5V 5A) from 6S main battery
  - Active cooler installed — thermal throttling not expected
  - Connect to FC via USB-C or UART with MAVLink Router

Camera: Raspberry Pi Camera Module 3 (IMX708, 12MP)
  - Sensor size: 6.287mm × 4.712mm
  - Focal length: 4.74mm
  - Resolution: 4608 × 2592
  - FOV horizontal: 66° (use 66 in all GSD calculations)
  - GSD at 3.5m: ~1.0 mm/px, at 4m: ~1.15 mm/px, at 2.5m: ~0.73 mm/px
  - Interface: CSI via 22-pin FFC ribbon (20cm)
  - Use picamera2 library for capture (NOT cv2.VideoCapture)
  - Focus control: picamera2 AfMode and LensPosition controls
    altitude → LensPosition mapping (tune on site):
      2.5m → LensPosition 6.0
      3.5m → LensPosition 4.5
      4.0m → LensPosition 4.0
    expose as camera_control.set_focus_for_altitude(alt_m)

GPS: Flywoo Goku GM10 Pro V3 (QMC5883L compass)
  - Connected to FC, telemetry forwarded via MAVLink
  - Expected accuracy: ±1m CEP in open field
  - Drift correction: apply XY offset from config before generating
    lawnmower waypoints

Motors: T-Motor Velox V3 2306 1950KV on 6S
  - Estimated hover current: 18–22A total
  - Usable flight time: 6–7 min at mission speeds (reserve 20%)

Battery: Tattu R-Line 6S 2200mAh 95C
  - Nominal voltage: 22.2V, full: 25.2V, cutoff: 21.0V (3.5V/cell)
  - Low battery warning threshold in config: 22.4V (3.73V/cell)
  - Critical threshold: 21.6V (3.6V/cell) → auto RTL

Drop mechanism: servo TowerPro MG995 on FC servo rail
  - MAVLink command: DO_SET_SERVO, servo index from config
  - Drop sequence: open 90°, wait 1.5s, close

Frame: TBS Source One V5 5"
  - No specific config needed

## ROS topics to publish (per competition spec)

After each object detection publish to the topic names found in the
repository files:
  - object coordinates (GPS lat/lon)
  - object type (YOLO class label)
  - annotated image (with object type, coordinates, QR ID overlaid)
  - mission map (top-down 2D matplotlib figure showing field boundary,
    flight path so far, and all detected objects as markers) — publish
    as CompressedImage

Also publish telemetry at 1 Hz:
  - GPS coordinates + altitude
  - barometer altitude (read from MAVLink SCALED_PRESSURE or VFR_HUD)
  - GPS timestamp

## Files to create

1. `config/mission_params.yaml` — all tuneable parameters: field
   dimensions, home GPS coordinates, drift correction offsets, flight
   altitudes, speeds, scan overlap %, YOLO model path, camera focal
   length, sensor size, MAVLink UDP port, hold time per target,
   false-positive frame threshold, battery voltage thresholds, servo
   index, LensPosition mapping table

2. `mission/phase1_scan.py` — lawnmower waypoint generator + flight
   executor + real-time CV detection pipeline running in parallel thread

3. `mission/phase2_identify.py` — target visit loop with camera lock,
   YOLO, QR, ROS publishing

4. `mission/gps_utils.py` — GPS ↔ pixel offset conversion, GSD
   calculator, haversine distance, waypoint list generator for
   lawnmower pattern

5. `mission/camera_control.py` — wrapper around ugv_raspberry focus
   control, altitude→LensPosition mapping, picamera2 frame grabber
   with dual-stream configuration (main 1920×1080 + lores 640×360)

6. `mission/ros_publisher.py` — all ROS 2 publishers, message
   builders, mission map renderer

7. `mission/mavsdk_controller.py` — thin wrapper around MAVSDK-Python:
   connect, arm, takeoff, fly_to_waypoint, set_position_ned_velocity,
   land, rtl, get_telemetry, do_set_servo

8. `run_mission.py` — orchestrator that imports all modules and runs
   phase1 then phase2 sequentially

9. `gui/terminal_gui.py` — rich-based terminal GUI with live layout:
   - Header bar: mission name, elapsed time, battery %, GPS fix status
   - Left panel: system health checklist — PASS/FAIL/WARN for:
     MAVLink connection, MAVSDK connection, ROS node running, camera
     reachable, YOLO model loaded, pyzbar importable, GPS fix acquired,
     barometer reading valid
   - Centre panel: live mission log (scrolling, colour-coded by level)
   - Right panel: mission stats — targets found, targets identified,
     current phase, current waypoint index / total
   - Bottom bar: [S] Start  [A] Abort  [Q] Quit  [C] Run checks only
   - [S] only enabled when all critical checks pass
   - On abort: send MAVLink RTL via MAVSDK then disarm

10. `README.md` — install deps, configure params, run the GUI

## Test node — standalone camera + model validation (no drone required)

Create `tools/test_vision.py` — a self-contained test script that runs
the full vision pipeline without any MAVLink or MAVSDK connection.

Purpose: validate on the bench that camera capture, YOLOv8 inference,
QR decoding, and focus control all work correctly before a flight.

### Camera capture vs display resolution

Capture pipeline: always 1920×1080 — this is what goes into YOLO
inference, QR decoding, and HSV crate detector. Never downscale before
inference.

Preview window: display at 640×360 (1/3 scale) using cv2.resize only
for the preview surface. Configure picamera2 with two output streams:

  picam2.configure(picamera2.create_preview_configuration(
      main={"size": (1920, 1080), "format": "RGB888"},
      lores={"size": (640, 360), "format": "YUV420"}
  ))

Grab main for inference every frame.
Grab lores for preview display only — never feed lores to any model.

Preview window is optional — if running headless (no $DISPLAY) skip
cv2.imshow silently and log "headless mode, preview disabled". Rich
terminal GUI remains fully functional in headless mode.

Annotated saved JPEGs always use the full 1920×1080 frame.

### Behaviour

For every captured frame run in parallel threads:
  - OpenCV HSV grey-crate detector (same params as phase1_scan.py)
  - pyzbar QR decoder
  - YOLOv8n inference (same model path from config yaml)
  - focus set once at startup via camera_control.set_focus_for_altitude()
    using --altitude arg, then adjustable live with +/- keys

Display a rich-based terminal split layout alongside the preview:

  Left panel — live stats updated every 200ms:
    Camera: resolution, measured fps, focus LensPosition, exposure
    YOLO: model loaded yes/no, last inference time ms,
          top detection (class + confidence %), total detections
    QR: last decoded content (truncated 40 chars), total decoded
    Crate detector: HSV candidates in current frame,
                    confirmed (3-frame filter) this session

  Right panel — scrolling event log:
    Every QR decode: timestamp, content, pixel position
    Every YOLO detection >0.5 conf: timestamp, class, confidence
    Every confirmed crate: timestamp, pixel bbox

### Saved outputs (tools/test_output/ with timestamp prefix)

  - On QR decode: annotated JPEG with QR outline + decoded text
  - On YOLO detection >0.5: annotated JPEG with bbox + label
  - On crate confirmed: annotated JPEG with contour highlighted
  - On exit: session_summary.json with total runtime, detections per
    class, QR codes list, crate count, avg YOLO ms, avg fps

### Keybindings (shown in bottom bar)

[Q] Quit and save summary
[S] Save current frame manually
[+] Focus LensPosition +0.5
[-] Focus LensPosition -0.5
[F] Freeze/unfreeze frame (pause inference, keep display)
[R] Reset session counters
[M] Toggle YOLO on/off
[H] Toggle HSV mask overlay

### CLI arguments

  --config    path to mission_params.yaml (default: config/mission_params.yaml)
  --altitude  simulated altitude metres for focus calc (default: 3.5)
  --model     override YOLO model path
  --res       capture resolution WxH (default: 1920x1080)
  --fps       target capture fps (default: 30)
  --no-yolo   skip YOLO inference entirely
  --no-qr     skip pyzbar entirely
  --output    override output directory

### ROS test launch file

Create `launch/test_vision_launch.py` — ROS 2 launch file that:
  - starts a minimal rclpy node wrapping test_vision.py logic
  - publishes on the same topics as the real mission so you can verify
    output with ros2 topic echo without flying
  - accepts the same CLI args as test_vision.py via launch arguments
  - prints all topic names it is publishing to on startup

Create `launch/full_mission_launch.py` — ROS 2 launch file that:
  - starts all mission nodes: mavsdk_controller, ros_publisher,
    camera_control, run_mission orchestrator
  - starts MAVLink router if not already running
  - checks all dependencies before launching nodes
  - accepts field dimensions and home GPS as launch arguments

## Code quality requirements

- Every file has a module-level docstring explaining its role
- All configurable values come from config yaml, never hardcoded
- Every MAVLink/MAVSDK call wrapped in try/except with meaningful
  error messages
- Phase 1 CV pipeline runs in a daemon thread so flight continues
  uninterrupted
- Phase 2 camera lock uses a P-controller (configurable Kp) on pixel
  offset to generate NED velocity corrections, max velocity clamped
  to 0.3 m/s
- YOLOv8 inference runs in a ThreadPoolExecutor to avoid blocking the
  async MAVSDK event loop
- Mission map updated and republished every time a new object is
  confirmed
- Log everything with Python logging to both console and
  logs/mission_TIMESTAMP.log
- No placeholder comments like "# TODO implement this" — every
  function must be fully implemented and runnable
- Battery voltage monitored in background task: warn at 22.4V,
  trigger auto RTL at 21.6V regardless of mission phase
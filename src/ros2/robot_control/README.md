# robot_control – Installation Guide

## Raspberry Pi 5 (Ubuntu Server 24.04)

### 1. System dependencies
```bash
sudo apt update
sudo apt install -y \
    libgpiod-dev \
    gpiod \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libcamera \
    libcamera-dev \
    python3-libcamera \
    ros-jazzy-cv-bridge
```

### 2. GPIO / PWM permissions

```bash
# GPIO (libgpiod)
sudo usermod -aG gpio $USER

# Hardware PWM -- export channels at boot
# Add to /etc/rc.local or udev:
echo 0 | sudo tee /sys/class/pwm/pwmchip2/export
echo 1 | sudo tee /sys/class/pwm/pwmchip2/export
sudo chmod a+rw /sys/class/pwm/pwmchip2/pwm0/*
sudo chmod a+rw /sys/class/pwm/pwmchip2/pwm1/*

# SCHED_FIFO without sudo (optional, reduces jitter)
sudo setcap cap_sys_nice+ep install/robot_control/lib/robot_control/motor_controller
```

### 3. Camera activation (RPi5 Ubuntu Server)
```bash
# In /boot/firmware/config.txt add:
# dtoverlay=camera-mux-2port   <- if using a camera multiplexer
# camera_auto_detect=1

# Verify camera is visible:
libcamera-hello --list-cameras
```

### 4. Building the ROS 2 package
```bash
cd ~/ros2_ws
colcon build --packages-select dualtech_msgs robot_control --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 5. Running

**On RPi 5:**
```bash
# Start everything (motors + camera + YOLO detector)
ros2 launch robot_control robot.launch.py

# Lower resolution for reduced latency:
ros2 launch robot_control robot.launch.py \
    cam_width:=320 cam_height:=240 fps:=30 jpeg_quality:=35
```

**On the operator laptop:**
```bash
# Must match the ROS_DOMAIN_ID set on the RPi
export ROS_DOMAIN_ID=42

# Keyboard control (WASD)
ros2 run UGV_RASPBERRY control_keys

# Camera view (option 1 – rqt)
ros2 run rqt_image_view rqt_image_view /camera/image/compressed

# Camera view (option 2 – image_transport)
ros2 run image_tools showimage --ros-args -r image:=/camera/image/compressed

# Verify detection topic (organizer subscriber)
ros2 topic echo /detection
```

---

## GPIO Wiring

```
RPi 5 GPIO          L298N / TB6612FNG
──────────          ─────────────────
GPIO27  ──────────► IN1  (left  direction A)
GPIO17  ──────────► IN2  (left  direction B)
GPIO12  ──────────► ENA  (left  HW PWM  <- pwmchip2/ch0)
GPIO23  ──────────► IN3  (right direction A)
GPIO22  ──────────► IN4  (right direction B)
GPIO13  ──────────► ENB  (right HW PWM  <- pwmchip2/ch1)
```

---

## Camera latency tuning

| Parameter    | Default | Aggressive (max speed) |
|-------------|---------|------------------------|
| cam_width   | 640     | 320                    |
| cam_height  | 480     | 240                    |
| fps         | 30      | 30                     |
| jpeg_quality| 50      | 30                     |
| QoS         | BestEffort | BestEffort          |

Typical end-to-end latency over WiFi: ~50–120 ms.

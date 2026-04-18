# ugv_raspberry – Setup Guide

## Raspberry Pi 5 (Ubuntu Server 24.04)

### 1. System Dependencies
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
    python3-libcamera
```

### 2. GPIO / PWM Permissions
```bash
# GPIO (libgpiod)
sudo usermod -aG gpio $USER

# Hardware PWM – export channels at system startup
# Add to /etc/rc.local or udev:
echo 0 | sudo tee /sys/class/pwm/pwmchip2/export
echo 1 | sudo tee /sys/class/pwm/pwmchip2/export
sudo chmod a+rw /sys/class/pwm/pwmchip2/pwm0/*
sudo chmod a+rw /sys/class/pwm/pwmchip2/pwm1/*

# SCHED_FIFO without sudo (optional, reduces jitter)
sudo setcap cap_sys_nice+ep install/robot_control/lib/robot_control/motor_controller
```

### 3. Enable Camera (RPi5 Ubuntu Server)
```bash
# In /boot/firmware/config.txt add:
# dtoverlay=camera-mux-2port   ← if using a multiplexer adapter
# camera_auto_detect=1

# Verify camera is detected:
libcamera-hello --list-cameras
```

### 4. Build ROS 2 Package
```bash
# Assuming workspace at ~/ros2_ws
cd ~/ros2_ws
colcon build --packages-select robot_control --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 5. Running

**On RPi 5:**
```bash
# Start everything (motors + camera)
ros2 launch robot_control robot.launch.py

# Lower resolution and quality for reduced latency:
ros2 launch robot_control robot.launch.py \
    cam_width:=320 cam_height:=240 fps:=30 jpeg_quality:=35
```

**On laptop (operator station):**
```bash
# Set the same ROS_DOMAIN_ID as on the RPi
export ROS_DOMAIN_ID=42  # (or any value, must match both sides)

# Keyboard control
ros2 run robot_control control_keys   # or: python3 control_keys.py

# Camera view (option 1 – rqt)
ros2 run rqt_image_view rqt_image_view /camera/image/compressed

# Camera view (option 2 – image_transport)
ros2 run image_tools showimage --ros-args -r image:=/camera/image/compressed
```

---

## GPIO Wiring Diagram

```
RPi 5 GPIO          L298N / TB6612FNG
──────────          ─────────────────
GPIO27  ──────────► IN1  (left  direction A)
GPIO17  ──────────► IN2  (left  direction B)
GPIO12  ──────────► ENA  (left  hardware PWM ← pwmchip2/ch0)
GPIO23  ──────────► IN3  (right direction A)
GPIO22  ──────────► IN4  (right direction B)
GPIO13  ──────────► ENB  (right hardware PWM ← pwmchip2/ch1)
```

---

## Camera Latency Optimization

| Parameter      | Default value | Aggressive (max speed) |
|----------------|---------------|------------------------|
| cam_width      | 640           | 320                    |
| cam_height     | 480           | 240                    |
| fps            | 30            | 30                     |
| jpeg_quality   | 50            | 30                     |
| QoS            | BestEffort    | BestEffort             |

On a typical WiFi network, end-to-end latency is ~50–120 ms with these settings.

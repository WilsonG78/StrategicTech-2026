#!/usr/bin/env bash
# =============================================================================
#  RASPBERRY PI 5 – CLEAN INSTALLATION
#  Ubuntu Server 24.04 LTS (Noble) + ROS 2 Jazzy + Camera Module 3 + GStreamer
#
#  RUN EACH BLOCK MANUALLY – do not execute as a single script!
#  Some steps require a reboot or manual text file editing.
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 0 – System verification
# ─────────────────────────────────────────────────────────────────────────────

# Check Ubuntu version – should be 24.04 (Noble)
lsb_release -a

# Check kernel – should contain "raspi" (e.g. 6.8.0-1013-raspi)
uname -r

# If you do NOT have the raspi kernel, install it:
sudo apt install -y linux-raspi linux-image-raspi linux-headers-raspi
# Then reboot: sudo reboot

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 1 – System update
# ─────────────────────────────────────────────────────────────────────────────

sudo apt update && sudo apt full-upgrade -y
sudo apt install -y \
    curl wget git build-essential cmake \
    python3-pip python3-dev \
    software-properties-common \
    gnupg2 lsb-release \
    htop nano ufw

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 2 – Configure /boot/firmware/config.txt
#  (Camera, GPIO PWM, GPU memory)
# ─────────────────────────────────────────────────────────────────────────────

sudo nano /boot/firmware/config.txt

# ── Make sure the following lines are present: ───────────────────────────────
#
# [all]
# # Camera Module 3 (IMX708) – required
# camera_auto_detect=1
# dtoverlay=vc4-kms-v3d
#
# # GPU memory – camera needs at least 128 MB
# gpu_mem=128
#
# # Hardware PWM on GPIO12 and GPIO13
# dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#
# # Optional: disable Bluetooth if unused (frees resources)
# # dtoverlay=disable-bt
# ─────────────────────────────────────────────────────────────────────────────

# Save the file and reboot:
sudo reboot

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 3 – GPIO / PWM permissions for the user
# ─────────────────────────────────────────────────────────────────────────────

# Add user to required groups
sudo usermod -aG gpio,video,i2c,spi "$USER"

# udev rule for PWM – persistent access without sudo
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-pwm.rules
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
    chown -R root:gpio /sys/class/pwm && chmod -R 775 /sys/class/pwm;\
    chown -R root:gpio /sys/devices/platform/axi/*.pcie/**.pwm/pwm/pwmchip* && \
    chmod -R 775 /sys/devices/platform/axi/**.pwm/pwm/pwmchip*\
'"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# Export PWM channels (GPIO12=ch0, GPIO13=ch1)
echo 0 | sudo tee /sys/class/pwm/pwmchip2/export
echo 1 | sudo tee /sys/class/pwm/pwmchip2/export

# Verify channels are available:
ls /sys/class/pwm/pwmchip2/
# Expected output: export  npwm  power  pwm0  pwm1  subsystem  uevent  unexport

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 4 – libcamera (Camera Module 3 / IMX708)
# ─────────────────────────────────────────────────────────────────────────────

sudo apt install -y \
    libcamera0t64 \
    libcamera-dev \
    libcamera-tools \
    python3-libcamera \
    libcamera-ipa \
    rpicam-apps          # replaces older libcamera-apps on Ubuntu 24.04

# Verify camera is detected (connect the cable first):
libcamera-hello --list-cameras
# Expected output: Available cameras: 1 ... imx708 ...

# Take a test photo:
libcamera-still -o test.jpg --width 1920 --height 1080

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 5 – GStreamer with libcamera support (key for low latency)
# ─────────────────────────────────────────────────────────────────────────────

sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libcamera \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-gl \
    v4l-utils

# Verify the libcamera plugin is available:
gst-inspect-1.0 libcamerasrc
# Must list the libcamerasrc element

# ── TEST PIPELINE – low latency (MJPEG, 640x480, 30fps) ──────────────────────
# Run on RPi; on the laptop open /camera/image/compressed via ROS.
# This pipeline verifies libcamera + GStreamer work before starting the node:

gst-launch-1.0 \
    libcamerasrc ! \
    "video/x-raw,width=640,height=480,framerate=30/1,format=NV12" ! \
    videoconvert ! \
    jpegenc quality=50 ! \
    multifilesink location=/tmp/frame_%05d.jpg max-files=5

# Check that files are being created:
ls -la /tmp/frame_*.jpg

# ── TEST pipeline with hardware H264 (lowest network latency) ─────────────────
# RPi 5 has a hardware H264 encoder accessible via V4L2:
gst-inspect-1.0 v4l2h264enc

gst-launch-1.0 \
    libcamerasrc ! \
    "video/x-raw,width=640,height=480,framerate=30/1,format=NV12" ! \
    v4l2h264enc extra-controls="controls,repeat_sequence_header=1,h264_level=11,h264_profile=4,video_bitrate=500000" ! \
    "video/x-h264,level=(string)4" ! \
    h264parse ! \
    fakesink sync=false

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 6 – libgpiod (motor control)
# ─────────────────────────────────────────────────────────────────────────────

sudo apt install -y \
    libgpiod-dev \
    libgpiod2 \
    gpiod \
    python3-gpiod

# Verify RP1 chip (RPi 5 uses gpiochip4):
gpiodetect
# Expected output: gpiochip4 [pinctrl-rp1] (54 lines)

# Test motor GPIO pins:
gpioinfo gpiochip4 | grep -E "line 27|line 17|line 23|line 22|line 12|line 13"

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 7 – ROS 2 JAZZY (Ubuntu 24.04 Noble)
#  NOTE: Humble only works on Ubuntu 22.04. Use Jazzy on 24.04.
# ─────────────────────────────────────────────────────────────────────────────

# Locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# GPG key and ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# Install ROS 2 Jazzy Base (no GUI – server build)
sudo apt install -y ros-jazzy-ros-base

# Developer tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-jazzy-rmw-cyclonedds-cpp  # CycloneDDS – lower latency than default FastDDS

# Initialize rosdep
sudo rosdep init
rosdep update

# Add source to .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc   # must match the laptop!
source ~/.bashrc

# Verify installation:
ros2 --version
# Expected output: ros2, jazzy

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 8 – Workspace and building the robot_control package
# ─────────────────────────────────────────────────────────────────────────────

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy packages from this project into src/
# cp -r /path/to/src/ros2/* ~/ros2_ws/src/

cd ~/ros2_ws

# Install dependencies via rosdep
rosdep install --from-paths src --ignore-src -y

# Build in Release mode (important for RT performance)
colcon build \
    --packages-select dualtech_msgs robot_control \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 9 – RT permissions for motor node (SCHED_FIFO without sudo)
# ─────────────────────────────────────────────────────────────────────────────

sudo setcap cap_sys_nice+ep \
    ~/ros2_ws/install/robot_control/lib/robot_control/motor_controller

# Verify:
getcap ~/ros2_ws/install/robot_control/lib/robot_control/motor_controller
# Expected output: cap_sys_nice=ep

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 10 – CycloneDDS: configuration for local network (laptop <-> RPi)
# ─────────────────────────────────────────────────────────────────────────────

cat <<'EOF' > ~/cyclone_dds.xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <!-- Enter local network IP address (e.g. 192.168.1.0/24) -->
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
EOF

echo 'export CYCLONEDDS_URI=file://$HOME/cyclone_dds.xml' >> ~/.bashrc
source ~/.bashrc

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 11 – Running the stack
# ─────────────────────────────────────────────────────────────────────────────

# On RPi 5:
source ~/ros2_ws/install/setup.bash
ros2 launch robot_control robot.launch.py

# Parameters for minimum latency (lower quality, smaller frame):
ros2 launch robot_control robot.launch.py \
    cam_width:=320 cam_height:=240 fps:=30 jpeg_quality:=35

# ─────────────────────────────────────────────────────────────────────────────
#  BLOCK 12 – On the LAPTOP (Ubuntu / Windows WSL2 with ROS 2 Jazzy)
# ─────────────────────────────────────────────────────────────────────────────

# Install ROS 2 Jazzy (same process as above)
# Set the same DOMAIN_ID:
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Check that RPi topics are visible:
ros2 topic list
# Expected: /cmd_vel, /camera/image/compressed, /detection, etc.

# Keyboard control:
ros2 run UGV_RASPBERRY control_keys

# Camera preview – option 1 (rqt, requires ROS 2 desktop):
sudo apt install -y ros-jazzy-rqt-image-view
ros2 run rqt_image_view rqt_image_view /camera/image/compressed

# Camera preview – option 2 (lightweight viewer):
sudo apt install -y ros-jazzy-image-tools
ros2 run image_tools showimage --ros-args \
    -r image:=/camera/image/compressed \
    -p reliability:=best_effort

# Monitor detection output sent to organizers:
ros2 topic echo /detection

# ─────────────────────────────────────────────────────────────────────────────
#  DIAGNOSTICS – if something is not working
# ─────────────────────────────────────────────────────────────────────────────

# Camera not detected:
dmesg | grep -i "imx708\|camera\|csi"
ls /dev/video*
v4l2-ctl --list-devices

# PWM not available:
ls -la /sys/class/pwm/
dmesg | grep pwm

# GPIO not available:
gpiodetect
ls -la /dev/gpiochip*

# ROS 2 cannot see the other device:
ros2 multicast receive  # on laptop
ros2 multicast send     # on RPi (communication test)

# Check camera latency (header timestamp vs receive time):
ros2 topic delay /camera/image/compressed

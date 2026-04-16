#!/usr/bin/env bash
# =============================================================================
#  RASPBERRY PI 5 – CZYSTA INSTALACJA
#  Ubuntu Server 24.04 LTS (Noble) + ROS 2 Jazzy + Camera Module 3 + GStreamer
#
#  URUCHAMIAJ KAŻDY BLOK RĘCZNIE – nie jako jeden skrypt!
#  Niektóre kroki wymagają restartu lub edycji pliku tekstowego.
# =============================================================================

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 0 – Weryfikacja systemu
# ─────────────────────────────────────────────────────────────────────────────

# Sprawdź wersję Ubuntu – powinno być 24.04 (Noble)
lsb_release -a

# Sprawdź kernel – powinien zawierać "raspi" (np. 6.8.0-1013-raspi)
uname -r

# Jeśli NIE masz kernela raspi, zainstaluj go:
sudo apt install -y linux-raspi linux-image-raspi linux-headers-raspi
# Następnie restart: sudo reboot

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 1 – Aktualizacja systemu
# ─────────────────────────────────────────────────────────────────────────────

sudo apt update && sudo apt full-upgrade -y
sudo apt install -y \
    curl wget git build-essential cmake \
    python3-pip python3-dev \
    software-properties-common \
    gnupg2 lsb-release \
    htop nano ufw

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 2 – Konfiguracja /boot/firmware/config.txt
#  (Kamera, GPIO PWM, pamięć GPU)
# ─────────────────────────────────────────────────────────────────────────────

sudo nano /boot/firmware/config.txt

# ── Dodaj/upewnij się że są te linie: ─────────────────────────────────────────
#
# [all]
# # Kamera Module 3 (IMX708) – wymagane
# camera_auto_detect=1
# dtoverlay=vc4-kms-v3d
#
# # GPU memory – kamera potrzebuje minimum 128MB
# gpu_mem=128
#
# # Hardware PWM na GPIO12 i GPIO13
# dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#
# # Opcjonalnie: wyłącz Bluetooth jeśli nie używasz (zwalnia zasoby)
# # dtoverlay=disable-bt
# ─────────────────────────────────────────────────────────────────────────────

# Zapisz plik i zrestartuj:
sudo reboot

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 3 – Uprawnienia GPIO / PWM dla użytkownika
# ─────────────────────────────────────────────────────────────────────────────

# Dodaj użytkownika do grup
sudo usermod -aG gpio,video,i2c,spi "$USER"

# Reguła udev dla PWM – stały dostęp bez sudo
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-pwm.rules
SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
    chown -R root:gpio /sys/class/pwm && chmod -R 775 /sys/class/pwm;\
    chown -R root:gpio /sys/devices/platform/axi/*.pcie/**.pwm/pwm/pwmchip* && \
    chmod -R 775 /sys/devices/platform/axi/**.pwm/pwm/pwmchip*\
'"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# Eksportuj kanały PWM (GPIO12=ch0, GPIO13=ch1)
echo 0 | sudo tee /sys/class/pwm/pwmchip2/export
echo 1 | sudo tee /sys/class/pwm/pwmchip2/export

# Sprawdź czy kanały są dostępne:
ls /sys/class/pwm/pwmchip2/
# Powinno pokazać: export  npwm  power  pwm0  pwm1  subsystem  uevent  unexport

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 4 – libcamera (Camera Module 3 / IMX708)
# ─────────────────────────────────────────────────────────────────────────────

sudo apt install -y \
    libcamera0t64 \
    libcamera-dev \
    libcamera-tools \
    python3-libcamera \
    libcamera-ipa \
    rpicam-apps          # zastępuje starsze libcamera-apps na Ubuntu 24.04

# Sprawdź czy kamera jest wykryta (podłącz kabel przed sprawdzeniem):
libcamera-hello --list-cameras
# Powinno pokazać: Available cameras: 1 ... imx708 ...

# Zrób testowe zdjęcie:
libcamera-still -o test.jpg --width 1920 --height 1080

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 5 – GStreamer z obsługą libcamera (kluczowe dla niskiego opóźnienia)
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

# Sprawdź czy plugin libcamera jest dostępny:
gst-inspect-1.0 libcamerasrc
# Musi wylistować element libcamerasrc

# ── TEST PIPELINE – niskie opóźnienie (MJPEG, 640x480, 30fps) ────────────────
# Uruchom na RPi, na laptopie otwórz temat /camera/image/compressed przez ROS
# Ten pipeline weryfikuje że libcamera + GStreamer działają zanim odpalisz węzeł:

gst-launch-1.0 \
    libcamerasrc ! \
    "video/x-raw,width=640,height=480,framerate=30/1,format=NV12" ! \
    videoconvert ! \
    jpegenc quality=50 ! \
    multifilesink location=/tmp/frame_%05d.jpg max-files=5

# Sprawdź czy pliki się tworzą:
ls -la /tmp/frame_*.jpg

# ── TEST pipeline z hardware H264 (najniższe opóźnienie w sieci) ──────────────
# RPi 5 ma sprzętowy koder H264 dostępny przez V4L2:
gst-inspect-1.0 v4l2h264enc

gst-launch-1.0 \
    libcamerasrc ! \
    "video/x-raw,width=640,height=480,framerate=30/1,format=NV12" ! \
    v4l2h264enc extra-controls="controls,repeat_sequence_header=1,h264_level=11,h264_profile=4,video_bitrate=500000" ! \
    "video/x-h264,level=(string)4" ! \
    h264parse ! \
    fakesink sync=false

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 6 – libgpiod (sterowanie silnikami)
# ─────────────────────────────────────────────────────────────────────────────

sudo apt install -y \
    libgpiod-dev \
    libgpiod2 \
    gpiod \
    python3-gpiod

# Sprawdź chip RP1 (RPi 5 używa gpiochip4):
gpiodetect
# Powinno pokazać: gpiochip4 [pinctrl-rp1] (54 lines)

# Test pinu GPIO (np. GPIO27):
gpioinfo gpiochip4 | grep -E "line 27|line 17|line 23|line 22|line 12|line 13"

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 7 – ROS 2 JAZZY (Ubuntu 24.04 Noble)
#  UWAGA: Humble działa TYLKO na Ubuntu 22.04. Na 24.04 używaj Jazzy.
# ─────────────────────────────────────────────────────────────────────────────

# Locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Klucz GPG i repozytorium ROS 2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# Instalacja ROS 2 Jazzy Base (bez GUI – serwer)
sudo apt install -y ros-jazzy-ros-base

# Narzędzia deweloperskie
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-jazzy-rmw-cyclonedds-cpp  # CycloneDDS – niższe opóźnienie niż domyślny FastDDS

# Inicjalizacja rosdep
sudo rosdep init
rosdep update

# Dodaj source do .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc   # ten sam ID na laptopie!
source ~/.bashrc

# Sprawdź instalację:
ros2 --version
# Powinno pokazać: ros2, jazzy

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 8 – Workspace i budowanie pakietu robot_control
# ─────────────────────────────────────────────────────────────────────────────

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Skopiuj pakiet robot_control (z tego projektu) do src/
# cp -r /ścieżka/do/robot_control ~/ros2_ws/src/

cd ~/ros2_ws

# Zainstaluj zależności przez rosdep
rosdep install --from-paths src --ignore-src -y

# Budowanie w trybie Release (ważne dla RT)
colcon build \
    --packages-select robot_control \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 9 – Uprawnienia RT dla węzła silników (SCHED_FIFO bez sudo)
# ─────────────────────────────────────────────────────────────────────────────

sudo setcap cap_sys_nice+ep \
    ~/ros2_ws/install/robot_control/lib/robot_control/motor_controller

# Sprawdź:
getcap ~/ros2_ws/install/robot_control/lib/robot_control/motor_controller
# Powinno pokazać: cap_sys_nice=ep

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 10 – CycloneDDS: konfiguracja dla sieci lokalnej (laptop ↔ RPi)
# ─────────────────────────────────────────────────────────────────────────────

cat <<'EOF' > ~/cyclone_dds.xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <!-- Wpisz adres IP sieci lokalnej (np. 192.168.1.0/24) -->
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
#  BLOK 11 – Uruchomienie
# ─────────────────────────────────────────────────────────────────────────────

# Na RPi 5:
source ~/ros2_ws/install/setup.bash
ros2 launch robot_control robot.launch.py

# Parametry dla maksymalnie niskiego opóźnienia (słaba jakość, mały rozmiar):
ros2 launch robot_control robot.launch.py \
    cam_width:=320 cam_height:=240 fps:=30 jpeg_quality:=35

# ─────────────────────────────────────────────────────────────────────────────
#  BLOK 12 – Na LAPTOPIE (Ubuntu / Windows WSL2 z ROS 2 Jazzy)
# ─────────────────────────────────────────────────────────────────────────────

# Zainstaluj ROS 2 Jazzy (ten sam proces co wyżej)
# Ustaw ten sam DOMAIN_ID:
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Sprawdź czy widzisz tematy z RPi:
ros2 topic list
# Powinno pokazać: /cmd_vel, /camera/image/compressed, itp.

# Sterowanie:
python3 control_keys.py

# Podgląd kamery – opcja 1 (rqt, wymaga ROS 2 desktop):
sudo apt install -y ros-jazzy-rqt-image-view
ros2 run rqt_image_view rqt_image_view /camera/image/compressed

# Podgląd kamery – opcja 2 (prosty viewer):
sudo apt install -y ros-jazzy-image-tools
ros2 run image_tools showimage --ros-args \
    -r image:=/camera/image/compressed \
    -p reliability:=best_effort

# ─────────────────────────────────────────────────────────────────────────────
#  DIAGNOZA – jeśli coś nie działa
# ─────────────────────────────────────────────────────────────────────────────

# Kamera nie wykryta:
dmesg | grep -i "imx708\|camera\|csi"
ls /dev/video*
v4l2-ctl --list-devices

# PWM niedostępne:
ls -la /sys/class/pwm/
dmesg | grep pwm

# GPIO niedostępne:
gpiodetect
ls -la /dev/gpiochip*

# ROS 2 nie widzi drugiego urządzenia:
ros2 multicast receive  # na laptopie
ros2 multicast send     # na RPi (test komunikacji)

# Sprawdź opóźnienie kamery (timestamp w nagłówku vs czas odbioru):
ros2 topic delay /camera/image/compressed

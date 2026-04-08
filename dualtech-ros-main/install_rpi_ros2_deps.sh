#!/bin/bash
# ============================================================
# ROS 2 Jazzy - Instalacja zależności ze źródeł
# System: Debian 12 (Bookworm) / Raspberry Pi 5
# Użycie: chmod +x install_ros2_deps.sh && sudo ./install_ros2_deps.sh
# ============================================================

set -e

echo "=== Aktualizacja repozytoriów ==="
apt update && apt upgrade -y

echo "=== Instalacja zależności ROS 2 Jazzy ==="
apt install -y \
  lttng-tools \
  liblttng-ust-dev \
  liblttng-ctl-dev \
  lttng-tools-dev \
  python3-lttng \
  python3-lttngust \
  python3-lttngust-dev \
  babeltrace \
  libfreetype-dev \
  libfreetype6-dev \
  qtbase5-dev \
  qtdeclarative5-dev \
  libqt5svg5-dev \
  libqt5opengl5-dev \
  pyqt5-dev \
  python3-pyqt5 \
  python3-sip-dev \
  python3-pyqt5.sip \
  libogre-1.12-dev \
  libfreeimage-dev \
  libzzip-dev \
  libgl-dev \
  libglu1-mesa-dev \
  libxaw7-dev \
  libxrandr-dev \
  libxcursor-dev \
  libxinerama-dev \
  libxi-dev \
  libbullet-dev \
  libeigen3-dev \
  libccd-dev \
  libfcl-dev \
  libassimp-dev \
  libyaml-dev \
  libtinyxml2-dev \
  libxml2-dev \
  libspdlog-dev \
  libsqlite3-dev \
  libcurl4-openssl-dev \
  libzmq3-dev \
  libacl1-dev \
  libcunit1-dev \
  libopencv-dev \
  pkg-config \
  python3-pkgconfig \
  zlib1g-dev \
  libdw-dev \
  libelf-dev \
  python3-numpy \
  python3-netifaces \
  python3-lark \
  pybind11-dev \
  build-essential \
  cmake \
  git \
  gcc \
  g++ \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete \
  libasio-dev \
  libssl-dev \
  libpython3-dev \
  python3-venv \
  curl \
  software-properties-common

echo "=== Wszystkie zależności zainstalowane pomyślnie ==="

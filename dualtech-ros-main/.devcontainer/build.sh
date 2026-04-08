#!/bin/bash
set -e

# Set the default build type
cd /home/developer/ros2_ws
BUILD_TYPE=RelWithDebInfo
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-clean-cache \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
        -Wall -Wextra -Wpedantic
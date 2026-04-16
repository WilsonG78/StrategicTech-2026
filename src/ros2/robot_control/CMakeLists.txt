cmake_minimum_required(VERSION 3.16)
project(robot_control)

# ─── Standard C++ ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Maksymalna optymalizacja + informacje o błędach
add_compile_options(-Wall -Wextra -O2)

# ─── Zależności ROS 2 ─────────────────────────────────────────────────────────
find_package(ament_cmake     REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(std_msgs        REQUIRED)

# ─── Zależności systemowe ─────────────────────────────────────────────────────

# libgpiod (C API)
find_library(GPIOD_LIB gpiod REQUIRED)
find_path(GPIOD_INCLUDE_DIR gpiod.h REQUIRED)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST      REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP  REQUIRED gstreamer-app-1.0)

# ─── motor_controller ─────────────────────────────────────────────────────────
add_executable(motor_controller robot_control/motor_controller.cpp)

target_include_directories(motor_controller PRIVATE ${GPIOD_INCLUDE_DIR})

target_link_libraries(motor_controller
  ${GPIOD_LIB}
  pthread
)

ament_target_dependencies(motor_controller
  rclcpp
  geometry_msgs
)

# ─── camera_streamer ──────────────────────────────────────────────────────────
add_executable(camera_streamer robot_control/camera_streamer.cpp)

target_include_directories(camera_streamer PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
)

target_link_libraries(camera_streamer
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  pthread
  stdc++fs
)

target_compile_options(camera_streamer PRIVATE
  ${GST_CFLAGS_OTHER}
  ${GST_APP_CFLAGS_OTHER}
)

ament_target_dependencies(camera_streamer
  rclcpp
  sensor_msgs
  std_msgs
)

# ─── Instalacja ───────────────────────────────────────────────────────────────
install(TARGETS
  motor_controller
  camera_streamer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

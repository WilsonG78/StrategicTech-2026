cmake_minimum_required(VERSION 3.16)
project(ugv_raspberry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake     REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(std_msgs        REQUIRED)
find_package(uav_msgs        REQUIRED)
find_library(GPIOD_LIB gpiod REQUIRED)
find_path(GPIOD_INCLUDE_DIR gpiod.h REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST      REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP  REQUIRED gstreamer-app-1.0)

# ── motor_controller ──────────────────────────────────────────────────────────
add_executable(motor_controller ugv_raspberry/motor_controller.cpp)

target_include_directories(motor_controller PRIVATE ${GPIOD_INCLUDE_DIR})

target_link_libraries(motor_controller ${GPIOD_LIB} pthread)

ament_target_dependencies(motor_controller rclcpp geometry_msgs)

# ── camera_streamer (original, no focus control) ──────────────────────────────
add_executable(camera_streamer ugv_raspberry/camera_streamer.cpp)

target_include_directories(camera_streamer PRIVATE
  ${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

target_link_libraries(camera_streamer
  ${GST_LIBRARIES} ${GST_APP_LIBRARIES} pthread stdc++fs)

target_compile_options(camera_streamer PRIVATE
  ${GST_CFLAGS_OTHER} ${GST_APP_CFLAGS_OTHER})

ament_target_dependencies(camera_streamer rclcpp sensor_msgs std_msgs)

# ── camera_streamer_focus (altitude-driven dynamic focus) ─────────────────────
add_executable(camera_streamer_focus ugv_raspberry/camera_streamer_focus.cpp)

target_include_directories(camera_streamer_focus PRIVATE
  ${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

target_link_libraries(camera_streamer_focus
  ${GST_LIBRARIES} ${GST_APP_LIBRARIES} pthread stdc++fs)

target_compile_options(camera_streamer_focus PRIVATE
  ${GST_CFLAGS_OTHER} ${GST_APP_CFLAGS_OTHER})

ament_target_dependencies(camera_streamer_focus
  rclcpp sensor_msgs std_msgs uav_msgs)

# ── lint / tests ─────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ── install ───────────────────────────────────────────────────────────────────
install(TARGETS
  motor_controller
  camera_streamer
  camera_streamer_focus
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

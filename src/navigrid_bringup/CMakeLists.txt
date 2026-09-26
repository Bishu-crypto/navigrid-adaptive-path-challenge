cmake_minimum_required(VERSION 3.8)
project(navigrid_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(harmonic_bridge src/harmonic_bridge.cpp)
ament_target_dependencies(harmonic_bridge
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  rosgraph_msgs
  tf2_ros
)
target_link_libraries(harmonic_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(
  TARGETS harmonic_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_tf_broadcaster.py
    scripts/spawn_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

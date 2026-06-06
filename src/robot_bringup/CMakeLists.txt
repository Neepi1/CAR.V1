cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(runtime_readiness_probe
  src/runtime_readiness_probe.cpp
)
ament_target_dependencies(runtime_readiness_probe
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS runtime_readiness_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

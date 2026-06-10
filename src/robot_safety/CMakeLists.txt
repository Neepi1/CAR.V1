cmake_minimum_required(VERSION 3.8)
project(robot_safety)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_safety_node src/robot_safety_node.cpp)
ament_target_dependencies(
  robot_safety_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS robot_safety_node DESTINATION lib/${PROJECT_NAME})

ament_package()

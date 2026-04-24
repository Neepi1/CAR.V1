cmake_minimum_required(VERSION 3.8)
project(robot_local_state)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(local_state_node src/local_state_node.cpp)
ament_target_dependencies(
  local_state_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS local_state_node DESTINATION lib/${PROJECT_NAME})

ament_package()

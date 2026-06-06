cmake_minimum_required(VERSION 3.8)
project(robot_localization_bridge)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(localization_bridge_node src/localization_bridge_node.cpp)
ament_target_dependencies(
  localization_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2_ros
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS localization_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()

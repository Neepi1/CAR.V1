cmake_minimum_required(VERSION 3.8)
project(robot_floor_manager)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(floor_manager_node src/floor_manager_node.cpp)
target_compile_features(floor_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(
  floor_manager_node
  lifecycle_msgs
  nav2_msgs
  rclcpp
  robot_interfaces
  std_msgs
  std_srvs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS floor_manager_node DESTINATION lib/${PROJECT_NAME})

ament_package()

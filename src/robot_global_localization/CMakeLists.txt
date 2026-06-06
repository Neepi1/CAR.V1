cmake_minimum_required(VERSION 3.8)
project(robot_global_localization)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(global_localization_node
  src/global_localization_node.cpp
)
target_compile_features(global_localization_node PUBLIC cxx_std_17)
ament_target_dependencies(global_localization_node
  geometry_msgs
  rclcpp
  robot_interfaces
  std_msgs
  std_srvs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS global_localization_node DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_docking_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(docking_manager_node src/docking_manager_node.cpp)
target_compile_features(docking_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(
  docking_manager_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS docking_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

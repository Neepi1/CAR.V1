cmake_minimum_required(VERSION 3.8)
project(robot_local_state)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(local_state_node src/local_state_node.cpp)
target_compile_features(local_state_node PUBLIC cxx_std_17)
ament_target_dependencies(
  local_state_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

add_executable(imu_gyro_bias_filter_node src/imu_gyro_bias_filter_node.cpp)
target_compile_features(imu_gyro_bias_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(
  imu_gyro_bias_filter_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS local_state_node imu_gyro_bias_filter_node DESTINATION lib/${PROJECT_NAME})

ament_package()

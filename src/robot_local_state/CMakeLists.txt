cmake_minimum_required(VERSION 3.8)
project(robot_local_state)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ranger_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(local_state_node src/local_state_node.cpp)
target_compile_features(local_state_node PUBLIC cxx_std_17)
target_include_directories(local_state_node PRIVATE include)
ament_target_dependencies(
  local_state_node
  geometry_msgs
  nav_msgs
  ranger_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

add_executable(imu_gyro_bias_filter_node src/imu_gyro_bias_filter_node.cpp)
target_compile_features(imu_gyro_bias_filter_node PUBLIC cxx_std_17)
target_include_directories(imu_gyro_bias_filter_node PRIVATE include)
ament_target_dependencies(
  imu_gyro_bias_filter_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stationary_gate test/test_stationary_gate.cpp)
  target_include_directories(test_stationary_gate PRIVATE include)

  ament_add_gtest(test_spin_yaw_corrector test/test_spin_yaw_corrector.cpp)
  target_include_directories(test_spin_yaw_corrector PRIVATE include)
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS local_state_node imu_gyro_bias_filter_node DESTINATION lib/${PROJECT_NAME})

ament_package()

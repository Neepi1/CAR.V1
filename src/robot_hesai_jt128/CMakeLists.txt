cmake_minimum_required(VERSION 3.8)
project(robot_hesai_jt128)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros)
ament_export_libraries(pointcloud_accel_core)
ament_export_targets(export_pointcloud_accel_core HAS_LIBRARY_TARGET)

add_library(pointcloud_accel_core src/pointcloud_accel_core.cpp)
target_compile_features(pointcloud_accel_core PUBLIC cxx_std_17)
target_include_directories(pointcloud_accel_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  pointcloud_accel_core
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros)

add_executable(pointcloud_axis_remap_node src/pointcloud_axis_remap_node.cpp)
ament_target_dependencies(pointcloud_axis_remap_node rclcpp rclcpp_components sensor_msgs std_msgs)
add_library(pointcloud_axis_remap_component SHARED src/pointcloud_axis_remap_node.cpp)
target_compile_definitions(pointcloud_axis_remap_component PRIVATE ROBOT_HESAI_JT128_COMPONENT_ONLY)
ament_target_dependencies(pointcloud_axis_remap_component rclcpp rclcpp_components sensor_msgs std_msgs)
rclcpp_components_register_nodes(pointcloud_axis_remap_component "PointCloudAxisRemapNode")
add_executable(pointcloud_accel_axis_node src/pointcloud_accel_axis_node.cpp)
target_include_directories(pointcloud_accel_axis_node PRIVATE include)
target_link_libraries(pointcloud_accel_axis_node pointcloud_accel_core)
ament_target_dependencies(
  pointcloud_accel_axis_node
  rclcpp
  nav_msgs
  sensor_msgs)
add_executable(hesai_accel_driver_node src/hesai_accel_driver_node.cpp)
target_include_directories(hesai_accel_driver_node PRIVATE include)
target_link_libraries(hesai_accel_driver_node pointcloud_accel_core)
ament_target_dependencies(
  hesai_accel_driver_node
  nav_msgs
  rclcpp)
add_executable(pointcloud_downsample_node src/pointcloud_downsample_node.cpp)
ament_target_dependencies(pointcloud_downsample_node rclcpp sensor_msgs)
add_executable(imu_axis_remap_node src/imu_axis_remap_node.cpp)
target_compile_features(imu_axis_remap_node PUBLIC cxx_std_17)
target_include_directories(imu_axis_remap_node PRIVATE include)
ament_target_dependencies(imu_axis_remap_node rclcpp sensor_msgs tf2)
add_library(imu_axis_remap_core SHARED src/imu_axis_remap_node.cpp)
target_compile_features(imu_axis_remap_core PUBLIC cxx_std_17)
target_compile_definitions(imu_axis_remap_core PRIVATE ROBOT_IMU_LIBRARY_ONLY)
target_include_directories(imu_axis_remap_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_axis_remap_core rclcpp sensor_msgs tf2)
ament_export_targets(export_imu_axis_remap_core HAS_LIBRARY_TARGET)
add_executable(scan_republisher_node src/scan_republisher_node.cpp)
ament_target_dependencies(scan_republisher_node rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fused_normalization test/test_fused_normalization.cpp)
  target_compile_features(test_fused_normalization PRIVATE cxx_std_17)
  target_include_directories(test_fused_normalization PRIVATE include)
  ament_target_dependencies(test_fused_normalization sensor_msgs)
  ament_add_gtest(test_pointcloud_accel_core test/test_pointcloud_accel_core.cpp)
  target_compile_features(test_pointcloud_accel_core PRIVATE cxx_std_17)
  target_link_libraries(test_pointcloud_accel_core pointcloud_accel_core)
  ament_target_dependencies(test_pointcloud_accel_core rclcpp sensor_msgs std_msgs)
  add_executable(benchmark_fused_normalization test/benchmark_fused_normalization.cpp)
  target_compile_features(benchmark_fused_normalization PRIVATE cxx_std_17)
  target_include_directories(benchmark_fused_normalization PRIVATE include)
  ament_target_dependencies(benchmark_fused_normalization sensor_msgs)
  ament_add_gtest(test_scan_self_mask test/test_scan_self_mask.cpp)
  target_include_directories(test_scan_self_mask PRIVATE include)
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS pointcloud_accel_core
  EXPORT export_pointcloud_accel_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS imu_axis_remap_core
  EXPORT export_imu_axis_remap_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_axis_remap_node
  pointcloud_axis_remap_component
  pointcloud_accel_axis_node
  hesai_accel_driver_node
  pointcloud_downsample_node
  imu_axis_remap_node
  scan_republisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

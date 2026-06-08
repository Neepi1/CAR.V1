cmake_minimum_required(VERSION 3.8)
project(robot_hesai_jt128)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(pointcloud_axis_remap_node src/pointcloud_axis_remap_node.cpp)
ament_target_dependencies(pointcloud_axis_remap_node rclcpp rclcpp_components sensor_msgs std_msgs)
add_library(pointcloud_axis_remap_component SHARED src/pointcloud_axis_remap_node.cpp)
target_compile_definitions(pointcloud_axis_remap_component PRIVATE ROBOT_HESAI_JT128_COMPONENT_ONLY)
ament_target_dependencies(pointcloud_axis_remap_component rclcpp rclcpp_components sensor_msgs std_msgs)
rclcpp_components_register_nodes(pointcloud_axis_remap_component "PointCloudAxisRemapNode")
add_executable(pointcloud_downsample_node src/pointcloud_downsample_node.cpp)
ament_target_dependencies(pointcloud_downsample_node rclcpp sensor_msgs)
add_executable(imu_axis_remap_node src/imu_axis_remap_node.cpp)
ament_target_dependencies(imu_axis_remap_node rclcpp sensor_msgs tf2)
add_executable(scan_republisher_node src/scan_republisher_node.cpp)
ament_target_dependencies(scan_republisher_node rclcpp sensor_msgs)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS
  pointcloud_axis_remap_node
  pointcloud_axis_remap_component
  pointcloud_downsample_node
  imu_axis_remap_node
  scan_republisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

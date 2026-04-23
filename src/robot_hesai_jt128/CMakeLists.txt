cmake_minimum_required(VERSION 3.8)
project(robot_hesai_jt128)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(pointcloud_axis_remap_node src/pointcloud_axis_remap_node.cpp)
ament_target_dependencies(pointcloud_axis_remap_node rclcpp sensor_msgs)
add_executable(imu_axis_remap_node src/imu_axis_remap_node.cpp)
ament_target_dependencies(imu_axis_remap_node rclcpp sensor_msgs tf2)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/jt128_wrapper_node.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS pointcloud_axis_remap_node imu_axis_remap_node DESTINATION lib/${PROJECT_NAME})

ament_package()

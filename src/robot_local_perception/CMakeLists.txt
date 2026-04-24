cmake_minimum_required(VERSION 3.8)
project(robot_local_perception)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(local_perception_node src/local_perception_node.cpp)
ament_target_dependencies(
  local_perception_node
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(local_perception_node ${PCL_LIBRARIES})

install(TARGETS local_perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()

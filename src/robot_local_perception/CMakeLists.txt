cmake_minimum_required(VERSION 3.8)
project(robot_local_perception)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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
  nav_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(local_perception_node ${PCL_LIBRARIES})

add_library(local_perception_component SHARED src/local_perception_node.cpp)
target_compile_definitions(local_perception_component PRIVATE ROBOT_LOCAL_PERCEPTION_COMPONENT_ONLY)
ament_target_dependencies(
  local_perception_component
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(local_perception_component ${PCL_LIBRARIES})
rclcpp_components_register_nodes(local_perception_component "LocalPerceptionNode")

install(TARGETS
  local_perception_node
  local_perception_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()

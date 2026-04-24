cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(robot_description_static_tf_node src/static_tf_node.cpp)
ament_target_dependencies(
  robot_description_static_tf_node
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS robot_description_static_tf_node DESTINATION lib/${PROJECT_NAME})

ament_package()

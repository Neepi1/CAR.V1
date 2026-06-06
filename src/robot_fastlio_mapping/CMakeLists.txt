cmake_minimum_required(VERSION 3.8)
project(robot_fastlio_mapping)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fastlio_odom_bridge_node src/fastlio_odom_bridge_node.cpp)
ament_target_dependencies(fastlio_odom_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_msgs
  tf2_ros
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS
  fastlio_odom_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/fastlio_wrapper_node.py
  scripts/fastlio_odom_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

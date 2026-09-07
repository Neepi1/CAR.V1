cmake_minimum_required(VERSION 3.8)
project(robot_fastlio_mapping)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

add_executable(mapping_scan_tf_gate_node src/mapping_scan_tf_gate_node.cpp)
target_include_directories(mapping_scan_tf_gate_node PRIVATE include)
ament_target_dependencies(mapping_scan_tf_gate_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  fastlio_odom_bridge_node
  mapping_scan_tf_gate_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/fastlio_wrapper_node.py
  scripts/fastlio_odom_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mapping_scan_tf_gate_policy
    test/test_mapping_scan_tf_gate_policy.cpp
  )
  target_include_directories(test_mapping_scan_tf_gate_policy PRIVATE include)
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_api_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(robot_api_server_node
  src/api_time_utils.cpp
  src/bms_contact.cpp
  src/docking_job_model.cpp
  src/docking_status_utils.cpp
  src/file_utils.cpp
  src/floor_asset_resolver.cpp
  src/http_common.cpp
  src/localization_result_model.cpp
  src/map_asset_io.cpp
  src/map_asset_writer.cpp
  src/map_catalog.cpp
  src/map_manifest_io.cpp
  src/navigation_cancel_job_model.cpp
  src/poses_io.cpp
  src/runtime_map_context_io.cpp
  src/runtime_map_lookup.cpp
  src/robot_pose_model.cpp
  src/runtime_process_utils.cpp
  src/semantic_layer_io.cpp
  src/storage_models.cpp
  src/subscription_api.cpp
  src/subscription_manager.cpp
  src/tf_pose_utils.cpp
  src/robot_api_server_node.cpp
)
target_compile_features(robot_api_server_node PUBLIC cxx_std_17)
target_include_directories(robot_api_server_node PRIVATE include)
ament_target_dependencies(
  robot_api_server_node
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  tf2_msgs
)

install(TARGETS robot_api_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

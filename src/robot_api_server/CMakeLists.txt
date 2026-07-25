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
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_elevator_manager REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_map_asset_identity REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(robot_api_server_keepout
  src/file_utils.cpp
  src/http_common.cpp
  src/keepout_layer.cpp
  src/map_asset_digest.cpp
  src/map_asset_identity_binding.cpp
  src/map_asset_io.cpp
  src/map_manifest_io.cpp
  src/storage_models.cpp
)
target_compile_features(robot_api_server_keepout PUBLIC cxx_std_17)
target_include_directories(robot_api_server_keepout
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  robot_api_server_keepout
  yaml-cpp
)
ament_target_dependencies(robot_api_server_keepout robot_map_asset_identity)

add_library(robot_api_server_elevator_config
  src/elevator_configuration_module.cpp
)
target_compile_features(robot_api_server_elevator_config PUBLIC cxx_std_17)
target_include_directories(robot_api_server_elevator_config
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  robot_api_server_elevator_config
  PUBLIC
    robot_api_server_keepout
    robot_elevator_manager::robot_elevator_core
    yaml-cpp
)

add_executable(robot_api_server_node
  src/api_time_utils.cpp
  src/bms_contact.cpp
  src/docking_job_model.cpp
  src/docking_status_utils.cpp
  src/floor_asset_resolver.cpp
  src/localization_result_model.cpp
  src/map_asset_writer.cpp
  src/map_catalog.cpp
  src/navigation_cancel_job_model.cpp
  src/poses_io.cpp
  src/runtime_map_context_io.cpp
  src/runtime_map_lookup.cpp
  src/robot_pose_model.cpp
  src/runtime_process_utils.cpp
  src/semantic_layer_io.cpp
  src/subscription_api.cpp
  src/subscription_manager.cpp
  src/tf_pose_utils.cpp
  src/robot_api_server_node.cpp
)
target_compile_features(robot_api_server_node PUBLIC cxx_std_17)
target_include_directories(robot_api_server_node PRIVATE include)
target_link_libraries(
  robot_api_server_node
  robot_api_server_keepout
)
ament_target_dependencies(
  robot_api_server_node
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  robot_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  tf2_msgs
)

install(TARGETS
  robot_api_server_keepout
  robot_api_server_elevator_config
  robot_api_server_node
  EXPORT export_robot_api_server_keepout
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_floor_runtime_interlock
    test/test_floor_runtime_interlock.cpp
    src/floor_runtime_interlock.cpp
  )
  target_include_directories(test_floor_runtime_interlock PRIVATE include)

  ament_add_gtest(test_keepout_layer test/test_keepout_layer.cpp)
  target_link_libraries(test_keepout_layer robot_api_server_keepout)

  ament_add_gtest(test_map_asset_digest test/test_map_asset_digest.cpp)
  target_link_libraries(test_map_asset_digest robot_api_server_keepout)

  ament_add_gtest(
    test_map_asset_identity_binding
    test/test_map_asset_identity_binding.cpp
  )
  target_link_libraries(
    test_map_asset_identity_binding
    robot_api_server_keepout
  )

  ament_add_gtest(test_map_manifest_io test/test_map_manifest_io.cpp)
  target_link_libraries(test_map_manifest_io robot_api_server_keepout)

  ament_add_gtest(
    test_map_catalog
    test/test_map_catalog.cpp
    src/api_time_utils.cpp
    src/map_catalog.cpp
  )
  target_include_directories(test_map_catalog PRIVATE include)
  target_link_libraries(test_map_catalog robot_api_server_keepout)

  ament_add_gtest(
    test_elevator_configuration_module
    test/test_elevator_configuration_module.cpp
  )
  target_link_libraries(
    test_elevator_configuration_module
    robot_api_server_elevator_config
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_robot_api_server_keepout HAS_LIBRARY_TARGET)
ament_export_dependencies(
  robot_elevator_manager
  robot_map_asset_identity
  yaml-cpp
)

ament_package()

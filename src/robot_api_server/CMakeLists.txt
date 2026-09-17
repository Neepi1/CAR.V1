cmake_minimum_required(VERSION 3.8)
project(robot_api_server)

# An unspecified single-config build otherwise compiles this resident gateway
# without optimization. Keep symbols for field diagnosis and respect explicit
# Debug/Release/toolchain and multi-config selections.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
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
find_package(robot_safety REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(robot_api_server_keepout
  src/features/maps/catalog_activation/file_utils.cpp
  src/features/maps/catalog_activation/map_asset_digest.cpp
  src/features/maps/catalog_activation/map_asset_identity_binding.cpp
  src/features/maps/catalog_activation/map_asset_io.cpp
  src/features/maps/catalog_activation/map_manifest_io.cpp
  src/features/maps/catalog_activation/storage_models.cpp
  src/features/maps/keepout/keepout_layer.cpp
  src/infrastructure/http/http_common.cpp
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
  src/features/elevator/configuration/elevator_configuration_module.cpp
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

add_library(robot_api_server_elevator_test
  src/features/elevator/execution/elevator_test_module.cpp
)
target_compile_features(robot_api_server_elevator_test PUBLIC cxx_std_17)
target_include_directories(robot_api_server_elevator_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  robot_api_server_elevator_test
  PUBLIC
    robot_elevator_manager::robot_elevator_core
)

add_library(robot_api_server_elevator_lease_keepalive STATIC
  src/features/elevator/execution/elevator_lease_keepalive.cpp
)
target_compile_features(
  robot_api_server_elevator_lease_keepalive PUBLIC cxx_std_17
)
target_include_directories(robot_api_server_elevator_lease_keepalive
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(robot_api_server_node
  src/application/composition/application_composition_module.cpp
  src/application/routing/application_router_module.cpp
  src/application/routing/application_router_wiring.cpp
  src/application/runtime_configuration/runtime_configuration_module.cpp
  src/application/runtime_mode/runtime_mode_coordinator.cpp
  src/application/subscriptions/subscription_api.cpp
  src/application/subscriptions/subscription_configuration_module.cpp
  src/application/subscriptions/subscription_feature_module.cpp
  src/application/subscriptions/subscription_manager.cpp
  src/application/subscriptions/subscription_module.cpp
  src/features/docking/configuration/docking_configuration_module.cpp
  src/features/docking/configuration/docking_predock_pose_resolver.cpp
  src/features/docking/docking_feature_module.cpp
  src/features/docking/lifecycle/docking_correction_pause_module.cpp
  src/features/docking/lifecycle/dock_contact_interlock_module.cpp
  src/features/docking/lifecycle/docking_job_execution_module.cpp
  src/features/docking/lifecycle/docking_job_model.cpp
  src/features/docking/lifecycle/docking_job_store.cpp
  src/features/docking/lifecycle/docking_job_executor.cpp
  src/features/docking/lifecycle/docking_http_module.cpp
  src/features/docking/lifecycle/docking_runtime_module.cpp
  src/features/docking/lifecycle/docking_status_module.cpp
  src/features/docking/lifecycle/docking_status_utils.cpp
  src/features/docking/lifecycle/pre_navigation_undock_module.cpp
  src/features/docking/predock_alignment/predock_alignment_policy.cpp
  src/features/docking/predock_alignment/predock_control_module.cpp
  src/features/elevator/configuration/elevator_runtime_configuration_module.cpp
  src/features/elevator/elevator_feature_module.cpp
  src/features/elevator/elevator_module.cpp
  src/features/elevator/execution/elevator_arm_client.cpp
  src/features/elevator/execution/elevator_recovery_action_barrier.cpp
  src/features/elevator/execution/elevator_ros_runtime_port.cpp
  src/features/elevator/execution/elevator_runtime_policy.cpp
  src/features/floor_switch/floor_runtime_interlock.cpp
  src/features/floor_switch/floor_switch_configuration_module.cpp
  src/features/floor_switch/floor_switch_feature_module.cpp
  src/features/floor_switch/floor_switch_handoff_tracker.cpp
  src/features/floor_switch/floor_switch_http_transaction.cpp
  src/features/floor_switch/floor_switch_module.cpp
  src/features/floor_switch/runtime_map_context_io.cpp
  src/features/localization/amcl_runtime_status.cpp
  src/features/localization/localization_configuration_module.cpp
  src/features/localization/localization_feature_module.cpp
  src/features/localization/localization_module.cpp
  src/features/localization/localization_result_model.cpp
  src/features/localization/post_relocalization_settle_module.cpp
  src/features/localization/tf_pose_utils.cpp
  src/features/mapping/map_asset_writer.cpp
  src/features/mapping/mapping_configuration_module.cpp
  src/features/mapping/mapping_feature_module.cpp
  src/features/mapping/mapping_module.cpp
  src/features/mapping/runtime/mapping_process_runtime.cpp
  src/features/mapping/runtime/mapping_start_job.cpp
  src/features/maps/catalog_activation/api_time_utils.cpp
  src/features/maps/catalog_activation/floor_asset_resolver.cpp
  src/features/maps/catalog_activation/map_asset_filesystem.cpp
  src/features/maps/catalog_activation/map_catalog.cpp
  src/features/maps/catalog_activation/map_runtime_state_store.cpp
  src/features/maps/catalog_activation/runtime_map_lookup.cpp
  src/features/maps/maps_configuration_module.cpp
  src/features/maps/maps_feature_module.cpp
  src/features/maps/maps_module.cpp
  src/features/maps/keepout/semantic_layer_io.cpp
  src/features/maps/poses/poses_io.cpp
  src/features/navigation/configuration/navigation_configuration_module.cpp
  src/features/navigation/navigation_feature_module.cpp
  src/features/navigation/navigation_module.cpp
  src/features/navigation/navigation_state.cpp
  src/features/navigation/mission/navigation_completion_policy.cpp
  src/features/navigation/mission/navigation_goal_execution_module.cpp
  src/features/navigation/mission/navigation_goal_executor.cpp
  src/features/navigation/mission/navigation_goal_http.cpp
  src/features/navigation/mission/navigation_goal_job.cpp
  src/features/navigation/mission/navigation_goal_policy.cpp
  src/features/navigation/mission/navigation_mission_runtime.cpp
  src/features/navigation/runtime/navigation_action_runtime.cpp
  src/features/navigation/runtime/navigation_bridge_wait.cpp
  src/features/navigation/runtime/navigation_cancel_job_model.cpp
  src/features/navigation/runtime/navigation_cancel_runtime.cpp
  src/features/navigation/runtime/navigation_process_runtime.cpp
  src/features/navigation/terminal_control/navigation_terminal_control.cpp
  src/features/navigation/terminal_control/navigation_terminal_runtime_module.cpp
  src/features/power/bms_contact.cpp
  src/features/power/power_configuration_module.cpp
  src/features/power/power_feature_module.cpp
  src/features/power/power_module.cpp
  src/features/safety/safety_configuration_module.cpp
  src/features/safety/safety_feature_module.cpp
  src/features/safety/safety_module.cpp
  src/features/safety/safety_ros_adapter.cpp
  src/features/safety/safety_state.cpp
  src/features/system_status/robot_pose_model.cpp
  src/features/system_status/system_status_module.cpp
  src/features/system_status/system_status_wiring.cpp
  src/features/teleop/teleop_configuration_module.cpp
  src/features/teleop/teleop_feature_module.cpp
  src/features/teleop/teleop_module.cpp
  src/features/teleop/teleop_session.cpp
  src/infrastructure/http/api_gateway_configuration_module.cpp
  src/infrastructure/http/api_gateway_module.cpp
  src/infrastructure/process/deferred_work_queue.cpp
  src/infrastructure/process/robot_api_process.cpp
  src/infrastructure/http/http_server.cpp
  src/infrastructure/http/websocket_transport.cpp
  src/infrastructure/process/runtime_process_utils.cpp
  src/robot_api_server_node.cpp
)
target_compile_features(robot_api_server_node PUBLIC cxx_std_17)
target_include_directories(robot_api_server_node PRIVATE include)
target_link_libraries(
  robot_api_server_node
  robot_api_server_keepout
  robot_api_server_elevator_config
  robot_api_server_elevator_lease_keepalive
  robot_api_server_elevator_test
  robot_safety::persistent_sequence_allocator
  yaml-cpp
)
ament_target_dependencies(
  robot_api_server_node
  action_msgs
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
  robot_api_server_elevator_test
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
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME api_default_build_type
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_default_build_type.py)

  ament_add_gtest(
    test_application_router_module
    test/application/routing/test_application_router_module.cpp
    src/application/routing/application_router_module.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_application_router_module PRIVATE cxx_std_17)
  target_include_directories(test_application_router_module PRIVATE include)

  add_test(
    NAME application_router_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/application/routing/application_router_module_contract.py
  )

  add_test(
    NAME application_composition_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/application/composition/application_composition_module_contract.py
  )

  add_test(
    NAME robot_api_process_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/infrastructure/process/robot_api_process_contract.py
  )

  ament_add_gtest(
    test_runtime_configuration_module
    test/application/runtime_configuration/test_runtime_configuration_module.cpp
    src/application/runtime_configuration/runtime_configuration_module.cpp
  )
  target_compile_features(test_runtime_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_runtime_configuration_module PRIVATE include)
  ament_target_dependencies(test_runtime_configuration_module rclcpp)

  add_test(
    NAME runtime_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/application/runtime_configuration/runtime_configuration_module_contract.py
  )

  ament_add_gtest(
    test_runtime_mode_coordinator
    test/application/runtime_mode/test_runtime_mode_coordinator.cpp
    src/application/runtime_mode/runtime_mode_coordinator.cpp
  )
  target_compile_features(test_runtime_mode_coordinator PRIVATE cxx_std_17)
  target_include_directories(test_runtime_mode_coordinator PRIVATE include)

  ament_add_gtest(
    test_subscription_configuration_module
    test/application/subscriptions/test_subscription_configuration_module.cpp
    src/application/subscriptions/subscription_configuration_module.cpp
  )
  target_compile_features(test_subscription_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_subscription_configuration_module PRIVATE include)
  ament_target_dependencies(test_subscription_configuration_module rclcpp)

  add_test(
    NAME subscription_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/application/subscriptions/subscription_configuration_module_contract.py
  )

  ament_add_gtest(
    test_subscription_module
    test/application/subscriptions/test_subscription_module.cpp
    src/application/subscriptions/subscription_api.cpp
    src/application/subscriptions/subscription_manager.cpp
    src/application/subscriptions/subscription_module.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_subscription_module PRIVATE cxx_std_17)
  target_include_directories(test_subscription_module PRIVATE include)
  target_link_libraries(test_subscription_module yaml-cpp)
  ament_target_dependencies(test_subscription_module rclcpp sensor_msgs)

  add_test(
    NAME subscription_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/application/subscriptions/subscription_module_contract.py
  )

  ament_add_gtest(
    test_predock_alignment_policy
    test/features/docking/predock_alignment/test_predock_alignment_policy.cpp
    src/features/docking/predock_alignment/predock_alignment_policy.cpp
  )
  target_compile_features(test_predock_alignment_policy PRIVATE cxx_std_17)
  target_include_directories(test_predock_alignment_policy PRIVATE include)

  ament_add_gtest(
    test_predock_control_module
    test/features/docking/predock_alignment/test_predock_control_module.cpp
    src/features/docking/predock_alignment/predock_control_module.cpp
    src/features/docking/predock_alignment/predock_alignment_policy.cpp
    src/features/docking/lifecycle/docking_job_store.cpp
    src/features/docking/lifecycle/docking_job_model.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_predock_control_module PRIVATE cxx_std_17)
  target_include_directories(test_predock_control_module PRIVATE include)
  ament_target_dependencies(test_predock_control_module geometry_msgs rclcpp)

  ament_add_gtest(
    test_teleop_configuration_module
    test/features/teleop/test_teleop_configuration_module.cpp
    src/features/teleop/teleop_configuration_module.cpp
  )
  target_compile_features(test_teleop_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_teleop_configuration_module PRIVATE include)
  ament_target_dependencies(
    test_teleop_configuration_module
    rclcpp
    robot_elevator_manager
  )

  add_test(
    NAME teleop_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/teleop/teleop_configuration_module_contract.py
  )

  ament_add_gtest(
    test_teleop_session
    test/features/teleop/test_teleop_session.cpp
    src/features/teleop/teleop_session.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_teleop_session PRIVATE cxx_std_17)
  target_include_directories(test_teleop_session PRIVATE include)

  ament_add_gtest(
    test_teleop_module
    test/features/teleop/test_teleop_module.cpp
    src/features/teleop/teleop_module.cpp
    src/features/teleop/teleop_session.cpp
    src/infrastructure/http/http_common.cpp
    src/infrastructure/http/http_server.cpp
    src/infrastructure/http/websocket_transport.cpp
  )
  target_compile_features(test_teleop_module PRIVATE cxx_std_17)
  target_include_directories(test_teleop_module PRIVATE include)
  ament_target_dependencies(
    test_teleop_module
    geometry_msgs
    rclcpp
    robot_elevator_manager
    std_msgs
  )

  ament_add_gtest(
    test_safety_state
    test/features/safety/test_safety_state.cpp
    src/features/safety/safety_state.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_safety_state PRIVATE cxx_std_17)
  target_include_directories(test_safety_state PRIVATE include)

  ament_add_gtest(
    test_safety_configuration_module
    test/features/safety/test_safety_configuration_module.cpp
    src/features/safety/safety_configuration_module.cpp
  )
  target_compile_features(test_safety_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_safety_configuration_module PRIVATE include)
  ament_target_dependencies(test_safety_configuration_module rclcpp)

  add_test(
    NAME safety_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/safety/safety_configuration_module_contract.py
  )

  ament_add_gtest(
    test_safety_module
    test/features/safety/test_safety_module.cpp
    src/features/safety/safety_module.cpp
    src/features/safety/safety_ros_adapter.cpp
    src/features/safety/safety_state.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_safety_module PRIVATE cxx_std_17)
  target_include_directories(test_safety_module PRIVATE include)
  ament_target_dependencies(test_safety_module rclcpp std_msgs)

  add_test(
    NAME safety_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/safety/safety_module_contract.py
  )

  ament_add_gtest(
    test_safety_ros_adapter
    test/features/safety/test_safety_ros_adapter.cpp
    src/features/safety/safety_ros_adapter.cpp
    src/features/safety/safety_state.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_safety_ros_adapter PRIVATE cxx_std_17)
  target_include_directories(test_safety_ros_adapter PRIVATE include)
  ament_target_dependencies(test_safety_ros_adapter rclcpp std_msgs)

  ament_add_gtest(
    test_power_configuration_module
    test/features/power/test_power_configuration_module.cpp
    src/features/power/power_configuration_module.cpp
  )
  target_compile_features(test_power_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_power_configuration_module PRIVATE include)
  ament_target_dependencies(test_power_configuration_module rclcpp sensor_msgs)

  add_test(
    NAME power_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/power/power_configuration_module_contract.py
  )

  ament_add_gtest(
    test_power_module
    test/features/power/test_power_module.cpp
    src/features/power/power_module.cpp
    src/features/power/bms_contact.cpp
  )
  target_compile_features(test_power_module PRIVATE cxx_std_17)
  target_include_directories(test_power_module PRIVATE include)
  ament_target_dependencies(test_power_module rclcpp sensor_msgs)

  ament_add_gtest(
    test_docking_status_utils
    test/features/docking/lifecycle/test_docking_status_utils.cpp
    src/features/docking/lifecycle/docking_status_utils.cpp
  )
  target_include_directories(test_docking_status_utils PRIVATE include)

  ament_add_gtest(
    test_docking_predock_pose_resolver
    test/features/docking/configuration/test_docking_predock_pose_resolver.cpp
    src/features/docking/configuration/docking_predock_pose_resolver.cpp
    src/features/localization/tf_pose_utils.cpp
    src/features/maps/catalog_activation/storage_models.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_docking_predock_pose_resolver PRIVATE cxx_std_17)
  target_include_directories(test_docking_predock_pose_resolver PRIVATE include)
  ament_target_dependencies(test_docking_predock_pose_resolver rclcpp)

  add_test(
    NAME docking_predock_pose_resolver_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/configuration/docking_predock_pose_resolver_contract.py
  )

  ament_add_gtest(
    test_docking_configuration_module
    test/features/docking/configuration/test_docking_configuration_module.cpp
    src/features/docking/configuration/docking_configuration_module.cpp
  )
  target_compile_features(test_docking_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_docking_configuration_module PRIVATE include)
  target_link_libraries(
    test_docking_configuration_module
    robot_elevator_manager::robot_elevator_core
  )
  ament_target_dependencies(
    test_docking_configuration_module
    action_msgs
    builtin_interfaces
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

  add_test(
    NAME docking_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/configuration/docking_configuration_module_contract.py
  )

  ament_add_gtest(
    test_docking_correction_pause_module
    test/features/docking/lifecycle/test_docking_correction_pause_module.cpp
    src/features/docking/lifecycle/docking_correction_pause_module.cpp
    src/features/docking/lifecycle/docking_job_store.cpp
    src/features/docking/lifecycle/docking_job_model.cpp
    src/features/docking/lifecycle/docking_status_utils.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_docking_correction_pause_module PRIVATE cxx_std_17)
  target_include_directories(test_docking_correction_pause_module PRIVATE include)
  ament_target_dependencies(test_docking_correction_pause_module rclcpp)

  add_test(
    NAME docking_correction_pause_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/lifecycle/docking_correction_pause_module_contract.py
  )

  ament_add_gtest(
    test_dock_contact_interlock_module
    test/features/docking/lifecycle/test_dock_contact_interlock_module.cpp
    src/features/docking/lifecycle/dock_contact_interlock_module.cpp
    src/features/docking/lifecycle/docking_status_utils.cpp
    src/features/maps/catalog_activation/file_utils.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_dock_contact_interlock_module PRIVATE cxx_std_17)
  target_include_directories(test_dock_contact_interlock_module PRIVATE include)
  ament_target_dependencies(test_dock_contact_interlock_module sensor_msgs)

  add_test(
    NAME dock_contact_interlock_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/lifecycle/dock_contact_interlock_module_contract.py
  )

  ament_add_gtest(
    test_pre_navigation_undock_module
    test/features/docking/lifecycle/test_pre_navigation_undock_module.cpp
    src/features/docking/lifecycle/pre_navigation_undock_module.cpp
    src/features/docking/lifecycle/docking_job_store.cpp
    src/features/docking/lifecycle/docking_job_model.cpp
    src/features/docking/lifecycle/docking_status_utils.cpp
    src/application/runtime_mode/runtime_mode_coordinator.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_pre_navigation_undock_module PRIVATE cxx_std_17)
  target_include_directories(test_pre_navigation_undock_module PRIVATE include)

  add_test(
    NAME pre_navigation_undock_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/lifecycle/pre_navigation_undock_module_contract.py
  )

  ament_add_gtest(
    test_docking_job_store
    test/features/docking/lifecycle/test_docking_job_store.cpp
    src/features/docking/lifecycle/docking_job_store.cpp
    src/features/docking/lifecycle/docking_job_model.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_include_directories(test_docking_job_store PRIVATE include)

  ament_add_gtest(
    test_docking_job_execution_module
    test/features/docking/lifecycle/test_docking_job_execution_module.cpp
    src/features/docking/lifecycle/docking_job_execution_module.cpp
    src/features/docking/lifecycle/docking_job_store.cpp
    src/features/docking/lifecycle/docking_job_model.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_docking_job_execution_module PRIVATE cxx_std_17)
  target_include_directories(test_docking_job_execution_module PRIVATE include)
  ament_target_dependencies(
    test_docking_job_execution_module
    action_msgs
    builtin_interfaces
    geometry_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
    sensor_msgs
  )

  add_test(
    NAME docking_job_execution_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/lifecycle/docking_job_execution_module_contract.py
  )

  ament_add_gtest(
    test_docking_runtime_module
    test/features/docking/lifecycle/test_docking_runtime_module.cpp
    src/features/docking/lifecycle/docking_runtime_module.cpp
    src/features/docking/lifecycle/docking_job_model.cpp
    src/features/docking/lifecycle/docking_status_utils.cpp
    src/infrastructure/http/http_common.cpp
    src/infrastructure/process/runtime_process_utils.cpp
  )
  target_compile_features(test_docking_runtime_module PRIVATE cxx_std_17)
  target_include_directories(test_docking_runtime_module PRIVATE include)
  ament_target_dependencies(
    test_docking_runtime_module
    geometry_msgs
    rclcpp
    robot_interfaces
    sensor_msgs
    std_msgs
    std_srvs
  )

  add_test(
    NAME docking_runtime_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/lifecycle/docking_runtime_module_contract.py
  )

  ament_add_gtest(
    test_deferred_work_queue
    test/infrastructure/process/test_deferred_work_queue.cpp
    src/infrastructure/process/deferred_work_queue.cpp
  )
  target_include_directories(test_deferred_work_queue PRIVATE include)

  add_test(
    NAME docking_pause_release_executor_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/lifecycle/docking_pause_release_executor_contract.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/features/docking/lifecycle/docking_status_module.cpp
  )

  add_test(
    NAME elevator_execution_lease_removed_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/elevator/execution/elevator_execution_lease_removed_contract.py
      --runtime-source
      ${CMAKE_CURRENT_SOURCE_DIR}/src/features/elevator/execution/elevator_ros_runtime_port.cpp
      --fsm-source
      ${CMAKE_CURRENT_SOURCE_DIR}/../robot_elevator_manager/src/elevator_fsm.cpp
  )

  add_test(
    NAME elevator_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/elevator/elevator_module_contract.py
      --node-source ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_api_server_node.cpp
      --module-header
      ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_api_server/features/elevator/elevator_module.hpp
      --module-source
      ${CMAKE_CURRENT_SOURCE_DIR}/src/features/elevator/elevator_module.cpp
  )

  add_test(
    NAME navigation_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/navigation/navigation_module_contract.py
  )

  add_test(
    NAME navigation_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/navigation/navigation_feature_module_contract.py
  )

  add_test(
    NAME docking_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/docking/docking_feature_module_contract.py
  )

  add_test(
    NAME localization_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/localization/localization_feature_module_contract.py
  )

  add_test(
    NAME maps_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/maps/maps_feature_module_contract.py
  )

  add_test(
    NAME floor_switch_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/floor_switch/floor_switch_feature_module_contract.py
  )

  add_test(
    NAME mapping_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/mapping/mapping_feature_module_contract.py
  )

  add_test(
    NAME safety_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/safety/safety_feature_module_contract.py
  )

  add_test(
    NAME power_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/power/power_feature_module_contract.py
  )

  add_test(
    NAME subscription_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/application/subscriptions/subscription_feature_module_contract.py
  )

  add_test(
    NAME teleop_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/teleop/teleop_feature_module_contract.py
  )

  add_test(
    NAME elevator_feature_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/elevator/elevator_feature_module_contract.py
  )

  ament_add_gtest(
    test_floor_runtime_interlock
    test/features/floor_switch/test_floor_runtime_interlock.cpp
    src/features/floor_switch/floor_runtime_interlock.cpp
  )
  target_include_directories(test_floor_runtime_interlock PRIVATE include)

  ament_add_gtest(
    test_floor_switch_configuration_module
    test/features/floor_switch/test_floor_switch_configuration_module.cpp
    src/features/floor_switch/floor_switch_configuration_module.cpp
  )
  target_compile_features(test_floor_switch_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_floor_switch_configuration_module PRIVATE include)
  ament_target_dependencies(
    test_floor_switch_configuration_module
    rclcpp
    robot_elevator_manager
  )

  add_test(
    NAME floor_switch_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/floor_switch/floor_switch_configuration_module_contract.py
  )

  ament_add_gtest(
    test_floor_switch_handoff_tracker
    test/features/floor_switch/test_floor_switch_handoff_tracker.cpp
    src/features/floor_switch/floor_switch_handoff_tracker.cpp
  )
  target_include_directories(test_floor_switch_handoff_tracker PRIVATE include)

  ament_add_gtest(
    test_floor_switch_http_transaction
    test/features/floor_switch/test_floor_switch_http_transaction.cpp
    src/features/floor_switch/floor_switch_http_transaction.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_include_directories(test_floor_switch_http_transaction PRIVATE include)

  ament_add_gtest(
    test_navigation_cancel_policy
    test/features/navigation/runtime/test_navigation_cancel_policy.cpp
  )
  target_include_directories(test_navigation_cancel_policy PRIVATE include)

  ament_add_gtest(
    test_navigation_state
    test/features/navigation/test_navigation_state.cpp
    src/application/runtime_mode/runtime_mode_coordinator.cpp
    src/features/navigation/navigation_state.cpp
    src/features/navigation/runtime/navigation_bridge_wait.cpp
    src/features/safety/safety_state.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_state PRIVATE cxx_std_17)
  target_include_directories(test_navigation_state PRIVATE include)
  target_link_libraries(test_navigation_state yaml-cpp)
  ament_target_dependencies(test_navigation_state rclcpp)

  ament_add_gtest(
    test_navigation_cancel_runtime
    test/features/navigation/runtime/test_navigation_cancel_runtime.cpp
    src/features/navigation/runtime/navigation_cancel_runtime.cpp
    src/features/navigation/runtime/navigation_cancel_job_model.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_cancel_runtime PRIVATE cxx_std_17)
  target_include_directories(test_navigation_cancel_runtime PRIVATE include)

  ament_add_gtest(
    test_navigation_process_runtime
    test/features/navigation/runtime/test_navigation_process_runtime.cpp
    src/features/navigation/runtime/navigation_process_runtime.cpp
    src/infrastructure/process/runtime_process_utils.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_process_runtime PRIVATE cxx_std_17)
  target_include_directories(test_navigation_process_runtime PRIVATE include)

  ament_add_gtest(test_mapping_stop_concurrency
    test/features/mapping/runtime/test_mapping_stop_concurrency.cpp
    src/features/mapping/runtime/mapping_process_runtime.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_mapping_stop_concurrency PRIVATE cxx_std_17)
  target_include_directories(test_mapping_stop_concurrency PRIVATE include)

  ament_add_gtest(
    test_mapping_runtime
    test/features/mapping/runtime/test_mapping_runtime.cpp
    src/features/mapping/runtime/mapping_process_runtime.cpp
    src/features/mapping/runtime/mapping_start_job.cpp
    src/infrastructure/http/http_common.cpp
    src/infrastructure/process/runtime_process_utils.cpp
  )
  target_compile_features(test_mapping_runtime PRIVATE cxx_std_17)
  target_include_directories(test_mapping_runtime PRIVATE include)

  ament_add_gtest(
    test_mapping_module
    test/features/mapping/test_mapping_module.cpp
    src/application/runtime_mode/runtime_mode_coordinator.cpp
    src/features/mapping/map_asset_writer.cpp
    src/features/mapping/mapping_module.cpp
    src/features/mapping/runtime/mapping_process_runtime.cpp
    src/features/mapping/runtime/mapping_start_job.cpp
    src/features/maps/catalog_activation/api_time_utils.cpp
    src/features/maps/catalog_activation/map_asset_filesystem.cpp
    src/features/maps/catalog_activation/map_catalog.cpp
    src/features/maps/catalog_activation/runtime_map_lookup.cpp
    src/infrastructure/process/runtime_process_utils.cpp
  )
  ament_add_gtest(
    test_mapping_save_job
    test/features/mapping/test_mapping_save_job.cpp
    src/features/maps/catalog_activation/map_asset_filesystem.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_mapping_save_job PRIVATE cxx_std_17)
  target_include_directories(test_mapping_save_job PRIVATE include)
  ament_add_gtest(
    test_mapping_save_routing
    test/features/mapping/test_mapping_save_routing.cpp
    src/application/routing/application_router_module.cpp
    src/application/runtime_mode/runtime_mode_coordinator.cpp
    src/features/mapping/map_asset_writer.cpp
    src/features/mapping/mapping_module.cpp
    src/features/mapping/runtime/mapping_process_runtime.cpp
    src/features/mapping/runtime/mapping_start_job.cpp
    src/features/maps/catalog_activation/api_time_utils.cpp
    src/features/maps/catalog_activation/map_asset_filesystem.cpp
    src/features/maps/catalog_activation/map_catalog.cpp
    src/features/maps/catalog_activation/runtime_map_lookup.cpp
  )
  target_compile_features(test_mapping_save_routing PRIVATE cxx_std_17)
  target_include_directories(test_mapping_save_routing PRIVATE include)
  target_link_libraries(test_mapping_save_routing robot_api_server_keepout yaml-cpp)
  ament_target_dependencies(test_mapping_save_routing nav_msgs rclcpp
    robot_elevator_manager robot_map_asset_identity std_srvs)
  target_compile_features(test_mapping_module PRIVATE cxx_std_17)
  target_include_directories(test_mapping_module PRIVATE include)
  target_link_libraries(test_mapping_module robot_api_server_keepout yaml-cpp)
  ament_target_dependencies(
    test_mapping_module
    nav_msgs
    rclcpp
    robot_elevator_manager
    robot_map_asset_identity
    std_srvs
  )

  ament_add_gtest(
    test_mapping_configuration_module
    test/features/mapping/test_mapping_configuration_module.cpp
    src/features/mapping/mapping_configuration_module.cpp
  )
  target_compile_features(test_mapping_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_mapping_configuration_module PRIVATE include)
  ament_target_dependencies(
    test_mapping_configuration_module
    rclcpp
    robot_elevator_manager
  )

  add_test(
    NAME mapping_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/mapping/mapping_configuration_module_contract.py
  )

  ament_add_gtest(
    test_robot_pose_model
    test/features/system_status/test_robot_pose_model.cpp
    src/features/system_status/robot_pose_model.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_include_directories(test_robot_pose_model PRIVATE include)

  ament_add_gtest(
    test_system_status_module
    test/features/system_status/test_system_status_module.cpp
    src/application/runtime_mode/runtime_mode_coordinator.cpp
    src/features/safety/safety_state.cpp
    src/features/system_status/robot_pose_model.cpp
    src/features/system_status/system_status_module.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_system_status_module PRIVATE cxx_std_17)
  target_include_directories(test_system_status_module PRIVATE include)
  target_link_libraries(test_system_status_module yaml-cpp)
  ament_target_dependencies(
    test_system_status_module
    rclcpp
    sensor_msgs
    std_msgs
  )

  add_test(
    NAME system_status_wiring_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/system_status/system_status_wiring_contract.py
  )

  ament_add_gtest(
    test_http_server
    test/infrastructure/http/test_http_server.cpp
    src/infrastructure/http/http_server.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_include_directories(test_http_server PRIVATE include)

  ament_add_gtest(
    test_api_gateway_configuration_module
    test/infrastructure/http/test_api_gateway_configuration_module.cpp
    src/infrastructure/http/api_gateway_configuration_module.cpp
  )
  target_compile_features(test_api_gateway_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_api_gateway_configuration_module PRIVATE include)
  ament_target_dependencies(test_api_gateway_configuration_module rclcpp)

  add_test(
    NAME api_gateway_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/infrastructure/http/api_gateway_configuration_module_contract.py
  )

  ament_add_gtest(
    test_api_gateway_module
    test/infrastructure/http/test_api_gateway_module.cpp
    src/infrastructure/http/api_gateway_module.cpp
    src/infrastructure/http/http_server.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_api_gateway_module PRIVATE cxx_std_17)
  target_include_directories(test_api_gateway_module PRIVATE include)
  target_link_libraries(test_api_gateway_module yaml-cpp)
  ament_target_dependencies(test_api_gateway_module rclcpp)

  add_test(
    NAME api_gateway_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/infrastructure/http/api_gateway_module_contract.py
  )

  ament_add_gtest(
    test_websocket_transport
    test/infrastructure/http/test_websocket_transport.cpp
    src/infrastructure/http/websocket_transport.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_websocket_transport PRIVATE cxx_std_17)
  target_include_directories(test_websocket_transport PRIVATE include)

  ament_add_gtest(
    test_map_asset_filesystem
    test/features/maps/catalog_activation/test_map_asset_filesystem.cpp
    src/features/maps/catalog_activation/map_asset_filesystem.cpp
  )
  target_compile_features(test_map_asset_filesystem PRIVATE cxx_std_17)
  target_include_directories(test_map_asset_filesystem PRIVATE include)

  ament_add_gtest(
    test_amcl_runtime_status
    test/features/localization/test_amcl_runtime_status.cpp
    src/features/localization/amcl_runtime_status.cpp
  )
  target_compile_features(test_amcl_runtime_status PRIVATE cxx_std_17)
  target_include_directories(test_amcl_runtime_status PRIVATE include)

  ament_add_gtest(
    test_navigation_terminal_control
    test/features/navigation/terminal_control/test_navigation_terminal_control.cpp
    src/features/navigation/terminal_control/navigation_terminal_control.cpp
  )
  target_compile_features(test_navigation_terminal_control PRIVATE cxx_std_17)
  target_include_directories(test_navigation_terminal_control PRIVATE include)
  ament_target_dependencies(
    test_navigation_terminal_control
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(
    test_navigation_terminal_runtime_module
    test/features/navigation/terminal_control/test_navigation_terminal_runtime_module.cpp
    src/features/navigation/terminal_control/navigation_terminal_runtime_module.cpp
    src/features/navigation/terminal_control/navigation_terminal_control.cpp
    src/features/localization/tf_pose_utils.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_terminal_runtime_module PRIVATE cxx_std_17)
  target_include_directories(test_navigation_terminal_runtime_module PRIVATE include)
  ament_target_dependencies(
    test_navigation_terminal_runtime_module
    builtin_interfaces
    geometry_msgs
    nav2_msgs
    nav_msgs
    rcl_interfaces
    rclcpp
    std_msgs
  )

  add_test(
    NAME navigation_terminal_runtime_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/navigation/terminal_control/navigation_terminal_runtime_module_contract.py
  )

  ament_add_gtest(
    test_navigation_configuration_module
    test/features/navigation/configuration/test_navigation_configuration_module.cpp
    src/features/navigation/configuration/navigation_configuration_module.cpp
    src/features/navigation/mission/navigation_goal_policy.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_navigation_configuration_module PRIVATE include)
  target_link_libraries(
    test_navigation_configuration_module
    robot_elevator_manager::robot_elevator_core
  )
  ament_target_dependencies(
    test_navigation_configuration_module
    action_msgs
    builtin_interfaces
    geometry_msgs
    lifecycle_msgs
    nav2_msgs
    nav_msgs
    rcl_interfaces
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    std_srvs
    tf2_msgs
  )

  add_test(
    NAME navigation_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/navigation/configuration/navigation_configuration_module_contract.py
  )

  add_test(
    NAME navigation_goal_execution_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/navigation/mission/navigation_goal_execution_module_contract.py
  )

  ament_add_gtest(
    test_navigation_goal_executor
    test/features/navigation/mission/test_navigation_goal_executor.cpp
    src/features/navigation/mission/navigation_goal_executor.cpp
    src/features/navigation/mission/navigation_completion_policy.cpp
    src/features/navigation/mission/navigation_goal_job.cpp
    src/features/navigation/mission/navigation_mission_runtime.cpp
    src/features/navigation/runtime/navigation_action_runtime.cpp
    src/features/navigation/terminal_control/navigation_terminal_control.cpp
    src/features/localization/tf_pose_utils.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_goal_executor PRIVATE cxx_std_17)
  target_link_libraries(test_navigation_goal_executor yaml-cpp)
  target_include_directories(test_navigation_goal_executor PRIVATE include ${robot_safety_INCLUDE_DIRS})
  ament_target_dependencies(test_navigation_goal_executor
    action_msgs builtin_interfaces geometry_msgs lifecycle_msgs nav2_msgs nav_msgs
    rclcpp rclcpp_action robot_elevator_manager robot_interfaces robot_map_asset_identity
    rcl_interfaces sensor_msgs std_msgs std_srvs tf2_msgs)

  ament_add_gtest(
    test_navigation_goal_execution_module
    test/features/navigation/mission/test_navigation_goal_execution_module.cpp
    src/features/navigation/mission/navigation_goal_execution_module.cpp
    src/features/navigation/mission/navigation_completion_policy.cpp
    src/features/navigation/mission/navigation_goal_job.cpp
    src/features/navigation/mission/navigation_mission_runtime.cpp
    src/features/navigation/runtime/navigation_action_runtime.cpp
    src/features/navigation/runtime/navigation_bridge_wait.cpp
    src/features/navigation/terminal_control/navigation_terminal_control.cpp
    src/features/navigation/terminal_control/navigation_terminal_runtime_module.cpp
    src/features/localization/tf_pose_utils.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_goal_execution_module PRIVATE cxx_std_17)
  target_link_libraries(test_navigation_goal_execution_module yaml-cpp)
  target_include_directories(test_navigation_goal_execution_module PRIVATE include)
  ament_add_gtest(test_navigation_recovery_wait
    test/features/navigation/runtime/test_navigation_recovery_wait.cpp)
  target_include_directories(test_navigation_recovery_wait PRIVATE include)
  target_compile_features(test_navigation_recovery_wait PRIVATE cxx_std_17)
  target_link_libraries(test_navigation_recovery_wait yaml-cpp)
  ament_target_dependencies(
    test_navigation_goal_execution_module
    action_msgs
    builtin_interfaces
    geometry_msgs
    nav2_msgs
    nav_msgs
    rcl_interfaces
    rclcpp
    rclcpp_action
    std_msgs
  )

  ament_add_gtest(
    test_navigation_goal_job
    test/features/navigation/mission/test_navigation_goal_job.cpp
    src/features/navigation/mission/navigation_goal_job.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_goal_job PRIVATE cxx_std_17)
  target_include_directories(test_navigation_goal_job PRIVATE include)

  ament_add_gtest(
    test_navigation_mission_runtime
    test/features/navigation/mission/test_navigation_mission_runtime.cpp
    src/features/navigation/mission/navigation_mission_runtime.cpp
    src/features/navigation/mission/navigation_goal_job.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_mission_runtime PRIVATE cxx_std_17)
  target_include_directories(test_navigation_mission_runtime PRIVATE include)

  ament_add_gtest(
    test_navigation_completion_policy
    test/features/navigation/mission/test_navigation_completion_policy.cpp
    src/features/navigation/mission/navigation_completion_policy.cpp
  )
  target_compile_features(test_navigation_completion_policy PRIVATE cxx_std_17)
  target_include_directories(test_navigation_completion_policy PRIVATE include)

  ament_add_gtest(
    test_navigation_bridge_wait
    test/features/navigation/runtime/test_navigation_bridge_wait.cpp
    src/features/navigation/runtime/navigation_bridge_wait.cpp
  )
  target_compile_features(test_navigation_bridge_wait PRIVATE cxx_std_17)
  target_include_directories(test_navigation_bridge_wait PRIVATE include)

  ament_add_gtest(
    test_navigation_goal_policy
    test/features/navigation/mission/test_navigation_goal_policy.cpp
    src/features/navigation/mission/navigation_goal_policy.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_goal_policy PRIVATE cxx_std_17)
  target_include_directories(test_navigation_goal_policy PRIVATE include)

  ament_add_gtest(
    test_navigation_goal_http
    test/features/navigation/mission/test_navigation_goal_http.cpp
    src/features/navigation/mission/navigation_goal_http.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_navigation_goal_http PRIVATE cxx_std_17)
  target_include_directories(test_navigation_goal_http PRIVATE include)

  ament_add_gtest(
    test_keepout_layer
    test/features/maps/keepout/test_keepout_layer.cpp
  )
  target_link_libraries(test_keepout_layer robot_api_server_keepout)

  ament_add_gtest(
    test_map_asset_digest
    test/features/maps/catalog_activation/test_map_asset_digest.cpp
  )
  target_link_libraries(test_map_asset_digest robot_api_server_keepout)

  ament_add_gtest(
    test_map_asset_identity_binding
    test/features/maps/catalog_activation/test_map_asset_identity_binding.cpp
  )
  target_link_libraries(
    test_map_asset_identity_binding
    robot_api_server_keepout
  )

  ament_add_gtest(
    test_map_manifest_io
    test/features/maps/catalog_activation/test_map_manifest_io.cpp
  )
  target_link_libraries(test_map_manifest_io robot_api_server_keepout)

  ament_add_gtest(
    test_map_catalog
    test/features/maps/catalog_activation/test_map_catalog.cpp
    src/features/maps/catalog_activation/api_time_utils.cpp
    src/features/maps/catalog_activation/map_catalog.cpp
  )
  target_include_directories(test_map_catalog PRIVATE include)
  target_link_libraries(test_map_catalog robot_api_server_keepout)

  ament_add_gtest(
    test_map_runtime_state_store
    test/features/maps/catalog_activation/test_map_runtime_state_store.cpp
    src/features/maps/catalog_activation/api_time_utils.cpp
    src/features/maps/catalog_activation/map_runtime_state_store.cpp
    src/features/floor_switch/runtime_map_context_io.cpp
  )
  target_compile_features(test_map_runtime_state_store PRIVATE cxx_std_17)
  target_include_directories(test_map_runtime_state_store PRIVATE include)
  target_link_libraries(test_map_runtime_state_store robot_api_server_keepout)

  add_test(
    NAME map_runtime_state_store_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/maps/catalog_activation/map_runtime_state_store_contract.py
  )

  ament_add_gtest(
    test_maps_configuration_module
    test/features/maps/test_maps_configuration_module.cpp
    src/features/maps/maps_configuration_module.cpp
  )
  target_compile_features(test_maps_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_maps_configuration_module PRIVATE include)
  target_link_libraries(
    test_maps_configuration_module
    robot_elevator_manager::robot_elevator_core
  )
  ament_target_dependencies(
    test_maps_configuration_module
    lifecycle_msgs
    rclcpp
  )

  add_test(
    NAME maps_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/maps/maps_configuration_module_contract.py
  )

  ament_add_gtest(
    test_elevator_configuration_module
    test/features/elevator/configuration/test_elevator_configuration_module.cpp
  )
  target_link_libraries(
    test_elevator_configuration_module
    robot_api_server_elevator_config
  )

  ament_add_gtest(
    test_elevator_runtime_configuration_module
    test/features/elevator/configuration/test_elevator_runtime_configuration_module.cpp
    src/features/elevator/configuration/elevator_runtime_configuration_module.cpp
  )
  target_compile_features(test_elevator_runtime_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_elevator_runtime_configuration_module PRIVATE include)
  target_link_libraries(
    test_elevator_runtime_configuration_module
    robot_elevator_manager::robot_elevator_core
  )
  ament_target_dependencies(
    test_elevator_runtime_configuration_module
    action_msgs
    builtin_interfaces
    geometry_msgs
    lifecycle_msgs
    nav2_msgs
    nav_msgs
    rcl_interfaces
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    std_srvs
    tf2_msgs
  )

  add_test(
    NAME elevator_runtime_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/elevator/configuration/elevator_runtime_configuration_module_contract.py
  )

  ament_add_gtest(
    test_elevator_test_module
    test/features/elevator/execution/test_elevator_test_module.cpp
  )
  target_link_libraries(
    test_elevator_test_module
    robot_api_server_elevator_test
  )

  ament_add_gtest(
    test_elevator_arm_client
    test/features/elevator/execution/test_elevator_arm_client.cpp
    src/features/elevator/execution/elevator_arm_client.cpp
  )
  target_include_directories(test_elevator_arm_client PRIVATE include)
  target_link_libraries(test_elevator_arm_client yaml-cpp)

  ament_add_gtest(
    test_elevator_recovery_action_barrier
    test/features/elevator/execution/test_elevator_recovery_action_barrier.cpp
    src/features/elevator/execution/elevator_recovery_action_barrier.cpp
  )
  target_include_directories(
    test_elevator_recovery_action_barrier PRIVATE include
  )

  ament_add_gtest(
    test_elevator_runtime_policy
    test/features/elevator/execution/test_elevator_runtime_policy.cpp
    src/features/elevator/execution/elevator_runtime_policy.cpp
  )
  target_include_directories(test_elevator_runtime_policy PRIVATE include)
  target_link_libraries(
    test_elevator_runtime_policy
    robot_elevator_manager::robot_elevator_core
  )

  ament_add_gtest(
    test_elevator_lease_keepalive
    test/features/elevator/execution/test_elevator_lease_keepalive.cpp
  )
  target_link_libraries(
    test_elevator_lease_keepalive
    robot_api_server_elevator_lease_keepalive
    robot_safety::motion_interlock_arbiter
  )

  ament_add_gtest(test_elevator_ros_executor
    test/features/elevator/execution/test_elevator_ros_executor.cpp)
  target_include_directories(test_elevator_ros_executor PRIVATE include)
  ament_target_dependencies(test_elevator_ros_executor rclcpp)
  ament_add_gtest(test_elevator_ros_events
    test/features/elevator/execution/test_elevator_ros_events.cpp)
  target_include_directories(test_elevator_ros_events PRIVATE include)
  ament_target_dependencies(test_elevator_ros_events
    rclcpp rclcpp_action nav2_msgs std_srvs robot_interfaces)

  add_test(
    NAME keepout_http_smoke
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/maps/keepout/keepout_http_smoke.py
      --node $<TARGET_FILE:robot_api_server_node>
      --port 18081
  )
  set_tests_properties(keepout_http_smoke PROPERTIES RUN_SERIAL TRUE TIMEOUT 45)

  add_test(
    NAME floor_runtime_http_smoke
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/floor_switch/floor_runtime_http_smoke.py
      --node $<TARGET_FILE:robot_api_server_node>
      --port 18082
  )
  set_tests_properties(floor_runtime_http_smoke PROPERTIES RUN_SERIAL TRUE TIMEOUT 45)

  add_test(
    NAME floor_switch_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/floor_switch/floor_switch_module_contract.py
      --node-source ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_api_server_node.cpp
      --module-header
      ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_api_server/features/floor_switch/floor_switch_module.hpp
      --module-source
      ${CMAKE_CURRENT_SOURCE_DIR}/src/features/floor_switch/floor_switch_module.cpp
  )

  add_test(
    NAME maps_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/maps/maps_module_contract.py
      --node-source ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_api_server_node.cpp
      --module-header
      ${CMAKE_CURRENT_SOURCE_DIR}/include/robot_api_server/features/maps/maps_module.hpp
      --module-source
      ${CMAKE_CURRENT_SOURCE_DIR}/src/features/maps/maps_module.cpp
  )

  add_test(
    NAME localization_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/localization/localization_module_contract.py
  )

  ament_add_gtest(
    test_localization_configuration_module
    test/features/localization/test_localization_configuration_module.cpp
    src/features/localization/localization_configuration_module.cpp
    src/features/localization/tf_pose_utils.cpp
  )
  ament_add_gtest(test_trigger_evidence
    test/features/localization/test_trigger_evidence.cpp)
  target_compile_features(test_trigger_evidence PRIVATE cxx_std_17)
  target_include_directories(test_trigger_evidence PRIVATE include)
  ament_target_dependencies(test_trigger_evidence robot_interfaces)
  ament_add_gtest(test_localization_trigger_transport
    test/features/localization/test_localization_trigger_transport.cpp
    src/features/localization/localization_module.cpp
    src/features/localization/localization_result_model.cpp
    src/features/localization/amcl_runtime_status.cpp
    src/features/localization/tf_pose_utils.cpp
    src/infrastructure/http/http_common.cpp)
  target_compile_features(test_localization_trigger_transport PRIVATE cxx_std_17)
  target_include_directories(test_localization_trigger_transport PRIVATE include)
  ament_target_dependencies(test_localization_trigger_transport
    rclcpp robot_interfaces geometry_msgs std_msgs std_srvs tf2_msgs)
  set_tests_properties(test_localization_trigger_transport PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=217;ROS_LOCALHOST_ONLY=1;FASTRTPS_DEFAULT_PROFILES_FILE=;FASTDDS_DEFAULT_PROFILES_FILE="
    RUN_SERIAL TRUE TIMEOUT 30)
  target_compile_features(test_localization_configuration_module PRIVATE cxx_std_17)
  target_include_directories(test_localization_configuration_module PRIVATE include)
  ament_target_dependencies(
    test_localization_configuration_module
    builtin_interfaces
    rclcpp
  )

  add_test(
    NAME localization_configuration_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/localization/localization_configuration_module_contract.py
  )

  ament_add_gtest(
    test_post_relocalization_settle_module
    test/features/localization/test_post_relocalization_settle_module.cpp
    src/features/localization/post_relocalization_settle_module.cpp
    src/infrastructure/http/http_common.cpp
  )
  target_compile_features(test_post_relocalization_settle_module PRIVATE cxx_std_17)
  target_include_directories(test_post_relocalization_settle_module PRIVATE include)
  ament_target_dependencies(test_post_relocalization_settle_module rclcpp)

  add_test(
    NAME post_relocalization_settle_module_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/localization/post_relocalization_settle_module_contract.py
  )

  add_test(
    NAME navigation_action_runtime_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/navigation/navigation_action_runtime_contract.py
  )

  add_test(
    NAME navigation_cancel_http_smoke
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/navigation/runtime/navigation_cancel_http_smoke.py
      --node $<TARGET_FILE:robot_api_server_node>
      --port 18084
  )
  set_tests_properties(
    navigation_cancel_http_smoke
    PROPERTIES RUN_SERIAL TRUE TIMEOUT 30
  )

  add_test(
    NAME elevator_configuration_http_smoke
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/features/elevator/configuration/elevator_configuration_http_smoke.py
      --node $<TARGET_FILE:robot_api_server_node>
      --port 18083
  )
  set_tests_properties(
    elevator_configuration_http_smoke
    PROPERTIES RUN_SERIAL TRUE TIMEOUT 60
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_robot_api_server_keepout HAS_LIBRARY_TARGET)
ament_export_dependencies(
  robot_elevator_manager
  robot_map_asset_identity
  robot_safety
  yaml-cpp
)

ament_package()

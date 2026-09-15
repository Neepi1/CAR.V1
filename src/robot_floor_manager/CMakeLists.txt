cmake_minimum_required(VERSION 3.8)
project(robot_floor_manager)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_map_asset_identity REQUIRED)
find_package(robot_safety REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(executor_runtime_policy
  src/executor_runtime_policy.cpp
)
target_compile_features(executor_runtime_policy PUBLIC cxx_std_17)
target_include_directories(executor_runtime_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(navigate_action_graph
  src/navigate_action_graph.cpp
)
target_compile_features(navigate_action_graph PUBLIC cxx_std_17)
target_include_directories(navigate_action_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(floor_asset_snapshot_loader
  src/floor_asset_snapshot_loader.cpp
)
target_compile_features(floor_asset_snapshot_loader PUBLIC cxx_std_17)
target_include_directories(floor_asset_snapshot_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  floor_asset_snapshot_loader
  robot_map_asset_identity
)

add_library(floor_transition_core
  src/floor_switch_preflight.cpp
  src/floor_transition_core.cpp
  src/floor_transition_reconciliation.cpp
)
target_compile_features(floor_transition_core PUBLIC cxx_std_17)
target_include_directories(floor_transition_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(floor_transition_executor
  src/floor_transition_executor.cpp
)
target_compile_features(floor_transition_executor PUBLIC cxx_std_17)
target_include_directories(floor_transition_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  floor_transition_executor
  floor_asset_snapshot_loader
  floor_transition_core
)

add_library(floor_transition_evidence_tracker
  src/floor_transition_evidence_tracker.cpp
)
target_compile_features(floor_transition_evidence_tracker PUBLIC cxx_std_17)
target_include_directories(floor_transition_evidence_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  floor_transition_evidence_tracker
  floor_transition_core
)

add_library(runtime_map_context_writer
  src/runtime_map_context_writer.cpp
)
target_compile_features(runtime_map_context_writer PUBLIC cxx_std_17)
target_link_libraries(runtime_map_context_writer yaml-cpp)
target_include_directories(runtime_map_context_writer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(floor_manager_node src/floor_manager_node.cpp)
target_compile_features(floor_manager_node PUBLIC cxx_std_17)
target_link_libraries(
  floor_manager_node
  executor_runtime_policy
  navigate_action_graph
  floor_asset_snapshot_loader
  floor_transition_core
  floor_transition_executor
  floor_transition_evidence_tracker
  runtime_map_context_writer
  robot_safety::persistent_sequence_allocator
)
ament_target_dependencies(
  floor_manager_node
  action_msgs
  lifecycle_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  std_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS
    executor_runtime_policy
    navigate_action_graph
    floor_asset_snapshot_loader
    floor_transition_core
    floor_transition_executor
    floor_transition_evidence_tracker
    runtime_map_context_writer
    floor_manager_node
  EXPORT export_floor_transition_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_floor_transition_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(robot_map_asset_identity yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(robot_localization_bridge REQUIRED)
  ament_add_gtest(test_floor_transition_cleanup test/test_floor_transition_cleanup.cpp)
  target_link_libraries(test_floor_transition_cleanup
    floor_transition_core
    robot_localization_bridge::floor_transition_context
  )
  ament_add_pytest_test(test_floor_manager_cleanup_contract
    test/test_floor_manager_cleanup_contract.py
  )
  ament_add_gtest(
    test_executor_runtime_policy
    test/test_executor_runtime_policy.cpp
  )
  target_link_libraries(
    test_executor_runtime_policy
    executor_runtime_policy
  )

  ament_add_gtest(
    test_navigate_action_graph
    test/test_navigate_action_graph.cpp
  )
  target_link_libraries(
    test_navigate_action_graph
    navigate_action_graph
  )

  ament_add_gtest(
    test_floor_asset_snapshot_loader
    test/test_floor_asset_snapshot_loader.cpp
  )
  target_link_libraries(
    test_floor_asset_snapshot_loader
    floor_asset_snapshot_loader
  )

  ament_add_gtest(
    test_floor_transition_core
    test/test_floor_transition_core.cpp
  )
  target_link_libraries(test_floor_transition_core floor_transition_core)

  ament_add_gtest(
    test_floor_transition_executor
    test/test_floor_transition_executor.cpp
  )
  target_link_libraries(
    test_floor_transition_executor
    floor_transition_executor
  )

  ament_add_gtest(
    test_floor_transition_evidence_tracker
    test/test_floor_transition_evidence_tracker.cpp
  )
  target_link_libraries(
    test_floor_transition_evidence_tracker
    floor_transition_evidence_tracker
  )

  ament_add_gtest(
    test_floor_transition_reconciliation
    test/test_floor_transition_reconciliation.cpp
  )
  target_link_libraries(
    test_floor_transition_reconciliation
    floor_transition_core
  )

  ament_add_gtest(
    test_runtime_map_context_writer
    test/test_runtime_map_context_writer.cpp
  )
  target_link_libraries(
    test_runtime_map_context_writer
    runtime_map_context_writer
  )

  ament_add_gtest(
    test_floor_manager_legacy_service
    test/test_floor_manager_legacy_service.cpp
  )
  target_compile_definitions(
    test_floor_manager_legacy_service
    PRIVATE ROBOT_FLOOR_MANAGER_DISABLE_MAIN
  )
  ament_target_dependencies(
    test_floor_manager_legacy_service
    action_msgs
    lifecycle_msgs
    nav_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
    robot_interfaces
    std_msgs
  )
  target_link_libraries(
    test_floor_manager_legacy_service
    navigate_action_graph
    floor_asset_snapshot_loader
    floor_transition_core
    floor_transition_executor
    floor_transition_evidence_tracker
    runtime_map_context_writer
    robot_safety::persistent_sequence_allocator
  )

  ament_add_gtest(
    test_floor_manager_exact_selection
    test/test_floor_manager_exact_selection.cpp
  )
  target_compile_definitions(
    test_floor_manager_exact_selection
    PRIVATE ROBOT_FLOOR_MANAGER_DISABLE_MAIN
  )
  ament_target_dependencies(
    test_floor_manager_exact_selection
    action_msgs
    lifecycle_msgs
    nav_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
    robot_interfaces
    std_msgs
  )
  target_link_libraries(
    test_floor_manager_exact_selection
    navigate_action_graph
    floor_asset_snapshot_loader
    floor_transition_core
    floor_transition_executor
    floor_transition_evidence_tracker
    runtime_map_context_writer
    robot_safety::persistent_sequence_allocator
  )

  ament_add_gtest(
    test_floor_switch_preflight
    test/test_floor_switch_preflight.cpp
  )
  target_link_libraries(test_floor_switch_preflight floor_transition_core)

  ament_add_gtest(test_floor_manager_startup_action
    test/test_floor_manager_startup_action.cpp
    TIMEOUT 90
  )
  target_compile_definitions(test_floor_manager_startup_action
    PRIVATE ROBOT_FLOOR_MANAGER_DISABLE_MAIN
  )
  ament_target_dependencies(test_floor_manager_startup_action
    action_msgs lifecycle_msgs nav_msgs nav2_msgs rclcpp rclcpp_action
    robot_interfaces robot_map_asset_identity std_msgs
  )
  target_link_libraries(test_floor_manager_startup_action
    navigate_action_graph floor_asset_snapshot_loader floor_transition_core
    floor_transition_executor floor_transition_evidence_tracker
    runtime_map_context_writer robot_safety::persistent_sequence_allocator
  )
  set_tests_properties(test_floor_manager_startup_action PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=213;ROS_LOCALHOST_ONLY=1"
    RUN_SERIAL TRUE
  )
endif()

ament_package()

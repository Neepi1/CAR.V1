cmake_minimum_required(VERSION 3.8)
project(robot_floor_manager)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_map_asset_identity REQUIRED)
find_package(std_msgs REQUIRED)

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
)
target_compile_features(floor_transition_core PUBLIC cxx_std_17)
target_include_directories(floor_transition_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(floor_manager_node src/floor_manager_node.cpp)
target_compile_features(floor_manager_node PUBLIC cxx_std_17)
target_link_libraries(floor_manager_node floor_transition_core)
ament_target_dependencies(
  floor_manager_node
  lifecycle_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  std_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS floor_asset_snapshot_loader floor_transition_core floor_manager_node
  EXPORT export_floor_transition_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_floor_transition_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(robot_map_asset_identity)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
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
    test_floor_manager_legacy_service
    test/test_floor_manager_legacy_service.cpp
  )
  target_compile_definitions(
    test_floor_manager_legacy_service
    PRIVATE ROBOT_FLOOR_MANAGER_DISABLE_MAIN
  )
  ament_target_dependencies(
    test_floor_manager_legacy_service
    lifecycle_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
    robot_interfaces
    std_msgs
  )
  target_link_libraries(test_floor_manager_legacy_service floor_transition_core)

  ament_add_gtest(
    test_floor_switch_preflight
    test/test_floor_switch_preflight.cpp
  )
  target_link_libraries(test_floor_switch_preflight floor_transition_core)
endif()

ament_package()

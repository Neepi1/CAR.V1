cmake_minimum_required(VERSION 3.8)
project(robot_nav_config)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_rotation_shim_controller REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(goal_scoped_rotation_shim_controller SHARED
  src/goal_scoped_rotation_shim_controller.cpp
)
target_include_directories(goal_scoped_rotation_shim_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(goal_scoped_rotation_shim_controller
  angles
  builtin_interfaces
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_rotation_shim_controller
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
  tf2
)

add_library(ranger_mini3_lattice_planner SHARED
  src/ranger_mini3_lattice_planner.cpp
)
target_include_directories(ranger_mini3_lattice_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ranger_mini3_lattice_planner
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_smac_planner
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  tf2
)

pluginlib_export_plugin_description_file(
  nav2_core robot_nav_config_controller_plugins.xml
)
pluginlib_export_plugin_description_file(
  nav2_core robot_nav_config_planner_plugins.xml
)

install(TARGETS
  goal_scoped_rotation_shim_controller
  ranger_mini3_lattice_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config behavior_trees docs lattice
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tools/generate_ranger_mini3_lattice.py
  tools/validate_ranger_mini3_lattice.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_goal_scope_tracker
    test/test_goal_scope_tracker.cpp
  )
  target_include_directories(test_goal_scope_tracker PRIVATE include)

  ament_add_gtest(test_terminal_rotation_braking
    test/test_terminal_rotation_braking.cpp
  )
  target_include_directories(test_terminal_rotation_braking PRIVATE include)

  ament_add_gtest(test_terminal_pose_handoff
    test/test_terminal_pose_handoff.cpp
  )
  target_include_directories(test_terminal_pose_handoff PRIVATE include)

  ament_add_gtest(test_startup_alignment_guard
    test/test_startup_alignment_guard.cpp
  )
  target_include_directories(test_startup_alignment_guard PRIVATE include)

  ament_add_gtest(test_ranger_direct_corridor
    test/test_ranger_direct_corridor.cpp
  )
  target_include_directories(test_ranger_direct_corridor PRIVATE include)
  ament_target_dependencies(test_ranger_direct_corridor
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_ranger_lattice_nav2_compatibility
    test/test_ranger_lattice_nav2_compatibility.cpp
  )
  target_compile_definitions(test_ranger_lattice_nav2_compatibility PRIVATE
    RANGER_LATTICE_TEST_PATH="${CMAKE_CURRENT_SOURCE_DIR}/lattice/ranger_mini3_ackermann_0p05m_0p81m_16.json"
  )
  ament_target_dependencies(test_ranger_lattice_nav2_compatibility
    nav2_smac_planner
  )

  ament_add_gtest(test_ranger_lattice_terminal_reachability
    test/test_ranger_lattice_terminal_reachability.cpp
  )
  target_compile_definitions(test_ranger_lattice_terminal_reachability PRIVATE
    RANGER_LATTICE_TEST_PATH="${CMAKE_CURRENT_SOURCE_DIR}/lattice/ranger_mini3_ackermann_0p05m_0p81m_16.json"
  )
  ament_target_dependencies(test_ranger_lattice_terminal_reachability
    nav2_costmap_2d
    nav2_smac_planner
    nav2_util
    rclcpp
  )

endif()

ament_export_include_directories(include)
ament_export_libraries(
  goal_scoped_rotation_shim_controller
  ranger_mini3_lattice_planner
)
ament_export_dependencies(
  angles
  builtin_interfaces
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_rotation_shim_controller
  nav2_smac_planner
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
  tf2
)

ament_package()

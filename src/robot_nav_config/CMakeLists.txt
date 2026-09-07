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
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(goal_scoped_rotation_shim_controller SHARED
  src/docking/dock_staging_goal_checker.cpp
  src/elevator_aware_progress_checker.cpp
  src/elevator_scoped_clearance.cpp
  src/elevator_scoped_replan_worker.cpp
  src/elevator_scoped_search.cpp
  src/elevator_scoped_controller.cpp
  src/goal_scoped_rotation_shim_controller.cpp
)
target_link_libraries(goal_scoped_rotation_shim_controller Threads::Threads)
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
  rclcpp_lifecycle
  robot_interfaces
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_library(ranger_mini3_lattice_planner SHARED
  src/elevator_scoped_clearance.cpp
  src/elevator_scoped_search.cpp
  src/elevator_scoped_planner.cpp
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
  tf2_geometry_msgs
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

  ament_add_gtest(test_dock_staging_goal_policy
    test/docking/test_dock_staging_goal_policy.cpp
  )
  target_include_directories(test_dock_staging_goal_policy PRIVATE include)

  ament_add_gtest(test_dock_staging_goal_checker
    test/docking/test_dock_staging_goal_checker.cpp
  )
  target_include_directories(test_dock_staging_goal_checker PRIVATE include)
  target_link_libraries(
    test_dock_staging_goal_checker
    goal_scoped_rotation_shim_controller
  )
  ament_target_dependencies(test_dock_staging_goal_checker
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    rclcpp
    rclcpp_lifecycle
    tf2_geometry_msgs
  )

  ament_add_gtest(test_elevator_localization_replan_gate
    test/test_elevator_localization_replan_gate.cpp
  )
  target_include_directories(test_elevator_localization_replan_gate PRIVATE include)

  ament_add_gtest(test_elevator_speed_limit
    test/test_elevator_speed_limit.cpp
  )
  target_include_directories(test_elevator_speed_limit PRIVATE include)

  ament_add_gtest(test_elevator_scoped_path
    test/test_elevator_scoped_path.cpp
  )
  target_include_directories(test_elevator_scoped_path PRIVATE include)

  ament_add_gtest(test_elevator_scoped_clearance
    test/test_elevator_scoped_clearance.cpp
    src/elevator_scoped_clearance.cpp
  )
  target_include_directories(test_elevator_scoped_clearance PRIVATE include)
  ament_target_dependencies(test_elevator_scoped_clearance
    geometry_msgs
    nav2_costmap_2d
  )

  ament_add_gtest(test_elevator_scoped_search
    test/test_elevator_scoped_search.cpp
    src/elevator_scoped_clearance.cpp
    src/elevator_scoped_search.cpp
  )
  target_include_directories(test_elevator_scoped_search PRIVATE include)
  ament_target_dependencies(test_elevator_scoped_search
    geometry_msgs
    nav2_costmap_2d
  )

  ament_add_gtest(test_elevator_scoped_route
    test/test_elevator_scoped_route.cpp
  )
  target_include_directories(test_elevator_scoped_route PRIVATE include)

  ament_add_gtest(test_elevator_scoped_plan_update
    test/test_elevator_scoped_plan_update.cpp
  )
  target_include_directories(test_elevator_scoped_plan_update PRIVATE include)

  ament_add_gtest(test_elevator_scoped_execution_session
    test/test_elevator_scoped_execution_session.cpp
  )
  target_include_directories(
    test_elevator_scoped_execution_session PRIVATE include
  )

  ament_add_gtest(test_elevator_scoped_replan_worker
    test/test_elevator_scoped_replan_worker.cpp
    src/elevator_scoped_clearance.cpp
    src/elevator_scoped_search.cpp
    src/elevator_scoped_replan_worker.cpp
  )
  target_include_directories(test_elevator_scoped_replan_worker PRIVATE include)
  ament_target_dependencies(test_elevator_scoped_replan_worker
    geometry_msgs
    nav2_costmap_2d
  )
  target_link_libraries(test_elevator_scoped_replan_worker Threads::Threads)

  ament_add_gtest(test_elevator_scoped_local_repair
    test/test_elevator_scoped_local_repair.cpp
    src/elevator_scoped_clearance.cpp
    src/elevator_scoped_local_repair.cpp
    src/elevator_scoped_search.cpp
  )
  target_include_directories(test_elevator_scoped_local_repair PRIVATE include)
  ament_target_dependencies(test_elevator_scoped_local_repair
    geometry_msgs
    nav2_costmap_2d
  )

  ament_add_gtest(test_elevator_scoped_blockage
    test/test_elevator_scoped_blockage.cpp
  )
  target_include_directories(test_elevator_scoped_blockage PRIVATE include)

  ament_add_gtest(test_elevator_scoped_replan_progress
    test/test_elevator_scoped_replan_progress.cpp
  )
  target_include_directories(test_elevator_scoped_replan_progress PRIVATE include)

  ament_add_gtest(test_elevator_aware_progress_policy
    test/test_elevator_aware_progress_policy.cpp
  )
  target_include_directories(test_elevator_aware_progress_policy PRIVATE include)

  ament_add_gtest(test_elevator_aware_progress_checker
    test/test_elevator_aware_progress_checker.cpp
  )
  target_include_directories(test_elevator_aware_progress_checker PRIVATE include)
  target_link_libraries(
    test_elevator_aware_progress_checker
    goal_scoped_rotation_shim_controller
  )
  ament_target_dependencies(test_elevator_aware_progress_checker
    geometry_msgs
    nav2_core
    rclcpp
    rclcpp_lifecycle
    std_msgs
  )

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

  ament_add_gtest(test_ranger_exact_goal_endpoint
    test/test_ranger_exact_goal_endpoint.cpp
  )
  target_include_directories(test_ranger_exact_goal_endpoint PRIVATE include)
  ament_target_dependencies(test_ranger_exact_goal_endpoint
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
  rclcpp_lifecycle
  robot_interfaces
  std_msgs
  tf2
  tf2_geometry_msgs
)

ament_package()

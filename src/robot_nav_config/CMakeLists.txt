cmake_minimum_required(VERSION 3.8)
project(robot_nav_config)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(angles REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_rotation_shim_controller REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ompl REQUIRED)
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

add_library(ranger_dynamics_mppi_controller SHARED src/chassis_dynamics/mppi_controller.cpp)
target_include_directories(ranger_dynamics_mppi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ranger_dynamics_mppi_controller nav2_mppi_controller nav2_costmap_2d
  nav2_core nav2_util pluginlib rclcpp rclcpp_lifecycle nav_msgs geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core robot_nav_config_dynamics_plugins.xml)
install(TARGETS ranger_dynamics_mppi_controller LIBRARY DESTINATION lib COMPONENT chassis_dynamics)

add_library(ranger_clearance_critic SHARED src/navigation_clearance/ranger_clearance_critic.cpp)
target_include_directories(ranger_clearance_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ranger_clearance_critic nav2_mppi_controller nav2_costmap_2d
  pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(nav2_mppi_controller robot_nav_config_mppi_critics.xml)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PrepareOrdinaryNavigationRecovery.srv"
  DEPENDENCIES builtin_interfaces nav_msgs)
rosidl_get_typesupport_target(recovery_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(goal_scoped_rotation_shim_controller SHARED
  src/docking/dock_staging_goal_checker.cpp
  src/elevator_aware_progress_checker.cpp
  src/elevator_scoped_clearance.cpp
  src/elevator_scoped_replan_worker.cpp
  src/elevator_scoped_search.cpp
  src/elevator_scoped_controller.cpp
  src/goal_scoped_rotation_shim_controller.cpp
  src/ordinary_local_path_repair.cpp
  src/ordinary_local_path_repair_runtime.cpp
  src/ordinary_local_path_repair_worker.cpp
  src/navigation_recovery/recovery_state.cpp
)
target_link_libraries(goal_scoped_rotation_shim_controller Threads::Threads)
target_link_libraries(goal_scoped_rotation_shim_controller "${recovery_typesupport}")
target_include_directories(goal_scoped_rotation_shim_controller BEFORE PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")
target_link_libraries(goal_scoped_rotation_shim_controller ${OMPL_LIBRARIES})
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

add_library(ordinary_navigation_recovery_bt_node SHARED
  src/navigation_recovery/bt_nodes.cpp
)
target_compile_definitions(ordinary_navigation_recovery_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(ordinary_navigation_recovery_bt_node PRIVATE include)
target_include_directories(ordinary_navigation_recovery_bt_node BEFORE PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")
ament_target_dependencies(ordinary_navigation_recovery_bt_node
  behaviortree_cpp_v3 nav2_behavior_tree nav2_msgs robot_interfaces rclcpp nav_msgs std_msgs
)
# Reuse Humble's action adapter, including cancellation and same-action path updates.
find_library(follow_path_bt_library NAMES nav2_follow_path_action_bt_node
  HINTS "${nav2_behavior_tree_DIR}/../../../lib" REQUIRED)
target_link_libraries(ordinary_navigation_recovery_bt_node "${follow_path_bt_library}")
target_link_libraries(ordinary_navigation_recovery_bt_node "${recovery_typesupport}")

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
  ranger_clearance_critic
  goal_scoped_rotation_shim_controller
  ARCHIVE DESTINATION lib COMPONENT navigation_clearance
  LIBRARY DESTINATION lib COMPONENT navigation_clearance
  RUNTIME DESTINATION bin COMPONENT navigation_clearance
)

install(TARGETS
  ordinary_navigation_recovery_bt_node
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
  find_package(nav2_controller REQUIRED)
  ament_add_gtest(test_chassis_response test/chassis_dynamics/test_response_model.cpp)
  target_include_directories(test_chassis_response PRIVATE include)
  add_executable(replay_chassis_can test/chassis_dynamics/replay_can.cpp)
  target_include_directories(replay_chassis_can PRIVATE include)
  ament_add_gtest(test_mppi_dynamics test/chassis_dynamics/test_mppi_dynamics.cpp)
  target_include_directories(test_mppi_dynamics PRIVATE include)
  ament_target_dependencies(test_mppi_dynamics nav2_mppi_controller nav2_costmap_2d
    nav2_controller nav2_util nav2_core pluginlib rclcpp rclcpp_lifecycle)
  target_compile_definitions(test_mppi_dynamics PRIVATE NAV_CONFIG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/nav2.yaml")
  set_tests_properties(test_mppi_dynamics PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=212;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  ament_add_gtest(test_ranger_clearance_critic test/navigation_clearance/test_ranger_clearance_critic.cpp)
  ament_target_dependencies(test_ranger_clearance_critic nav2_mppi_controller nav2_costmap_2d
    nav2_controller nav2_util rclcpp rclcpp_lifecycle)
  target_compile_definitions(test_ranger_clearance_critic PRIVATE
    NAV_CONFIG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/nav2.yaml")
  ament_add_gtest(test_keepout_and_constraints
    test/navigation_clearance/test_keepout_and_constraints.cpp)
  target_include_directories(test_keepout_and_constraints PRIVATE include)
  ament_target_dependencies(test_keepout_and_constraints nav2_mppi_controller nav2_costmap_2d
    nav2_msgs nav2_util rclcpp rclcpp_lifecycle)
  target_compile_definitions(test_keepout_and_constraints PRIVATE
    NAV_CONFIG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/nav2.yaml")
  set_tests_properties(test_keepout_and_constraints PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=212;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  ament_add_gtest(test_ordinary_recovery_state
    test/navigation_recovery/test_recovery_state.cpp
    src/navigation_recovery/recovery_state.cpp)
  target_include_directories(test_ordinary_recovery_state PRIVATE include)
  ament_target_dependencies(test_ordinary_recovery_state nav_msgs)

  ament_add_gtest(test_ordinary_recovery_bt
    test/navigation_recovery/test_recovery_bt.cpp
    src/navigation_recovery/recovery_state.cpp)
  target_include_directories(test_ordinary_recovery_bt PRIVATE include)
  ament_target_dependencies(test_ordinary_recovery_bt behaviortree_cpp_v3 nav_msgs rclcpp std_msgs)
  target_include_directories(test_ordinary_recovery_bt BEFORE PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")
  target_link_libraries(test_ordinary_recovery_bt "${recovery_typesupport}")
  target_compile_definitions(test_ordinary_recovery_bt PRIVATE
    RECOVERY_BT_PLUGIN="$<TARGET_FILE:ordinary_navigation_recovery_bt_node>"
    RECOVERY_BT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/navigate_to_pose_ranger_lattice_recovery.xml")

  ament_add_gtest(test_ordinary_controller_recovery
    test/navigation_recovery/test_controller_recovery.cpp)
  target_include_directories(test_ordinary_controller_recovery PRIVATE include)
  target_include_directories(test_ordinary_controller_recovery BEFORE PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")
  target_link_libraries(test_ordinary_controller_recovery goal_scoped_rotation_shim_controller)
  ament_target_dependencies(test_ordinary_controller_recovery
    nav2_controller nav2_costmap_2d nav2_util rclcpp tf2_ros robot_interfaces)
  set_tests_properties(test_ordinary_controller_recovery PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=212;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")

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

  find_package(nav_2d_utils REQUIRED)
  ament_add_gtest(test_elevator_goal_stamp test/test_elevator_goal_stamp.cpp)
  target_include_directories(test_elevator_goal_stamp PRIVATE include)
  target_link_libraries(test_elevator_goal_stamp ranger_mini3_lattice_planner)
  ament_target_dependencies(test_elevator_goal_stamp
    nav2_controller nav_2d_utils nav2_costmap_2d nav2_util rclcpp tf2_geometry_msgs)
  set_tests_properties(test_elevator_goal_stamp PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=184;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")

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

  ament_add_gtest(test_ordinary_local_path_repair
    test/test_ordinary_local_path_repair.cpp
    src/ordinary_local_path_repair.cpp
  )
  target_include_directories(test_ordinary_local_path_repair PRIVATE include)
  target_link_libraries(test_ordinary_local_path_repair ${OMPL_LIBRARIES})
  ament_target_dependencies(test_ordinary_local_path_repair
    geometry_msgs
    nav2_costmap_2d
    nav2_smac_planner
    nav2_util
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    tf2
  )

  ament_add_gtest(test_ordinary_local_path_repair_worker
    test/test_ordinary_local_path_repair_worker.cpp
    src/ordinary_local_path_repair.cpp
    src/ordinary_local_path_repair_worker.cpp
  )
  target_include_directories(test_ordinary_local_path_repair_worker PRIVATE include)
  target_link_libraries(test_ordinary_local_path_repair_worker Threads::Threads)
  target_link_libraries(test_ordinary_local_path_repair_worker ${OMPL_LIBRARIES})
  ament_target_dependencies(test_ordinary_local_path_repair_worker
    geometry_msgs
    nav2_costmap_2d
    nav2_smac_planner
    nav2_util
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    tf2
  )

  ament_add_gtest(test_ordinary_local_path_repair_runtime
    test/test_ordinary_local_path_repair_runtime.cpp
    src/ordinary_local_path_repair.cpp
    src/ordinary_local_path_repair_worker.cpp
    src/ordinary_local_path_repair_runtime.cpp
  )
  target_include_directories(test_ordinary_local_path_repair_runtime PRIVATE include)
  target_link_libraries(test_ordinary_local_path_repair_runtime Threads::Threads)
  target_link_libraries(test_ordinary_local_path_repair_runtime ${OMPL_LIBRARIES})
  ament_target_dependencies(test_ordinary_local_path_repair_runtime
    geometry_msgs nav2_costmap_2d nav2_smac_planner nav2_util nav_msgs
    rclcpp rclcpp_lifecycle std_msgs tf2 tf2_geometry_msgs
  )

endif()

ament_export_include_directories(include)
ament_export_libraries(
  goal_scoped_rotation_shim_controller
  ranger_mini3_lattice_planner
)
ament_export_dependencies(
  nav2_mppi_controller
  rosidl_default_runtime
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_msgs
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

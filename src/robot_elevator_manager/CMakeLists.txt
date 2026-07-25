cmake_minimum_required(VERSION 3.8)
project(robot_elevator_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(robot_elevator_core
  src/elevator_fsm.cpp
  src/elevator_release_loader.cpp
  src/elevator_topology.cpp
  src/elevator_topology_loader.cpp
  src/threshold_footprint.cpp
)
target_compile_features(robot_elevator_core PUBLIC cxx_std_17)
target_include_directories(robot_elevator_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_elevator_core yaml-cpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  TARGETS robot_elevator_core
  EXPORT export_robot_elevator_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(robot_floor_manager REQUIRED)
  find_package(robot_localization_bridge REQUIRED)
  find_package(robot_mission_manager REQUIRED)
  find_package(robot_mode_manager REQUIRED)
  find_package(robot_safety REQUIRED)

  ament_add_gtest(test_elevator_topology test/test_elevator_topology.cpp)
  target_link_libraries(test_elevator_topology robot_elevator_core)

  ament_add_gtest(
    test_elevator_topology_loader
    test/test_elevator_topology_loader.cpp
  )
  target_link_libraries(test_elevator_topology_loader robot_elevator_core)

  ament_add_gtest(test_threshold_footprint test/test_threshold_footprint.cpp)
  target_link_libraries(test_threshold_footprint robot_elevator_core)

  ament_add_gtest(test_elevator_fsm test/test_elevator_fsm.cpp)
  target_link_libraries(test_elevator_fsm robot_elevator_core)

  ament_add_gtest(
    test_elevator_release_loader
    test/test_elevator_release_loader.cpp
  )
  target_link_libraries(test_elevator_release_loader robot_elevator_core)

  ament_add_gtest(
    test_nonmoving_cross_floor_scenario
    test/test_nonmoving_cross_floor_scenario.cpp
  )
  target_link_libraries(
    test_nonmoving_cross_floor_scenario
    robot_elevator_core
    robot_floor_manager::floor_transition_core
    robot_localization_bridge::correction_pause_arbiter
    robot_mission_manager::mission_fsm
    robot_mode_manager::mode_lease_arbiter
    robot_safety::motion_interlock_arbiter
  )
endif()

ament_export_include_directories(include)
ament_export_targets(export_robot_elevator_core HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

ament_package()

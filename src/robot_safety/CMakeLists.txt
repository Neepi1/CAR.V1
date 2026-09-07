cmake_minimum_required(VERSION 3.8)
project(robot_safety)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(motion_interlock_arbiter src/motion_interlock_arbiter.cpp)
target_compile_features(motion_interlock_arbiter PUBLIC cxx_std_17)
target_include_directories(motion_interlock_arbiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(persistent_sequence_allocator
  src/persistent_sequence_allocator.cpp
)
target_compile_features(persistent_sequence_allocator PUBLIC cxx_std_17)
target_include_directories(persistent_sequence_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(elevator_entry_collision_bypass_policy
  src/elevator_entry_collision_bypass_policy.cpp
)
target_compile_features(elevator_entry_collision_bypass_policy PUBLIC cxx_std_17)
target_include_directories(elevator_entry_collision_bypass_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(dock_contact_policy src/dock_contact_policy.cpp)
target_compile_features(dock_contact_policy PUBLIC cxx_std_17)
target_include_directories(dock_contact_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(robot_safety_node src/robot_safety_node.cpp)
target_link_libraries(
  robot_safety_node
  motion_interlock_arbiter
  elevator_entry_collision_bypass_policy
  dock_contact_policy
)
ament_target_dependencies(
  robot_safety_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  sensor_msgs
  std_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS
    motion_interlock_arbiter
    persistent_sequence_allocator
    elevator_entry_collision_bypass_policy
    dock_contact_policy
    robot_safety_node
  EXPORT export_motion_interlock_arbiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_motion_interlock_arbiter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_motion_interlock_arbiter
    test/test_motion_interlock_arbiter.cpp
  )
  target_link_libraries(test_motion_interlock_arbiter motion_interlock_arbiter)

  ament_add_gtest(
    test_persistent_sequence_allocator
    test/test_persistent_sequence_allocator.cpp
  )
  target_link_libraries(
    test_persistent_sequence_allocator
    persistent_sequence_allocator
  )

  ament_add_gtest(
    test_elevator_entry_collision_bypass_policy
    test/test_elevator_entry_collision_bypass_policy.cpp
  )
  target_link_libraries(
    test_elevator_entry_collision_bypass_policy
    elevator_entry_collision_bypass_policy
  )

  ament_add_gtest(
    test_navigation_speed_envelope
    test/test_navigation_speed_envelope.cpp
  )
  target_link_libraries(test_navigation_speed_envelope motion_interlock_arbiter)

  ament_add_gtest(
    test_dock_contact_policy
    test/test_dock_contact_policy.cpp
  )
  target_link_libraries(test_dock_contact_policy dock_contact_policy)
endif()

ament_package()

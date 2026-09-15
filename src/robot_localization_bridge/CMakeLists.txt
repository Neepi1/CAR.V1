cmake_minimum_required(VERSION 3.8)
project(robot_localization_bridge)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(correction_pause_arbiter
  src/correction_pause_arbiter.cpp
)
target_compile_features(correction_pause_arbiter PUBLIC cxx_std_17)
target_include_directories(correction_pause_arbiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(floor_transition_context
  src/floor_transition_context.cpp
)
target_compile_features(floor_transition_context PUBLIC cxx_std_17)
target_include_directories(floor_transition_context PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(localization_bridge_node src/localization_bridge_node.cpp)
target_include_directories(localization_bridge_node PRIVATE include)
target_link_libraries(
  localization_bridge_node
  correction_pause_arbiter
  floor_transition_context
)
ament_target_dependencies(
  localization_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_msgs
)

add_executable(amcl_scan_admission_node src/amcl_scan_admission_node.cpp)
ament_target_dependencies(
  amcl_scan_admission_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS correction_pause_arbiter floor_transition_context localization_bridge_node
    amcl_scan_admission_node
  EXPORT export_correction_pause_arbiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_correction_pause_arbiter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME isolated_correction_application
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/isolated_correction_application.py
      --bridge $<TARGET_FILE:localization_bridge_node>)
  set_tests_properties(isolated_correction_application PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_se2_correction test/test_se2_correction.cpp)
  target_include_directories(test_se2_correction PRIVATE include)
  ament_add_gtest(test_post_isaac_refine_gate test/test_post_isaac_refine_gate.cpp)
  target_include_directories(test_post_isaac_refine_gate PRIVATE include)
  ament_add_gtest(
    test_explicit_localization_result_policy
    test/test_explicit_localization_result_policy.cpp
  )
  target_include_directories(test_explicit_localization_result_policy PRIVATE include)
  ament_add_gtest(test_correction_pause_arbiter test/test_correction_pause_arbiter.cpp)
  target_link_libraries(test_correction_pause_arbiter correction_pause_arbiter)
  ament_add_gtest(test_floor_transition_context test/test_floor_transition_context.cpp)
  target_link_libraries(test_floor_transition_context floor_transition_context)
endif()

ament_package()

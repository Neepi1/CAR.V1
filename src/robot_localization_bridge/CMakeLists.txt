cmake_minimum_required(VERSION 3.8)
project(robot_localization_bridge)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(correction_pause_arbiter
  src/correction_pause_arbiter.cpp
)
target_compile_features(correction_pause_arbiter PUBLIC cxx_std_17)
target_include_directories(correction_pause_arbiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(localization_bridge_node src/localization_bridge_node.cpp)
ament_target_dependencies(
  localization_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2_ros
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
  TARGETS correction_pause_arbiter
  EXPORT export_correction_pause_arbiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(TARGETS localization_bridge_node amcl_scan_admission_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_correction_pause_arbiter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_correction_pause_arbiter test/test_correction_pause_arbiter.cpp)
  target_link_libraries(test_correction_pause_arbiter correction_pause_arbiter)
endif()

ament_package()

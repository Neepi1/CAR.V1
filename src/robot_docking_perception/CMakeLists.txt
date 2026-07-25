cmake_minimum_required(VERSION 3.8)
project(robot_docking_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(depth_dock_geometry src/depth_dock_geometry.cpp)
target_compile_features(depth_dock_geometry PUBLIC cxx_std_17)
target_include_directories(
  depth_dock_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(orbbec_depth_dock_node src/orbbec_depth_dock_node.cpp)
target_compile_features(orbbec_depth_dock_node PUBLIC cxx_std_17)
target_link_libraries(orbbec_depth_dock_node depth_dock_geometry)
ament_target_dependencies(
  orbbec_depth_dock_node
  rclcpp
  robot_interfaces
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS depth_dock_geometry orbbec_depth_dock_node
  EXPORT depth_dock_geometry_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_dock_geometry test/test_depth_dock_geometry.cpp)
  target_link_libraries(test_depth_dock_geometry depth_dock_geometry)
endif()

ament_export_targets(depth_dock_geometry_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()

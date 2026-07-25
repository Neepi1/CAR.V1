cmake_minimum_required(VERSION 3.8)
project(robot_mode_manager)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(mode_lease_arbiter
  src/mode_lease_arbiter.cpp
)
target_compile_features(mode_lease_arbiter PUBLIC cxx_std_17)
target_include_directories(mode_lease_arbiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(mode_manager_node src/mode_manager_node.cpp)
target_compile_features(mode_manager_node PUBLIC cxx_std_17)
target_link_libraries(mode_manager_node mode_lease_arbiter)
ament_target_dependencies(
  mode_manager_node
  builtin_interfaces
  rclcpp
  robot_interfaces
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS mode_lease_arbiter mode_manager_node
  EXPORT export_mode_lease_arbiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_mode_lease_arbiter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mode_lease_arbiter test/test_mode_lease_arbiter.cpp)
  target_link_libraries(test_mode_lease_arbiter mode_lease_arbiter)
endif()

ament_package()

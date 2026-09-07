cmake_minimum_required(VERSION 3.8)
project(robot_docking_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ranger_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(
  docking_manager_node
  src/docking_manager_node.cpp
  src/near_field_docking_controller.cpp
)
target_compile_features(docking_manager_node PUBLIC cxx_std_17)
target_include_directories(docking_manager_node PRIVATE include)
ament_target_dependencies(
  docking_manager_node
  geometry_msgs
  nav_msgs
  ranger_msgs
  rclcpp
  robot_interfaces
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS docking_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_near_field_docking_controller
    test/test_near_field_docking_controller.cpp
    src/near_field_docking_controller.cpp
  )
  target_compile_features(test_near_field_docking_controller PRIVATE cxx_std_17)
  target_include_directories(test_near_field_docking_controller PRIVATE include)
endif()

ament_package()

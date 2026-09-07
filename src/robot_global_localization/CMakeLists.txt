cmake_minimum_required(VERSION 3.8)
project(robot_global_localization)

find_package(ament_cmake REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(isaac_ros_pointcloud_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(isaac_asset_reloader
  src/isaac_asset_reloader.cpp
  src/post_reload_readiness_gate.cpp
  src/ros_component_manager_port.cpp
)
target_compile_features(isaac_asset_reloader PUBLIC cxx_std_17)
target_include_directories(isaac_asset_reloader
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(isaac_asset_reloader yaml-cpp)
ament_target_dependencies(
  isaac_asset_reloader
  composition_interfaces
  rcl_interfaces
  rclcpp
)

add_executable(global_localization_node
  src/global_localization_node.cpp
)
target_compile_features(global_localization_node PUBLIC cxx_std_17)
target_link_libraries(global_localization_node isaac_asset_reloader)
ament_target_dependencies(global_localization_node
  composition_interfaces
  geometry_msgs
  isaac_ros_pointcloud_interfaces
  rclcpp
  rcl_interfaces
  robot_interfaces
  std_msgs
  std_srvs
  tf2_ros
)

install(DIRECTORY config docs launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS isaac_asset_reloader global_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME trigger_client_lifetime_contract
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_trigger_client_lifetime_contract.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/global_localization_node.cpp
  )

  ament_add_gtest(
    test_isaac_asset_reloader
    test/test_isaac_asset_reloader.cpp
  )
  target_link_libraries(test_isaac_asset_reloader isaac_asset_reloader)

  ament_add_gtest(
    test_ros_component_manager_port
    test/test_ros_component_manager_port.cpp
  )
  ament_target_dependencies(
    test_ros_component_manager_port
    composition_interfaces
    rcl_interfaces
    rclcpp
  )
  target_link_libraries(test_ros_component_manager_port isaac_asset_reloader)
  set_tests_properties(
    test_ros_component_manager_port
    PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=198;ROS_LOCALHOST_ONLY=1"
      RUN_SERIAL TRUE
  )

  ament_add_gtest(
    test_post_reload_readiness_gate
    test/test_post_reload_readiness_gate.cpp
  )
  target_link_libraries(
    test_post_reload_readiness_gate
    isaac_asset_reloader
  )
endif()

ament_export_dependencies(composition_interfaces rcl_interfaces rclcpp yaml-cpp)
ament_export_libraries(isaac_asset_reloader)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_hesai_jt128 REQUIRED)
find_package(robot_local_state REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_path(RAPIDJSON_INCLUDE_DIR rapidjson/document.h REQUIRED)

add_executable(imu_pipeline_node src/imu_pipeline_node.cpp)
target_compile_features(imu_pipeline_node PRIVATE cxx_std_17)
ament_target_dependencies(imu_pipeline_node rclcpp)
target_link_libraries(imu_pipeline_node
  robot_hesai_jt128::imu_axis_remap_core
  robot_local_state::imu_gyro_bias_filter_core pthread)
set_target_properties(imu_pipeline_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN/../../../robot_hesai_jt128/lib;$ORIGIN/../../../robot_local_state/lib")

add_library(runtime_amcl_status_core STATIC src/runtime_amcl_status_server.cpp)
target_compile_features(runtime_amcl_status_core PUBLIC cxx_std_17)
target_include_directories(runtime_amcl_status_core PUBLIC include ${RAPIDJSON_INCLUDE_DIR})

add_executable(runtime_amcl_status src/runtime_amcl_status.cpp)
target_link_libraries(runtime_amcl_status PRIVATE runtime_amcl_status_core)

add_executable(runtime_process_check src/runtime_process_check.cpp)
target_compile_features(runtime_process_check PRIVATE cxx_std_17)

add_executable(runtime_flatscan_check src/runtime_flatscan_check.cpp)
target_compile_features(runtime_flatscan_check PRIVATE cxx_std_17)
target_include_directories(runtime_flatscan_check PRIVATE include ${RAPIDJSON_INCLUDE_DIR})

add_executable(runtime_health_guard src/runtime_health_guard.cpp src/runtime_flatscan_monitor.cpp)
target_compile_features(runtime_health_guard PRIVATE cxx_std_17)
target_include_directories(runtime_health_guard PRIVATE include ${RAPIDJSON_INCLUDE_DIR})
target_link_libraries(runtime_health_guard runtime_amcl_status_core)
ament_target_dependencies(runtime_health_guard rclcpp geometry_msgs lifecycle_msgs nav_msgs robot_interfaces sensor_msgs std_msgs tf2_msgs)

add_executable(runtime_health_check src/runtime_health_check.cpp)
target_compile_features(runtime_health_check PRIVATE cxx_std_17)
target_include_directories(runtime_health_check PRIVATE include ${RAPIDJSON_INCLUDE_DIR})

add_executable(startup_context_observer src/startup_context_observer.cpp)
target_compile_features(startup_context_observer PRIVATE cxx_std_17)
target_include_directories(startup_context_observer PRIVATE ${RAPIDJSON_INCLUDE_DIR})
ament_target_dependencies(startup_context_observer rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_executable(test_startup_context_observer test/test_startup_context_observer.cpp)
  target_compile_features(test_startup_context_observer PRIVATE cxx_std_17)
  target_compile_options(test_startup_context_observer PRIVATE -UNDEBUG)
  target_include_directories(test_startup_context_observer PRIVATE ${RAPIDJSON_INCLUDE_DIR})
  add_test(NAME startup_context_observer_core COMMAND test_startup_context_observer)
  add_executable(test_runtime_health_policy test/test_runtime_health_policy.cpp)
  target_compile_features(test_runtime_health_policy PRIVATE cxx_std_17)
  target_compile_options(test_runtime_health_policy PRIVATE -UNDEBUG)
  target_include_directories(test_runtime_health_policy PRIVATE include)
  add_test(NAME runtime_health_policy COMMAND test_runtime_health_policy)
  add_test(NAME runtime_health_cli
    COMMAND ${CMAKE_COMMAND} -E env NJRH_RUNTIME_HEALTH_CHECK_BIN=$<TARGET_FILE:runtime_health_check>
      ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_health_check.py)
  add_test(NAME runtime_process_cli
    COMMAND ${CMAKE_COMMAND} -E env NJRH_RUNTIME_PROCESS_CHECK_BIN=$<TARGET_FILE:runtime_process_check>
      ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_process_check.py)
  add_executable(test_runtime_flatscan_snapshot test/test_runtime_flatscan_snapshot.cpp)
  target_compile_features(test_runtime_flatscan_snapshot PRIVATE cxx_std_17)
  target_compile_options(test_runtime_flatscan_snapshot PRIVATE -UNDEBUG)
  target_include_directories(test_runtime_flatscan_snapshot PRIVATE include ${RAPIDJSON_INCLUDE_DIR})
  add_test(NAME runtime_flatscan_snapshot COMMAND test_runtime_flatscan_snapshot)
  add_test(NAME runtime_flatscan_management
    COMMAND ${Python3_EXECUTABLE} -m pytest -q
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_flatscan_management.py)
  add_executable(test_runtime_amcl_status test/test_runtime_amcl_status.cpp)
  target_compile_features(test_runtime_amcl_status PRIVATE cxx_std_17)
  target_compile_options(test_runtime_amcl_status PRIVATE -UNDEBUG)
  target_link_libraries(test_runtime_amcl_status PRIVATE runtime_amcl_status_core pthread)
  add_test(NAME runtime_amcl_status COMMAND test_runtime_amcl_status)
  add_test(NAME runtime_amcl_status_cli
    COMMAND ${CMAKE_COMMAND} -E env
      RUNTIME_AMCL_STATUS_BIN=$<TARGET_FILE:runtime_amcl_status>
      RUNTIME_AMCL_STATUS_FIXTURE_BIN=$<TARGET_FILE:test_runtime_amcl_status>
      ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_amcl_status_cli.py -v)
endif()

add_executable(runtime_readiness_probe
  src/runtime_readiness_probe.cpp
)
target_compile_features(runtime_readiness_probe PRIVATE cxx_std_17)
ament_target_dependencies(runtime_readiness_probe
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_msgs
  tf2_ros
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS imu_pipeline_node runtime_readiness_probe runtime_health_guard runtime_health_check
  runtime_process_check runtime_amcl_status runtime_flatscan_check startup_context_observer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

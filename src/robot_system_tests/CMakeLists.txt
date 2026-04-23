cmake_minimum_required(VERSION 3.8)
project(robot_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_workspace_contracts test/test_workspace_contracts.py)
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_elevator_cleanup_identity_contract
    test/test_elevator_cleanup_identity_contract.py)
  ament_add_pytest_test(
    test_elevator_scoped_navigation_contract
    test/test_elevator_scoped_navigation_contract.py)
  ament_add_pytest_test(
    test_nav2_footprint_inflation_contract
    test/test_nav2_footprint_inflation_contract.py)
  ament_add_pytest_test(test_workspace_contracts test/test_workspace_contracts.py)
  ament_add_pytest_test(test_build_production_release test/test_build_production_release.py)
  ament_add_pytest_test(test_production_provisioning test/test_production_provisioning.py)
  ament_add_pytest_test(test_hardware_preflight test/test_hardware_preflight.py)
  ament_add_pytest_test(test_production_runtime_shell test/test_production_runtime_shell.py)
  ament_add_pytest_test(test_production_site_assets test/test_production_site_assets.py)
endif()

ament_package()

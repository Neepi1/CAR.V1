cmake_minimum_required(VERSION 3.8)
project(robot_occupancy_builder)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/occupancy_builder_live_node.py
    scripts/occupancy_builder_release_node.py
    scripts/rebuild_from_bag.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES
    scripts/occupancy_postprocess.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_occupancy_builder_contracts test/test_occupancy_builder_contracts.py)
endif()

ament_package()

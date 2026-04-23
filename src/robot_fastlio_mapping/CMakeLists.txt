cmake_minimum_required(VERSION 3.8)
project(robot_fastlio_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fastlio_wrapper_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()

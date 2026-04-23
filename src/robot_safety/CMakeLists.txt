cmake_minimum_required(VERSION 3.8)
project(robot_safety)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/robot_safety_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()

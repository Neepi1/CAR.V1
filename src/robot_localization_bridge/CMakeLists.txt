cmake_minimum_required(VERSION 3.8)
project(robot_localization_bridge)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/localization_bridge_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()

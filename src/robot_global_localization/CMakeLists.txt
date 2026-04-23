cmake_minimum_required(VERSION 3.8)
project(robot_global_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/global_localization_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()

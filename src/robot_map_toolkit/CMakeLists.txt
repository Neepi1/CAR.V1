cmake_minimum_required(VERSION 3.8)
project(robot_map_toolkit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/map_toolkit_cli.py DESTINATION lib/${PROJECT_NAME})

ament_package()

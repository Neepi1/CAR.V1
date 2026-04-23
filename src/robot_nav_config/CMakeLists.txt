cmake_minimum_required(VERSION 3.8)
project(robot_nav_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config behavior_trees docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

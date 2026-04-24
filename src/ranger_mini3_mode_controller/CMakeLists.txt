cmake_minimum_required(VERSION 3.8)
project(ranger_mini3_mode_controller)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mode_controller_node src/mode_controller_node.cpp)
ament_target_dependencies(mode_controller_node geometry_msgs rclcpp std_msgs)

install(TARGETS mode_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()

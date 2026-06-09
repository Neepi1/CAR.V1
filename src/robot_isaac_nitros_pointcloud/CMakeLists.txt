cmake_minimum_required(VERSION 3.8)
project(robot_isaac_nitros_pointcloud)

find_package(ament_cmake REQUIRED)

option(ROBOT_ISAAC_NITROS_POINTCLOUD_ENABLE "Build experimental Isaac ROS NITROS pointcloud components" OFF)

if(ROBOT_ISAAC_NITROS_POINTCLOUD_ENABLE)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(isaac_ros_nitros REQUIRED)
  find_package(isaac_ros_managed_nitros REQUIRED)
  find_package(isaac_ros_nitros_point_cloud_type REQUIRED)

  message(STATUS "robot_isaac_nitros_pointcloud NITROS skeleton enabled")
else()
  message(STATUS "robot_isaac_nitros_pointcloud NITROS skeleton disabled; legacy/ipc_worker builds do not require Isaac ROS NITROS")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()

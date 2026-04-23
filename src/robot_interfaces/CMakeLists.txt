cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HealthStatus.msg"
  "msg/LocalizationHealth.msg"
  "msg/MappingJobStatus.msg"
  "srv/ApplyFloorAssets.srv"
  "srv/TriggerLocalization.srv"
  "srv/SetMode.srv"
  "action/MissionTask.action"
  "action/ElevatorTask.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

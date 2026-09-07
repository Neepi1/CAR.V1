cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmState.msg"
  "msg/CorrectionPauseState.msg"
  "msg/DockTargetObservation.msg"
  "msg/ElevatorObservation.msg"
  "msg/FloorSwitchStatus.msg"
  "msg/HealthStatus.msg"
  "msg/LocalizationHealth.msg"
  "msg/LocalizerAssetState.msg"
  "msg/MappingJobStatus.msg"
  "msg/MotionInterlockState.msg"
  "msg/OperatingModeState.msg"
  "srv/ApplyFloorAssets.srv"
  "srv/BeginFloorTransition.srv"
  "srv/SwitchFloor.srv"
  "srv/TriggerLocalization.srv"
  "srv/SetMode.srv"
  "srv/SetMotionHold.srv"
  "srv/ReleaseMotionHoldIfExecutionIdle.srv"
  "srv/SetExecutionLease.srv"
  "srv/SetCorrectionPause.srv"
  "srv/SetElevatorNavigationSession.srv"
  "action/FloorSwitch.action"
  "action/MissionTask.action"
  "action/ElevatorTask.action"
  "action/PressButton.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

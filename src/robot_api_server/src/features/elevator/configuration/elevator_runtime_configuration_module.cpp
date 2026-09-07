#include \
  "robot_api_server/features/elevator/configuration/elevator_runtime_configuration_module.hpp"

#include <algorithm>

namespace robot_api_server::features::elevator::configuration
{

ElevatorModuleConfig ElevatorRuntimeConfigurationModule::declare_parameters(
  rclcpp::Node & node,
  const ElevatorRuntimeConfigurationInputs & inputs)
{
  ElevatorModuleConfig config;
  config.maps_root = inputs.maps_root;
  config.runtime_map_context_file = inputs.runtime_map_context_file;
  config.navigate_to_pose_action = inputs.navigate_to_pose_action;
  config.floor_switch_action = inputs.floor_switch_action;
  config.floor_switch_timeout_sec = inputs.floor_switch_timeout_sec;
  config.motion_allowed_topic = inputs.motion_allowed_topic;
  config.safety_status_topic = inputs.safety_status_topic;
  config.navigation_status_topic = inputs.navigation_status_topic;

  config.runtime_adapter_enabled = node.declare_parameter<bool>(
    "elevator_runtime_adapter_enabled", false);
  config.arm_button_control_enabled = node.declare_parameter<bool>(
    "elevator_arm_button_control_enabled", false);
  config.arm_service_host = node.declare_parameter<std::string>(
    "elevator_arm_service_host", "127.0.0.1");
  config.arm_service_port = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "elevator_arm_service_port", 8083)),
    1,
    65535);
  config.arm_request_timeout_sec = std::clamp(
    node.declare_parameter<double>("elevator_arm_request_timeout_sec", 12.0),
    0.1,
    60.0);
  config.arm_task_timeout_sec = std::clamp(
    node.declare_parameter<double>("elevator_arm_task_timeout_sec", 180.0),
    1.0,
    600.0);
  config.arm_poll_interval_sec = std::clamp(
    node.declare_parameter<double>("elevator_arm_poll_interval_sec", 0.20),
    0.02,
    2.0);
  config.elevator_scoped_behavior_tree = node.declare_parameter<std::string>(
    "elevator_scoped_behavior_tree",
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/navigate_elevator_scoped_motion.xml");
  config.elevator_hall_call_scoped_behavior_tree =
    node.declare_parameter<std::string>(
    "elevator_hall_call_scoped_behavior_tree",
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/"
    "navigate_elevator_hall_call_scoped_motion.xml");
  config.elevator_reverse_entry_staging_behavior_tree =
    node.declare_parameter<std::string>(
    "elevator_reverse_entry_staging_behavior_tree",
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/"
    "navigate_elevator_reverse_entry_staging.xml");
  config.elevator_reverse_docking_behavior_tree =
    node.declare_parameter<std::string>(
    "elevator_reverse_docking_behavior_tree",
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/navigate_elevator_reverse_docking.xml");
  config.elevator_cabin_entry_direct_behavior_tree =
    node.declare_parameter<std::string>(
    "elevator_cabin_entry_direct_behavior_tree",
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/navigate_elevator_cabin_entry_direct.xml");
  config.elevator_entry_collision_bypass_permit_topic =
    node.declare_parameter<std::string>(
    "elevator_entry_collision_bypass_permit_topic",
    "/ranger_mini3/elevator_entry_collision_bypass");
  config.elevator_entry_collision_bypass_refresh_sec = std::max(
    0.05,
    node.declare_parameter<double>(
      "elevator_entry_collision_bypass_refresh_sec", 0.20));
  config.recovery_hold_release_service = node.declare_parameter<std::string>(
    "elevator_recovery_hold_release_service",
    "/safety/release_motion_hold_if_execution_idle");

  return config;
}

}  // namespace robot_api_server::features::elevator::configuration

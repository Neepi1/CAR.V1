#include "robot_api_server/features/safety/safety_configuration_module.hpp"

namespace robot_api_server::features::safety
{

SafetyConfiguration SafetyConfigurationModule::declare_parameters(rclcpp::Node & node)
{
  SafetyConfiguration config;
  config.module.ros.estop_topic = node.declare_parameter<std::string>(
    "safety_estop_topic", "/safety/estop");
  config.module.ros.status_topic = node.declare_parameter<std::string>(
    "safety_status_topic", "/safety/status");
  config.module.ros.motion_allowed_topic = node.declare_parameter<std::string>(
    "safety_motion_allowed_topic", "/safety/motion_allowed");
  return config;
}

}  // namespace robot_api_server::features::safety

#include \
  "robot_api_server/application/runtime_configuration/runtime_configuration_module.hpp"

#include <chrono>
#include <string>

namespace robot_api_server::application::runtime_configuration
{

std::chrono::nanoseconds RuntimeConfiguration::service_timeout() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(service_timeout_sec));
}

RuntimeConfiguration RuntimeConfigurationModule::declare_parameters(rclcpp::Node & node)
{
  RuntimeConfiguration config;
  config.navigate_to_pose_action = node.declare_parameter<std::string>(
    "navigate_to_pose_action", "/navigate_to_pose");
  config.navigate_to_pose_status_topic = node.declare_parameter<std::string>(
    "navigate_to_pose_status_topic", "/navigate_to_pose/_action/status");
  config.service_timeout_sec = node.declare_parameter<double>(
    "service_timeout_sec", 8.0);
  return config;
}

}  // namespace robot_api_server::application::runtime_configuration

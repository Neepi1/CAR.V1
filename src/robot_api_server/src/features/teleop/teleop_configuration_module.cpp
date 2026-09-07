#include "robot_api_server/features/teleop/teleop_configuration_module.hpp"

#include <algorithm>
#include <string>

namespace robot_api_server::features::teleop
{

TeleopConfiguration TeleopConfigurationModule::declare_parameters(
  rclcpp::Node & node,
  const TeleopConfigurationInputs & inputs)
{
  TeleopConfiguration config;
  auto & module = config.module;

  module.stop_on_charging = node.declare_parameter<bool>(
    "teleop_stop_on_charging", true);
  module.cmd_topic = node.declare_parameter<std::string>(
    "teleop_cmd_topic", "/cmd_vel_api");
  module.reverse_enable_topic = node.declare_parameter<std::string>(
    "teleop_reverse_enable_topic", "/ranger_mini3/teleop_allow_reverse");
  module.pose_topic = node.declare_parameter<std::string>(
    "teleop_pose_topic", "/local_state/odometry");
  module.max_linear_x_mps = std::max(
    0.0,
    node.declare_parameter<double>("teleop_max_linear_x_mps", 1.00));
  module.max_angular_z_radps = std::max(
    0.0,
    node.declare_parameter<double>("teleop_max_angular_z_radps", 0.55));
  module.allow_reverse = node.declare_parameter<bool>(
    "teleop_allow_reverse", false);
  module.require_mapping_active = node.declare_parameter<bool>(
    "teleop_require_mapping_active", true);
  module.watchdog_timeout_sec = std::max(
    0.1,
    node.declare_parameter<double>("teleop_watchdog_timeout_sec", 0.5));
  module.socket_idle_timeout_sec = std::max(
    module.watchdog_timeout_sec,
    node.declare_parameter<double>("teleop_socket_idle_timeout_sec", 5.0));
  module.repeat_rate_hz = std::max(
    1.0,
    node.declare_parameter<double>("teleop_repeat_rate_hz", 20.0));
  module.subscription_max_ttl_ms = inputs.subscription_max_ttl_ms;

  return config;
}

}  // namespace robot_api_server::features::teleop

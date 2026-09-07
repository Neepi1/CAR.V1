#pragma once

#include <chrono>
#include <string>

#include "rclcpp/node.hpp"

namespace robot_api_server::application::runtime_configuration
{

// Shared names and the base service budget projected into multiple feature
// configuration graphs. This aggregate owns no runtime behavior.
struct RuntimeConfiguration
{
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string navigate_to_pose_status_topic{"/navigate_to_pose/_action/status"};
  double service_timeout_sec{8.0};

  std::chrono::nanoseconds service_timeout() const;
};

class RuntimeConfigurationModule
{
public:
  static RuntimeConfiguration declare_parameters(rclcpp::Node & node);
};

}  // namespace robot_api_server::application::runtime_configuration

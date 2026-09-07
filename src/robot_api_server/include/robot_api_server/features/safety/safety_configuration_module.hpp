#pragma once

#include "rclcpp/node.hpp"

#include "robot_api_server/features/safety/safety_module.hpp"

namespace robot_api_server::features::safety
{

struct SafetyConfiguration
{
  SafetyModuleConfig module;
};

class SafetyConfigurationModule
{
public:
  static SafetyConfiguration declare_parameters(rclcpp::Node & node);
};

}  // namespace robot_api_server::features::safety

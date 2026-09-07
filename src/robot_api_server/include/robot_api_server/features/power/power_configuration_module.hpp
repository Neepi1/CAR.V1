#pragma once

#include "rclcpp/node.hpp"

#include "robot_api_server/features/power/power_module.hpp"

namespace robot_api_server::features::power
{

// The complete validated configuration consumed by the process-resident BMS
// edge. Neighboring modules consume fields from this aggregate and never
// redeclare the underlying ROS parameters.
struct PowerConfiguration
{
  PowerModuleConfig module;
};

class PowerConfigurationModule
{
public:
  static PowerConfiguration declare_parameters(rclcpp::Node & node);
};

}  // namespace robot_api_server::features::power

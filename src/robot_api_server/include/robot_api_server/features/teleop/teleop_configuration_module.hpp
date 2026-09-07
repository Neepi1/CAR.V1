#pragma once

#include "rclcpp/node.hpp"

#include "robot_api_server/features/teleop/teleop_module.hpp"

namespace robot_api_server::features::teleop
{

// Values owned and normalized by neighboring application modules. Teleop
// consumes the projection without redeclaring their ROS parameters.
struct TeleopConfigurationInputs
{
  int subscription_max_ttl_ms{60000};
};

// The complete validated configuration consumed by the Teleop vertical slice.
struct TeleopConfiguration
{
  TeleopModuleConfig module;
};

class TeleopConfigurationModule
{
public:
  static TeleopConfiguration declare_parameters(
    rclcpp::Node & node,
    const TeleopConfigurationInputs & inputs);
};

}  // namespace robot_api_server::features::teleop

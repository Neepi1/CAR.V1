#pragma once

#include "rclcpp/node.hpp"

#include "robot_api_server/infrastructure/http/api_gateway_module.hpp"

namespace robot_api_server::infrastructure::http
{

// Declares the complete ROS-facing gateway configuration. Runtime-only
// defaults and normalization remain in ApiGatewayModule.
class ApiGatewayConfigurationModule
{
public:
  static ApiGatewayModuleConfig declare_parameters(rclcpp::Node & node);
};

}  // namespace robot_api_server::infrastructure::http

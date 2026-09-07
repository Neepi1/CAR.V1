#pragma once

#include "rclcpp/node.hpp"

#include "robot_api_server/application/subscriptions/subscription_module.hpp"

namespace robot_api_server::application::subscriptions
{

// Declares the complete ROS-facing page-subscription configuration. Runtime
// normalization remains colocated with the lease/cache implementation.
class SubscriptionConfigurationModule
{
public:
  static SubscriptionModuleConfig declare_parameters(rclcpp::Node & node);
};

}  // namespace robot_api_server::application::subscriptions

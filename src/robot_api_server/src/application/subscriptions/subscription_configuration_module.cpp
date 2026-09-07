#include "robot_api_server/application/subscriptions/subscription_configuration_module.hpp"

#include <string>

namespace robot_api_server::application::subscriptions
{

SubscriptionModuleConfig SubscriptionConfigurationModule::declare_parameters(
  rclcpp::Node & node)
{
  SubscriptionModuleConfig config;
  config.scan_topic = node.declare_parameter<std::string>("scan_topic", "/scan");
  config.scan_max_age_sec = node.declare_parameter<double>("scan_max_age_sec", 2.0);
  config.default_ttl_ms = node.declare_parameter<int>(
    "subscription_default_ttl_ms", 10000);
  config.max_ttl_ms = node.declare_parameter<int>(
    "subscription_max_ttl_ms", 60000);
  return config;
}

}  // namespace robot_api_server::application::subscriptions

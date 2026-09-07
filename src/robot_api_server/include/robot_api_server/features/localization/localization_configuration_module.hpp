#pragma once

#include <string>

#include "rclcpp/node.hpp"

#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/localization/post_relocalization_settle_module.hpp"

namespace robot_api_server::features::localization
{

// Values owned by shared application composition. This module consumes them
// without redeclaring their ROS parameters.
struct LocalizationConfigurationInputs
{
  double service_timeout_sec{8.0};
};

// Complete validated construction graph for the API-side localization
// facade and its post-relocalization stability transaction.
struct LocalizationConfiguration
{
  LocalizationModuleConfig module;
  PostRelocalizationSettleConfig settle;

  // Explicit projection into FloorSwitchModule.
  std::string floor_health_topic{"/localization/floor_health"};
};

// Declares every API-localization-owned ROS parameter exactly once, applies
// the established normalization/clamp graph, and emits ready-to-consume
// configs. It performs no runtime probe, localization trigger, TF publish, or
// motion command.
class LocalizationConfigurationModule
{
public:
  static LocalizationConfiguration declare_parameters(
    rclcpp::Node & node,
    const LocalizationConfigurationInputs & inputs);
};

}  // namespace robot_api_server::features::localization

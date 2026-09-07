#include "robot_api_server/features/power/power_configuration_module.hpp"

#include <algorithm>

namespace robot_api_server::features::power
{

PowerConfiguration PowerConfigurationModule::declare_parameters(rclcpp::Node & node)
{
  PowerConfiguration config;
  auto & module = config.module;

  module.state_topic = node.declare_parameter<std::string>(
    "bms_state_topic", "/battery_state");
  module.state_max_age_sec = std::max(
    0.1,
    node.declare_parameter<double>("bms_state_max_age_sec", 3.0));

  // Keep the deployed parameter name for compatibility. This threshold is
  // shared BMS contact evidence, not a teleop-owned decision.
  module.charging_current_min_a = std::max(
    0.0,
    node.declare_parameter<double>("teleop_charging_current_min_a", 0.10));
  module.charging_contact_voltage_min_v = std::max(
    0.0,
    node.declare_parameter<double>("bms_charging_contact_voltage_min_v", 40.0));
  module.charging_contact_voltage_max_v = std::max(
    module.charging_contact_voltage_min_v,
    node.declare_parameter<double>("bms_charging_contact_voltage_max_v", 1000.0));
  module.full_soc_threshold_pct = std::clamp(
    node.declare_parameter<double>("bms_full_soc_threshold_pct", 99.0),
    0.0,
    100.0);
  module.full_soc_voltage_contact_enable = node.declare_parameter<bool>(
    "bms_full_soc_voltage_contact_enable", true);
  module.contact_stable_required_sec = std::max(
    0.0,
    node.declare_parameter<double>("dock_contact_latch_bms_require_contact_sec", 2.0));

  return config;
}

}  // namespace robot_api_server::features::power

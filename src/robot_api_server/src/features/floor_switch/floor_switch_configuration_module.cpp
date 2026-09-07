#include \
  "robot_api_server/features/floor_switch/floor_switch_configuration_module.hpp"

#include <algorithm>
#include <string>

namespace robot_api_server::features::floor_switch
{

FloorSwitchConfiguration FloorSwitchConfigurationModule::declare_parameters(
  rclcpp::Node & node,
  const FloorSwitchConfigurationInputs & inputs)
{
  FloorSwitchConfiguration result;
  auto & module = result.module;

  module.maps_root = inputs.maps_root;
  module.runtime_map_context_file = inputs.runtime_map_context_file;
  module.localization_health_topic = inputs.localization_health_topic;
  module.service_timeout = inputs.service_timeout;

  result.floor_status_topic = node.declare_parameter<std::string>(
    "floor_status_topic", "/floor_manager/status");
  module.transition_status_topic = node.declare_parameter<std::string>(
    "floor_transition_status_topic", "/floor_manager/transition_status");
  module.negative_interlock_enabled = node.declare_parameter<bool>(
    "floor_runtime_negative_interlock_enabled", true);
  module.legacy_service = node.declare_parameter<std::string>(
    "floor_switch_service", "/floor_manager/switch_floor");
  module.live_action = node.declare_parameter<std::string>(
    "live_floor_switch_action", "/floor_manager/floor_switch");
  module.live_timeout_sec = std::clamp(
    node.declare_parameter<double>("live_floor_switch_timeout_sec", 120.0),
    15.0,
    300.0);

  return result;
}

}  // namespace robot_api_server::features::floor_switch

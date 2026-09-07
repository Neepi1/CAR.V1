#pragma once

#include <chrono>
#include <filesystem>
#include <string>

#include "rclcpp/node.hpp"

#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"

namespace robot_api_server::features::floor_switch
{

// Values owned by maps, localization, and shared application composition.
struct FloorSwitchConfigurationInputs
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_map_context_file;
  std::string localization_health_topic{"/localization/floor_health"};
  std::chrono::nanoseconds service_timeout{std::chrono::seconds(8)};
};

struct FloorSwitchConfiguration
{
  FloorSwitchModuleConfig module;

  // Explicit projection into the read-only system-status module.
  std::string floor_status_topic{"/floor_manager/status"};
};

// Declares every API floor-switch-owned ROS parameter exactly once and emits
// the complete module configuration. It performs no switch, localization,
// filesystem mutation, safety hold, or ROS request.
class FloorSwitchConfigurationModule
{
public:
  static FloorSwitchConfiguration declare_parameters(
    rclcpp::Node & node,
    const FloorSwitchConfigurationInputs & inputs);
};

}  // namespace robot_api_server::features::floor_switch

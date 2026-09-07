#pragma once

#include <filesystem>

#include "rclcpp/node.hpp"

#include "robot_api_server/features/maps/maps_module.hpp"

namespace robot_api_server::features::maps
{

// Immutable paths shared with neighboring mapping, floor-switch, navigation,
// elevator, and status modules. They are declared once by the maps feature and
// projected by the composition root without a second parameter owner.
struct MapsRuntimePaths
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_maps_dir;
  std::filesystem::path runtime_map_context_file;
  std::filesystem::path last_navigation_map_file;
};

// Complete parameter result for the API-side maps vertical slice.
struct MapsConfiguration
{
  MapsModuleConfig module;
  MapsRuntimePaths runtime_paths;
};

// Declares all maps/keepout-owned ROS parameters exactly once and preserves
// the established defaults and timeout bounds. It performs no filesystem
// mutation, map activation, ROS service call, or runtime probe.
class MapsConfigurationModule
{
public:
  static MapsConfiguration declare_parameters(rclcpp::Node & node);
};

}  // namespace robot_api_server::features::maps

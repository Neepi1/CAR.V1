#include "robot_api_server/features/maps/maps_configuration_module.hpp"

#include <algorithm>
#include <string>

namespace robot_api_server::features::maps
{

MapsConfiguration MapsConfigurationModule::declare_parameters(rclcpp::Node & node)
{
  MapsConfiguration result;
  auto & module = result.module;
  auto & paths = result.runtime_paths;

  paths.maps_root = node.declare_parameter<std::string>(
    "maps_root", "/workspaces/njrh-v3/workspace1/maps_release");
  paths.runtime_maps_dir = node.declare_parameter<std::string>(
    "runtime_maps_dir",
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps");
  paths.runtime_map_context_file = node.declare_parameter<std::string>(
    "runtime_map_context_file", "/tmp/njrh_runtime_map_context.json");
  paths.last_navigation_map_file = node.declare_parameter<std::string>(
    "last_navigation_map_file",
    "/workspaces/njrh-v3/workspace1/maps_release/last_navigation_map.json");

  module.maps_root = paths.maps_root;
  module.runtime_maps_dir = paths.runtime_maps_dir;
  module.runtime_map_context_file = paths.runtime_map_context_file;
  module.keepout_mask_load_service = node.declare_parameter<std::string>(
    "keepout_mask_load_service", "/keepout_filter_mask_server/load_map");
  module.keepout_mask_state_service = node.declare_parameter<std::string>(
    "keepout_mask_state_service", "/keepout_filter_mask_server/get_state");
  module.keepout_filter_info_state_service = node.declare_parameter<std::string>(
    "keepout_filter_info_state_service", "/keepout_costmap_filter_info_server/get_state");
  module.keepout_mask_get_parameters_service = node.declare_parameter<std::string>(
    "keepout_mask_get_parameters_service", "/keepout_filter_mask_server/get_parameters");
  module.keepout_runtime_stage_root = node.declare_parameter<std::string>(
    "keepout_runtime_stage_root",
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/filters/runtime_nav2");
  module.keepout_mask_topic = node.declare_parameter<std::string>(
    "keepout_mask_topic", "/keepout_filter_mask");
  module.keepout_filter_info_topic = node.declare_parameter<std::string>(
    "keepout_filter_info_topic", "/costmap_filter_info/keepout");
  module.global_costmap_get_parameters_service = node.declare_parameter<std::string>(
    "global_costmap_get_parameters_service",
    "/global_costmap/global_costmap/get_parameters");
  module.global_costmap_clear_service = node.declare_parameter<std::string>(
    "global_costmap_clear_service", "/global_costmap/clear_entirely_global_costmap");
  module.global_costmap_topic = node.declare_parameter<std::string>(
    "global_costmap_topic", "/global_costmap/costmap");
  module.keepout_runtime_apply_timeout_sec = std::clamp(
    node.declare_parameter<double>("keepout_runtime_apply_timeout_sec", 5.0),
    0.5,
    15.0);

  return result;
}

}  // namespace robot_api_server::features::maps

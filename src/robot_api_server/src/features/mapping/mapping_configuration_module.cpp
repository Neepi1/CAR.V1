#include "robot_api_server/features/mapping/mapping_configuration_module.hpp"

#include <algorithm>
#include <string>

namespace robot_api_server::features::mapping
{

MappingModuleConfig MappingConfigurationModule::declare_parameters(
  rclcpp::Node & node,
  const MappingConfigurationInputs & inputs)
{
  MappingModuleConfig config;
  config.maps_root = inputs.maps_root;
  config.runtime_maps_dir = inputs.runtime_maps_dir;

  config.process.start_command = node.declare_parameter<std::string>(
    "mapping_2d_start_command",
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/"
    "run_projected_map.sh");
  config.process.log_file = node.declare_parameter<std::string>(
    "mapping_2d_log_file", "/tmp/njrh_mapping2d_slam_toolbox.log");
  config.process.lidar_rps_xps_state_dir = node.declare_parameter<std::string>(
    "mapping_lidar_rps_xps_state_dir", "/tmp/njrh_slam2d_lidar_rps_xps");
  config.process.graceful_stop_timeout_sec = std::clamp(
    node.declare_parameter<double>("mapping_graceful_stop_timeout_sec", 30.0),
    5.0,
    60.0);
  config.scan_owner_restore_timeout_sec = std::clamp(
    node.declare_parameter<double>("mapping_scan_owner_restore_timeout_sec", 30.0),
    5.0,
    60.0);
  config.scan_owner_topic = node.declare_parameter<std::string>(
    "mapping_scan_owner_topic", "/scan");
  config.navigation_scan_owner_node = node.declare_parameter<std::string>(
    "mapping_navigation_scan_owner_node", "pointcloud_accel_axis_node");
  config.resident_scan_control_service = node.declare_parameter<std::string>(
    "mapping_resident_scan_control_service",
    "/pointcloud_accel_axis_node/set_scan_output_enabled");
  config.live_map_topic = node.declare_parameter<std::string>(
    "mapping_2d_live_map_topic", "/map");
  config.live_map_max_age_sec = std::max(
    0.1,
    node.declare_parameter<double>("mapping_2d_live_map_max_age_sec", 3.0));

  return config;
}

}  // namespace robot_api_server::features::mapping

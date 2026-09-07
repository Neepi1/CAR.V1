#include "robot_api_server/features/localization/localization_configuration_module.hpp"

#include <algorithm>
#include <filesystem>

#include "robot_api_server/features/localization/tf_pose_utils.hpp"

namespace robot_api_server::features::localization
{

LocalizationConfiguration LocalizationConfigurationModule::declare_parameters(
  rclcpp::Node & node,
  const LocalizationConfigurationInputs & inputs)
{
  LocalizationConfiguration result;
  auto & module = result.module;
  auto & settle = result.settle;

  module.trigger_service = node.declare_parameter<std::string>(
    "localization_trigger_service", "/global_localization/trigger");
  module.result_topic = node.declare_parameter<std::string>(
    "localization_result_topic", "/localization_result");
  module.bridge_status_topic = node.declare_parameter<std::string>(
    "localization_bridge_status_topic", "/localization/bridge_status");
  result.floor_health_topic = node.declare_parameter<std::string>(
    "localization_floor_health_topic", "/localization/floor_health");

  module.amcl_runtime_status_file = std::filesystem::path(
    node.declare_parameter<std::string>(
      "amcl_runtime_status_file", "/tmp/njrh_amcl_runtime_status.env"));
  module.amcl_runtime_status_ttl_sec = std::max(
    0.0,
    node.declare_parameter<double>("amcl_runtime_status_ttl_sec", 5.0));

  module.tf_topic = node.declare_parameter<std::string>("tf_topic", "/tf");
  module.tf_static_topic = node.declare_parameter<std::string>(
    "tf_static_topic", "/tf_static");
  module.map_frame = normalized_frame_id(
    node.declare_parameter<std::string>("tf_map_frame", "map"));
  module.odom_frame = normalized_frame_id(
    node.declare_parameter<std::string>("tf_odom_frame", "odom"));
  module.base_frame = normalized_frame_id(
    node.declare_parameter<std::string>("tf_base_frame", "base_link"));
  module.static_lidar_frame = normalized_frame_id(
    node.declare_parameter<std::string>(
      "post_relocalization_static_lidar_frame", "lidar_level_link"));
  module.tf_pose_max_age_sec = std::max(
    0.1,
    node.declare_parameter<double>("tf_pose_max_age_sec", 2.0));
  module.robot_pose_freshness_sec = std::max(
    0.05,
    node.declare_parameter<double>("robot_pose_freshness_sec", 0.5));
  module.tf_chain_freshness_sec = std::max(
    0.05,
    node.declare_parameter<double>("tf_chain_freshness_sec", 0.30));
  module.tf_chain_settle_timeout_sec = std::max(
    module.tf_chain_freshness_sec,
    node.declare_parameter<double>("tf_chain_settle_timeout_sec", 2.0));

  module.service_timeout_sec = inputs.service_timeout_sec;
  module.trigger_service_timeout_sec = std::max(
    inputs.service_timeout_sec,
    node.declare_parameter<double>("localization_trigger_service_timeout_sec", 15.0));
  module.bridge_acceptance_timeout_sec = std::max(
    0.0,
    node.declare_parameter<double>("localization_bridge_acceptance_timeout_sec", 3.0));
  module.bridge_acceptance_max_distance_m = std::max(
    0.05,
    node.declare_parameter<double>(
      "localization_bridge_acceptance_max_distance_m", 1.0));
  module.bridge_acceptance_max_yaw_rad = std::max(
    0.01,
    node.declare_parameter<double>(
      "localization_bridge_acceptance_max_yaw_rad", 0.35));
  module.manual_amcl_refine_enabled = node.declare_parameter<bool>(
    "manual_relocalization_amcl_refine_enabled", true);
  module.manual_amcl_refine_required = node.declare_parameter<bool>(
    "manual_relocalization_amcl_refine_required", true);
  module.manual_amcl_refine_timeout_sec = std::clamp(
    node.declare_parameter<double>("manual_relocalization_amcl_refine_timeout_sec", 4.0),
    0.5,
    20.0);
  module.manual_amcl_refine_poll_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "manual_relocalization_amcl_refine_poll_ms", 100)),
    20,
    1000);
  module.manual_amcl_refine_request_period_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "manual_relocalization_amcl_refine_request_period_ms", 500)),
    100,
    5000);
  module.bridge_correction_pause_service = node.declare_parameter<std::string>(
    "localization_bridge_correction_pause_service",
    "/robot_localization_bridge/set_correction_paused");

  settle.enabled = node.declare_parameter<bool>(
    "post_relocalization_settle_enabled", true);
  settle.min_ms = std::max(
    0,
    static_cast<int>(node.declare_parameter<int>(
      "post_relocalization_settle_min_ms", 800)));
  settle.max_ms = std::max(
    settle.min_ms,
    static_cast<int>(node.declare_parameter<int>(
      "post_relocalization_settle_max_ms", 3000)));
  settle.stable_tf_samples = std::max(
    1,
    static_cast<int>(node.declare_parameter<int>(
      "post_relocalization_stable_tf_samples", 5)));
  settle.tf_sample_period_ms = std::max(
    20,
    static_cast<int>(node.declare_parameter<int>(
      "post_relocalization_tf_sample_period_ms", 100)));
  settle.zero_cmd = node.declare_parameter<bool>(
    "post_relocalization_zero_cmd", true);
  settle.require_local_costmap_update = node.declare_parameter<bool>(
    "post_relocalization_require_local_costmap_update", true);
  settle.required_local_costmap_updates = std::max(
    0,
    static_cast<int>(node.declare_parameter<int>(
      "post_relocalization_required_local_costmap_updates", 2)));
  settle.reject_if_new_message_filter_drop = node.declare_parameter<bool>(
    "post_relocalization_reject_if_new_message_filter_drop", true);
  settle.map_odom_publish_gap_warn_ms = std::max(
    1.0,
    node.declare_parameter<double>(
      "post_relocalization_map_odom_publish_gap_warn_ms", 100.0));
  settle.map_odom_publish_gap_fail_ms = std::max(
    settle.map_odom_publish_gap_warn_ms,
    node.declare_parameter<double>(
      "post_relocalization_map_odom_publish_gap_fail_ms", 250.0));

  settle.post_undock_enabled = node.declare_parameter<bool>(
    "post_undock_relocalization_settle_enabled", true);
  settle.post_undock_min_ms = std::max(
    0,
    static_cast<int>(node.declare_parameter<int>(
      "post_undock_relocalization_settle_min_ms", 800)));
  settle.post_undock_max_ms = std::max(
    settle.post_undock_min_ms,
    static_cast<int>(node.declare_parameter<int>(
      "post_undock_relocalization_settle_max_ms", 3000)));
  settle.post_undock_stable_tf_samples = std::max(
    1,
    static_cast<int>(node.declare_parameter<int>(
      "post_undock_stable_tf_samples", 5)));
  settle.post_undock_tf_sample_period_ms = std::max(
    20,
    static_cast<int>(node.declare_parameter<int>(
      "post_undock_tf_sample_period_ms", 100)));
  settle.post_undock_required_local_costmap_updates = std::max(
    0,
    static_cast<int>(node.declare_parameter<int>(
      "post_undock_required_local_costmap_updates", 2)));
  settle.post_undock_reject_if_new_message_filter_drop =
    node.declare_parameter<bool>(
    "post_undock_reject_if_new_message_filter_drop", true);
  settle.post_undock_zero_cmd_during_settle = node.declare_parameter<bool>(
    "post_undock_zero_cmd_during_settle", true);

  settle.large_correction_translation_m = std::max(
    0.0,
    node.declare_parameter<double>(
      "post_relocalization_large_correction_translation_m", 0.5));
  settle.large_correction_yaw_rad = std::max(
    0.0,
    node.declare_parameter<double>(
      "post_relocalization_large_correction_yaw_rad", 0.3));
  settle.large_correction_min_ms = std::max(
    settle.min_ms,
    static_cast<int>(node.declare_parameter<int>(
      "post_relocalization_large_correction_min_ms", 1500)));
  settle.tf_chain_freshness_sec = module.tf_chain_freshness_sec;
  settle.robot_pose_freshness_sec = module.robot_pose_freshness_sec;
  settle.base_frame = module.base_frame;
  settle.static_lidar_frame = module.static_lidar_frame;

  return result;
}

}  // namespace robot_api_server::features::localization

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/localization/localization_configuration_module.hpp"

namespace robot_api_server::features::localization
{
namespace
{

class LocalizationConfigurationModuleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(LocalizationConfigurationModuleTest, PreservesProductionDefaultsAndProjection)
{
  auto node = std::make_shared<rclcpp::Node>("localization_configuration_defaults_test");
  LocalizationConfigurationInputs inputs;
  inputs.service_timeout_sec = 8.0;

  const auto config = LocalizationConfigurationModule::declare_parameters(*node, inputs);

  EXPECT_EQ(config.module.trigger_service, "/global_localization/trigger");
  EXPECT_EQ(config.module.result_topic, "/localization_result");
  EXPECT_EQ(config.module.bridge_status_topic, "/localization/bridge_status");
  EXPECT_EQ(
    config.module.bridge_correction_pause_service,
    "/robot_localization_bridge/set_correction_paused");
  EXPECT_EQ(config.floor_health_topic, "/localization/floor_health");
  EXPECT_EQ(config.module.amcl_runtime_status_file, "/tmp/njrh_amcl_runtime_status.env");
  EXPECT_DOUBLE_EQ(config.module.amcl_runtime_status_ttl_sec, 5.0);
  EXPECT_EQ(config.module.tf_topic, "/tf");
  EXPECT_EQ(config.module.tf_static_topic, "/tf_static");
  EXPECT_EQ(config.module.map_frame, "map");
  EXPECT_EQ(config.module.odom_frame, "odom");
  EXPECT_EQ(config.module.base_frame, "base_link");
  EXPECT_EQ(config.module.static_lidar_frame, "lidar_level_link");
  EXPECT_DOUBLE_EQ(config.module.tf_pose_max_age_sec, 2.0);
  EXPECT_DOUBLE_EQ(config.module.robot_pose_freshness_sec, 0.5);
  EXPECT_DOUBLE_EQ(config.module.tf_chain_freshness_sec, 0.30);
  EXPECT_DOUBLE_EQ(config.module.tf_chain_settle_timeout_sec, 2.0);
  EXPECT_DOUBLE_EQ(config.module.service_timeout_sec, 8.0);
  EXPECT_DOUBLE_EQ(config.module.trigger_service_timeout_sec, 15.0);
  EXPECT_DOUBLE_EQ(config.module.bridge_acceptance_timeout_sec, 3.0);
  EXPECT_DOUBLE_EQ(config.module.bridge_acceptance_max_distance_m, 1.0);
  EXPECT_DOUBLE_EQ(config.module.bridge_acceptance_max_yaw_rad, 0.35);
  EXPECT_TRUE(config.module.manual_amcl_refine_enabled);
  EXPECT_TRUE(config.module.manual_amcl_refine_required);
  EXPECT_DOUBLE_EQ(config.module.manual_amcl_refine_timeout_sec, 4.0);
  EXPECT_EQ(config.module.manual_amcl_refine_poll_ms, 100);
  EXPECT_EQ(config.module.manual_amcl_refine_request_period_ms, 500);

  EXPECT_TRUE(config.settle.enabled);
  EXPECT_EQ(config.settle.min_ms, 800);
  EXPECT_EQ(config.settle.max_ms, 3000);
  EXPECT_EQ(config.settle.stable_tf_samples, 5);
  EXPECT_EQ(config.settle.tf_sample_period_ms, 100);
  EXPECT_TRUE(config.settle.zero_cmd);
  EXPECT_TRUE(config.settle.require_local_costmap_update);
  EXPECT_EQ(config.settle.required_local_costmap_updates, 2);
  EXPECT_TRUE(config.settle.reject_if_new_message_filter_drop);
  EXPECT_DOUBLE_EQ(config.settle.map_odom_publish_gap_warn_ms, 100.0);
  EXPECT_DOUBLE_EQ(config.settle.map_odom_publish_gap_fail_ms, 250.0);
  EXPECT_TRUE(config.settle.post_undock_enabled);
  EXPECT_EQ(config.settle.post_undock_min_ms, 800);
  EXPECT_EQ(config.settle.post_undock_max_ms, 3000);
  EXPECT_EQ(config.settle.post_undock_stable_tf_samples, 5);
  EXPECT_EQ(config.settle.post_undock_tf_sample_period_ms, 100);
  EXPECT_EQ(config.settle.post_undock_required_local_costmap_updates, 2);
  EXPECT_TRUE(config.settle.post_undock_reject_if_new_message_filter_drop);
  EXPECT_TRUE(config.settle.post_undock_zero_cmd_during_settle);
  EXPECT_DOUBLE_EQ(config.settle.large_correction_translation_m, 0.5);
  EXPECT_DOUBLE_EQ(config.settle.large_correction_yaw_rad, 0.3);
  EXPECT_EQ(config.settle.large_correction_min_ms, 1500);
  EXPECT_DOUBLE_EQ(
    config.settle.tf_chain_freshness_sec,
    config.module.tf_chain_freshness_sec);
  EXPECT_DOUBLE_EQ(
    config.settle.robot_pose_freshness_sec,
    config.module.robot_pose_freshness_sec);
  EXPECT_EQ(config.settle.base_frame, config.module.base_frame);
  EXPECT_EQ(config.settle.static_lidar_frame, config.module.static_lidar_frame);
}

TEST_F(LocalizationConfigurationModuleTest, AppliesLegacyNormalizationAndDependentClamps)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("amcl_runtime_status_ttl_sec", -1.0);
  options.append_parameter_override("tf_map_frame", "/test_map");
  options.append_parameter_override("tf_odom_frame", "//test_odom");
  options.append_parameter_override("tf_base_frame", "/test_base");
  options.append_parameter_override("post_relocalization_static_lidar_frame", "/test_lidar");
  options.append_parameter_override("tf_pose_max_age_sec", 0.01);
  options.append_parameter_override("robot_pose_freshness_sec", 0.01);
  options.append_parameter_override("tf_chain_freshness_sec", 0.01);
  options.append_parameter_override("tf_chain_settle_timeout_sec", 0.01);
  options.append_parameter_override("localization_trigger_service_timeout_sec", 1.0);
  options.append_parameter_override("localization_bridge_acceptance_timeout_sec", -1.0);
  options.append_parameter_override("localization_bridge_acceptance_max_distance_m", 0.01);
  options.append_parameter_override("localization_bridge_acceptance_max_yaw_rad", 0.001);
  options.append_parameter_override("manual_relocalization_amcl_refine_timeout_sec", 0.1);
  options.append_parameter_override("manual_relocalization_amcl_refine_poll_ms", 1);
  options.append_parameter_override("manual_relocalization_amcl_refine_request_period_ms", 10);
  options.append_parameter_override("post_relocalization_settle_min_ms", 900);
  options.append_parameter_override("post_relocalization_settle_max_ms", 100);
  options.append_parameter_override("post_relocalization_stable_tf_samples", 0);
  options.append_parameter_override("post_relocalization_tf_sample_period_ms", 1);
  options.append_parameter_override("post_relocalization_required_local_costmap_updates", -1);
  options.append_parameter_override("post_relocalization_map_odom_publish_gap_warn_ms", 0.5);
  options.append_parameter_override("post_relocalization_map_odom_publish_gap_fail_ms", 0.1);
  options.append_parameter_override("post_undock_relocalization_settle_min_ms", 700);
  options.append_parameter_override("post_undock_relocalization_settle_max_ms", 100);
  options.append_parameter_override("post_undock_stable_tf_samples", 0);
  options.append_parameter_override("post_undock_tf_sample_period_ms", 1);
  options.append_parameter_override("post_undock_required_local_costmap_updates", -1);
  options.append_parameter_override("post_relocalization_large_correction_translation_m", -1.0);
  options.append_parameter_override("post_relocalization_large_correction_yaw_rad", -1.0);
  options.append_parameter_override("post_relocalization_large_correction_min_ms", 100);
  auto node = std::make_shared<rclcpp::Node>(
    "localization_configuration_clamps_test", options);
  LocalizationConfigurationInputs inputs;
  inputs.service_timeout_sec = 8.0;

  const auto config = LocalizationConfigurationModule::declare_parameters(*node, inputs);

  EXPECT_DOUBLE_EQ(config.module.amcl_runtime_status_ttl_sec, 0.0);
  EXPECT_EQ(config.module.map_frame, "test_map");
  EXPECT_EQ(config.module.odom_frame, "test_odom");
  EXPECT_EQ(config.module.base_frame, "test_base");
  EXPECT_EQ(config.module.static_lidar_frame, "test_lidar");
  EXPECT_DOUBLE_EQ(config.module.tf_pose_max_age_sec, 0.1);
  EXPECT_DOUBLE_EQ(config.module.robot_pose_freshness_sec, 0.05);
  EXPECT_DOUBLE_EQ(config.module.tf_chain_freshness_sec, 0.05);
  EXPECT_DOUBLE_EQ(config.module.tf_chain_settle_timeout_sec, 0.05);
  EXPECT_DOUBLE_EQ(config.module.trigger_service_timeout_sec, 8.0);
  EXPECT_DOUBLE_EQ(config.module.bridge_acceptance_timeout_sec, 0.0);
  EXPECT_DOUBLE_EQ(config.module.bridge_acceptance_max_distance_m, 0.05);
  EXPECT_DOUBLE_EQ(config.module.bridge_acceptance_max_yaw_rad, 0.01);
  EXPECT_DOUBLE_EQ(config.module.manual_amcl_refine_timeout_sec, 0.5);
  EXPECT_EQ(config.module.manual_amcl_refine_poll_ms, 20);
  EXPECT_EQ(config.module.manual_amcl_refine_request_period_ms, 100);
  EXPECT_EQ(config.settle.min_ms, 900);
  EXPECT_EQ(config.settle.max_ms, 900);
  EXPECT_EQ(config.settle.stable_tf_samples, 1);
  EXPECT_EQ(config.settle.tf_sample_period_ms, 20);
  EXPECT_EQ(config.settle.required_local_costmap_updates, 0);
  EXPECT_DOUBLE_EQ(config.settle.map_odom_publish_gap_warn_ms, 1.0);
  EXPECT_DOUBLE_EQ(config.settle.map_odom_publish_gap_fail_ms, 1.0);
  EXPECT_EQ(config.settle.post_undock_min_ms, 700);
  EXPECT_EQ(config.settle.post_undock_max_ms, 700);
  EXPECT_EQ(config.settle.post_undock_stable_tf_samples, 1);
  EXPECT_EQ(config.settle.post_undock_tf_sample_period_ms, 20);
  EXPECT_EQ(config.settle.post_undock_required_local_costmap_updates, 0);
  EXPECT_DOUBLE_EQ(config.settle.large_correction_translation_m, 0.0);
  EXPECT_DOUBLE_EQ(config.settle.large_correction_yaw_rad, 0.0);
  EXPECT_EQ(config.settle.large_correction_min_ms, 900);
}

}  // namespace
}  // namespace robot_api_server::features::localization

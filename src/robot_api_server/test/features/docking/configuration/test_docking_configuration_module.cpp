#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/docking/configuration/docking_configuration_module.hpp"

namespace robot_api_server::features::docking::configuration
{
namespace
{

class DockingConfigurationModuleTest : public ::testing::Test
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

DockingConfigurationInputs test_inputs()
{
  DockingConfigurationInputs inputs;
  inputs.service_timeout_sec = 8.0;
  inputs.navigate_to_pose_action = "/test_navigate_to_pose";
  inputs.map_frame = "test_map";
  inputs.robot_pose_freshness_sec = 0.42;
  inputs.charging_current_min_a = 0.17;
  inputs.full_soc_threshold_pct = 98.5;
  return inputs;
}

TEST_F(DockingConfigurationModuleTest, PreservesProductionDefaultsAndCrossModuleProjection)
{
  auto node = std::make_shared<rclcpp::Node>("docking_configuration_defaults_test");
  const auto config = DockingConfigurationModule::declare_parameters(*node, test_inputs());

  EXPECT_EQ(config.runtime.start_service, "/docking/start");
  EXPECT_EQ(config.runtime.command_topic, "/cmd_vel_docking");
  EXPECT_EQ(config.runtime.forced_mode_topic, "/ranger_mini3/forced_mode");
  EXPECT_DOUBLE_EQ(config.runtime.stop_service_wait_sec, 3.0);
  EXPECT_DOUBLE_EQ(config.http.pre_dock_distance_m, 0.60);
  EXPECT_EQ(config.http.default_dock_profile_id, "gs2_rear_charging_dock");
  EXPECT_DOUBLE_EQ(config.contact_interlock.charging_current_min_a, 0.17);
  EXPECT_DOUBLE_EQ(config.contact_interlock.full_soc_threshold_pct, 98.5);
  EXPECT_DOUBLE_EQ(config.alignment_policy.pose_max_distance_m, 0.35);
  EXPECT_DOUBLE_EQ(config.alignment_policy.yaw_tolerance_rad, 0.0698);
  EXPECT_DOUBLE_EQ(config.predock_control.yaw_align_tolerance_rad, 0.0698);
  EXPECT_DOUBLE_EQ(config.predock_control.lateral_align_target_m, 0.03);
  EXPECT_DOUBLE_EQ(config.predock_control.fine_entry_max_yaw_rad, 0.0349);
  EXPECT_EQ(config.predock_control.observation_backend, "target_observation");
  EXPECT_EQ(config.predock_control.map_frame, "test_map");
  EXPECT_DOUBLE_EQ(config.predock_control.robot_pose_freshness_sec, 0.42);

  EXPECT_EQ(config.job_executor.navigate_to_pose_action, "/test_navigate_to_pose");
  EXPECT_DOUBLE_EQ(config.job_executor.predock_nav_timeout_sec, 180.0);
  EXPECT_DOUBLE_EQ(config.job_execution.navigation_start_wait_sec, 45.0);
  EXPECT_TRUE(config.pre_navigation_undock.relocalize_after_success);
  EXPECT_DOUBLE_EQ(config.pre_navigation_undock.auto_undock_timeout_sec, 28.0);
  EXPECT_FALSE(config.status.relocalize_after_fine_docking);
  EXPECT_EQ(config.status.observation_backend, config.predock_control.observation_backend);
  EXPECT_EQ(
    config.correction_pause.enabled,
    config.predock_control.pause_global_correction_during_fine);
  EXPECT_EQ(config.correction_pause.service_timeout, std::chrono::seconds(8));
}

TEST_F(DockingConfigurationModuleTest, AppliesTheLegacyClampAndDependencyRules)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("dock_contact_latch_bms_ttl_sec", -5.0);
  options.append_parameter_override("docking_navigation_start_wait_sec", 1.0);
  options.append_parameter_override("docking_predock_nav_timeout_sec", -1.0);
  options.append_parameter_override("docking_manual_predock_min_distance_m", 10.0);
  options.append_parameter_override("docking_manual_predock_max_distance_m", 0.0);
  options.append_parameter_override("predock_yaw_align_tolerance_rad", 1.0);
  options.append_parameter_override("predock_yaw_align_trigger_rad", 0.01);
  options.append_parameter_override("predock_yaw_align_hard_fail_rad", 0.10);
  options.append_parameter_override("predock_lateral_align_target_m", 0.30);
  options.append_parameter_override("predock_lateral_align_trigger_m", 0.01);
  options.append_parameter_override("predock_lateral_align_max_correction_m", 0.02);
  options.append_parameter_override("predock_lateral_align_command_sign", 0.0);
  options.append_parameter_override("fine_docking_entry_max_distance_m", -1.0);
  options.append_parameter_override("docking_max_retries", 9);
  options.append_parameter_override("docking_fine_bridge_smoothing_wait_timeout_ms", 70000);
  auto node = std::make_shared<rclcpp::Node>("docking_configuration_clamps_test", options);

  const auto config = DockingConfigurationModule::declare_parameters(*node, test_inputs());

  EXPECT_DOUBLE_EQ(config.contact_interlock.bms_ttl_sec, 0.0);
  EXPECT_DOUBLE_EQ(config.job_execution.navigation_start_wait_sec, 8.0);
  EXPECT_DOUBLE_EQ(config.job_executor.predock_nav_timeout_sec, 5.0);
  EXPECT_DOUBLE_EQ(config.predock_pose_resolver.min_distance_m, 5.0);
  EXPECT_DOUBLE_EQ(config.predock_pose_resolver.max_distance_m, 5.05);
  EXPECT_DOUBLE_EQ(config.alignment_policy.yaw_tolerance_rad, 0.35);
  EXPECT_DOUBLE_EQ(config.alignment_policy.yaw_hard_fail_rad, 0.35);
  EXPECT_DOUBLE_EQ(config.alignment_policy.lateral_target_m, 0.20);
  EXPECT_DOUBLE_EQ(config.alignment_policy.lateral_max_correction_m, 0.20);
  EXPECT_DOUBLE_EQ(config.predock_control.lateral_align_command_sign, -1.0);
  EXPECT_DOUBLE_EQ(config.predock_control.fine_entry_max_distance_m, 0.02);
  EXPECT_EQ(config.http.max_retries, 5);
  EXPECT_EQ(config.predock_control.fine_bridge_smoothing_wait_timeout_ms, 60000);
}

}  // namespace
}  // namespace robot_api_server::features::docking::configuration

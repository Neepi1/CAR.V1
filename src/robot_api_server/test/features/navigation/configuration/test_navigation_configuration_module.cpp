#include <chrono>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/navigation/configuration/navigation_configuration_module.hpp"

namespace robot_api_server::features::navigation::configuration
{
namespace
{

class NavigationConfigurationModuleTest : public ::testing::Test
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

NavigationConfigurationInputs test_inputs()
{
  NavigationConfigurationInputs inputs;
  inputs.service_timeout_sec = 8.0;
  inputs.navigate_to_pose_action = "/test_navigate_to_pose";
  inputs.navigate_to_pose_status_topic = "/test_navigate_to_pose/_action/status";
  inputs.maps_root = "/test/maps";
  inputs.runtime_map_context_file = "/test/runtime_map_context.json";
  inputs.map_frame = "test_map";
  inputs.base_frame = "test_base";
  inputs.robot_pose_freshness_sec = 0.42;
  inputs.lateral_divergence_epsilon_m = 0.017;
  inputs.lateral_divergence_count = 4;
  inputs.lateral_forced_mode = "test_side_slip";
  inputs.lateral_release_mode = "test_auto";
  return inputs;
}

TEST_F(NavigationConfigurationModuleTest, PreservesProductionDefaultsAndSubmoduleProjection)
{
  auto node = std::make_shared<rclcpp::Node>("navigation_configuration_defaults_test");
  const auto config = NavigationConfigurationModule::declare_parameters(*node, test_inputs());

  EXPECT_EQ(config.module.action.action_name, "/test_navigate_to_pose");
  EXPECT_EQ(
    config.module.action.status_topic,
    "/test_navigate_to_pose/_action/status");
  EXPECT_EQ(config.module.action.operation_timeout, std::chrono::seconds(8));
  EXPECT_EQ(config.module.maps_root, "/test/maps");
  EXPECT_EQ(config.module.runtime_map_context_file, "/test/runtime_map_context.json");
  EXPECT_DOUBLE_EQ(config.module.resume_starting_context_ttl_sec, 300.0);
  EXPECT_DOUBLE_EQ(config.module.completion_policy.position_tolerance_m, 0.06);
  EXPECT_DOUBLE_EQ(config.module.completion_policy.yaw_tolerance_rad, 0.05);
  EXPECT_DOUBLE_EQ(config.module.completion_policy.yaw_align_trigger_rad, 0.08);
  EXPECT_EQ(config.module.goal_policy.default_completion_policy, "pose_required");
  EXPECT_EQ(config.module.goal_policy.position_only_nav2_yaw_mode, "approach_heading");
  EXPECT_TRUE(config.module.nav2_native_goal_completion_enabled);
  EXPECT_FALSE(config.module.api_final_yaw_align_fallback_enabled);
  EXPECT_FALSE(config.module.navigation_final_yaw_align_enabled);

  EXPECT_EQ(config.terminal_runtime.command_topic, "/cmd_vel_api");
  EXPECT_EQ(config.terminal_runtime.speed_limit_topic, "/speed_limit");
  EXPECT_EQ(config.terminal_runtime.reverse_enable_topic, "/ranger_mini3/allow_reverse");
  EXPECT_EQ(config.terminal_runtime.mode_controller_status_topic, "/ranger_base/status");
  EXPECT_EQ(config.terminal_runtime.local_costmap_topic, "/local_costmap/costmap");
  EXPECT_EQ(config.terminal_runtime.map_frame, "test_map");
  EXPECT_EQ(config.terminal_runtime.base_frame, "test_base");
  EXPECT_DOUBLE_EQ(config.terminal_runtime.robot_pose_freshness_sec, 0.42);

  EXPECT_DOUBLE_EQ(config.module.terminal_control.goal_position_success_tolerance_m, 0.06);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.lateral_target_m, 0.03);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.lateral_divergence_epsilon_m, 0.017);
  EXPECT_EQ(config.module.terminal_control.lateral_divergence_count, 4);
  EXPECT_EQ(config.module.terminal_control.lateral_forced_mode, "test_side_slip");
  EXPECT_EQ(config.module.terminal_control.lateral_release_mode, "test_auto");
  EXPECT_DOUBLE_EQ(config.module.terminal_control.final_yaw_max_speed_radps, 0.60);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.yaw_stop_lead_time_sec, 0.125);

  EXPECT_EQ(config.goal_execution.action_name, "/test_navigate_to_pose");
  EXPECT_DOUBLE_EQ(config.goal_execution.goal_result_timeout_sec, 600.0);
  EXPECT_FALSE(config.goal_execution.near_goal_stalled_handoff_enabled);
  EXPECT_DOUBLE_EQ(config.goal_executor.navigation_goal_position_success_tolerance_m, 0.06);
  EXPECT_FALSE(config.goal_executor.navigation_final_yaw_align_enabled);
  EXPECT_EQ(config.amcl_nomotion_update_service, "/request_nomotion_update");
}

TEST_F(NavigationConfigurationModuleTest, AppliesLegacyClampDependencyAndFallbackRules)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("navigation_cancel_action_wait_sec", 50.0);
  options.append_parameter_override("navigation_goal_result_timeout_sec", 1.0);
  options.append_parameter_override("navigation_goal_position_success_tolerance_m", 0.001);
  options.append_parameter_override("navigation_terminal_speed_limit_far_distance_m", 0.1);
  options.append_parameter_override("navigation_terminal_speed_limit_mid_distance_m", 9.0);
  options.append_parameter_override("navigation_terminal_speed_limit_near_distance_m", 9.0);
  options.append_parameter_override("navigation_terminal_speed_limit_crawl_distance_m", 9.0);
  options.append_parameter_override("navigation_default_goal_completion_policy", "dock_staging");
  options.append_parameter_override(
    "navigation_delivery_point_goal_completion_policy", "invalid");
  options.append_parameter_override("navigation_position_only_nav2_yaw_mode", "invalid");
  options.append_parameter_override("api_final_yaw_align_fallback_enabled", false);
  options.append_parameter_override("navigation_final_yaw_align_enable", true);
  options.append_parameter_override("navigation_final_yaw_tolerance_rad", 0.20);
  options.append_parameter_override("navigation_final_yaw_align_trigger_rad", 0.01);
  options.append_parameter_override("navigation_final_yaw_align_cmd_topic", "/unsafe_cmd");
  options.append_parameter_override(
    "navigation_final_yaw_align_bypass_collision_monitor", false);
  options.append_parameter_override(
    "post_nav2_final_verify_terminal_lateral_command_sign", 0.0);
  options.append_parameter_override("post_nav2_final_verify_xy_retry_min_error_m", 0.30);
  options.append_parameter_override("navigation_terminal_recovery_max_distance_m", 0.10);
  auto node = std::make_shared<rclcpp::Node>(
    "navigation_configuration_clamps_test", options);

  const auto config = NavigationConfigurationModule::declare_parameters(*node, test_inputs());

  EXPECT_EQ(config.module.cancel_action_wait, std::chrono::seconds(8));
  EXPECT_DOUBLE_EQ(config.goal_execution.goal_result_timeout_sec, 5.0);
  EXPECT_DOUBLE_EQ(config.module.completion_policy.position_tolerance_m, 0.05);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.speed_limit_far_distance_m, 0.5);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.speed_limit_mid_distance_m, 0.5);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.speed_limit_near_distance_m, 0.5);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.speed_limit_crawl_distance_m, 0.5);
  EXPECT_EQ(config.module.goal_policy.default_completion_policy, "pose_required");
  EXPECT_EQ(config.module.goal_policy.delivery_point_completion_policy, "pose_required");
  EXPECT_EQ(config.module.goal_policy.position_only_nav2_yaw_mode, "approach_heading");
  EXPECT_FALSE(config.module.navigation_final_yaw_align_enabled);
  EXPECT_DOUBLE_EQ(config.module.completion_policy.yaw_align_trigger_rad, 0.20);
  EXPECT_EQ(config.terminal_runtime.command_topic, "/cmd_vel_api");
  EXPECT_TRUE(config.module.navigation_final_yaw_align_bypass_collision_monitor);
  EXPECT_DOUBLE_EQ(config.module.terminal_control.lateral_command_sign, 1.0);
  EXPECT_DOUBLE_EQ(config.module.completion_policy.terminal_recovery_max_distance_m, 0.30);
}

}  // namespace
}  // namespace robot_api_server::features::navigation::configuration

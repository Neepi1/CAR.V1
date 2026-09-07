#include "robot_api_server/features/navigation/configuration/navigation_configuration_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace robot_api_server::features::navigation::configuration
{

namespace
{

std::chrono::nanoseconds seconds(const double value)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(value));
}

}  // namespace

NavigationConfiguration NavigationConfigurationModule::declare_parameters(
  rclcpp::Node & node,
  const NavigationConfigurationInputs & inputs)
{
  NavigationConfiguration config;
  auto & module = config.module;
  auto & goal_policy = module.goal_policy;
  auto & completion = module.completion_policy;
  auto & bridge_wait = module.bridge_wait;
  auto & terminal = module.terminal_control;
  auto & terminal_runtime = config.terminal_runtime;
  auto & goal_execution = config.goal_execution;
  auto & goal_executor = config.goal_executor;

  module.action.action_name = inputs.navigate_to_pose_action;
  module.action.status_topic = inputs.navigate_to_pose_status_topic;
  module.action.operation_timeout = seconds(inputs.service_timeout_sec);
  module.process.resume_command = node.declare_parameter<std::string>(
    "navigation_resume_command",
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/"
    "run_navigation_runtime_services.sh");
  module.process.resume_log_file = node.declare_parameter<std::string>(
    "navigation_resume_log_file",
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/web_dashboard/"
    "runtime_logs/resident_navigation_runtime.log");
  module.process.stop_command = node.declare_parameter<std::string>(
    "navigation_stop_command",
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/"
    "stop_floor_navigation.sh");
  module.process.stop_log_file = node.declare_parameter<std::string>(
    "navigation_stop_log_file", "/tmp/njrh_navigation_stop.log");
  module.resume_starting_context_ttl_sec = std::clamp(
    node.declare_parameter<double>("navigation_resume_starting_context_ttl_sec", 300.0),
    10.0,
    900.0);
  module.cancel_action_wait = seconds(
    std::clamp(
      node.declare_parameter<double>("navigation_cancel_action_wait_sec", 0.75),
      0.05,
      inputs.service_timeout_sec));

  // Kept declared for deployed YAML compatibility. Ordinary navigation no
  // longer triggers global relocalization in its normal goal path.
  (void)node.declare_parameter<bool>("navigation_relocalize_before_goal", true);
  (void)node.declare_parameter<bool>("navigation_relocalize_before_goal_always", false);
  (void)node.declare_parameter<bool>("navigation_relocalize_before_goal_required", true);
  (void)std::max(
    0.5,
    node.declare_parameter<double>("navigation_relocalize_wait_sec", 8.0));

  const double goal_result_timeout_sec = std::max(
    5.0,
    node.declare_parameter<double>("navigation_goal_result_timeout_sec", 600.0));
  const double goal_position_success_tolerance_m = std::clamp(
    node.declare_parameter<double>("navigation_goal_position_success_tolerance_m", 0.06),
    0.05,
    1.0);

  terminal_runtime.speed_limit_enabled = node.declare_parameter<bool>(
    "navigation_terminal_speed_limit_enabled", true);
  terminal_runtime.speed_limit_topic = node.declare_parameter<std::string>(
    "navigation_terminal_speed_limit_topic", "/speed_limit");
  terminal.speed_limit_far_distance_m = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_far_distance_m", 2.0),
    0.5,
    10.0);
  terminal.speed_limit_mid_distance_m = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_mid_distance_m", 1.2),
    0.25,
    terminal.speed_limit_far_distance_m);
  terminal.speed_limit_near_distance_m = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_near_distance_m", 0.6),
    0.10,
    terminal.speed_limit_mid_distance_m);
  terminal.speed_limit_crawl_distance_m = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_crawl_distance_m", 0.15),
    0.05,
    terminal.speed_limit_near_distance_m);
  terminal.speed_limit_far_mps = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_far_mps", 1.20),
    0.05,
    1.50);
  terminal.speed_limit_mid_mps = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_mid_mps", 0.70),
    0.05,
    terminal.speed_limit_far_mps);
  terminal.speed_limit_near_mps = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_near_mps", 0.40),
    0.05,
    terminal.speed_limit_mid_mps);
  terminal.speed_limit_crawl_mps = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_crawl_mps", 0.25),
    0.03,
    terminal.speed_limit_near_mps);
  terminal.speed_limit_final_mps = std::clamp(
    node.declare_parameter<double>("navigation_terminal_speed_limit_final_mps", 0.10),
    0.02,
    terminal.speed_limit_crawl_mps);
  terminal_runtime.speed_limit_far_mps = terminal.speed_limit_far_mps;

  goal_policy.default_completion_policy = normalize_navigation_goal_completion_policy(
    node.declare_parameter<std::string>(
      "navigation_default_goal_completion_policy", "pose_required"));
  if (!is_valid_navigation_goal_completion_policy(goal_policy.default_completion_policy) ||
    goal_policy.default_completion_policy == "dock_staging")
  {
    RCLCPP_WARN(
      node.get_logger(),
      "navigation_default_goal_completion_policy=%s is not valid for normal navigation; "
      "using pose_required",
      goal_policy.default_completion_policy.c_str());
    goal_policy.default_completion_policy = "pose_required";
  }
  goal_policy.delivery_point_completion_policy = normalize_navigation_goal_completion_policy(
    node.declare_parameter<std::string>(
      "navigation_delivery_point_goal_completion_policy", "pose_required"));
  if (!is_valid_navigation_goal_completion_policy(
      goal_policy.delivery_point_completion_policy) ||
    goal_policy.delivery_point_completion_policy == "dock_staging")
  {
    RCLCPP_WARN(
      node.get_logger(),
      "navigation_delivery_point_goal_completion_policy=%s is not valid for delivery "
      "navigation; using pose_required",
      goal_policy.delivery_point_completion_policy.c_str());
    goal_policy.delivery_point_completion_policy = "pose_required";
  }
  goal_policy.position_only_nav2_yaw_mode = normalize_position_only_nav2_yaw_mode(
    node.declare_parameter<std::string>(
      "navigation_position_only_nav2_yaw_mode", "approach_heading"));
  if (!is_valid_position_only_nav2_yaw_mode(goal_policy.position_only_nav2_yaw_mode)) {
    RCLCPP_WARN(
      node.get_logger(),
      "navigation_position_only_nav2_yaw_mode=%s is not valid; using approach_heading",
      goal_policy.position_only_nav2_yaw_mode.c_str());
    goal_policy.position_only_nav2_yaw_mode = "approach_heading";
  }
  goal_policy.position_only_approach_heading_min_distance_m = std::clamp(
    node.declare_parameter<double>(
      "navigation_position_only_approach_heading_min_distance_m", 0.20),
    0.01,
    2.0);

  module.nav2_native_goal_completion_enabled = node.declare_parameter<bool>(
    "nav2_native_goal_completion_enabled", true);
  module.nav2_rotation_shim_enabled = node.declare_parameter<bool>(
    "nav2_rotation_shim_enabled", true);
  module.api_final_yaw_align_fallback_enabled = node.declare_parameter<bool>(
    "api_final_yaw_align_fallback_enabled", false);
  const bool final_verify_enabled = node.declare_parameter<bool>(
    "post_nav2_final_verify_enabled", true);
  const bool final_verify_wait_bridge_smoothing = node.declare_parameter<bool>(
    "post_nav2_final_verify_wait_bridge_smoothing", true);
  const int final_verify_bridge_wait_timeout_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "post_nav2_final_verify_bridge_wait_timeout_ms", 2000)),
    100,
    10000);
  const int final_verify_bridge_wait_sample_period_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "post_nav2_final_verify_bridge_wait_sample_period_ms", 100)),
    20,
    1000);
  const bool final_verify_request_amcl_nomotion_update = node.declare_parameter<bool>(
    "post_nav2_final_verify_request_amcl_nomotion_update", true);
  config.amcl_nomotion_update_service = node.declare_parameter<std::string>(
    "post_nav2_final_verify_amcl_nomotion_update_service",
    "/request_nomotion_update");
  const int final_verify_max_retry_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "post_nav2_final_verify_max_retry_count", 3)),
    0,
    3);
  const double final_verify_acceptance_slack_m = std::clamp(
    node.declare_parameter<double>("post_nav2_final_verify_acceptance_slack_m", 0.02),
    0.0,
    0.20);
  const double final_verify_xy_retry_min_error_m = std::clamp(
    node.declare_parameter<double>("post_nav2_final_verify_xy_retry_min_error_m", 0.06),
    0.0,
    2.0);
  const double terminal_recovery_max_distance_m = std::clamp(
    node.declare_parameter<double>("navigation_terminal_recovery_max_distance_m", 0.40),
    final_verify_xy_retry_min_error_m,
    0.60);
  const bool final_verify_yaw_retry_if_failed = node.declare_parameter<bool>(
    "post_nav2_final_verify_yaw_retry_if_failed", true);
  const bool final_verify_retry_uses_same_nav2_goal = node.declare_parameter<bool>(
    "post_nav2_final_verify_retry_uses_same_nav2_goal", true);
  const bool final_verify_api_velocity_correction_enabled = node.declare_parameter<bool>(
    "post_nav2_final_verify_api_velocity_correction_enabled", true);
  terminal_runtime.post_nav2_reverse_permit_enabled = node.declare_parameter<bool>(
    "post_nav2_final_verify_reverse_permit_enabled", true);
  terminal_runtime.reverse_enable_topic = node.declare_parameter<std::string>(
    "post_nav2_final_verify_reverse_enable_topic", "/ranger_mini3/allow_reverse");
  terminal_runtime.navigation_reverse_permit_enabled = node.declare_parameter<bool>(
    "navigation_terminal_reverse_permit_enabled", true);
  terminal_runtime.reverse_permit_enter_distance_m = std::clamp(
    node.declare_parameter<double>(
      "navigation_terminal_reverse_permit_enter_distance_m", 0.30),
    goal_position_success_tolerance_m,
    1.0);
  terminal_runtime.reverse_permit_exit_distance_m = std::clamp(
    node.declare_parameter<double>(
      "navigation_terminal_reverse_permit_exit_distance_m", 0.35),
    terminal_runtime.reverse_permit_enter_distance_m,
    1.5);
  terminal_runtime.reverse_permit_refresh_period_sec = std::clamp(
    node.declare_parameter<double>(
      "navigation_terminal_reverse_permit_refresh_period_sec", 0.20),
    0.05,
    0.50);

  const bool terminal_lateral_correction_enabled = node.declare_parameter<bool>(
    "post_nav2_final_verify_terminal_lateral_correction_enabled", true);
  const double terminal_lateral_target_m = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_lateral_target_m", 0.03),
    0.01,
    goal_position_success_tolerance_m);
  const double terminal_lateral_trigger_m = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_lateral_trigger_m", 0.04),
    terminal_lateral_target_m,
    terminal_recovery_max_distance_m);
  const double terminal_lateral_max_forward_m = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_lateral_max_forward_m", 0.15),
    0.01,
    terminal_recovery_max_distance_m);
  const double terminal_lateral_speed_mps = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_lateral_speed_mps", 0.04),
    0.005,
    0.15);
  const double terminal_lateral_kp = std::clamp(
    node.declare_parameter<double>("post_nav2_final_verify_terminal_lateral_kp", 0.8),
    0.05,
    5.0);
  const double terminal_lateral_timeout_sec = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_lateral_timeout_sec", 20.0),
    0.5,
    30.0);
  const bool costmap_guard_enabled = node.declare_parameter<bool>(
    "navigation_terminal_recovery_costmap_guard_enabled", true);
  const double costmap_max_age_sec = std::clamp(
    node.declare_parameter<double>(
      "navigation_terminal_recovery_costmap_max_age_sec", 0.50),
    0.05,
    2.0);
  const int costmap_occupied_threshold = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "navigation_terminal_recovery_costmap_occupied_threshold", 50)),
    1,
    100);
  const double costmap_lookahead_m = std::clamp(
    node.declare_parameter<double>(
      "navigation_terminal_recovery_costmap_lookahead_m", 0.15),
    0.05,
    terminal_recovery_max_distance_m);
  double terminal_lateral_command_sign = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_lateral_command_sign", 1.0),
    -1.0,
    1.0);
  if (std::fabs(terminal_lateral_command_sign) < 0.5) {
    terminal_lateral_command_sign = 1.0;
  }

  terminal_runtime.terminal_settle_enabled = node.declare_parameter<bool>(
    "post_nav2_final_verify_terminal_settle_enabled", true);
  terminal_runtime.terminal_settle_linear_speed_threshold_mps = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_settle_linear_speed_threshold_mps", 0.01),
    0.0,
    0.10);
  terminal_runtime.terminal_settle_angular_speed_threshold_radps = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_settle_angular_speed_threshold_radps", 0.02),
    0.0,
    0.20);
  terminal_runtime.terminal_settle_stable_duration_sec = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_settle_stable_duration_sec", 0.30),
    0.05,
    2.0);
  terminal_runtime.terminal_settle_timeout_sec = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_settle_timeout_sec", 2.50),
    0.10,
    5.0);
  terminal_runtime.terminal_settle_odom_max_age_sec = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_settle_odom_max_age_sec", 0.20),
    0.02,
    1.0);
  terminal_runtime.terminal_settle_require_dual_ackermann_mode =
    node.declare_parameter<bool>(
    "post_nav2_final_verify_terminal_settle_require_dual_ackermann_mode", true);
  terminal_runtime.terminal_settle_mode_status_max_age_sec = std::clamp(
    node.declare_parameter<double>(
      "post_nav2_final_verify_terminal_settle_mode_status_max_age_sec", 0.50),
    0.05,
    2.0);
  terminal.terminal_settle_max_recheck_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "post_nav2_final_verify_terminal_settle_max_recheck_count", 1)),
    0,
    3);

  const bool near_goal_stalled_handoff_enabled = node.declare_parameter<bool>(
    "navigation_near_goal_stalled_handoff_enabled", false);
  const double near_goal_stalled_handoff_min_wait_sec = std::clamp(
    node.declare_parameter<double>(
      "navigation_near_goal_stalled_handoff_min_wait_sec", 3.0),
    1.0,
    goal_result_timeout_sec);
  const double near_goal_stalled_handoff_stall_sec = std::clamp(
    node.declare_parameter<double>(
      "navigation_near_goal_stalled_handoff_stall_sec", 1.5),
    0.5,
    goal_result_timeout_sec);
  const double near_goal_stalled_handoff_improvement_epsilon_m = std::clamp(
    node.declare_parameter<double>(
      "navigation_near_goal_stalled_handoff_improvement_epsilon_m", 0.02),
    0.001,
    0.10);
  const bool nav2_failed_near_goal_retry_enabled = node.declare_parameter<bool>(
    "navigation_nav2_failed_near_goal_retry_enabled", true);
  const int nav2_failed_near_goal_retry_max_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "navigation_nav2_failed_near_goal_retry_max_count", 1)),
    0,
    3);
  const bool nav2_failed_near_goal_retry_requires_yaw_error =
    node.declare_parameter<bool>(
    "navigation_nav2_failed_near_goal_retry_requires_yaw_error", false);
  // Retained as a declared no-op while deployed YAML is migrated; the
  // executor's bounded retry count is the current source of truth.
  (void)std::clamp(
    node.declare_parameter<int>("navigation_max_reposition_after_yaw_retry", 1),
    0L,
    3L);
  const double reposition_after_yaw_drift_timeout_sec = std::clamp(
    node.declare_parameter<double>(
      "navigation_reposition_after_yaw_drift_timeout_sec", 30.0),
    5.0,
    goal_result_timeout_sec);

  bool final_yaw_align_enabled = node.declare_parameter<bool>(
    "navigation_final_yaw_align_enable", true);
  if (!module.api_final_yaw_align_fallback_enabled) {
    final_yaw_align_enabled = false;
  }
  const double final_yaw_tolerance_rad = std::clamp(
    node.declare_parameter<double>("navigation_final_yaw_tolerance_rad", 0.05),
    0.01,
    1.57);
  const double final_yaw_align_trigger_rad = std::max(
    final_yaw_tolerance_rad,
    std::clamp(
      node.declare_parameter<double>("navigation_final_yaw_align_trigger_rad", 0.08),
      0.01,
      1.57));
  const double final_yaw_align_success_tolerance_rad = std::clamp(
    node.declare_parameter<double>(
      "navigation_final_yaw_align_success_tolerance_rad",
      std::min(0.045, final_yaw_tolerance_rad)),
    0.005,
    final_yaw_tolerance_rad);
  const double final_yaw_align_speed_radps = std::clamp(
    node.declare_parameter<double>("navigation_final_yaw_align_speed_radps", 0.60),
    0.05,
    0.8);
  const double final_yaw_align_max_speed_radps = std::clamp(
    node.declare_parameter<double>(
      "navigation_final_yaw_align_max_speed_radps", final_yaw_align_speed_radps),
    0.05,
    0.8);
  const double final_yaw_align_min_speed_radps = std::clamp(
    node.declare_parameter<double>("navigation_final_yaw_align_min_speed_radps", 0.06),
    0.0,
    final_yaw_align_max_speed_radps);
  const double final_yaw_align_slowdown_start_rad = std::clamp(
    node.declare_parameter<double>(
      "navigation_final_yaw_align_slowdown_start_rad", 0.30),
    final_yaw_align_success_tolerance_rad,
    1.57);
  const double final_yaw_align_slow_max_speed_radps = std::clamp(
    node.declare_parameter<double>(
      "navigation_final_yaw_align_slow_max_speed_radps", 0.20),
    final_yaw_align_min_speed_radps,
    final_yaw_align_max_speed_radps);
  const double final_yaw_align_kp = std::clamp(
    node.declare_parameter<double>("navigation_final_yaw_align_kp", 1.2),
    0.1,
    5.0);
  const double final_yaw_align_timeout_sec = std::clamp(
    node.declare_parameter<double>("navigation_final_yaw_align_timeout_sec", 8.0),
    0.5,
    20.0);
  const double final_yaw_align_max_xy_drift_m = std::clamp(
    node.declare_parameter<double>("navigation_final_yaw_align_max_xy_drift_m", 0.08),
    0.01,
    0.50);
  const bool final_yaw_align_require_fresh_pose = node.declare_parameter<bool>(
    "navigation_final_yaw_align_require_fresh_pose", true);
  std::string final_yaw_align_cmd_topic = node.declare_parameter<std::string>(
    "navigation_final_yaw_align_cmd_topic", "/cmd_vel_api");
  if (final_yaw_align_cmd_topic != "/cmd_vel_nav" &&
    final_yaw_align_cmd_topic != "/cmd_vel_api")
  {
    RCLCPP_WARN(
      node.get_logger(),
      "navigation_final_yaw_align_cmd_topic=%s is not allowed; using /cmd_vel_api",
      final_yaw_align_cmd_topic.c_str());
    final_yaw_align_cmd_topic = "/cmd_vel_api";
  }
  (void)node.declare_parameter<bool>(
    "navigation_final_yaw_align_bypass_collision_monitor",
    final_yaw_align_cmd_topic == "/cmd_vel_api");
  const bool final_yaw_align_bypass_collision_monitor =
    final_yaw_align_cmd_topic == "/cmd_vel_api";
  const int final_yaw_align_zero_cmd_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "navigation_final_yaw_align_zero_cmd_count", 3)),
    1,
    10);

  terminal_runtime.yaw_actual_stop_check_enabled = node.declare_parameter<bool>(
    "yaw_align_actual_stop_check_enabled", true);
  terminal_runtime.actual_stop_odom_topic = node.declare_parameter<std::string>(
    "yaw_align_actual_stop_odom_topic", "/wheel/odom");
  terminal_runtime.yaw_actual_wz_threshold_radps = std::max(
    0.0,
    node.declare_parameter<double>("yaw_align_actual_wz_threshold_radps", 0.02));
  terminal_runtime.yaw_actual_wz_stable_samples = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "yaw_align_actual_wz_stable_samples", 5)),
    1,
    50);
  terminal_runtime.yaw_actual_stop_timeout_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "yaw_align_actual_stop_timeout_ms", 800)),
    0,
    3000);
  terminal_runtime.yaw_actual_wz_max_age_sec = std::max(
    0.02,
    node.declare_parameter<double>("yaw_align_actual_wz_max_age_sec", 0.20));
  const bool yaw_stop_lead_enabled = node.declare_parameter<bool>(
    "yaw_align_stop_lead_enabled", true);
  const double yaw_stop_lead_time_sec = std::clamp(
    node.declare_parameter<double>("yaw_align_stop_lead_time_sec", 0.125),
    0.0,
    0.50);
  const double yaw_stop_lead_min_rad = std::clamp(
    node.declare_parameter<double>("yaw_align_stop_lead_min_rad", 0.0),
    0.0,
    0.20);
  const double yaw_stop_lead_max_rad = std::clamp(
    node.declare_parameter<double>("yaw_align_stop_lead_max_rad", 0.09),
    yaw_stop_lead_min_rad,
    0.30);
  const bool final_yaw_wait_bridge_smoothing = node.declare_parameter<bool>(
    "navigation_final_yaw_align_wait_bridge_smoothing", true);
  const int final_yaw_bridge_wait_timeout_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "navigation_final_yaw_align_bridge_wait_timeout_ms", 2000)),
    100,
    60000);
  const int final_yaw_bridge_wait_sample_period_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "navigation_final_yaw_align_bridge_wait_sample_period_ms", 100)),
    20,
    1000);
  const bool pause_global_correction_during_final_yaw = node.declare_parameter<bool>(
    "navigation_pause_global_correction_during_final_yaw", true);
  module.lifecycle_check_timeout_sec = std::clamp(
    node.declare_parameter<double>("navigation_lifecycle_check_timeout_sec", 1.5),
    0.05,
    3.0);
  terminal_runtime.mode_controller_status_topic = node.declare_parameter<std::string>(
    "mode_controller_status_topic", "/ranger_base/status");
  terminal_runtime.local_costmap_topic = node.declare_parameter<std::string>(
    "local_costmap_topic", "/local_costmap/costmap");

  completion.map_frame = inputs.map_frame;
  completion.robot_pose_freshness_sec = inputs.robot_pose_freshness_sec;
  completion.position_tolerance_m = goal_position_success_tolerance_m;
  completion.yaw_tolerance_rad = final_yaw_tolerance_rad;
  completion.yaw_align_trigger_rad = final_yaw_align_trigger_rad;
  completion.final_verify_enabled = final_verify_enabled;
  completion.retry_uses_same_nav2_goal = final_verify_retry_uses_same_nav2_goal;
  completion.final_verify_max_retry_count = final_verify_max_retry_count;
  completion.acceptance_slack_m = final_verify_acceptance_slack_m;
  completion.xy_retry_min_error_m = final_verify_xy_retry_min_error_m;
  completion.yaw_retry_if_failed = final_verify_yaw_retry_if_failed;
  completion.terminal_recovery_max_distance_m = terminal_recovery_max_distance_m;
  completion.nav2_failed_near_goal_retry_enabled = nav2_failed_near_goal_retry_enabled;
  completion.nav2_failed_near_goal_retry_max_count =
    nav2_failed_near_goal_retry_max_count;
  completion.nav2_failed_near_goal_retry_requires_yaw_error =
    nav2_failed_near_goal_retry_requires_yaw_error;

  bridge_wait.final_verify_enabled = final_verify_enabled;
  bridge_wait.final_verify_wait_enabled = final_verify_wait_bridge_smoothing;
  bridge_wait.final_verify_timeout = std::chrono::milliseconds(
    final_verify_bridge_wait_timeout_ms);
  bridge_wait.final_verify_sample_period = std::chrono::milliseconds(
    final_verify_bridge_wait_sample_period_ms);
  bridge_wait.final_yaw_wait_enabled = final_yaw_wait_bridge_smoothing;
  bridge_wait.final_yaw_timeout = std::chrono::milliseconds(
    final_yaw_bridge_wait_timeout_ms);
  bridge_wait.final_yaw_sample_period = std::chrono::milliseconds(
    final_yaw_bridge_wait_sample_period_ms);

  terminal.final_verify_enabled = final_verify_enabled;
  terminal.api_velocity_correction_enabled = final_verify_api_velocity_correction_enabled;
  terminal.lateral_correction_enabled = terminal_lateral_correction_enabled;
  terminal.reverse_permit_enabled = terminal_runtime.post_nav2_reverse_permit_enabled;
  terminal.costmap_guard_enabled = costmap_guard_enabled;
  terminal.goal_position_success_tolerance_m = goal_position_success_tolerance_m;
  terminal.terminal_recovery_max_distance_m = terminal_recovery_max_distance_m;
  terminal.final_yaw_success_tolerance_rad = final_yaw_align_success_tolerance_rad;
  terminal.lateral_target_m = terminal_lateral_target_m;
  terminal.lateral_trigger_m = terminal_lateral_trigger_m;
  terminal.lateral_max_forward_m = terminal_lateral_max_forward_m;
  terminal.lateral_speed_mps = terminal_lateral_speed_mps;
  terminal.lateral_kp = terminal_lateral_kp;
  terminal.lateral_timeout_sec = terminal_lateral_timeout_sec;
  terminal.lateral_command_sign = terminal_lateral_command_sign;
  terminal.lateral_divergence_epsilon_m = inputs.lateral_divergence_epsilon_m;
  terminal.lateral_divergence_count = inputs.lateral_divergence_count;
  terminal.lateral_forced_mode = inputs.lateral_forced_mode;
  terminal.lateral_release_mode = inputs.lateral_release_mode;
  terminal.final_yaw_kp = final_yaw_align_kp;
  terminal.final_yaw_min_speed_radps = final_yaw_align_min_speed_radps;
  terminal.final_yaw_max_speed_radps = final_yaw_align_max_speed_radps;
  terminal.final_yaw_slow_max_speed_radps = final_yaw_align_slow_max_speed_radps;
  terminal.final_yaw_slowdown_start_rad = final_yaw_align_slowdown_start_rad;
  terminal.final_yaw_timeout_sec = final_yaw_align_timeout_sec;
  terminal.final_yaw_max_xy_drift_m = final_yaw_align_max_xy_drift_m;
  terminal.final_yaw_require_fresh_pose = final_yaw_align_require_fresh_pose;
  terminal.robot_pose_freshness_sec = inputs.robot_pose_freshness_sec;
  terminal.map_frame = inputs.map_frame;
  terminal.yaw_stop_lead_enabled = yaw_stop_lead_enabled;
  terminal.yaw_stop_lead_time_sec = yaw_stop_lead_time_sec;
  terminal.yaw_stop_lead_min_rad = yaw_stop_lead_min_rad;
  terminal.yaw_stop_lead_max_rad = yaw_stop_lead_max_rad;
  terminal.zero_command_count = final_yaw_align_zero_cmd_count;
  terminal.costmap_max_age_sec = costmap_max_age_sec;
  terminal.costmap_occupied_threshold = costmap_occupied_threshold;
  terminal.costmap_lookahead_m = costmap_lookahead_m;

  terminal_runtime.command_topic = final_yaw_align_cmd_topic;
  terminal_runtime.zero_command_count = final_yaw_align_zero_cmd_count;
  terminal_runtime.map_frame = inputs.map_frame;
  terminal_runtime.base_frame = inputs.base_frame;
  terminal_runtime.robot_pose_freshness_sec = inputs.robot_pose_freshness_sec;

  module.maps_root = inputs.maps_root;
  module.runtime_map_context_file = inputs.runtime_map_context_file;
  module.navigation_final_yaw_align_enabled = final_yaw_align_enabled;
  module.post_nav2_final_verify_enabled = final_verify_enabled;
  module.post_nav2_final_verify_wait_bridge_smoothing =
    final_verify_wait_bridge_smoothing;
  module.post_nav2_final_verify_max_retry_count = final_verify_max_retry_count;
  module.navigation_nav2_failed_near_goal_retry_max_count =
    nav2_failed_near_goal_retry_max_count;
  module.post_nav2_final_verify_api_velocity_correction_enabled =
    final_verify_api_velocity_correction_enabled;
  module.navigation_final_yaw_align_timeout_sec = final_yaw_align_timeout_sec;
  module.navigation_final_yaw_align_max_xy_drift_m = final_yaw_align_max_xy_drift_m;
  module.navigation_final_yaw_align_cmd_topic = final_yaw_align_cmd_topic;
  module.navigation_final_yaw_align_bypass_collision_monitor =
    final_yaw_align_bypass_collision_monitor;
  module.position_only_nav2_yaw_mode = goal_policy.position_only_nav2_yaw_mode;

  goal_execution.action_name = inputs.navigate_to_pose_action;
  goal_execution.service_timeout = seconds(inputs.service_timeout_sec);
  goal_execution.goal_result_timeout_sec = goal_result_timeout_sec;
  goal_execution.near_goal_stalled_handoff_enabled = near_goal_stalled_handoff_enabled;
  goal_execution.final_verify_enabled = final_verify_enabled;
  goal_execution.final_verify_wait_bridge_smoothing = final_verify_wait_bridge_smoothing;
  goal_execution.final_yaw_wait_bridge_smoothing = final_yaw_wait_bridge_smoothing;
  goal_execution.final_verify_request_amcl_nomotion_update =
    final_verify_request_amcl_nomotion_update;
  goal_execution.near_goal_stalled_handoff_min_wait_sec =
    near_goal_stalled_handoff_min_wait_sec;
  goal_execution.near_goal_stalled_handoff_stall_sec =
    near_goal_stalled_handoff_stall_sec;
  goal_execution.near_goal_stalled_handoff_improvement_epsilon_m =
    near_goal_stalled_handoff_improvement_epsilon_m;
  goal_execution.terminal_recovery_max_distance_m = terminal_recovery_max_distance_m;
  goal_execution.terminal_lateral_max_forward_m = terminal_lateral_max_forward_m;
  goal_execution.pause_global_correction_during_final_yaw =
    pause_global_correction_during_final_yaw;
  goal_execution.reposition_after_yaw_drift_timeout_sec =
    reposition_after_yaw_drift_timeout_sec;

  goal_executor.api_final_yaw_align_fallback_enabled =
    module.api_final_yaw_align_fallback_enabled;
  goal_executor.navigation_final_yaw_align_enabled = final_yaw_align_enabled;
  goal_executor.navigation_nav2_failed_near_goal_retry_enabled =
    nav2_failed_near_goal_retry_enabled;
  goal_executor.navigation_goal_result_timeout_sec = goal_result_timeout_sec;
  goal_executor.navigation_goal_position_success_tolerance_m =
    goal_position_success_tolerance_m;
  goal_executor.navigation_final_yaw_align_max_xy_drift_m =
    final_yaw_align_max_xy_drift_m;
  goal_executor.navigation_final_yaw_align_trigger_rad = final_yaw_align_trigger_rad;
  goal_executor.navigation_final_yaw_align_timeout_sec = final_yaw_align_timeout_sec;
  goal_executor.navigation_final_yaw_tolerance_rad = final_yaw_tolerance_rad;
  goal_executor.navigation_final_yaw_align_success_tolerance_rad =
    final_yaw_align_success_tolerance_rad;
  goal_executor.navigation_terminal_recovery_max_distance_m =
    terminal_recovery_max_distance_m;
  goal_executor.post_nav2_final_verify_terminal_lateral_target_m =
    terminal_lateral_target_m;
  goal_executor.post_nav2_final_verify_max_retry_count = final_verify_max_retry_count;
  goal_executor.service_timeout = seconds(inputs.service_timeout_sec);

  return config;
}

}  // namespace robot_api_server::features::navigation::configuration

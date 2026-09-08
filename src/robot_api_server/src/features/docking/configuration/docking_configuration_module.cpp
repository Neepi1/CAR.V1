#include "robot_api_server/features/docking/configuration/docking_configuration_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <utility>

namespace robot_api_server::features::docking::configuration
{

DockingConfiguration DockingConfigurationModule::declare_parameters(
  rclcpp::Node & node,
  const DockingConfigurationInputs & inputs)
{
  DockingConfiguration config;

  config.runtime.manager_start_command = node.declare_parameter<std::string>(
    "docking_manager_start_command",
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_docking_manager.sh");
  config.runtime.manager_log_file = node.declare_parameter<std::string>(
    "docking_manager_log_file", "/tmp/njrh_docking_manager.log");
  config.runtime.start_service = node.declare_parameter<std::string>(
    "docking_start_service", "/docking/start");
  config.runtime.stop_service = node.declare_parameter<std::string>(
    "docking_stop_service", "/docking/stop");
  config.runtime.undock_service = node.declare_parameter<std::string>(
    "docking_undock_service", "/docking/undock");
  config.runtime.status_topic = node.declare_parameter<std::string>(
    "docking_status_topic", "/docking/status");
  config.runtime.observation_backend = node.declare_parameter<std::string>(
    "docking_observation_backend", "target_observation");
  config.runtime.gs2_scan_topic = node.declare_parameter<std::string>(
    "docking_gs2_scan_topic", "/dock/gs2_scan");
  config.runtime.target_observation_topic = node.declare_parameter<std::string>(
    "docking_target_observation_topic", "/dock/target_observation");
  config.runtime.target_observation_source = node.declare_parameter<std::string>(
    "docking_target_observation_source", "orbbec_336l_depth");
  config.runtime.service_timeout_sec = inputs.service_timeout_sec;
  config.runtime.stop_service_wait_sec = std::clamp(
    node.declare_parameter<double>("docking_stop_service_wait_sec", 3.0),
    0.5,
    inputs.service_timeout_sec);
  config.runtime.undock_charging_retry_sec = std::max(
    0.0,
    node.declare_parameter<double>("docking_undock_charging_retry_sec", 3.0));

  config.contact_interlock.latch_file = std::filesystem::path(
    node.declare_parameter<std::string>(
      "docking_contact_latch_file",
      "/workspaces/njrh-v3/workspace1/maps_release/docking_contact_latch.json"));
  config.contact_interlock.bms_ttl_sec = std::max(
    0.0,
    node.declare_parameter<double>("dock_contact_latch_bms_ttl_sec", 300.0));
  config.contact_interlock.bms_clear_no_contact_sec = std::max(
    0.0,
    node.declare_parameter<double>("dock_contact_latch_bms_clear_no_contact_sec", 3.0));
  config.contact_interlock.allow_bms_stale_auto_undock = node.declare_parameter<bool>(
    "dock_contact_latch_allow_bms_stale_auto_undock", false);
  config.contact_interlock.clear_when_live_undocked_no_contact = node.declare_parameter<bool>(
    "dock_contact_latch_clear_when_live_undocked_no_contact", true);
  config.contact_interlock.max_age_warn_sec = std::max(
    0.0,
    node.declare_parameter<double>("dock_contact_latch_max_age_warn_sec", 600.0));
  config.contact_interlock.charging_current_min_a = inputs.charging_current_min_a;
  config.contact_interlock.full_soc_threshold_pct = inputs.full_soc_threshold_pct;
  config.contact_interlock.docking_status_topic = config.runtime.status_topic;
  config.contact_interlock.require_safety_interlock_state = node.declare_parameter<bool>(
    "navigation_require_dock_safety_interlock_state", true);
  config.contact_interlock.safety_interlock_state_topic = node.declare_parameter<std::string>(
    "dock_safety_interlock_state_topic", "/safety/dock_interlock_state");
  config.contact_interlock.safety_interlock_reconcile_service =
    node.declare_parameter<std::string>(
    "dock_safety_interlock_reconcile_service", "/safety/reconcile_dock_interlock");
  config.contact_interlock.safety_interlock_state_max_age_sec = std::max(
    0.2,
    node.declare_parameter<double>("dock_safety_interlock_state_max_age_sec", 1.0));
  config.contact_interlock.safety_interlock_reconcile_timeout_sec = std::max(
    0.5,
    node.declare_parameter<double>("dock_safety_interlock_reconcile_timeout_sec", 3.0));
  config.contact_interlock.dock_zone_pose_max_age_sec = std::max(
    0.1,
    node.declare_parameter<double>(
      "dock_interlock_zone_pose_max_age_sec", inputs.robot_pose_freshness_sec));
  config.contact_interlock.dock_zone_near_radius_m = std::max(
    0.1,
    node.declare_parameter<double>("dock_interlock_zone_near_radius_m", 1.0));
  config.contact_interlock.dock_zone_clear_radius_m = std::max(
    config.contact_interlock.dock_zone_near_radius_m + 0.1,
    node.declare_parameter<double>("dock_interlock_zone_clear_radius_m", 1.5));

  config.http.pre_dock_distance_m = std::max(
    0.05,
    node.declare_parameter<double>("docking_pre_dock_distance_m", 0.60));
  config.http.default_dock_profile_id = node.declare_parameter<std::string>(
    "docking_default_dock_profile_id", "gs2_rear_charging_dock");
  config.http.default_dock_profile_type = node.declare_parameter<std::string>(
    "docking_default_dock_profile_type", "gs2_near_field");
  config.http.default_approach_direction = node.declare_parameter<std::string>(
    "docking_default_approach_direction", "reverse");
  config.http.default_contact_frame = node.declare_parameter<std::string>(
    "docking_default_contact_frame", "charge_contact_link");
  config.http.default_sensor_frame = node.declare_parameter<std::string>(
    "docking_default_sensor_frame", "gs2_link");
  config.http.max_retries = std::clamp(
    static_cast<int>(node.declare_parameter<int>("docking_max_retries", 2)),
    0,
    5);

  config.job_execution.service_timeout_sec = inputs.service_timeout_sec;
  config.job_execution.navigation_start_wait_sec = std::max(
    inputs.service_timeout_sec,
    node.declare_parameter<double>("docking_navigation_start_wait_sec", 45.0));

  config.job_executor.predock_nav_timeout_sec = std::max(
    5.0,
    node.declare_parameter<double>("docking_predock_nav_timeout_sec", 180.0));
  config.job_executor.predock_behavior_tree = node.declare_parameter<std::string>(
    "docking_predock_behavior_tree",
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/robot_nav_config/behavior_trees/"
    "navigate_to_predock.xml");
  config.job_executor.predock_early_handoff_enabled = node.declare_parameter<bool>(
    "docking_predock_early_handoff_enabled", false);
  config.job_executor.relocalize_before_predock = node.declare_parameter<bool>(
    "docking_relocalize_before_predock", false);
  config.job_executor.relocalize_after_predock = node.declare_parameter<bool>(
    "docking_relocalize_after_predock", false);
  config.job_executor.relocalize_after_predock_required = node.declare_parameter<bool>(
    "docking_relocalize_after_predock_required", false);
  config.job_executor.validate_predock_pose_after_relocalization =
    node.declare_parameter<bool>(
    "docking_validate_predock_pose_after_relocalization", true);
  config.job_executor.cancel_active_goal_before_predock = node.declare_parameter<bool>(
    "docking_cancel_active_goal_before_predock", true);
  config.job_executor.navigate_to_pose_action = inputs.navigate_to_pose_action;

  config.status.relocalize_after_fine_docking = node.declare_parameter<bool>(
    "docking_relocalize_after_fine_docking", false);
  config.status.relocalize_after_fine_docking_required = node.declare_parameter<bool>(
    "docking_relocalize_after_fine_docking_required", false);

  const double predock_pose_max_distance_m = std::max(
    0.05,
    node.declare_parameter<double>("docking_predock_pose_max_distance_m", 0.35));
  const double predock_pose_max_yaw_rad = std::max(
    0.01,
    node.declare_parameter<double>("docking_predock_pose_max_yaw_rad", 0.35));
  config.predock_pose_resolver.distance_check_enabled = node.declare_parameter<bool>(
    "docking_manual_predock_distance_check_enable", false);
  config.predock_pose_resolver.min_distance_m = std::clamp(
    node.declare_parameter<double>("docking_manual_predock_min_distance_m", 0.50),
    0.05,
    5.00);
  config.predock_pose_resolver.max_distance_m = std::max(
    config.predock_pose_resolver.min_distance_m + 0.05,
    node.declare_parameter<double>("docking_manual_predock_max_distance_m", 1.20));
  config.predock_pose_resolver.max_yaw_error_rad = std::clamp(
    node.declare_parameter<double>("docking_manual_predock_max_yaw_error_rad", 0.80),
    0.05,
    3.14);

  const double docking_relocalize_wait_sec = std::max(
    0.5,
    node.declare_parameter<double>("docking_relocalize_wait_sec", 8.0));
  config.localization_default_relocalization_wait_sec = docking_relocalize_wait_sec;
  config.localization_recent_result_max_age_sec = std::max(
    0.0,
    node.declare_parameter<double>("docking_relocalize_recent_result_max_age_sec", 5.0));
  config.status.docking_relocalize_wait_sec = docking_relocalize_wait_sec;
  config.status.undock_relocalize_after_success = node.declare_parameter<bool>(
    "undock_relocalize_after_success", true);
  config.status.undock_relocalize_wait_sec = std::max(
    0.5,
    node.declare_parameter<double>("undock_relocalize_wait_sec", docking_relocalize_wait_sec));
  config.status.observation_backend = config.runtime.observation_backend;

  config.pre_navigation_undock.relocalize_after_success =
    config.status.undock_relocalize_after_success;
  config.pre_navigation_undock.relocalize_wait_sec =
    config.status.undock_relocalize_wait_sec;
  config.pre_navigation_undock.auto_undock_timeout_sec = std::max(
    inputs.service_timeout_sec,
    node.declare_parameter<double>("navigation_auto_undock_timeout_sec", 28.0));

  config.framework_state_machine_enabled = node.declare_parameter<bool>(
    "docking_framework_state_machine_enabled", true);

  const bool yaw_align_enabled = node.declare_parameter<bool>(
    "predock_yaw_align_enabled", true);
  config.predock_control.delegate_staging_motion_to_manager = node.declare_parameter<bool>(
    "docking_delegate_staging_motion_to_manager", true);
  const bool yaw_align_fallback_enabled = node.declare_parameter<bool>(
    "predock_yaw_align_fallback_enabled", true);
  config.runtime.command_topic = node.declare_parameter<std::string>(
    "predock_yaw_align_cmd_topic", "/cmd_vel_docking");
  const double yaw_align_tolerance_rad = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_tolerance_rad", 0.0698),
    0.005,
    0.35);
  const double yaw_align_trigger_rad = std::max(
    yaw_align_tolerance_rad,
    std::clamp(
      node.declare_parameter<double>("predock_yaw_align_trigger_rad", 0.0698),
      0.005,
      0.80));
  const double yaw_align_hard_fail_rad = std::max(
    yaw_align_trigger_rad,
    std::clamp(
      node.declare_parameter<double>("predock_yaw_align_hard_fail_rad", 0.80),
      0.10,
      3.14));

  const bool lateral_align_enabled = node.declare_parameter<bool>(
    "predock_lateral_align_enabled", true);
  const double lateral_align_target_m = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_target_m", 0.03),
    0.005,
    0.20);
  const double lateral_align_trigger_m = std::max(
    lateral_align_target_m,
    std::clamp(
      node.declare_parameter<double>("predock_lateral_align_trigger_m", 0.03),
      0.005,
      0.25));
  const double lateral_align_max_correction_m = std::max(
    lateral_align_trigger_m,
    std::clamp(
      node.declare_parameter<double>("predock_lateral_align_max_correction_m", 0.25),
      0.02,
      0.30));
  const double lateral_align_yaw_slack_rad = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_yaw_slack_rad", 0.02),
    0.0,
    0.20);
  const int staging_capture_max_cycles = std::clamp(
    static_cast<int>(node.declare_parameter<int>("predock_staging_capture_max_cycles", 6)),
    1,
    20);
  const double forward_capture_min_m = std::clamp(
    node.declare_parameter<double>("predock_forward_capture_min_m", -0.40),
    -2.0,
    0.0);
  const double forward_capture_max_m = std::clamp(
    node.declare_parameter<double>("predock_forward_capture_max_m", 0.55),
    0.0,
    2.0);

  config.alignment_policy.pose_max_distance_m = predock_pose_max_distance_m;
  config.alignment_policy.handoff_max_yaw_rad = predock_pose_max_yaw_rad;
  config.alignment_policy.yaw_tolerance_rad = yaw_align_tolerance_rad;
  config.alignment_policy.yaw_hard_fail_rad = yaw_align_hard_fail_rad;
  config.alignment_policy.lateral_target_m = lateral_align_target_m;
  config.alignment_policy.lateral_max_correction_m = lateral_align_max_correction_m;
  config.alignment_policy.lateral_yaw_slack_rad = lateral_align_yaw_slack_rad;
  config.alignment_policy.forward_capture_min_m = forward_capture_min_m;
  config.alignment_policy.forward_capture_max_m = forward_capture_max_m;

  config.predock_control.validate_pose_after_relocalization =
    config.job_executor.validate_predock_pose_after_relocalization;
  config.predock_control.yaw_align_enabled = yaw_align_enabled;
  config.predock_control.yaw_align_fallback_enabled = yaw_align_fallback_enabled;
  config.predock_control.yaw_align_tolerance_rad = yaw_align_tolerance_rad;
  config.predock_control.yaw_align_hard_fail_rad = yaw_align_hard_fail_rad;
  config.predock_control.yaw_align_timeout_sec = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_timeout_sec", 10.0),
    0.5,
    30.0);
  config.predock_control.yaw_align_min_speed_radps = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_min_speed_radps", 0.05),
    0.0,
    0.40);
  config.predock_control.yaw_align_max_speed_radps = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_max_speed_radps", 0.20),
    0.05,
    0.50);
  config.predock_control.yaw_align_kp = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_kp", 1.2),
    0.1,
    5.0);
  config.predock_control.yaw_align_success_hold_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>("predock_yaw_align_success_hold_count", 3)),
    1,
    20);
  config.predock_control.yaw_align_period_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>("predock_yaw_align_period_ms", 67)),
    20,
    250);
  config.predock_control.yaw_align_zero_cmd_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>("predock_yaw_align_zero_cmd_count", 5)),
    1,
    20);
  config.predock_control.yaw_align_require_actual_spin = node.declare_parameter<bool>(
    "predock_yaw_align_require_actual_spin", true);
  config.predock_control.yaw_align_mode_switch_timeout_sec = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_mode_switch_timeout_sec", 2.0),
    0.2,
    10.0);
  config.predock_control.yaw_align_no_yaw_motion_timeout_sec = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_no_yaw_motion_timeout_sec", 2.0),
    0.2,
    10.0);
  config.predock_control.yaw_align_motion_epsilon_rad = std::clamp(
    node.declare_parameter<double>("predock_yaw_align_motion_epsilon_rad", 0.01),
    0.001,
    0.10);

  config.predock_control.lateral_align_enabled = lateral_align_enabled;
  config.predock_control.lateral_align_target_m = lateral_align_target_m;
  config.predock_control.lateral_align_max_correction_m = lateral_align_max_correction_m;
  config.predock_control.forward_capture_min_m = forward_capture_min_m;
  config.predock_control.forward_capture_max_m = forward_capture_max_m;
  config.predock_control.staging_capture_max_cycles = staging_capture_max_cycles;
  config.predock_control.lateral_align_timeout_sec = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_timeout_sec", 8.0),
    0.5,
    30.0);
  config.predock_control.lateral_align_speed_mps = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_speed_mps", 0.025),
    0.005,
    0.15);
  config.predock_control.lateral_align_kp = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_kp", 0.7),
    0.05,
    5.0);
  config.predock_control.lateral_align_period_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>("predock_lateral_align_period_ms", 67)),
    20,
    250);
  config.predock_control.lateral_align_zero_cmd_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>("predock_lateral_align_zero_cmd_count", 5)),
    1,
    20);
  config.predock_control.lateral_align_command_sign = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_command_sign", -1.0),
    -1.0,
    1.0);
  if (std::fabs(config.predock_control.lateral_align_command_sign) < 0.5) {
    config.predock_control.lateral_align_command_sign = -1.0;
  }
  config.predock_control.lateral_align_no_motion_timeout_sec = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_no_motion_timeout_sec", 2.0),
    0.2,
    10.0);
  config.predock_control.lateral_align_motion_epsilon_m = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_motion_epsilon_m", 0.005),
    0.001,
    0.05);
  config.predock_control.lateral_align_divergence_epsilon_m = std::clamp(
    node.declare_parameter<double>("predock_lateral_align_divergence_epsilon_m", 0.015),
    0.002,
    0.08);
  config.predock_control.lateral_align_divergence_count = std::clamp(
    static_cast<int>(node.declare_parameter<int>("predock_lateral_align_divergence_count", 2)),
    1,
    10);
  config.predock_control.lateral_align_auto_reverse_on_divergence =
    node.declare_parameter<bool>(
    "predock_lateral_align_auto_reverse_on_divergence", true);
  config.runtime.forced_mode_topic = node.declare_parameter<std::string>(
    "predock_lateral_align_forced_mode_topic", "/ranger_mini3/forced_mode");
  config.predock_control.lateral_align_forced_mode = node.declare_parameter<std::string>(
    "predock_lateral_align_forced_mode", "side_slip");
  config.predock_control.lateral_align_release_mode = node.declare_parameter<std::string>(
    "predock_lateral_align_release_mode", "auto");

  config.predock_control.fine_entry_require_observation_fresh = node.declare_parameter<bool>(
    "fine_docking_entry_require_gs2_fresh", true);
  config.predock_control.fine_entry_require_predock_yaw_aligned = node.declare_parameter<bool>(
    "fine_docking_entry_require_predock_yaw_aligned", true);
  config.predock_control.fine_entry_max_distance_m = std::max(
    0.02,
    node.declare_parameter<double>(
      "fine_docking_entry_max_distance_m", predock_pose_max_distance_m));
  config.predock_control.fine_entry_max_yaw_rad = std::max(
    0.005,
    node.declare_parameter<double>("fine_docking_entry_max_yaw_rad", 0.0349));
  config.predock_control.fine_entry_max_lateral_m = std::max(
    0.005,
    node.declare_parameter<double>("fine_docking_entry_max_lateral_m", 0.05));
  config.predock_control.fine_retry_on_yaw_reject = node.declare_parameter<bool>(
    "fine_docking_retry_on_yaw_reject", true);
  config.predock_control.fine_wait_for_bridge_smoothing_enabled =
    node.declare_parameter<bool>(
    "docking_fine_wait_for_bridge_smoothing_enabled", true);
  config.predock_control.fine_bridge_smoothing_wait_timeout_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "docking_fine_bridge_smoothing_wait_timeout_ms", 60000)),
    100,
    60000);
  config.predock_control.fine_bridge_smoothing_sample_period_ms = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "docking_fine_bridge_smoothing_sample_period_ms", 100)),
    20,
    1000);
  config.predock_control.fine_bridge_smoothing_stable_samples = std::clamp(
    static_cast<int>(node.declare_parameter<int>(
      "docking_fine_bridge_smoothing_stable_samples", 3)),
    1,
    20);
  config.predock_control.fine_bridge_smoothing_zero_cmd_during_wait =
    node.declare_parameter<bool>(
    "docking_fine_bridge_smoothing_zero_cmd_during_wait", true);
  config.predock_control.pause_global_correction_during_fine = node.declare_parameter<bool>(
    "docking_pause_global_correction_during_fine", true);
  config.predock_control.observation_backend = config.runtime.observation_backend;
  config.predock_control.map_frame = inputs.map_frame;
  config.predock_control.robot_pose_freshness_sec = inputs.robot_pose_freshness_sec;

  config.correction_pause.enabled =
    config.predock_control.pause_global_correction_during_fine;
  config.correction_pause.service_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(inputs.service_timeout_sec));

  return config;
}

}  // namespace robot_api_server::features::docking::configuration

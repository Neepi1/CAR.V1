#!/usr/bin/env python3
"""Static ownership contract for the ordinary-navigation configuration module."""

from pathlib import Path
import re
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
ROOT_SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "composition"
    / "application_composition_module.cpp"
)
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "navigation"
    / "configuration"
    / "navigation_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "navigation"
    / "configuration"
    / "navigation_configuration_module.cpp"
)

NAVIGATION_PARAMETERS = {
    "navigation_resume_command",
    "navigation_resume_log_file",
    "navigation_resume_starting_context_ttl_sec",
    "navigation_stop_command",
    "navigation_stop_log_file",
    "navigation_cancel_action_wait_sec",
    "navigation_relocalize_before_goal",
    "navigation_relocalize_before_goal_always",
    "navigation_relocalize_before_goal_required",
    "navigation_relocalize_wait_sec",
    "navigation_goal_result_timeout_sec",
    "navigation_goal_position_success_tolerance_m",
    "navigation_terminal_speed_limit_enabled",
    "navigation_terminal_speed_limit_topic",
    "navigation_terminal_speed_limit_far_distance_m",
    "navigation_terminal_speed_limit_mid_distance_m",
    "navigation_terminal_speed_limit_near_distance_m",
    "navigation_terminal_speed_limit_crawl_distance_m",
    "navigation_terminal_speed_limit_far_mps",
    "navigation_terminal_speed_limit_mid_mps",
    "navigation_terminal_speed_limit_near_mps",
    "navigation_terminal_speed_limit_crawl_mps",
    "navigation_terminal_speed_limit_final_mps",
    "navigation_default_goal_completion_policy",
    "navigation_delivery_point_goal_completion_policy",
    "navigation_position_only_nav2_yaw_mode",
    "navigation_position_only_approach_heading_min_distance_m",
    "nav2_native_goal_completion_enabled",
    "nav2_rotation_shim_enabled",
    "api_final_yaw_align_fallback_enabled",
    "post_nav2_final_verify_enabled",
    "post_nav2_final_verify_wait_bridge_smoothing",
    "post_nav2_final_verify_bridge_wait_timeout_ms",
    "post_nav2_final_verify_bridge_wait_sample_period_ms",
    "post_nav2_final_verify_request_amcl_nomotion_update",
    "post_nav2_final_verify_amcl_nomotion_update_service",
    "post_nav2_final_verify_max_retry_count",
    "post_nav2_final_verify_acceptance_slack_m",
    "post_nav2_final_verify_xy_retry_min_error_m",
    "navigation_terminal_recovery_max_distance_m",
    "post_nav2_final_verify_yaw_retry_if_failed",
    "post_nav2_final_verify_retry_uses_same_nav2_goal",
    "post_nav2_final_verify_api_velocity_correction_enabled",
    "post_nav2_final_verify_reverse_permit_enabled",
    "post_nav2_final_verify_reverse_enable_topic",
    "navigation_terminal_reverse_permit_enabled",
    "navigation_terminal_reverse_permit_enter_distance_m",
    "navigation_terminal_reverse_permit_exit_distance_m",
    "navigation_terminal_reverse_permit_refresh_period_sec",
    "post_nav2_final_verify_terminal_lateral_correction_enabled",
    "post_nav2_final_verify_terminal_lateral_target_m",
    "post_nav2_final_verify_terminal_lateral_trigger_m",
    "post_nav2_final_verify_terminal_lateral_max_forward_m",
    "post_nav2_final_verify_terminal_lateral_speed_mps",
    "post_nav2_final_verify_terminal_lateral_kp",
    "post_nav2_final_verify_terminal_lateral_timeout_sec",
    "navigation_terminal_recovery_costmap_guard_enabled",
    "navigation_terminal_recovery_costmap_max_age_sec",
    "navigation_terminal_recovery_costmap_occupied_threshold",
    "navigation_terminal_recovery_costmap_lookahead_m",
    "post_nav2_final_verify_terminal_lateral_command_sign",
    "post_nav2_final_verify_terminal_settle_enabled",
    "post_nav2_final_verify_terminal_settle_linear_speed_threshold_mps",
    "post_nav2_final_verify_terminal_settle_angular_speed_threshold_radps",
    "post_nav2_final_verify_terminal_settle_stable_duration_sec",
    "post_nav2_final_verify_terminal_settle_timeout_sec",
    "post_nav2_final_verify_terminal_settle_odom_max_age_sec",
    "post_nav2_final_verify_terminal_settle_require_dual_ackermann_mode",
    "post_nav2_final_verify_terminal_settle_mode_status_max_age_sec",
    "post_nav2_final_verify_terminal_settle_max_recheck_count",
    "navigation_near_goal_stalled_handoff_enabled",
    "navigation_near_goal_stalled_handoff_min_wait_sec",
    "navigation_near_goal_stalled_handoff_stall_sec",
    "navigation_near_goal_stalled_handoff_improvement_epsilon_m",
    "navigation_nav2_failed_near_goal_retry_enabled",
    "navigation_nav2_failed_near_goal_retry_max_count",
    "navigation_nav2_failed_near_goal_retry_requires_yaw_error",
    "navigation_max_reposition_after_yaw_retry",
    "navigation_reposition_after_yaw_drift_timeout_sec",
    "navigation_final_yaw_align_enable",
    "navigation_final_yaw_tolerance_rad",
    "navigation_final_yaw_align_trigger_rad",
    "navigation_final_yaw_align_success_tolerance_rad",
    "navigation_final_yaw_align_speed_radps",
    "navigation_final_yaw_align_max_speed_radps",
    "navigation_final_yaw_align_min_speed_radps",
    "navigation_final_yaw_align_slowdown_start_rad",
    "navigation_final_yaw_align_slow_max_speed_radps",
    "navigation_final_yaw_align_kp",
    "navigation_final_yaw_align_timeout_sec",
    "navigation_final_yaw_align_max_xy_drift_m",
    "navigation_final_yaw_align_require_fresh_pose",
    "navigation_final_yaw_align_cmd_topic",
    "navigation_final_yaw_align_bypass_collision_monitor",
    "navigation_final_yaw_align_zero_cmd_count",
    "yaw_align_actual_stop_check_enabled",
    "yaw_align_actual_stop_odom_topic",
    "yaw_align_actual_wz_threshold_radps",
    "yaw_align_actual_wz_stable_samples",
    "yaw_align_actual_stop_timeout_ms",
    "yaw_align_actual_wz_max_age_sec",
    "yaw_align_stop_lead_enabled",
    "yaw_align_stop_lead_time_sec",
    "yaw_align_stop_lead_min_rad",
    "yaw_align_stop_lead_max_rad",
    "navigation_final_yaw_align_wait_bridge_smoothing",
    "navigation_final_yaw_align_bridge_wait_timeout_ms",
    "navigation_final_yaw_align_bridge_wait_sample_period_ms",
    "navigation_pause_global_correction_during_final_yaw",
    "navigation_lifecycle_check_timeout_sec",
    "mode_controller_status_topic",
    "local_costmap_topic",
}


def declared_parameters(text: str) -> set[str]:
    return set(
        re.findall(
            r'declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"',
            text,
            flags=re.DOTALL,
        )
    )


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    for path in (HEADER, SOURCE, ROOT_SOURCE):
        if not path.is_file():
            fail(f"missing required file: {path}")

    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    root = ROOT_SOURCE.read_text(encoding="utf-8")

    for token in (
        "struct NavigationConfigurationInputs",
        "struct NavigationConfiguration",
        "class NavigationConfigurationModule",
        "NavigationModuleConfig module",
        "NavigationTerminalRuntimeConfig terminal_runtime",
        "NavigationGoalExecutionConfig goal_execution",
        "NavigationGoalExecutorConfig goal_executor",
        "std::string amcl_nomotion_update_service",
    ):
        if token not in header:
            fail(f"aggregate interface is missing {token!r}")

    owned = declared_parameters(source)
    missing = sorted(NAVIGATION_PARAMETERS - owned)
    extra_root = sorted(NAVIGATION_PARAMETERS & declared_parameters(root))
    if missing:
        fail("configuration module does not own: " + ", ".join(missing))
    if extra_root:
        fail("composition root still declares: " + ", ".join(extra_root))

    if "NavigationConfigurationModule::declare_parameters" not in root:
        fail("composition root does not consume NavigationConfigurationModule")

    legacy_members = re.findall(
        r"^\s+(?:bool|int|double|std::string)\s+"
        r"(?:navigation_|nav2_|post_nav2_|yaw_align_)[A-Za-z0-9_]*_;",
        root,
        flags=re.MULTILINE,
    )
    legacy_members.extend(
        re.findall(
            r"^\s+std::string\s+(?:local_costmap_topic_|mode_controller_status_topic_);",
            root,
            flags=re.MULTILINE,
        )
    )
    if legacy_members:
        fail(
            "legacy scalar navigation members remain: "
            + ", ".join(x.strip() for x in legacy_members)
        )

    for dead_token in (
        "struct NavigationRelocalizationDecision",
        "navigation_goal_relocalization_decision(",
    ):
        if dead_token in root:
            fail(f"obsolete root-only navigation helper remains: {dead_token}")

    print(
        "PASS: navigation configuration owns all "
        f"{len(NAVIGATION_PARAMETERS)} ROS parameters and emits composed sub-configs"
    )


if __name__ == "__main__":
    main()

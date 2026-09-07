#!/usr/bin/env python3
"""Static ownership contract for the docking ROS parameter composition module."""

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
    / "docking"
    / "configuration"
    / "docking_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "configuration"
    / "docking_configuration_module.cpp"
)

DOCKING_PARAMETERS = {
    "docking_manager_start_command",
    "docking_manager_log_file",
    "docking_start_service",
    "docking_stop_service",
    "docking_undock_service",
    "docking_status_topic",
    "docking_observation_backend",
    "docking_gs2_scan_topic",
    "docking_target_observation_topic",
    "docking_target_observation_source",
    "docking_contact_latch_file",
    "dock_contact_latch_bms_ttl_sec",
    "dock_contact_latch_bms_clear_no_contact_sec",
    "dock_contact_latch_allow_bms_stale_auto_undock",
    "dock_contact_latch_clear_when_live_undocked_no_contact",
    "dock_contact_latch_max_age_warn_sec",
    "docking_pre_dock_distance_m",
    "docking_stop_service_wait_sec",
    "docking_navigation_start_wait_sec",
    "docking_predock_nav_timeout_sec",
    "docking_predock_behavior_tree",
    "docking_predock_early_handoff_enabled",
    "docking_relocalize_before_predock",
    "docking_relocalize_after_predock",
    "docking_relocalize_after_predock_required",
    "docking_relocalize_after_fine_docking",
    "docking_relocalize_after_fine_docking_required",
    "docking_validate_predock_pose_after_relocalization",
    "docking_predock_pose_max_distance_m",
    "docking_predock_pose_max_yaw_rad",
    "docking_manual_predock_distance_check_enable",
    "docking_manual_predock_min_distance_m",
    "docking_manual_predock_max_distance_m",
    "docking_manual_predock_max_yaw_error_rad",
    "docking_relocalize_wait_sec",
    "docking_relocalize_recent_result_max_age_sec",
    "undock_relocalize_after_success",
    "undock_relocalize_wait_sec",
    "docking_cancel_active_goal_before_predock",
    "navigation_auto_undock_timeout_sec",
    "docking_undock_charging_retry_sec",
    "docking_framework_state_machine_enabled",
    "docking_default_dock_profile_id",
    "docking_default_dock_profile_type",
    "docking_default_approach_direction",
    "docking_default_contact_frame",
    "docking_default_sensor_frame",
    "docking_max_retries",
    "predock_yaw_align_enabled",
    "predock_yaw_align_fallback_enabled",
    "predock_yaw_align_cmd_topic",
    "predock_yaw_align_tolerance_rad",
    "predock_yaw_align_trigger_rad",
    "predock_yaw_align_hard_fail_rad",
    "predock_yaw_align_timeout_sec",
    "predock_yaw_align_min_speed_radps",
    "predock_yaw_align_max_speed_radps",
    "predock_yaw_align_kp",
    "predock_yaw_align_success_hold_count",
    "predock_yaw_align_period_ms",
    "predock_yaw_align_zero_cmd_count",
    "predock_yaw_align_require_actual_spin",
    "predock_yaw_align_mode_switch_timeout_sec",
    "predock_yaw_align_no_yaw_motion_timeout_sec",
    "predock_yaw_align_motion_epsilon_rad",
    "predock_lateral_align_enabled",
    "predock_lateral_align_target_m",
    "predock_lateral_align_trigger_m",
    "predock_lateral_align_max_correction_m",
    "predock_lateral_align_yaw_slack_rad",
    "predock_staging_capture_max_cycles",
    "predock_forward_capture_min_m",
    "predock_forward_capture_max_m",
    "predock_lateral_align_timeout_sec",
    "predock_lateral_align_speed_mps",
    "predock_lateral_align_kp",
    "predock_lateral_align_period_ms",
    "predock_lateral_align_zero_cmd_count",
    "predock_lateral_align_command_sign",
    "predock_lateral_align_no_motion_timeout_sec",
    "predock_lateral_align_motion_epsilon_m",
    "predock_lateral_align_divergence_epsilon_m",
    "predock_lateral_align_divergence_count",
    "predock_lateral_align_auto_reverse_on_divergence",
    "predock_lateral_align_forced_mode_topic",
    "predock_lateral_align_forced_mode",
    "predock_lateral_align_release_mode",
    "fine_docking_entry_require_gs2_fresh",
    "fine_docking_entry_require_predock_yaw_aligned",
    "fine_docking_entry_max_distance_m",
    "fine_docking_entry_max_yaw_rad",
    "fine_docking_entry_max_lateral_m",
    "fine_docking_retry_on_yaw_reject",
    "docking_fine_wait_for_bridge_smoothing_enabled",
    "docking_fine_bridge_smoothing_wait_timeout_ms",
    "docking_fine_bridge_smoothing_sample_period_ms",
    "docking_fine_bridge_smoothing_stable_samples",
    "docking_fine_bridge_smoothing_zero_cmd_during_wait",
    "docking_pause_global_correction_during_fine",
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
        "struct DockingConfigurationInputs",
        "struct DockingConfiguration",
        "class DockingConfigurationModule",
        "DockingRuntimeConfig runtime",
        "PredockControlConfig predock_control",
        "DockingJobExecutorConfig job_executor",
        "DockingHttpConfig http",
        "DockingStatusConfig status",
    ):
        if token not in header:
            fail(f"aggregate interface is missing {token!r}")

    owned = declared_parameters(source)
    missing = sorted(DOCKING_PARAMETERS - owned)
    extra_root = sorted(DOCKING_PARAMETERS & declared_parameters(root))
    if missing:
        fail("configuration module does not own: " + ", ".join(missing))
    if extra_root:
        fail("composition root still declares: " + ", ".join(extra_root))

    if "DockingConfigurationModule::declare_parameters" not in root:
        fail("composition root does not consume DockingConfigurationModule")

    legacy_members = re.findall(
        r"^\s+(?:bool|int|double|std::string)\s+"
        r"(?:docking_|predock_|fine_docking_)[A-Za-z0-9_]*_;",
        root,
        flags=re.MULTILINE,
    )
    if legacy_members:
        fail("legacy scalar docking members remain: " + ", ".join(x.strip() for x in legacy_members))

    print(
        "PASS: docking configuration owns all "
        f"{len(DOCKING_PARAMETERS)} ROS parameters and emits composed sub-configs"
    )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import re
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "localization"
    / "localization_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "localization"
    / "localization_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "localization_trigger_service",
    "localization_result_topic",
    "localization_bridge_status_topic",
    "localization_floor_health_topic",
    "amcl_runtime_status_file",
    "amcl_runtime_status_ttl_sec",
    "tf_topic",
    "tf_static_topic",
    "tf_map_frame",
    "tf_odom_frame",
    "tf_base_frame",
    "post_relocalization_static_lidar_frame",
    "tf_pose_max_age_sec",
    "robot_pose_freshness_sec",
    "tf_chain_freshness_sec",
    "tf_chain_settle_timeout_sec",
    "localization_trigger_service_timeout_sec",
    "localization_bridge_acceptance_timeout_sec",
    "localization_bridge_acceptance_max_distance_m",
    "localization_bridge_acceptance_max_yaw_rad",
    "manual_relocalization_amcl_refine_enabled",
    "manual_relocalization_amcl_refine_required",
    "manual_relocalization_amcl_refine_timeout_sec",
    "manual_relocalization_amcl_refine_poll_ms",
    "manual_relocalization_amcl_refine_request_period_ms",
    "localization_bridge_correction_pause_service",
    "post_relocalization_settle_enabled",
    "post_relocalization_settle_min_ms",
    "post_relocalization_settle_max_ms",
    "post_relocalization_stable_tf_samples",
    "post_relocalization_tf_sample_period_ms",
    "post_relocalization_zero_cmd",
    "post_relocalization_require_local_costmap_update",
    "post_relocalization_required_local_costmap_updates",
    "post_relocalization_reject_if_new_message_filter_drop",
    "post_relocalization_map_odom_publish_gap_warn_ms",
    "post_relocalization_map_odom_publish_gap_fail_ms",
    "post_undock_relocalization_settle_enabled",
    "post_undock_relocalization_settle_min_ms",
    "post_undock_relocalization_settle_max_ms",
    "post_undock_stable_tf_samples",
    "post_undock_tf_sample_period_ms",
    "post_undock_required_local_costmap_updates",
    "post_undock_reject_if_new_message_filter_drop",
    "post_undock_zero_cmd_during_settle",
    "post_relocalization_large_correction_translation_m",
    "post_relocalization_large_correction_yaw_rad",
    "post_relocalization_large_correction_min_ms",
}


def declared_parameters(text: str) -> list[str]:
    return re.findall(r'declare_parameter<[^>]+>\s*\(\s*"([^"]+)"', text)


def main() -> None:
    assert HEADER.is_file(), "localization configuration header is missing"
    assert SOURCE.is_file(), "localization configuration source is missing"

    root = ROOT_NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    declarations = declared_parameters(source)

    assert len(EXPECTED_PARAMETERS) == 48
    assert len(declarations) == 48, declarations
    assert len(declarations) == len(set(declarations)), "duplicate parameter declaration"
    assert set(declarations) == EXPECTED_PARAMETERS
    for parameter in EXPECTED_PARAMETERS:
        assert not re.search(
            rf'declare_parameter<[^>]+>\s*\(\s*"{re.escape(parameter)}"', root
        ), f"composition root still declares localization parameter: {parameter}"

    for token in (
        "struct LocalizationConfiguration",
        "LocalizationModuleConfig module",
        "PostRelocalizationSettleConfig settle",
        "std::string floor_health_topic",
    ):
        assert token in header
    for token in (
        "settle.tf_chain_freshness_sec = module.tf_chain_freshness_sec",
        "settle.robot_pose_freshness_sec = module.robot_pose_freshness_sec",
        "settle.base_frame = module.base_frame",
        "settle.static_lidar_frame = module.static_lidar_frame",
    ):
        assert token in source
    for legacy_scalar in (
        "tf_map_frame_",
        "tf_odom_frame_",
        "tf_base_frame_",
        "post_relocalization_static_lidar_frame_",
        "tf_pose_max_age_sec_",
        "robot_pose_freshness_sec_",
        "tf_chain_freshness_sec_",
    ):
        assert legacy_scalar not in root, f"legacy localization scalar remains: {legacy_scalar}"

    print(
        "PASS: localization configuration owns all 48 ROS parameters and emits "
        "module/settle projections"
    )


if __name__ == "__main__":
    main()

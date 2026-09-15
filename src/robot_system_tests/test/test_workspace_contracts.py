import importlib.util
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]


def navigation_goal_execution_source() -> str:
    return (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "mission"
        / "navigation_goal_execution_module.cpp"
    ).read_text(encoding="utf-8")


def docking_configuration_source() -> str:
    return (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "configuration"
        / "docking_configuration_module.cpp"
    ).read_text(encoding="utf-8")


def docking_feature_source() -> str:
    return (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "docking_feature_module.cpp"
    ).read_text(encoding="utf-8")


def navigation_configuration_source() -> str:
    return (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "configuration"
        / "navigation_configuration_module.cpp"
    ).read_text(encoding="utf-8")


def localization_configuration_source() -> str:
    return (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_configuration_module.cpp"
    ).read_text(encoding="utf-8")


def localization_feature_source() -> str:
    return (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_feature_module.cpp"
    ).read_text(encoding="utf-8")


def application_composition_source() -> str:
    return (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "application"
        / "composition"
        / "application_composition_module.cpp"
    ).read_text(encoding="utf-8")


def local_costmap_config_block(nav2_yaml: str) -> str:
    start = nav2_yaml.index("local_costmap:\n  local_costmap:")
    end = nav2_yaml.index("\ncollision_monitor:", start)
    return nav2_yaml[start:end]


def yaml_number(text: str, key: str) -> float:
    match = re.search(rf"^\s*{re.escape(key)}:\s*([-+]?\d+(?:\.\d+)?)\s*$", text, re.MULTILINE)
    assert match, key
    return float(match.group(1))


def assert_nav2_local_costmap_frame_contract(nav2_yaml: str, local_costmap: str) -> None:
    assert "global_frame: odom" in local_costmap
    assert "global_frame: base_link" not in local_costmap
    assert "robot_base_frame: base_link" in local_costmap
    assert 'plugins: ["obstacle_layer", "local_inflation_layer"]' in local_costmap
    assert 'plugin: "nav2_costmap_2d::ObstacleLayer"' in local_costmap
    assert "voxel_layer:" not in local_costmap
    assert "nav2_costmap_2d::VoxelLayer" not in local_costmap
    assert "combination_method: 0" in local_costmap
    assert "footprint_clearing_enabled: true" in local_costmap
    assert 'plugin: "robot_nav_config::ElevatorAwareProgressChecker"' in nav2_yaml
    assert 'plugin: "nav2_controller::PoseProgressChecker"' not in nav2_yaml
    assert 'plugin: "nav2_controller::SimpleProgressChecker"' not in nav2_yaml
    assert "required_movement_radius: 0.03" in nav2_yaml
    assert "required_movement_angle: 0.05" in nav2_yaml
    assert "movement_time_allowance: 12.0" in nav2_yaml
    assert (
        "        scan:\n"
        "          topic: /scan\n"
        "          sensor_frame: lidar_level_link\n"
        "          data_type: LaserScan\n"
    ) in local_costmap
    assert "          marking: true\n" in local_costmap
    assert "          clearing: true\n" in local_costmap
    assert "          inf_is_valid: true\n" in local_costmap
    assert "/perception/obstacle_points" not in local_costmap
    assert "/perception/clearing_points" not in local_costmap


def test_reports_exist():
    for name in (
        "car_project_reuse_report.md",
        "tf_audit_report.md",
        "third_party_resolution_report.md",
        "occupancy_builder_design.md",
    ):
        assert (ROOT / "reports" / name).exists(), name


def test_navigation_failure_observer_uses_ros2_rosout_log_type():
    script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "observe_navigation_failure_minimal.sh"
    ).read_text(encoding="utf-8")
    assert "from rcl_interfaces.msg import Log" in script
    assert "from rosgraph_msgs.msg import Log" not in script
    assert "from nav2_msgs.msg import SpeedLimit" in script
    assert "from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath" in script
    assert '"/speed_limit"' in script
    assert '"/transformed_global_plan"' in script
    assert '"/local_costmap/costmap"' in script
    assert "cmd_frames.jsonl" in script
    assert "subscribes_tf\": False" in script
    assert "publishes_topics\": False" in script
    assert '"stores_full_costmap": store_costmap_snapshots' in script
    assert "--store-costmap-snapshots" in script
    assert "--costmap-snapshot-period-sec" in script
    assert "costmap_snapshots" in script
    assert ".int8.bin.gz" in script
    assert 'request.add_header("X-Robot-Token", api_token)' in script
    assert "except KeyboardInterrupt:" in script
    assert "if rclpy.ok():" in script
    assert 'if "rcl_shutdown already called" not in str(exc):' in script


def test_navigation_terminal_adjustment_observer_is_read_only_and_captures_handoff():
    script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "observe_navigation_terminal_adjustment.sh"
    ).read_text(encoding="utf-8")
    assert "does not send goals, publish velocity, call services, set params, or restart nodes" in script
    assert "from rcl_interfaces.msg import Log" in script
    assert "from nav2_msgs.msg import SpeedLimit" in script
    assert '"/cmd_vel_nav_raw"' in script
    assert '"/cmd_vel_api"' in script
    assert '"/speed_limit"' in script
    assert '"/navigate_to_pose/_action/status"' in script
    assert "/api/v1/navigation/state" in script
    assert "/api/v1/robot/pose" in script
    assert "DURATION_SEC=40" in script
    assert "--stop-when-terminal" in script
    assert "goal_terminal_elapsed_sec" in script
    assert "capture_stopped_after_terminal_goal" in script
    assert "observed_goal_id" in script
    assert "ignored_pre_active_samples" in script
    assert "current_observed_goal and near_goal" in script
    assert "no_running_goal_observed_after_observer_start" in script
    assert "nav2_near_goal_tiny_cmd_elapsed_sec" in script
    assert "near_goal_to_api_yaw_delay_sec" in script
    assert "api_final_yaw_started_after_" in script
    assert "ros2 topic pub" not in script
    assert "ros2 service call" not in script
    assert "ros2 param set" not in script


def test_navigation_amcl_odom_correlation_observer_is_read_only():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "record_navigation_amcl_odom_correlation.sh"
    ).read_text(encoding="utf-8")
    assert "does not send goals, publish velocity, call services, set params, or restart nodes" in script
    assert "tf2_ros.TransformListener" in script
    assert '"/localization/bridge_status"' in script
    assert '"/motion_state"' in script
    assert '"/wheel/odom"' in script
    assert '"/wheel/odom_ekf"' in script
    assert '"/local_state/odometry"' in script
    assert '"/cmd_vel_nav_raw"' in script
    assert '"/cmd_vel_api"' in script
    assert '"/cmd_vel"' in script
    assert '"/speed_limit"' in script
    assert "/api/v1/navigation/state" in script
    assert "/api/v1/robot/pose" in script
    assert "amcl_bridge_events.csv" in script
    assert "delta_amcl_accepted_count" in script
    assert "delta_amcl_rejected_count" in script
    assert "last_candidate_correction_translation_m" in script
    assert "last_accepted_correction_translation_m" in script
    assert "Largest Rejected Candidate Events" in script
    assert "Largest Accepted Correction Events" in script
    assert "--stop-when-terminal" in script
    assert "create_publisher" not in script
    assert "ros2 topic pub" not in script
    assert "ros2 action send_goal" not in script
    assert "ros2 service call" not in script
    assert "ros2 param set" not in script


def test_localization_bridge_gates_physical_base_pose_innovation():
    bridge = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    math_header = (
        ROOT
        / "src"
        / "robot_localization_bridge"
        / "include"
        / "robot_localization_bridge"
        / "se2_correction.hpp"
    ).read_text(encoding="utf-8")
    math_test = (
        ROOT / "src" / "robot_localization_bridge" / "test" / "test_se2_correction.cpp"
    ).read_text(encoding="utf-8")
    relocalize_capture = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "capture_relocalize_correction_compare.sh"
    ).read_text(encoding="utf-8")

    assert "solve_correction(" in bridge
    assert "solution.base_translation_m" in bridge
    assert "solution.map_odom_parameter_translation_m" in bridge
    assert "correction_metric_frame\\\":\\\"map_base_link" in bridge
    assert "compare_map_odom_hypotheses_at_reference(" in bridge
    assert "smoothing_remaining_duration_sec" in bridge
    assert "const double progress_step" in bridge
    assert "solve_map_odom(" in math_header
    assert "predicted_map_base = compose(current_map_odom, odom_base)" in math_header
    assert "YawAtLongOdomLeverArmIsNotRobotTranslation" in math_test
    assert "GateMetricPreservesPhysicalMediumCorrection" in math_test
    assert "def compose_pose(" in relocalize_capture
    assert "composed_map_odom_x_odom_base_link" in relocalize_capture
    assert "TF parameter only: map->odom" in relocalize_capture


def test_local_costmap_scan_clearing_observer_is_read_only():
    script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "observe_local_costmap_scan_clearing.sh"
    ).read_text(encoding="utf-8")
    assert 'SCAN_TOPIC="${SCAN_TOPIC:-/scan}"' in script
    assert 'COSTMAP_TOPIC="${COSTMAP_TOPIC:-/local_costmap/costmap}"' in script
    assert "RAYTRACE_MIN_RANGE_M=0.20" in script
    assert "Raytrace min range. Default: 0.20." in script
    assert "tf2_ros.TransformListener" in script
    assert "supported_by_current_scan_endpoint" in script
    assert "behind_current_scan_endpoint_blocked" in script
    assert "inside_current_clear_ray_but_still_occupied" in script
    assert '"calls_services": False' in script
    assert '"sets_params": False' in script
    assert '"publishes_control": False' in script
    assert '"clears_costmap": False' in script
    assert "clear_entirely_local_costmap" not in script
    assert "ros2 service call" not in script


def test_local_costmap_raytrace_min_range_contract():
    nav2 = (ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml").read_text(encoding="utf-8")
    overlay_nav2 = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml"
    ).read_text(encoding="utf-8")

    for config in (nav2, overlay_nav2):
        local_costmap = local_costmap_config_block(config)
        assert "          raytrace_min_range: 0.20\n" in local_costmap


def test_initial_localization_wrapper_timeout_fallback_accepts_plain_tf_edge():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    start = script.index("initial_localization_ready_from_bridge_after_wrapper_failure()")
    end = script.index("\n}\n\ntrigger_global_localization_for_navigation()", start)
    fallback = script[start:end]
    assert 'wait_for_fresh_tf_transform "map" "odom" "${tf_timeout}" "${max_tf_age_sec}"' in fallback
    assert 'wait_for_tf_transform "map" "odom" "${tf_timeout}"' in fallback
    assert "plain TF edge exists" in fallback
    assert "active bridge/map->odom ownership was already verified" in fallback
    assert "baseline_explicit_sequence" in fallback
    assert "wrapper-timeout fallback has no pre-trigger bridge sequence baseline" in fallback
    assert "wait_for_bridge_relocalization_sequence_after" in fallback
    assert "newer than baseline=" in fallback


def test_resident_navigation_prioritizes_localization_stack_before_held_nav2_prestart():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    source_index = script.index('navigation_start_source="${NJRH_NAVIGATION_START_SOURCE:-direct}"')
    api_preflight_index = script.index(
        'if [[ "${navigation_start_source}" == "api_resume" ]]; then',
        source_index,
    )
    stable_gate_index = script.index(
        'ensure_common_local_state_ready_for_navigation_start "stable" || {',
        api_preflight_index,
    )
    localization_index = script.index('bash "${SCRIPT_DIR}/run_occupancy_grid_localization.sh" &')
    boot_fallback_index = script.index(
        'ensure_common_local_state_ready_for_navigation_start "fresh" || {',
        localization_index,
    )
    held_prestart_index = script.index('env_flag_true "${NJRH_NAV2_HELD_PRESTART_AFTER_LOCAL_STATE:-true}"')
    localization_stack_index = script.index("ensure_localization_stack_ready_for_navigation || {")
    floor_context_index = script.index('log_startup_stage "floor_asset_context_verified"')
    initial_background_index = script.index(
        'if env_flag_true "${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}"; then'
    )
    initial_localization_index = script.index("if ! wait_for_initial_global_localization; then")
    lifecycle_activation_index = script.index("if ! activate_prestarted_nav2_lifecycle; then")
    assert (
        source_index
        < api_preflight_index
        < stable_gate_index
        < localization_index
        < boot_fallback_index
        < localization_stack_index
        < floor_context_index
        < initial_background_index
        < initial_localization_index
        < held_prestart_index
        < lifecycle_activation_index
    )


def test_nav2_helper_cleanup_does_not_kill_global_localization_wrapper():
    helpers = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "nav_runtime_helpers.sh"
    ).read_text(encoding="utf-8")
    start = helpers.index("stop_existing_overlay_nav_helpers()")
    end = helpers.index("\n}\n\nstart_overlay_helper()", start)
    cleanup = helpers[start:end]
    assert "robot_global_localization" not in cleanup
    assert "run_global_localization.sh" not in cleanup


def test_production_local_obstacle_scan_slice_is_not_too_low():
    overlay_config = ROOT / "scripts" / "jetson" / "runtime_overlay" / "config"
    for path in (
        overlay_config / "pointcloud_accel_axis.yaml",
        overlay_config / "hesai_accel_driver.yaml",
    ):
        text = path.read_text(encoding="utf-8")
        assert "scan_worker_min_height: -0.50" in text, path
        assert "scan_worker_max_height: 0.35" in text, path
        assert "scan_worker_min_height: -0.75" not in text, path


def test_nav_defaults_are_fixed():
    nav2 = (ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml").read_text(encoding="utf-8")
    overlay_nav2 = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml").read_text(
        encoding="utf-8"
    )
    nav_to_pose_bt = (ROOT / "src" / "robot_nav_config" / "behavior_trees" / "navigate_to_pose.xml").read_text(
        encoding="utf-8"
    )
    nav_through_poses_bt = (
        ROOT / "src" / "robot_nav_config" / "behavior_trees" / "navigate_through_poses.xml"
    ).read_text(encoding="utf-8")
    nav2_local_costmap = local_costmap_config_block(nav2)
    overlay_local_costmap = local_costmap_config_block(overlay_nav2)
    assert "SmacPlanner2D" in nav2
    assert "MPPIController" in nav2
    assert "RegulatedPurePursuitController" in nav2
    assert "tolerance: 0.05" in nav2
    assert "tolerance: 0.05" in overlay_nav2
    assert "tolerance: 0.25" not in nav2
    assert "tolerance: 0.25" not in overlay_nav2
    assert 'motion_model: "Ackermann"' in nav2
    assert "AckermannConstraints:" in nav2
    assert "min_turning_r: 0.81" in nav2
    assert "time_steps: 48" in nav2
    assert "vx_std: 0.30" in nav2
    assert "vy_std: 0.0" in nav2
    assert "wz_std: 0.32" in nav2
    assert "vx_max: 1.20" in nav2
    assert "vx_min: 0.0" in nav2
    assert "vx_min: 0.0" in overlay_nav2
    assert "vx_min: -0.08" not in nav2
    assert "vx_min: -0.08" not in overlay_nav2
    assert "vx_min: -0.20" not in nav2
    assert "vx_min: -0.20" not in overlay_nav2
    assert "vy_max: 0.0" in nav2
    assert "wz_max: 0.70" in nav2
    assert "ax_max:" not in nav2
    assert "ax_min:" not in nav2
    assert "az_max:" not in nav2
    for cfg in (nav2, overlay_nav2):
        assert "GoalCritic:" in cfg
        assert "cost_weight: 16.0" in cfg
        assert "threshold_to_consider: 1.5" in cfg
        assert "GoalAngleCritic:" in cfg
        assert "cost_weight: 6.0" in cfg
        assert "PathAlignCritic:" in cfg
        assert "cost_weight: 2.4" in cfg
        assert "PathFollowCritic:" in cfg
        assert "cost_weight: 7.0" in cfg
        assert "TwirlingCritic:" in cfg
        assert "cost_weight: 1.0" in cfg
        assert "twirling_cost_weight" not in cfg
        assert "twirling_cost_power" not in cfg
        assert '- "PreferForwardCritic"' in cfg
        assert "PreferForwardCritic:" in cfg
        assert "cost_weight: 20.0" in cfg
        assert "threshold_to_consider: 0.50" in cfg
        assert '- "VelocityDeadbandCritic"' in cfg
        assert "VelocityDeadbandCritic:" in cfg
        assert "deadband_velocities: [0.025, 0.0, 0.025]" in cfg
        assert "cost_weight: 90.0" in cfg
    assert "desired_linear_vel: 0.80" in nav2
    assert 'feedback: "OPEN_LOOP"' in nav2
    assert 'feedback: "OPEN_LOOP"' in overlay_nav2
    assert "odom_duration: 0.2" in nav2
    assert "odom_duration: 0.2" in overlay_nav2
    assert "max_velocity: [1.20, 0.40, 0.70]" in nav2
    assert "max_velocity: [1.20, 0.40, 0.70]" in overlay_nav2
    assert "min_velocity: [-0.40, -0.40, -0.70]" in nav2
    assert "min_velocity: [-0.40, -0.40, -0.70]" in overlay_nav2
    assert "min_velocity: [0.0, 0.0, -1.00]" not in nav2
    assert "min_velocity: [0.0, 0.0, -1.00]" not in overlay_nav2
    assert "min_velocity: [-0.20, 0.0, -1.00]" not in nav2
    assert "min_velocity: [-0.20, 0.0, -1.00]" not in overlay_nav2
    assert "max_accel: [0.55, 0.20, 0.90]" in nav2
    assert "max_accel: [0.55, 0.20, 0.90]" in overlay_nav2
    assert "max_decel: [-0.95, -0.30, -1.10]" in nav2
    assert "max_decel: [-0.95, -0.30, -1.10]" in overlay_nav2
    assert "robot_nav_config::GoalScopedRotationShimController" in nav2
    assert 'primary_controller: "robot_nav_config::RangerMPPIController"' in nav2
    assert "angular_dist_threshold: 0.45" in nav2
    assert "angular_disengage_threshold: 0.075" in nav2
    assert "angular_dist_threshold: 0.45" in overlay_nav2
    assert "angular_disengage_threshold: 0.075" in overlay_nav2
    assert "angular_dist_threshold: 1.20" not in nav2
    assert "angular_dist_threshold: 1.20" not in overlay_nav2
    assert "angular_dist_threshold: 3.20" not in nav2
    assert "angular_dist_threshold: 3.20" not in overlay_nav2
    assert 'plugin: "robot_nav_config::ElevatorAwareProgressChecker"' in nav2
    assert 'plugin: "nav2_controller::PoseProgressChecker"' not in nav2
    assert 'plugin: "nav2_controller::SimpleProgressChecker"' not in nav2
    assert "required_movement_radius: 0.03" in nav2
    assert "required_movement_angle: 0.05" in nav2
    assert "movement_time_allowance: 12.0" in nav2
    assert "elevator_required_movement_radius: 0.015" in nav2
    assert "elevator_required_movement_angle: 0.015" in nav2
    assert "elevator_movement_time_allowance: 20.0" in nav2
    assert "rotate_to_goal_heading: true" in nav2
    assert "rotate_to_goal_heading: false" not in nav2
    for cfg in (nav2, overlay_nav2):
        assert '".position_checker.stateful": false' in cfg
        assert "- nav2_rate_controller_bt_node" in cfg
    assert "use_rotate_to_heading: false" in nav2
    assert "observation_sources: scan" in nav2
    assert "observation_sources: scan" in overlay_nav2
    assert "data_type: LaserScan" in nav2
    assert "data_type: LaserScan" in overlay_nav2
    assert "inf_is_valid: true" in nav2
    assert "inf_is_valid: true" in overlay_nav2
    assert "/perception/obstacle_points" not in nav2
    assert "/perception/clearing_points" not in nav2
    assert "/perception/obstacle_points" not in overlay_nav2
    assert "/perception/clearing_points" not in overlay_nav2
    assert 'plugins: ["obstacle_layer", "local_inflation_layer"]' in nav2
    assert 'plugin: "nav2_costmap_2d::ObstacleLayer"' in nav2
    assert "voxel_layer:" not in nav2_local_costmap
    assert "nav2_costmap_2d::VoxelLayer" not in nav2_local_costmap
    assert "min_obstacle_height: -0.20" in nav2
    assert "max_obstacle_height: 1.40" in nav2
    assert "global_frame: odom" in nav2_local_costmap
    assert "global_frame: odom" in overlay_local_costmap
    assert "global_frame: base_link" not in nav2_local_costmap
    assert "global_frame: base_link" not in overlay_local_costmap
    assert "robot_base_frame: base_link" in nav2_local_costmap
    assert "robot_base_frame: base_link" in overlay_local_costmap
    assert "sensor_frame: lidar_level_link" in nav2
    assert "sensor_frame: lidar_level_link" in overlay_nav2
    assert "clearing: true" in nav2
    assert "clearing: false" not in nav2_local_costmap
    assert "clearing: false" not in overlay_local_costmap
    assert "observation_persistence: 0.0" in nav2
    assert "controller_frequency: 15.0" in nav2
    assert "bt_loop_duration: 50" in nav2
    assert "default_server_timeout: 5000" in nav2
    assert "wait_for_service_timeout: 8000" in nav2
    assert "bt_loop_duration: 50" in overlay_nav2
    assert "default_server_timeout: 5000" in overlay_nav2
    assert "wait_for_service_timeout: 8000" in overlay_nav2
    assert "transform_tolerance: 0.10" in nav2
    assert "transform_tolerance: 0.15" in nav2_local_costmap
    assert "transform_tolerance: 0.15" in overlay_local_costmap
    assert "tf_filter_tolerance" not in nav2
    assert "tf_filter_tolerance" not in overlay_nav2
    assert "transform_tolerance: 0.00" not in nav2
    assert "transform_tolerance: 0.50" not in nav2
    assert "transform_tolerance: 0.5" not in nav2
    assert "speed_limit_topic: /speed_limit" in nav2
    assert "time_steps: 48" in nav2
    assert "time_steps: 48" in overlay_nav2
    assert "time_steps: 44" not in nav2
    assert "time_steps: 44" not in overlay_nav2
    assert "model_dt: 0.0666666667" in nav2
    assert (
        "default_nav_to_pose_bt_xml: "
        "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/robot_nav_config/behavior_trees/navigate_to_pose.xml"
    ) in nav2
    assert (
        "default_nav_to_pose_bt_xml: "
        "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/robot_nav_config/behavior_trees/navigate_to_pose.xml"
    ) in overlay_nav2
    assert (
        "default_nav_through_poses_bt_xml: "
        "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/robot_nav_config/behavior_trees/navigate_through_poses.xml"
    ) in nav2
    assert (
        "default_nav_through_poses_bt_xml: "
        "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/robot_nav_config/behavior_trees/navigate_through_poses.xml"
    ) in overlay_nav2
    assert "navigate_to_pose_w_replanning_and_recovery.xml" not in nav2
    assert "navigate_to_pose_w_replanning_and_recovery.xml" not in overlay_nav2
    assert "navigate_through_poses_w_replanning_and_recovery.xml" not in nav2
    assert "navigate_through_poses_w_replanning_and_recovery.xml" not in overlay_nav2
    for bt_xml in (nav_to_pose_bt, nav_through_poses_bt):
        assert bt_xml.count('<RateController hz="1.0">') == 1
        assert '<RateController hz="0.33">' not in bt_xml
        assert "<SmoothPath " not in bt_xml
        assert 'smoother_id="SavitzkyGolay"' not in bt_xml
        assert 'max_smoothing_duration="0.10"' not in bt_xml
        assert 'check_for_collisions="true"' not in bt_xml
        assert (
            '<FollowPath path="{path}" controller_id="FollowPath" '
            'goal_checker_id="goal_checker"/>' in bt_xml
        )
        assert '<Wait wait_duration="1.0"/>' not in bt_xml
        assert '<Wait wait_duration="5.0"/>' not in bt_xml
    assert re.search(
        r'<RateController hz="1\.0">\s*<ComputePathToPose\b',
        nav_to_pose_bt,
    )
    assert re.search(
        r'<RateController hz="1\.0">\s*<ComputePathThroughPoses\b',
        nav_through_poses_bt,
    )
    assert 'smoother_plugins: ["SimpleSmoother"]' in nav2
    assert 'smoother_plugins: ["SimpleSmoother"]' in overlay_nav2
    assert 'plugin: "nav2_smoother::SimpleSmoother"' in nav2
    assert 'plugin: "nav2_smoother::SimpleSmoother"' in overlay_nav2
    assert "SavitzkyGolay" not in nav2
    assert "SavitzkyGolay" not in overlay_nav2
    for bt_plugin in (
        "nav2_compute_path_to_pose_action_bt_node",
        "nav2_compute_path_through_poses_action_bt_node",
        "nav2_follow_path_action_bt_node",
        "nav2_recovery_node_bt_node",
        "nav2_wait_action_bt_node",
        "nav2_pipeline_sequence_bt_node",
        "nav2_rate_controller_bt_node",
        "nav2_navigate_to_pose_action_bt_node",
        "nav2_navigate_through_poses_action_bt_node",
    ):
        assert bt_plugin in nav2
        assert bt_plugin in overlay_nav2
    for unused_bt_plugin in (
        "nav2_spin_action_bt_node",
        "nav2_back_up_action_bt_node",
        "nav2_goal_updated_condition_bt_node",
        "nav2_round_robin_node_bt_node",
        "nav2_remove_passed_goals_action_bt_node",
        "nav2_smooth_path_action_bt_node",
        "nav2_clear_costmap_service_bt_node",
    ):
        assert unused_bt_plugin not in nav2
        assert unused_bt_plugin not in overlay_nav2
    assert "failure_tolerance: 10.0" in nav2
    assert "failure_tolerance: 10.0" in overlay_nav2
    assert "batch_size: 1200" in nav2
    assert "width: 10" in nav2
    assert "height: 10" in nav2
    assert "obstacle_max_range: 5.50" in nav2
    assert "raytrace_max_range: 8.00" in nav2
    assert "repulsion_weight: 2.0" in nav2
    assert "critical_weight: 20.0" in nav2
    assert "collision_margin_distance: 0.08" in nav2
    assert nav2.count("inflation_radius: 0.60") == 3
    assert overlay_nav2.count("inflation_radius: 0.60") == 3
    assert "inflation_radius: 0.35" not in nav2
    assert "inflation_radius: 0.35" not in overlay_nav2
    assert 'inflation_layer_name: "local_inflation_layer"' in nav2
    assert "max_path_occupancy_ratio: 0.05" in nav2
    assert "source_timeout: 1.5" in nav2
    assert "stop_pub_timeout: 0.3" in nav2
    assert 'polygons: ["StopZone", "SlowZone", "FootprintApproach"]' in nav2
    assert 'points: [0.42, 0.28, 0.42, -0.28, -0.42, -0.28, -0.42, 0.28]' in nav2
    assert 'footprint_topic: "/local_costmap/published_footprint"' in nav2
    assert 'action_type: "approach"' in nav2
    assert "/local_state/odometry" in nav2
    assert "/cmd_vel_collision_checked" in nav2
    assert "collision_monitor" in nav2
    assert "keepout_filter_mask_server" in nav2
    assert "keepout_costmap_filter_info_server" in nav2
    costmap_filter_nodes = nav2[
        nav2.index("lifecycle_manager_costmap_filters:") : nav2.index("planner_server:")
    ]
    assert "speed_filter_mask_server" not in costmap_filter_nodes
    assert "speed_costmap_filter_info_server" not in costmap_filter_nodes
    assert 'filters: ["keepout_filter"]' in nav2
    assert 'plugin: "nav2_costmap_2d::KeepoutFilter"' in nav2
    assert "filter_info_topic: /costmap_filter_info/keepout" in nav2
    assert 'plugin: "nav2_costmap_2d::SpeedFilter"' in nav2
    assert "filter_info_topic: /costmap_filter_info/speed" in nav2
    assert "xy_goal_tolerance: 0.06" in nav2
    assert "xy_goal_tolerance: 0.06" in overlay_nav2
    assert "xy_goal_tolerance: 0.10" not in nav2
    assert "xy_goal_tolerance: 0.10" not in overlay_nav2
    assert "xy_goal_tolerance: 0.20" not in nav2
    assert "xy_goal_tolerance: 0.20" not in overlay_nav2
    assert "yaw_goal_tolerance: 0.05" in nav2
    assert "yaw_goal_tolerance: 0.05" in overlay_nav2
    assert "yaw_goal_tolerance: 0.0873" not in nav2
    assert "yaw_goal_tolerance: 0.0873" not in overlay_nav2
    assert "yaw_goal_tolerance: 0.15" not in nav2
    assert "yaw_goal_tolerance: 0.15" not in overlay_nav2
    assert 'plugin: "robot_nav_config::ElevatorAwareProgressChecker"' in overlay_nav2
    assert 'plugin: "nav2_controller::PoseProgressChecker"' not in overlay_nav2
    assert 'plugin: "nav2_controller::SimpleProgressChecker"' not in overlay_nav2
    assert "required_movement_angle: 0.05" in overlay_nav2
    assert "rotate_to_goal_heading: true" in overlay_nav2
    assert "rotate_to_goal_heading: false" not in overlay_nav2
    assert "rotate_to_heading_angular_vel: 0.60" in overlay_nav2
    assert "max_angular_accel: 1.2" in overlay_nav2
    assert "terminal_rotation_braking_enabled: true" in overlay_nav2
    assert "closed_loop: true" in overlay_nav2
    assert 'goal_checker_plugins: ["goal_checker", "dock_staging_goal_checker"]' in overlay_nav2
    assert "goal_checker:" in overlay_nav2
    assert "pose_goal_checker:" not in overlay_nav2
    assert "position_goal_checker:" not in overlay_nav2
    assert 'plugin: "nav2_controller::PositionGoalChecker"' not in overlay_nav2
    assert "stateful: false" in nav2
    assert "stateful: false" in overlay_nav2
    assert "stateful: true" not in nav2
    assert "stateful: true" not in overlay_nav2

    local_perception = (
        ROOT / "src" / "robot_local_perception" / "config" / "local_perception.yaml"
    ).read_text(encoding="utf-8")
    overlay_local_perception = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_perception.yaml"
    ).read_text(encoding="utf-8")
    for cfg in (local_perception, overlay_local_perception):
        assert "enabled: false" in cfg
        assert 'input_topic: ""' in cfg
        assert 'output_topic: ""' in cfg
        assert 'clearing_output_topic: ""' in cfg
        assert "clearing.enabled: false" in cfg
        assert 'status_topic: ""' in cfg
        assert "/lidar_points\n" not in cfg
        assert "/_internal/lidar_points_local" not in cfg
        assert "/jt128/vendor/points_raw" not in cfg
        assert "/perception/obstacle_points" not in cfg
        assert "/perception/clearing_points" not in cfg
        assert "/perception/local_perception_status" not in cfg


def test_normal_navigation_terminal_speed_limit_matches_ranger_dynamics():
    execution_cpp = navigation_goal_execution_source()
    navigation_config_cpp = navigation_configuration_source()
    terminal_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_control.cpp"
    ).read_text(encoding="utf-8")
    terminal_runtime_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")

    for cfg in (api_cfg, overlay_api_cfg):
        assert "navigation_terminal_speed_limit_enabled: true" in cfg
        assert 'navigation_terminal_speed_limit_topic: "/speed_limit"' in cfg
        assert "navigation_terminal_speed_limit_far_distance_m: 2.4" in cfg
        assert "navigation_terminal_speed_limit_mid_distance_m: 1.5" in cfg
        assert "navigation_terminal_speed_limit_near_distance_m: 0.9" in cfg
        assert "navigation_terminal_speed_limit_crawl_distance_m: 0.35" in cfg
        assert "navigation_terminal_speed_limit_far_mps: 1.20" in cfg
        assert "navigation_terminal_speed_limit_mid_mps: 0.65" in cfg
        assert "navigation_terminal_speed_limit_near_mps: 0.32" in cfg
        assert "navigation_terminal_speed_limit_crawl_mps: 0.16" in cfg
        assert "navigation_terminal_speed_limit_final_mps: 0.08" in cfg

    assert '#include "nav2_msgs/msg/speed_limit.hpp"' in terminal_runtime_cpp
    assert "create_publisher<nav2_msgs::msg::SpeedLimit>" in terminal_runtime_cpp
    assert "terminal_control_.speed_limit_for_distance(distance_m)" in terminal_runtime_cpp
    assert "NavigationTerminalControl::speed_limit_for_distance" in terminal_cpp
    assert "terminal_runtime_.publish_speed_limit_for_goal(target);" in execution_cpp
    assert "terminal_runtime_.clear_speed_limit();" in execution_cpp
    assert "publish_speed_limit_value(config_.speed_limit_far_mps);" in terminal_runtime_cpp
    assert "publish_speed_limit_value(0.0);" not in terminal_runtime_cpp
    assert "msg.speed_limit = std::max(0.0, speed_limit_mps);" in terminal_runtime_cpp
    assert "msg.speed_limit = 0.0" not in terminal_runtime_cpp


def test_docking_predock_nav_reuses_terminal_speed_limit_loop():
    executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    predock_block = executor_cpp[
        executor_cpp.index("StoredPose predock_speed_limit_target"):
        executor_cpp.index("if (nav2_predock_succeeded)")
    ]

    assert "predock_speed_limit_target.x = job.approach_x;" in predock_block
    assert "predock_speed_limit_target.y = job.approach_y;" in predock_block
    assert "while (result_future.wait_for(200ms) != std::future_status::ready)" in predock_block
    assert predock_block.count(
        "publish_navigation_terminal_speed_limit_for_goal(predock_speed_limit_target);"
    ) >= 2
    assert "clear_navigation_terminal_speed_limit();" in predock_block


def test_nav2_terminal_reverse_is_leased_near_goal_and_hard_limited_by_safety():
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    execution_cpp = navigation_goal_execution_source()
    docking_feature_cpp = docking_feature_source()
    terminal_runtime_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    safety_cpp = (ROOT / "src" / "robot_safety" / "src" / "robot_safety_node.cpp").read_text(
        encoding="utf-8"
    )
    api_configs = (
        ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml",
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml",
    )
    safety_configs = (
        ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml",
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_safety.yaml",
    )
    dynamic_test = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "test_robot_safety_terminal_reverse_contract.sh"
    )

    for config_path in api_configs:
        config = config_path.read_text(encoding="utf-8")
        assert "navigation_terminal_reverse_permit_enabled: true" in config
        assert "navigation_terminal_reverse_permit_enter_distance_m: 0.30" in config
        assert "navigation_terminal_reverse_permit_exit_distance_m: 0.35" in config
        assert "navigation_terminal_reverse_permit_refresh_period_sec: 0.20" in config

    for config_path in safety_configs:
        config = config_path.read_text(encoding="utf-8")
        assert "allow_reverse: false" in config
        assert "normal_navigation_reverse_max_mps: 0.08" in config

    assert "update_navigation_terminal_reverse_permit_for_goal" in docking_feature_cpp
    assert execution_cpp.count("terminal_runtime_.update_reverse_permit_for_goal(") >= 5
    assert "config_.reverse_permit_enter_distance_m" in terminal_runtime_cpp
    assert "config_.reverse_permit_exit_distance_m" in terminal_runtime_cpp
    assert "config_.reverse_permit_refresh_period_sec" in terminal_runtime_cpp
    assert "navigation terminal reverse permit enabled" in terminal_runtime_cpp
    assert "navigation terminal reverse permit cleared" in terminal_runtime_cpp

    assert 'declare_parameter<double>("normal_navigation_reverse_max_mps", 0.08)' in safety_cpp
    assert "source == CommandSource::NORMAL" in safety_cpp
    assert "-navigation_limits.reverse_max_mps" in safety_cpp
    assert "return geometry_msgs::msg::Twist{};" in safety_cpp

    dynamic_test_text = dynamic_test.read_text(encoding="utf-8")
    assert "forbidden reverse arc was not rejected atomically" in dynamic_test_text
    assert "permitted reverse exceeded -0.08 m/s safety cap" in dynamic_test_text
    assert "expired reverse permit did not restore atomic rejection" in dynamic_test_text


def test_fine_docking_yaw_retry_uses_fine_entry_tolerance():
    predock_control_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.cpp"
    ).read_text(encoding="utf-8")
    yaw_align_block = predock_control_cpp[
        predock_control_cpp.index("PredockYawAlignResult PredockControlModule::run_yaw_align("):
        predock_control_cpp.index("PredockLateralAlignResult PredockControlModule::run_lateral_align(")
    ]
    fine_retry_block = predock_control_cpp[
        predock_control_cpp.index('fine_entry_failure_code == "FINE_DOCKING_REJECTED_YAW_TOO_LARGE"'):
        predock_control_cpp.index(
            "record_fine_entry(",
            predock_control_cpp.index(
                'fine_entry_failure_code == "FINE_DOCKING_REJECTED_YAW_TOO_LARGE"'
            ),
        )
    ]

    assert "const double success_tolerance_rad" in yaw_align_block
    assert "target_tolerance_rad" in yaw_align_block
    assert "initial_check.base_yaw_error_rad <= target_tolerance_rad" in yaw_align_block
    assert "result.final_error_rad <= target_tolerance_rad" in yaw_align_block
    assert "config_.fine_entry_max_yaw_rad" in fine_retry_block
    assert "run_yaw_align(" in fine_retry_block
    assert "fine_entry_check.expected_base_yaw" in fine_retry_block
    assert "config_.fine_entry_max_yaw_rad" in fine_retry_block


def test_predock_alignment_runs_once_after_bridge_settle_and_pause():
    predock_control_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.cpp"
    ).read_text(encoding="utf-8")
    yaw_align_block = predock_control_cpp[
        predock_control_cpp.index("PredockYawAlignResult PredockControlModule::run_yaw_align("):
        predock_control_cpp.index("PredockLateralAlignResult PredockControlModule::run_lateral_align(")
    ]
    handoff_start = predock_control_cpp.index(
        "bool PredockControlModule::start_fine_docking_handoff("
    )
    bridge_wait_start = predock_control_cpp.index(
        "if (!wait_for_bridge_smoothing(", handoff_start
    )
    pre_bridge_block = predock_control_cpp[handoff_start:bridge_wait_start]
    post_bridge_verify_start = predock_control_cpp.index(
        'job_store_.set_phase(job_id, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE")',
        bridge_wait_start,
    )
    fine_entry_start = predock_control_cpp.index(
        'job_store_.set_phase(job_id, "FINE_DOCKING_ENTRY_CHECK")',
        post_bridge_verify_start,
    )
    post_bridge_block = predock_control_cpp[bridge_wait_start:fine_entry_start]

    assert "run_yaw_align(" not in pre_bridge_block
    assert "ensure_lateral_alignment(" not in pre_bridge_block
    pause_call = "ports_.set_global_correction_paused("
    assert pause_call in post_bridge_block
    assert post_bridge_block.index(pause_call) < post_bridge_block.index(
        'job_store_.set_phase(job_id, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE")'
    )
    assert "run_yaw_align(" in post_bridge_block
    assert (
        "std::min(config_.yaw_align_tolerance_rad, config_.fine_entry_max_yaw_rad)"
        in post_bridge_block
    )
    assert "ensure_lateral_alignment(" in post_bridge_block
    assert "PREDOCK_YAW_ALIGN_MODE_SWITCH_TIMEOUT" in yaw_align_block
    assert "continuing predock yaw alignment" not in yaw_align_block


def test_nav2_local_costmap_frame_contract():
    nav2 = (ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml").read_text(encoding="utf-8")
    overlay_nav2 = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml").read_text(
        encoding="utf-8"
    )
    assert_nav2_local_costmap_frame_contract(nav2, local_costmap_config_block(nav2))
    assert_nav2_local_costmap_frame_contract(overlay_nav2, local_costmap_config_block(overlay_nav2))


def test_verify_nav2_local_costmap_frame_script_contract():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_nav2_local_costmap_frame.sh"
    ).read_text(encoding="utf-8")
    assert "read_nav2_value \"local_costmap.local_costmap.ros__parameters.global_frame\"" in script
    assert "param_value /local_costmap/local_costmap global_frame" in script
    assert "param_value /local_costmap/local_costmap robot_base_frame" in script
    assert "ros2 run tf2_ros tf2_echo odom base_link" in script
    assert "ros2 topic echo /local_state/odometry --once" in script
    assert "ros2 topic info -v /scan" in script
    assert "Node name: local_costmap" in script
    assert "Node name: collision_monitor" in script
    assert "ros2 lifecycle get /controller_server" in script


def test_verify_navigation_final_yaw_align_script_contract():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_navigation_final_yaw_align.sh"
    ).read_text(encoding="utf-8")
    assert "Default mode is dry-run and never sends a navigation goal." in script
    assert "--execute-goal" in script
    assert "--goal-json" in script
    assert "POST /api/v1/navigation/goal" in script
    assert "navigation_final_yaw_tolerance_rad" in script
    assert "navigation_final_yaw_align_trigger_rad" in script
    assert "navigation_final_yaw_align_timeout_sec" in script
    assert "navigation_final_yaw_align_cmd_topic" in script
    assert "navigation_final_yaw_align_bypass_collision_monitor" in script
    assert 'runtime_lc_frame="$(param_value /local_costmap/local_costmap global_frame || true)"' in script
    assert '[[ "$runtime_lc_frame" == "odom" ]]' in script
    assert '"/cmd_vel_safe"' in script
    assert '"/cmd_vel"' in script
    assert "FAST-LIO2-like node is present during navigation" in script
    assert "Node name: local_costmap" in script
    assert "Node name: collision_monitor" in script
    assert "final_pose_verified" in script
    assert "final_pose_verify_reason" in script
    assert "final_yaw_align_attempted" in script


def test_navigation_goal_diagnostic_records_bridge_status_contract():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "record_navigation_goal_diagnostic.sh"
    ).read_text(encoding="utf-8")

    assert "/localization/bridge_status" in script
    assert "echo_bridge_status" in script
    assert "ros2 topic echo --field data /localization/bridge_status" in script
    assert "--post-goal-file" in script
    assert "--data-binary @\"${POST_GOAL_FILE}\"" in script
    assert "/ranger_mini3/allow_reverse" in script
    assert "echo_nav_action_feedback" in script
    assert "/compute_path_to_pose/_action/status" in script
    assert "/compute_path_to_pose/_action/feedback" in script
    assert "/follow_path/_action/status" in script
    assert "/follow_path/_action/feedback" in script
    assert "ranger_allow_reverse_true_observed" in script
    assert 'stripped.startswith("data:")' in script
    assert "hz_bridge_status" in script
    assert "AMCL Bridge Corrections" in script
    assert "amcl_accepted_delta" in script
    assert "amcl_rejected_delta" in script
    assert "last_accepted_correction_translation_m" in script
    assert "last_accepted_correction_yaw_rad" in script
    assert "final_verify_retry_count" in script
    assert "task_complete" in script


def test_phase_n2_goal_completion_semantics_contracts():
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    navigation_module_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "navigation" / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    goal_executor_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "navigation" / "mission"
        / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    execution_cpp = navigation_goal_execution_source()
    navigation_config_cpp = navigation_configuration_source()
    localization_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_cpp = "\n".join(
        (api_cpp, navigation_config_cpp, navigation_module_cpp, goal_executor_cpp, execution_cpp)
    )
    terminal_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_control.cpp"
    ).read_text(encoding="utf-8")
    completion_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "mission"
        / "navigation_completion_policy.cpp"
    ).read_text(encoding="utf-8")
    bridge_wait_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "runtime"
        / "navigation_bridge_wait.cpp"
    ).read_text(encoding="utf-8")
    goal_policy_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "mission"
        / "navigation_goal_policy.cpp"
    ).read_text(encoding="utf-8")
    docking_job_hpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_model.hpp"
    ).read_text(encoding="utf-8")
    docking_job_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_model.cpp"
    ).read_text(
        encoding="utf-8"
    )
    predock_policy_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_alignment_policy.cpp"
    ).read_text(encoding="utf-8")
    docking_executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    predock_control_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.cpp"
    ).read_text(encoding="utf-8")
    docking_runtime_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "docking_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    docking_execution_cpp = "\n".join(
        (
            api_cpp,
            navigation_config_cpp,
            execution_cpp,
            docking_executor_cpp,
            predock_control_cpp,
            docking_runtime_cpp,
        )
    )
    config = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    verify_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_goal_completion_semantics.sh"
    ).read_text(encoding="utf-8")
    observe_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "observe_navigation_final_yaw_align.sh"
    ).read_text(encoding="utf-8")

    for token in (
        "goal_completion_policy",
        "position_only",
        "pose_required",
        "dock_staging",
        "yaw_align_required",
        "yaw_align_active",
        "yaw_align_failed",
        "final_pose_verified",
        "task_complete",
        "final_yaw_align_retry_count",
        "reposition_after_yaw_drift_retry_count",
    ):
        assert token in navigation_cpp

    assert '"navigation_default_goal_completion_policy", "pose_required"' in navigation_config_cpp
    assert '"navigation_delivery_point_goal_completion_policy", "pose_required"' in navigation_config_cpp
    assert '"navigation_position_only_nav2_yaw_mode", "approach_heading"' in navigation_config_cpp
    assert "stored_pose_type_uses_delivery_default" in goal_policy_cpp
    assert "resolve_nav2_goal_yaw(" in navigation_module_cpp
    assert "nav2_goal_yaw_source" in navigation_module_cpp
    assert "approach_heading" in goal_policy_cpp
    assert 'navigation_default_goal_completion_policy: "pose_required"' in config
    assert 'navigation_delivery_point_goal_completion_policy: "pose_required"' in config
    assert 'navigation_position_only_nav2_yaw_mode: "approach_heading"' in config
    assert "navigation_position_only_approach_heading_min_distance_m: 0.20" in config
    assert 'navigation_default_goal_completion_policy: "pose_required"' in overlay_config
    assert 'navigation_delivery_point_goal_completion_policy: "pose_required"' in overlay_config
    assert 'navigation_position_only_nav2_yaw_mode: "approach_heading"' in overlay_config
    assert "navigation_position_only_approach_heading_min_distance_m: 0.20" in overlay_config
    assert "navigation_max_reposition_after_yaw_retry: 0" in config
    assert "navigation_max_reposition_after_yaw_retry: 0" in overlay_config
    assert "navigation_reposition_after_yaw_drift_timeout_sec: 30.0" in config
    assert "navigation_reposition_after_yaw_drift_timeout_sec: 30.0" in overlay_config
    assert "nav2_native_goal_completion_enabled: true" in config
    assert "nav2_native_goal_completion_enabled: true" in overlay_config
    assert "nav2_rotation_shim_enabled: true" in config
    assert "nav2_rotation_shim_enabled: true" in overlay_config
    assert "api_final_yaw_align_fallback_enabled: true" in config
    assert "api_final_yaw_align_fallback_enabled: true" in overlay_config

    assert "goal_completion_policy=dock_staging is reserved for /api/v1/docking/start" in navigation_module_cpp
    assert "under position_only policy" in goal_executor_cpp
    assert "manual localization correction accepted; post-settle not requested" in localization_cpp
    assert 'json_bool_value(body, "wait_for_settle", false)' in localization_cpp
    assert "post_relocalization_settle_requested" in localization_cpp
    assert "post_relocalization_settle_ok" in localization_cpp
    assert "run_reposition_after_yaw_drift" in execution_cpp
    assert "REPOSITION_AFTER_YAW_DRIFT" in execution_cpp
    assert "nav2_failed_yaw_aligning" in goal_executor_cpp
    assert "navigation_pause_global_correction_during_final_yaw" in navigation_config_cpp
    assert "waiting for bridge map->odom smoothing before final yaw alignment" in execution_cpp
    assert "bridge smoothing wait timed out before final yaw alignment" in bridge_wait_cpp
    assert "navigation_nav2_failed_near_goal_retry_enabled" in navigation_cpp
    assert "nav2_failed_near_goal_retry_allowed" in execution_cpp
    assert "nav2_failed_near_goal_yaw_aligning" in goal_executor_cpp
    assert "aligning yaw before same-goal retry" in goal_executor_cpp
    assert "retry_nav2_after_nav2_failed_near_goal" in completion_cpp
    assert "near_goal_nav2_retry_attempted=true" in goal_executor_cpp
    assert "global correction paused for final_yaw_align" in execution_cpp
    assert "global correction resumed after final_yaw_align" in execution_cpp
    assert "commercial_final_verify=true" in goal_executor_cpp
    assert "degraded_final_pose_verify" in goal_executor_cpp
    assert "Nav2 native goal completion succeeded but final yaw alignment fallback failed" not in goal_executor_cpp
    assert '"failed_final_yaw_align"' not in goal_executor_cpp
    assert "navigation goal reached by commercial final verification" in goal_executor_cpp
    assert "position_reached_yaw_warning" not in goal_executor_cpp

    assert "ordinary_final_yaw_align_active_" in docking_execution_cpp
    assert "predock_yaw_align_active_" in docking_execution_cpp
    assert "predock_lateral_align_active_" in docking_execution_cpp
    assert "PREDOCK_YAW_ALIGN_OWNER_CONFLICT" in docking_execution_cpp
    assert "PREDOCK_LATERAL_ALIGN_OWNER_CONFLICT" in docking_execution_cpp
    assert "STAGING_NAV2_EARLY_HANDOFF" in docking_execution_cpp
    assert "predock native Nav2 early handoff to docking-owned yaw/lateral capture" in docking_execution_cpp
    assert "predock Nav2 goal canceled for docking-owned final capture" in docking_execution_cpp
    assert "docking request preempts ordinary final_yaw_align" in docking_execution_cpp
    assert 'final_yaw_align_cmd_topic != "/cmd_vel_nav"' in docking_execution_cpp
    assert 'final_yaw_align_cmd_topic != "/cmd_vel_api"' in docking_execution_cpp
    assert 'final_yaw_align_cmd_topic = "/cmd_vel_api"' in docking_execution_cpp
    assert (
        'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_collision_checked"'
        not in docking_execution_cpp
    )
    assert 'config_.command_topic != "/cmd_vel_docking"' in docking_execution_cpp
    assert 'navigation_final_yaw_align_cmd_topic: "/cmd_vel_docking"' not in config
    assert 'navigation_final_yaw_align_cmd_topic: "/cmd_vel_collision_checked"' not in config
    assert 'predock_yaw_align_cmd_topic: "/cmd_vel_docking"' in config
    assert 'predock_yaw_align_cmd_topic: "/cmd_vel_docking"' in overlay_config
    assert "predock_yaw_align_enabled: true" in config
    assert "predock_yaw_align_enabled: true" in overlay_config
    assert "predock_yaw_align_fallback_enabled: true" in config
    assert "predock_yaw_align_fallback_enabled: true" in overlay_config
    assert "predock_lateral_align_enabled: true" in config
    assert "predock_lateral_align_enabled: true" in overlay_config
    assert 'predock_lateral_align_forced_mode_topic: "/ranger_mini3/forced_mode"' in config
    assert 'predock_lateral_align_forced_mode_topic: "/ranger_mini3/forced_mode"' in overlay_config
    assert 'predock_lateral_align_forced_mode: "side_slip"' in config
    assert 'predock_lateral_align_forced_mode: "side_slip"' in overlay_config

    for token in (
        "goal_completion_policy",
        "predock_nav_early_handoff",
        "predock_nav_handoff_detail",
        "dock_staging_handoff_ready",
        "post_predock_settle_complete",
        "predock_pose_verified",
        "predock_yaw_verified_by_nav2",
        "reverse_yaw_offset_applied",
        "contact_frame_available",
        "predock_forward_m",
        "predock_lateral_m",
        "predock_lateral_abs_m",
        "ordinary_final_yaw_align_active",
        "predock_yaw_align_active",
        "predock_lateral_align_active",
        "predock_lateral_align_failure_code",
        "cmd_owner_conflict_detected",
        "final_yaw_align_blocked_by_docking",
        "docking_blocked_by_final_yaw_align",
    ):
        assert token in docking_job_hpp
        assert token in docking_job_cpp
        assert token in docking_execution_cpp or token in predock_policy_cpp

    assert "global_correction_pause_applied" in docking_execution_cpp
    assert (
        "config_.fine_entry_require_predock_yaw_aligned && !yaw_aligned"
        in docking_execution_cpp
    )
    assert "policy_.pose_inside_handoff_window(check)" in predock_control_cpp
    assert "PredockAlignmentPolicy::yaw_target_met" in predock_policy_cpp
    assert "PredockAlignmentPolicy::lateral_target_met" in predock_policy_cpp
    assert "config_.fine_retry_on_yaw_reject" in predock_control_cpp
    assert "FINE_DOCKING_REJECTED_YAW_TOO_LARGE" in docking_execution_cpp
    assert "PREDOCK_YAW_ALIGN" in docking_execution_cpp
    assert "PREDOCK_LATERAL_ALIGN" in docking_execution_cpp
    assert "RESTAGE_RETRY" in docking_execution_cpp
    assert "docking_max_retries: 2" in config
    assert "docking_max_retries: 2" in overlay_config

    assert "verify_goal_completion_semantics.sh" in verify_script
    assert "observe_navigation_final_yaw_align.sh" in verify_script
    assert "ros2 topic pub" not in observe_script
    assert "/api/v1/navigation/state" in observe_script


def test_phase_n3_nav2_native_goal_completion_contracts():
    nav2 = (ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml").read_text(encoding="utf-8")
    overlay_nav2 = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml"
    ).read_text(encoding="utf-8")
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    navigation_module_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "navigation" / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    goal_executor_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "navigation" / "mission"
        / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    execution_cpp = navigation_goal_execution_source()
    navigation_cpp = "\n".join(
        (api_cpp, navigation_module_cpp, goal_executor_cpp, execution_cpp)
    )
    docking_executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    predock_control_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.cpp"
    ).read_text(encoding="utf-8")
    docking_runtime_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "docking_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    docking_execution_cpp = "\n".join(
        (api_cpp, execution_cpp, docking_executor_cpp, predock_control_cpp, docking_runtime_cpp)
    )
    terminal_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_control.cpp"
    ).read_text(encoding="utf-8")
    config = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    verify_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_nav2_native_goal_completion.sh"
    ).read_text(encoding="utf-8")
    observe_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "observe_nav2_native_pose_required_goal.sh"
    ).read_text(encoding="utf-8")
    pose_bt = (ROOT / "src" / "robot_nav_config" / "behavior_trees" / "navigate_to_pose.xml").read_text(
        encoding="utf-8"
    )
    audit_reports = list((ROOT / "reports").glob("nav2_rotation_shim_compat_audit_*.md"))
    audit_text = "\n".join(p.read_text(encoding="utf-8") for p in audit_reports)

    for params in (nav2, overlay_nav2):
        assert 'plugin: "robot_nav_config::GoalScopedRotationShimController"' in params
        assert 'primary_controller: "robot_nav_config::RangerMPPIController"' in params
        assert "rotate_to_heading_once: true" in params
        assert "goal_change_xy_threshold: 0.01" in params
        assert "goal_change_yaw_threshold: 0.01" in params
        assert "rotate_to_goal_heading: true" in params
        assert "rotate_to_goal_heading: false" not in params
        assert "rotate_to_heading_angular_vel: 0.60" in params
        assert "max_angular_accel: 1.2" in params
        assert "terminal_rotation_braking_enabled: true" in params
        assert "closed_loop: true" in params
        assert 'goal_checker_plugins: ["goal_checker", "dock_staging_goal_checker"]' in params
        assert 'plugin: "nav2_controller::SimpleGoalChecker"' in params
        assert 'plugin: "nav2_controller::PositionGoalChecker"' not in params
        assert "goal_checker:" in params
        assert "pose_goal_checker:" not in params
        assert "position_goal_checker:" not in params
        assert "stateful: false" in params
        assert "stateful: true" not in params
        assert '".position_checker.stateful": false' in params
        assert "- nav2_rate_controller_bt_node" in params
        assert "xy_goal_tolerance: 0.06" in params
        assert "xy_goal_tolerance: 0.10" not in params
        assert "xy_goal_tolerance: 0.20" not in params
        assert "yaw_goal_tolerance: 0.05" in params
        assert "yaw_goal_tolerance: 0.0873" not in params
        assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in params
        assert 'plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"' in params
        assert "transform_tolerance: 0.10" in params
        assert "global_frame: odom" in params
        assert "/scan" in params
        assert "/perception/obstacle_points" not in params

    for params in (config, overlay_config):
        assert 'navigation_default_goal_completion_policy: "pose_required"' in params
        assert 'navigation_delivery_point_goal_completion_policy: "pose_required"' in params
        assert 'navigation_position_only_nav2_yaw_mode: "approach_heading"' in params
        assert "navigation_position_only_approach_heading_min_distance_m: 0.20" in params
        assert "nav2_native_goal_completion_enabled: true" in params
        assert "nav2_rotation_shim_enabled: true" in params
        assert "api_final_yaw_align_fallback_enabled: true" in params
        assert "navigation_final_yaw_align_enable: true" in params
        assert "navigation_nav2_failed_near_goal_retry_enabled: true" in params
        assert "navigation_nav2_failed_near_goal_retry_max_count: 1" in params
        assert "navigation_terminal_recovery_max_distance_m: 0.40" in params
        assert "navigation_final_yaw_align_wait_bridge_smoothing: true" in params
        assert "navigation_final_yaw_align_bridge_wait_timeout_ms: 2000" in params
        assert "navigation_final_yaw_align_bridge_wait_sample_period_ms: 100" in params
        assert "navigation_pause_global_correction_during_final_yaw: true" in params
        assert "yaw_align_stop_lead_enabled: true" in params
        assert "yaw_align_stop_lead_time_sec: 0.125" in params
        assert "yaw_align_stop_lead_max_rad: 0.09" in params
        assert "navigation_pose_required_behavior_tree:" not in params
        assert "navigation_position_only_behavior_tree:" not in params
        assert "predock_yaw_align_enabled: true" in params
        assert "predock_yaw_align_fallback_enabled: true" in params
        assert 'predock_yaw_align_cmd_topic: "/cmd_vel_docking"' in params
        assert "docking_predock_pose_max_yaw_rad: 0.35" in params
        assert "predock_yaw_align_tolerance_rad: 0.0698" in params
        assert "predock_yaw_align_trigger_rad: 0.0698" in params
        assert "predock_lateral_align_enabled: true" in params
        assert "predock_lateral_align_target_m: 0.03" in params
        assert "predock_lateral_align_trigger_m: 0.03" in params
        assert "predock_lateral_align_max_correction_m: 0.25" in params
        assert 'predock_lateral_align_forced_mode: "side_slip"' in params
        assert "predock_lateral_align_timeout_sec: 14.0" in params
        assert "predock_lateral_align_speed_mps: 0.04" in params
        assert "fine_docking_entry_max_lateral_m: 0.08" in params
        assert "fine_docking_entry_max_yaw_rad: 0.0349" in params
        assert 'navigation_final_yaw_align_cmd_topic: "/cmd_vel_docking"' not in params

    assert "native_nav2_goal_completion" in navigation_module_cpp
    assert "api_final_yaw_align_enabled" in navigation_module_cpp
    assert "nav2_rotation_shim_enabled" in navigation_module_cpp
    assert "navigation_behavior_tree_for_goal_completion_policy" not in navigation_cpp
    assert "goal.behavior_tree = navigation_behavior_tree_for_goal_completion_policy(goal_completion_policy);" not in navigation_cpp
    assert "commercial_final_verify=true" in goal_executor_cpp
    assert "navigation goal reached by commercial final verification" in goal_executor_cpp
    assert "navigation goal reached by Nav2 native completion" not in navigation_cpp
    run_goal_block = goal_executor_cpp
    assert "final_yaw_align_allowed" in run_goal_block
    assert "nav2_failed_yaw_aligning" in run_goal_block
    assert '"nav2_failed"' not in run_goal_block
    assert "run_final_yaw_align(job_id, target, pose_check)" in run_goal_block
    final_yaw_block = execution_cpp[
        execution_cpp.index("FinalYawAlignResult NavigationGoalExecutionModule::run_final_yaw_align("):
        execution_cpp.index(
            "NavigationRepositionResult NavigationGoalExecutionModule::run_reposition_after_yaw_drift("
        )
    ]
    assert "config_.pause_global_correction_during_final_yaw" in final_yaw_block
    assert "wait_for_bridge_smoothing_before_final_yaw_align(job_id)" in final_yaw_block
    assert "bridge_smoothing_active_before_final_yaw" in final_yaw_block
    assert "ports_.request_bridge_correction_pause(" in final_yaw_block
    assert "run_final_yaw_motion(" in final_yaw_block
    assert "final yaw alignment stopped because xy drift=" in terminal_cpp
    assert "final yaw aligned after actual angular velocity settled" in terminal_cpp
    assert "goal_completion_policy=dock_staging is reserved for /api/v1/docking/start" in navigation_module_cpp
    assert "PREDOCK_YAW_ALIGN_RECOVERY" in docking_execution_cpp
    assert "PREDOCK_LATERAL_ALIGN" in docking_execution_cpp
    assert "PREDOCK_NATIVE_GOAL_VERIFY_FAILED" in docking_execution_cpp
    assert "PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2" in docking_execution_cpp
    assert 'config_.command_topic != "/cmd_vel_docking"' in docking_execution_cpp
    assert "PREDOCK_YAW_ALIGN_OWNER_CONFLICT" in docking_execution_cpp
    assert "PREDOCK_LATERAL_ALIGN_OWNER_CONFLICT" in docking_execution_cpp
    assert (
        'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe"'
        not in docking_execution_cpp
    )
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel"' not in docking_execution_cpp

    assert "requires_nav2_rotation_shim_backport_or_upgrade`: false" in audit_text
    assert "RotationShimController is supported" in audit_text
    assert "GoalScopedRotationShimController" in verify_script
    assert "rotate_to_heading_once=true" in verify_script
    assert "rotate_to_goal_heading=true" in verify_script
    assert 'RateController hz="1.0"' in verify_script
    assert ".position_checker.stateful" in verify_script
    assert "api_final_yaw_align_fallback_enabled: true" in verify_script
    assert "/api/v1/navigation/state" in observe_script
    assert "Read-only observer" in observe_script
    assert "api_final_yaw_align_used" in observe_script
    assert "final_yaw_handled_by_nav2" in observe_script
    assert "cmd_owner_conflict" in observe_script
    assert 'goal_checker_id="goal_checker"' in pose_bt
    assert "position_goal_checker" not in pose_bt
    assert '<RateController hz="1.0">' in pose_bt
    assert not (ROOT / "src" / "robot_nav_config" / "behavior_trees" / "navigate_to_pose_position_only.xml").exists()


def test_phase_n4_post_nav2_final_verify_recovery_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"
    config_dir = overlay / "config"
    composition_cpp = application_composition_source()
    goal_executor_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "navigation" / "mission"
        / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    execution_cpp = navigation_goal_execution_source()
    navigation_config_cpp = navigation_configuration_source()
    localization_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    terminal_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_control.cpp"
    ).read_text(encoding="utf-8")
    terminal_runtime_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    goal_job_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "mission"
        / "navigation_goal_job.cpp"
    ).read_text(encoding="utf-8")
    bridge_wait_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "runtime"
        / "navigation_bridge_wait.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_cfg = (config_dir / "robot_api_server.yaml").read_text(encoding="utf-8")
    nav2_cfg = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    verify_path = scripts_dir / "verify_post_nav2_final_verify_recovery.sh"
    observe_path = scripts_dir / "observe_post_nav2_final_verify_recovery.sh"
    verify = verify_path.read_text(encoding="utf-8")
    observe = observe_path.read_text(encoding="utf-8")

    for token in (
        "post_nav2_final_verify_enabled",
        "final_pose_auditing",
        "commercial_final_verify=true",
        "navigation goal reached by commercial final verification",
        "commercial_final_verify_retry_attempted=true",
        "degraded_final_pose_verify",
        "final_verify_failure_is_terminal",
        "final_verify_retry_count",
        "final_verify_retry_reason",
        "final_verify_retry_goal_sent",
    ):
        assert (
            token in composition_cpp
            or token in navigation_config_cpp
            or token in goal_executor_cpp
            or token in execution_cpp
        )

    for params in (api_cfg, overlay_api_cfg):
        assert "navigation_goal_position_success_tolerance_m: 0.06" in params
        assert "post_nav2_final_verify_enabled: true" in params
        assert "post_nav2_final_verify_wait_bridge_smoothing: true" in params
        assert "post_nav2_final_verify_bridge_wait_timeout_ms: 2000" in params
        assert "post_nav2_final_verify_bridge_wait_sample_period_ms: 100" in params
        assert "post_nav2_final_verify_request_amcl_nomotion_update: true" in params
        assert 'post_nav2_final_verify_amcl_nomotion_update_service: "/request_nomotion_update"' in params
        assert "post_nav2_final_verify_max_retry_count: 3" in params
        assert "post_nav2_final_verify_acceptance_slack_m: 0.02" in params
        assert "post_nav2_final_verify_xy_retry_min_error_m: 0.06" in params
        assert "navigation_terminal_recovery_max_distance_m: 0.40" in params
        assert "post_nav2_final_verify_yaw_retry_if_failed: true" in params
        assert "post_nav2_final_verify_retry_uses_same_nav2_goal: true" in params
        assert "post_nav2_final_verify_api_velocity_correction_enabled: true" in params
        assert "post_nav2_final_verify_reverse_permit_enabled: true" in params
        assert 'post_nav2_final_verify_reverse_enable_topic: "/ranger_mini3/allow_reverse"' in params
        assert "post_nav2_final_verify_terminal_lateral_correction_enabled: true" in params
        assert "post_nav2_final_verify_terminal_lateral_target_m: 0.03" in params
        assert "post_nav2_final_verify_terminal_lateral_trigger_m: 0.04" in params
        assert "post_nav2_final_verify_terminal_lateral_max_forward_m: 0.15" in params
        assert "post_nav2_final_verify_terminal_lateral_speed_mps: 0.04" in params
        assert "post_nav2_final_verify_terminal_lateral_timeout_sec: 20.0" in params
        assert "navigation_terminal_recovery_costmap_guard_enabled: true" in params
        assert "navigation_terminal_recovery_costmap_max_age_sec: 0.50" in params
        assert "navigation_terminal_recovery_costmap_occupied_threshold: 50" in params
        assert "navigation_terminal_recovery_costmap_lookahead_m: 0.15" in params
        assert "post_nav2_final_verify_terminal_lateral_command_sign: 1.0" in params
        assert "post_nav2_final_verify_terminal_settle_enabled: true" in params
        assert "post_nav2_final_verify_terminal_settle_linear_speed_threshold_mps: 0.01" in params
        assert "post_nav2_final_verify_terminal_settle_angular_speed_threshold_radps: 0.02" in params
        assert "post_nav2_final_verify_terminal_settle_stable_duration_sec: 0.30" in params
        assert "post_nav2_final_verify_terminal_settle_timeout_sec: 2.50" in params
        assert "post_nav2_final_verify_terminal_settle_odom_max_age_sec: 0.20" in params
        assert "post_nav2_final_verify_terminal_settle_require_dual_ackermann_mode: true" in params
        assert "post_nav2_final_verify_terminal_settle_mode_status_max_age_sec: 0.50" in params
        assert "post_nav2_final_verify_terminal_settle_max_recheck_count: 1" in params
        assert "predock_lateral_align_command_sign: -1.0" in params
        assert "navigation_near_goal_stalled_handoff_enabled: false" in params
        assert "navigation_near_goal_stalled_handoff_distance_m" not in params
        assert "navigation_near_goal_stalled_handoff_min_wait_sec: 3.0" in params
        assert "navigation_near_goal_stalled_handoff_stall_sec: 1.5" in params
        assert "navigation_near_goal_stalled_handoff_improvement_epsilon_m: 0.02" in params
        assert "api_final_yaw_align_fallback_enabled: true" in params
        assert "navigation_final_yaw_align_enable: true" in params

    run_goal_block = goal_executor_cpp
    assert "nav2_failed_near_goal_retry_allowed(" in run_goal_block
    assert "nav2_failed_near_goal_yaw_first_candidate()" in run_goal_block
    assert "!nav2_failed_near_goal_yaw_first_candidate()" in run_goal_block
    assert "nav2_failed_near_goal_yaw_aligning" in run_goal_block
    assert "run_post_nav2_final_verify_retry(job_id, target, retry_reason, retry_phase)" in run_goal_block
    assert "terminal_publish_reverse_permit(true)" in terminal_cpp
    assert "terminal_publish_reverse_permit(false)" in terminal_cpp
    assert "msg.data = enabled && config_.post_nav2_reverse_permit_enabled;" in terminal_runtime_cpp
    assert "run_post_nav2_terminal_lateral_correction(" in execution_cpp
    assert "terminal_lateral_correction_attempted=true" in goal_executor_cpp
    assert "maybe_navigation_near_goal_stalled_handoff(" in execution_cpp
    assert "ports_.cancel_goal_for_handoff(" in execution_cpp
    assert "near_goal_nav2_stalled_handoff=true" in run_goal_block
    assert "waiting_for_nav2_result_near_goal_watch" in run_goal_block
    assert "post-Nav2 final verify retry handed off near goal before Nav2 result" in execution_cpp
    assert "terminal.lateral_forced_mode = inputs.lateral_forced_mode" in navigation_config_cpp
    assert (
        "docking_configuration.predock_control.lateral_align_forced_mode;"
        in composition_cpp
    )
    assert "runtime.terminal_publish_motion_mode(config_.lateral_forced_mode)" in terminal_cpp
    assert "terminal_publish_command(step.command)" in terminal_cpp
    assert "direction_reversed" in terminal_cpp
    assert "terminal lateral correction diverged after side-slip command" in terminal_cpp
    assert "terminal lateral correction kept original side-slip direction after meaningful progress" in terminal_cpp
    assert "reversal_min_progress_m" in terminal_cpp
    assert '"post_nav2_final_verify_terminal_lateral_command_sign", 1.0' in navigation_config_cpp
    assert '"navigation_terminal_recovery_max_distance_m", 0.40' in navigation_config_cpp
    assert '"post_nav2_final_verify_terminal_lateral_timeout_sec", 20.0' in navigation_config_cpp
    assert '"navigation_terminal_recovery_costmap_guard_enabled", true' in navigation_config_cpp
    assert "NavigationTerminalControl::costmap_path_clear(" in terminal_cpp
    assert "terminal recovery blocked by local costmap" in terminal_cpp
    assert '"post_nav2_final_verify_terminal_lateral_trigger_m", 0.04' in navigation_config_cpp
    assert '"post_nav2_final_verify_terminal_settle_enabled", true' in navigation_config_cpp
    handoff_block = execution_cpp[
        execution_cpp.index(
            "bool NavigationGoalExecutionModule::maybe_navigation_near_goal_stalled_handoff("
        ) : execution_cpp.index(
            "void NavigationGoalExecutionModule::update_post_nav2_bridge_wait_fields("
        )
    ]
    assert "post_nav2_terminal_lateral_correction_allowed(" in handoff_block
    assert "handoff requires executable terminal recovery" in handoff_block
    assert "navigation_near_goal_stalled_handoff_distance_m" not in navigation_config_cpp
    assert '"navigation_near_goal_stalled_handoff_min_wait_sec", 3.0' in navigation_config_cpp
    assert '"navigation_near_goal_stalled_handoff_stall_sec", 1.5' in navigation_config_cpp
    assert "terminal lateral correction reached strict settled pose gate" in terminal_cpp
    assert "TerminalCorrectionAxis::kYaw" in terminal_cpp
    assert "TerminalCorrectionAxis::kLateral" in terminal_cpp
    assert "TerminalCorrectionAxis::kForward" in terminal_cpp
    assert "terminal_pose_yaw_aligning" in terminal_cpp
    assert "terminal_pose_lateral_correcting" in terminal_cpp
    assert "terminal_pose_forward_correcting" in terminal_cpp
    assert "terminal pose correction: yaw first" in terminal_cpp
    assert "terminal pose correction: lateral second" in terminal_cpp
    assert "terminal pose correction: forward/reverse third" in terminal_cpp
    assert "terminal_lateral_candidate_before_salvage" in goal_executor_cpp
    assert "!terminal_lateral_candidate_before_salvage" in goal_executor_cpp
    assert "terminal pose correction skipped" in terminal_cpp
    assert "needs_terminal_xy_correction" in terminal_cpp
    assert "needs_terminal_yaw_correction" in terminal_cpp
    assert "lateral_hysteresis_entry" in terminal_cpp
    assert "config_.lateral_trigger_m" in terminal_cpp
    assert "config_.lateral_target_m * 0.5" in terminal_cpp
    assert "lateral_abs > config_.lateral_target_m" in terminal_cpp
    assert "strict_terminal_pose_reached" in terminal_cpp
    assert "terminal_wait_actual_stop(" in terminal_cpp
    assert "terminal_pose_settling" in terminal_cpp
    assert "settle_recheck_count" in terminal_cpp
    assert "actual_motion_mode_code == 0" in terminal_runtime_cpp
    assert "terminal_pose_correction_pending" in run_goal_block
    assert "!terminal_pose_correction_pending" in run_goal_block
    assert "NavigationTerminalControl::lateral_correction_diagnostics" in goal_executor_cpp
    terminal_allowed_block = execution_cpp[
        execution_cpp.index(
            "bool NavigationGoalExecutionModule::post_nav2_terminal_lateral_correction_allowed("
        ) : execution_cpp.index(
            "NavigationGoalExecutionModule::run_post_nav2_terminal_lateral_correction("
        )
    ]
    assert "check.yaw_error_rad > navigation_final_yaw_tolerance_rad_" not in terminal_allowed_block
    correction_step_block = terminal_cpp[
        terminal_cpp.index("TerminalCorrectionStep NavigationTerminalControl::correction_step(") :
        terminal_cpp.index("TerminalCostmapCheck NavigationTerminalControl::costmap_path_clear(")
    ]
    yaw_stage = correction_step_block[
        correction_step_block.index("if (correcting_yaw)") :
        correction_step_block.index("} else if (correcting_lateral)")
    ]
    assert "command.angular.z" in yaw_stage
    assert "command.linear.x =" not in yaw_stage
    assert "command.linear.y =" not in yaw_stage
    lateral_stage = correction_step_block[
        correction_step_block.index("} else if (correcting_lateral)") :
        correction_step_block.index("} else if (correcting_forward)")
    ]
    assert "command.linear.y" in lateral_stage
    assert "command.angular.z" not in lateral_stage
    forward_stage = correction_step_block[
        correction_step_block.index("} else if (correcting_forward)") :
        correction_step_block.index("return result;")
    ]
    assert "command.linear.x" in forward_stage
    assert "command.angular.z" not in forward_stage
    assert "near_goal_nav2_retry_attempted=true" in run_goal_block
    assert "post_nav2_final_verify_retry_allowed(" in run_goal_block
    assert "post_nav2_final_verify_acceptance_slack_allowed(" in run_goal_block
    assert "wait_for_bridge_smoothing_before_final_verify(job_id)" in run_goal_block
    final_verify_bridge_wait_block = execution_cpp[
        execution_cpp.index(
            "NavigationGoalExecutionModule::wait_for_bridge_smoothing_before_final_verify("
        ) : execution_cpp.index(
            "NavigationGoalExecutionModule::wait_for_bridge_smoothing_before_final_yaw_align("
        )
    ]
    assert "bridge_wait_.wait_before_final_verify(job_id, *this)" in final_verify_bridge_wait_block
    assert "request_amcl_nomotion_update_for_final_verify(" in execution_cpp
    final_verify_wait_module_block = bridge_wait_cpp[
        bridge_wait_cpp.index("NavigationBridgeWaitResult NavigationBridgeWait::wait_before_final_verify(") :
        bridge_wait_cpp.index("NavigationBridgeWaitResult NavigationBridgeWait::wait_before_final_yaw_align(")
    ]
    assert "runtime.request_bridge_wait_nomotion_update(initial_bridge, nomotion_detail)" in final_verify_wait_module_block
    assert "BridgeReadinessPurpose::kFinalPoseVerify" in final_verify_wait_module_block
    assert "BridgeReadinessPurpose::kGoalStart" not in final_verify_wait_module_block
    final_verify_gate_block = bridge_wait_cpp[
        bridge_wait_cpp.index("BridgeReadinessDecision evaluate_bridge_readiness(") :
        bridge_wait_cpp.index("NavigationBridgeWait::NavigationBridgeWait(")
    ]
    assert "amcl_static_pending_is_standby" in final_verify_gate_block
    assert "bridge.amcl_static_standby && bridge.amcl_not_moving_no_update_ok" in final_verify_gate_block
    assert re.search(
        r"bridge\.amcl_input_enabled\s*&&\s*bridge\.amcl_correction_pending\s*&&\s*"
        r"!amcl_static_pending_is_standby",
        final_verify_gate_block,
    )
    assert "AMCL static standby pending tolerated for final pose verify" in final_verify_gate_block
    assert "std_srvs/srv/empty.hpp" in localization_cpp
    assert "run_final_yaw_align(job_id, target, pose_check)" in run_goal_block
    final_yaw_block = execution_cpp[
        execution_cpp.index("FinalYawAlignResult NavigationGoalExecutionModule::run_final_yaw_align(") :
        execution_cpp.index(
            "NavigationRepositionResult NavigationGoalExecutionModule::run_reposition_after_yaw_drift("
        )
    ]
    assert "run_final_yaw_motion(" in final_yaw_block
    assert "result.final_yaw_error_rad <= config_.final_yaw_success_tolerance_rad" in terminal_cpp
    assert "yaw_stop_threshold(" in terminal_cpp
    assert "terminal_reset_yaw_actual_stop_stability()" in terminal_cpp
    assert "final_pose_bridge_ready &&" in run_goal_block
    assert "final_pose_bridge_wait_detail=" in run_goal_block
    assert "run_reposition_after_yaw_drift(job_id, target)" not in run_goal_block
    assert "api_final_yaw_align_enabled" not in run_goal_block
    assert '"failed_final_yaw_align"' not in run_goal_block
    assert "final_pose_auditing" in run_goal_block
    assert '"nav2_failed"' not in run_goal_block
    assert "commercial_final_verify=true" in run_goal_block
    assert "navigation goal reached by commercial final verification" in run_goal_block
    assert "degraded_final_pose_verify" in run_goal_block
    assert "final pose audit warning" not in run_goal_block
    assert run_goal_block.index("if (!commercial_position_complete || !commercial_yaw_complete)") < run_goal_block.index(
        "navigation goal reached by commercial final verification"
    )
    assert "/cmd_vel_docking" not in run_goal_block
    assert (
        "module.post_nav2_final_verify_api_velocity_correction_enabled ="
        in navigation_config_cpp
    )
    assert "publish_predock_yaw_align_command" not in run_goal_block
    assert 'degraded ? "degraded" : "failed"' in goal_job_cpp

    assert 'primary_controller: "robot_nav_config::RangerMPPIController"' in nav2_cfg
    assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in nav2_cfg
    assert "transform_tolerance: 0.10" in nav2_cfg
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    for forbidden in ("PointCloud2", "FAST-LIO", "ranger_base_node", "ekf_node"):
        assert forbidden not in run_goal_block
    assert "pkill -9" not in composition_cpp + verify + observe
    assert "killall -9" not in composition_cpp + verify + observe

    assert verify_path.exists()
    assert observe_path.exists()
    assert "commercial final verification" in verify
    assert "--mock-final-distance" in verify
    assert "--expect-task-complete" in verify
    assert "--expect-final-fail" in verify
    assert "Read-only observer" in observe
    assert "does not send navigation goals" in observe
    assert "final_yaw_align_attempted" in observe
    assert "final_verify_retry_count" in observe

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            subprocess.run([bash, "-n", str(verify_path)], check=True)
            subprocess.run([bash, "-n", str(observe_path)], check=True)


def test_terminal_recovery_envelope_covers_observed_predock_residual():
    """Keep measured 35-37 cm terminal residuals out of policy dead zones."""
    config_paths = (
        ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml",
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "robot_api_server.yaml",
    )
    observed_residuals = (
        # Original pre-dock hard failure.
        (0.354192, 0.126303, 0.330907, 0.206939),
        # Sixth repeated-route attempt: the former 0.35 m handoff/retry gate
        # would have rejected this executable side-slip correction.
        (0.366420, 0.012700, 0.366200, 0.027738),
    )

    for config_path in config_paths:
        params = config_path.read_text(encoding="utf-8")
        recovery_max_distance_m = yaml_number(
            params, "navigation_terminal_recovery_max_distance_m"
        )
        recovery_max_forward_m = yaml_number(
            params, "post_nav2_final_verify_terminal_lateral_max_forward_m"
        )
        lateral_target_m = yaml_number(
            params, "post_nav2_final_verify_terminal_lateral_target_m"
        )
        correction_speed_mps = yaml_number(
            params, "post_nav2_final_verify_terminal_lateral_speed_mps"
        )
        correction_timeout_sec = yaml_number(
            params, "post_nav2_final_verify_terminal_lateral_timeout_sec"
        )

        for observed_distance_m, observed_forward_m, observed_lateral_m, observed_yaw_error_rad in observed_residuals:
            assert observed_distance_m <= recovery_max_distance_m
            assert abs(observed_forward_m) <= recovery_max_forward_m

            # Axis-staged recovery performs yaw, lateral, and forward
            # corrections serially. The timeout must cover each captured
            # residual at configured speeds, plus physical settle.
            correction_motion_sec = (
                max(0.0, abs(observed_lateral_m) - lateral_target_m)
                + max(0.0, abs(observed_forward_m) - lateral_target_m * 0.5)
            ) / correction_speed_mps
            correction_motion_sec += max(0.0, observed_yaw_error_rad - 0.045) / 0.20
            required_timeout_sec = correction_motion_sec + 2.5
            assert correction_timeout_sec >= required_timeout_sec

        # One canonical distance owns final retry, failed-Nav2 recovery, and
        # direct terminal correction. Independent maxima caused the 30-35 cm
        # uncovered interval reproduced on hardware.
        assert "post_nav2_final_verify_xy_retry_max_error_m:" not in params
        assert "post_nav2_final_verify_terminal_lateral_max_xy_m:" not in params
        assert "navigation_nav2_failed_near_goal_retry_max_distance_m:" not in params
        assert "navigation_near_goal_stalled_handoff_distance_m:" not in params


def test_manual_relocalization_keeps_amcl_post_isaac_refine_explicit():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    localization_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(
        encoding="utf-8"
    )
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_cfg = (overlay / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    bridge_cfg = (
        ROOT / "src" / "robot_localization_bridge" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    overlay_bridge_cfg = (overlay / "config" / "localization_bridge.yaml").read_text(
        encoding="utf-8"
    )
    bridge_cpp = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")

    trigger_block = localization_cpp[
        localization_cpp.index("HttpResponse handle_trigger_localization_body") :
        localization_cpp.index("void handle_localization_result")
    ]
    refine_wait_block = localization_cpp[
        localization_cpp.index("bool wait_for_manual_relocalization_amcl_refine(") :
        localization_cpp.index("bool wait_for_localization_result_after(")
    ]
    bridge_status_block = localization_cpp[
        localization_cpp.index("void handle_localization_bridge_status") :
        localization_cpp.index("void handle_tf_static_message")
    ]

    assert "wait_for_manual_relocalization_amcl_refine(" in trigger_block
    assert "manual_relocalization_amcl_refine_requested" in trigger_block
    assert "manual_relocalization_amcl_refine_ok" in trigger_block
    assert "manual_relocalization_amcl_refine_detail" in trigger_block
    assert '"amcl_refine", config_.manual_amcl_refine_enabled' in trigger_block
    assert (
        '"amcl_refine_required", config_.manual_amcl_refine_required'
    ) in trigger_block
    assert "request_amcl_nomotion_update(" in refine_wait_block
    assert '"manual_relocalization_amcl_refine"' in refine_wait_block
    assert "bridge.amcl_post_isaac_refined_sequence == relocalization_sequence" in refine_wait_block
    assert "refine_fully_applied" in refine_wait_block
    assert "bridge.last_published_sequence >= bridge.current_sequence" in refine_wait_block
    assert "bridge_owns_nomotion_requests" in refine_wait_block
    assert "!bridge_owns_nomotion_requests" in refine_wait_block
    assert "manual relocalization AMCL refine timed out" in refine_wait_block
    assert "json_uint64_value" in bridge_status_block
    assert '"amcl_post_isaac_refined_sequence"' in bridge_status_block
    assert '"amcl_post_isaac_refine_candidate_count"' in bridge_status_block
    assert "json_string_value" in bridge_status_block
    assert '"amcl_last_reject_reason"' in bridge_status_block

    for cfg in (api_cfg, overlay_api_cfg):
        assert "manual_relocalization_amcl_refine_enabled: true" in cfg
        assert "manual_relocalization_amcl_refine_required: true" in cfg
        assert "manual_relocalization_amcl_refine_timeout_sec: 4.0" in cfg
        assert "manual_relocalization_amcl_refine_poll_ms: 100" in cfg
        assert "manual_relocalization_amcl_refine_request_period_ms: 500" in cfg

    for cfg in (bridge_cfg, overlay_bridge_cfg):
        assert "amcl_initial_pose_xy_covariance: 0.01" in cfg
        assert "amcl_initial_pose_yaw_covariance: 0.0076" in cfg
        assert "amcl_post_isaac_refine_enabled: true" in cfg
        assert "amcl_post_isaac_refine_max_translation_m: 10.0" in cfg
        assert "amcl_post_isaac_refine_max_yaw_rad: 0.872664626" in cfg

    assert '"amcl_post_isaac_refine_max_translation_m", 10.0' in bridge_cpp
    assert '"amcl_post_isaac_refine_max_yaw_rad", 0.872664626' in bridge_cpp


def test_post_isaac_amcl_refine_cannot_preempt_the_unsettled_isaac_target():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    bridge_cpp = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    refine_gate_hpp = (
        ROOT
        / "src"
        / "robot_localization_bridge"
        / "include"
        / "robot_localization_bridge"
        / "post_isaac_refine_gate.hpp"
    ).read_text(encoding="utf-8")
    global_cpp = (
        ROOT / "src" / "robot_global_localization" / "src" / "global_localization_node.cpp"
    ).read_text(encoding="utf-8")
    bridge_cfg = (
        ROOT / "src" / "robot_localization_bridge" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    overlay_bridge_cfg = (overlay / "config" / "localization_bridge.yaml").read_text(
        encoding="utf-8"
    )

    amcl_callback = bridge_cpp[
        bridge_cpp.index("void on_amcl_pose(") : bridge_cpp.index("void on_odom(")
    ]
    refine_gate = bridge_cpp[
        bridge_cpp.index("PostIsaacRefineDisposition post_isaac_refine_disposition(") :
        bridge_cpp.index("bool post_isaac_refine_candidate_agrees(")
    ]
    bridge_wait = global_cpp[
        global_cpp.index("bool wait_for_bridge_acceptance(") :
        global_cpp.index("bool bridge_reject_is_transient_triggered_stale(")
    ]

    assert amcl_callback.index("post_isaac_refine_disposition(") < amcl_callback.index(
        "build_candidate("
    )
    assert "explicit_isaac_target_settled()" in refine_gate
    assert "pose_stamp_sec" in refine_gate
    assert "input.refine_reference_sec + input.min_pose_stamp_delta_sec" in refine_gate_hpp
    assert "input.received_sec < input.refine_reference_sec" in refine_gate_hpp
    assert "AMCL_POST_ISAAC_REFINE_WAITING_FOR_ISAAC_SETTLE" in amcl_callback
    assert "AMCL_POST_ISAAC_REFINE_PRESEED_POSE" in amcl_callback
    assert "maybe_request_post_isaac_nomotion_update()" in bridge_cpp
    assert 'create_client<std_srvs::srv::Empty>' in bridge_cpp
    assert '"amcl_post_isaac_refine_nomotion_update_service"' in bridge_cpp
    assert "should_request_post_isaac_nomotion_update" in refine_gate_hpp

    assert "latest.safe_for_goal_start" in bridge_wait
    assert "!latest.correction_active" in bridge_wait
    assert "latest.current_sequence == latest.target_sequence" in bridge_wait
    assert "bridge_explicit_relocalization_target_settled(latest)" in bridge_wait

    for cfg in (bridge_cfg, overlay_bridge_cfg):
        assert "amcl_post_isaac_refine_min_delay_sec: 0.25" in cfg
        assert "amcl_post_isaac_refine_min_pose_stamp_delta_sec: 0.0" in cfg
        assert "amcl_post_isaac_refine_request_nomotion_update: true" in cfg
        assert "amcl_post_isaac_refine_nomotion_request_period_sec: 0.5" in cfg
        assert "amcl_post_isaac_refine_nomotion_max_requests: 4" in cfg


def test_nav2_rotation_progress_scripts_contract():
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    ab = (scripts_root / "run_nav2_rotation_shim_ab.sh").read_text(encoding="utf-8")
    diagnose = (scripts_root / "diagnose_nav2_zero_linear_progress_failure.sh").read_text(encoding="utf-8")
    verify = (scripts_root / "verify_nav2_progress_checker_config.sh").read_text(encoding="utf-8")

    assert "pose_progress_only" in ab
    assert "relaxed_shim_1p8" in ab
    assert "relaxed_shim_2p2" in ab
    assert "no_start_shim_diagnostic" in ab
    assert "--restore" in ab
    assert "--apply" in ab
    assert "--restart" in ab
    assert "nav2_controller::PoseProgressChecker" in ab
    assert "required_movement_angle=0.05" in ab
    assert "movement_time_allowance=12.0" in ab
    assert "pointcloud, DDS/RMW, EKF, FAST-LIO2, App API" in ab
    assert "pkill -9" not in ab
    assert "kill -9" not in ab

    assert "Default mode only records existing topics" in diagnose
    assert "--execute-goal" in diagnose
    assert "--bag" in diagnose
    assert "--capture-tf true|false" in diagnose
    assert "CAPTURE_TF=false" in diagnose
    assert "timeout --kill-after" in diagnose
    assert "setsid bash -c" in diagnose
    assert "write_dds_env_log" in diagnose
    assert 'if [[ "${CAPTURE_TF}" == "true" ]]' in diagnose
    assert "skipped; rerun with --capture-tf true" in diagnose
    assert "reports/nav2_zero_linear_progress_" in diagnose
    assert "/cmd_vel_nav_raw" in diagnose
    assert "/cmd_vel_nav" in diagnose
    assert "/cmd_vel_collision_checked" in diagnose
    assert "/cmd_vel_safe" in diagnose
    assert "/cmd_vel" in diagnose
    assert "/wheel/odom" in diagnose
    assert "/local_state/odometry" in diagnose
    assert "ros2 topic hz /scan" in diagnose
    assert "ros2 topic hz /local_costmap/costmap" in diagnose
    assert "timeout --signal=INT" not in diagnose
    assert "CASE_A_CONTROLLER_ZERO_LINEAR" in diagnose
    assert "CASE_B_COLLISION_ZERO_LINEAR" in diagnose
    assert "CASE_C_SAFETY_ZERO_LINEAR" in diagnose
    assert "CASE_D_MODE_CONTROLLER_OR_CHASSIS_NOT_EXECUTING" in diagnose
    assert "CASE_E_ODOM_NOT_REFLECTING_MOTION" in diagnose
    assert "CASE_F_ROTATION_PROGRESS_ONLY" in diagnose
    assert "CASE_G_ROTATION_STALL" in diagnose

    assert "progress_checker.plugin" in verify
    assert "progress_checker.required_movement_angle" in verify
    assert "robot_nav_config::ElevatorAwareProgressChecker" in verify
    assert "progress_checker.elevator_required_movement_radius" in verify
    assert "progress_checker.elevator_required_movement_angle" in verify
    assert "progress_checker.elevator_movement_time_allowance" in verify
    assert "progress_checker.elevator_heartbeat_timeout" in verify
    assert "progress_checker.elevator_heartbeat_topic" in verify
    assert "for attempt in 1 2" in verify
    assert "FollowPath.rotate_to_goal_heading" in verify
    assert "FollowPath.angular_dist_threshold" in verify
    assert "FollowPath.angular_disengage_threshold" in verify
    assert "local_costmap.global_frame=odom" in verify
    assert "/cmd_vel_safe" in verify
    assert "/cmd_vel" in verify


def test_tf_policy_is_canonical():
    tf_policy = (ROOT / "src" / "robot_nav_config" / "config" / "tf_policy.yaml").read_text(encoding="utf-8")
    assert "robot_localization_bridge" in tf_policy
    assert "robot_local_state" in tf_policy
    assert "suppress_third_party_tf: true" in tf_policy


def test_runtime_health_guard_replaces_hot_readiness_probes():
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    bringup = ROOT / "src" / "robot_bringup"
    guard = (bringup / "src" / "runtime_health_guard.cpp").read_text(encoding="utf-8")
    checker = (bringup / "src" / "runtime_health_check.cpp").read_text(encoding="utf-8")
    cmake = (bringup / "CMakeLists.txt").read_text(encoding="utf-8")
    helpers = (scripts_root / "runtime_health_helpers.sh").read_text(encoding="utf-8")
    runner = (scripts_root / "run_runtime_health_guard.sh").read_text(encoding="utf-8")
    common = (scripts_root / "run_common_services.sh").read_text(encoding="utf-8")
    local_state_runner = (scripts_root / "run_local_state.sh").read_text(encoding="utf-8")
    canonical = (scripts_root / "canonical_tf_helpers.sh").read_text(encoding="utf-8")
    nav_helpers = (scripts_root / "nav_runtime_helpers.sh").read_text(encoding="utf-8")
    container_tool = (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(encoding="utf-8")
    cpu_affinity = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "cpu_affinity.env"
    ).read_text(encoding="utf-8")

    assert not (scripts_root / "runtime_health_guard.py").exists()
    assert 'Node("runtime_health_guard"' in guard
    assert '"/local_state/odometry"' in guard
    assert '"/fastlio/base_odometry"' in guard
    assert '"/scan"' in guard
    assert '"/dock/target_observation"' in guard
    assert "DockTargetObservation" in guard
    assert '"docking_sensor_healthy"' in guard
    assert '"always_observed"' in guard
    assert guard.count('observe<nav_msgs::msg::Odometry>("/local_state/odometry"') == 1
    assert '"/dock/target_observation",' in guard
    assert "NJRH_RUNTIME_HEALTH_OBSERVE_TOPIC_MESSAGES" in guard
    assert "NJRH_RUNTIME_HEALTH_OBSERVE_HEAVY_TOPICS" in guard
    assert "if (messages_)" in guard
    assert "if (heavy_)" in guard
    assert 'NJRH_RUNTIME_HEALTH_SAMPLE_PERIOD_SEC", 1.0' in guard
    assert 'NJRH_RUNTIME_HEALTH_GRAPH_PERIOD_SEC", 5.0' in guard
    assert 'NJRH_RUNTIME_HEALTH_WRITE_PERIOD_SEC", 1.0' in guard
    assert 'NJRH_RUNTIME_HEALTH_ODOM_NO_UPDATE_TIMEOUT_SEC", 3.0' in guard
    assert 'NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC", 0.75' in guard
    assert "rclcpp::QoS qos(1)" in guard
    assert "CallbackGroupType::MutuallyExclusive, false" in guard
    assert "sub->take(msg, info)" in guard
    assert "rclcpp::spin" not in guard
    assert '"sampled_messages_not_publisher_rate"' in guard
    assert "NJRH_RUNTIME_HEALTH_OBSERVE_TF" in guard
    assert "NJRH_RUNTIME_HEALTH_OBSERVE_ALL_TF" in guard
    assert "NJRH_RUNTIME_HEALTH_TF_TRACKED_EDGES" in guard
    assert '"map->odom,odom->base_link"' in guard
    assert "if (tf_)" in guard
    assert "tf_tracking" in guard
    assert "topic_tracking" in guard
    assert '"observe_topic_messages", messages_' in guard
    assert '"observe_tf", tf_' in guard
    assert "tracked_.count(key)" in guard
    assert '"local_state_topic_ready"' in guard
    assert guard.count('create_subscription<tf2_msgs::msg::TFMessage>("/tf"') == 1
    assert '"/tf"' in guard
    assert '"schema", "njrh.runtime_health.v1"' in guard
    assert '"implementation", "cpp"' in guard
    assert '"odom_watch"' in guard
    assert '"generation"' in guard and '"sequence"' in guard
    assert '"updated_monotonic_sec"' in guard and '"boot_id"' in guard
    assert "rename(pattern.c_str(), path.c_str())" in guard
    assert "--once" in guard
    assert "runtime_health_check()" in helpers
    assert "runtime_health_local_state_diagnostic()" in helpers
    assert 'runtime_health_query diagnostic' in helpers
    assert 'NJRH_RUNTIME_HEALTH_CHECK_BIN' in helpers
    assert 'install/robot_bringup/lib/robot_bringup/runtime_health_check' in helpers
    assert 'python' not in helpers
    assert '"observer_stale"' in checker
    assert 'emit("odom_stamp_stale", 53' in checker
    assert 'emit("observer_sampling_warmup", 45' in checker
    assert 'emit("odom_no_fresh_update", 56' in checker
    assert "runtime_health_fresh_tf_ready()" in helpers
    assert "runtime_health_topic_message_ready()" in helpers
    assert 'runtime_health_query topic "$1"' in helpers
    assert "runtime_health_tf_seen()" in helpers
    assert "NJRH_RUNTIME_HEALTH_TF_SEEN_MAX_AGE_SEC" in helpers
    assert 'njrh_exec_affined runtime_health_guard "${guard_binary}"' in runner
    assert 'python' not in runner
    assert 'exec njrh_exec_affined runtime_health_guard' not in runner
    assert 'add_executable(runtime_health_guard src/runtime_health_guard.cpp)' in cmake
    assert 'add_executable(runtime_health_check src/runtime_health_check.cpp)' in cmake
    assert re.search(r'install\(TARGETS[^)]*\bruntime_health_guard\b[^)]*\bruntime_health_check\b', cmake)
    assert not re.search(r'(?:ament_target_dependencies|target_link_libraries)\(runtime_health_check\b', cmake)
    assert not re.search(r'#include\s*[<"](?:rclcpp|rmw|dds|fastdds)[/\.]', checker)
    assert "NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART:-true" in common
    assert 'start_common_process "runtime_health_guard"' in common
    assert "verify_docking_sensor_common_health_or_exit" in common
    assert "verify_robot_local_state_common_health_or_exit" in common
    assert "runtime_health_local_state_diagnostic" in common
    assert "NJRH_COMMON_LOCAL_STATE_HEALTH_MAX_FAILURES:-3" in common
    assert "runtime health observer degraded" in common
    assert "does not authorize complete-chain recovery" in common
    assert "robot_local_state fault confirmed" in common
    assert "systemd restarts the complete navigation chain" in common
    assert "start_runtime_health_guard_common()" in common
    assert 'health_file="$(runtime_health_file)"' in common
    assert 'rm -f "${health_file}"' in common
    assert "wait_for_runtime_health_local_state_ready()" in common
    assert 'runtime_health_check "local_state_ready"' in common
    assert 'runtime_health_check "local_state_topic_ready"' in common
    assert 'runtime_health_check "local_state_endpoint"' in common
    assert "runtime health confirms local_state_ready before resident navigation autostart" in common
    assert "runtime health confirms local_state_topic_ready before resident navigation autostart" in common
    assert "runtime health confirms local_state_endpoint before resident navigation autostart" in common
    assert "continuing because robot_local_state direct readiness already passed" in common
    health_wait_block = common[
        common.index("wait_for_runtime_health_local_state_ready()") :
        common.index("wait_for_runtime_health_local_state_endpoint_ready()")
    ]
    assert "return 1" not in health_wait_block
    common_main_flow = common[common.index("require_can_interface_up\n") :]
    assert common_main_flow.index("NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART") < common_main_flow.index(
        "wait_for_robot_local_state_common_background_if_started"
    )
    assert common_main_flow.index("NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART") < common_main_flow.index(
        "local_perception_common disabled"
    )
    assert common_main_flow.index("wait_for_runtime_health_local_state_endpoint_ready") < common_main_flow.index(
        "resident_navigation_started"
    )
    assert 'source "${SCRIPT_DIR}/runtime_health_helpers.sh"' in canonical
    assert 'source "${SCRIPT_DIR}/runtime_health_helpers.sh"' in nav_helpers
    assert 'runtime_health_check "local_state_fastlio_endpoint"' in canonical
    assert 'runtime_health_check "local_state_ready"' not in canonical
    assert 'runtime_health_topic_message_ready "${topic}"' in nav_helpers
    assert 'runtime_health_fresh_tf_ready "${target_frame}" "${source_frame}" "${max_age_sec}"' in nav_helpers
    assert "local costmap observation ready from runtime health snapshot" not in nav_helpers
    assert "wait_for_transformable_local_scan" in nav_helpers
    assert 'export NJRH_CPUSET_TF_STATE="${NJRH_CPUSET_TF_STATE:-2}"' in cpu_affinity
    assert 'export NJRH_CPUSET_COLLISION_MONITOR="${NJRH_CPUSET_COLLISION_MONITOR:-${NJRH_CPUSET_BASE_CONTROL}}"' in cpu_affinity
    assert (
        'export NJRH_CPUSET_RUNTIME_HEALTH_GUARD="${NJRH_CPUSET_RUNTIME_HEALTH_GUARD:-${NJRH_CPUSET_SYSTEM}}"'
        in cpu_affinity
    )
    assert "NJRH_COMMON_STOP_INT_WAIT_SEC" in container_tool
    assert "NJRH_COMMON_STOP_TERM_WAIT_SEC" in container_tool
    assert 'kill -TERM "${pid}"' in container_tool
    assert 'kill -KILL "${pid}"' in container_tool
    assert 'LOCAL_STATE_RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS' in local_state_runner
    assert 'env RMW_FASTRTPS_PUBLICATION_MODE="${LOCAL_STATE_RMW_FASTRTPS_PUBLICATION_MODE}"' in local_state_runner


def test_runtime_health_observer_failure_cannot_restart_the_complete_runtime(tmp_path):
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    helpers = (scripts_root / "runtime_health_helpers.sh").read_text(encoding="utf-8")
    common = (scripts_root / "run_common_services.sh").read_text(encoding="utf-8")

    assert "runtime_health_local_state_diagnostic()" in helpers
    checker = (ROOT / "src/robot_bringup/src/runtime_health_check.cpp").read_text(encoding="utf-8")
    assert "observer_stale" in checker
    assert "observer_graph_inconsistent" in checker

    match = re.search(
        r"(?ms)^verify_robot_local_state_common_health_or_exit\(\) \{\n.*?^\}\n",
        common,
    )
    assert match is not None
    verifier = match.group(0)
    assert "runtime_health_local_state_diagnostic" in verifier
    assert "observer degraded" in verifier
    assert "does not authorize complete-chain recovery" in verifier
    assert 'runtime_health_check "local_state_topic_ready"' not in verifier
    assert '40|41|42|43|44|45|46)' in verifier
    assert '50|51|52|53|54|55|56)' in verifier
    assert 'evidence_id=' in verifier

    docking_match = re.search(
        r"(?ms)^verify_docking_sensor_common_health_or_exit\(\) \{\n.*?^\}\n",
        common,
    )
    assert docking_match is not None
    docking_verifier = docking_match.group(0)
    assert "runtime_health_available" in docking_verifier
    assert "docking health observer unavailable/stale" in docking_verifier
    assert "does not authorize complete-chain recovery" in docking_verifier
    assert "docking manager remains fail-closed; common runtime stays alive" in docking_verifier
    assert "return 1" not in docking_verifier
    assert docking_verifier.index("runtime_health_available") < docking_verifier.index(
        'runtime_health_check "docking_sensor_healthy"'
    )

    bash_candidates = []
    if sys.platform == "win32":
        bash_candidates.extend(
            (
                Path(r"C:\Program Files\Git\bin\bash.exe"),
                Path(r"C:\Program Files\Git\usr\bin\bash.exe"),
            )
        )
    discovered_bash = shutil.which("bash")
    if discovered_bash:
        bash_candidates.append(Path(discovered_bash))
    bash = next((candidate for candidate in bash_candidates if candidate.exists()), None)
    if bash is None:
        pytest.skip("bash is required for the runtime-health recovery policy regression")

    cli_tests_path = ROOT / "src/robot_bringup/test/test_runtime_health_check.py"
    spec = importlib.util.spec_from_file_location("health_cli_test_support", cli_tests_path)
    cli_tests = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli_tests)
    binary = cli_tests.resolve_runtime_health_check(tmp_path / "cli_build")

    harness = f"""
set -euo pipefail
robot_local_state_common_health_failures=0
robot_local_state_fault_generation=""
robot_local_state_fault_sequence=0
runtime_health_observer_failures=0
docking_sensor_common_health_failures=0
docking_health_observer_failures=0
DOCKING_SENSOR_BACKEND=orbbec_336l
export NJRH_COMMON_LOCAL_STATE_HEALTH_MONITOR=true
export NJRH_COMMON_LOCAL_STATE_HEALTH_MAX_FAILURES=3
export NJRH_COMMON_DOCKING_SENSOR_HEALTH_MONITOR=true
export NJRH_COMMON_DOCKING_SENSOR_HEALTH_MAX_FAILURES=3
health_file="$(mktemp)"
trap 'rm -f "${{health_file}}"' EXIT
export NJRH_RUNTIME_HEALTH_FILE="${{health_file}}"
helper_path="$1"
if command -v cygpath >/dev/null 2>&1; then
  helper_path="$(cygpath -u "${{helper_path}}")"
fi
source "${{helper_path}}"
python_executable="$2"
if command -v cygpath >/dev/null 2>&1; then
  python_executable="$(cygpath -u "${{python_executable}}")"
fi
python3() {{
  "${{python_executable}}" "$@"
}}
# These functions are extracted in isolation. Never source the common main flow.
systemctl() {{ echo "unexpected systemctl call" >&2; exit 90; }}
ros2() {{ echo "unexpected ROS call" >&2; exit 91; }}
ssh() {{ echo "unexpected SSH call" >&2; exit 92; }}
docker() {{ echo "unexpected Docker call" >&2; exit 93; }}
kill() {{ echo "unexpected signal to runtime" >&2; exit 94; }}
checker_binary="$3"
if command -v cygpath >/dev/null 2>&1; then
  checker_binary="$(cygpath -u "${{checker_binary}}")"
fi
export NJRH_RUNTIME_HEALTH_CHECK_BIN="${{checker_binary}}"
export NJRH_RUNTIME_HEALTH_MAX_AGE_SEC=2.0
export NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC=0.75
TEST_DIRECT_CONFIRM_CALLS=0
local_state_required_processes_running() {{
  [[ "${{TEST_REQUIRED_PROCESSES_RUNNING:-true}}" == "true" ]]
}}
direct_local_state_odom_ready_for_health_confirmation() {{
  TEST_DIRECT_CONFIRM_CALLS=$((TEST_DIRECT_CONFIRM_CALLS + 1))
  [[ "${{TEST_DIRECT_LOCAL_STATE_READY:-false}}" == "true" ]]
}}
refresh_legacy_snapshot() {{
  python3 - "${{health_file}}" <<'PY'
import json, sys, time
path = sys.argv[1]
with open(path, encoding="utf-8") as stream:
    snapshot = json.load(stream)
snapshot["updated_at"] = time.time()
with open(path, "w", encoding="utf-8") as stream:
    json.dump(snapshot, stream)
PY
}}
{verifier}
{docking_verifier}

# A stale/missing observer is not evidence that robot_local_state failed. It
# must never consume the real-fault retry budget or exit the common owner.
printf '%s\n' '{{"updated_at":1,"summary":{{"local_state_endpoint_ready":true,"local_state_topic_ready":true,"local_odom_fresh":true}},"topics":{{"/local_state/odometry":{{"publishers":1,"last_received_at":1,"last_age_sec":0.03,"message_count":100}}}}}}' >"${{health_file}}"
for _ in 1 2 3 4; do
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# Invalid snapshots are observer failures, even across repeated reads.
printf '%s\\n' '{{broken' >"${{health_file}}"
for _ in 1 2 3 4; do
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# A fresh odometry callback wins over a temporarily inconsistent ROS graph
# sample. This was the second false-restart shape seen under DDS load.
now="$(python3 -c 'import time; print(time.time())')"
printf '{{"updated_at":%s,"summary":{{"local_state_endpoint_ready":false,"local_state_topic_ready":false,"local_odom_fresh":false}},"topics":{{"/local_state/odometry":{{"publishers":0,"last_received_at":%s,"last_stamp_sec":%s,"last_age_sec":0.03,"message_count":100}}}}}}\n' "${{now}}" "${{now}}" "${{now}}" >"${{health_file}}"
for _ in 1 2 3 4; do
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# A fresh-but-empty health snapshot is not proof that robot_local_state failed
# when an independent fresh-odom observer can still receive the canonical
# stream. This is the startup false-restart shape captured on the Jetson.
export TEST_DIRECT_LOCAL_STATE_READY=true
now="$(python3 -c 'import time; print(time.time())')"
printf '{{"updated_at":%s,"summary":{{"local_state_endpoint_ready":false,"local_state_topic_ready":false,"local_odom_fresh":false}},"topics":{{"/local_state/odometry":{{"publishers":0,"last_received_at":null,"last_stamp_sec":null,"last_age_sec":null,"message_count":0}}}}}}\n' "${{now}}" >"${{health_file}}"
for _ in 1 2 3 4; do
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# A fresh snapshot that proves a real local-state fault still fails closed,
# but only after the independent fresh-odom confirmation also fails for the
# configured consecutive confirmation budget.
export TEST_DIRECT_LOCAL_STATE_READY=false
now="$(python3 -c 'import time; print(time.time())')"
printf '{{"updated_at":%s,"summary":{{"local_state_endpoint_ready":true,"local_state_topic_ready":false,"local_odom_fresh":false}},"topics":{{"/local_state/odometry":{{"publishers":1,"last_received_at":%s,"last_stamp_sec":1,"last_age_sec":10.0,"message_count":100}}}}}}\n' "${{now}}" "${{now}}" >"${{health_file}}"
verify_robot_local_state_common_health_or_exit
for _ in 1 2 3 4; do
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 1 ]]
refresh_legacy_snapshot
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 2 ]]
refresh_legacy_snapshot
if verify_robot_local_state_common_health_or_exit; then
  exit 31
fi
[[ "${{robot_local_state_common_health_failures}}" -eq 3 ]]

# Recovery resets the genuine-fault counter.
now="$(python3 -c 'import time; print(time.time())')"
printf '{{"updated_at":%s,"summary":{{"local_state_endpoint_ready":true,"local_state_topic_ready":true,"local_odom_fresh":true}},"topics":{{"/local_state/odometry":{{"publishers":1,"last_received_at":%s,"last_stamp_sec":%s,"last_age_sec":0.03,"message_count":101}}}}}}\n' "${{now}}" "${{now}}" "${{now}}" >"${{health_file}}"
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# The v1 writer has a distinct runtime watch, independent from startup freshness.
write_v1_snapshot() {{
  python3 - "${{health_file}}" "$1" "$2" "$3" "${{4:-generation-a}}" "${{5:-false}}" <<'PY'
import json, sys, time
from pathlib import Path
path, age, seen, sequence, generation, delayed = sys.argv[1:]
now = time.time()
snapshot = {{
    "schema": "njrh.runtime_health.v1", "implementation": "cpp",
    "updated_at": now, "generation": generation, "sequence": int(sequence),
    "updated_monotonic_sec": time.monotonic(), "clock_valid": True,
    "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8"),
    "sampling_delayed": delayed == "true",
    "odom_watch": {{"no_update_age_sec": float(age), "timeout_sec": 3.0,
                   "seen_valid": seen == "true"}},
    "summary": {{"local_state_endpoint_ready": True, "local_odom_fresh": False,
                 "local_state_topic_ready": False}},
    "topics": {{"/local_state/odometry": {{"publishers": 1, "last_received_at": now,
               "last_stamp_sec": now - float(age), "last_age_sec": float(age),
               "message_count": int(sequence)}}}}
}}
with open(path, "w", encoding="utf-8") as stream:
    json.dump(snapshot, stream)
PY
}}
export TEST_DIRECT_LOCAL_STATE_READY=false
write_v1_snapshot 2.5 true 1
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]
write_v1_snapshot 2.5 false 2
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# Re-reading one failed generation:sequence is not another confirmation.
write_v1_snapshot 3.0 true 3
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 1 ]]
for _ in 1 2 3 4; do
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 1 ]]

# Independent fresh odom contradicts a new fault snapshot and clears the count.
export TEST_DIRECT_LOCAL_STATE_READY=true
write_v1_snapshot 4.0 true 4
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]
export TEST_DIRECT_LOCAL_STATE_READY=false
write_v1_snapshot 4.0 true 5
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 1 ]]
write_v1_snapshot 5.0 true 6
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 2 ]]
if verify_robot_local_state_common_health_or_exit; then :; else exit 32; fi
[[ "${{robot_local_state_common_health_failures}}" -eq 2 ]]
write_v1_snapshot 5.0 true 5
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 2 ]]

# A new generation starts its own budget, even with the same sequence.
write_v1_snapshot 5.0 true 6 generation-b
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 1 ]]
write_v1_snapshot 5.0 true 7 generation-b
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 2 ]]
write_v1_snapshot 5.0 true 8 generation-b
if verify_robot_local_state_common_health_or_exit; then
  exit 33
fi
[[ "${{robot_local_state_common_health_failures}}" -eq 3 ]]
write_v1_snapshot 0.0 true 9 generation-b
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# Missing process identity still requires an independent odometry confirmation.
export TEST_REQUIRED_PROCESSES_RUNNING=false
export TEST_DIRECT_LOCAL_STATE_READY=true
direct_calls_before="${{TEST_DIRECT_CONFIRM_CALLS}}"
for sequence in 10 11 12 13; do
  write_v1_snapshot 4.0 true "${{sequence}}" generation-b
  verify_robot_local_state_common_health_or_exit
done
[[ "${{TEST_DIRECT_CONFIRM_CALLS}}" -eq $((direct_calls_before + 4)) ]]
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]
export TEST_REQUIRED_PROCESSES_RUNNING=true
export TEST_DIRECT_LOCAL_STATE_READY=false

# Sampling delay and invalid observer data break a consecutive fault run.
write_v1_snapshot 4.0 true 14 generation-b
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 1 ]]
write_v1_snapshot 4.0 true 15 generation-b true
for _ in 1 2 3 4; do
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]
write_v1_snapshot 4.0 true 16 generation-b
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 1 ]]
printf '%s\\n' '{{broken' >"${{health_file}}"
verify_robot_local_state_common_health_or_exit
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# Exercise corrupted diagnostic transport, still using the actual C++ reader.
# The normal wrapper is already covered above; only remove its evidence token.
runtime_health_local_state_diagnostic() {{
  local output="" rc=0
  output="$(runtime_health_query diagnostic "${{NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC}}")" || rc=$?
  printf '%s\\n' "${{output%% evidence_id=*}}"
  return "${{rc}}"
}}
for sequence in 17 18 19 20; do
  write_v1_snapshot 4.0 true "${{sequence}}" generation-b
  verify_robot_local_state_common_health_or_exit
done
[[ "${{robot_local_state_common_health_failures}}" -eq 0 ]]

# An unavailable or stale observer cannot prove a docking-camera failure and
# must not consume the real sensor-fault retry budget.
printf '%s\n' '{{"updated_at":1,"summary":{{"docking_sensor_healthy":false}}}}' >"${{health_file}}"
for _ in 1 2 3 4; do
  verify_docking_sensor_common_health_or_exit
done
[[ "${{docking_sensor_common_health_failures}}" -eq 0 ]]

# A fresh snapshot that explicitly reports an unhealthy docking stream is a
# diagnostic alarm. Motion remains fail-closed in docking_manager, but camera
# health alone must never kill Nav2/localization/the complete common runtime.
now="$(python3 -c 'import time; print(time.time())')"
printf '{{"updated_at":%s,"summary":{{"docking_sensor_healthy":false}}}}\n' "${{now}}" >"${{health_file}}"
for _ in 1 2 3 4; do
  verify_docking_sensor_common_health_or_exit
done
[[ "${{docking_sensor_common_health_failures}}" -eq 3 ]]

# A subsequent fresh healthy snapshot resets the genuine sensor-fault count.
now="$(python3 -c 'import time; print(time.time())')"
printf '{{"updated_at":%s,"summary":{{"docking_sensor_healthy":true}}}}\n' "${{now}}" >"${{health_file}}"
verify_docking_sensor_common_health_or_exit
[[ "${{docking_sensor_common_health_failures}}" -eq 0 ]]
"""
    result = subprocess.run(
        [
            str(bash),
            "-s",
            str(scripts_root / "runtime_health_helpers.sh"),
            sys.executable,
            str(binary),
        ],
        check=False,
        capture_output=True,
        text=True,
        input=harness,
        timeout=60,
        env={**os.environ, "MSYS2_ARG_CONV_EXCL": ""},
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_robot_description_includes_gs2_mount():
    sensors = (ROOT / "src" / "robot_description" / "config" / "sensors.yaml").read_text(encoding="utf-8")
    overlay_sensors = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "sensors.yaml").read_text(
        encoding="utf-8"
    )
    urdf = (ROOT / "src" / "robot_description" / "urdf" / "robot.urdf.xacro").read_text(encoding="utf-8")
    static_tf = (ROOT / "src" / "robot_description" / "src" / "static_tf_node.cpp").read_text(encoding="utf-8")

    assert "gs2_frame: gs2_link" in sensors
    assert "charge_contact_frame: charge_contact_link" in sensors
    assert "gs2_xyz: [0.36, 0.0, 0.290]" in sensors
    assert "gs2_rpy: [0.0, 0.0, 0.0]" in sensors
    assert "lidar_xyz: [0.3686, 0.0000, 0.85]" in sensors
    assert "lidar_rpy: [0.0, -0.257374941072, 3.070994091224]" in sensors
    assert "imu_xyz: [0.3686, 0.0000, 0.85]" in sensors
    assert "imu_rpy: [0.0, -0.257374941072, 3.070994091224]" in sensors
    assert "charge_contact_xyz: [0.398, 0.0, 0.255]" in sensors
    assert "ranger_base_frame: ranger_base_link" in sensors
    assert "gs2_frame: gs2_link" in overlay_sensors
    assert "charge_contact_frame: charge_contact_link" in overlay_sensors
    assert "ranger_base_frame: ranger_base_link" in overlay_sensors
    assert "gs2_x: 0.36" in overlay_sensors
    assert "gs2_z: 0.290" in overlay_sensors
    assert "lidar_x: 0.3686" in overlay_sensors
    assert "lidar_pitch: -0.257374941072" in overlay_sensors
    assert "lidar_yaw: 3.070994091224" in overlay_sensors
    assert "lidar_axis_yaw: 0.0" in overlay_sensors
    assert "imu_x: 0.3686" in overlay_sensors
    assert "imu_pitch: -0.257374941072" in overlay_sensors
    assert "imu_yaw: 3.070994091224" in overlay_sensors
    assert "charge_contact_x: 0.398" in overlay_sensors
    assert '<link name="$(arg ranger_base_frame)"/>' in urdf
    assert '<link name="$(arg gs2_frame)"/>' in urdf
    assert '<link name="$(arg charge_contact_frame)"/>' in urdf
    assert '<joint name="$(arg base_frame)_to_$(arg ranger_base_frame)" type="fixed">' in urdf
    assert '<joint name="$(arg base_frame)_to_$(arg gs2_frame)" type="fixed">' in urdf
    assert '<joint name="$(arg base_frame)_to_$(arg charge_contact_frame)" type="fixed">' in urdf
    assert 'ranger_base_frame' in static_tf
    assert 'const auto gs2_frame = config.count("gs2_frame")' in static_tf
    assert 'charge_contact_frame' in static_tf
    assert 'gs2_frame,' in static_tf


def test_docking_geometry_is_configured():
    docking = (ROOT / "src" / "robot_nav_config" / "config" / "docking.yaml").read_text(encoding="utf-8")
    overlay_docking = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "docking.yaml").read_text(
        encoding="utf-8"
    )

    for cfg in (docking, overlay_docking):
        assert "gs2_frame: gs2_link" in cfg
        assert "charge_contact_frame: charge_contact_link" in cfg
        assert "gs2_scan_topic: /dock/gs2_scan" in cfg
        assert "observation_backend: target_observation" in cfg
        assert "target_observation_topic: /dock/target_observation" in cfg
        assert "target_observation_source: orbbec_336l_depth" in cfg
        assert "cmd_vel_topic: /cmd_vel_docking" in cfg
        assert "status_topic: /docking/status" in cfg
        assert "start_service: /docking/start" in cfg
        assert "stop_service: /docking/stop" in cfg
        assert "undock_service: /docking/undock" in cfg
        assert "forced_mode_topic: /ranger_mini3/forced_mode" in cfg
        assert "park_topic: /ranger_mini3/park" in cfg
        assert "reverse_enable_topic: /ranger_mini3/docking_allow_reverse" in cfg
        assert "use_crab_mode: true" in cfg
        assert "crab_forced_mode: side_slip" in cfg
        assert "yaw_forced_mode: spinning" in cfg
        assert "gs2_z_m: 0.290" in cfg
        assert "charge_contact_x_m: 0.398" in cfg
        assert "gs2_to_contact_x_m: 0.038" in cfg
        assert "housing_lateral_length_m: 0.235" in cfg
        assert "housing_vertical_width_m: 0.080" in cfg
        assert "electrode_lateral_length_m: 0.185" in cfg
        assert "electrode_vertical_width_m: 0.030" in cfg
        assert "positive_electrode_position: upper" in cfg
        assert "min_points: 8" in cfg
        assert "min_lateral_span_m: 0.035" in cfg
        assert "lateral_gate_m: 0.20" in cfg
        assert "front_cluster_x_window_m: 0.015" in cfg
        assert "min_confidence: 0.10" in cfg
        assert "yaw_fit_min_lateral_span_m: 0.055" in cfg
        assert "stable_frames_required: 3" in cfg
        assert "filter_alpha: 0.25" in cfg
        assert "use_yaw_fit: true" in cfg
        assert "pre_dock_distance_m: 0.60" in cfg
        assert "allow_blind_approach: false" in cfg
        assert "final_target_distance_m: 0.34" in cfg
        undock = cfg[cfg.index("    undock:") : cfg.index("    contact_stop:")]
        assert "distance_m: 0.60" in undock
        assert "speed_mps: 0.50" in undock
        assert "max_speed_mps: 0.50" in undock
        assert "min_clear_distance_m: 0.45" in cfg
        assert "timeout_s: 20.0" in cfg
        assert "odom_topic: /local_state/odometry" in cfg
        assert "odom_timeout_s: 0.50" in cfg
        assert "odom_start_timeout_s: 2.0" in cfg
        assert "command_settle_s: 0.5" in cfg
        assert "motion_start_timeout_s: 6.0" in cfg
        assert "no_progress_timeout_s: 2.0" in cfg
        assert "progress_epsilon_m: 0.005" in cfg
        assert "motion_state_topic: /motion_state" in cfg
        assert "wheel_odom_topic: /wheel/odom" in cfg
        assert "feedback_max_age_s: 0.50" in cfg
        assert "linear_speed_threshold_mps: 0.01" in cfg
        assert "angular_speed_threshold_radps: 0.02" in cfg
        assert "stable_duration_s: 0.50" in cfg
        assert "stable_samples: 5" in cfg
        assert "feedback_timeout_s: 3.0" in cfg
        assert yaml_number(cfg, "motion_start_timeout_s") >= 5.0
        assert yaml_number(cfg, "no_progress_timeout_s") == 2.0
        assert yaml_number(undock, "speed_mps") == 0.50
        assert yaml_number(undock, "max_speed_mps") == 0.50
        timeout_budget = (
            yaml_number(undock, "command_settle_s")
            + yaml_number(undock, "motion_start_timeout_s")
            + yaml_number(undock, "distance_m") / yaml_number(undock, "speed_mps")
            + 2.0
        )
        assert yaml_number(cfg, "timeout_s") >= timeout_budget
        assert "max_angular_speed_radps: 0.12" in cfg
        assert "ky: 0.55" in cfg
        assert "ky_lateral: 0.70" in cfg
        assert "lateral_command_sign: 1.0" in cfg
        assert "kyaw: 0.70" in cfg
        assert "lateral_deadband_m: 0.005" in cfg
        assert "yaw_deadband_deg: 0.25" in cfg
        assert "min_align_speed_mps: 0.025" in cfg
        assert "min_angular_speed_radps: 0.05" in cfg
        assert "min_lateral_speed_mps: 0.025" in cfg
        assert "max_lateral_speed_mps: 0.04" in cfg
        assert "yaw_priority_threshold_deg: 1.0" in cfg
        assert "yaw_realign_enter_deg: 1.0" in cfg
        assert "yaw_realign_stable_frames: 3" in cfg
        assert "max_forward_while_lateral_mps: 0.000" in cfg
        assert "lock_lateral_during_final_insert: true" in cfg
        assert "yaw_spin_priority_enabled: true" in cfg
        assert "max_command_steering_rad: 0.35" in cfg
        assert "max_linear_speed_mps: 0.15" in cfg
        assert "contact_crawl_speed_mps: 0.05" in cfg
        assert "contact_verify_max_distance_m: 0.31" in cfg
        assert "contact_verify_retry_enabled: true" in cfg
        assert "contact_retry_max_count: 2" in cfg
        assert "contact_retry_backoff_distance_m: 0.60" in cfg
        assert "contact_retry_backoff_speed_mps: 0.06" in cfg
        assert "contact_retry_backoff_timeout_s: 20.0" in cfg
        assert "contact_retry_backoff_command_settle_s: 0.5" in cfg
        assert "contact_retry_backoff_motion_start_timeout_s: 6.0" in cfg
        assert "contact_retry_backoff_no_progress_timeout_s: 2.0" in cfg
        assert "contact_retry_backoff_progress_epsilon_m: 0.005" in cfg
        assert "contact_retry_backoff_max_lateral_drift_m: 0.05" in cfg
        assert "contact_confirm_timeout_s: 16.0" in cfg
        contact_backoff_budget = (
            yaml_number(cfg, "contact_retry_backoff_command_settle_s")
            + yaml_number(cfg, "contact_retry_backoff_motion_start_timeout_s")
            + yaml_number(cfg, "contact_retry_backoff_distance_m")
            / yaml_number(cfg, "contact_retry_backoff_speed_mps")
            + yaml_number(cfg, "contact_retry_backoff_no_progress_timeout_s")
        )
        assert yaml_number(cfg, "contact_retry_backoff_timeout_s") >= contact_backoff_budget
        assert "lateral_soft_limit_m: 0.030" in cfg
        assert "lateral_hard_limit_m: 0.050" in cfg
        assert "yaw_soft_limit_deg: 0.5" in cfg
        assert "yaw_hard_limit_deg: 7.0" in cfg
        assert "contact_voltage_min_v: 40.0" in cfg
        assert "contact_voltage_max_v: 1000.0" in cfg
        assert "full_soc_threshold_pct: 99.0" in cfg
        assert "full_soc_voltage_contact_enable: true" in cfg


def test_robot_docking_manager_is_safety_chained_cpp():
    cmake = (ROOT / "src" / "robot_docking_manager" / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (ROOT / "src" / "robot_docking_manager" / "package.xml").read_text(encoding="utf-8")
    node = (ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    launch = (ROOT / "src" / "robot_docking_manager" / "launch" / "docking_manager.launch.py").read_text(
        encoding="utf-8"
    )
    runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_docking_manager.sh"
    ).read_text(encoding="utf-8")
    contact_stop_test = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "test_docking_contact_stop_contract.sh"
    ).read_text(encoding="utf-8")
    readme = (ROOT / "src" / "robot_docking_manager" / "README.md").read_text(encoding="utf-8")
    gs2_doc = (ROOT / "docs" / "gs2_docking_lidar.md").read_text(encoding="utf-8")

    assert "add_executable(docking_manager_node src/docking_manager_node.cpp)" in cmake
    assert "find_package(nav_msgs REQUIRED)" in cmake
    assert "find_package(ranger_msgs REQUIRED)" in cmake
    assert "nav_msgs" in cmake
    assert "ranger_msgs" in cmake
    assert "<name>robot_docking_manager</name>" in package_xml
    assert "<depend>nav_msgs</depend>" in package_xml
    assert "<depend>ranger_msgs</depend>" in package_xml
    assert "<exec_depend>robot_nav_config</exec_depend>" in package_xml
    assert 'declare_parameter<std::string>("gs2_scan_topic", "/dock/gs2_scan")' in node
    assert 'declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_docking")' in node
    assert 'declare_parameter<std::string>("start_service", "/docking/start")' in node
    assert 'declare_parameter<std::string>("stop_service", "/docking/stop")' in node
    assert 'declare_parameter<std::string>("undock_service", "/docking/undock")' in node
    assert 'declare_parameter<std::string>("undock.odom_topic", "/local_state/odometry")' in node
    assert 'declare_parameter<double>("undock.odom_timeout_s", 0.50)' in node
    assert 'declare_parameter<double>("undock.command_settle_s", 0.5)' in node
    assert 'declare_parameter<double>("undock.motion_start_timeout_s", 6.0)' in node
    assert 'declare_parameter<double>("undock.no_progress_timeout_s", 2.0)' in node
    assert 'declare_parameter<std::string>("mode.forced_mode_topic", "/ranger_mini3/forced_mode")' in node
    assert 'declare_parameter<std::string>("mode.reverse_enable_topic", "/ranger_mini3/docking_allow_reverse")' in node
    assert 'declare_parameter<bool>("mode.use_crab_mode", true)' in node
    assert 'declare_parameter<std::string>("mode.crab_forced_mode", "side_slip")' in node
    assert 'declare_parameter<std::string>("mode.yaw_forced_mode", "spinning")' in node
    assert "create_service<std_srvs::srv::Trigger>" in node
    assert "create_subscription<sensor_msgs::msg::LaserScan>" in node
    assert "create_subscription<robot_interfaces::msg::DockTargetObservation>" in node
    assert 'observation_backend_ == "target_observation"' in node
    assert 'declare_parameter<bool>("approach.allow_blind_approach", true)' in node
    assert "create_subscription<sensor_msgs::msg::BatteryState>" in node
    assert "create_subscription<nav_msgs::msg::Odometry>" in node
    assert "create_subscription<ranger_msgs::msg::MotionState>" in node
    assert '"contact_stop.motion_state_topic", "/motion_state"' in node
    assert '"contact_stop.wheel_odom_topic", "/wheel/odom"' in node
    assert '"contact_stop.feedback_max_age_s", 0.50' in node
    assert '"contact_stop.linear_speed_threshold_mps", 0.01' in node
    assert '"contact_stop.angular_speed_threshold_radps", 0.02' in node
    assert '"contact_stop.stable_duration_s", 0.50' in node
    assert '"contact_stop.stable_samples", 5' in node
    assert '"contact_stop.feedback_timeout_s", 3.0' in node
    assert "State::BlindApproach" in node
    assert "State::ContactVerify" in node
    assert "State::ContactBackoff" in node
    assert "State::ContactStopping" in node
    assert "State::Undocking" in node
    assert "start_undocking" in node
    assert "handle_undocking" in node
    assert "undock_traveled_m()" in node
    assert "capture_undock_start_odom" in node
    assert "enum class UndockPhase" in node
    assert "UNDOCK_PREPARE" in node
    assert "UNDOCK_WAIT_FIRST_MOTION" in node
    assert "UNDOCK_ACTIVE" in node
    assert "UNDOCK_SUCCEEDED" in node
    assert "UNDOCK_FAILED_NO_COMMAND_PUBLISHED" in node
    assert "UNDOCK_FAILED_MOTION_START_TIMEOUT" in node
    assert "UNDOCK_FAILED_NO_PROGRESS" in node
    assert "UNDOCK_FAILED_TIMEOUT" in node
    assert "undock_nonzero_cmd_publish_count_" in node


    assert "undock_nonzero_cmd_start_time_" in node
    assert "first_undock_motion_time_" in node
    assert "publish_undock_reverse_command(speed, stamp)" in node
    assert "undocking waiting_first_motion" in node
    assert "cmd_count=" in node
    assert "cmd_x=" in node
    assert "reverse_enable=true" in node
    assert "undock_failed_motion_start_timeout" in node
    assert "undock_failed_no_command_published" in node
    assert "undock_failed_no_progress" in node
    assert "undock_failed_no_motion" not in node
    assert "undock_failed_stale_odom" in node
    assert 'undock_running_status("waiting_for_fresh_odom"' in node
    assert "elapsed * speed" not in node
    assert "publish_reverse_enable(true)" in node
    assert "cmd.linear.x = -speed" in node
    assert "POWER_SUPPLY_STATUS_CHARGING" in node
    assert "POWER_SUPPLY_STATUS_FULL" in node
    assert "battery_indicates_charging" in node
    assert "battery_indicates_charging_contact" in node
    assert "dock_contact_latch_is_docked()" in node
    assert "dock_latch_detected" in node
    assert "present_voltage_valid" in node
    assert "full_soc_present_voltage_valid" in node
    assert "normalized_soc_percent" in node
    assert "begin_contact_stop(\"docked_charging_detected\"" in node
    assert "handle_contact_stopping()" in node
    assert "publish_contact_stop_zero" in node
    assert "std::hypot(wheel_twist.linear.x, wheel_twist.linear.y)" in node
    assert "wheel_odom_sequence_ > contact_stop_start_wheel_odom_sequence_" in node
    assert "motion_state_sequence_ > contact_stop_start_motion_state_sequence_" in node
    assert "contact_stop_stable_samples_ >= contact_stop_stable_samples_required_" in node
    assert "contact_stop_feedback_timeout_s_" in node
    assert '" contact_stop_feedback_timeout=" << bool_text(feedback_timeout)' in node
    assert "brake_confirmed=true" in node
    assert "bms_to_first_zero_ms=" in node
    assert "first_zero_to_stop_ms=" in node
    assert "first_zero_to_stop_confirmed_ms=" in node
    assert "contact_verify_traveled_at_bms_m=" in node
    assert "contact_verify_elapsed_at_bms_s=" in node
    assert "post_bms_distance_m=" in node
    assert "zero_cmd_count=" in node
    battery_callback = node[node.index("battery_sub_ =") : node.index("odom_sub_ =")]
    assert "begin_contact_stop" in battery_callback
    assert "finalize_docked_stop" not in battery_callback
    contact_stopping = node[
        node.index("void handle_contact_stopping") : node.index("void handle_contact_verify")
    ]
    assert "publish_contact_stop_zero" in contact_stopping
    assert "finalize_docked_stop" in contact_stopping
    assert contact_stopping.index("publish_contact_stop_zero") < contact_stopping.index("finalize_docked_stop")
    finalize_docked = node[
        node.index("void finalize_docked_stop") : node.index("bool dock_contact_latch_is_docked")
    ]
    assert finalize_docked.index("publish_reverse_enable(false)") < finalize_docked.index(
        "release_docking_motion_mode(park_on_docked_)"
    )
    assert 'declare_parameter<double>("detector.front_cluster_x_window_m", 0.015)' in node
    assert 'declare_parameter<double>("detector.min_confidence", 0.10)' in node
    assert 'declare_parameter<double>("detector.yaw_fit_min_lateral_span_m", 0.055)' in node
    assert "detector_front_cluster_x_window_m_ > 0.0" in node
    assert "front_x_max = *nearest_x_it + detector_front_cluster_x_window_m_" in node
    assert "if (xs[i] <= front_x_max)" in node
    assert "detection.lateral_span >= detector_yaw_fit_min_lateral_span_m_" in node
    assert "detection.confidence > detector_min_confidence_" in node
    assert 'declare_parameter<bool>("detector.use_yaw_fit", false)' in node
    assert "estimate_yaw_error" in node
    assert "filter_detection" in node
    assert "limit_yaw_rate_for_ackermann" in node
    assert "valid_detection_streak_" in node
    assert "min_align_speed_mps_" in node
    assert "lateral_command_sign_ * ky_lateral_ * lateral_error" in node
    assert "yaw_spin_priority_enabled_ && !yaw_ok" in node
    assert "publish_forced_mode(yaw_forced_mode_)" in node
    assert '"yaw_settle"' in node
    assert '"yaw_realign"' in node
    assert "detection.yaw_error, distance_ok, detection.sequence" in node
    assert "yaw_alignment_command(detection.yaw_error)" in node
    assert "absolute_yaw_error >= yaw_realign_enter_rad_" in node
    assert "yaw_alignment_stable_frames_ < yaw_realign_stable_frames_required_" in node
    assert "observation_sequence != last_yaw_alignment_observation_sequence_" in node
    assert '"yaw_settle" : "yaw_realign"' in node
    assert "yaw_realign_active=" in node
    assert "cmd.angular.z = 0.0;" in node
    assert "pivot_compensation_vy = 0.0;" in node
    assert "Mixing angular.z with side-slip near the dock" in node
    assert "min_lateral_speed_mps_" in node
    assert "lock_lateral_during_final_insert_" in node
    assert "cmd.linear.y = 0.0;" in node
    assert "if (!final_insert_locked && (!lateral_ok || !yaw_ok" in node
    assert "release_docking_motion_mode(park_on_docked_)" in node
    assert "if (!distance_ok && distance_error > 0.0)" in node
    assert "FindPackageShare(\"robot_nav_config\")" in launch
    assert "install/robot_docking_manager/lib/robot_docking_manager/docking_manager_node" in runner
    assert "Python fallback has been removed" in runner
    assert 'prefix="/njrh_test/docking_contact_stop/' in contact_stop_test
    assert '-p cmd_vel_topic:="${prefix}/cmd_vel"' in contact_stop_test
    assert '-p contact_stop.motion_state_topic:="${prefix}/motion_state"' in contact_stop_test
    assert '-p contact_stop.wheel_odom_topic:="${prefix}/wheel_odom"' in contact_stop_test
    assert "feedback timeout held continuous zero without Park" in contact_stop_test
    assert "moving feedback held continuous zero without Park" in contact_stop_test
    assert "stable stopped feedback allowed Park" in contact_stop_test
    assert "systemctl" not in contact_stop_test
    assert "/cmd_vel_docking" in readme
    assert "/cmd_vel_collision_checked" in readme
    assert "controller.kyaw=0.70" in readme
    assert "/ranger_mini3/forced_mode=spinning" in readme
    assert "/ranger_mini3/forced_mode=side_slip" in readme
    assert "`linear.y` alone corrects centerline error" in readme
    assert "yaw exit tolerance of `0.5deg` for three consecutive observations" in readme
    assert "re-enters yaw correction at `1.0deg`" in readme
    contact_verify = node[
        node.index("void handle_contact_verify") : node.index("void begin_contact_retry")
    ]
    assert "cmd.linear.y = 0.0;" in contact_verify
    assert "cmd.angular.z = 0.0;" in contact_verify
    assert "contact_final_slow_zone_m_" in contact_verify
    assert "contact_final_crawl_speed_mps_" in contact_verify
    assert '" final_slow_zone=" << bool_text(final_slow_zone)' in contact_verify
    assert "/docking/undock" in readme
    assert "/local_state/odometry" in readme
    assert "elapsed command time is not treated as distance" in readme
    assert "/ranger_mini3/docking_allow_reverse=true" in readme
    assert "undock.speed_mps=0.50" in readme
    assert "undock.motion_start_timeout_s" in readme
    assert "undock.no_progress_timeout_s" in readme
    assert "robot_safety" in readme
    assert "rosbag" in readme
    assert "Do not publish docking control directly to `/cmd_vel_safe`" in gs2_doc
    assert "POST /api/v1/docking/undock" in gs2_doc
    assert "Undocking completion is odometry-confirmed" in gs2_doc


def test_bms_docking_interlock_and_contact_slow_zone_contract():
    safety_cpp = (ROOT / "src" / "robot_safety" / "src" / "robot_safety_node.cpp").read_text(
        encoding="utf-8"
    )
    safety_readme = (ROOT / "src" / "robot_safety" / "README.md").read_text(encoding="utf-8")
    docking_cpp = (
        ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp"
    ).read_text(encoding="utf-8")
    interlock_test = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "test_robot_safety_bms_docking_interlock.sh"
    ).read_text(encoding="utf-8")
    slow_zone_test = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "test_docking_contact_slow_zone_contract.sh"
    ).read_text(encoding="utf-8")

    safety_configs = [
        (ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml").read_text(
            encoding="utf-8"
        ),
        (
            ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_safety.yaml"
        ).read_text(encoding="utf-8"),
    ]
    docking_configs = [
        (ROOT / "src" / "robot_nav_config" / "config" / "docking.yaml").read_text(
            encoding="utf-8"
        ),
        (
            ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "docking.yaml"
        ).read_text(encoding="utf-8"),
    ]

    assert 'declare_parameter<bool>("bms_docking_interlock_enabled", true)' in safety_cpp
    assert (
        'declare_parameter<bool>("bms_docking_interlock_allow_reverse_undock", true)'
        in safety_cpp
    )
    assert "docking_command_allowed_during_bms_contact" in safety_cpp
    assert "publish_bms_docking_interlock_stop(\"bms_contact_rising_edge\")" in safety_cpp
    assert "BMS_DOCKING_INTERLOCK" in safety_cpp
    assert "bms_docking_contact_latched_" in safety_cpp
    assert "bms_interlock_reverse_session_seen_" in safety_cpp
    assert "fresh no-contact feedback" in safety_cpp
    assert "try_release_bms_docking_interlock" in safety_cpp
    assert 'try_release_bms_docking_interlock("battery_state")' in safety_cpp
    assert 'try_release_bms_docking_interlock("reverse_disabled")' in safety_cpp
    assert "cmd.linear.x < -epsilon" in safety_cpp
    assert "reverse_allowed(CommandSource::DOCKING)" in safety_cpp
    for config in safety_configs:
        assert "bms_docking_interlock_enabled: true" in config
        assert "bms_docking_interlock_allow_reverse_undock: true" in config

    assert 'declare_parameter<double>("controller.contact_final_slow_zone_m", 0.06)' in docking_cpp
    assert 'declare_parameter<double>("controller.contact_final_crawl_speed_mps", 0.02)' in docking_cpp
    assert "remaining <= contact_final_slow_zone_m_" in docking_cpp
    assert "contact_stop_post_bms_distance_m()" in docking_cpp
    for config in docking_configs:
        assert "contact_final_slow_zone_m: 0.30" in config
        assert "contact_final_crawl_speed_mps: 0.02" in config

    assert 'prefix="/njrh_test/robot_safety_bms_interlock/' in interlock_test
    assert '-p cmd_vel_out_topic:="${prefix}/cmd_out"' in interlock_test
    assert "BMS contact allowed a stale/competing owner" in interlock_test
    assert "dock_contact_max_age_sec:=0.25" in interlock_test
    assert "survives stale BMS" in interlock_test
    assert "fresh no-contact plus reverse-permit release clears the interlock" in interlock_test
    assert "reverse-permit release plus later fresh no-contact clears the interlock" in interlock_test
    assert "explicit pure-reverse undock remains available" in interlock_test
    assert "systemctl" not in interlock_test
    assert 'prefix="/njrh_test/docking_contact_slow_zone/' in slow_zone_test
    assert "ContactVerify normal zone commands 0.05 m/s" in slow_zone_test
    assert "ContactVerify final 30 cm is capped at 0.02 m/s" in slow_zone_test
    assert "systemctl" not in slow_zone_test
    assert "first fresh BMS contact immediately clears" in safety_readme


def test_undock_diagnostic_contract():
    diagnose = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "diagnose_undock_logic_and_no_motion.sh"
    diagnose_text = diagnose.read_text(encoding="utf-8")
    assert diagnose.exists()
    assert "--dry-run" in diagnose_text
    assert "--execute-undock" in diagnose_text
    assert "POST /api/v1/docking/undock" in diagnose_text
    assert "CASE_DRY_RUN_NO_UNDOCK_EXECUTED" in diagnose_text
    assert "CASE_DOCKING_MANAGER_NO_CMD" in diagnose_text
    assert "CASE_DOCKING_MANAGER_WAITING_FIRST_MOTION_WITH_CMD" in diagnose_text
    assert "CASE_LOGIC_NO_PROGRESS_TIMER_TOO_EARLY" in diagnose_text
    assert "CASE_DOCKING_MANAGER_REJECTS_API_LATCH" in diagnose_text
    assert "CASE_SAFETY_BLOCKED_DOCKING_CMD" in diagnose_text
    assert "CASE_MODE_CONTROLLER_BLOCKED_REVERSE" in diagnose_text
    assert "CASE_CHASSIS_NO_MOTION_AFTER_CMD" in diagnose_text
    assert "CASE_ODOM_NOT_UPDATING" in diagnose_text
    assert "first_reverse_enable_true_time" in diagnose_text
    assert "status_cmd_count_max" in diagnose_text
    assert "api_status_cmd_count_max" in diagnose_text
    assert "api_json_cmd_count_max" in diagnose_text
    assert "cmd_source_evidence" in diagnose_text
    assert "text_cmd_count" in diagnose_text
    assert "Internal Docking Status" in diagnose_text
    assert "External Topic Observation" in diagnose_text
    assert "observed_cmd_vel_docking_nonzero_count" in diagnose_text
    assert "observed_cmd_vel_safe_nonzero_count" in diagnose_text
    assert "CASE_OBSERVER_MISSED_DOCKING_CMD_OR_TOPIC_MISMATCH" in diagnose_text
    assert "CASE_STATE_MACHINE_COUNT_CONTRADICTION" in diagnose_text
    assert "CASE_DOCKING_MANAGER_NO_CMD_PUBLISHER" in diagnose_text
    assert "topic_info_armed" in diagnose_text
    assert "samplers_armed.env" in diagnose_text
    assert "echo_safety_status" in diagnose_text
    assert "FAIL cannot create report directory" in diagnose_text
    assert "FAIL report directory is not writable" in diagnose_text
    assert ".write_probe" in diagnose_text
    assert diagnose_text.index("start_timed_echo \"echo_docking_status\"") < diagnose_text.index(
        "curl_json POST /api/v1/docking/undock"
    )
    assert diagnose_text.index("sleep 0.5") < diagnose_text.index(
        "curl_json POST /api/v1/docking/undock"
    )
    assert "ros2 topic pub" not in diagnose_text


def test_orbbec_336l_docking_backend_is_safe_and_rollbackable():
    interface = (ROOT / "src" / "robot_interfaces" / "msg" / "DockTargetObservation.msg").read_text(
        encoding="utf-8"
    )
    perception_cmake = (ROOT / "src" / "robot_docking_perception" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    perception_node = (
        ROOT / "src" / "robot_docking_perception" / "src" / "orbbec_depth_dock_node.cpp"
    ).read_text(encoding="utf-8")
    geometry = (
        ROOT / "src" / "robot_docking_perception" / "src" / "depth_dock_geometry.cpp"
    ).read_text(encoding="utf-8")
    driver_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_orbbec_336l_depth.sh"
    ).read_text(encoding="utf-8")
    perception_runner = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_orbbec_docking_perception.sh"
    ).read_text(encoding="utf-8")
    perception_config = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "orbbec_docking_perception.yaml"
    ).read_text(encoding="utf-8")
    manager_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_docking_manager.sh"
    ).read_text(encoding="utf-8")
    manager_cpp = (
        ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp"
    ).read_text(encoding="utf-8")
    api_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_api_server.sh"
    ).read_text(encoding="utf-8")
    common_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh"
    ).read_text(encoding="utf-8")
    systemd_runner = (ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh").read_text(
        encoding="utf-8"
    )
    autostart_installer = (ROOT / "scripts" / "jetson" / "install_njrh_autostart.sh").read_text(
        encoding="utf-8"
    )
    static_tf = (ROOT / "src" / "robot_description" / "src" / "static_tf_node.cpp").read_text(
        encoding="utf-8"
    )
    overlay_sensors = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "sensors.yaml"
    ).read_text(encoding="utf-8")
    cpu_affinity = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "cpu_affinity.env"
    ).read_text(encoding="utf-8")
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    docking_runtime_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "docking_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    predock_control_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.cpp"
    ).read_text(encoding="utf-8")
    readiness_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")

    for field in (
        "bool sensor_healthy",
        "bool valid",
        "float64 forward_gap_m",
        "float64 lateral_error_m",
        "float64 yaw_error_rad",
        "float64 confidence",
    ):
        assert field in interface
    assert "add_executable(orbbec_depth_dock_node" in perception_cmake
    assert "robot_interfaces/msg/dock_target_observation.hpp" in perception_node
    assert "create_publisher<robot_interfaces::msg::DockTargetObservation>" in perception_node
    assert "geometry_msgs/msg/twist" not in perception_node
    assert "cmd_vel" not in perception_node
    assert "camera_info_resolution_mismatch" in perception_node
    assert "camera_info_frame_mismatch" in perception_node
    assert "max_processing_rate_hz" in perception_node
    assert "input_stale_timeout_sec" in perception_node
    assert "health_publish_period_sec" in perception_node
    assert 'observation.reason = "depth_stream_stale"' in perception_node
    assert "lateral_span_too_large" in geometry
    assert "no_known_dock_feature_cluster" in geometry
    assert "FeatureKind::NearProtrusion" in geometry

    for disabled_arg in (
        "enable_color:=false",
        "enable_left_ir:=false",
        "enable_right_ir:=false",
        "enable_accel:=false",
        "enable_gyro:=false",
        "enable_point_cloud:=false",
        "enable_colored_point_cloud:=false",
        "publish_tf:=false",
    ):
        assert disabled_arg in driver_runner
    assert "njrh_exec_affined docking_camera" in driver_runner
    assert "njrh_exec_affined docking_vision" in perception_runner
    assert 'AMR_PRESET_NAME="G336X AMR Default"' in driver_runner
    assert 'preset_firmware_path:="${PRESET_FIRMWARE_PATH}"' in driver_runner
    assert 'device_preset:="${DEVICE_PRESET}"' in driver_runner
    assert 'NJRH_ORBBEC_ENABLE_HEARTBEAT:-false' in driver_runner
    assert 'NJRH_ORBBEC_UVC_BACKEND:-libuvc' in driver_runner
    assert 'uvc_backend:="${UVC_BACKEND}"' in driver_runner
    assert 'NJRH_ORBBEC_DIAGNOSTIC_PERIOD_SEC:-1.0' in driver_runner
    assert 'diagnostic_period:="${DIAGNOSTIC_PERIOD_SEC}"' in driver_runner
    assert "enumerate_net_device:=false" in driver_runner
    assert yaml_number(perception_config, "geometry.min_lateral_span_m") == pytest.approx(0.10)
    assert yaml_number(
        perception_config, "geometry.expected_lateral_span_tolerance_m"
    ) == pytest.approx(0.060)
    assert yaml_number(
        perception_config, "geometry.near_feature_expected_lateral_span_m"
    ) == pytest.approx(0.113)
    assert yaml_number(
        perception_config, "geometry.near_feature_lateral_span_tolerance_m"
    ) == pytest.approx(0.025)
    assert yaml_number(
        perception_config, "geometry.near_feature_max_forward_gap_m"
    ) == pytest.approx(0.300)
    assert yaml_number(
        perception_config, "geometry.near_feature_center_y_offset_m"
    ) == pytest.approx(-0.0418)

    assert 'DOCKING_SENSOR_BACKEND="${NJRH_DOCKING_SENSOR_BACKEND:-orbbec_336l}"' in common_runner
    assert 'DOCKING_SENSOR_BACKEND="${NJRH_DOCKING_SENSOR_BACKEND:-orbbec_336l}"' in systemd_runner
    assert '"-e" "NJRH_DOCKING_SENSOR_BACKEND=${DOCKING_SENSOR_BACKEND}"' in systemd_runner
    assert '"-e" "NJRH_GS2_AUTOSTART=${GS2_AUTOSTART}"' in systemd_runner
    assert "GS2 disabled" in systemd_runner
    assert 'NJRH_DOCKING_SENSOR_BACKEND=orbbec_336l' in autostart_installer
    assert 'NJRH_GS2_AUTOSTART=false' in autostart_installer
    assert "missing ${required_key}" in common_runner
    assert "wait_for_fresh_header_topic_message" in common_runner
    assert '"/dock/target_observation"' in common_runner
    assert 'start_orbbec_336l_depth_common || return 1' in common_runner
    assert 'start_common_process "orbbec_336l_depth" "camera336l"' not in common_runner
    assert 'start_common_process "orbbec_336l_depth" "orbbec_camera|camera336l"' not in common_runner
    assert 'robot_interfaces/msg/DockTargetObservation' in readiness_cpp
    assert "wait_for_fresh_stamped_topic_typed<robot_interfaces::msg::DockTargetObservation>" in readiness_cpp
    assert 'gs2)' in common_runner
    assert 'approach.final_target_distance_m:=0.05' in manager_runner
    assert 'controller.lateral_command_sign:=-1.0' in manager_runner
    assert 'observation_backend:=target_observation' in manager_runner
    assert 'approach.allow_blind_approach:=false' in manager_runner
    assert 'approach.final_target_distance_m:=0.34' in manager_runner
    assert 'controller.lateral_command_sign:=1.0' in manager_runner
    assert 'controller.contact_verify_max_distance_m:=0.31' in manager_runner
    assert 'controller.contact_verify_retry_enabled:=true' in manager_runner
    assert 'controller.contact_retry_max_count:=2' in manager_runner
    assert 'controller.contact_retry_backoff_distance_m:=0.60' in manager_runner
    assert 'controller.contact_retry_backoff_speed_mps:=0.06' in manager_runner
    assert 'controller.contact_retry_backoff_timeout_s:=20.0' in manager_runner
    assert 'tolerances.contact_confirm_timeout_s:=16.0' in manager_runner
    assert "capture_contact_start_odom" in manager_cpp
    assert "contact_verify_traveled_m" in manager_cpp
    assert 'begin_contact_retry("distance_limit", "contact_verify_failed_distance_limit")' in manager_cpp
    assert 'begin_contact_retry("contact_wait_expired", "contact_verify_timeout")' in manager_cpp
    assert "handle_contact_backoff" in manager_cpp
    assert "capture_contact_backoff_start_odom" in manager_cpp
    assert "contact_backoff_traveled_m" in manager_cpp
    assert "contact_backoff_lateral_m" in manager_cpp
    assert "finish_contact_backoff" in manager_cpp
    assert "contact_retry_count_ >= contact_retry_max_count_" in manager_cpp
    assert 'cmd.linear.x = -speed;' in manager_cpp
    assert 'cmd.linear.y = 0.0;' in manager_cpp
    assert 'cmd.angular.z = 0.0;' in manager_cpp
    assert 'fail("contact_retry_backoff_failed_lateral_drift")' in manager_cpp
    assert 'fail("contact_retry_backoff_failed_motion_start_timeout")' in manager_cpp
    assert 'fail("contact_retry_backoff_failed_no_progress")' in manager_cpp
    assert 'transition(State::Acquire, status.str())' in manager_cpp
    assert "retry_waiting_for_dock_observation_after_contact_timeout" not in manager_cpp
    assert 'fail("contact_verify_failed_observation_stale")' not in manager_cpp
    assert 'fail("contact_verify_failed_alignment_lost")' not in manager_cpp
    assert "calibrated visual handoff boundary" in manager_cpp
    assert 'fail("contact_verify_failed_stale_odom")' in manager_cpp
    assert 'docking_observation_backend:=target_observation' in api_runner
    assert 'config_.observation_backend == "target_observation"' in docking_runtime_cpp
    assert "DOCK_TARGET_OBSERVATION_TIMEOUT" in predock_control_cpp

    assert 'config.count("docking_camera_x")' in static_tf
    assert '"camera336l_depth_optical_frame"' in static_tf
    active_sensor_keys = {
        line.split(":", 1)[0].strip()
        for line in overlay_sensors.splitlines()
        if line.strip() and not line.lstrip().startswith("#") and ":" in line
    }
    for camera_key in (
        "docking_camera_frame",
        "docking_camera_depth_optical_frame",
        "docking_camera_x",
        "docking_camera_y",
        "docking_camera_z",
        "docking_camera_roll",
        "docking_camera_pitch",
        "docking_camera_yaw",
    ):
        assert camera_key in active_sensor_keys
    assert "docking_camera_frame: camera336l_link" in overlay_sensors
    assert "docking_camera_depth_optical_frame: camera336l_depth_optical_frame" in overlay_sensors
    assert yaml_number(overlay_sensors, "docking_camera_x") == pytest.approx(0.394)
    assert yaml_number(overlay_sensors, "docking_camera_y") == pytest.approx(0.0)
    assert yaml_number(overlay_sensors, "docking_camera_z") == pytest.approx(0.350)
    assert yaml_number(overlay_sensors, "docking_camera_roll") == pytest.approx(0.0)
    assert yaml_number(overlay_sensors, "docking_camera_pitch") == pytest.approx(0.0)
    assert yaml_number(overlay_sensors, "docking_camera_yaw") == pytest.approx(0.0)
    assert 'NJRH_CPUSET_DOCKING_CAMERA="${NJRH_CPUSET_DOCKING_CAMERA:-${NJRH_CPUSET_SYSTEM}}"' in cpu_affinity
    assert 'NJRH_CPUSET_DOCKING_VISION="${NJRH_CPUSET_DOCKING_VISION:-${NJRH_CPUSET_NAV_CONTROL}}"' in cpu_affinity


def test_contact_retry_backoff_is_bounded_and_straight():
    configs = [
        (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "docking.yaml").read_text(
            encoding="utf-8"
        ),
        (ROOT / "src" / "robot_nav_config" / "config" / "docking.yaml").read_text(
            encoding="utf-8"
        ),
    ]
    manager = (ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_docking_manager.sh"
    ).read_text(encoding="utf-8")

    for config in configs:
        assert yaml_number(config, "contact_verify_max_distance_m") == pytest.approx(0.31)
        assert "contact_verify_retry_enabled: true" in config
        assert yaml_number(config, "contact_retry_max_count") == 2
        assert yaml_number(config, "contact_retry_backoff_distance_m") == pytest.approx(0.60)
        assert yaml_number(config, "contact_retry_backoff_speed_mps") == pytest.approx(0.06)
        assert yaml_number(config, "contact_retry_backoff_max_lateral_drift_m") == pytest.approx(0.05)
        budget = (
            yaml_number(config, "contact_retry_backoff_command_settle_s")
            + yaml_number(config, "contact_retry_backoff_motion_start_timeout_s")
            + yaml_number(config, "contact_retry_backoff_distance_m")
            / yaml_number(config, "contact_retry_backoff_speed_mps")
            + yaml_number(config, "contact_retry_backoff_no_progress_timeout_s")
        )
        assert yaml_number(config, "contact_retry_backoff_timeout_s") >= budget

    assert 'controller.contact_verify_max_distance_m:=0.31' in runner
    assert 'controller.contact_verify_retry_enabled:=true' in runner
    assert 'controller.contact_retry_max_count:=2' in runner
    assert 'controller.contact_retry_backoff_distance_m:=0.60' in runner
    assert "State::ContactBackoff" in manager
    assert 'begin_contact_retry("distance_limit", "contact_verify_failed_distance_limit")' in manager
    assert 'begin_contact_retry("contact_wait_expired", "contact_verify_timeout")' in manager
    assert "contact_backoff_traveled_m" in manager
    assert "contact_backoff_lateral_m" in manager
    assert "contact_retry_count_ >= contact_retry_max_count_" in manager
    assert "cmd.linear.x = -speed;" in manager
    assert "cmd.linear.y = 0.0;" in manager
    assert "cmd.angular.z = 0.0;" in manager
    assert 'fail("contact_retry_backoff_failed_lateral_drift")' in manager
    assert 'fail("contact_retry_backoff_failed_no_progress")' in manager
    assert "retry_waiting_for_dock_observation_after_contact_timeout" not in manager


def test_undock_motion_start_state_machine_publishes_before_waiting():
    node = (ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    nav2 = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml").read_text(
        encoding="utf-8"
    )
    diagnose = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "diagnose_undock_logic_and_no_motion.sh"
    ).read_text(encoding="utf-8")

    wait_start = node.index("if (undock_phase_ == UndockPhase::UNDOCK_WAIT_FIRST_MOTION)")
    active_start = node.index("if (undock_phase_ == UndockPhase::UNDOCK_ACTIVE)")
    wait_block = node[wait_start:active_start]
    active_block = node[active_start:node.index("if (traveled >= distance)", active_start)]

    assert wait_block.index("publish_undock_reverse_command(speed, stamp)") < wait_block.index(
        "undock_motion_start_timeout_s_"
    )
    assert "++undock_nonzero_cmd_publish_count_" in node
    assert "undock_nonzero_cmd_publish_count_ == 0U" in wait_block
    assert "undock_failed_no_command_published" in wait_block
    assert "undock_nonzero_cmd_publish_count_ > 0U && motion_wait > undock_motion_start_timeout_s_" in wait_block
    assert 'undock_failure_status("undock_failed_motion_start_timeout"' in wait_block
    assert "cmd_count=" in wait_block
    assert "cmd_x=" in wait_block
    assert "last_cmd_x=" in node
    assert "reverse_enable_count=" in node
    assert "last_cmd_stamp_age_s=" in node
    assert "command_start_elapsed_s=" in node
    assert "first_motion_started=" in node
    assert "failure_reason=" in node
    assert "undocking waiting_first_motion phase=waiting_first_motion" in wait_block
    assert "phase=waiting_first_motion" in wait_block
    assert "reverse_enable=true reverse_enable_count=" in wait_block
    assert "command_start_elapsed_s=" in wait_block
    assert "motion_start_timeout_s=" in wait_block
    assert wait_block.index("undock_nonzero_cmd_publish_count_ > 0U") < wait_block.index(
        "undock_failed_motion_start_timeout"
    )
    assert "undock_no_progress_timeout_s_" in active_block
    assert "last_undock_progress_time_" in active_block
    assert 'undock_failure_status("undock_failed_no_progress"' in active_block
    assert "phase=active" in node
    assert "no_progress_timeout_s=" in node
    assert "publish_undock_reverse_command(speed, stamp)" in node
    assert "publish_reverse_enable(true)" in node[node.index("void publish_undock_reverse_command") :]
    assert "speed_mps: 0.50" in (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "docking.yaml"
    ).read_text(encoding="utf-8")
    assert yaml_number(nav2, "vx_min") >= -0.08
    assert "CASE_DOCKING_MANAGER_NO_CMD" in diagnose
    assert "status_has_waiting_first_motion_with_cmd" in diagnose
    assert "CASE_DOCKING_MANAGER_WAITING_FIRST_MOTION_WITH_CMD" in diagnose


def test_undock_uses_dedicated_half_meter_per_second_speed_envelope():
    node = (
        ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp"
    ).read_text(encoding="utf-8")
    configs = [
        (ROOT / "src" / "robot_nav_config" / "config" / "docking.yaml").read_text(
            encoding="utf-8"
        ),
        (
            ROOT
            / "scripts"
            / "jetson"
            / "runtime_overlay"
            / "config"
            / "docking.yaml"
        ).read_text(encoding="utf-8"),
    ]
    diagnose = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "diagnose_undock_logic_and_no_motion.sh"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter<double>("undock.speed_mps", 0.50)' in node
    assert 'declare_parameter<double>("undock.max_speed_mps", 0.50)' in node
    undock_handler = node[
        node.index("void handle_undocking()") : node.index(
            "void publish_undock_reverse_command", node.index("void handle_undocking()")
        )
    ]
    assert "clamp(undock_speed_mps_, 0.0, undock_max_speed_mps_)" in undock_handler
    assert "clamp(undock_speed_mps_, 0.0, max_linear_speed_mps_)" not in undock_handler
    assert 'speed = scalar(undock, "speed_mps")' in diagnose
    assert 'max_speed = scalar(undock, "max_speed_mps")' in diagnose
    assert 'read("static_audit.md")' in diagnose
    assert "expected_reverse_tokens" in diagnose
    assert '"-0.06" not in mode_status_text' not in diagnose
    assert '"robot_safety requires fresh docking reverse permit"' in diagnose
    assert '"reverse_allowed(CommandSource::DOCKING)" in safety_cpp' in diagnose

    for config in configs:
        undock = config[config.index("    undock:") : config.index("    contact_stop:")]
        assert yaml_number(undock, "speed_mps") == pytest.approx(0.50)
        assert yaml_number(undock, "max_speed_mps") == pytest.approx(0.50)
        assert yaml_number(config, "max_linear_speed_mps") == pytest.approx(0.15)


def test_bms_charging_contact_truth_table_for_full_dock_contact():
    package_root = ROOT / "src" / "robot_api_server"
    bms_header = (
        package_root / "include" / "robot_api_server" / "features" / "power" / "bms_contact.hpp"
    ).read_text(
        encoding="utf-8"
    )
    bms_cpp = (package_root / "src" / "features" / "power" / "bms_contact.cpp").read_text(
        encoding="utf-8"
    )

    assert "struct BatteryContactEvaluation" in bms_header
    assert "normalized_soc_percent" in bms_header
    assert "evaluate_battery_charging_contact" in bms_header
    assert "POWER_SUPPLY_STATUS_CHARGING" in bms_cpp
    assert "POWER_SUPPLY_STATUS_FULL" in bms_cpp
    assert "present_voltage_valid" in bms_cpp
    assert "full_soc_present_voltage_valid" in bms_cpp
    assert "current_above_threshold" in bms_cpp

    power_supply_status_charging = 1
    power_supply_status_full = 4
    power_supply_status_unknown = 0

    def charging_contact(status, present, voltage, current, soc):
        voltage_valid = 40.0 <= voltage <= 1000.0
        return (
            status == power_supply_status_charging
            or status == power_supply_status_full
            or current > 0.10
            or (present and voltage_valid)
            or (present and soc >= 99.0 and voltage_valid)
        )

    assert charging_contact(power_supply_status_full, True, 508.0, 0.0, 100.0)
    assert charging_contact(power_supply_status_unknown, False, 508.0, 0.2, 80.0)
    assert charging_contact(power_supply_status_unknown, True, 508.0, 0.0, 80.0)
    assert charging_contact(power_supply_status_unknown, True, 508.0, 0.0, 100.0)
    assert not charging_contact(power_supply_status_unknown, False, 508.0, 0.0, 100.0)
    assert not charging_contact(power_supply_status_unknown, False, 0.0, 0.0, 50.0)


def test_navigation_goal_pre_undock_gate_observability_contract():
    package_root = ROOT / "src" / "robot_api_server"
    node_cpp = (package_root / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    dock_contact_header = (
        package_root / "include" / "robot_api_server" / "features" / "docking"
        / "lifecycle" / "dock_contact_interlock_module.hpp"
    ).read_text(encoding="utf-8")
    dock_contact_cpp = (
        package_root / "src" / "features" / "docking" / "lifecycle"
        / "dock_contact_interlock_module.cpp"
    ).read_text(encoding="utf-8")
    pre_navigation_undock_header = (
        package_root / "include" / "robot_api_server" / "features" / "docking"
        / "lifecycle" / "pre_navigation_undock_module.hpp"
    ).read_text(encoding="utf-8")
    pre_navigation_undock_cpp = (
        package_root / "src" / "features" / "docking" / "lifecycle"
        / "pre_navigation_undock_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_cpp = (
        package_root / "src" / "features" / "navigation" / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    goal_executor_cpp = (
        package_root / "src" / "features" / "navigation" / "mission"
        / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    execution_cpp = navigation_goal_execution_source()
    api_gateway_cpp = (
        package_root / "src" / "infrastructure" / "http" / "api_gateway_module.cpp"
    ).read_text(encoding="utf-8")
    bms_cpp = (package_root / "src" / "features" / "power" / "bms_contact.cpp").read_text(
        encoding="utf-8"
    )
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_pre_navigation_undock_gate.sh"
    ).read_text(encoding="utf-8")

    assert "GET /api/v1/navigation/pre_goal_check" in api_gateway_cpp
    assert "handle_pre_goal_check" in navigation_cpp
    assert "struct PreNavigationDockCheck" in dock_contact_header
    assert "PreNavigationDockCheck snapshot()" in dock_contact_header
    assert "pre_navigation_check_json" in dock_contact_header
    assert "api_bms_charging_contact" in dock_contact_cpp
    assert "api_bms_charging_contact_reason" in dock_contact_cpp
    assert "final_is_docked_or_charging" in dock_contact_cpp
    assert "final_auto_undock_required" in dock_contact_cpp
    assert "can_auto_undock" in dock_contact_cpp
    assert "auto_undock_reason" in dock_contact_cpp
    assert "docking_status_charging" in dock_contact_cpp
    assert "docking_status_docked" in dock_contact_cpp
    assert "runtime_docking_state_docked" in dock_contact_cpp
    assert "bms_charging_contact:" in dock_contact_cpp
    assert "class PreNavigationUndockModule" in pre_navigation_undock_header
    assert "bool run_if_needed(" in pre_navigation_undock_header
    assert "auto_undock_before_navigation" in pre_navigation_undock_cpp
    assert "post-undock navigation readiness failed, " in pre_navigation_undock_cpp

    assert "POWER_SUPPLY_STATUS_CHARGING" in bms_cpp
    assert "POWER_SUPPLY_STATUS_FULL" in bms_cpp
    assert "current_above_threshold" in bms_cpp
    assert "present_voltage_valid" in bms_cpp
    assert "full_soc_present_voltage_valid" in bms_cpp

    goal_block = navigation_cpp[
        navigation_cpp.index("HttpResponse handle_goal(") : navigation_cpp.index(
            "void refresh_runtime_state"
        )
    ]
    assert "const bool by_pose_id = !pose_id.empty();" in goal_block
    assert '"navigation_goal"' in goal_block
    assert "dock_payload" in goal_block
    assert "undock_before_navigation_if_needed(" not in goal_block
    assert "bridge_safe_for_goal_start(\"navigation goal\"" not in goal_block
    assert "pre_navigation_undock" in goal_block
    assert "pre_navigation_undock_detail" in goal_block
    assert "pre_goal_dock_snapshot" in goal_block
    assert "navigation_goal_error_response" in goal_block

    precheck_block = navigation_cpp[
        navigation_cpp.index("HttpResponse handle_pre_goal_check") : navigation_cpp.index(
            "HttpResponse handle_goal("
        )
    ]
    assert "read_only" in precheck_block
    assert "find_floor_catalog_pose(" in precheck_block
    assert "maps_module_.catalog().find_map_by_id(map_id)" in precheck_block
    assert 'query_number_value("x")' in precheck_block
    assert 'query_number_value("theta")' in precheck_block
    assert "goal_admission_snapshot()" in precheck_block
    assert "start_pre_navigation_undock" not in precheck_block
    assert "async_send_goal" not in precheck_block
    assert "call_undock_service_with_charging_retry" not in precheck_block
    assert "lifecycle_snapshot()" not in goal_block
    assert "async_send_goal(goal)" not in goal_block
    assert "ports_.run_goal_job(" in goal_block
    assert "queued controlled undock in navigation background job" in goal_block
    assert "goal-start readiness will be checked in navigation background job" in goal_block
    assert "dock](const std::uint64_t job_id)" in goal_block

    pre_send_block = goal_executor_cpp[
        goal_executor_cpp.index("bool NavigationGoalExecutor::run_pre_send_sequence") : goal_executor_cpp.index(
            "void NavigationGoalExecutor::run_guarded"
        )
    ]
    assert "undock_before_navigation_if_needed(" in pre_send_block
    assert "navigation requires successful undock first" in pre_send_block
    assert "navigation requires post-undock localization readiness before goal start" in pre_send_block
    assert "wait_for_goal_start_readiness(job_id, readiness_detail)" in pre_send_block
    assert "job.pre_navigation_relocalization_succeeded = false;" in pre_send_block
    readiness_block = goal_executor_cpp[
        goal_executor_cpp.index("bool NavigationGoalExecutor::wait_for_goal_start_readiness") : goal_executor_cpp.index(
            "bool NavigationGoalExecutor::run_pre_send_sequence"
        )
    ]
    assert "bridge_safe_for_goal_start(\"navigation goal\"" in readiness_block
    assert '"waiting_for_goal_start_readiness"' in readiness_block
    assert '"failed_goal_start_readiness"' in readiness_block
    goal_send_start = execution_cpp.index(
        "bool NavigationGoalExecutionModule::send_initial_navigation_goal_to_nav2"
    )
    goal_send_block = execution_cpp[goal_send_start:]
    assert "async_send_goal(goal)" in goal_send_block
    assert '"sending_nav2_goal"' in goal_send_block
    assert '"failed_send_nav2_goal"' in goal_send_block

    assert "pre_navigation_dock_check" in script
    assert "/api/v1/navigation/pre_goal_check" in script
    assert "--execute-goal" in script
    assert "Read-only by default" in script
    assert "POST /api/v1/navigation/goal" in script


def test_phase25_docked_motion_interlock_contract():
    api_root = ROOT / "src" / "robot_api_server"
    entry_cpp = (api_root / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    composition_cpp = application_composition_source()
    safety_state_cpp = (
        api_root / "src" / "features" / "safety" / "safety_state.cpp"
    ).read_text(encoding="utf-8")
    docking_feature_cpp = docking_feature_source()
    execution_cpp = navigation_goal_execution_source()
    dock_contact_header = (
        api_root / "include" / "robot_api_server" / "features" / "docking"
        / "lifecycle" / "dock_contact_interlock_module.hpp"
    ).read_text(encoding="utf-8")
    dock_contact_cpp = (
        api_root / "src" / "features" / "docking" / "lifecycle"
        / "dock_contact_interlock_module.cpp"
    ).read_text(encoding="utf-8")
    dock_contact_contract = dock_contact_header + dock_contact_cpp
    pre_navigation_undock_cpp = (
        api_root / "src" / "features" / "docking" / "lifecycle"
        / "pre_navigation_undock_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_module_cpp = (
        api_root / "src" / "features" / "navigation" / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_executor_cpp = (
        api_root
        / "src"
        / "features"
        / "navigation"
        / "mission"
        / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (api_root / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    overlay_api_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    safety_root = ROOT / "src" / "robot_safety"
    safety_cpp = (safety_root / "src" / "robot_safety_node.cpp").read_text(encoding="utf-8")
    safety_cfg = (safety_root / "config" / "robot_safety.yaml").read_text(encoding="utf-8")
    overlay_safety_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_safety.yaml"
    ).read_text(encoding="utf-8")
    safety_cmake = (safety_root / "CMakeLists.txt").read_text(encoding="utf-8")
    safety_package = (safety_root / "package.xml").read_text(encoding="utf-8")
    docking_cpp = (
        ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp"
    ).read_text(encoding="utf-8")
    docking_cfg = (ROOT / "src" / "robot_nav_config" / "config" / "docking.yaml").read_text(
        encoding="utf-8"
    )
    overlay_docking_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "docking.yaml"
    ).read_text(encoding="utf-8")
    verify_script_path = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_docked_navigation_undock_gate.sh"
    )
    verify_script = verify_script_path.read_text(encoding="utf-8")

    assert verify_script_path.exists()
    assert "Read-only by default" in verify_script
    assert "--execute-goal" in verify_script
    assert "--test-normal-cmd-block" in verify_script
    assert "--test-docking-cmd-allowed" in verify_script
    assert "DANGEROUS optional test" in verify_script
    assert "/api/v1/navigation/pre_goal_check" in verify_script
    assert "/api/v1/navigation/goal" in verify_script
    assert "TEST_NORMAL_CMD_BLOCK=false" in verify_script
    assert "TEST_DOCKING_CMD_ALLOWED=false" in verify_script
    assert "/cmd_vel" in verify_script
    assert "/lidar_points" not in verify_script

    assert "struct DockContactLatchSnapshot" in dock_contact_header
    assert "docking_contact_latch_file" in docking_configuration_source()
    assert "read_latch" in dock_contact_contract
    assert "update_latch" in dock_contact_contract
    assert "dock_latch_indicates_docked" in dock_contact_contract
    assert "dock_contact_snapshot" in dock_contact_cpp
    assert "dock_contact_latch:" in dock_contact_cpp
    assert "DOCKED_OR_CHARGING_CONTACT" in execution_cpp
    assert "blocked_by_docked_contact" in execution_cpp
    assert "final_auto_undock_required" in dock_contact_cpp
    assert "safety_feature_module_" in composition_cpp
    assert "normal_motion_blocked_reason" in safety_state_cpp
    assert "pre_navigation dock gate" in navigation_module_cpp
    assert "pre_navigation_check_json" in dock_contact_contract
    assert "PreNavigationUndockModule" in docking_feature_cpp
    assert "normal_motion_blocked_reason" not in entry_cpp
    assert "PreNavigationUndockModule" not in entry_cpp
    assert "auto_undock_before_navigation" in pre_navigation_undock_cpp
    assert "NavigateToPose::Goal goal" in navigation_module_cpp
    pre_send_block = navigation_executor_cpp[
        navigation_executor_cpp.index("bool NavigationGoalExecutor::run_pre_send_sequence") :
        navigation_executor_cpp.index("void NavigationGoalExecutor::run_guarded")
    ]
    guarded_block = navigation_executor_cpp[
        navigation_executor_cpp.index("void NavigationGoalExecutor::run_guarded") :
        navigation_executor_cpp.index("void NavigationGoalExecutor::run(")
    ]
    assert "undock_before_navigation_if_needed(" in pre_send_block
    assert guarded_block.index("run_pre_send_sequence(") < guarded_block.index(
        "send_initial_navigation_goal_to_nav2("
    )
    gate_block = dock_contact_cpp[
        dock_contact_cpp.index("  PreNavigationDockCheck snapshot()") : dock_contact_cpp.index(
            "  std::string bms_snapshot_json"
        )
    ]
    assert "current_robot_pose_snapshot" not in gate_block
    assert "find_floor_catalog_pose" not in gate_block
    assert "std::hypot" not in gate_block

    assert "docking_contact_latch_file" in api_cfg
    assert "docking_contact_latch_file" in overlay_api_cfg
    assert "docking_contact_latch_file" in docking_cfg
    assert "docking_contact_latch_file" in overlay_docking_cfg
    assert "dock_contact_latch_is_docked" in docking_cpp
    assert "state_ != State::Docked && !charging_contact_detected_ && !dock_latch_detected" in docking_cpp
    assert "update_dock_contact_latch(false" in docking_cpp
    assert "update_dock_contact_latch(true" in docking_cpp

    assert "DOCKED_CONTACT_BLOCK" in safety_cpp
    assert "block_normal_motion_when_docked" in safety_cpp
    assert "allow_docking_cmd_when_docked" in safety_cpp
    assert "enable_bms_contact_guard" in safety_cpp
    assert "enable_docking_status_guard" in safety_cpp
    assert "docked_status_prefixes" in safety_cpp
    assert "battery_state_topic" in safety_cpp
    assert "docking_status_topic" in safety_cpp
    assert "dock_contact_latch_is_docked" in safety_cpp
    assert "fresh_battery_sample()" in safety_cpp
    assert "docking_status_indicates_docked()" in safety_cpp
    assert "const bool bms_contradicts_latch" in safety_cpp
    assert "const bool status_allows_latch_clear" in safety_cpp
    assert "const bool live_no_contact = bms_contradicts_latch && status_allows_latch_clear" in safety_cpp
    assert "return !live_no_contact" in safety_cpp
    assert "dock_contact_active()" in safety_cpp
    assert "publish_checked_command(*msg, CommandSource::DOCKING)" in safety_cpp
    assert "publish_checked_command(*msg);" in safety_cpp
    assert "fresh_docking_command_active()" in safety_cpp
    assert "last_docking_cmd_ = *msg" in safety_cpp
    assert "have_last_docking_cmd_ = true" in safety_cpp
    assert "publish_command(last_docking_cmd_, snapshot)" in safety_cpp
    assert "sensor_msgs" in safety_cmake
    assert "<depend>sensor_msgs</depend>" in safety_package
    for cfg in (safety_cfg, overlay_safety_cfg):
        assert "block_normal_motion_when_docked: true" in cfg
        assert "allow_docking_cmd_when_docked: true" in cfg
        assert "enable_bms_contact_guard: true" in cfg
        assert "enable_docking_status_guard: true" in cfg
        assert "docked_status_prefixes: [docked, charging]" in cfg
        assert "battery_state_topic: /battery_state" in cfg
        assert "docking_status_topic: /docking/status" in cfg
        assert "docking_contact_latch_file:" in cfg
        assert "dock_contact_max_age_sec: 3.0" in cfg


def test_phase26_persistent_docked_latch_contract():
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    dock_contact_header = (
        ROOT / "src" / "robot_api_server" / "include" / "robot_api_server"
        / "features" / "docking" / "lifecycle" / "dock_contact_interlock_module.hpp"
    ).read_text(encoding="utf-8")
    dock_contact_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "dock_contact_interlock_module.cpp"
    ).read_text(encoding="utf-8")
    docking_http_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_http_module.cpp"
    ).read_text(encoding="utf-8")
    docking_cpp = (
        ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp"
    ).read_text(encoding="utf-8")
    safety_cpp = (ROOT / "src" / "robot_safety" / "src" / "robot_safety_node.cpp").read_text(
        encoding="utf-8"
    )
    safety_cfg = (ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml").read_text(
        encoding="utf-8"
    )
    overlay_safety_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_safety.yaml"
    ).read_text(encoding="utf-8")
    verify_script_path = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_docked_navigation_undock_gate.sh"
    )
    set_latch_script_path = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "set_docked_latch.sh"
    )
    verify_script = verify_script_path.read_text(encoding="utf-8")
    set_latch_script = set_latch_script_path.read_text(encoding="utf-8")
    bms_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "power" / "bms_contact.cpp"
    ).read_text(
        encoding="utf-8"
    )

    assert "latched_docked" in dock_contact_header + dock_contact_cpp
    assert "docked_state_class" in dock_contact_header + dock_contact_cpp
    assert "DOCKED_CONFIRMED" in dock_contact_cpp
    assert "DOCKED_LATCHED" in dock_contact_cpp
    assert "DOCKED_CANDIDATE" not in dock_contact_cpp or "current_robot_pose_snapshot" not in dock_contact_cpp[
        dock_contact_cpp.index("  PreNavigationDockCheck snapshot()") : dock_contact_cpp.index(
            "  std::string bms_snapshot_json"
        )
    ]
    assert "docked_evidence" in dock_contact_header + dock_contact_cpp
    assert "docked_warnings" in dock_contact_header + dock_contact_cpp
    assert "handle_confirm_docked" in docking_http_cpp
    assert "handle_clear_latch" in docking_http_cpp
    assert '"/api/v1/docking/confirm_docked"' in docking_http_cpp
    assert '"/api/v1/docking/clear_docked_latch"' in docking_http_cpp
    assert "sent_velocity" in docking_http_cpp
    assert "maintenance_only" in docking_http_cpp
    assert "fs::rename" in dock_contact_cpp
    assert ".tmp" in dock_contact_cpp

    gate_block = dock_contact_cpp[
        dock_contact_cpp.index("  PreNavigationDockCheck snapshot()") : dock_contact_cpp.index(
            "  std::string bms_snapshot_json"
        )
    ]
    assert "current_robot_pose_snapshot" not in gate_block
    assert "find_floor_catalog_pose" not in gate_block
    assert "std::hypot" not in gate_block
    assert "strong_live_docked" in gate_block
    assert "latch_valid_for_auto_undock" in gate_block
    assert "dock_latch_indicates_docked" in gate_block

    assert "latched_docked" in docking_cpp
    assert "std::filesystem::rename" in docking_cpp
    assert "latched_docked" in safety_cpp
    assert "enable_docked_latch_file_guard" in safety_cpp
    assert "allow_docking_cmd_when_docked" in safety_cpp
    assert "publish_checked_command(*msg, CommandSource::DOCKING)" in safety_cpp
    assert "enable_docked_latch_file_guard: true" in safety_cfg
    assert "enable_docked_latch_file_guard: true" in overlay_safety_cfg

    assert set_latch_script_path.exists()
    assert "--confirm" in set_latch_script
    assert "--clear" in set_latch_script
    assert "--print" in set_latch_script
    assert "ros2 topic pub" not in set_latch_script
    assert "verify_docked_navigation_undock_gate.sh" in set_latch_script
    assert "--confirm-latch" in verify_script
    assert "--clear-latch" in verify_script
    assert "--print-latch" in verify_script
    assert "/api/v1/docking/confirm_docked" in verify_script
    assert "/api/v1/docking/clear_docked_latch" in verify_script

    assert "POWER_SUPPLY_STATUS_FULL" in bms_cpp
    assert "current_above_threshold" in bms_cpp
    assert "present_voltage_valid" in bms_cpp
    assert "full_soc_present_voltage_valid" in bms_cpp
    assert "no_contact status=" in bms_cpp


def test_phase_d2_bms_dock_contact_latch_expiry_contract():
    composition_cpp = application_composition_source()
    dock_contact_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "dock_contact_interlock_module.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    verify_script_path = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_dock_contact_latch_gate.sh"
    )
    verify_script = verify_script_path.read_text(encoding="utf-8")
    power_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "power"
        / "power_module.cpp"
    ).read_text(encoding="utf-8")
    power_feature_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "power"
        / "power_feature_module.cpp"
    ).read_text(encoding="utf-8")
    power_configuration_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "power"
        / "power_configuration_module.cpp"
    ).read_text(encoding="utf-8")

    for text in (docking_configuration_source(), api_cfg, overlay_api_cfg):
        assert "dock_contact_latch_bms_ttl_sec" in text
        assert "dock_contact_latch_bms_clear_no_contact_sec" in text
        assert "dock_contact_latch_allow_bms_stale_auto_undock" in text
        assert "dock_contact_latch_clear_when_live_undocked_no_contact" in text
        assert "dock_contact_latch_max_age_warn_sec" in text
    for text in (power_configuration_cpp, api_cfg, overlay_api_cfg):
        assert "dock_contact_latch_bms_require_contact_sec" in text
    assert "dock_contact_latch_bms_require_contact_sec" not in docking_configuration_source()
    assert "dock_contact_latch_bms_require_contact_sec" not in composition_cpp

    gate_block = dock_contact_cpp[
        dock_contact_cpp.index("  PreNavigationDockCheck snapshot()") : dock_contact_cpp.index(
            "  std::string bms_snapshot_json"
        )
    ]
    assert "check.strong_live_docked" in gate_block
    assert "check.latch_valid_for_auto_undock" in gate_block
    assert 'check.dock_occupancy_state == "DOCKED_CHARGE_IDLE"' in gate_block
    assert "check.final_auto_undock_required = check.final_is_docked_or_charging" in gate_block
    assert "stale_bms_latch_cleared_live_undocked_no_contact" in gate_block
    assert "check.dock_contact_latch_contradicted_by_live_state" in gate_block
    assert "check.live_bms_charging_contact_stable" in gate_block
    assert "check.bms.contact ||" not in gate_block

    handle_bms_block = power_module_cpp[
        power_module_cpp.index("void handle_state") : power_module_cpp.index(
            "BmsChargingContactSnapshot snapshot() const"
        )
    ]
    assert "on_bms_contact_evidence" in power_feature_cpp
    assert "current.contact_stable_duration_sec >= config.contact_stable_required_sec" in handle_bms_block
    assert "ports.on_contact_evidence(contact, contact_stable, contact_stable_duration_sec)" in handle_bms_block
    assert 'update_dock_contact_latch(\n        true,\n        "bms"' not in handle_bms_block
    assert "runtime.navigation_active && ports_.navigation_goal_running()" in dock_contact_cpp

    assert "dock_contact_latch_age_sec" in dock_contact_cpp
    assert "dock_contact_latch_stale" in dock_contact_cpp
    assert "dock_contact_latch_contradicted_by_live_state" in dock_contact_cpp
    assert "dock_contact_latch_auto_cleared" in dock_contact_cpp
    assert "strong_live_docked" in dock_contact_cpp
    assert "latch_valid_for_auto_undock" in dock_contact_cpp
    assert "navigation_relocalize_before_goal_always: false" in api_cfg
    assert "navigation_relocalize_before_goal_always: false" in overlay_api_cfg

    assert verify_script_path.exists()
    assert "Read-only by default" in verify_script
    assert "--clear-stale-bms-latch" in verify_script
    assert "/api/v1/docking/clear_docked_latch" in verify_script
    assert "source=bms stale latch alone" in verify_script
    assert "live undocked/no_contact" in verify_script
    assert "topic_once /docking/status" in verify_script
    assert "topic_once /battery_state" in verify_script


def test_phase_d23_full_charge_charging_session_dock_gate_contract():
    api_cpp_path = ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp"
    api_cpp = api_cpp_path.read_text(encoding="utf-8")
    dock_contact_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "dock_contact_interlock_module.cpp"
    ).read_text(encoding="utf-8")
    bms_contact_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "power"
        / "bms_contact.cpp"
    ).read_text(encoding="utf-8")
    docking_cpp = (
        ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp"
    ).read_text(encoding="utf-8")
    verify_script_path = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_full_charge_dock_session_gate.sh"
    )
    verify_script = verify_script_path.read_text(encoding="utf-8")
    nav2_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml").read_text(
        encoding="utf-8"
    )
    bridge_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")

    gate_block = dock_contact_cpp[
        dock_contact_cpp.index("  PreNavigationDockCheck snapshot()") : dock_contact_cpp.index(
            "  std::string bms_snapshot_json"
        )
    ]
    bms_latch_block = dock_contact_cpp[
        dock_contact_cpp.index("  void on_bms_contact_evidence(") : dock_contact_cpp.index(
            "  void copy_latch_fields"
        )
    ]
    update_latch_block = dock_contact_cpp[
        dock_contact_cpp.index("  void update_latch(") : dock_contact_cpp.index(
            "  bool bms_latch_write_allowed_by_runtime"
        )
    ]

    assert "charging_session" in dock_contact_cpp
    assert '"charging_session"' in bms_latch_block
    assert "bms_charging_observed:" in bms_latch_block
    assert '"bms"' not in bms_latch_block
    assert 'update_dock_contact_latch(true, "charging_session", "bms_charging_observed", "")' in docking_cpp

    assert "latch_source_is_charging_session" in dock_contact_cpp
    assert "source_charging_session" in dock_contact_cpp
    assert "dock_latch_source_strength" in dock_contact_cpp
    assert "dock_contact_latch_source_strength" in dock_contact_cpp
    assert "charging_session_latched" in dock_contact_cpp
    assert "charging_session_age_sec" in dock_contact_cpp
    assert "charging_session_last_confirmed_at" in dock_contact_cpp

    assert "full_charge_idle_on_dock" in gate_block
    assert "bms_soc_full" in gate_block
    assert "bms_current_idle" in gate_block
    assert "strong_session_latch" in gate_block
    assert "SOC=100" not in gate_block
    assert "normalized_soc_percent" in bms_contact_cpp
    assert '"DOCKED_CHARGE_IDLE"' in gate_block
    assert '"UNCERTAIN_ON_DOCK"' in gate_block
    assert '"CONFIRMED_UNDOCKED"' in gate_block
    assert '"CONFIRMED_DOCKED"' in gate_block
    assert '"DOCKED_CHARGING"' in gate_block
    assert "dock_occupancy_state:" in dock_contact_cpp
    assert "check.final_auto_undock_required = check.final_is_docked_or_charging" in gate_block

    assert "check.dock_latch.source_bms" in gate_block
    assert "stale_bms_latch_cleared_live_undocked_no_contact" in gate_block
    assert "latch_source_can_be_cleared_by_live_no_contact" in gate_block
    assert "bms_latch_can_be_cleared_by_live_no_contact" in gate_block
    assert "charging_session_latch_can_be_cleared_by_live_no_contact" in gate_block
    assert "check.dock_latch.source_charging_session" in gate_block
    assert (
        "check.dock_latch.source_charging_session && check.live_docking_state_undocked"
        in gate_block
    )
    assert "charging_session_latch_cleared_confirmed_undocked_no_contact" in gate_block
    assert "stale_charging_session_latch_cleared_live_undocked_no_contact" not in gate_block
    assert "latch_source_is_docking_evidence(check.dock_latch.source)" in gate_block
    assert "latch_source_is_manual_evidence(check.dock_latch.source)" in gate_block
    assert "BMS no_contact" not in dock_contact_cpp
    assert update_latch_block.index("const auto previous = read_latch();") < update_latch_block.index(
        "if (have_last_write_"
    )
    assert "previous.latched_docked == docked" in update_latch_block
    assert "previous.source == source" in update_latch_block
    assert "previous.reason == reason" in update_latch_block

    for field in [
        "dock_occupancy_state",
        "dock_occupancy_evidence",
        "dock_occupancy_reason",
        "charging_session_latched",
        "charging_session_age_sec",
        "charging_session_last_confirmed_at",
        "dock_contact_latch_source_strength",
        "bms_live_contact",
        "bms_live_contact_reason",
        "bms_percentage",
        "bms_current",
        "bms_present",
        "full_charge_idle_on_dock",
        "final_auto_undock_required",
        "auto_undock_reason",
    ]:
        assert field in dock_contact_cpp

    assert verify_script_path.exists()
    for option in [
        "--dry-run",
        "--mock-charging-observed",
        "--mock-full-charge-idle",
        "--mock-bms-no-contact",
        "--mock-docking-job-latch",
        "--expect-auto-undock",
        "--expect-docked-charge-idle",
        "--expect-no-latch-clear",
    ]:
        assert option in verify_script
    assert "does not send goals" in verify_script
    assert "does not publish velocity" in verify_script
    assert "ros2 action send_goal" not in verify_script
    assert "ros2 topic pub" not in verify_script

    assert "MPPIController" in nav2_cfg
    assert "SmacPlanner2D" in nav2_cfg
    assert "transform_tolerance: 0.10" in nav2_cfg
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "PointCloud2" not in dock_contact_cpp
    assert "FAST-LIO" not in dock_contact_cpp
    assert "ranger_base_node" not in gate_block
    assert "pkill -9" not in api_cpp + dock_contact_cpp + verify_script
    assert "killall -9" not in api_cpp + dock_contact_cpp + verify_script

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            subprocess.run([bash, "-n", str(verify_script_path)], check=True)


def test_local_state_uses_robot_localization_ekf_with_system_time_driver():
    cmake = (ROOT / "src" / "robot_local_state" / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (ROOT / "src" / "robot_local_state" / "package.xml").read_text(encoding="utf-8")
    launch_file = (ROOT / "src" / "robot_local_state" / "launch" / "local_state.launch.py").read_text(
        encoding="utf-8"
    )
    ekf_cfg = (ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf.yaml").read_text(
        encoding="utf-8"
    )
    wheel_only_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_wheel_only.yaml"
    ).read_text(encoding="utf-8")
    twist_imu_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_twist_imu.yaml"
    ).read_text(encoding="utf-8")
    twist_imu_vyaw_only_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_twist_imu_vyaw_only.yaml"
    ).read_text(encoding="utf-8")
    twist_wheel_yaw_imu_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_twist_wheel_yaw_imu.yaml"
    ).read_text(encoding="utf-8")
    wheel_xy_imu_yaw_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_wheel_xy_imu_yaw.yaml"
    ).read_text(encoding="utf-8")
    wheel_xy_diff_yaw_imu_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_wheel_xy_diff_yaw_imu.yaml"
    ).read_text(encoding="utf-8")
    wheel_xy_imu_vyaw_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_wheel_xy_imu_vyaw.yaml"
    ).read_text(encoding="utf-8")
    wheel_pose_imu_vyaw_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_wheel_pose_imu_vyaw.yaml"
    ).read_text(encoding="utf-8")
    wheel_imu_primary_vyaw_ekf_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf_wheel_imu_primary_vyaw.yaml"
    ).read_text(encoding="utf-8")
    overlay_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_only_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf_wheel_only.yaml"
    ).read_text(encoding="utf-8")
    overlay_twist_imu_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf_twist_imu.yaml"
    ).read_text(encoding="utf-8")
    overlay_twist_imu_vyaw_only_ekf_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_ekf_twist_imu_vyaw_only.yaml"
    ).read_text(encoding="utf-8")
    overlay_twist_wheel_yaw_imu_ekf_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_ekf_twist_wheel_yaw_imu.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_xy_imu_yaw_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf_wheel_xy_imu_yaw.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_xy_diff_yaw_imu_ekf_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_ekf_wheel_xy_diff_yaw_imu.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_xy_imu_vyaw_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf_wheel_xy_imu_vyaw.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_pose_imu_vyaw_ekf_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_ekf_wheel_pose_imu_vyaw.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_imu_primary_vyaw_ekf_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_ekf_wheel_imu_primary_vyaw.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_odom_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_wheel_odom_ekf.yaml"
    ).read_text(encoding="utf-8")
    source_wheel_odom_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_wheel_odom_ekf.yaml"
    ).read_text(encoding="utf-8")
    overlay_imu_primary_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_imu_primary.yaml"
    ).read_text(encoding="utf-8")
    source_imu_primary_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_imu_primary.yaml"
    ).read_text(encoding="utf-8")
    overlay_pose_soft_yaw_015_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_pose_soft_yaw_015.yaml"
    ).read_text(encoding="utf-8")
    source_pose_soft_yaw_015_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_pose_soft_yaw_015.yaml"
    ).read_text(encoding="utf-8")
    overlay_twist_soft_yaw_012_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_twist_soft_yaw_012.yaml"
    ).read_text(encoding="utf-8")
    source_twist_soft_yaw_012_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_twist_soft_yaw_012.yaml"
    ).read_text(encoding="utf-8")
    overlay_twist_soft_yaw_010_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_twist_soft_yaw_010.yaml"
    ).read_text(encoding="utf-8")
    source_twist_soft_yaw_010_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_twist_soft_yaw_010.yaml"
    ).read_text(encoding="utf-8")
    overlay_twist_soft_yaw_015_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_twist_soft_yaw_015.yaml"
    ).read_text(encoding="utf-8")
    source_twist_soft_yaw_015_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_twist_soft_yaw_015.yaml"
    ).read_text(encoding="utf-8")
    overlay_yaw_offset_m061_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_yaw_offset_m061.yaml"
    ).read_text(encoding="utf-8")
    source_yaw_offset_m061_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_yaw_offset_m061.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_shear_p062_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_shear_p062.yaml"
    ).read_text(encoding="utf-8")
    source_xy_shear_p062_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_shear_p062.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_lateral_m061_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m061.yaml"
    ).read_text(encoding="utf-8")
    source_xy_lateral_m061_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m061.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_lateral_m040_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m040.yaml"
    ).read_text(encoding="utf-8")
    source_xy_lateral_m040_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m040.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_lateral_m050_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m050.yaml"
    ).read_text(encoding="utf-8")
    source_xy_lateral_m050_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m050.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_lateral_m085_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m085.yaml"
    ).read_text(encoding="utf-8")
    source_xy_lateral_m085_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m085.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_lateral_m120_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m120.yaml"
    ).read_text(encoding="utf-8")
    source_xy_lateral_m120_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_m120.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_lateral_soft_yaw_016_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_soft_yaw_016.yaml"
    ).read_text(encoding="utf-8")
    source_xy_lateral_soft_yaw_016_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_soft_yaw_016.yaml"
    ).read_text(encoding="utf-8")
    overlay_xy_lateral_yaw_p979_n1011_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_yaw_p979_n1011.yaml"
    ).read_text(encoding="utf-8")
    source_xy_lateral_yaw_p979_n1011_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_xy_lateral_yaw_p979_n1011.yaml"
    ).read_text(encoding="utf-8")
    overlay_soft_yaw_018_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_018.yaml"
    ).read_text(encoding="utf-8")
    source_soft_yaw_018_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_018.yaml"
    ).read_text(encoding="utf-8")
    overlay_soft_yaw_016_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_016.yaml"
    ).read_text(encoding="utf-8")
    source_soft_yaw_016_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_016.yaml"
    ).read_text(encoding="utf-8")
    overlay_soft_yaw_010_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_010.yaml"
    ).read_text(encoding="utf-8")
    source_soft_yaw_010_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_010.yaml"
    ).read_text(encoding="utf-8")
    overlay_soft_yaw_014_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_014.yaml"
    ).read_text(encoding="utf-8")
    source_soft_yaw_014_wheel_odom_cfg = (
        ROOT
        / "src"
        / "robot_local_state"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_014.yaml"
    ).read_text(encoding="utf-8")
    overlay_soft_yaw_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw.yaml"
    ).read_text(encoding="utf-8")
    source_soft_yaw_wheel_odom_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_wheel_odom_ekf_soft_yaw.yaml"
    ).read_text(encoding="utf-8")
    overlay_soft_yaw_015_wheel_odom_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "local_state_wheel_odom_ekf_soft_yaw_015.yaml"
    ).read_text(encoding="utf-8")
    source_soft_yaw_015_wheel_odom_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_wheel_odom_ekf_soft_yaw_015.yaml"
    ).read_text(encoding="utf-8")
    fastlio_cfg = (ROOT / "src" / "robot_local_state" / "config" / "local_state_fastlio.yaml").read_text(
        encoding="utf-8"
    )
    overlay_fastlio_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_fastlio.yaml"
    ).read_text(encoding="utf-8")
    mapping_fastlio_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "fastlio.yaml"
    ).read_text(encoding="utf-8")
    overlay_imu_bias_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_imu_bias_filter.yaml"
    ).read_text(encoding="utf-8")
    source_imu_bias_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_imu_bias_filter.yaml"
    ).read_text(encoding="utf-8")
    verify_local_state_rates = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_local_state_input_rates.sh"
    ).read_text(encoding="utf-8")
    common_env = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "common_env.sh"
    ).read_text(encoding="utf-8")
    local_state_ekf_profile_env = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf_profile.env"
    ).read_text(encoding="utf-8")
    overlay_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_state.sh"
    ).read_text(encoding="utf-8")
    overlay_tf_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "canonical_tf_helpers.sh"
    ).read_text(encoding="utf-8")
    overlay_common_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh"
    ).read_text(encoding="utf-8")
    local_state_node = (ROOT / "src" / "robot_local_state" / "src" / "local_state_node.cpp").read_text(
        encoding="utf-8"
    )
    imu_bias_node = (
        ROOT / "src" / "robot_local_state" / "src" / "imu_gyro_bias_filter_node.cpp"
    ).read_text(encoding="utf-8")
    overlay_passthrough_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state.yaml"
    ).read_text(encoding="utf-8")
    ranger_chassis_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_ranger_chassis.sh"
    ).read_text(encoding="utf-8")
    ranger_messenger = (
        ROOT
        / "src"
        / "ranger_base"
        / "src"
        / "ranger_messenger.cpp"
    ).read_text(encoding="utf-8")
    cmd_vel_stop_latency = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "record_cmd_vel_stop_latency.sh"
    ).read_text(encoding="utf-8")
    readme = (ROOT / "src" / "robot_local_state" / "README.md").read_text(encoding="utf-8")

    assert "find_package(sensor_msgs REQUIRED)" in cmake
    assert "find_package(tf2 REQUIRED)" in cmake
    assert "add_executable(imu_gyro_bias_filter_node" in cmake
    assert "target_compile_features(imu_gyro_bias_filter_node PUBLIC cxx_std_17)" in cmake
    assert "sensor_msgs" in cmake
    assert "tf2" in cmake
    assert "<exec_depend>robot_localization</exec_depend>" in package_xml
    assert "<depend>sensor_msgs</depend>" in package_xml
    assert "<depend>tf2</depend>" in package_xml
    assert 'package="robot_localization"' in launch_file
    assert 'executable="local_state_node"' in launch_file
    assert 'name="wheel_odom_ekf_input"' in launch_file
    assert 'executable="imu_gyro_bias_filter_node"' in launch_file
    assert 'name="imu_gyro_bias_filter"' in launch_file
    assert 'executable="ekf_node"' in launch_file
    assert "local_state_ekf.yaml" in launch_file
    assert "local_state_wheel_odom_ekf.yaml" in launch_file
    assert "local_state_imu_bias_filter.yaml" in launch_file
    assert '("/odometry/filtered", "/local_state/odometry")' in launch_file
    driver_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_driver.sh"
    ).read_text(encoding="utf-8")
    imu_remap_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_canonical_imu_remap.yaml"
    ).read_text(encoding="utf-8")

    assert 'LOCAL_STATE_MODE:-ekf' in overlay_runner
    assert "robot_local_state/local_state_node" in overlay_runner
    assert "ros2 pkg prefix robot_localization" in overlay_runner
    assert 'EKF_NODE_BIN="${ROBOT_LOCALIZATION_PREFIX}/lib/robot_localization/ekf_node"' in overlay_runner
    assert "ros2 run robot_localization ekf_node" not in overlay_runner
    assert 'njrh_start_affined_background ekf_pid robot_local_state "${EKF_NODE_BIN}"' in overlay_runner
    assert "-r __node:=robot_local_state" in overlay_runner
    assert "LOCAL_STATE_WHEEL_ODOM_EKF_PARAMS_FILE" in overlay_runner
    assert "-r __node:=wheel_odom_ekf_input" in overlay_runner
    assert "LOCAL_STATE_EKF_PROFILE" in overlay_runner
    assert "NJRH_LOCAL_STATE_EKF_PROFILE" in overlay_runner
    assert "local_state_ekf_wheel_pose_imu_vyaw.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_pose_soft_yaw_015.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_twist_soft_yaw_012.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_twist_soft_yaw_010.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_twist_soft_yaw_015.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_yaw_offset_m061.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_shear_p062.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_lateral_m061.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_lateral_m040.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_lateral_m050.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_lateral_m085.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_lateral_m120.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_lateral_soft_yaw_016.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_xy_lateral_yaw_p979_n1011.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_soft_yaw_018.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_soft_yaw_016.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_soft_yaw_015.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_soft_yaw_014.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_soft_yaw_010.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_soft_yaw.yaml" in overlay_runner
    assert "local_state_ekf_wheel_xy_imu_vyaw.yaml" in overlay_runner
    assert "local_state_ekf_wheel_xy_imu_yaw.yaml" in overlay_runner
    assert "local_state_ekf_wheel_xy_diff_yaw_imu.yaml" in overlay_runner
    assert "local_state_ekf_twist_imu.yaml" in overlay_runner
    assert "local_state_ekf_twist_imu_vyaw_only.yaml" in overlay_runner
    assert "local_state_ekf_twist_wheel_yaw_imu.yaml" in overlay_runner
    assert "local_state_ekf_wheel_only.yaml" in overlay_runner
    assert 'invalid LOCAL_STATE_EKF_PROFILE=${EKF_PROFILE}; expected wheel_imu_primary_vyaw' in overlay_runner
    assert "twist_wheel_yaw_imu, or wheel_only" in overlay_runner
    assert "LOCAL_STATE_IMU_BIAS_FILTER_ENABLED" in overlay_runner
    assert (
        "EKF imu0 fusion disabled while IMU bias filter stays available for safety consumers"
        in overlay_runner
    )
    assert "LOCAL_STATE_IMU_BIAS_FILTER_ENABLED=false; skipping IMU gyro bias filter" in overlay_runner
    assert "LOCAL_STATE_IMU_BIAS_FILTER_PARAMS_FILE" in overlay_runner
    assert "imu_gyro_bias_filter_node" in overlay_runner
    assert "-r __node:=imu_gyro_bias_filter" in overlay_runner
    assert "LOCAL_STATE_IMU_BIAS_FILTER_READY_CHECK" in overlay_runner
    assert "runtime_readiness_probe imu-bias-filter" in overlay_runner
    assert "njrh_start_affined_background wheel_odom_pid robot_local_state_odom_preprocessor" in overlay_runner
    assert "njrh_start_affined_background imu_bias_pid robot_local_state_imu_bias_filter" in overlay_runner
    assert "njrh_start_affined_background ekf_pid robot_local_state" in overlay_runner
    assert 'if [[ "${MODE}" == "fastlio" ]]' in overlay_runner
    assert "LOCAL_STATE_FASTLIO_PARAMS_FILE" in overlay_runner
    assert 'fastlio_odom_bridge_node"' in overlay_runner
    assert "njrh_start_affined_background bridge_pid fastlio_odom_bridge" in overlay_runner
    assert 'LOCAL_STATE_FASTLIO_INPUT_TOPIC:-/Odometry' in overlay_runner
    assert 'LOCAL_STATE_FASTLIO_BASE_ODOM_TOPIC:-/fastlio/base_odometry' in overlay_runner
    assert "LOCAL_STATE_FASTLIO_RESTAMP_OUTPUT_TO_NOW:-true" in overlay_runner
    assert "LOCAL_STATE_FASTLIO_OUTPUT_STAMP_OFFSET_SEC:-0.0" in overlay_runner
    assert "LOCAL_STATE_FASTLIO_INPUT_RELIABLE:-false" in overlay_runner
    assert "LOCAL_STATE_FASTLIO_INPUT_QOS_DEPTH:-1" in overlay_runner
    assert "LOCAL_STATE_FASTLIO_OUTPUT_RELIABLE:-true" in overlay_runner
    assert "-p publish_tf:=false" in overlay_runner
    assert "wait_for_fastlio_local_state_endpoint" not in overlay_runner
    assert "FAST-LIO local-state launched; startup readiness probes are disabled" in overlay_runner
    assert "monitor_fastlio_local_state_endpoint()" not in overlay_runner
    assert "LOCAL_STATE_FASTLIO_HEALTH_MAX_FAILURES" not in overlay_runner
    assert "robot_local_state FAST-LIO endpoints stayed missing" not in overlay_runner
    assert 'wait -n "${bridge_pid}" "${local_state_pid}"' in overlay_runner
    assert 'wait -n "${bridge_pid}" "${local_state_pid}" "${monitor_pid}"' not in overlay_runner
    assert "terminate_child()" in overlay_runner
    assert "LOCAL_STATE_STOP_INT_ATTEMPTS" in overlay_runner
    assert "LOCAL_STATE_STOP_TERM_ATTEMPTS" in overlay_runner
    assert 'runtime_readiness_probe local-state-endpoint "${timeout_sec}" fastlio' not in overlay_runner
    assert 'node.get_publishers_info_by_topic("/local_state/odometry")' not in overlay_runner
    assert 'node.get_subscriptions_info_by_topic("/fastlio/base_odometry")' not in overlay_runner
    assert "robot_local_state failed to stay alive" in overlay_runner
    assert "-r /odometry/filtered:=/local_state/odometry" in overlay_runner
    assert "LOCAL_STATE_MODE=ekf" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_only" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_pose_imu_vyaw" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_pose_soft_yaw_015" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_012" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_010" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_015" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_yaw_offset_m061" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m061" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m040" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m050" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m085" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m120" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_soft_yaw_016" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_yaw_p979_n1011" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_018" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_016" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_010" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_015" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_014" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_vyaw" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_yaw" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_xy_diff_yaw_imu" in readme
    assert "LOCAL_STATE_EKF_PROFILE=twist_imu" in readme
    assert "LOCAL_STATE_EKF_PROFILE=twist_imu_vyaw_only" in readme
    assert "LOCAL_STATE_EKF_PROFILE=twist_wheel_yaw_imu" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_only" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu" in readme
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu_primary_vyaw" in readme
    assert "LOCAL_STATE_MODE=passthrough" in readme
    assert 'declare_parameter<double>("odom_yaw_offset_rad", 0.0)' in local_state_node
    assert 'declare_parameter<bool>("rotate_odom_position_with_yaw_offset", true)' in local_state_node
    assert 'declare_parameter<double>("odom_position_scale_x", 1.0)' in local_state_node
    assert 'declare_parameter<double>("odom_position_scale_y", 1.0)' in local_state_node
    assert 'declare_parameter<double>("odom_position_y_to_x_shear", 0.0)' in local_state_node
    assert 'declare_parameter<double>("odom_position_x_to_y_shear", 0.0)' in local_state_node
    assert 'declare_parameter<double>("odom_yaw_scale_positive", 1.0)' in local_state_node
    assert 'declare_parameter<double>("odom_yaw_scale_negative", 1.0)' in local_state_node
    assert 'declare_parameter<bool>("scale_odom_twist_with_yaw_scale", true)' in local_state_node
    assert 'declare_parameter<bool>("anchor_pose_to_first_sample", false)' in local_state_node
    assert 'declare_parameter<bool>("apply_pose_covariance_floor", false)' in local_state_node
    assert 'declare_parameter<double>("pose_covariance_floor_yaw", 0.0)' in local_state_node
    assert 'declare_parameter<std::string>("input_base_frame", "base_link")' in local_state_node
    assert 'declare_parameter<bool>("publish_on_callback", false)' in local_state_node
    assert 'declare_parameter<bool>("republish_latest", true)' in local_state_node
    assert 'declare_parameter<double>("republish_latest_max_age_sec", 0.5)' in local_state_node
    assert "apply_pose_anchor(local_odom)" in local_state_node
    assert "apply_planar_position_calibration(local_odom)" in local_state_node
    assert "apply_yaw_scale_calibration(local_odom)" in local_state_node
    assert "apply_canonical_odom_transform(local_odom)" in local_state_node
    assert "apply_pose_covariance_floor(local_odom)" in local_state_node
    assert "apply_twist_covariance_floor(local_odom)" in local_state_node
    assert "latest_local_odom_ = local_odom" in local_state_node
    assert "if (publish_on_callback_)" in local_state_node
    assert "on_republish_timer" in local_state_node
    assert "publish_on_callback=false and republish_latest=false" in local_state_node
    assert "odom.header.stamp = stamp" in local_state_node
    assert "if (publish_tf_)" in local_state_node
    assert "if (!publish_tf_ || !tf_broadcaster_)" in local_state_node
    assert 'export BASE_FRAME="${BASE_FRAME:-base_link}"' in ranger_chassis_runner
    assert 'RANGER_SPINNING_BASE_TO_CENTER_X="${RANGER_SPINNING_BASE_TO_CENTER_X:-0.0}"' in ranger_chassis_runner
    assert 'RANGER_SPINNING_BASE_TO_CENTER_Y="${RANGER_SPINNING_BASE_TO_CENTER_Y:-0.0}"' in ranger_chassis_runner
    assert 'RANGER_SPINNING_YAW_SCALE_POSITIVE="${RANGER_SPINNING_YAW_SCALE_POSITIVE:-0.976386}"' in ranger_chassis_runner
    assert 'RANGER_SPINNING_YAW_SCALE_NEGATIVE="${RANGER_SPINNING_YAW_SCALE_NEGATIVE:-0.986000}"' in ranger_chassis_runner
    assert (
        'RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE="${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE:-0.960}"'
        in ranger_chassis_runner
    )
    assert (
        'RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE:-0.060}"'
        in ranger_chassis_runner
    )
    assert (
        'RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE:-0.060}"'
        in ranger_chassis_runner
    )
    assert (
        'RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE:-1.041151}"'
        in ranger_chassis_runner
    )
    assert (
        'RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE:-0.977549}"'
        in ranger_chassis_runner
    )
    assert (
        'RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE:-0.030}"'
        in ranger_chassis_runner
    )
    assert (
        'RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER:-0.0}"'
        in ranger_chassis_runner
    )
    assert "Keep navigation base_link at the chassis motion center" in ranger_chassis_runner
    assert "violates the canonical Nav2" in ranger_chassis_runner
    assert '"spinning_base_to_center_x:=${RANGER_SPINNING_BASE_TO_CENTER_X}"' in ranger_chassis_runner
    assert '"spinning_base_to_center_y:=${RANGER_SPINNING_BASE_TO_CENTER_Y}"' in ranger_chassis_runner
    assert '"spinning_yaw_scale_positive:=${RANGER_SPINNING_YAW_SCALE_POSITIVE}"' in ranger_chassis_runner
    assert '"spinning_yaw_scale_negative:=${RANGER_SPINNING_YAW_SCALE_NEGATIVE}"' in ranger_chassis_runner
    assert '"dual_ackermann_odom_use_feedback_twist:=${RANGER_DUAL_ACKERMANN_ODOM_USE_FEEDBACK_TWIST}"' in ranger_chassis_runner
    assert '"dual_ackermann_linear_odom_scale:=${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE}"' in ranger_chassis_runner
    assert (
        '"dual_ackermann_linear_odom_scale_max_abs_yaw_rate:=${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE}"'
        in ranger_chassis_runner
    )
    assert (
        '"dual_ackermann_yaw_scale_max_abs_yaw_rate:=${RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE}"'
        in ranger_chassis_runner
    )
    assert (
        '"dual_ackermann_near_straight_yaw_scale_positive:=${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE}"'
        in ranger_chassis_runner
    )
    assert (
        '"dual_ackermann_near_straight_yaw_scale_negative:=${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE}"'
        in ranger_chassis_runner
    )
    assert (
        '"dual_ackermann_yaw_bias_max_abs_yaw_rate:=${RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE}"'
        in ranger_chassis_runner
    )
    assert (
        '"dual_ackermann_near_straight_yaw_bias_per_meter:=${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER}"'
        in ranger_chassis_runner
    )
    assert '"spinning_zero_cmd_hold_enabled:=${RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED}"' in ranger_chassis_runner
    assert (
        '"spinning_zero_cmd_hold_wz_threshold_radps:=${RANGER_SPINNING_ZERO_CMD_HOLD_WZ_THRESHOLD_RADPS}"'
        in ranger_chassis_runner
    )
    assert "const auto feedback_motion_mode = state.motion_mode_state.motion_mode" in ranger_messenger
    assert "motion_mode_ = feedback_motion_mode" in ranger_messenger
    assert ranger_messenger.index("motion_mode_ = feedback_motion_mode") < ranger_messenger.index("UpdateOdometry(")
    assert 'declare_parameter<double>("dual_ackermann_linear_odom_scale", 1.0)' in ranger_messenger
    assert 'declare_parameter<double>(\n          "dual_ackermann_linear_odom_scale_max_abs_yaw_rate", 0.06)' in ranger_messenger
    assert 'declare_parameter<double>(\n          "dual_ackermann_yaw_scale_max_abs_yaw_rate", 0.06)' in ranger_messenger
    assert 'declare_parameter<double>(\n          "dual_ackermann_near_straight_yaw_scale_positive", 1.0)' in ranger_messenger
    assert 'declare_parameter<double>(\n          "dual_ackermann_near_straight_yaw_scale_negative", 1.0)' in ranger_messenger
    assert 'declare_parameter<double>(\n          "dual_ackermann_yaw_bias_max_abs_yaw_rate", 0.03)' in ranger_messenger
    assert 'declare_parameter<double>(\n          "dual_ackermann_near_straight_yaw_bias_per_meter", 0.0)' in ranger_messenger
    assert "dual_ackermann_linear_odom_scale_ > 0.0" in ranger_messenger
    assert "const bool near_straight = std::abs(yaw_rate) <= yaw_rate_scale_limit" in ranger_messenger
    assert "const double body_vx = raw_body_vx * linear_scale" in ranger_messenger
    assert "SelectDualAckermannYawRate(raw_body_vx, angular, angle)" in ranger_messenger
    assert "ScaleDualAckermannYawRate(yaw_rate, linear)" in ranger_messenger
    assert "dual_ackermann_near_straight_yaw_scale_positive_" in ranger_messenger
    assert "dual_ackermann_near_straight_yaw_scale_negative_" in ranger_messenger
    assert "dual_ackermann_near_straight_yaw_bias_per_meter_" in ranger_messenger
    assert "dual_ackermann_odom_use_feedback_twist" in ranger_messenger
    assert "dual_ackermann_linear_odom_scale" in ranger_messenger
    assert "spinning_base_to_center_x_" in ranger_messenger
    assert "spinning_base_to_center_y_" in ranger_messenger
    assert "spinning_yaw_scale_positive" in ranger_messenger
    assert "spinning_yaw_scale_negative" in ranger_messenger
    assert "ScaleSpinningYawRate(angular)" in ranger_messenger
    assert 'declare_parameter<bool>("spinning_zero_cmd_hold_enabled", true)' in ranger_messenger
    assert 'declare_parameter<double>("spinning_zero_cmd_hold_wz_threshold_radps", 0.03)' in ranger_messenger
    assert "latest_feedback_mode_changing_" in ranger_messenger
    assert "latest_odom_twist_valid_" in ranger_messenger
    assert "latest_odom_angular_velocity_" in ranger_messenger
    assert "const double stop_angular_velocity" in ranger_messenger
    assert "ShouldHoldZeroCommandInSpinningMode()" in ranger_messenger
    assert "robot_->SetMotionCommand(0.0, 0.0, 0.0)" in ranger_messenger
    assert "--linear-speed MPS" in cmd_vel_stop_latency
    assert "linear_speed = float(sys.argv[3])" in cmd_vel_stop_latency
    assert '"wheel_vx"' in cmd_vel_stop_latency
    assert "first_time_zero_twist" in cmd_vel_stop_latency
    assert "input_base_frame: base_link" in overlay_passthrough_cfg
    assert "odom_yaw_offset_rad: 0.0" in overlay_passthrough_cfg
    assert "rotate_odom_position_with_yaw_offset: false" in overlay_passthrough_cfg
    assert "robot_localization/ekf_node" in overlay_tf_helpers
    assert "ekf_node --ros-args.*__node:=robot_local_state" in overlay_tf_helpers
    assert "local_state_node --ros-args" in overlay_tf_helpers
    local_state_reuse_pattern = overlay_tf_helpers.split("local_state*|robot_local_state*)", 1)[1].split(";;", 1)[0]
    assert "__node:=wheel_odom_ekf_input" in local_state_reuse_pattern
    assert "imu_gyro_bias_filter_node" in local_state_reuse_pattern
    assert "__node:=imu_gyro_bias_filter" in local_state_reuse_pattern
    assert "fastlio_odom_bridge_node.py" not in local_state_reuse_pattern
    assert "robot_fastlio_mapping/fastlio_odom_bridge_node" in local_state_reuse_pattern
    assert "fastlio_odom_bridge_node --ros-args" in local_state_reuse_pattern
    assert "local_state_node_process_running()" in overlay_tf_helpers
    assert "fastlio_odom_bridge_process_running()" in overlay_tf_helpers
    assert "local_state_required_processes_running()" in overlay_tf_helpers
    assert "wait_for_local_state_required_processes()" in overlay_tf_helpers
    assert "canonical_cmdline_matches()" in overlay_tf_helpers
    assert "CANONICAL_CMDLINE_PATTERN" in overlay_tf_helpers
    assert 'os.listdir("/proc")' in overlay_tf_helpers
    assert 'pgrep -f "robot_localization/ekf_node|ekf_node --ros-args.*__node:=robot_local_state"' not in overlay_tf_helpers
    assert "fastlio_odom_bridge_process_running && local_state_node_process_running" in overlay_tf_helpers
    assert 'canonical_helper_start_ready "${helper_name}"' in overlay_tf_helpers
    assert "LOCAL_STATE_START_READY_STAGE stage=processes_ready" in overlay_tf_helpers
    assert "LOCAL_STATE_START_READY_STAGE stage=endpoint_ready" in overlay_tf_helpers
    assert "LOCAL_STATE_START_READY_STAGE stage=fresh_tf_ready" in overlay_tf_helpers
    assert 'local_state_endpoint_ready "${LOCAL_STATE_START_READY_TIMEOUT_SEC:-12}" || return 1' in overlay_tf_helpers
    assert 'local_state_runtime_ready "${LOCAL_STATE_START_READY_TIMEOUT_SEC:-12}" || return 1' in overlay_tf_helpers
    assert "helper child process did not become ready" in overlay_tf_helpers
    assert "canonical_wait_for_pid_exit()" in overlay_tf_helpers
    assert "terminate_canonical_helper_pid()" in overlay_tf_helpers
    assert "canonical_descendant_pids()" in overlay_tf_helpers
    assert "helper became ready during final recheck" in overlay_tf_helpers
    assert "LOCAL_STATE_READY_RECHECK_TIMEOUT_SEC" in overlay_tf_helpers
    assert "helper ignored SIGINT; escalating to SIGTERM" in overlay_tf_helpers
    assert "helper did not exit after SIGTERM; killing helper process tree" in overlay_tf_helpers
    assert "canonical_helper_ready()" in overlay_tf_helpers
    assert "canonical_helper_start_ready()" in overlay_tf_helpers
    assert "forget_canonical_helper_pid()" in overlay_tf_helpers
    assert 'runtime_readiness_probe local-state-endpoint "${timeout_sec}" "${mode}"' in overlay_tf_helpers
    assert "local_state_tf_ready()" in overlay_tf_helpers
    assert "local_state_runtime_ready()" in overlay_tf_helpers
    assert 'runtime_health_fresh_tf_ready "odom" "base_link" "${max_age_sec}"' in overlay_tf_helpers
    assert "runtime_readiness_probe_bin" in overlay_tf_helpers
    assert 'fresh-tf "odom" "base_link" "${timeout_sec}" "${max_age_sec}"' in overlay_tf_helpers
    assert "NJRH_RUNTIME_READINESS_PROBE_EXIT_GRACE_SEC" in overlay_tf_helpers
    assert "fresh TF probe did not exit after success" in overlay_tf_helpers
    assert "ranger_chassis_runtime_ready()" in overlay_tf_helpers
    assert "ranger_chassis_liveness_ready()" in overlay_tf_helpers
    assert 'runtime_readiness_probe ranger-chassis "${timeout_sec}"' in overlay_tf_helpers
    assert 'canonical_process_running "${process_pattern}"' in overlay_tf_helpers
    liveness_block = overlay_tf_helpers[
        overlay_tf_helpers.index("ranger_chassis_liveness_ready()") :
        overlay_tf_helpers.index("canonical_helper_ready()", overlay_tf_helpers.index("ranger_chassis_liveness_ready()"))
    ]
    assert 'wait_for_topic_message "/motion_state"' not in liveness_block
    assert 'wait_for_topic_publisher_from_node "/motion_state"' not in liveness_block
    assert 'ranger_chassis_runtime_ready "${RANGER_CHASSIS_REUSE_READY_TIMEOUT_SEC:-3}"' in overlay_tf_helpers
    assert 'ranger_chassis_runtime_ready "${RANGER_CHASSIS_START_READY_TIMEOUT_SEC:-8}"' in overlay_tf_helpers
    assert "RANGER_CHASSIS_START_READY_STAGE stage=topics_ready" in overlay_tf_helpers
    assert "ranger_chassis_common_health_failures=0" in overlay_common_runner
    assert 'NJRH_COMMON_RANGER_CHASSIS_HEALTH_MONITOR:-false' in overlay_common_runner
    assert 'ranger_chassis_liveness_ready "${timeout_sec}"' in overlay_common_runner
    assert "NJRH_COMMON_RANGER_CHASSIS_HEALTH_MAX_FAILURES" in overlay_common_runner
    assert 'NJRH_COMMON_RANGER_CHASSIS_HEALTH_TIMEOUT_SEC:-3' in overlay_common_runner
    assert 'NJRH_COMMON_RANGER_CHASSIS_HEALTH_MAX_FAILURES:-5' in overlay_common_runner
    assert "ranger_chassis_common liveness degraded" in overlay_common_runner
    assert "ranger_chassis_common health lost after" in overlay_common_runner
    assert "NJRH_COMMON_RANGER_CHASSIS_HEALTH_EXIT_ON_LOSS:-false" in overlay_common_runner
    assert "continuing because NJRH_COMMON_RANGER_CHASSIS_HEALTH_EXIT_ON_LOSS=false" in overlay_common_runner
    assert "waiting before next health check" in overlay_common_runner
    assert 'ranger_chassis_runtime_ready "${timeout_sec}"' not in overlay_common_runner
    assert 'local_state_runtime_ready "${LOCAL_STATE_REUSE_READY_TIMEOUT_SEC:-3}"' in overlay_tf_helpers
    assert 'local_state_runtime_ready "${LOCAL_STATE_START_READY_TIMEOUT_SEC:-12}"' in overlay_tf_helpers
    assert "helper endpoint readiness failed" not in overlay_tf_helpers
    assert "existing ${helper_name} process is stale" not in overlay_tf_helpers
    assert "existing ${helper_name} process will be restarted" in overlay_tf_helpers
    assert "cleanup_stale_canonical_helper" in overlay_tf_helpers
    assert 'forget_canonical_helper_pid "${helper_pid}"' in overlay_tf_helpers
    assert "cleanup_owner=common" in overlay_tf_helpers
    assert "robot_fastlio_mapping/fastlio_odom_bridge_node" in overlay_tf_helpers
    assert "robot_local_state/imu_gyro_bias_filter_node" in overlay_tf_helpers
    assert "rosidl_runtime_py.utilities import get_message" not in overlay_tf_helpers
    assert "create_subscription(message_type, topic_name, on_message, qos)" not in overlay_tf_helpers
    assert 'runtime_readiness_probe topic "${topic_name}" "${timeout_sec}"' in overlay_tf_helpers
    assert 'runtime_readiness_probe fresh-header-topic "${topic_name}"' in overlay_tf_helpers
    assert "wait_for_topic_publisher()" in overlay_tf_helpers
    assert 'runtime_readiness_probe topic-publisher "${topic_name}" "${timeout_sec}"' in overlay_tf_helpers
    assert "wait_for_node_name()" in overlay_tf_helpers
    assert 'runtime_readiness_probe node "${expected_node}" "${timeout_sec}"' in overlay_tf_helpers
    assert "wait_for_topic_publisher_from_node()" in overlay_tf_helpers
    assert 'runtime_readiness_probe publisher-from-node "${topic_name}" "${node_name}" "${timeout_sec}"' in overlay_tf_helpers
    assert "LOCAL_STATE_CLEAN_STALE_EKF_MODE" in overlay_runner
    assert "cleanup_stale_ekf_mode_processes" in overlay_runner
    for cfg in (fastlio_cfg, overlay_fastlio_cfg):
        assert "input_odom_topic: /fastlio/base_odometry" in cfg
        assert "publish_rate_hz: 30.0" in cfg
        assert "republish_latest: true" in cfg
        assert "republish_latest_max_age_sec: 0.50" in cfg
        assert "output_topic: /local_state/odometry" in cfg
        assert "publish_tf: true" in cfg
        assert "odom_frame: odom" in cfg
        assert "base_frame: base_link" in cfg
        assert "anchor_pose_to_first_sample: false" in cfg
        assert "apply_pose_covariance_floor: true" in cfg
        assert "apply_twist_covariance_floor: true" in cfg
    for cfg in (ekf_cfg, overlay_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "reset_on_time_jump: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [true, true, false," in cfg
        assert "false, false, true," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
        assert "imu0_pose_rejection_threshold" not in cfg
        assert "imu0_twist_rejection_threshold" not in cfg
    for cfg in (wheel_xy_imu_yaw_ekf_cfg, overlay_wheel_xy_imu_yaw_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [true, true, false," in cfg
        assert "false, false, false," in cfg
        assert "true, false, false," in cfg
        assert "false, false, true," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
    for cfg in (wheel_xy_imu_vyaw_ekf_cfg, overlay_wheel_xy_imu_vyaw_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [true, true, false," in cfg
        assert "false, false, false," in cfg
        assert "true, false, false," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
        assert "corrected JT128 IMU gyro be the only yaw-rate measurement" in cfg
    for cfg in (wheel_xy_diff_yaw_imu_ekf_cfg, overlay_wheel_xy_diff_yaw_imu_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [false, false, false," in cfg
        assert "false, false, true," in cfg
        assert "true, false, false," in cfg
        assert "odom1: /wheel/odom_ekf" in cfg
        assert "odom1_config: [true, true, false," in cfg
        assert "odom1_differential: true" in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
        assert "absolute wheel pose" in cfg
    for cfg in (wheel_pose_imu_vyaw_ekf_cfg, overlay_wheel_pose_imu_vyaw_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [true, true, false," in cfg
        assert "false, false, true," in cfg
        assert "true, false, false," in cfg
        assert "false, false, false," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
        assert "only yaw-rate input" in cfg
    for cfg in (wheel_imu_primary_vyaw_ekf_cfg, overlay_wheel_imu_primary_vyaw_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "reset_on_time_jump: true" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert """odom0_config: [true, true, false,
                   false, false, false,
                   true, false, false,
                   false, false, true,""" in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
        assert "imu0_queue_size: 10" in cfg
        assert "imu0_twist_rejection_threshold" not in cfg
        assert "yaw-rate remains a deliberately soft fallback" in cfg
    for cfg in (twist_imu_ekf_cfg, overlay_twist_imu_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [false, false, false," in cfg
        assert "true, false, false," in cfg
        assert "false, false, true," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
    for cfg in (twist_imu_vyaw_only_ekf_cfg, overlay_twist_imu_vyaw_only_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "reset_on_time_jump: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [false, false, false," in cfg
        assert "false, false, false," in cfg
        assert "true, true, false," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
        assert "corrected JT128 IMU gyro is the only yaw-rate measurement" in cfg
        assert "odom0_pose_rejection_threshold" not in cfg
        assert "odom0_twist_rejection_threshold" not in cfg
        assert "imu0_pose_rejection_threshold" not in cfg
        assert "imu0_twist_rejection_threshold" not in cfg
    for cfg in (twist_wheel_yaw_imu_ekf_cfg, overlay_twist_wheel_yaw_imu_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [false, false, false," in cfg
        assert "false, false, true," in cfg
        assert "true, false, false," in cfg
        assert "false, false, true," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
        assert "do not fuse the chassis-integrated" in cfg
    for cfg in (wheel_only_ekf_cfg, overlay_wheel_only_ekf_cfg):
        assert "frequency: 50.0" in cfg
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [true, true, false," in cfg
        assert "false, false, true," in cfg
        assert "imu0:" not in cfg
        assert "/lidar_imu_bias_corrected" not in cfg
    for wheel_cfg in (source_wheel_odom_cfg, overlay_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "publish_on_callback: false" in wheel_cfg
        assert "republish_latest: true" in wheel_cfg
        assert "republish_latest_max_age_sec: 0.5" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
    for wheel_cfg in (source_yaw_offset_m061_wheel_odom_cfg, overlay_yaw_offset_m061_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: -0.061" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic calibration candidate" in wheel_cfg
    for wheel_cfg in (source_xy_shear_p062_wheel_odom_cfg, overlay_xy_shear_p062_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.062" in wheel_cfg
        assert "odom_position_x_to_y_shear: 0.0" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic shear candidate" in wheel_cfg
    for wheel_cfg in (source_xy_lateral_m061_wheel_odom_cfg, overlay_xy_lateral_m061_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.0" in wheel_cfg
        assert "odom_position_x_to_y_shear: -0.061" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic body-lateral candidate" in wheel_cfg
    for wheel_cfg in (source_xy_lateral_m040_wheel_odom_cfg, overlay_xy_lateral_m040_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.0" in wheel_cfg
        assert "odom_position_x_to_y_shear: -0.040" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic small body-lateral candidate" in wheel_cfg
    for wheel_cfg in (source_xy_lateral_m050_wheel_odom_cfg, overlay_xy_lateral_m050_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.0" in wheel_cfg
        assert "odom_position_x_to_y_shear: -0.050" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic interpolated body-lateral candidate" in wheel_cfg
    for wheel_cfg in (source_xy_lateral_m085_wheel_odom_cfg, overlay_xy_lateral_m085_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.0" in wheel_cfg
        assert "odom_position_x_to_y_shear: -0.085" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic mid body-lateral candidate" in wheel_cfg
    for wheel_cfg in (source_xy_lateral_m120_wheel_odom_cfg, overlay_xy_lateral_m120_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.0" in wheel_cfg
        assert "odom_position_x_to_y_shear: -0.120" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic stronger body-lateral candidate" in wheel_cfg
    for wheel_cfg in (
        source_xy_lateral_soft_yaw_016_wheel_odom_cfg,
        overlay_xy_lateral_soft_yaw_016_wheel_odom_cfg,
    ):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.0" in wheel_cfg
        assert "odom_position_x_to_y_shear: -0.061" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.16" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.16" in wheel_cfg
        assert "Diagnostic combo candidate" in wheel_cfg
    for wheel_cfg in (
        source_xy_lateral_yaw_p979_n1011_wheel_odom_cfg,
        overlay_xy_lateral_yaw_p979_n1011_wheel_odom_cfg,
    ):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "odom_yaw_offset_rad: 0.0" in wheel_cfg
        assert "rotate_odom_position_with_yaw_offset: false" in wheel_cfg
        assert "odom_position_scale_x: 1.0" in wheel_cfg
        assert "odom_position_scale_y: 1.0" in wheel_cfg
        assert "odom_position_y_to_x_shear: 0.0" in wheel_cfg
        assert "odom_position_x_to_y_shear: -0.061" in wheel_cfg
        assert "odom_yaw_scale_positive: 0.979" in wheel_cfg
        assert "odom_yaw_scale_negative: 1.011" in wheel_cfg
        assert "scale_odom_twist_with_yaw_scale: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "Diagnostic combined calibration" in wheel_cfg
    for wheel_cfg in (source_pose_soft_yaw_015_wheel_odom_cfg, overlay_pose_soft_yaw_015_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.15" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.08" in wheel_cfg
        assert "soften integrated wheel yaw while keeping wheel yaw-rate" in wheel_cfg
    for wheel_cfg in (source_twist_soft_yaw_015_wheel_odom_cfg, overlay_twist_soft_yaw_015_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.15" in wheel_cfg
        assert "soften wheel" in wheel_cfg
    for wheel_cfg in (source_twist_soft_yaw_012_wheel_odom_cfg, overlay_twist_soft_yaw_012_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.12" in wheel_cfg
        assert "larger 0.15 candidate" in wheel_cfg
    for wheel_cfg in (source_twist_soft_yaw_010_wheel_odom_cfg, overlay_twist_soft_yaw_010_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.08" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.10" in wheel_cfg
        assert "smallest yaw-rate-only step above the stable 0.08 default" in wheel_cfg
    for wheel_cfg in (source_soft_yaw_018_wheel_odom_cfg, overlay_soft_yaw_018_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.18" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.18" in wheel_cfg
        assert "slightly softer than the 0.15 candidate" in wheel_cfg
    for wheel_cfg in (source_soft_yaw_016_wheel_odom_cfg, overlay_soft_yaw_016_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.16" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.16" in wheel_cfg
        assert "just softer than the 0.15 candidate" in wheel_cfg
    for wheel_cfg in (source_soft_yaw_010_wheel_odom_cfg, overlay_soft_yaw_010_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.10" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.10" in wheel_cfg
        assert "smallest soft-yaw step above the stable 0.08 default" in wheel_cfg
    for wheel_cfg in (source_soft_yaw_014_wheel_odom_cfg, overlay_soft_yaw_014_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.14" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.14" in wheel_cfg
        assert "slightly firmer than the 0.15 candidate" in wheel_cfg
    for wheel_cfg in (source_soft_yaw_wheel_odom_cfg, overlay_soft_yaw_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.25" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.25" in wheel_cfg
        assert "make wheel heading less dominant" in wheel_cfg
    for wheel_cfg in (source_soft_yaw_015_wheel_odom_cfg, overlay_soft_yaw_015_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "apply_pose_covariance_floor: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.15" in wheel_cfg
        assert "apply_twist_covariance_floor: true" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.15" in wheel_cfg
        assert "moderate wheel-heading softening" in wheel_cfg
    for wheel_cfg in (source_imu_primary_wheel_odom_cfg, overlay_imu_primary_wheel_odom_cfg):
        assert "output_topic: /wheel/odom_ekf" in wheel_cfg
        assert "input_odom_topic: /wheel/odom" in wheel_cfg
        assert "anchor_pose_to_first_sample: true" in wheel_cfg
        assert "pose_covariance_floor_x: 0.05" in wheel_cfg
        assert "pose_covariance_floor_y: 0.05" in wheel_cfg
        assert "pose_covariance_floor_yaw: 0.25" in wheel_cfg
        assert "twist_covariance_floor_vx: 0.02" in wheel_cfg
        assert "twist_covariance_floor_vyaw: 0.25" in wheel_cfg
        assert "publish_rate_hz: 50.0" in wheel_cfg
    for imu_cfg in (source_imu_bias_cfg, overlay_imu_bias_cfg):
        assert "imu_topic: /lidar_imu" in imu_cfg
        assert "odom_topic: /wheel/odom_ekf" in imu_cfg
        assert "cmd_vel_topic: /cmd_vel_safe" in imu_cfg
        assert "output_imu_topic: /lidar_imu_bias_corrected" in imu_cfg
        assert "bias_topic: /local_state/imu_bias" in imu_cfg
        assert "transform_output_to_target_frame: true" in imu_cfg
        assert "output_target_frame: base_link" in imu_cfg
        assert "transform_lookup_timeout_sec: 0.02" in imu_cfg
        assert "drop_output_on_transform_failure: true" in imu_cfg
        assert "stationary_required_sec: 2.0" in imu_cfg
        assert "command_motion_holdoff_sec: 1.0" in imu_cfg
        assert "odom_linear_threshold_mps: 0.003" in imu_cfg
        assert "odom_angular_threshold_radps: 0.003" in imu_cfg
        assert "cmd_linear_threshold_mps: 0.001" in imu_cfg
        assert "cmd_angular_threshold_radps: 0.001" in imu_cfg
        assert "max_bias_sample_abs_radps: 0.03" in imu_cfg
        assert "accumulator_alpha: 0.002" in imu_cfg
        assert "zero_output_when_stationary: true" in imu_cfg
        assert "corrected_output_rate_hz: 100.0" in imu_cfg
        assert "bias_publish_rate_hz: 10.0" in imu_cfg
        assert "corrected_output_latest_on_timer: true" in imu_cfg
        assert "corrected_output_preserve_source_stamp: false" in imu_cfg
        assert "corrected_output_max_source_age_sec: 0.20" in imu_cfg
        assert "corrected_output_publish_only_new_sample: true" in imu_cfg
        assert "corrected_output_require_monotonic_stamp: true" in imu_cfg
        assert "override_output_angular_velocity_z_covariance: true" in imu_cfg
        assert "output_angular_velocity_z_covariance: 0.0025" in imu_cfg
        assert "bias_publish_preserve_source_stamp: true" in imu_cfg
        assert yaml_number(imu_cfg, "corrected_output_rate_hz") <= 100.0
        assert yaml_number(imu_cfg, "bias_publish_rate_hz") <= 10.0
    assert "lid_topic: /lidar_points" in mapping_fastlio_cfg
    assert "imu_topic: /lidar_imu" in mapping_fastlio_cfg
    assert "imu_topic: /lidar_imu_bias_corrected" not in mapping_fastlio_cfg
    assert 'declare_parameter<std::string>("output_imu_topic", "/lidar_imu_bias_corrected")' in imu_bias_node
    assert 'declare_parameter<std::string>("bias_topic", "/local_state/imu_bias")' in imu_bias_node
    assert 'declare_parameter<bool>("transform_output_to_target_frame", false)' in imu_bias_node
    assert 'declare_parameter<std::string>("output_target_frame", "base_link")' in imu_bias_node
    assert 'declare_parameter<double>("transform_lookup_timeout_sec", 0.02)' in imu_bias_node
    assert 'declare_parameter<bool>("drop_output_on_transform_failure", true)' in imu_bias_node
    assert 'declare_parameter<double>("command_motion_holdoff_sec", 1.0)' in imu_bias_node
    assert 'declare_parameter<double>("corrected_output_rate_hz", 100.0)' in imu_bias_node
    assert 'declare_parameter<double>("bias_publish_rate_hz", 10.0)' in imu_bias_node
    assert 'declare_parameter<bool>("corrected_output_latest_on_timer", true)' in imu_bias_node
    assert 'declare_parameter<bool>("corrected_output_preserve_source_stamp", true)' in imu_bias_node
    assert 'declare_parameter<double>("corrected_output_max_source_age_sec", 0.20)' in imu_bias_node
    assert 'declare_parameter<bool>("corrected_output_publish_only_new_sample", true)' in imu_bias_node
    assert 'declare_parameter<bool>("corrected_output_require_monotonic_stamp", true)' in imu_bias_node
    assert 'declare_parameter<bool>("override_output_angular_velocity_z_covariance", false)' in imu_bias_node
    assert 'declare_parameter<double>("output_angular_velocity_z_covariance", 0.0025)' in imu_bias_node
    assert 'declare_parameter<bool>("bias_publish_preserve_source_stamp", true)' in imu_bias_node
    assert "stationary_confirmed" in imu_bias_node
    assert "StationaryGate" in imu_bias_node
    assert "sample_is_safe_for_bias_update" in imu_bias_node
    assert "lookupTransform(" in imu_bias_node
    assert "rotate_covariance(" in imu_bias_node
    assert "corrected.header.frame_id = output_target_frame_" in imu_bias_node
    assert "create_publisher<sensor_msgs::msg::Imu>(" in imu_bias_node
    assert "rclcpp::SensorDataQoS().keep_last(1)" in imu_bias_node
    assert "corrected_output_timer_" in imu_bias_node
    assert "bias_publish_timer_" in imu_bias_node
    assert "latest_corrected_imu_" in imu_bias_node
    assert "corrected_source_age_sec" in imu_bias_node
    assert "IMU bias filter rates input=" in imu_bias_node
    assert "corrected.angular_velocity.z = 0.0" in imu_bias_node
    assert "corrected.angular_velocity.z -= bias_.z" in imu_bias_node
    assert "latest_corrected_generation_ == last_published_corrected_generation_" in imu_bias_node
    assert "nonmonotonic_input_skip_count_" in imu_bias_node
    assert "apply_output_angular_velocity_covariance(corrected)" in imu_bias_node
    assert "TransformBroadcaster" not in imu_bias_node
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env
    assert "LOCAL_STATE_EKF_PROFILE_FILE" in common_env
    assert "local_state_ekf_profile.env" in common_env
    assert 'source "${LOCAL_STATE_EKF_PROFILE_FILE}"' in common_env
    assert (
        'export NJRH_LOCAL_STATE_EKF_PROFILE="${NJRH_LOCAL_STATE_EKF_PROFILE:-wheel_imu}"'
        in local_state_ekf_profile_env
    )
    assert "wheel_pose_imu_vyaw" in local_state_ekf_profile_env
    assert "wheel_imu_primary_vyaw" in local_state_ekf_profile_env
    assert "local_state_ekf_wheel_imu_primary_vyaw.yaml" in overlay_runner
    assert "local_state_wheel_odom_ekf_imu_primary.yaml" in overlay_runner
    assert "wheel_imu_pose_soft_yaw_015" in local_state_ekf_profile_env
    assert "wheel_imu_twist_soft_yaw_012" in local_state_ekf_profile_env
    assert "wheel_imu_twist_soft_yaw_010" in local_state_ekf_profile_env
    assert "wheel_imu_twist_soft_yaw_015" in local_state_ekf_profile_env
    assert "wheel_imu_yaw_offset_m061" in local_state_ekf_profile_env
    assert "wheel_imu_xy_shear_p062" in local_state_ekf_profile_env
    assert "wheel_imu_xy_lateral_m061" in local_state_ekf_profile_env
    assert "wheel_imu_xy_lateral_m040" in local_state_ekf_profile_env
    assert "wheel_imu_xy_lateral_m050" in local_state_ekf_profile_env
    assert "wheel_imu_xy_lateral_m085" in local_state_ekf_profile_env
    assert "wheel_imu_xy_lateral_m120" in local_state_ekf_profile_env
    assert "wheel_imu_xy_lateral_soft_yaw_016" in local_state_ekf_profile_env
    assert "wheel_imu_xy_lateral_yaw_p979_n1011" in local_state_ekf_profile_env
    assert "wheel_imu_soft_yaw_018" in local_state_ekf_profile_env
    assert "wheel_imu_soft_yaw_016" in local_state_ekf_profile_env
    assert "wheel_imu_soft_yaw_010" in local_state_ekf_profile_env
    assert "wheel_imu_soft_yaw_015" in local_state_ekf_profile_env
    assert "wheel_imu_soft_yaw_014" in local_state_ekf_profile_env
    assert "wheel_imu_soft_yaw" in local_state_ekf_profile_env
    assert "wheel_xy_imu_vyaw" in local_state_ekf_profile_env
    assert "wheel_xy_imu_yaw" in local_state_ekf_profile_env
    assert "wheel_xy_diff_yaw_imu" in local_state_ekf_profile_env
    assert "twist_imu" in local_state_ekf_profile_env
    assert "twist_imu_vyaw_only" in local_state_ekf_profile_env
    assert "twist_wheel_yaw_imu" in local_state_ekf_profile_env
    assert "wheel_imu" in local_state_ekf_profile_env
    assert "record_rate /lidar_imu " in verify_local_state_rates
    assert "record_rate /lidar_imu_bias_corrected 30 best_effort" in verify_local_state_rates
    assert "record_rate /local_state/imu_bias " in verify_local_state_rates
    assert "ReliabilityPolicy.BEST_EFFORT" in verify_local_state_rates
    assert "candidate == node || candidate == \"/\" node" in verify_local_state_rates
    assert "LOCAL_STATE_IMU_BIAS_FILTER_ENABLED=false skips corrected IMU rate checks" in verify_local_state_rates
    assert "/lidar_imu_bias_corrected has imu_gyro_bias_filter publisher" in verify_local_state_rates
    assert "/lidar_imu_bias_corrected is missing imu_gyro_bias_filter publisher" in verify_local_state_rates
    assert "wheel_only profile must not have EKF subscriber on /lidar_imu_bias_corrected" in verify_local_state_rates
    assert "wheel_only profile has no EKF subscriber on /lidar_imu_bias_corrected" in verify_local_state_rates
    assert "record_rate /wheel/odom_ekf " in verify_local_state_rates
    assert "record_rate /local_state/odometry " in verify_local_state_rates
    assert "RcvbufErrors" in verify_local_state_rates
    assert "robot_local_state EKF process is alive but /robot_local_state is missing" in verify_local_state_rates
    assert "local_state_input_rates_${TIMESTAMP}.md" in verify_local_state_rates
    assert "pkill" not in verify_local_state_rates
    assert "ros2 param set" not in verify_local_state_rates
    assert "ros2 bag" not in verify_local_state_rates
    assert "set_pointcloud_accel_profile" not in verify_local_state_rates
    for cfg in (ekf_cfg, overlay_ekf_cfg):
        assert "imu0_remove_gravitational_acceleration: true" in cfg
        assert "publish_acceleration: false" in cfg
        assert "true, false, false," in cfg
        assert "false, false, true," in cfg
    assert "use_timestamp_type" in driver_script
    assert "use_timestamp_type: 1" in driver_script
    assert "override_angular_velocity_covariance: true" in imu_remap_cfg
    assert "angular_velocity_covariance_diagonal:" in imu_remap_cfg
    assert "- 0.25" in imu_remap_cfg
    assert "mark_orientation_unavailable: true" in imu_remap_cfg


def test_ekf_ab_report_validator_rejects_dirty_navigation_samples(tmp_path):
    script_path = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "validate_ekf_ab_report.py"
    )
    replay_script_path = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "replay_navigation_pingpong_ekf_gate.py"
    )
    profile_ab_script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_ekf_profile_delivery_ab_guarded.sh"
    ).read_text(encoding="utf-8")
    pingpong_script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_delivery_pingpong_guarded.sh"
    ).read_text(encoding="utf-8")
    script_text = script_path.read_text(encoding="utf-8")
    replay_script = replay_script_path.read_text(encoding="utf-8")
    assert "offline-only" in script_text
    assert "ros2 " not in script_text
    assert "curl " not in script_text
    assert "final_pose_verified_not_true" in script_text
    assert "post_relocalize_trigger_not_accepted" in script_text
    assert "trace_last_api_nav_state_not_succeeded" in script_text
    assert "--max-map-base-translation-m" in script_text
    assert "--max-map-odom-translation-m" in script_text
    assert 'VALIDATOR="${SCRIPT_DIR}/validate_ekf_ab_report.py"' in pingpong_script
    assert "--max-online-xy-m" in pingpong_script
    assert "--max-online-yaw-rad" in pingpong_script
    assert "--target-a" in pingpong_script
    assert "--target-b" in pingpong_script
    assert "normalize_delivery_id()" in pingpong_script
    assert 'run_navigation_pose_error_test.sh" \\' in pingpong_script
    assert '--pose-id "$(pose_id_for_target "${target}")"' in pingpong_script
    assert "--goal-completion-policy" in pingpong_script
    assert 'GOAL_COMPLETION_POLICY="pose_required"' in pingpong_script
    assert '--goal-completion-policy "${GOAL_COMPLETION_POLICY}"' in pingpong_script
    assert "--inter-leg-ready-timeout-sec" in pingpong_script
    assert "wait_for_api_navigation_ready()" in pingpong_script
    assert '"post_relocalization_settle_complete"' in pingpong_script
    assert '"active_navigation_goal"' in pingpong_script
    assert "stop_api_not_ready" in pingpong_script
    assert "--max-map-base-translation-m" in pingpong_script
    assert "--max-map-odom-translation-m" in pingpong_script
    assert "--max-correction-yaw-deg" in pingpong_script
    assert "leg${leg}_delivery_${target}_ekf_ab_validation.json" in pingpong_script
    assert "validator_rc" in pingpong_script
    assert "stop_ekf_ab_rejected" in pingpong_script
    assert "correction_metrics.json" not in pingpong_script
    assert "read-only/offline helper" in replay_script
    assert "never sends navigation goals" in replay_script
    assert "never triggers relocalization" in replay_script
    assert "never publishes velocity" in replay_script
    assert "import validate_ekf_ab_report as validator" in replay_script
    assert "--pose-report" in replay_script
    assert "--trace-report" in replay_script
    assert "replay_results.json" in replay_script
    assert "summary.md" in replay_script
    assert "ros2 " not in replay_script
    assert "curl " not in replay_script
    assert 'APPLY="false"' in profile_ab_script
    assert 'PREFLIGHT_ONLY="false"' in profile_ab_script
    assert "--apply" in profile_ab_script
    assert "--preflight-only" in profile_ab_script
    assert "--target-a" in profile_ab_script
    assert "--target-b" in profile_ab_script
    assert "normalize_delivery_id()" in profile_ab_script
    assert "parse_poses_yaml" in profile_ab_script
    assert "--target-a ${TARGET_A} --target-b ${TARGET_B}" in profile_ab_script
    assert "Default mode is dry-run" in profile_ab_script
    assert "does not restart" in profile_ab_script
    assert "Does not restart nodes or move the robot" in profile_ab_script
    assert "wheel_imu_twist_soft_yaw_012|twist_soft_yaw_012" in profile_ab_script
    assert "wheel_imu_twist_soft_yaw_010|twist_soft_yaw_010" in profile_ab_script
    assert "--nav-restart-ready-timeout-sec" in profile_ab_script
    assert "send navigation goals" in profile_ab_script
    assert "--expected-start" in profile_ab_script
    assert "--start-guard-policy" in profile_ab_script
    assert 'START_GUARD_POLICY="readiness_only"' in profile_ab_script
    assert "readiness_only|pose_required" in profile_ab_script
    assert 'policy=${START_GUARD_POLICY}' in profile_ab_script
    assert 'EXPECTED_START="${TARGET_B}"' in profile_ab_script
    assert 'EXPECTED_START="${TARGET_A}"' in profile_ab_script
    assert "--max-start-xy-m" in profile_ab_script
    assert "--max-start-yaw-deg" in profile_ab_script
    assert "--goal-completion-policy" in profile_ab_script
    assert 'GOAL_COMPLETION_POLICY="pose_required"' in profile_ab_script
    assert '--goal-completion-policy "${GOAL_COMPLETION_POLICY}"' in profile_ab_script
    assert "--goal-completion-policy ${GOAL_COMPLETION_POLICY}" in profile_ab_script
    assert "run_start_guard()" in profile_ab_script
    assert '"start_guard_policy": policy' in profile_ab_script
    assert '"pose_within_start_gate": pose_ok' in profile_ab_script
    assert "pose_ok or not pose_required" in profile_ab_script
    assert 'get_json("/api/v1/robot/pose")' in profile_ab_script
    assert 'get_json("/api/v1/navigation/state")' in profile_ab_script
    assert 'get_json("/api/v1/status")' in profile_ab_script
    assert 'status_response.get("localization")' in profile_ab_script
    assert "safe_for_goal_start" in profile_ab_script
    assert "safe_for_goal_start is True" in profile_ab_script
    assert "active_navigation_goal" in profile_ab_script
    assert 'if [[ "${PREFLIGHT_ONLY}" == "true" ]]; then' in profile_ab_script
    assert "preflight-only complete rc=" in profile_ab_script
    assert "no EKF profile switch or navigation was executed" in profile_ab_script
    assert "start guard failed; no EKF profile switch or navigation was executed" in profile_ab_script
    assert 'RESTORE_PROFILE="wheel_imu"' in profile_ab_script
    assert "trap restore_stable_profile EXIT" in profile_ab_script
    assert "LOCAL_STATE_EKF_PROFILE=${profile}" in profile_ab_script
    assert "NJRH_LOCAL_STATE_EKF_PROFILE=${profile}" in profile_ab_script
    assert "LOCAL_STATE_EKF_PROFILE=${PROFILE}" in profile_ab_script
    assert "restart_navigation_runtime_with_profile()" in profile_ab_script
    assert 'FULL_RUNTIME_RESTART_CMD="${NJRH_FULL_RUNTIME_RESTART_CMD:-sudo systemctl restart njrh-runtime.service}"' in profile_ab_script
    assert 'RUNTIME_OVERRIDE_ENV="${NJRH_RUNTIME_OVERRIDE_ENV:-/tmp/njrh_runtime_override.env}"' in profile_ab_script
    assert "write_runtime_override_profile()" in profile_ab_script
    assert "restart_full_runtime_owner()" in profile_ab_script
    assert "NJRH_NAV_LOCAL_STATE_MODE=ekf" in profile_ab_script
    assert 'bash -lc "${FULL_RUNTIME_RESTART_CMD}"' in profile_ab_script
    assert 'bash "${SCRIPT_DIR}/stop_floor_navigation.sh"' not in profile_ab_script
    assert 'nohup bash "${SCRIPT_DIR}/run_floor_navigation.sh"' not in profile_ab_script
    assert 'bash "${SCRIPT_DIR}/check_commercial_runtime_ready.sh"' in profile_ab_script
    assert "wait_for_api_navigation_ready()" in profile_ab_script
    assert "run_api_ready_relocalize()" in profile_ab_script
    assert "api_ready_relocalize_${label}_${profile}" in profile_ab_script
    assert "ekf_ab_${label}_${profile}_api_ready_seed" in profile_ab_script
    assert 'api_get("/api/v1/status")' in profile_ab_script
    assert 'api_get("/api/v1/navigation/state")' in profile_ab_script
    assert '"safe_for_goal_start"' in profile_ab_script
    assert "API navigation context profile=${profile} did not become ready" in profile_ab_script
    assert "run_local_state.sh" not in profile_ab_script
    assert "start_local_state_profile" not in profile_ab_script
    assert "capture_relocalize_correction_compare.sh" in profile_ab_script
    assert "run_navigation_delivery_pingpong_guarded.sh" in profile_ab_script
    assert "restore_relocalize" in profile_ab_script
    assert "pre_ab_relocalize" in profile_ab_script
    assert profile_ab_script.index('run_api_ready_relocalize "${profile}" "${label}"') < profile_ab_script.index('wait_for_api_navigation_ready "${profile}" "${label}"')
    assert "movement_path" in profile_ab_script
    assert "robot_api_server/Nav2 -> robot_safety -> chassis" in profile_ab_script
    assert profile_ab_script.index('if [[ "${PREFLIGHT_ONLY}" == "true" ]]; then') < profile_ab_script.index('if [[ "${APPLY}" != "true" ]]; then')
    assert profile_ab_script.index('run_start_guard "${OUT_DIR}/start_guard"\nstart_guard_rc=$?') < profile_ab_script.index("trap restore_stable_profile EXIT")
    assert profile_ab_script.index("trap restore_stable_profile EXIT") < profile_ab_script.index('restart_navigation_runtime_with_profile "${PROFILE}" "candidate"')
    assert "ros2 topic pub" not in profile_ab_script
    assert "ros2 action send_goal" not in profile_ab_script
    assert "curl " not in profile_ab_script

    clean = tmp_path / "clean_pose"
    clean_post = clean / "post_relocalize_compare"
    clean_post.mkdir(parents=True)
    clean.joinpath("summary.md").write_text(
        "\n".join(
            [
                "# Navigation Pose Error Test Summary",
                "- state: `succeeded`",
                "- phase: `final_pose_verified`",
                "- nav2_result_code: `4`",
                "- final_distance_m: `0.120`",
                "- final_yaw_error_rad: `0.030`",
                "- final_pose_verified: `True`",
                "- relocalize_exit_code: `0`",
                "- map_base_link_delta: translation_m=`0.200`, dyaw_deg=`1.000`, forward_m=`0.0`, left_m=`0.0`",
                "- map_odom_delta: translation_m=`0.100`, dyaw_deg=`1.000`, forward_m=`0.0`, left_m=`0.0`",
                "",
            ]
        ),
        encoding="utf-8",
    )
    clean_post.joinpath("summary.md").write_text(
        "\n".join(
            [
                "# Relocalize Correction Compare",
                "- trigger_accepted: `true`",
                "- trigger_message: `bridge accepted result gate_mode=triggered`",
                "",
            ]
        ),
        encoding="utf-8",
    )

    dirty = tmp_path / "dirty_pose"
    dirty_post = dirty / "post_relocalize_compare"
    dirty_post.mkdir(parents=True)
    dirty.joinpath("summary.md").write_text(
        "\n".join(
            [
                "# Navigation Pose Error Test Summary",
                "- state: `failed`",
                "- phase: `nav2_failed`",
                "- nav2_result_code: `6`",
                "- final_distance_m: `10.703`",
                "- final_yaw_error_rad: `3.063`",
                "- final_pose_verified: `False`",
                "- relocalize_exit_code: `1`",
                "- map_base_link_delta: translation_m=`0.4216`, dyaw_deg=`-0.310`, forward_m=`0.0`, left_m=`0.0`",
                "- map_odom_delta: translation_m=`0.2549`, dyaw_deg=`2.153`, forward_m=`0.0`, left_m=`0.0`",
                "",
            ]
        ),
        encoding="utf-8",
    )
    dirty_post.joinpath("summary.md").write_text(
        "\n".join(
            [
                "# Relocalize Correction Compare",
                "- trigger_accepted: `false`",
                "- trigger_message: `failure_code=BRIDGE_REJECTED_RESULT last_reject_reason=`",
                "",
            ]
        ),
        encoding="utf-8",
    )
    dirty_trace = tmp_path / "dirty_trace"
    dirty_trace.mkdir()
    dirty_trace.joinpath("summary.md").write_text(
        "\n".join(
            [
                "# Navigation Terminal Yaw Trace",
                "- child_rc: `0`",
                "- last_api_nav_state: `failed`",
                "- last_api_nav_phase: `nav2_failed`",
                "- last_api_nav2_result_code: `6`",
                "- last_api_final_distance_m: `10.703`",
                "- last_api_final_yaw_error_rad: `3.063`",
                "",
            ]
        ),
        encoding="utf-8",
    )

    clean_result = subprocess.run(
        [sys.executable, str(script_path), "--pose-report", str(clean), "--json"],
        text=True,
        capture_output=True,
        check=False,
    )
    assert clean_result.returncode == 0, clean_result.stdout + clean_result.stderr
    assert '"accepted": true' in clean_result.stdout

    dirty_result = subprocess.run(
        [
            sys.executable,
            str(script_path),
            "--pose-report",
            str(dirty),
            "--trace-report",
            str(dirty_trace),
            "--json",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert dirty_result.returncode == 10
    for reason in (
        "nav2_result_code_not_4:6",
        "final_pose_verified_not_true:False",
        "post_relocalize_trigger_not_accepted",
        "trace_last_api_nav_state_not_succeeded:failed",
    ):
        assert reason in dirty_result.stdout

    replay_output = tmp_path / "replay"
    replay_result = subprocess.run(
        [
            sys.executable,
            str(replay_script_path),
            "--pose-report",
            str(clean),
            "--trace-report",
            "NONE",
            "--pose-report",
            str(dirty),
            "--trace-report",
            str(dirty_trace),
            "--output-dir",
            str(replay_output),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert replay_result.returncode == 10
    summary = replay_output.joinpath("summary.md").read_text(encoding="utf-8")
    results = replay_output.joinpath("replay_results.json").read_text(encoding="utf-8")
    assert "- read_only: `true`" in summary
    assert "- sends_navigation_goals: `false`" in summary
    assert "| 1 | ACCEPT |" in summary
    assert "| 2 | REJECT |" in summary
    assert "trace_last_api_nav_state_not_succeeded:failed" in summary
    assert '"accepted": false' in results


def test_navigation_terminal_yaw_trace_records_motion_state_and_best_effort_imu():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_terminal_yaw_trace.sh"
    ).read_text(encoding="utf-8")
    assert "This script never publishes velocity commands." in script
    assert "from rclpy.qos import qos_profile_sensor_data" in script
    assert 'create_subscription(Imu, "/lidar_imu_bias_corrected", self.on_imu, qos_profile_sensor_data)' in script
    assert 'self.node.create_subscription(MotionState, "/motion_state", self.on_motion_state, 20)' in script
    for field in (
        "motion_linear_velocity",
        "motion_lateral_velocity",
        "motion_angular_velocity",
        "motion_steering_angle",
        "max_abs_motion_linear_velocity",
        "max_abs_motion_angular_velocity",
        "max_abs_motion_steering_angle",
    ):
        assert field in script
    assert 'motion_float(msg, "linear_velocity")' in script
    assert 'motion_float(msg, "angular_velocity")' in script
    assert "ros2 topic pub" not in script
    assert "ros2 action send_goal" not in script


def test_navigation_pose_error_waits_for_motion_quiet_before_post_relocalize():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_pose_error_test.sh"
    ).read_text(encoding="utf-8")
    wait_call = 'wait_for_motion_quiet "${OUT_DIR}/post_goal_motion_settle.json"'
    relocalize_call = (
        'bash "${SCRIPT_DIR}/capture_relocalize_correction_compare.sh" \\\n'
        '    --output-dir "${OUT_DIR}/post_relocalize_compare"'
    )
    assert "navigation_pose_error_motion_settle_wait" in script
    assert 'GOAL_POST_TIMEOUT_SEC="30.0"' in script
    assert "--goal-post-timeout-sec" in script
    assert "pose_required|position_only|api_default" in script
    assert 'body.pop("goal_completion_policy", None)' in script
    assert '"goal_completion_policy",' in script
    assert '"nav2_goal_yaw_rad",' in script
    assert '"nav2_goal_yaw_source",' in script
    assert "goal_post_timeout_sec = max(float(sys.argv[6]), 1.0)" in script
    assert 'timeout=goal_post_timeout_sec' in script
    assert "timeout=8.0" not in script
    assert "--motion-settle-timeout-sec" in script
    assert "## Post-Goal Motion Settle" in script
    assert wait_call in script
    assert relocalize_call in script
    assert script.index(wait_call) < script.index(relocalize_call)
    assert 'nav_rc="${motion_settle_rc}"' in script
    assert "post-goal relocalize skipped because motion did not settle" in script


def test_navigation_pose_error_records_read_only_odom_motion_samples():
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_pose_error_test.sh"
    ).read_text(encoding="utf-8")
    assert "navigation_pose_error_odom_motion_sampler" in script
    assert "odom_motion_samples.jsonl" in script
    assert "odom_motion_summary.json" in script
    assert 'for topic in ("/wheel/odom", "/wheel/odom_ekf", "/local_state/odometry")' in script
    assert 'for topic in ("/cmd_vel_nav", "/cmd_vel_collision_checked", "/cmd_vel_safe", "/cmd_vel")' in script
    assert 'node.create_subscription(' in script
    assert 'create_publisher' not in script[script.index("start_odom_sampler()") : script.index("summarize_odom_sampler()")]
    assert "## Odom/Motion Samples" in script
    assert "--no-odom-sampler" in script


def test_local_reuse_manifest_exists():
    manifest = ROOT / "src" / "robot_nav_config" / "config" / "local_reuse_sources.yaml"
    text = manifest.read_text(encoding="utf-8")
    assert "D:/codespace/car" in text
    assert "192.168.31.23" in text
    assert "/home/nvidia/workspaces/njrh-v3/workspace1" in text
    assert "/home/nvidia/workspaces/isaac_ros-dev" in text
    assert "/workspaces/isaac_ros-dev" in text
    assert "Dockerfile.car" in text
    assert "njrh_container.sh" in text


def test_bringup_includes_perception_and_safety():
    launch_file = (ROOT / "src" / "robot_bringup" / "launch" / "mock_navigation.launch.py").read_text(encoding="utf-8")
    assert 'include("robot_local_perception"' not in launch_file
    assert 'include("robot_safety"' in launch_file
    assert (ROOT / "src" / "robot_bringup" / "launch" / "localization_bringup.launch.py").exists()
    assert (ROOT / "src" / "robot_bringup" / "launch" / "navigation_bringup.launch.py").exists()


def test_jetson_runtime_assets_exist():
    dockerfile = (ROOT / "Dockerfile.car").read_text(encoding="utf-8")
    assert "ros-humble-robot-localization" in dockerfile
    assert (ROOT / "scripts" / "jetson" / "njrh_container.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "Invoke-NJRHJetson.ps1").exists()
    assert (ROOT / "docs" / "jetson_njrh_container_runtime.md").exists()
    assert (ROOT / "docs" / "commercial_runtime_architecture.md").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_web_dashboard.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_perception.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_costmap_debug.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_safety.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "stop_floor_navigation.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "commercial_runtime_helpers.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "check_commercial_runtime_ready.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_global_localization.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_gs2_driver.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_docking_manager.sh").exists()
    assert "ensure_gs2_device_in_container" in (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(
        encoding="utf-8"
    )
    assert "mknod -m 666 '/dev/${target_name}'" in (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(
        encoding="utf-8"
    )
    assert (ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "install_njrh_autostart.sh").exists()
    assert (ROOT / "docs" / "autostart_systemd.md").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_floor_manager.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "floor_asset_helpers.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "ensure_costmap_filter_masks.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "select_floor_assets.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "promote_map_to_floor.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "bringup_ranger_can.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "shutdown_ranger_can.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_perception.yaml").exists()


def test_orbbec_driver_is_image_owned_and_usb_reenumeration_is_dynamic():
    dockerfile = (ROOT / "Dockerfile.car").read_text(encoding="utf-8")
    orbbec_dockerfile = (ROOT / "Dockerfile.orbbec-runtime").read_text(encoding="utf-8")
    dockerignore = (ROOT / ".dockerignore").read_text(encoding="utf-8")
    container = (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(encoding="utf-8")
    vendor_root = ROOT / "scripts" / "jetson" / "vendor" / "orbbec"

    expected_packages = {
        "ros-humble-diagnostic-updater_4.0.7-1jammy.20260605.154445_arm64.deb",
        "ros-humble-orbbec-camera_2.8.6-1jammy.20260610.141607_arm64.deb",
        "ros-humble-orbbec-camera-msgs_2.8.6-1jammy.20260610.135119_arm64.deb",
        "ros-humble-orbbec-description_2.8.6-1jammy.20260610.135452_arm64.deb",
    }

    assert "COPY scripts/jetson/vendor/orbbec/*.deb" in dockerfile
    assert "/tmp/njrh-orbbec/ros-humble-diagnostic-updater_*.deb" in dockerfile
    assert "/tmp/njrh-orbbec/ros-humble-orbbec-camera_*.deb" in dockerfile
    assert "!scripts/jetson/vendor/orbbec/*.deb" in dockerignore
    assert expected_packages <= {path.name for path in vendor_root.glob("*.deb")}
    assert (vendor_root / "SHA256SUMS").exists()
    assert 'docker_args+=("-v" "/dev/bus/usb:/dev/bus/usb")' in container
    assert "container_has_dynamic_usb_bus_bind" in container
    assert "running container lacks /dev/bus/usb dynamic bind" in container
    assert "prepare_image_build_context" in container
    assert 'sha256sum -c SHA256SUMS' in container
    assert 'rebuild-image)' in container
    assert "build_orbbec_layer_image" in container
    assert 'build-orbbec-layer)' in container
    assert "dpkg -i" in orbbec_dockerfile
    assert "apt-get" not in orbbec_dockerfile
    assert "/tmp/njrh-orbbec/ros-humble-diagnostic-updater_*.deb" in orbbec_dockerfile
    assert 'ldd /opt/ros/humble/lib/liborbbec_camera.so | grep -q "not found"' in orbbec_dockerfile
    assert "njrh.runtime.orbbec" in orbbec_dockerfile
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_safety.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "docking.yaml").exists()
    assert (ROOT / "src" / "robot_nav_config" / "config" / "docking.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "canonical_tf_helpers.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_description.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_state.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_localization_bridge.sh").exists()
    stop_navigation = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "stop_floor_navigation.sh"
    ).read_text(encoding="utf-8")
    assert "controller_server" in stop_navigation
    assert "bt_navigator" in stop_navigation
    assert "occupancy_grid_localizer" in stop_navigation
    assert "map_server" in stop_navigation
    assert "localization_bridge_node" in stop_navigation
    assert "robot_global_localization/global_localization_node" in stop_navigation
    assert "ranger_base_node" not in stop_navigation
    assert "hesai_ros_driver" not in stop_navigation
    assert "robot_api_server" not in stop_navigation
    assert "robot_safety" not in stop_navigation
    assert "run_floor_manager.sh" not in stop_navigation
    assert "robot_floor_manager/floor_manager_node" not in stop_navigation
    assert "floor_manager_node --ros-args" not in stop_navigation
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "prepare_localizer_map.py").exists()
    assert (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_axis_remap_node.cpp").exists()
    assert (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_downsample_node.cpp").exists()
    assert (ROOT / "src" / "robot_hesai_jt128" / "src" / "imu_axis_remap_node.cpp").exists()
    assert (ROOT / "src" / "robot_hesai_jt128" / "src" / "scan_republisher_node.cpp").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "frontend_pose_from_odometry.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "fastlio_mapping_odom_bridge.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "release_rebuild_compat.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization.launch.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization_stack.launch.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_localization_sensing.launch.py").exists()
    assert (ROOT / "src" / "robot_bringup" / "launch" / "local_costmap_debug.launch.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "fastlio.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "patches" / "fast_lio_reliable_lidar_qos.patch").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "localization_bridge.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "sensors.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "occupancy_builder_live.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_flatscan.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_canonical_pointcloud_remap.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_canonical_imu_remap.yaml").exists()
    assert (ROOT / "src" / "robot_nav_config" / "config" / "neutral_keepout_mask.yaml").exists()
    assert (ROOT / "src" / "robot_nav_config" / "config" / "neutral_speed_mask.yaml").exists()
    ensure_masks = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "ensure_costmap_filter_masks.py"
    ).read_text(encoding="utf-8")
    neutral_keepout_yaml = (
        ROOT / "src" / "robot_nav_config" / "config" / "neutral_keepout_mask.yaml"
    ).read_text(encoding="utf-8")
    neutral_speed_yaml = (
        ROOT / "src" / "robot_nav_config" / "config" / "neutral_speed_mask.yaml"
    ).read_text(encoding="utf-8")
    neutral_keepout_pgm = (
        ROOT / "src" / "robot_nav_config" / "config" / "neutral_keepout_mask.pgm"
    ).read_text(encoding="ascii")
    neutral_speed_pgm = (
        ROOT / "src" / "robot_nav_config" / "config" / "neutral_speed_mask.pgm"
    ).read_text(encoding="ascii")
    assert "NEUTRAL_FILTER_PIXEL = 254" in ensure_masks
    assert "stage_source_mask" in ensure_masks
    assert "--keepout-yaml" in ensure_masks
    assert "--stable-wait-sec" in ensure_masks
    assert "using staged costmap filter masks" in ensure_masks
    assert "os.replace(tmp_path, path)" in ensure_masks
    assert "mode: trinary" in ensure_masks
    assert "free_thresh: 0.196" in ensure_masks
    assert "mode: trinary" in neutral_keepout_yaml
    assert "free_thresh: 0.196" in neutral_keepout_yaml
    assert "mode: trinary" in neutral_speed_yaml
    assert "free_thresh: 0.196" in neutral_speed_yaml
    assert neutral_keepout_pgm.endswith("254\n")
    assert neutral_speed_pgm.endswith("254\n")


def test_foreground_nav2_lifecycle_waits_for_current_held_launch_ready():
    resident_runtime = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    foreground_lifecycle_activation = resident_runtime[
        resident_runtime.index("activate_prestarted_nav2_lifecycle()") :
        resident_runtime.index("wait_for_prestarted_nav2_launch_hold_ready()")
    ]

    assert foreground_lifecycle_activation.index(
        "ensure_navigation_layer_alive || return 1"
    ) < foreground_lifecycle_activation.index(
        "wait_for_prestarted_nav2_launch_hold_ready || return 1"
    ) < foreground_lifecycle_activation.index(
        'log_startup_stage "nav2_lifecycle_activation_started"'
    ) < foreground_lifecycle_activation.index(
        'run_nav2_lifecycle_sequence "${timeout_sec}" "${nodes[@]}"'
    )


def test_commercial_runtime_architecture_contract_is_documented():
    doc = (ROOT / "docs" / "commercial_runtime_architecture.md").read_text(encoding="utf-8")
    readiness = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "check_commercial_runtime_ready.sh"
    ).read_text(encoding="utf-8")
    commercial_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "commercial_runtime_helpers.sh"
    ).read_text(encoding="utf-8")
    map_server_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "map_server_helpers.sh"
    ).read_text(encoding="utf-8")
    resident_runtime = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    overlay_nav2_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_nav2_navigation.sh"
    ).read_text(encoding="utf-8")
    standard_navigation_launch = (
        ROOT / "src" / "robot_bringup" / "launch" / "standard_navigation.launch.py"
    ).read_text(encoding="utf-8")
    overlay_common_services = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh"
    ).read_text(encoding="utf-8")
    nav2_lifecycle_sequence = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "nav2_lifecycle_sequence.py"
    ).read_text(encoding="utf-8")
    trigger_client = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "call_global_localization_trigger.py"
    ).read_text(encoding="utf-8")
    lifecycle_doc = (ROOT / "docs" / "runtime_service_lifecycle.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    assert "process state" in doc
    assert "lifecycle state" in doc
    assert "task state" in doc
    assert "Only `odom->base_link` publisher" in doc
    assert "Only `map->odom` publisher" in doc
    assert "Map server" in doc
    assert "Nav2 stack" in doc
    assert "Navigation Goal Admission" in doc
    assert "robot_safety" in doc
    assert "Migration Phases" in doc
    assert "check_commercial_runtime_ready.sh" in lifecycle_doc
    assert "run_navigation_runtime_services.sh" in lifecycle_doc
    assert "commercial_runtime_architecture.md" in readme
    assert "check_lifecycle_active" in readiness
    assert "ros2 lifecycle get" not in readiness
    assert "active [3]" in readiness
    assert "wait_for_global_costmap_static" in readiness
    assert "check_tf \"map\" \"odom\"" in readiness
    assert "check_topic \"/scan\"" in readiness
    assert 'check_lifecycle_active "/collision_monitor" 12' in readiness
    assert 'check_tf "odom" "base_link" 10' in readiness
    assert 'check_topic "/scan" 10' in readiness
    assert 'wait_for_global_costmap_static 10' in readiness
    assert "NJRH_GLOBAL_COSTMAP_FULL_MESSAGE_GATE:-false" in map_server_helpers
    assert "runtime_readiness_probe global-costmap" in map_server_helpers
    assert "NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC:-15" in map_server_helpers
    assert "full OccupancyGrid message gate is deferred" in map_server_helpers
    assert "navigation_runtime_ready_for_current_floor()" in commercial_helpers
    assert "write_runtime_map_context()" in commercial_helpers
    assert "os.getpid()" in commercial_helpers
    assert "NJRH_RUNTIME_STARTUP_STAGE" in commercial_helpers
    assert "startup_elapsed_sec" in commercial_helpers
    assert 'tmp_path = f"{path}.tmp"' not in commercial_helpers
    assert "nav2_lifecycle_manager_reported_active()" in commercial_helpers
    assert "nav2_point_navigation_core_reported_active()" in commercial_helpers
    assert "nav2_critical_processes_running()" in commercial_helpers
    assert "nav2_lifecycle_ready_status_matches()" in commercial_helpers
    assert "NJRH_NAV2_LIFECYCLE_READY_STATUS_FILE" in commercial_helpers
    assert "NAV2_LIFECYCLE_READY_OWNER_PID" in commercial_helpers
    assert ".readlines()" not in commercial_helpers
    assert "__node:=controller_server" in commercial_helpers
    assert "__node:=bt_navigator" in commercial_helpers
    nav2_critical_block = commercial_helpers[
        commercial_helpers.index("nav2_critical_processes_running()") :
        commercial_helpers.index(
            "map_server_asset_matches_current_floor()",
            commercial_helpers.index("nav2_critical_processes_running()"),
        )
    ]
    assert "__node:=planner_server" in nav2_critical_block
    assert "__node:=smoother_server" in nav2_critical_block
    assert "__node:=velocity_smoother" in nav2_critical_block
    assert "__node:=collision_monitor" in nav2_critical_block
    assert "__node:=behavior_server" in nav2_critical_block
    assert "Nav2 lifecycle manager reported managed nodes active and critical processes are running" in commercial_helpers
    assert "Nav2 repo lifecycle sequence reported point-navigation core nodes active" in commercial_helpers
    assert "nav2_lifecycle_node_active()" in commercial_helpers
    assert 'runtime_readiness_probe lifecycle-active "${node_name}" "${timeout_sec}"' in commercial_helpers
    assert "ros2 lifecycle get" not in commercial_helpers
    assert "from lifecycle_msgs.srv import GetState" not in commercial_helpers
    assert 'service_name = f"{node_name}/get_state"' not in commercial_helpers
    assert "runtime_map_context_matches_current_floor()" in commercial_helpers
    assert "navigation_map_source_diagnostics()" in commercial_helpers
    assert "map_server asset is not directly confirmable" in commercial_helpers
    assert 'output="$("$@" 2>&1)"' in commercial_helpers
    assert 'echo "${output}" >&2' in commercial_helpers
    assert "navigation readiness failed: map_server_asset" not in commercial_helpers
    assert "navigation readiness failed: map_topic" not in commercial_helpers
    assert 'source "${SCRIPT_DIR}/canonical_tf_helpers.sh"' in commercial_helpers
    assert "local_state_endpoint" in commercial_helpers
    assert "wait_for_fresh_header_topic_message" in commercial_helpers
    assert "NJRH_NAV_LOCAL_ODOM_MAX_AGE_SEC:-0.75" in commercial_helpers
    assert "/safety/status" in commercial_helpers
    assert "local_costmap_observation" in commercial_helpers
    assert "wait_for_local_costmap_observation_ready" in commercial_helpers
    assert "resolve_floor_assets" in resident_runtime
    assert "run_occupancy_grid_localization.sh" in resident_runtime
    assert "run_nav2_navigation.sh" in resident_runtime
    assert 'NJRH_SKIP_PRESTART_NAV2_STOP="${NJRH_SKIP_PRESTART_NAV2_STOP:-true}"' in resident_runtime
    assert "ensure_common_local_state_ready_for_navigation_start()" in resident_runtime
    assert 'runtime_health_check "local_state_ready"' in resident_runtime
    assert "common local_state ready from runtime health snapshot before navigation startup" in resident_runtime
    assert "LOCAL_STATE_ENDPOINT_NOT_READY" in resident_runtime
    assert "LOCAL_STATE_ODOM_NOT_FRESH" in resident_runtime
    assert "ODOM_BASE_TF_NOT_FRESH" in resident_runtime
    assert "LOCAL_STATE_ODOM_TF_NOT_STABLE" in resident_runtime
    assert "wait_for_stable_local_state_observations" in resident_runtime
    assert 'log_startup_stage "common_local_state_ready"' in resident_runtime
    resident_main_startup_flow = resident_runtime[
        resident_runtime.index('write_runtime_map_context "starting" "false" "resident navigation runtime starting"') :
    ]
    api_preflight_index = resident_main_startup_flow.index(
        'if [[ "${navigation_start_source}" == "api_resume" ]]; then'
    )
    localization_index = resident_main_startup_flow.index("run_occupancy_grid_localization.sh")
    boot_fallback_index = resident_main_startup_flow.index(
        'ensure_common_local_state_ready_for_navigation_start "fresh"',
        localization_index,
    )
    assert api_preflight_index < localization_index < boot_fallback_index
    assert (
        'ensure_common_local_state_ready_for_navigation_start "stable"'
        in resident_main_startup_flow[api_preflight_index:localization_index]
    )
    assert "NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION:-true" in resident_runtime
    assert "NJRH_NAV2_HELD_PRESTART_AFTER_LOCAL_STATE:-true" in resident_runtime
    assert 'log_startup_stage "nav2_layer_started_after_initial_localization"' in resident_runtime
    assert "STARTUP_STAGE" in resident_runtime
    assert "NJRH_RUNTIME_STARTUP_STAGE" in resident_runtime
    assert "NJRH_RUNTIME_STARTUP_ELAPSED_SEC" in resident_runtime
    assert 'log_startup_stage "localization_stack_ready"' in resident_runtime
    assert 'log_startup_stage "initial_global_localization_ready"' in resident_runtime
    assert "trigger_output_reports_retryable_before_dispatch()" in resident_runtime
    assert "NJRH_INITIAL_LOCALIZATION_REQUIRE_RESULT_PUBLISHER:-false" in resident_runtime
    assert "skipping /localization_result publisher pre-gate" in resident_runtime
    assert "NJRH_INITIAL_LOCALIZATION_FLATSCAN_WAIT_SEC:-5" in resident_runtime
    assert "NJRH_INITIAL_LOCALIZATION_FLATSCAN_REPAIR_WAIT_SEC:-20" in resident_runtime
    assert "isaac_triggered_pose_stale_ms" not in resident_runtime
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_ATTEMPT_TIMEOUT:-75" in resident_runtime
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-90" in resident_runtime
    assert "call_global_localization_trigger.py" in resident_runtime
    assert "if rclpy.ok():" in trigger_client
    assert "rclpy.shutdown()" in trigger_client
    assert "ros2 service call \\\n      /global_localization/trigger" not in resident_runtime
    assert "NJRH_FLOOR_MANAGER_SWITCH_CALL_TIMEOUT" not in resident_runtime
    assert "floor_manager source preflight is not repeated during runtime startup" in resident_runtime
    assert "global localization trigger attempt=" in resident_runtime
    assert "proved Isaac was not dispatched; retrying" in resident_runtime
    assert "retrying for fresh result" not in resident_runtime
    assert "transient stale map->odom during startup; retrying" not in resident_runtime
    assert "transient localization result timeout during startup; retrying" not in resident_runtime
    assert "NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION:-true" in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_HOLD" in resident_runtime
    assert '"nav2_layer_prestarted"' in resident_runtime
    assert "activate_prestarted_nav2_lifecycle()" in resident_runtime
    assert "start_prestarted_nav2_lifecycle_background()" in resident_runtime
    assert "wait_for_prestarted_nav2_lifecycle_background()" in resident_runtime
    assert "wait_for_prestarted_nav2_launch_hold_ready()" in resident_runtime
    assert "prestarted Nav2 held launch ready from" in resident_runtime
    assert "NAV2_HOLD_READY_WRAPPER_PID" in resident_runtime
    assert "NAV2_HOLD_READY_BASHPID" in resident_runtime
    assert "NAV2_HOLD_READY_CONTROLLER_PID" in resident_runtime
    assert "NJRH_NAV2_HOLD_READY_FILE" in resident_runtime
    assert "NJRH_NAV2_PRESTART_HOLD_READY_TIMEOUT_SEC:-25" in resident_runtime
    assert "NJRH_NAV2_PRESTART_HOLD_READY_MAX_AGE_SEC" not in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_BACKGROUND_START:-false" in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false" in resident_runtime
    assert "starting prestarted Nav2 lifecycle background after localization stack readiness" in resident_runtime
    assert "final ready still waits for bridge map->odom and active Nav2" in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false" in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false" in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_PARALLEL_BT:-true" in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE:-true" in resident_runtime
    assert "Nav2 lifecycle parallel core activation enabled" in resident_runtime
    assert "Nav2 startup worker running in background" in resident_runtime
    assert "nav2_lifecycle_sequence.py" in resident_runtime
    assert "--trust-change-state-response" in nav2_lifecycle_sequence
    assert "--configure-all-before-activate" in nav2_lifecycle_sequence
    assert "lifecycle sequence: configuring all managed nodes before activation" in nav2_lifecycle_sequence
    assert "lifecycle configure complete node=" in nav2_lifecycle_sequence
    assert "lifecycle activate complete node=" in nav2_lifecycle_sequence
    assert "NJRH_NAV2_LIFECYCLE_NODE_TIMEOUT_SEC:-60" in resident_runtime
    assert "lifecycle_manager_navigation external lifecycle sequence: Managed nodes are active" in resident_runtime
    assert "NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-true" not in resident_runtime
    assert 'run_nav2_lifecycle_sequence "${timeout_sec}" "${nodes[@]}"' in resident_runtime
    assert "write_nav2_hold_ready_status()" in overlay_nav2_script
    assert "NAV2_HOLD_READY_WRAPPER_PID" in overlay_nav2_script
    assert "NAV2_HOLD_READY_CONTROLLER_PID" in overlay_nav2_script
    assert 'rm -f "${NJRH_NAV2_HOLD_READY_FILE}"' in overlay_nav2_script
    assert 'NJRH_NAV2_LAUNCH_NONCRITICAL_NODES="${NJRH_NAV2_LAUNCH_NONCRITICAL_NODES:-false}"' in overlay_nav2_script
    assert "Nav2 noncritical nodes enabled=" in overlay_nav2_script
    assert "NJRH_NAV2_LAUNCH_NONCRITICAL_NODES" in standard_navigation_launch
    assert "launch_noncritical_navigation_nodes" in standard_navigation_launch
    assert "noncritical_navigation_nodes = []" in standard_navigation_launch
    assert "start_navigation_lifecycle_with_repo_sequence()" in overlay_nav2_script
    assert "NJRH_NAV2_USE_REPO_LIFECYCLE_SEQUENCE:-true" in overlay_nav2_script
    assert "NJRH_NAV2_BACKGROUND_NONCRITICAL_LIFECYCLE:-true" in overlay_nav2_script
    assert "lifecycle_manager_navigation external repo lifecycle sequence: point-navigation core nodes are active" in overlay_nav2_script
    assert overlay_nav2_script.index("behavior_server") < overlay_nav2_script.index("smoother_server")
    assert overlay_nav2_script.index("smoother_server") < overlay_nav2_script.index("bt_navigator")
    assert overlay_nav2_script.index("bt_navigator") < overlay_nav2_script.index("waypoint_follower")
    assert "Nav2 point-navigation core lifecycle nodes are active" in commercial_helpers
    critical_lifecycle_block = commercial_helpers[
        commercial_helpers.index("critical_nav2_lifecycle_nodes()") :
        commercial_helpers.index("nav2_lifecycle_node_active()", commercial_helpers.index("critical_nav2_lifecycle_nodes()"))
    ]
    assert "/behavior_server" in critical_lifecycle_block
    assert "/smoother_server" in critical_lifecycle_block
    critical_process_block = commercial_helpers[
        commercial_helpers.index("nav2_critical_processes_running()") :
        commercial_helpers.index("standard_nav_stack_lifecycle_active()", commercial_helpers.index("nav2_critical_processes_running()"))
    ]
    assert "__node:=behavior_server" in critical_process_block
    assert "__node:=smoother_server" in critical_process_block
    resident_prestart_lifecycle_nodes = resident_runtime[
        resident_runtime.index("activate_prestarted_nav2_lifecycle()") :
        resident_runtime.index("ensure_navigation_layer_alive || return 1", resident_runtime.index("activate_prestarted_nav2_lifecycle()"))
    ]
    assert resident_prestart_lifecycle_nodes.index("planner_server") < resident_prestart_lifecycle_nodes.index(
        "controller_server"
    )
    assert "waypoint_follower" not in resident_prestart_lifecycle_nodes
    assert "behavior_server" in resident_prestart_lifecycle_nodes
    assert "smoother_server" in resident_prestart_lifecycle_nodes
    assert resident_prestart_lifecycle_nodes.index("behavior_server") < resident_prestart_lifecycle_nodes.index(
        "smoother_server"
    )
    assert resident_prestart_lifecycle_nodes.index("smoother_server") < resident_prestart_lifecycle_nodes.index(
        "bt_navigator"
    )
    resident_background_lifecycle_nodes = resident_runtime[
        resident_runtime.index("start_prestarted_nav2_lifecycle_background()") :
        resident_runtime.index(
            "ensure_navigation_layer_alive || return 1",
            resident_runtime.index("start_prestarted_nav2_lifecycle_background()"),
        )
    ]
    assert "waypoint_follower" not in resident_background_lifecycle_nodes
    assert "behavior_server" in resident_background_lifecycle_nodes
    assert "smoother_server" in resident_background_lifecycle_nodes
    assert 'log_startup_stage "nav2_lifecycle_activation_started"' in resident_runtime
    assert "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-false" in resident_runtime
    assert "prestarting resident AMCL warmup before initial global localization" in resident_runtime
    assert "deferring resident AMCL warmup until after initial global localization" in resident_runtime
    assert "NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false" in resident_runtime
    assert (
        "initial global localization trigger is starting after full localization-stack "
        "and floor-context readiness"
    ) in resident_runtime
    assert (
        "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION="
        '"${NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-false}"'
    ) in overlay_common_services
    assert (
        "NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START="
        '"${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}"'
    ) in overlay_common_services
    resident_runtime_main_flow = resident_runtime[resident_runtime.index("if resident_navigation_ready; then") :]
    assert resident_runtime_main_flow.index("start_amcl_resident_background_if_enabled_for_navigation") < resident_runtime_main_flow.index(
        "start_initial_global_localization_background"
    )
    assert resident_runtime_main_flow.index("ensure_localization_stack_ready_for_navigation") < resident_runtime_main_flow.index(
        "start_initial_global_localization_background"
    )
    assert resident_runtime_main_flow.index("wait_for_initial_global_localization") < resident_runtime_main_flow.index(
        "start_amcl_readiness_background_if_enabled_for_navigation"
    )
    assert "start_amcl_readiness_background_if_enabled_for_navigation()" in resident_runtime
    assert "wait_for_amcl_readiness_background_if_running()" in resident_runtime
    assert "complete_amcl_readiness_with_retries_for_navigation()" in resident_runtime
    assert "AMCL readiness running in background" in resident_runtime
    assert "NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY:-false" in resident_runtime
    assert "NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE:-false" in resident_runtime
    assert "deferring AMCL readiness until after Nav2 lifecycle activation" in resident_runtime
    assert "AMCL tracking readiness continues in background" in resident_runtime
    assert "AMCL tracking continues in background" in resident_runtime
    resident_amcl_background_block = resident_runtime[
        resident_runtime.index("start_amcl_readiness_background_if_enabled_for_navigation()") :
        resident_runtime.index(
            "wait_for_amcl_readiness_background_if_running()",
            resident_runtime.index("start_amcl_readiness_background_if_enabled_for_navigation()"),
        )
    ]
    assert "wait_for_amcl_resident_background_if_running" in resident_amcl_background_block
    assert "complete_amcl_readiness_with_retries_for_navigation" in resident_amcl_background_block
    assert "AMCL readiness completed in background" in resident_amcl_background_block
    assert "continuing into bounded complete-readiness retries" in resident_amcl_background_block
    assert "maintain_amcl_readiness_background_for_navigation()" in resident_runtime
    assert "AMCL readiness background is not running and status is not ready" in resident_runtime
    resident_supervision_loop = resident_runtime[
        resident_runtime.index("while true; do", resident_runtime.index("resident navigation runtime launched")) :
    ]
    assert "maintain_amcl_readiness_background_for_navigation" in resident_supervision_loop
    assert "AMCL runtime status heartbeat exited" not in resident_supervision_loop
    assert "amcl_status_heartbeat_pid" not in resident_runtime
    assert "amcl_status_client_for_navigation ping >/dev/null" in resident_runtime
    assert "AMCL readiness completion attempt=" in resident_runtime
    assert "AMCL_READINESS_TIMEOUT" in resident_runtime
    assert "NJRH_NAV2_EXTERNAL_LIFECYCLE_BRINGUP:-true" in resident_runtime
    assert "NJRH_NAV2_EXTERNAL_LIFECYCLE_READY_TIMEOUT:-210" in resident_runtime
    assert "complete_amcl_readiness_if_enabled_for_navigation ||" not in resident_runtime
    assert 'log_startup_stage "nav2_layer_ready"' in resident_runtime
    assert 'log_startup_stage "amcl_tracking_ready"' in resident_runtime
    assert "Nav2 activation, and AMCL tracking readiness" in resident_runtime
    assert "resident Nav2 layer failed startup; tearing down the incomplete navigation runtime" in resident_runtime
    assert "stop_navigation_layer_after_failure()" in resident_runtime
    assert "sweeping standard Nav2 stack after resident Nav2" in resident_runtime
    assert "stopping the localization layer so a later /navigation/start begins from a clean process set" in resident_runtime
    assert "stop_existing_standard_nav_stack" in resident_runtime
    assert 'navigation_pid=""' in resident_runtime
    nav2_startup_failure_block = resident_runtime[
        resident_runtime.index('if ! wait_for_nav2_layer_ready; then') :
        resident_runtime.index('else', resident_runtime.index('if ! wait_for_nav2_layer_ready; then'))
    ]
    assert 'stop_navigation_layer_after_failure "startup failure"' in nav2_startup_failure_block
    assert "exit 1" in nav2_startup_failure_block
    assert 'terminate_child "${navigation_pid}" "resident Nav2 layer"\n  navigation_pid=""' not in nav2_startup_failure_block
    resident_runtime_main = resident_runtime[
        resident_runtime.index('write_runtime_map_context "starting" "false" "resident navigation runtime starting"') :
    ]
    assert resident_runtime_main.index('log_startup_stage "initial_global_localization_ready"') < resident_runtime_main.index(
        "start_amcl_readiness_background_if_enabled_for_navigation"
    )
    assert resident_runtime_main.index('log_startup_stage "initial_global_localization_ready"') < resident_runtime_main.index(
        'log_startup_stage "nav2_layer_started_after_initial_localization"'
    )
    assert resident_runtime_main.index("activate_prestarted_nav2_lifecycle") < resident_runtime_main.index(
        'log_startup_stage "nav2_layer_ready"'
    )
    assert resident_runtime_main.index('log_startup_stage "nav2_layer_ready"') < resident_runtime_main.index(
        "wait_for_amcl_readiness_background_if_running"
    )
    assert resident_runtime_main.index("wait_for_amcl_readiness_background_if_running") < resident_runtime_main.index(
        'commit_runtime_ready_context "${ready_context_message}"'
    )
    assert "resident_navigation_ready" in resident_runtime
    assert "localization stack ready for initial relocalization" in resident_runtime
    assert "initial localization accepted: bridge_status.has_map_to_odom=true and map->odom are ready" in resident_runtime
    assert "Nav2 lifecycle and global costmap are ready" in resident_runtime
    assert "navigation_runtime_ready_for_current_floor 3" not in resident_runtime
    assert "wait_for_resident_navigation_runtime_ready" not in resident_runtime
    assert "floor_id is required for resident navigation runtime" in resident_runtime


def test_runtime_readiness_probe_has_process_timeout_guard():
    common_env = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "common_env.sh"
    ).read_text(encoding="utf-8")
    probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "runtime_readiness_probe()" in common_env
    assert "runtime_readiness_probe_output_reports_success()" in common_env
    assert "NJRH_RUNTIME_READINESS_PROBE_PROCESS_TIMEOUT_SEC" in common_env
    assert "NJRH_RUNTIME_READINESS_PROBE_SHUTDOWN_GRACE_SEC" in common_env
    assert "NJRH_RUNTIME_READINESS_PROBE_KILL_AFTER_SEC" in common_env
    assert 'timeout --kill-after="${NJRH_RUNTIME_READINESS_PROBE_KILL_AFTER_SEC:-2}"' in common_env
    assert '"${timeout_sec}" "${probe}" "$@"' in common_env
    assert "readiness probe reported success but did not exit cleanly" in common_env
    assert "[runtime-overlay] lifecycle node active:" in common_env
    assert "[runtime-overlay] TF ready:" in common_env
    assert '"${probe}" "$@"' not in common_env.replace('"${timeout_sec}" "${probe}" "$@"', "")
    assert "std::_Exit(code)" in probe_cpp
    assert "exit_probe(ok ? 0 : 1)" in probe_cpp
    assert "Fast DDS" in probe_cpp
    assert "rclcpp::shutdown()" not in probe_cpp


def test_api_navigation_resume_uses_a_stable_local_state_gate_without_changing_boot_source():
    probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")
    helpers = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "nav_runtime_helpers.sh"
    ).read_text(encoding="utf-8")
    common = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_common_services.sh"
    ).read_text(encoding="utf-8")
    api_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp"
    ).read_text(encoding="utf-8")

    assert "stable-local-state" in probe_cpp
    assert "required_consecutive_good" in probe_cpp
    assert "stable local_state ready:" in probe_cpp
    stable_probe = probe_cpp.split("bool wait_for_stable_local_state", 1)[1].split(
        "bool wait_for_transformable_scan", 1
    )[0]
    assert "TransformListener" not in stable_probe
    assert stable_probe.count(
        "create_subscription<nav_msgs::msg::Odometry>"
    ) == 1
    assert stable_probe.count(
        "create_subscription<tf2_msgs::msg::TFMessage>"
    ) == 1
    assert '"/tf"' in stable_probe
    assert "wait_for_stable_local_state_observations()" in helpers
    assert "runtime_readiness_probe stable-local-state" in helpers
    assert 'NJRH_NAVIGATION_START_SOURCE="systemd_autostart"' in common
    assert '::setenv("NJRH_NAVIGATION_START_SOURCE", "api_resume", 1);' in api_cpp


def test_ranger_chassis_readiness_uses_one_dds_participant():
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    common_env = (scripts_root / "common_env.sh").read_text(encoding="utf-8")
    canonical_helpers = (scripts_root / "canonical_tf_helpers.sh").read_text(encoding="utf-8")
    probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")

    readiness_block = canonical_helpers[
        canonical_helpers.index("ranger_chassis_runtime_ready()") :
        canonical_helpers.index("ranger_chassis_liveness_ready()")
    ]
    assert 'runtime_readiness_probe ranger-chassis "${timeout_sec}"' in readiness_block
    assert 'wait_for_topic_publisher_from_node "/wheel/odom"' not in readiness_block
    assert 'wait_for_fresh_header_topic_message "/wheel/odom"' not in readiness_block
    assert 'wait_for_topic_publisher_from_node "/motion_state"' not in readiness_block
    assert 'wait_for_topic_message "/motion_state"' not in readiness_block
    assert "wait_for_ranger_chassis" in probe_cpp
    assert "ranger chassis ready:" in probe_cpp
    assert 'command == "ranger-chassis"' in probe_cpp
    assert "ranger-chassis)" in common_env
    assert "[runtime-overlay] ranger chassis ready:" in common_env


def test_imu_bias_filter_readiness_uses_one_dds_participant():
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    common_env = (scripts_root / "common_env.sh").read_text(encoding="utf-8")
    local_state = (scripts_root / "run_local_state.sh").read_text(encoding="utf-8")
    probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")

    readiness_block = local_state[
        local_state.index('if [[ "${LOCAL_STATE_IMU_BIAS_FILTER_READY_CHECK:-true}" == "true" ]]') :
        local_state.index("  fi\nelse", local_state.index("LOCAL_STATE_IMU_BIAS_FILTER_READY_CHECK"))
    ]
    assert "runtime_readiness_probe imu-bias-filter" in readiness_block
    assert "runtime_readiness_probe publisher-from-node" not in readiness_block
    assert "runtime_readiness_probe topic" not in readiness_block
    assert "wait_for_imu_bias_filter" in probe_cpp
    assert "IMU bias filter ready:" in probe_cpp
    assert 'command == "imu-bias-filter"' in probe_cpp
    assert "imu-bias-filter)" in common_env
    assert "[runtime-overlay] IMU bias filter ready:" in common_env


def test_localization_stack_readiness_uses_one_dds_participant_on_normal_path():
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    common_env = (scripts_root / "common_env.sh").read_text(encoding="utf-8")
    resident = (scripts_root / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")

    readiness_block = resident[
        resident.index("ensure_localization_stack_ready_for_navigation()") :
        resident.index("wait_for_nav2_layer_ready()")
    ]
    compound_index = readiness_block.index("runtime_readiness_probe localization-stack")
    fallback_index = readiness_block.index('wait_for_ros_service "/global_localization/trigger"')
    assert compound_index < fallback_index
    assert "falling back to detailed localization readiness diagnostics" in readiness_block
    assert "wait_for_localization_stack" in probe_cpp
    assert "localization stack ready:" in probe_cpp
    assert 'command == "localization-stack"' in probe_cpp
    assert "localization-stack)" in common_env
    assert "[runtime-overlay] localization stack ready:" in common_env


def test_global_costmap_readiness_uses_one_dds_participant():
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    common_env = (scripts_root / "common_env.sh").read_text(encoding="utf-8")
    map_helpers = (scripts_root / "map_server_helpers.sh").read_text(encoding="utf-8")
    probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")

    readiness_block = map_helpers[
        map_helpers.index("wait_for_global_costmap_static()") :
        map_helpers.index("\n}", map_helpers.index("wait_for_global_costmap_static()"))
    ]
    assert "runtime_readiness_probe global-costmap" in readiness_block
    assert 'runtime_readiness_probe lifecycle-active "/global_costmap/global_costmap"' not in readiness_block
    assert 'runtime_readiness_probe topic-publisher "/global_costmap/costmap"' not in readiness_block
    assert "wait_for_global_costmap" in probe_cpp
    assert "global costmap ready:" in probe_cpp
    assert 'command == "global-costmap"' in probe_cpp
    assert "global-costmap)" in common_env
    assert "[runtime-overlay] global costmap ready:" in common_env


def test_global_localization_requires_stable_post_arm_flatscan_window():
    source = (
        ROOT / "src" / "robot_global_localization" / "src" / "global_localization_node.cpp"
    ).read_text(encoding="utf-8")
    config = (
        ROOT / "src" / "robot_global_localization" / "config" / "global_localization.yaml"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter<int>("localizer_input_required_consecutive_good", 2)' in source
    assert "localizer_input_required_consecutive_good_" in source
    post_arm_block = source[
        source.index("bool wait_for_localizer_input_after_arm(") :
        source.index("bool localization_result_is_fresh_for_trigger(")
    ]
    assert "last_examined_seq" in post_arm_block
    assert "consecutive_good" in post_arm_block
    assert "consecutive_good = 0" in post_arm_block
    assert "snapshot.seq == last_examined_seq" in post_arm_block
    assert "required_consecutive_good" in post_arm_block
    assert "consecutive_good >= required_consecutive_good" in post_arm_block
    assert "localizer_input_required_consecutive_good: 2" in config


def test_nav2_prestart_waits_for_bridge_odom_in_one_dds_participant():
    scripts_root = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    common_env = (scripts_root / "common_env.sh").read_text(encoding="utf-8")
    resident = (scripts_root / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")
    bridge_cpp = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")

    prestart_block = resident[
        resident.index('NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE:-true') :
        resident.index('start_resident_navigation_layer "true" "nav2_layer_prestarted_held"')
    ]
    assert "runtime_readiness_probe localization-prestart" in prestart_block
    assert 'wait_for_ros_service "/trigger_grid_search_localization"' not in prestart_block
    assert "wait_for_localization_prestart" in probe_cpp
    assert "wait_for_bridge_odom_status" in probe_cpp
    assert '"\\\"has_odom\\\":true"' in probe_cpp
    assert 'command == "localization-prestart"' in probe_cpp
    assert "localization-prestart)" in common_env
    assert "[runtime-overlay] localization prestart ready:" in common_env
    assert '<< ",\\\"has_odom\\\":" << (has_odom_ ? "true" : "false")' in bridge_cpp


def test_runtime_overlay_lidar_view_defaults_to_base_link():
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    run_web_dashboard = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_web_dashboard.sh").read_text(encoding="utf-8")
    common_env = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "common_env.sh").read_text(encoding="utf-8")
    assert "--lidar-view-html" in dashboard_patch
    assert "DEFAULT_DISPLAY_FRAME = 'base_link'" in dashboard_patch
    assert "小车底盘坐标 base_link" in dashboard_patch
    assert "原始传感器坐标（仅调试）" in dashboard_patch
    assert "cp \"${UPSTREAM_WEB}/lidar_view.html\"" in run_web_dashboard
    assert "--lidar-view-html" in run_web_dashboard
    assert "ReliabilityPolicy.BEST_EFFORT" in dashboard_patch
    assert "depth=10" in dashboard_patch
    assert "def _driver_stack_running(self) -> bool:" in dashboard_patch
    assert "def _probe_axis_remap_status_external(self, timeout: float = 6.0) -> bool:" in dashboard_patch
    assert "ros2 topic echo /lidar/axis_remap_status --field data" in dashboard_patch
    assert "ros2 topic hz /lidar_points" not in dashboard_patch
    assert "Timed out waiting for lidar axis remap status" in dashboard_patch
    assert "driver live axis status ok; process stack probe incomplete" in dashboard_patch
    assert "stale driver stack restarted for canonical lidar ingress" in dashboard_patch
    assert "pointcloud_axis_remap" in dashboard_patch
    assert "imu_axis_remap" in dashboard_patch
    assert "def stop_core(self) -> dict:" in dashboard_patch
    assert "'scripts/run_driver.sh'" in dashboard_patch
    assert "'run_projected_map.sh'" in dashboard_patch
    assert "self._invalidate_runtime_caches()" in dashboard_patch
    assert "seed_runtime_dir" in run_web_dashboard
    assert 'seed_runtime_dir "maps"' in run_web_dashboard
    assert 'export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"' in run_web_dashboard
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert 'export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"' in common_env
    assert "configure_fastdds_interface_whitelist" in common_env
    assert 'NJRH_FASTDDS_ALLOWED_INTERFACES:-lo,wlan0' in common_env
    assert 'profile_file="/tmp/njrh_fastdds_profile_$(id -u).xml"' in common_env
    assert 'export FASTRTPS_DEFAULT_PROFILES_FILE="${profile_file}"' in common_env
    assert 'export FASTDDS_DEFAULT_PROFILES_FILE="${profile_file}"' in common_env
    assert "<interfaceWhiteList>" in common_env
    assert 'export NJRH_FASTLIO_PATCHED_OVERLAY="${NJRH_FASTLIO_PATCHED_OVERLAY:-${PROJECT_ROOT}/.runtime/fast_lio_overlay/install}"' in common_env
    assert 'export NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY="${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY:-${PROJECT_ROOT}/.runtime/jt128_nav_tools_overlay/install}"' in common_env
    assert "project_overlay_missing()" in common_env
    assert 'if [[ "${common_env_parent_ready}" != "1" ]] &&' in common_env
    assert '{ [[ "${NJRH_COMMON_ENV_SETUP_DONE:-}" != "1" ]] || project_overlay_missing; }; then' in common_env
    assert "export NJRH_COMMON_ENV_SETUP_DONE=1" in common_env
    assert 'source "${NJRH_FASTLIO_PATCHED_OVERLAY}/local_setup.bash"' in common_env
    assert 'source "${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY}/local_setup.bash"' in common_env
    assert 'source "${PROJECT_ROOT}/install/local_setup.bash"' in common_env


def test_project_runtime_helpers_are_wired():
    local_perception = (ROOT / "src" / "robot_local_perception" / "config" / "local_perception.yaml").read_text(encoding="utf-8")
    robot_safety = (ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml").read_text(encoding="utf-8")
    nav2_yaml = (ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml").read_text(encoding="utf-8")
    overlay_nav = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_nav2_navigation.sh").read_text(encoding="utf-8")
    standard_navigation_launch = (
        ROOT / "src" / "robot_bringup" / "launch" / "standard_navigation.launch.py"
    ).read_text(encoding="utf-8")
    floor_manager_code = (ROOT / "src" / "robot_floor_manager" / "src" / "floor_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    overlay_tf_helpers = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "canonical_tf_helpers.sh").read_text(encoding="utf-8")
    overlay_nav_helpers = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "nav_runtime_helpers.sh").read_text(encoding="utf-8")
    overlay_mapping = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_fastlio_tf.sh").read_text(encoding="utf-8")
    overlay_localization = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    overlay_local_costmap_debug = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_costmap_debug.sh").read_text(encoding="utf-8")
    overlay_floor_helpers = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "floor_asset_helpers.sh").read_text(encoding="utf-8")
    overlay_floor_manager = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_floor_manager.sh").read_text(encoding="utf-8")
    overlay_promote_floor = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "promote_map_to_floor.sh").read_text(encoding="utf-8")
    overlay_localizer_prepare = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "prepare_localizer_map.py").read_text(encoding="utf-8")
    overlay_localization_launch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization.launch.py").read_text(encoding="utf-8")
    overlay_local_perception = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_perception.sh").read_text(encoding="utf-8")
    overlay_robot_safety = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_safety.sh").read_text(encoding="utf-8")
    overlay_common_services = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh").read_text(encoding="utf-8")
    resident_runtime = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    overlay_nav_runtime_helpers = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "nav_runtime_helpers.sh").read_text(encoding="utf-8")
    overlay_stop_navigation = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "stop_floor_navigation.sh").read_text(encoding="utf-8")
    overlay_canonical_helpers = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "canonical_tf_helpers.sh").read_text(encoding="utf-8")
    overlay_driver = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_driver.sh").read_text(encoding="utf-8")
    overlay_robot_description = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_description.sh").read_text(encoding="utf-8")
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    dashboard_patch_v2 = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime_v2.py").read_text(encoding="utf-8")
    lifecycle_doc = (ROOT / "docs" / "runtime_service_lifecycle.md").read_text(encoding="utf-8")
    overlay_can_up = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "bringup_ranger_can.sh").read_text(encoding="utf-8")
    overlay_can_down = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "shutdown_ranger_can.sh").read_text(encoding="utf-8")
    container_script = (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(encoding="utf-8")
    autostart_runner = (ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh").read_text(encoding="utf-8")
    autostart_cleanup = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "stop_runtime_processes.sh"
    ).read_text(encoding="utf-8")
    autostart_installer = (ROOT / "scripts" / "jetson" / "install_njrh_autostart.sh").read_text(encoding="utf-8")
    autostart_doc = (ROOT / "docs" / "autostart_systemd.md").read_text(encoding="utf-8")
    powershell_helper = (ROOT / "scripts" / "jetson" / "Invoke-NJRHJetson.ps1").read_text(encoding="utf-8")
    for token in (
        "run_navigation_runtime_services.sh",
        "nav2_lifecycle_sequence.py",
        "call_global_localization_trigger.py",
        "run_nav2_navigation.sh",
        "run_occupancy_grid_localization.sh",
        "standard_navigation.launch.py",
        "robot_docking_manager/docking_manager_node",
        "docking_manager_node --ros-args",
        "occupancy_grid_localizer_container",
        "robot_localization_bridge/localization_bridge_node",
        "amcl --ros-args",
        "amcl_scan_admission",
        "__node:=controller_server",
        "__node:=planner_server",
        "__node:=bt_navigator",
    ):
        assert token in (autostart_runner + autostart_cleanup)
    assert (
        "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION="
        "${NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-false}"
    ) in autostart_runner
    assert "kill_by_pattern()" not in autostart_runner
    assert "kill -9" not in autostart_runner
    assert "stop_exact_process_set" in autostart_cleanup
    assert "killing exact stale" in autostart_cleanup
    assert "stale ros2 diagnostics cli" in autostart_cleanup
    assert "ros2_cli_pattern" in autostart_cleanup
    assert 'ros2_cli_pattern="/' in autostart_cleanup
    assert '$0 !~ /stop_runtime_processes[.]sh/' in autostart_cleanup
    assert "service call /amcl/(change_state|get_state)" in autostart_cleanup
    assert "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=false" in autostart_installer
    assert (
        "NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START="
        "${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}"
    ) in autostart_runner
    assert (
        "NJRH_NAV2_LIFECYCLE_PARALLEL_CORE="
        "${NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false}"
    ) in autostart_runner
    assert (
        "NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK="
        "${NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false}"
    ) in autostart_runner
    assert "NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=false" in autostart_installer
    assert "NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE:-once" in autostart_runner
    assert "runtime permission preparation already complete marker=" in autostart_runner
    assert "NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=once" in autostart_installer
    assert (
        "NJRH_COMMON_LOCAL_STATE_START_READY_MODE="
        "${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}"
    ) in autostart_runner
    assert (
        "NJRH_COMMON_LOCAL_STATE_BACKGROUND_START="
        "${NJRH_COMMON_LOCAL_STATE_BACKGROUND_START:-true}"
    ) in autostart_runner
    assert (
        "NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE="
        "${NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE:-false}"
    ) in autostart_runner
    assert "NJRH_COMMON_LOCAL_STATE_START_READY_MODE=endpoint" in autostart_installer
    assert "NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=true" in autostart_installer
    assert "NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false" in autostart_installer
    assert "NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE=true" in autostart_installer
    assert "NJRH_LOCAL_STATE_START_READY_MODE=\"${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}\"" in overlay_common_services
    assert "NJRH_COMMON_LOCAL_STATE_BACKGROUND_START:-true" in overlay_common_services
    assert '[[ "${NAV_LOCAL_STATE_MODE}" != "fastlio" ]] || return 1' in overlay_common_services
    assert "wait_for_runtime_health_local_state_endpoint_ready" in overlay_common_services
    assert "NJRH_RUNTIME_HEALTH_LOCAL_STATE_ENDPOINT_TIMEOUT_SEC:-0" in overlay_common_services
    assert "NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE:-false" in overlay_common_services
    assert "common local_state ready from runtime health snapshot before navigation startup" in resident_runtime
    assert "wait_for_fresh_tf_transform \"odom\" \"base_link\"" in resident_runtime
    assert (
        "NJRH_NAV2_LIFECYCLE_PARALLEL_BT="
        "${NJRH_NAV2_LIFECYCLE_PARALLEL_BT:-true}"
    ) in autostart_runner
    assert (
        "NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST="
        "${NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false}"
    ) in autostart_runner
    assert (
        "NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC="
        "${NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC:-15}"
    ) in autostart_runner
    assert "/tmp/njrh_runtime_map_context.json" in autostart_runner
    assert "/tmp/njrh_amcl_runtime_status.env" in autostart_runner
    assert "/tmp/njrh_nav2_launch_hold_ready.env" in autostart_runner
    assert "mode_topic" in local_perception
    assert "enabled: false" in local_perception
    assert 'input_topic: ""' in local_perception
    assert 'output_topic: ""' in local_perception
    assert 'clearing_output_topic: ""' in local_perception
    assert 'status_topic: ""' in local_perception
    assert "input_topic: /lidar_points\n" not in local_perception
    assert "input_topic: /_internal/lidar_points_local" not in local_perception
    assert "input_topic: /jt128/vendor/points_raw" not in local_perception
    assert "input_reliable: false" in local_perception
    assert "input_qos_depth: 1" in local_perception
    assert "input_frame_id_override: lidar_link" in local_perception
    assert "input_transform_use_latest: true" in local_perception
    assert "input_rotation_matrix:" in local_perception
    assert "/perception/obstacle_points" not in local_perception
    assert "/perception/clearing_points" not in local_perception
    assert "/perception/local_perception_status" not in local_perception
    assert "processing_rate_hz: 15.0" in local_perception
    assert "process_on_callback: false" in local_perception
    assert "point_sample_stride: 1" in local_perception
    assert "restamp_to_now: true" in local_perception
    assert "restamp_to_latest_tf: true" in local_perception
    assert "require_output_stamp_tf: false" in local_perception
    assert "output_stamp_tf_target_frame: odom" in local_perception
    assert "max_output_tf_stamp_age_sec: 0.25" in local_perception
    assert "output_stamp_tf_backoff_sec: 0.0" in local_perception
    assert "output_stamp_forward_sec: 0.0" in local_perception
    assert "require_startup_tf_ready: true" in local_perception
    assert "startup_tf_warmup_sec: 1.0" in local_perception
    assert "clearing.enabled: false" in local_perception
    assert "clearing.virtual_rays.enabled: true" in local_perception
    assert "clearing.virtual_rays.angular_resolution_deg: 1.0" in local_perception
    assert "clearing.virtual_rays.range: 8.00" in local_perception
    assert "clearing.virtual_rays.range_steps: [0.50, 1.00, 2.00, 3.50, 5.50, 8.00]" in local_perception
    assert "clearing.max_points: 15000" in local_perception
    assert "status_publish_period_sec: 2.0" in local_perception
    assert "-0.10, 0.05, 0.20, 0.40, 0.60, 0.85, 1.10, 1.30" in local_perception
    assert "profiles.NORMAL.range_filter.max: 5.50" in local_perception
    assert "profiles.NORMAL.height_filter.min_z: 0.40" in local_perception
    assert "profiles.NORMAL.height_filter.max_z: 1.30" in local_perception
    assert "profiles.NORMAL.outlier_filter.enabled: true" in local_perception
    assert "profiles.ELEVATOR_WAIT.range_filter.max: 8.0" in local_perception
    assert "profiles.DOORWAY.range_filter.max: 12.0" in local_perception
    assert "require_localization_health" in robot_safety
    assert "status_topic: /safety/status" in robot_safety
    assert "motion_allowed_topic: /safety/motion_allowed" in robot_safety
    assert "cmd_vel_out_topic: /cmd_vel" in robot_safety
    assert "cmd_vel_mirror_topic: /cmd_vel_safe" in robot_safety
    assert "local_perception helper disabled" in overlay_nav
    assert "run_floor_manager.sh" in overlay_nav
    assert "run_robot_safety.sh" in overlay_nav
    assert "run_ranger_mini3_mode_controller.sh" not in overlay_nav
    assert "ensure_resident_overlay_helper_process()" in overlay_nav
    assert 'ensure_resident_overlay_helper_process "robot_safety" "robot_safety"' in overlay_nav
    assert "ranger_mini3_mode_controller" not in overlay_nav
    assert "require_resident_overlay_helper()" not in overlay_nav
    assert "canonical_tf_helpers.sh" in overlay_nav
    assert "ensure_canonical_local_state_for_nav2()" not in overlay_nav
    assert "canonical local_state endpoints are already ready for Nav2" not in overlay_nav
    assert "waiting for resident common services to recover" not in overlay_nav
    assert "canonical local_state recovered without restart" not in overlay_nav
    assert "resident canonical local_state stayed alive but TF freshness did not recover" not in overlay_nav
    assert "restart_canonical_local_state_for_nav2()" not in overlay_nav
    assert "repairing canonical local_state for Nav2" not in overlay_nav
    assert "resident canonical local_state is not stable; refusing to start Nav2" not in overlay_nav
    assert "canonical_local_state_fresh_for_nav2()" not in overlay_nav
    assert "wait_for_local_costmap_observation_ready_with_local_state_check()" not in overlay_nav
    assert "NJRH_NAV_LOCAL_COSTMAP_LOCAL_STATE_CHECK_ATTEMPTS" not in overlay_nav
    assert "NJRH_NAV_LOCAL_COSTMAP_LOCAL_STATE_REPAIR_ATTEMPTS" not in overlay_nav
    assert "refusing to restart odom under local costmap" not in overlay_nav
    assert "canonical local_state is stale during active Nav2 startup" not in overlay_nav
    assert "pause_local_perception_for_nav2_costmap_warmup()" not in overlay_nav
    assert "prime_local_costmap_tf_buffer_before_observations()" not in overlay_nav
    assert "preserving source pointcloud stamps" not in overlay_nav
    assert "NJRH_NAV_LOCAL_COSTMAP_TF_BUFFER_PRIME_SEC:-3.0" not in overlay_nav
    assert "local_costmap_costmap_no_update_before_observation_enable" not in overlay_nav
    assert "odom_base_tf_not_fresh_before_observation_enable" not in overlay_nav
    assert 'wait_for_topic_message "/perception/obstacle_points"' not in overlay_nav
    assert 'start_canonical_helper \\' not in overlay_nav
    assert '"robot_local_state_navigation"' not in overlay_nav
    assert 'env LOCAL_STATE_MODE="${mode}" bash "${SCRIPT_DIR}/run_local_state.sh"' not in overlay_nav
    assert "/local_state/odometry or odom->base_link is not ready from resident common services" not in overlay_nav
    assert "blocking readiness probes are disabled" in overlay_nav
    assert "robot_local_perception PointCloud2 obstacle pipeline has been removed" in overlay_local_perception
    assert "Nav2 local marking+clearing now uses /scan through nav2_costmap_2d::ObstacleLayer" in overlay_local_perception
    assert "install/robot_local_perception/lib/robot_local_perception/local_perception_node" not in overlay_local_perception
    assert 'wait_for_topic_message "/local_state/odometry"' not in overlay_local_perception
    assert 'wait_for_tf_transform "base_link" "lidar_link"' not in overlay_local_perception
    assert 'wait_for_fresh_tf_transform \\' not in overlay_local_perception
    assert "local perception TF prerequisites ready" not in overlay_local_perception
    assert "starting local perception without startup topic/TF probes" not in overlay_local_perception
    assert "src/robot_local_perception/scripts/local_perception_node.py" not in overlay_nav_helpers
    assert "python3 .*local_perception_node.py" not in overlay_nav_helpers
    assert "robot_local_perception/local_perception_node" in overlay_nav_helpers
    assert "local_perception_runtime_config_ready()" not in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception' not in overlay_nav_helpers
    assert "disabled: production local obstacle marking+clearing uses /scan" in overlay_nav_helpers
    assert "robot_floor_manager/floor_manager_node" in overlay_nav_helpers
    assert "install/robot_safety/lib/robot_safety/robot_safety_node" in overlay_robot_safety
    assert "Python fallback has been removed" in overlay_robot_safety
    assert "ranger_mini3_mode_controller" not in overlay_nav_helpers
    assert "require_can_interface_up" in overlay_tf_helpers
    assert 'if [[ -e "${helper_log}" && ! -w "${helper_log}" ]]; then' in overlay_tf_helpers
    assert 'rm -f "${helper_log}"' in overlay_tf_helpers
    assert ': >"${helper_log}"' in overlay_tf_helpers
    assert "require_can_interface_up" in overlay_mapping
    assert "require_can_interface_up" in overlay_localization
    assert "localizer_map_yaml" in overlay_localization
    assert "NAV2_LOCALIZER_MAP_YAML" in overlay_localization
    assert "resolve_floor_assets" in overlay_localization
    assert "resolve_floor_assets" in overlay_nav
    assert "validate_floor_assets" in overlay_floor_helpers
    assert "/current" in overlay_floor_helpers
    assert "selected floor asset root" in overlay_floor_helpers
    assert "NAV2_MAP_YAML" in overlay_floor_helpers
    assert "NAV2_LOCALIZER_MAP_YAML" in overlay_floor_helpers
    assert "NAV2_KEEP_OUT_MASK_YAML" in overlay_floor_helpers
    assert "NAV2_SPEED_MASK_YAML" in overlay_floor_helpers
    assert "ensure_costmap_filter_masks.py" in overlay_nav
    assert "runtime_nav2" in overlay_nav
    assert "source_keepout" in overlay_nav
    assert "--keepout-yaml" in overlay_nav
    assert "--stable-wait-sec" in overlay_nav
    assert '[[ "${NAV2_KEEP_OUT_MASK_YAML}" == "${runtime_dir}/"* ]]' in overlay_nav
    assert "costmap_filter_mask_is_neutral()" in overlay_nav
    assert "disable_neutral_costmap_filters_if_needed()" in overlay_nav
    assert "NJRH_NAV2_DISABLE_NEUTRAL_COSTMAP_FILTERS:-false" in overlay_nav
    assert "nav2.neutral_keepout_disabled.yaml" in overlay_nav
    assert 'disabled_filter_manager = False' in overlay_nav
    assert 'removed_global_keepout_filter_list = False' in overlay_nav
    assert 'disabled_keepout_plugin = False' in overlay_nav
    assert "lifecycle_manager_costmap_filters autostart" in overlay_nav
    assert 'global_costmap filters: ["keepout_filter"]' in overlay_nav
    assert "global costmap keepout filter removed and filter lifecycle autostart disabled" in overlay_nav
    assert "NJR H_NAV2_COSTMAP_FILTER_SERVERS_ENABLED" not in overlay_nav
    assert "NJR H_NAV2_COSTMAP_FILTER_SERVERS_ENABLED" not in standard_navigation_launch
    assert "NJRH_NAV2_COSTMAP_FILTER_SERVERS_ENABLED=false" in overlay_nav
    assert "neutral keepout filter mask detected" in overlay_nav
    assert 'export NAV2_PARAMS_FILE="${runtime_params}"' in overlay_nav
    assert 'keepout_mask_yaml:="${NAV2_KEEP_OUT_MASK_YAML}"' in overlay_nav
    assert 'speed_mask_yaml:="${NAV2_SPEED_MASK_YAML}"' in overlay_nav
    assert "nav2_speed_filter_enabled()" in overlay_nav
    assert "Nav2 speed filter enabled=" in overlay_nav
    assert "costmap_filter_info_server" in standard_navigation_launch
    assert "cpu_affinity_prefix" in standard_navigation_launch
    assert "TimerAction" in standard_navigation_launch
    assert 'DeclareLaunchArgument(\n                "nav_lifecycle_start_delay"' in standard_navigation_launch
    assert 'DeclareLaunchArgument(\n                "navigation_lifecycle_autostart"' in standard_navigation_launch
    assert 'NJRH_NAVIGATION_LIFECYCLE_AUTOSTART", "true"' in standard_navigation_launch
    assert 'NJRH_NAV_LIFECYCLE_START_DELAY_SEC", "2.0"' in standard_navigation_launch
    assert 'export NJRH_NAV_LIFECYCLE_START_DELAY_SEC="${NJRH_NAV_LIFECYCLE_START_DELAY_SEC:-2.0}"' in overlay_nav
    assert 'log_level:="${NJRH_NAV2_LOG_LEVEL:-warn}"' in overlay_nav
    assert 'NJRH_COSTMAP_FILTER_MASK_STABLE_WAIT_SEC:-0.5' in overlay_nav
    assert 'NJRH_NAV2_LAUNCH_SETTLE_SEC:-1' in overlay_nav
    assert 'enable_costmap_filter_servers = os.environ.get(' in standard_navigation_launch
    assert '"NJRH_NAV2_COSTMAP_FILTER_SERVERS_ENABLED", "true"' in standard_navigation_launch
    assert "costmap_filter_nodes = []" in standard_navigation_launch
    assert "if enable_costmap_filter_servers:" in standard_navigation_launch
    assert "filter_lifecycle_manager_nodes = []" in standard_navigation_launch
    assert "if filter_lifecycle_nodes:" in standard_navigation_launch
    assert "*costmap_filter_nodes" in standard_navigation_launch
    assert "*filter_lifecycle_manager_nodes" in standard_navigation_launch
    assert "filter_lifecycle_nodes = []" in standard_navigation_launch
    assert "navigation_lifecycle_nodes = [" in standard_navigation_launch
    assert 'name="lifecycle_manager_costmap_filters"' in standard_navigation_launch
    assert 'name="lifecycle_manager_navigation"' in standard_navigation_launch
    assert '{"node_names": filter_lifecycle_nodes}' in standard_navigation_launch
    assert '{"node_names": navigation_lifecycle_nodes}' in standard_navigation_launch
    assert '{"autostart": navigation_lifecycle_autostart}' in standard_navigation_launch
    assert '{"bond_timeout": 0.0}' in standard_navigation_launch
    assert 'navigation_lifecycle_autostart:="${navigation_lifecycle_autostart}"' in overlay_nav
    assert "NJRH_NAV2_EXTERNAL_LIFECYCLE_BRINGUP:-true" in overlay_nav
    assert "NJRH_NAV2_LIFECYCLE_HOLD:-false" in overlay_nav
    assert "resident runtime is holding lifecycle activation" in overlay_nav
    assert "Nav2 launch process is running with lifecycle activation held" in overlay_nav
    assert "/opt/ros/humble/lib/nav2_util/lifecycle_bringup" in overlay_nav
    assert "nav_lifecycle_bringup_pid" in overlay_nav
    overlay_nav_lifecycle_nodes = overlay_nav[
        overlay_nav.index("start_navigation_lifecycle_with_nav2_util()") :
        overlay_nav.index(
            'echo "[runtime-overlay] starting Nav2 core lifecycle',
            overlay_nav.index("start_navigation_lifecycle_with_nav2_util()"),
        )
    ]
    assert overlay_nav_lifecycle_nodes.index("planner_server") < overlay_nav_lifecycle_nodes.index(
        "controller_server"
    )
    assert 'kill -TERM "${nav_lifecycle_bringup_pid}"' in overlay_nav
    assert "NJRH_NAV2_LIFECYCLE_BRINGUP_TIMEOUT_SEC:-180" in overlay_nav
    assert "Nav2 core lifecycle autostart disabled; lifecycle_bringup will manage core nodes" in overlay_nav
    assert "lifecycle_manager_navigation external lifecycle_bringup: Managed nodes are active" in overlay_nav
    assert 'with_cpu_affinity("controller_server", node_kwargs)' in standard_navigation_launch
    assert 'with_cpu_affinity("smoother_server", node_kwargs)' in standard_navigation_launch
    assert 'with_cpu_affinity("velocity_smoother", node_kwargs)' in standard_navigation_launch
    assert 'with_cpu_affinity("collision_monitor", node_kwargs)' in standard_navigation_launch
    assert standard_navigation_launch.index('"smoother_server",') < standard_navigation_launch.index('"bt_navigator",')
    assert standard_navigation_launch.index('"velocity_smoother",') < standard_navigation_launch.index('"bt_navigator",')
    assert standard_navigation_launch.index('"collision_monitor",') < standard_navigation_launch.index('"bt_navigator",')
    assert nav2_yaml.index("- smoother_server") < nav2_yaml.index("- bt_navigator")
    assert nav2_yaml.index("- velocity_smoother") < nav2_yaml.index("- bt_navigator")
    assert nav2_yaml.index("- collision_monitor") < nav2_yaml.index("- bt_navigator")
    assert "keepout_filter_mask_server" in standard_navigation_launch
    assert "speed_filter_mask_server" in standard_navigation_launch
    assert 'enable_speed_filter = os.environ.get("NJRH_ENABLE_SPEED_FILTER", "false")' in standard_navigation_launch
    assert "speed_filter_nodes = []" in standard_navigation_launch
    assert '"/costmap_filter_info/keepout"' in standard_navigation_launch
    assert '"/costmap_filter_info/speed"' in standard_navigation_launch
    assert '"/keepout_filter_mask"' in standard_navigation_launch
    assert '"/speed_filter_mask"' in standard_navigation_launch
    assert "keepout_mask_load_service" in floor_manager_code
    assert "speed_mask_load_service" in floor_manager_code
    assert "lifecycle_state_service_for_load_map" in floor_manager_code
    assert "PRIMARY_STATE_ACTIVE" in floor_manager_code
    assert "deferring filter mask reload to Nav2 startup" in floor_manager_code
    assert "load_filter_masks" in floor_manager_code
    assert 'floor_root / "current"' in floor_manager_code
    assert "install/robot_floor_manager/lib/robot_floor_manager/floor_manager_node" in overlay_floor_manager
    assert (
        "colcon build --packages-select robot_map_asset_identity "
        "robot_interfaces robot_floor_manager"
    ) in overlay_floor_manager
    assert "--flat-map-name" in overlay_promote_floor
    assert "robot_map_toolkit/scripts/map_toolkit_cli.py" in overlay_promote_floor
    assert "local_costmap_debug.launch.py" in overlay_local_costmap_debug
    assert 'source "${SCRIPT_DIR}/cpu_affinity.sh"' in overlay_local_costmap_debug
    assert "run_local_perception.sh" not in overlay_local_costmap_debug
    assert "local_perception debug helper disabled" in overlay_local_costmap_debug
    assert "run_robot_description.sh" in overlay_local_costmap_debug
    assert "run_local_state.sh" in overlay_local_costmap_debug
    assert "run_robot_safety.sh" not in overlay_local_costmap_debug
    assert "ensure_png" in overlay_localizer_prepare
    assert "prepare_localizer_assets" in overlay_localizer_prepare
    assert '.localizer.png' in overlay_localizer_prepare
    assert '.localizer.yaml' in overlay_localizer_prepare
    assert "localizer_map_yaml" in overlay_localization_launch
    assert "CAN_BITRATE" in overlay_can_up
    assert 'link set "${CAN_IFACE}" up type can bitrate "${CAN_BITRATE}"' in overlay_can_up
    assert 'link set "${CAN_IFACE}" down' in overlay_can_down
    assert "NJRH_REUSE_COMMON_SERVICES:-true" in overlay_nav_runtime_helpers
    assert "NJRH_FORCE_RESTART_NAV_HELPERS:-false" in overlay_nav_runtime_helpers
    assert "reusing existing ${helper_name}" in overlay_nav_runtime_helpers
    assert "stop_existing_standard_nav_stack()" in overlay_nav_runtime_helpers
    assert "NJRH_STANDARD_NAV_STACK_STOP_INT_WAIT_SEC:-0.5" in overlay_nav_runtime_helpers
    assert "NJRH_OVERLAY_HELPER_START_SETTLE_SEC:-0.2" in overlay_nav_runtime_helpers
    assert "NJRH_OVERLAY_HELPER_STOP_INT_WAIT_SEC:-0.5" in overlay_nav_runtime_helpers
    assert '"laser_scan_to_flatscan"' not in overlay_stop_navigation
    assert '"laser_scan_to_flatscan"' not in overlay_nav_runtime_helpers

    assert "__node:=lifecycle_manager_costmap_filters" in overlay_nav_runtime_helpers
    assert "__node:=lifecycle_manager_navigation" in overlay_nav_runtime_helpers
    assert "/opt/ros/humble/lib/nav2_util/lifecycle_bringup" in overlay_nav_runtime_helpers
    assert '"lifecycle_bringup"' in overlay_stop_navigation
    assert "__node:=collision_monitor" in overlay_nav_runtime_helpers
    assert "NJRH_FORCE_RESTART_CANONICAL_TF:-false" in overlay_canonical_helpers
    assert "reusing existing ${helper_name}" in overlay_canonical_helpers
    assert "NJRH_FORCE_RESTART_DRIVER:-false" in overlay_driver
    assert "canonical JT128 driver/remap chain already running; reusing existing ingress" in overlay_driver
    assert "canonical_jt128_ingress_running()" in overlay_common_services
    assert "canonical driver/remap chain is complete" in overlay_common_services
    assert "__njrh_force_start_jt128_driver_chain__" in overlay_common_services
    assert "robot_description_static_tf_node already running; reusing existing static TF publisher" in overlay_robot_description
    assert "run_driver.sh" in overlay_common_services
    assert "run_ranger_chassis.sh" in overlay_common_services
    assert "run_local_state.sh" in overlay_common_services
    assert "NJRH_NAV_LOCAL_STATE_MODE:-ekf" in overlay_common_services
    assert "NJRH_FASTLIO_AUTOSTART:-false" in overlay_common_services
    assert 'FASTLIO_CONFIG_FILE="${NJRH_FASTLIO_CONFIG_FILE:-${NJRH_OVERLAY_ROOT}/config/fastlio.yaml}"' in overlay_common_services
    assert "start_fastlio_common()" in overlay_common_services
    assert "ros2 run fast_lio fastlio_mapping" in overlay_common_services
    assert "-r /tf:=/tf_fastlio_internal" in overlay_common_services
    assert "fastlio_runtime_topics_fresh()" not in overlay_common_services
    assert "wait_for_fresh_header_topic_message" not in overlay_common_services
    assert "existing fastlio_mapping process is stale" not in overlay_common_services
    assert "FAST-LIO odometry did not become fresh" not in overlay_common_services
    assert "fastlio_runtime_output_fresh()" in overlay_common_services
    assert "wait_for_fastlio_runtime_output()" in overlay_common_services
    assert "stop_non_mapping_fastlio_runtime_processes()" in overlay_common_services
    assert "fastlio_pid_is_mapping_owned()" in overlay_common_services
    assert "FAST-LIO2 common autostart disabled; stopping non-mapping FAST-LIO leftovers" in overlay_common_services
    assert "fresh-header-topic" in overlay_common_services
    assert "existing fastlio_mapping process has stale/missing ${FASTLIO_ODOM_TOPIC}" in overlay_common_services
    assert "FAST-LIO failed to publish fresh ${FASTLIO_ODOM_TOPIC}" in overlay_common_services
    assert "FAST-LIO2 common autostart disabled; mapping starts FAST-LIO2 only while mapping is active" in overlay_common_services
    assert "startup readiness probes are disabled" in overlay_common_services
    assert 'env NJRH_LOCAL_STATE_START_READY_MODE="${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}"' in overlay_common_services
    assert 'LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}"' in overlay_common_services
    assert 'bash "${SCRIPT_DIR}/run_local_state.sh"' in overlay_common_services
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' in overlay_common_services
    assert "run_local_perception.sh" not in overlay_common_services
    assert "local_perception_common disabled" in overlay_common_services
    assert "run_robot_safety.sh" in overlay_common_services
    assert "run_robot_api_server.sh" in overlay_common_services
    assert "run_gs2_driver.sh" in overlay_common_services
    assert "NJRH_GS2_AUTOSTART" in overlay_common_services
    assert "robot_eai_gs2/gs2_driver_node" in overlay_common_services
    assert "NJRH_DOCKING_MANAGER_AUTOSTART:-true" in overlay_common_services
    assert "run_docking_manager.sh" in overlay_common_services
    assert "robot_docking_manager/docking_manager_node" in overlay_common_services
    assert "LAST_NAVIGATION_MAP_FILE" in overlay_common_services
    assert "load_last_navigation_map_selection()" in overlay_common_services
    assert "NJRH_RESIDENT_NAVIGATION_AUTOSTART:-auto" in overlay_common_services
    assert "resident_navigation_context_status()" in overlay_common_services
    assert "wait_for_resident_navigation_context_ready()" in overlay_common_services
    assert "context ready but API goal-start not safe" in overlay_common_services
    assert "resident_navigation_runtime_process_running()" in overlay_common_services
    assert "resident_navigation_runtime_pids()" in overlay_common_services
    assert "stop_stale_resident_navigation_runtime_processes()" in overlay_common_services
    assert "cleanup_resident_navigation_runtime_layers()" in overlay_common_services
    assert "prepare_resident_navigation_autostart()" in overlay_common_services
    assert "clearing stale resident navigation runtime before autostart" in overlay_common_services
    assert "/run_navigation_runtime_services.sh/ && !/awk/" in overlay_common_services
    assert "stopping stale resident navigation runtime processes" in overlay_common_services
    assert "stale resident navigation runtime ignored SIGTERM; killing exact pids" in overlay_common_services
    assert "kill -KILL ${pids}" in overlay_common_services
    assert "stop_stale_resident_navigation_runtime_processes" in overlay_common_services[
        overlay_common_services.index("cleanup_resident_navigation_runtime_layers()") :
        overlay_common_services.index('bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --stop')
    ]
    assert "NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC:-0.2" in overlay_common_services
    assert "NJRH_AMCL_SEED_HELPER_STOP_INT_WAIT_SEC:-0.2" in overlay_common_services
    assert "NJRH_AMCL_SEED_HELPER_STOP_TERM_WAIT_SEC:-0.2" in overlay_common_services
    assert "NJRH_COMMON_AMCL_STOP_TIMEOUT_SEC:-1.5" in overlay_common_services
    assert "NJRH_COMMON_AMCL_STOP_KILL_AFTER_SEC:-1" in overlay_common_services
    assert 'bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --stop' in overlay_common_services
    assert "NJRH_STANDARD_NAV_STACK_STOP_INT_WAIT_SEC=\"${NJRH_STANDARD_NAV_STACK_STOP_INT_WAIT_SEC:-0.2}\"" in overlay_common_services
    assert "NJRH_LOCALIZATION_STACK_STOP_WAIT_SEC=\"${NJRH_LOCALIZATION_STACK_STOP_WAIT_SEC:-0.2}\"" in overlay_common_services
    assert "stop_existing_standard_nav_stack || true" in overlay_common_services
    assert "stop_existing_localization_stack || true" in overlay_common_services
    assert "rm -f /tmp/njrh_runtime_map_context.json /tmp/njrh_amcl_runtime_status.env" in overlay_common_services
    cleanup_block = overlay_common_services[
        overlay_common_services.index("cleanup() {") :
        overlay_common_services.index("on_signal()", overlay_common_services.index("cleanup() {"))
    ]
    assert cleanup_block.count("cleanup_resident_navigation_runtime_layers") >= 2
    assert "NJRH_RESIDENT_NAVIGATION_READY_TIMEOUT_SEC:-120" in overlay_common_services
    assert "NJRH_RESIDENT_NAVIGATION_READY_HARD_TIMEOUT_SEC:-240" in overlay_common_services
    assert "exceeded soft SLA" in overlay_common_services
    assert "hard timeout" in overlay_common_services
    assert "resident navigation context ready" in overlay_common_services
    assert "resident navigation context did not become ready" in overlay_common_services
    resident_autostart_wait_block = overlay_common_services[
        overlay_common_services.index("wait_for_resident_navigation_autostart_if_started()") :
        overlay_common_services.index("stop_stale_pointcloud_accel_pipeline_processes()")
    ]
    assert "common services will not block on navigation readiness" in resident_autostart_wait_block
    assert "wait_for_resident_navigation_context_ready" not in resident_autostart_wait_block
    assert "return 0" in resident_autostart_wait_block
    assert "no valid last navigation map; common services stay alive in NO_MAP mode" in overlay_common_services
    assert "last_navigation_map.json" in overlay_common_services
    resident_navigation_runtime = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    nav2_ready_block = resident_navigation_runtime[
        resident_navigation_runtime.index("log_startup_stage \"nav2_layer_ready\"") :
        resident_navigation_runtime.index("log_startup_stage \"amcl_tracking_ready\"")
    ]
    assert "wait_for_amcl_readiness_background_if_running" in nav2_ready_block
    assert "AMCL readiness background did not complete cleanly" in nav2_ready_block
    assert "complete_amcl_readiness_with_retries_for_navigation" in nav2_ready_block
    assert nav2_ready_block.index("wait_for_amcl_readiness_background_if_running") < nav2_ready_block.index(
        "complete_amcl_readiness_with_retries_for_navigation"
    )
    assert overlay_common_services.index("prepare_resident_navigation_autostart") < overlay_common_services.index(
        'start_common_process "resident_navigation_runtime"'
    )
    assert overlay_common_services.index("wait_for_resident_navigation_autostart_if_started") < overlay_common_services.index(
        "common services are running; start mapping or resident navigation scripts in reuse mode"
    )
    overlay_gs2_driver = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_gs2_driver.sh"
    ).read_text(encoding="utf-8")
    assert "/dev/gs2 missing inside container" in overlay_gs2_driver
    assert "/dev/ttyUSB0" in overlay_gs2_driver
    assert "/dev/ttyUSB4" not in overlay_gs2_driver
    assert "needs_restart = False" in dashboard_patch
    assert "common driver/chassis services kept running" in dashboard_patch
    assert "floor manager stopCore kill patterns" not in dashboard_patch_v2
    assert "Common services should stay up during daily operation" in lifecycle_doc
    assert "Force restart is explicit" in lifecycle_doc
    assert "start-common)" in container_script
    assert "start-debug-runtime)" in container_script
    assert 'prepare_release_asset_permissions' in container_script
    assert 'prepare_runtime_overlay_permissions' in container_script
    assert "'${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs'" in container_script
    assert "'${DASHBOARD_RUNTIME_ROOT}/maps'" in container_script
    assert '"${WORKSPACE_HOST}/maps_release"' in container_script
    assert 'RUNTIME_USER="${NJRH_RUNTIME_USER:-root}"' in container_script
    assert '--user "${RUNTIME_USER}"' in container_script
    assert '"USER=${RUNTIME_USER}"' in container_script
    assert '"HOME=${RUNTIME_HOME}"' in container_script
    assert "chown -R '${RUNTIME_USER}:${RUNTIME_GROUP}'" in container_script
    assert "chmod 2775" in container_script
    assert "chmod 664" in container_script
    assert 'ROBOT_API_READY_TIMEOUT_SEC="${NJRH_ROBOT_API_READY_TIMEOUT_SEC:-120}"' in container_script
    assert 'ROBOT_API_READY_POLL_SEC="${NJRH_ROBOT_API_READY_POLL_SEC:-1}"' in container_script
    assert 'ROBOT_NAV_READY_TIMEOUT_SEC="${NJRH_ROBOT_NAV_READY_TIMEOUT_SEC:-120}"' in container_script
    assert 'ROBOT_NAV_READY_POLL_SEC="${NJRH_ROBOT_NAV_READY_POLL_SEC:-1}"' in container_script
    assert 'deadline=$((SECONDS + ROBOT_API_READY_TIMEOUT_SEC))' in container_script
    assert "resident_navigation_context_status()" in container_script
    assert "wait_for_resident_navigation_runtime()" in container_script
    assert 'state == "ready" and confirmed' in container_script
    assert 'startup_elapsed_sec=' in container_script
    assert "stop_detached_runtime_processes()" in container_script
    assert "run_navigation_runtime_services.sh" in container_script
    assert "standard_navigation.launch.py" in container_script
    assert "robot_api_server/robot_api_server_node" in container_script
    assert "/tmp/njrh_runtime_map_context.json" in container_script
    assert "pkill -9" not in container_script
    assert "seq 1 30" not in container_script
    start_common_services_block = container_script[
        container_script.index("start_common_services()") : container_script.index("stop_detached_runtime_processes()")
    ]
    assert "wait_for_robot_api" in start_common_services_block
    assert "wait_for_resident_navigation_runtime" in start_common_services_block
    assert start_common_services_block.index("wait_for_robot_api") < start_common_services_block.index(
        "wait_for_resident_navigation_runtime"
    )
    stop_common_services_block = container_script[
        container_script.index("stop_common_services()") : container_script.index("wait_for_dashboard()")
    ]
    assert "stop_detached_runtime_processes" in stop_common_services_block
    start_runtime_case = container_script.split("start-runtime)", 1)[1].split(";;", 1)[0]
    assert "start_container" in start_runtime_case
    assert "start_dashboard" not in start_runtime_case
    assert "'start-common'" in powershell_helper
    assert "'start-debug-runtime'" in powershell_helper
    assert "bash scripts/jetson/njrh_container.sh start" in autostart_runner
    assert "bash scripts/jetson/njrh_container.sh start-runtime" not in autostart_runner
    assert "prepare_container_permissions" in autostart_runner
    assert "exec bash scripts/run_common_services.sh" in autostart_runner
    assert "robot_eai_gs2/gs2_driver_node" in autostart_cleanup
    assert "resolve_gs2_serial_port" in autostart_runner
    assert "NJRH_GS2_SERIAL_PORT=${GS2_SERIAL_PORT}" in autostart_runner
    assert "systemctl enable" in autostart_installer
    assert "njrh-runtime.service" in autostart_installer
    assert "ExecStart=" in autostart_installer
    assert "start the Web dashboard" in autostart_doc
    assert "It does not start:" in autostart_doc
    assert "gs2_driver_node" in autostart_doc
    assert "NJRH_GS2_AUTOSTART=false" in autostart_doc
    assert "docking_manager_node" in autostart_doc
    assert "NJRH_DOCKING_MANAGER_AUTOSTART=false" in autostart_doc


def test_runtime_overlay_cpu_affinity_policy_is_wired():
    affinity_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "cpu_affinity.env").read_text(
        encoding="utf-8"
    )
    affinity_helper = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "cpu_affinity.sh").read_text(
        encoding="utf-8"
    )
    apply_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "apply_cpu_affinity.sh"
    ).read_text(encoding="utf-8")
    for service, cpuset in (
        ("NJRH_CPUSET_BASE_CONTROL", "1"),
        ("NJRH_CPUSET_TF_STATE", "2"),
        ("NJRH_CPUSET_NAV_CONTROL", "3"),
        ("NJRH_CPUSET_NAV_PLANNING", "3,5"),
        ("NJRH_CPUSET_NAVIGATION_RUNTIME_OWNER", "0-7"),
        ("NJRH_CPUSET_LIDAR_PERCEPTION", "4,5"),
        ("NJRH_CPUSET_LIDAR_DRIVER", "4,6"),
        ("NJRH_CPUSET_LIDAR_PIPELINE", "5"),
        ("NJRH_CPUSET_LOCALIZATION", "6"),
        ("NJRH_CPUSET_MAPPING_FRONTEND", "6,7"),
        ("NJRH_CPUSET_MAPPING_BACKEND", "7"),
        ("NJRH_CPUSET_ARM_CONTROL", "6"),
        ("NJRH_CPUSET_ARM_PLANNING", "7"),
    ):
        assert f'export {service}="${{{service}:-{cpuset}}}"' in affinity_cfg
    assert "NJRH_CPUSET_ROBOT_SAFETY" in affinity_cfg
    assert "NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE" in affinity_cfg
    assert "NJRH_CPUSET_CONTROLLER_SERVER" in affinity_cfg
    assert 'NJRH_CPUSET_HESAI_ROS_DRIVER="${NJRH_CPUSET_HESAI_ROS_DRIVER:-${NJRH_CPUSET_LIDAR_DRIVER}}"' in affinity_cfg
    assert 'NJRH_CPUSET_POINTCLOUD_AXIS_REMAP="${NJRH_CPUSET_POINTCLOUD_AXIS_REMAP:-${NJRH_CPUSET_LIDAR_PIPELINE}}"' in affinity_cfg
    assert (
        'NJRH_CPUSET_POINTCLOUD_PERCEPTION_PIPELINE="${NJRH_CPUSET_POINTCLOUD_PERCEPTION_PIPELINE:-${NJRH_CPUSET_LIDAR_PIPELINE},${NJRH_CPUSET_LOCALIZATION}}"'
        in affinity_cfg
    )
    assert 'NJRH_CPUSET_POINTCLOUD_DOWNSAMPLE="${NJRH_CPUSET_POINTCLOUD_DOWNSAMPLE:-${NJRH_CPUSET_LIDAR_PIPELINE}}"' in affinity_cfg
    assert 'NJRH_CPUSET_POINTCLOUD_FASTLIO_REMAP="${NJRH_CPUSET_POINTCLOUD_FASTLIO_REMAP:-${NJRH_CPUSET_LIDAR_PIPELINE}}"' in affinity_cfg
    assert 'NJRH_CPUSET_IMU_AXIS_REMAP="${NJRH_CPUSET_IMU_AXIS_REMAP:-${NJRH_CPUSET_LOCALIZATION}}"' in affinity_cfg
    assert 'NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR="${NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR:-${NJRH_CPUSET_LOCALIZATION}}"' in affinity_cfg
    assert 'NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION="${NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION:-6}"' in affinity_cfg
    assert 'NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE="${NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE:-7}"' in affinity_cfg
    assert 'export NJRH_CPUSET_NAV_SUPERVISION="${NJRH_CPUSET_NAV_SUPERVISION:-${NJRH_CPUSET_SYSTEM},${NJRH_CPUSET_BASE_CONTROL}}"' in affinity_cfg
    assert '"${NJRH_CPUSET_NAV2_MAP_SERVER}" == "${NJRH_CPUSET_NAV_PLANNING}"' in affinity_cfg
    assert 'export NJRH_CPUSET_NAV2_MAP_SERVER="${NJRH_CPUSET_SYSTEM}"' in affinity_cfg
    assert '"${NJRH_CPUSET_NAV2_LIFECYCLE_MANAGER}" == "${NJRH_CPUSET_NAV_PLANNING}"' in affinity_cfg
    assert '"${NJRH_CPUSET_NAV2_LIFECYCLE_MANAGER}" == "${NJRH_CPUSET_SYSTEM}"' in affinity_cfg
    assert 'export NJRH_CPUSET_NAV2_LIFECYCLE_MANAGER="${NJRH_CPUSET_NAV_SUPERVISION}"' in affinity_cfg
    assert 'NJRH_CPUSET_FASTLIO_MAPPING="${NJRH_CPUSET_FASTLIO_MAPPING:-${NJRH_CPUSET_MAPPING_BACKEND}}"' in affinity_cfg
    assert 'NJRH_CPUSET_FASTLIO_ODOM_BRIDGE="${NJRH_CPUSET_FASTLIO_ODOM_BRIDGE:-${NJRH_CPUSET_TF_STATE}}"' in affinity_cfg
    assert 'NJRH_CPUSET_FASTLIO_DESKEW="${NJRH_CPUSET_FASTLIO_DESKEW:-${NJRH_CPUSET_MAPPING_FRONTEND}}"' in affinity_cfg
    assert '"${NJRH_CPUSET_SLAM_TOOLBOX_MAPPING}" == "${NJRH_CPUSET_MAPPING_BACKEND}"' in affinity_cfg
    assert 'export NJRH_CPUSET_SLAM_TOOLBOX_MAPPING="3,7"' in affinity_cfg
    assert 'NJRH_CPUSET_PGO_MAPPING="${NJRH_CPUSET_PGO_MAPPING:-${NJRH_CPUSET_MAPPING_BACKEND}}"' in affinity_cfg
    assert "njrh_exec_affined()" in affinity_helper
    assert "njrh_run_affined()" in affinity_helper
    assert "njrh_start_affined_background()" in affinity_helper
    assert "njrh_apply_affinity_to_current_process()" in affinity_helper
    assert "taskset -c" in affinity_helper
    assert "taskset -pc" in affinity_helper
    assert 'for task_path in /proc/"${pid}"/task/*' in affinity_helper
    assert "tasks=${task_count}" in affinity_helper
    assert "failed=${failed_count}" in affinity_helper
    assert 'export NJRH_OVERLAY_ROOT="${NJRH_OVERLAY_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"' in apply_script
    assert 'source "${SCRIPT_DIR}/common_env.sh"' not in apply_script
    assert 'apply_pattern robot_localization_bridge "localization_bridge_node"' in apply_script
    assert 'apply_pattern controller_server "controller_server"' in apply_script
    assert 'apply_pattern fastlio_mapping "fastlio_mapping|laser_mapping"' in apply_script
    assert 'apply_pattern slam_toolbox_mapping "slam_toolbox"' in apply_script
    assert 'apply_pattern pgo_mapping "pgo_node|fastlio_pgo|run_pgo.sh"' in apply_script
    for runner in (
        "run_driver.sh",
        "run_local_state.sh",
        "run_localization_bridge.sh",
        "run_robot_safety.sh",
        "run_nav2_navigation.sh",
        "run_fastlio_tf.sh",
        "run_pgo.sh",
    ):
        text = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / runner).read_text(encoding="utf-8")
        assert 'source "${SCRIPT_DIR}/cpu_affinity.sh"' in text
    retired_local_perception = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_perception.sh"
    ).read_text(encoding="utf-8")
    assert "robot_local_perception PointCloud2 obstacle pipeline has been removed" in retired_local_perception


def test_api_navigation_resume_resets_inherited_owner_cpu_affinity():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    affinity_helper = (scripts_dir / "cpu_affinity.sh").read_text(encoding="utf-8")
    runtime = (scripts_dir / "run_navigation_runtime_services.sh").read_text(
        encoding="utf-8"
    )

    helper_match = re.search(
        r"(?ms)^njrh_apply_affinity_to_current_process\(\) \{\n.*?^\}\n",
        affinity_helper,
    )
    assert helper_match is not None
    helper = helper_match.group(0)

    assert 'command -v taskset' in helper
    assert 'taskset -pc "${cpuset}" "${pid}"' in helper
    assert '"/proc/${pid}/status"' in helper
    assert "return 1" in helper
    assert (
        "njrh_apply_affinity_to_current_process navigation_runtime_owner"
        in runtime
    )
    affinity_index = runtime.index(
        "njrh_apply_affinity_to_current_process navigation_runtime_owner"
    )
    assert affinity_index < runtime.index('[[ -n "${floor_id}" ]]')
    assert affinity_index < runtime.index("resolve_floor_assets_if_needed")
    assert affinity_index < runtime.index(
        "cleanup_stale_amcl_runtime_status_owner"
    )
    assert affinity_index < runtime.index('log_startup_stage "script_start"')
    assert (
        "navigation runtime owner could not leave its inherited CPU affinity"
        in runtime
    )

    bash_candidates = []
    if sys.platform == "win32":
        bash_candidates.extend(
            (
                Path(r"C:\Program Files\Git\bin\bash.exe"),
                Path(r"C:\Program Files\Git\usr\bin\bash.exe"),
            )
        )
    discovered_bash = shutil.which("bash")
    if discovered_bash:
        bash_candidates.append(Path(discovered_bash))
    bash = next((candidate for candidate in bash_candidates if candidate.exists()), None)
    if bash is None:
        pytest.skip("bash is required for the current-process affinity regression")

    harness = f"""
set -euo pipefail
NJRH_CPU_AFFINITY_ENABLED=true
MOCK_TASKSET_FAIL=0
njrh_affinity_truthy() {{
  case "${{1:-}}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}}
njrh_cpuset_for() {{
  printf '%s\n' "0-7"
}}
taskset() {{
  [[ "${{MOCK_TASKSET_FAIL}}" -eq 0 ]]
}}
awk() {{
  printf '%s\n' "0-7"
}}
{helper}

njrh_apply_affinity_to_current_process navigation_runtime_owner
MOCK_TASKSET_FAIL=1
if njrh_apply_affinity_to_current_process navigation_runtime_owner; then
  exit 91
fi
"""
    result = subprocess.run(
        [str(bash), "-c", harness],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_robot_api_server_directory_layout_matches_domain_taxonomy():
    package_root = ROOT / "src" / "robot_api_server"
    source_root = package_root / "src"
    include_root = package_root / "include" / "robot_api_server"
    test_root = package_root / "test"

    expected_source_directories = {
        "application",
        "application/composition",
        "application/routing",
        "application/runtime_configuration",
        "application/runtime_mode",
        "application/subscriptions",
        "features",
        "features/system_status",
        "features/maps",
        "features/maps/catalog_activation",
        "features/maps/poses",
        "features/maps/keepout",
        "features/mapping",
        "features/mapping/runtime",
        "features/localization",
        "features/floor_switch",
        "features/navigation",
        "features/navigation/configuration",
        "features/navigation/runtime",
        "features/navigation/mission",
        "features/navigation/terminal_control",
        "features/elevator",
        "features/elevator/configuration",
        "features/elevator/execution",
        "features/power",
        "features/docking",
        "features/docking/configuration",
        "features/docking/lifecycle",
        "features/docking/predock_alignment",
        "features/safety",
        "features/teleop",
        "infrastructure",
        "infrastructure/http",
        "infrastructure/process",
    }
    actual_source_directories = {
        path.relative_to(source_root).as_posix()
        for path in source_root.rglob("*")
        if path.is_dir()
    }

    assert actual_source_directories == expected_source_directories
    assert {
        path.name for path in source_root.iterdir() if path.is_file() and path.suffix == ".cpp"
    } == {"robot_api_server_node.cpp"}
    assert not any(path.is_file() for path in include_root.iterdir())
    assert not any(
        path.is_file() and path.suffix in {".cpp", ".hpp", ".py"}
        for path in test_root.iterdir()
    )
    assert all((source_root / directory / "README.md").is_file() for directory in expected_source_directories)


def test_robot_api_server_is_cpp_gateway_not_dashboard_backend():
    package_root = ROOT / "src" / "robot_api_server"
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
    include_root = package_root / "include" / "robot_api_server"
    source_root = package_root / "src"
    application_subscriptions_include = include_root / "application" / "subscriptions"
    application_subscriptions_source = source_root / "application" / "subscriptions"
    application_routing_include = include_root / "application" / "routing"
    application_routing_source = source_root / "application" / "routing"
    application_runtime_configuration_include = (
        include_root / "application" / "runtime_configuration"
    )
    application_runtime_configuration_source = (
        source_root / "application" / "runtime_configuration"
    )
    application_runtime_mode_include = include_root / "application" / "runtime_mode"
    application_runtime_mode_source = source_root / "application" / "runtime_mode"
    docking_configuration_include = include_root / "features" / "docking" / "configuration"
    docking_configuration_source = source_root / "features" / "docking" / "configuration"
    docking_lifecycle_include = include_root / "features" / "docking" / "lifecycle"
    docking_lifecycle_source = source_root / "features" / "docking" / "lifecycle"
    map_catalog_include = include_root / "features" / "maps" / "catalog_activation"
    map_catalog_source = source_root / "features" / "maps" / "catalog_activation"
    map_keepout_include = include_root / "features" / "maps" / "keepout"
    map_keepout_source = source_root / "features" / "maps" / "keepout"
    map_poses_include = include_root / "features" / "maps" / "poses"
    map_poses_source = source_root / "features" / "maps" / "poses"
    mapping_include = include_root / "features" / "mapping"
    mapping_source = source_root / "features" / "mapping"
    floor_switch_include = include_root / "features" / "floor_switch"
    floor_switch_source = source_root / "features" / "floor_switch"
    localization_include = include_root / "features" / "localization"
    localization_source = source_root / "features" / "localization"
    navigation_runtime_include = include_root / "features" / "navigation" / "runtime"
    navigation_runtime_source = source_root / "features" / "navigation" / "runtime"
    navigation_module_cpp = (
        source_root / "features" / "navigation" / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    goal_executor_cpp = (
        source_root / "features" / "navigation" / "mission" / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    power_include = include_root / "features" / "power"
    power_source = source_root / "features" / "power"
    safety_include = include_root / "features" / "safety"
    safety_source = source_root / "features" / "safety"
    system_status_include = include_root / "features" / "system_status"
    system_status_source = source_root / "features" / "system_status"
    teleop_include = include_root / "features" / "teleop"
    teleop_source = source_root / "features" / "teleop"
    http_include = include_root / "infrastructure" / "http"
    http_source = source_root / "infrastructure" / "http"
    process_include = include_root / "infrastructure" / "process"
    process_source = source_root / "infrastructure" / "process"
    api_time_header = (map_catalog_include / "api_time_utils.hpp").read_text(
        encoding="utf-8"
    )
    api_time_cpp = (map_catalog_source / "api_time_utils.cpp").read_text(encoding="utf-8")
    bms_header = (power_include / "bms_contact.hpp").read_text(
        encoding="utf-8"
    )
    bms_cpp = (power_source / "bms_contact.cpp").read_text(encoding="utf-8")
    power_module_header = (power_include / "power_module.hpp").read_text(encoding="utf-8")
    power_module_cpp = (power_source / "power_module.cpp").read_text(encoding="utf-8")
    power_configuration_header = (
        power_include / "power_configuration_module.hpp"
    ).read_text(encoding="utf-8")
    power_configuration_cpp = (
        power_source / "power_configuration_module.cpp"
    ).read_text(encoding="utf-8")
    docking_job_header = (docking_lifecycle_include / "docking_job_model.hpp").read_text(
        encoding="utf-8"
    )
    docking_job_cpp = (docking_lifecycle_source / "docking_job_model.cpp").read_text(encoding="utf-8")
    docking_status_header = (docking_lifecycle_include / "docking_status_utils.hpp").read_text(
        encoding="utf-8"
    )
    docking_status_cpp = (docking_lifecycle_source / "docking_status_utils.cpp").read_text(
        encoding="utf-8"
    )
    docking_status_module_cpp = (
        docking_lifecycle_source / "docking_status_module.cpp"
    ).read_text(encoding="utf-8")
    docking_runtime_header = (
        docking_lifecycle_include / "docking_runtime_module.hpp"
    ).read_text(encoding="utf-8")
    docking_runtime_cpp = (
        docking_lifecycle_source / "docking_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    docking_correction_pause_header = (
        docking_lifecycle_include / "docking_correction_pause_module.hpp"
    ).read_text(encoding="utf-8")
    docking_correction_pause_cpp = (
        docking_lifecycle_source / "docking_correction_pause_module.cpp"
    ).read_text(encoding="utf-8")
    pre_navigation_undock_header = (
        docking_lifecycle_include / "pre_navigation_undock_module.hpp"
    ).read_text(encoding="utf-8")
    pre_navigation_undock_cpp = (
        docking_lifecycle_source / "pre_navigation_undock_module.cpp"
    ).read_text(encoding="utf-8")
    docking_predock_pose_resolver_header = (
        docking_configuration_include / "docking_predock_pose_resolver.hpp"
    ).read_text(encoding="utf-8")
    docking_predock_pose_resolver_cpp = (
        docking_configuration_source / "docking_predock_pose_resolver.cpp"
    ).read_text(encoding="utf-8")
    file_utils_header = (map_catalog_include / "file_utils.hpp").read_text(
        encoding="utf-8"
    )
    file_utils_cpp = (map_catalog_source / "file_utils.cpp").read_text(encoding="utf-8")
    floor_asset_header = (map_catalog_include / "floor_asset_resolver.hpp").read_text(
        encoding="utf-8"
    )
    floor_asset_cpp = (map_catalog_source / "floor_asset_resolver.cpp").read_text(encoding="utf-8")
    http_header = (http_include / "http_common.hpp").read_text(encoding="utf-8")
    http_cpp = (http_source / "http_common.cpp").read_text(encoding="utf-8")
    http_server_header = (http_include / "http_server.hpp").read_text(encoding="utf-8")
    http_server_cpp = (http_source / "http_server.cpp").read_text(encoding="utf-8")
    api_gateway_header = (http_include / "api_gateway_module.hpp").read_text(
        encoding="utf-8"
    )
    api_gateway_cpp = (http_source / "api_gateway_module.cpp").read_text(
        encoding="utf-8"
    )
    api_gateway_configuration_header = (
        http_include / "api_gateway_configuration_module.hpp"
    ).read_text(encoding="utf-8")
    api_gateway_configuration_cpp = (
        http_source / "api_gateway_configuration_module.cpp"
    ).read_text(encoding="utf-8")
    websocket_transport_header = (http_include / "websocket_transport.hpp").read_text(
        encoding="utf-8"
    )
    websocket_transport_cpp = (http_source / "websocket_transport.cpp").read_text(
        encoding="utf-8"
    )
    localization_result_header = (localization_include / "localization_result_model.hpp").read_text(
        encoding="utf-8"
    )
    localization_result_cpp = (localization_source / "localization_result_model.cpp").read_text(
        encoding="utf-8"
    )
    storage_header = (map_catalog_include / "storage_models.hpp").read_text(encoding="utf-8")
    storage_cpp = (map_catalog_source / "storage_models.cpp").read_text(encoding="utf-8")
    map_asset_header = (map_catalog_include / "map_asset_io.hpp").read_text(
        encoding="utf-8"
    )
    map_asset_cpp = (map_catalog_source / "map_asset_io.cpp").read_text(encoding="utf-8")
    map_asset_writer_header = (mapping_include / "map_asset_writer.hpp").read_text(encoding="utf-8")
    map_asset_writer_cpp = (mapping_source / "map_asset_writer.cpp").read_text(encoding="utf-8")
    mapping_module_cpp = (mapping_source / "mapping_module.cpp").read_text(encoding="utf-8")
    mapping_runtime_cpp = (
        mapping_source / "runtime" / "mapping_process_runtime.cpp"
    ).read_text(encoding="utf-8")
    map_catalog_header = (map_catalog_include / "map_catalog.hpp").read_text(encoding="utf-8")
    map_catalog_cpp = (map_catalog_source / "map_catalog.cpp").read_text(encoding="utf-8")
    manifest_io_header = (map_catalog_include / "map_manifest_io.hpp").read_text(encoding="utf-8")
    manifest_io_cpp = (map_catalog_source / "map_manifest_io.cpp").read_text(encoding="utf-8")
    navigation_cancel_header = (navigation_runtime_include / "navigation_cancel_job_model.hpp").read_text(
        encoding="utf-8"
    )
    navigation_cancel_cpp = (navigation_runtime_source / "navigation_cancel_job_model.cpp").read_text(
        encoding="utf-8"
    )
    navigation_goal_job_cpp = (
        source_root / "features" / "navigation" / "mission" / "navigation_goal_job.cpp"
    ).read_text(encoding="utf-8")
    navigation_terminal_cpp = (
        source_root
        / "features"
        / "navigation"
        / "terminal_control"
        / "navigation_terminal_control.cpp"
    ).read_text(encoding="utf-8")
    poses_io_header = (map_poses_include / "poses_io.hpp").read_text(encoding="utf-8")
    poses_io_cpp = (map_poses_source / "poses_io.cpp").read_text(encoding="utf-8")
    runtime_context_header = (floor_switch_include / "runtime_map_context_io.hpp").read_text(
        encoding="utf-8"
    )
    runtime_context_cpp = (floor_switch_source / "runtime_map_context_io.cpp").read_text(encoding="utf-8")
    runtime_map_lookup_header = (map_catalog_include / "runtime_map_lookup.hpp").read_text(
        encoding="utf-8"
    )
    runtime_map_lookup_cpp = (map_catalog_source / "runtime_map_lookup.cpp").read_text(encoding="utf-8")
    robot_pose_header = (system_status_include / "robot_pose_model.hpp").read_text(encoding="utf-8")
    robot_pose_cpp = (system_status_source / "robot_pose_model.cpp").read_text(encoding="utf-8")
    system_status_module_header = (system_status_include / "system_status_module.hpp").read_text(
        encoding="utf-8"
    )
    system_status_module_cpp = (system_status_source / "system_status_module.cpp").read_text(
        encoding="utf-8"
    )
    system_status_wiring_header = (
        system_status_include / "system_status_wiring.hpp"
    ).read_text(encoding="utf-8")
    system_status_wiring_cpp = (
        system_status_source / "system_status_wiring.cpp"
    ).read_text(encoding="utf-8")
    safety_state_header = (safety_include / "safety_state.hpp").read_text(encoding="utf-8")
    safety_state_cpp = (safety_source / "safety_state.cpp").read_text(encoding="utf-8")
    safety_adapter_header = (safety_include / "safety_ros_adapter.hpp").read_text(encoding="utf-8")
    safety_adapter_cpp = (safety_source / "safety_ros_adapter.cpp").read_text(encoding="utf-8")
    safety_configuration_header = (
        safety_include / "safety_configuration_module.hpp"
    ).read_text(encoding="utf-8")
    safety_configuration_cpp = (
        safety_source / "safety_configuration_module.cpp"
    ).read_text(encoding="utf-8")
    safety_module_header = (safety_include / "safety_module.hpp").read_text(encoding="utf-8")
    safety_module_cpp = (safety_source / "safety_module.cpp").read_text(encoding="utf-8")
    teleop_header = (teleop_include / "teleop_session.hpp").read_text(encoding="utf-8")
    teleop_cpp = (teleop_source / "teleop_session.cpp").read_text(encoding="utf-8")
    teleop_module_header = (teleop_include / "teleop_module.hpp").read_text(
        encoding="utf-8"
    )
    teleop_module_cpp = (teleop_source / "teleop_module.cpp").read_text(
        encoding="utf-8"
    )
    teleop_configuration_header = (
        teleop_include / "teleop_configuration_module.hpp"
    ).read_text(encoding="utf-8")
    teleop_configuration_cpp = (
        teleop_source / "teleop_configuration_module.cpp"
    ).read_text(encoding="utf-8")
    runtime_process_header = (process_include / "runtime_process_utils.hpp").read_text(encoding="utf-8")
    runtime_process_cpp = (process_source / "runtime_process_utils.cpp").read_text(encoding="utf-8")
    semantic_header = (map_keepout_include / "semantic_layer_io.hpp").read_text(encoding="utf-8")
    semantic_cpp = (map_keepout_source / "semantic_layer_io.cpp").read_text(encoding="utf-8")
    subscription_api_header = (application_subscriptions_include / "subscription_api.hpp").read_text(
        encoding="utf-8"
    )
    subscription_api_cpp = (application_subscriptions_source / "subscription_api.cpp").read_text(
        encoding="utf-8"
    )
    subscription_header = (application_subscriptions_include / "subscription_manager.hpp").read_text(
        encoding="utf-8"
    )
    subscription_cpp = (application_subscriptions_source / "subscription_manager.cpp").read_text(
        encoding="utf-8"
    )
    subscription_module_header = (
        application_subscriptions_include / "subscription_module.hpp"
    ).read_text(encoding="utf-8")
    subscription_module_cpp = (
        application_subscriptions_source / "subscription_module.cpp"
    ).read_text(encoding="utf-8")
    subscription_configuration_header = (
        application_subscriptions_include / "subscription_configuration_module.hpp"
    ).read_text(encoding="utf-8")
    subscription_configuration_cpp = (
        application_subscriptions_source / "subscription_configuration_module.cpp"
    ).read_text(encoding="utf-8")
    runtime_configuration_header = (
        application_runtime_configuration_include / "runtime_configuration_module.hpp"
    ).read_text(encoding="utf-8")
    runtime_configuration_cpp = (
        application_runtime_configuration_source / "runtime_configuration_module.cpp"
    ).read_text(encoding="utf-8")
    runtime_mode_header = (
        application_runtime_mode_include / "runtime_mode_coordinator.hpp"
    ).read_text(encoding="utf-8")
    runtime_mode_cpp = (
        application_runtime_mode_source / "runtime_mode_coordinator.cpp"
    ).read_text(encoding="utf-8")
    application_router_header = (
        application_routing_include / "application_router_module.hpp"
    ).read_text(encoding="utf-8")
    application_router_cpp = (
        application_routing_source / "application_router_module.cpp"
    ).read_text(encoding="utf-8")
    application_router_wiring_cpp = (
        application_routing_source / "application_router_wiring.cpp"
    ).read_text(encoding="utf-8")
    tf_pose_header = (localization_include / "tf_pose_utils.hpp").read_text(encoding="utf-8")
    tf_pose_cpp = (localization_source / "tf_pose_utils.cpp").read_text(encoding="utf-8")
    entry_cpp = (package_root / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    composition_cpp = application_composition_source()
    node_cpp = "\n".join(
        path.read_text(encoding="utf-8")
        for root, suffix in ((source_root, "*.cpp"), (include_root, "*.hpp"))
        for path in sorted(root.rglob(suffix))
    )
    config = (package_root / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    launch = (package_root / "launch" / "robot_api_server.launch.py").read_text(encoding="utf-8")
    readme = (package_root / "README.md").read_text(encoding="utf-8")
    overlay_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    overlay_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_api_server.sh"
    ).read_text(encoding="utf-8")
    overlay_supervisor_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_api_server_supervised.sh"
    ).read_text(encoding="utf-8")
    systemd_runtime_script = (
        ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh"
    ).read_text(encoding="utf-8")
    container_runtime_script = (
        ROOT / "scripts" / "jetson" / "njrh_container.sh"
    ).read_text(encoding="utf-8")
    floor_navigation_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_floor_navigation.sh"
    ).read_text(encoding="utf-8")
    resident_runtime_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    commercial_runtime_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "commercial_runtime_helpers.sh"
    ).read_text(encoding="utf-8")
    overlay_nav2_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_nav2_navigation.sh"
    ).read_text(encoding="utf-8")
    occupancy_localization_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh"
    ).read_text(encoding="utf-8")
    occupancy_localization_launch = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization.launch.py"
    ).read_text(encoding="utf-8")
    stop_navigation_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "stop_floor_navigation.sh"
    ).read_text(encoding="utf-8")
    floor_asset_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "floor_asset_helpers.sh"
    ).read_text(encoding="utf-8")
    floor_manager_code = (ROOT / "src" / "robot_floor_manager" / "src" / "floor_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    global_localization_node = (
        ROOT / "src" / "robot_global_localization" / "src" / "global_localization_node.cpp"
    ).read_text(encoding="utf-8")
    global_localization_config = (
        ROOT / "src" / "robot_global_localization" / "config" / "global_localization.yaml"
    ).read_text(encoding="utf-8")
    global_localization_package = (ROOT / "src" / "robot_global_localization" / "package.xml").read_text(
        encoding="utf-8"
    )
    nav_runtime_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "nav_runtime_helpers.sh"
    ).read_text(encoding="utf-8")
    runtime_probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")
    map_server_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "map_server_helpers.sh"
    ).read_text(encoding="utf-8")
    global_localization_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_global_localization.sh"
    ).read_text(encoding="utf-8")
    app_doc = (ROOT / "docs" / "android_app_api.md").read_text(encoding="utf-8")
    lifecycle_doc = (ROOT / "docs" / "runtime_service_lifecycle.md").read_text(encoding="utf-8")

    assert "add_executable(robot_api_server_node" in cmake
    assert "src/features/maps/catalog_activation/api_time_utils.cpp" in cmake
    assert "src/features/power/bms_contact.cpp" in cmake
    assert "src/features/power/power_module.cpp" in cmake
    assert "src/features/docking/lifecycle/docking_job_model.cpp" in cmake
    assert "src/features/docking/lifecycle/docking_runtime_module.cpp" in cmake
    assert "src/features/docking/lifecycle/docking_correction_pause_module.cpp" in cmake
    assert "src/features/docking/lifecycle/docking_status_utils.cpp" in cmake
    assert "src/features/maps/catalog_activation/file_utils.cpp" in cmake
    assert "src/features/maps/catalog_activation/floor_asset_resolver.cpp" in cmake
    assert "src/infrastructure/http/http_common.cpp" in cmake
    assert "src/infrastructure/http/api_gateway_module.cpp" in cmake
    assert "src/infrastructure/http/http_server.cpp" in cmake
    assert "src/features/localization/localization_result_model.cpp" in cmake
    assert "src/features/maps/catalog_activation/map_asset_io.cpp" in cmake
    assert "src/features/mapping/map_asset_writer.cpp" in cmake
    assert "src/features/maps/catalog_activation/map_catalog.cpp" in cmake
    assert "src/features/maps/catalog_activation/map_manifest_io.cpp" in cmake
    assert "src/features/navigation/runtime/navigation_cancel_job_model.cpp" in cmake
    assert "src/features/maps/poses/poses_io.cpp" in cmake
    assert "src/features/floor_switch/runtime_map_context_io.cpp" in cmake
    assert "src/features/maps/catalog_activation/runtime_map_lookup.cpp" in cmake
    assert "src/features/system_status/robot_pose_model.cpp" in cmake
    assert "src/features/system_status/system_status_module.cpp" in cmake
    assert "src/features/system_status/system_status_wiring.cpp" in cmake
    assert "src/infrastructure/process/runtime_process_utils.cpp" in cmake
    assert "src/features/maps/keepout/semantic_layer_io.cpp" in cmake
    assert "src/features/maps/catalog_activation/storage_models.cpp" in cmake
    assert "src/application/runtime_configuration/runtime_configuration_module.cpp" in cmake
    assert "src/application/routing/application_router_module.cpp" in cmake
    assert "src/application/routing/application_router_wiring.cpp" in cmake
    assert "src/application/subscriptions/subscription_api.cpp" in cmake
    assert "src/application/subscriptions/subscription_configuration_module.cpp" in cmake
    assert "src/application/subscriptions/subscription_manager.cpp" in cmake
    assert "src/application/subscriptions/subscription_module.cpp" in cmake
    assert "src/application/runtime_mode/runtime_mode_coordinator.cpp" in cmake
    assert "src/features/localization/tf_pose_utils.cpp" in cmake
    assert "target_include_directories(robot_api_server_node PRIVATE include)" in cmake
    assert '#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"' in node_cpp
    assert "class RuntimeModeCoordinator" in runtime_mode_header
    assert "RuntimeModeSnapshot snapshot() const" in runtime_mode_header
    assert "try_begin_transition" in runtime_mode_header
    assert "RuntimeModeCoordinator::accept_docking" in runtime_mode_cpp
    assert "RuntimeModeCoordinator::finish_docking" in runtime_mode_cpp
    assert 'snapshot.mode = "ERROR"' in runtime_mode_cpp
    assert 'snapshot.mode = "DOCKING"' in runtime_mode_cpp
    assert 'snapshot.mode = "MAPPING_2D"' in runtime_mode_cpp
    assert 'snapshot.mode = "NAVIGATION"' in runtime_mode_cpp
    assert "RuntimeModeCoordinator runtime_mode_;" in node_cpp
    assert "runtime_mode_mutex_" not in composition_cpp
    assert "mode_transition_mutex_" not in composition_cpp
    assert "class DockingRuntimeModule" in docking_runtime_header
    assert "DockingObservationSnapshot observation_snapshot() const" in docking_runtime_header
    assert "create_client<std_srvs::srv::Trigger>" in docking_runtime_cpp
    assert "create_subscription<robot_interfaces::msg::DockTargetObservation>" in docking_runtime_cpp
    assert "create_subscription<sensor_msgs::msg::LaserScan>" in docking_runtime_cpp
    assert "prepare_child_process" in docking_runtime_cpp
    assert "std::unique_ptr<DockingRuntimeModule> runtime_;" in node_cpp
    assert "docking_manager_pid_" not in composition_cpp
    assert "docking_target_observation_mutex_" not in composition_cpp
    assert "predock_yaw_align_cmd_pub_" not in composition_cpp
    assert "docking_undock_client_" not in composition_cpp
    assert "class DockingCorrectionPauseModule" in docking_correction_pause_header
    assert "bool set_paused(" in docking_correction_pause_header
    assert "bool release_stale_if_needed(" in docking_correction_pause_header
    assert "DockingCorrectionPauseModule::set_paused" in docking_correction_pause_cpp
    assert "DockingCorrectionPauseModule::release_stale_if_needed" in docking_correction_pause_cpp
    assert "std::unique_ptr<DockingCorrectionPauseModule> correction_pause_;" in node_cpp
    assert "set_global_correction_paused_for_docking" not in composition_cpp
    assert "release_stale_docking_fine_pause_if_needed" not in composition_cpp
    assert "class DockingPredockPoseResolver" in docking_predock_pose_resolver_header
    assert "std::optional<StoredPose> resolve(" in docking_predock_pose_resolver_header
    assert "bool validate(" in docking_predock_pose_resolver_header
    assert "DockingPredockPoseResolver::resolve" in docking_predock_pose_resolver_cpp
    assert "DockingPredockPoseResolver::validate" in docking_predock_pose_resolver_cpp
    assert "std::unique_ptr<configuration::DockingPredockPoseResolver> predock_pose_resolver_;" in node_cpp
    assert "resolve_docking_predock_pose" not in composition_cpp
    assert "validate_manual_docking_predock_pose" not in composition_cpp
    assert "geometry_msgs" in cmake
    assert "lifecycle_msgs" in cmake
    assert "nav_msgs" in cmake
    assert "robot_interfaces" in cmake
    assert "sensor_msgs" in cmake
    assert "std_msgs" in cmake
    assert "tf2_msgs" in cmake
    assert "<depend>geometry_msgs</depend>" in package_xml
    assert "<depend>lifecycle_msgs</depend>" in package_xml
    assert "<depend>nav_msgs</depend>" in package_xml
    assert "<depend>robot_interfaces</depend>" in package_xml
    assert "<depend>sensor_msgs</depend>" in package_xml
    assert "<depend>tf2_msgs</depend>" in package_xml
    assert '#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"' in node_cpp
    assert "utc_timestamp_compact" in api_time_header
    assert "utc_timestamp_iso8601" in api_time_header
    assert "wall_time_seconds" in api_time_header
    assert "generate_current_pose_id" in api_time_header
    assert "generate_map_id" in api_time_header
    assert 'std::put_time(&tm, "%Y%m%dT%H%M%SZ")' in api_time_cpp
    assert 'std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ")' in api_time_cpp
    assert "safe_pose_id(prefix)" in api_time_cpp
    assert "fixed_hex(fnv1a64(seed), 8)" in api_time_cpp
    assert "fixed_hex(fnv1a64(seed), 10)" in api_time_cpp
    assert "std::string utc_timestamp_compact() const" not in composition_cpp
    assert "std::string utc_timestamp_iso8601() const" not in composition_cpp
    assert "double wall_time_seconds() const" not in composition_cpp
    assert "std::string generated_current_pose_id" not in composition_cpp
    assert '#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"' in node_cpp
    assert "read_text_file" in file_utils_header
    assert "read_optional_text_file" in file_utils_header
    assert "read_binary_file" in file_utils_header
    assert "write_text_file" in file_utils_header
    assert "write_binary_file" in file_utils_header
    assert "write_pgm_file" in file_utils_header
    assert "yaml_with_image_file" in file_utils_header
    assert "copy_file_if_exists" in file_utils_header
    assert "copy_yaml_with_image_if_exists" in file_utils_header
    assert "std::ifstream file(path)" in file_utils_cpp
    assert "std::ofstream file(path" in file_utils_cpp
    assert 'out << "image: " << image_file' in file_utils_cpp
    assert "fs::copy_file(source, target" in file_utils_cpp
    assert "write_binary_file(path, payload)" in file_utils_cpp
    assert "std::string read_text_file(const fs::path" not in composition_cpp
    assert "std::string read_optional_text_file(const fs::path" not in composition_cpp
    assert "void write_text_file(const fs::path" not in composition_cpp
    assert "void write_pgm_file(" not in composition_cpp
    assert "std::string yaml_with_image_file" not in composition_cpp
    assert '#include "robot_api_server/features/maps/catalog_activation/floor_asset_resolver.hpp"' in node_cpp
    assert "struct FloorAssetPaths" in floor_asset_header
    assert "resolve_floor_asset_paths" in floor_asset_header
    assert "poses_yaml_path" in floor_asset_header
    assert "find_floor_catalog_pose" in floor_asset_header
    assert "bool resolve_floor_asset_paths" in floor_asset_cpp
    assert "floor asset is incomplete, missing" in floor_asset_cpp
    assert "current_root / \"nav\" / \"nav_map.yaml\"" in floor_asset_cpp
    assert "fs::path poses_yaml_path" in floor_asset_cpp
    assert "current_poses" in floor_asset_cpp
    assert "find_floor_pose(path, pose_id)" in floor_asset_cpp
    assert "struct FloorAssetPaths" not in composition_cpp
    assert "bool resolve_floor_asset_paths" not in composition_cpp
    assert "fs::path poses_yaml_path" not in composition_cpp
    assert "std::optional<StoredPose> find_floor_pose" not in composition_cpp
    assert '#include "robot_api_server/features/navigation/runtime/navigation_cancel_job_model.hpp"' in node_cpp
    assert "struct NavigationCancelJob" in navigation_cancel_header
    assert "navigation_cancel_job_json" in navigation_cancel_header
    assert "std::string navigation_cancel_job_json" in navigation_cancel_cpp
    assert "cancel_all_detail" in navigation_cancel_cpp
    assert '"navigation_stack_stopped\\\":" << (' not in navigation_cancel_cpp
    assert "job.stop_stack && job.stop_stack_ok" in navigation_cancel_cpp
    assert "struct NavigationCancelJob" not in composition_cpp
    assert "std::string navigation_cancel_job_json(const NavigationCancelJob" not in composition_cpp
    assert '#include "robot_api_server/features/docking/lifecycle/docking_job_model.hpp"' in node_cpp
    assert "struct DockingJob" in docking_job_header
    assert "docking_job_json" in docking_job_header
    assert "std::string docking_job_json" in docking_job_cpp
    assert "post_undock_relocalization_detail" in docking_job_cpp
    assert "post_predock_relocalization_detail" in docking_job_cpp
    assert "post_fine_docking_relocalization_detail" in docking_job_cpp
    assert "predock_pose_id" in docking_job_header
    assert "approach_source" in docking_job_header
    assert "predock_pose_id" in docking_job_cpp
    assert "approach_source" in docking_job_cpp
    assert "post_predock_relocalization_requested" in docking_job_cpp
    assert "post_predock_relocalization_succeeded" in docking_job_cpp
    assert "post_predock_relocalization_required" in docking_job_cpp
    assert "post_fine_docking_relocalization_requested" in docking_job_cpp
    assert "post_fine_docking_relocalization_succeeded" in docking_job_cpp
    assert "post_fine_docking_relocalization_required" in docking_job_cpp
    assert "active_navigation_cancel_detail" in docking_job_cpp
    assert "approach_distance_m" in docking_job_cpp
    assert "struct DockingJob" not in composition_cpp
    assert "std::string docking_job_json(const DockingJob" not in composition_cpp
    assert "socket(AF_INET, SOCK_STREAM" in http_server_cpp
    assert "socket(AF_INET, SOCK_STREAM" not in composition_cpp
    assert "class HttpServer" in http_server_header
    assert "HttpServerCallbacks" in http_server_header
    assert "active_connections() const" in http_server_header
    assert "callbacks_.route(*request, peer_is_loopback(client_fd))" in http_server_cpp
    assert "class ApiGatewayModule" in api_gateway_header
    assert "class ApiGatewayConfigurationModule" in api_gateway_configuration_header
    assert "max_http_connections" in api_gateway_configuration_cpp
    assert "ApiGatewayConfigurationModule::declare_parameters" in node_cpp
    assert "std::unique_ptr<HttpServer> server_" in api_gateway_header
    assert "std::unique_ptr<ApiGatewayModule> api_gateway_module_" in node_cpp
    assert "dependencies.api_gateway->active_connections()" in node_cpp
    assert "std::unique_ptr<HttpServer>" not in composition_cpp
    assert "Sec-WebSocket-Accept" in websocket_transport_cpp
    assert "websocket_accept_key" in websocket_transport_cpp
    assert '#include "robot_api_server/infrastructure/http/http_common.hpp"' in node_cpp
    assert "struct HttpRequest" in http_header
    assert "struct HttpResponse" in http_header
    assert "struct WebSocketFrame" in http_header
    assert "parse_http_request" in http_cpp
    assert "content_length_from_headers" in http_cpp
    assert "reason_phrase" in http_cpp
    assert "websocket_accept_key" in http_cpp
    assert "json_string_value" in http_cpp
    assert "json_object_array_value" in http_cpp
    assert "struct HttpRequest" not in composition_cpp
    assert "std::optional<HttpRequest> parse_http_request" not in composition_cpp
    assert '#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"' in node_cpp
    assert "struct StoredPose" in storage_header
    assert "struct MapManifest" in storage_header
    assert "struct RuntimeMapContext" in storage_header
    assert "std::string startup_stage" in storage_header
    assert "struct MapYamlInfo" in storage_header
    assert "safe_pose_id" in storage_cpp
    assert "safe_asset_id" in storage_cpp
    assert "valid_display_map_name" in storage_cpp
    assert "safe_file_stem_from_display_name" in storage_cpp
    assert "fnv1a64" in storage_cpp
    assert "fixed_hex" in storage_cpp
    assert "struct StoredPose" not in composition_cpp
    assert "struct MapManifest" not in composition_cpp
    assert "bool safe_pose_id" not in composition_cpp
    assert "bool safe_asset_id" not in composition_cpp
    assert '#include "robot_api_server/features/maps/catalog_activation/map_asset_io.hpp"' in node_cpp
    assert "encode_grayscale_png" in map_asset_header
    assert "read_pgm_dimensions" in map_asset_header
    assert "read_nav_map_info" in map_asset_header
    assert "map_info_json" in map_asset_header
    assert "append_u32_be" in map_asset_cpp
    assert "zlib_store_blocks" in map_asset_cpp
    assert "parse_yaml_origin" in map_asset_cpp
    assert "read_pgm_dimensions" in map_asset_cpp
    assert "read_nav_map_info" in map_asset_cpp
    assert "map_info_json" in map_asset_cpp
    assert "std::optional<MapYamlInfo> read_nav_map_info" not in composition_cpp
    assert "std::optional<std::pair<std::uint32_t, std::uint32_t>> read_pgm_dimensions" not in composition_cpp
    assert '#include "robot_api_server/features/mapping/map_asset_writer.hpp"' in node_cpp
    assert "occupancy_to_gray" in map_asset_writer_header
    assert "occupancy_grid_to_image_pixels" in map_asset_writer_header
    assert "map_yaml_text" in map_asset_writer_header
    assert "write_neutral_filter_assets" in map_asset_writer_header
    assert "write_asset_report" in map_asset_writer_header
    assert "std::uint8_t occupancy_to_gray" in map_asset_writer_cpp
    assert "std::vector<std::uint8_t> occupancy_grid_to_image_pixels" in map_asset_writer_cpp
    assert '"keepout_mask", "speed_mask", "binary_mask"' in map_asset_writer_cpp
    assert "robot_api_server_slam_toolbox_save" in map_asset_writer_cpp
    assert "254U" in map_asset_writer_cpp
    assert "std::uint8_t occupancy_to_gray" not in composition_cpp
    assert "std::vector<std::uint8_t> occupancy_grid_to_image_pixels" not in composition_cpp
    assert "std::string map_yaml_text" not in composition_cpp
    assert '#include "robot_api_server/features/maps/catalog_activation/map_catalog.hpp"' in node_cpp
    assert "class MapCatalog" in map_catalog_header
    assert "LegacyMigrationCallback" in map_catalog_header
    assert "floor_root_path" in map_catalog_header
    assert "read_floor_map_manifests" in map_catalog_header
    assert "read_all_map_manifests" in map_catalog_header
    assert "find_map_by_id" in map_catalog_header
    assert "active_floor_map" in map_catalog_header
    assert "unique_active_map_manifest" in map_catalog_header
    assert "MapCatalog::read_floor_map_manifests" in map_catalog_cpp
    assert "legacy_migration_(building_id, floor_id)" in map_catalog_cpp
    assert "read_map_manifest(entry.path() / \"manifest.json\")" in map_catalog_cpp
    assert "MapCatalog::read_all_map_manifests" in map_catalog_cpp
    assert "MapCatalog::find_map_by_id" in map_catalog_cpp
    assert "MapCatalog::active_floor_map" in map_catalog_cpp
    assert "MapCatalog::unique_active_map_manifest" in map_catalog_cpp
    assert "std::vector<MapManifest> read_floor_map_manifests" not in composition_cpp
    assert "std::vector<MapManifest> read_all_map_manifests" not in composition_cpp
    assert "std::optional<MapManifest> find_map_by_id" not in composition_cpp
    assert "std::optional<MapManifest> active_floor_map" not in composition_cpp
    assert "std::optional<MapManifest> unique_active_map_manifest" not in composition_cpp
    assert "fs::path floor_root_path" not in composition_cpp
    assert '#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"' in node_cpp
    assert "fill_manifest_paths" in manifest_io_header
    assert "map_manifest_json" in manifest_io_header
    assert "read_map_manifest" in manifest_io_header
    assert "write_map_manifest" in manifest_io_header
    assert "manifest.nav_map_yaml" in manifest_io_cpp
    assert "manifest.localizer_map_png" in manifest_io_cpp
    assert "manifest.asset_report_json" in manifest_io_cpp
    assert "StrictJsonObjectParser" in manifest_io_cpp
    assert "contains a duplicate object key" in manifest_io_cpp
    assert "JsonScalarKind::kString" in manifest_io_cpp
    assert "JsonScalarKind::kBoolean" in manifest_io_cpp
    assert "std::optional<MapManifest> read_map_manifest" not in composition_cpp
    assert "void write_map_manifest" not in composition_cpp
    assert "std::string map_manifest_json" not in composition_cpp
    assert "void fill_manifest_paths" not in composition_cpp
    assert '#include "robot_api_server/features/floor_switch/runtime_map_context_io.hpp"' in node_cpp
    assert "write_runtime_map_context_file" in runtime_context_header
    assert "read_runtime_map_context_file" in runtime_context_header
    assert "njrh.runtime_map_context.v1" in runtime_context_cpp
    assert "json_string(state)" in runtime_context_cpp
    assert 'json_bool_value(text, "confirmed", false)' in runtime_context_cpp
    assert "safe_asset_id(*map_id)" in runtime_context_cpp
    assert "write_runtime_map_context_file(" in node_cpp
    assert "read_runtime_map_context_file(" in node_cpp
    assert "njrh.runtime_map_context.v1" not in composition_cpp
    assert "double updated_at_sec{0.0};" in storage_header
    assert 'context.updated_at_sec = json_number_value(text, "updated_at").value_or(0.0);' in runtime_context_cpp
    assert '#include "robot_api_server/features/maps/catalog_activation/runtime_map_lookup.hpp"' in node_cpp
    assert "safe_map_name" in runtime_map_lookup_header
    assert "runtime_map_asset_paths" in runtime_map_lookup_header
    assert "newest_png_in_directory" in runtime_map_lookup_header
    assert "newest_floor_localizer_png" in runtime_map_lookup_header
    assert "resolve_mapping_2d_png" in runtime_map_lookup_header
    assert "bool safe_map_name" in runtime_map_lookup_cpp
    assert "std::vector<fs::path> runtime_map_asset_paths" in runtime_map_lookup_cpp
    assert "std::optional<fs::path> newest_png_in_directory" in runtime_map_lookup_cpp
    assert "std::optional<fs::path> newest_floor_localizer_png" in runtime_map_lookup_cpp
    assert "std::optional<fs::path> resolve_mapping_2d_png" in runtime_map_lookup_cpp
    assert "name.empty() || name == \".\"" in runtime_map_lookup_cpp
    assert "name.find(\"..\")" in runtime_map_lookup_cpp
    assert "localizer_map.png" in runtime_map_lookup_cpp
    assert "std::optional<fs::path> resolve_mapping_2d_png" not in composition_cpp
    assert "std::optional<fs::path> newest_png_in_directory" not in composition_cpp
    assert "std::vector<fs::path> runtime_map_asset_paths" not in composition_cpp
    assert '#include "robot_api_server/infrastructure/process/runtime_process_utils.hpp"' in docking_runtime_cpp
    assert '#include "robot_api_server/infrastructure/process/runtime_process_utils.hpp"' not in composition_cpp
    assert "prepare_child_process" in runtime_process_header
    assert "read_proc_cmdline" in runtime_process_header
    assert "process_group_has_live_process" in runtime_process_header
    assert "signal_process_group" in runtime_process_header
    assert "void set_close_on_exec" in runtime_process_cpp
    assert "void close_inherited_fds" in runtime_process_cpp
    assert "::setsid()" in runtime_process_cpp
    assert "::dup2(log_fd, STDOUT_FILENO)" in runtime_process_cpp
    assert 'fs::path("/proc")' in runtime_process_cpp
    assert "return trim(cmdline)" in runtime_process_cpp
    assert "std::vector<pid_t> list_proc_pids" in runtime_process_header
    assert "fs::directory_options::skip_permission_denied" in runtime_process_cpp
    assert "catch (const std::ios_base::failure &)" in runtime_process_cpp
    assert 'for (const pid_t process_id : list_proc_pids())' in mapping_runtime_cpp
    assert 'fs::directory_iterator("/proc")' not in composition_cpp
    assert "void set_close_on_exec" not in composition_cpp
    assert "void close_inherited_fds" not in composition_cpp
    assert "std::string read_proc_cmdline" not in composition_cpp
    assert "bool process_group_has_live_process" not in composition_cpp
    assert '#include "robot_api_server/features/maps/keepout/semantic_layer_io.hpp"' in node_cpp
    assert "keepout_semantic_json_path" in semantic_header
    assert "json_raw_or_null" in semantic_header
    assert "keepout_semantic_payload_json" in semantic_header
    assert "keepout_filter_json" in semantic_header
    assert '"keepout_semantic_layer.json"' in semantic_cpp
    assert 'json_object_value(semantic_json, "keepout")' in semantic_cpp
    assert "keepout_mask_yaml" in semantic_cpp
    assert "keepout_payload" in semantic_cpp
    assert "std::string json_raw_or_null" not in composition_cpp
    assert "std::string keepout_semantic_payload_json" not in composition_cpp
    assert "std::string keepout_filter_json" not in composition_cpp
    assert '#include "robot_api_server/application/subscriptions/subscription_module.hpp"' in node_cpp
    assert "class SubscriptionConfigurationModule" in subscription_configuration_header
    assert "subscription_default_ttl_ms" in subscription_configuration_cpp
    assert "SubscriptionConfigurationModule::declare_parameters" in node_cpp
    assert "class RuntimeConfigurationModule" in runtime_configuration_header
    assert "navigate_to_pose_status_topic" in runtime_configuration_cpp
    assert "RuntimeConfigurationModule::declare_parameters" in node_cpp
    assert "class ApplicationRouterModule" in application_router_header
    assert "HttpResponse ApplicationRouterModule::route" in application_router_cpp
    assert "make_application_router_ports" in application_router_wiring_cpp
    assert '#include "robot_api_server/application/routing/application_router_module.hpp"' in node_cpp
    assert "HttpResponse route(" not in composition_cpp
    assert '#include "robot_api_server/application/subscriptions/subscription_api.hpp"' not in composition_cpp
    assert '#include "robot_api_server/application/subscriptions/subscription_api.hpp"' in subscription_module_cpp
    assert "class SubscriptionModule" in subscription_module_header
    assert "std::optional<HttpResponse> handle_http" in subscription_module_header
    assert "safe_client_id" in subscription_api_header
    assert "subscription_ttl_ms_from_body" in subscription_api_header
    assert "subscription_resources_from_body" in subscription_api_header
    assert "subscription_client_id_from_body" in subscription_api_header
    assert "resource_list_json" in subscription_api_header
    assert "client_id\", \"clientId\", \"lease_id\"" in subscription_api_cpp
    assert "http:compat-default" in subscription_api_cpp
    assert "std::clamp(static_cast<int>(*ttl), 1000, max_ttl_ms)" in subscription_api_cpp
    assert "json_string_array_value(body, \"resources\")" in subscription_api_cpp
    assert "std::sort(resources.begin(), resources.end())" in subscription_api_cpp
    assert "bool safe_client_id" not in composition_cpp
    assert "int subscription_ttl_ms_from_body" not in composition_cpp
    assert "std::vector<std::string> subscription_resources_from_body" not in composition_cpp
    assert "std::pair<std::string, std::string> subscription_client_id_from_body" not in composition_cpp
    assert "std::string resource_list_json" not in composition_cpp
    assert '#include "robot_api_server/features/maps/poses/poses_io.hpp"' in node_cpp
    assert "poses_json_array" in poses_io_header
    assert "read_floor_poses" in poses_io_header
    assert "write_floor_poses" in poses_io_header
    assert "find_floor_pose" in poses_io_header
    assert "apply_pose_yaml_field" in poses_io_cpp
    assert 'yaml << "poses:\\n"' in poses_io_cpp
    assert "json_string(pose.id)" in poses_io_cpp
    assert "normalize_angle(pose.yaw)" in poses_io_cpp
    assert "std::vector<StoredPose> read_floor_poses" not in composition_cpp
    assert "void write_floor_poses" not in composition_cpp
    assert "std::string poses_json_array" not in composition_cpp
    assert "void apply_pose_yaml_field" not in composition_cpp
    assert '#include "robot_api_server/application/subscriptions/subscription_manager.hpp"' not in composition_cpp
    assert '#include "robot_api_server/application/subscriptions/subscription_manager.hpp"' in subscription_module_cpp
    assert "class SubscriptionManager" in subscription_header
    assert "validate_resources" in subscription_header
    assert "snapshot_json" in subscription_header
    assert "leases_" in subscription_header
    assert "SubscriptionManager::acquire" in subscription_cpp
    assert "SubscriptionManager::expire" in subscription_cpp
    assert "json_string(resource)" in subscription_cpp
    assert "class SubscriptionManager" not in composition_cpp
    assert "std::map<std::string, std::map<std::string, Clock::time_point>> leases_" not in composition_cpp
    assert '#include "robot_api_server/features/localization/tf_pose_utils.hpp"' in node_cpp
    assert '#include "robot_api_server/features/localization/tf_pose_utils.hpp"' in poses_io_cpp
    assert "normalized_frame_id" in tf_pose_header
    assert "normalize_angle" in tf_pose_header
    assert "stamp_to_seconds" in tf_pose_header
    assert "older_nonzero_stamp" in tf_pose_header
    assert "quaternion_yaw" in tf_pose_header
    assert "frame.front() == '/'" in tf_pose_cpp
    assert "std::atan2(std::sin(angle), std::cos(angle))" in tf_pose_cpp
    assert "static_cast<double>(stamp.sec)" in tf_pose_cpp
    assert "std::min(lhs, rhs)" in tf_pose_cpp
    assert "siny_cosp" in tf_pose_cpp
    assert "std::string normalized_frame_id(std::string frame) const" not in composition_cpp
    assert "double normalize_angle(const double angle) const" not in composition_cpp
    assert "double stamp_to_seconds(const builtin_interfaces::msg::Time" not in composition_cpp
    assert "double older_nonzero_stamp(const double lhs" not in composition_cpp
    assert "double quaternion_yaw(const double x" not in composition_cpp
    assert '#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"' in node_cpp
    assert "docking_status_is_success" in docking_status_header
    assert "docking_status_is_failure" in docking_status_header
    assert "docking_status_is_undocking" in docking_status_header
    assert "docking_status_is_undocked" in docking_status_header
    assert "docking_status_is_undock_failed" in docking_status_header
    assert "docking_status_is_stopped" in docking_status_header
    assert 'starts_with(code, "docked")' in docking_status_cpp
    assert 'starts_with(code, "charging")' in docking_status_cpp
    assert 'starts_with(leading_status_code(status), "undocking")' in docking_status_cpp
    assert 'starts_with(leading_status_code(status), "undocked")' in docking_status_cpp
    assert 'starts_with(leading_status_code(status), "undock_failed")' in docking_status_cpp
    assert 'starts_with(leading_status_code(status), "stopped")' in docking_status_cpp
    assert "bool docking_status_is_success(const std::string & status) const" not in composition_cpp
    assert "bool docking_status_is_failure(const std::string & status) const" not in composition_cpp
    assert "bool docking_status_is_undocking(const std::string & status) const" not in composition_cpp
    assert "bool docking_status_is_undocked(const std::string & status) const" not in composition_cpp
    assert "bool docking_status_is_stopped(const std::string & status) const" not in composition_cpp
    assert "configure_runtime_permissions" in node_cpp
    assert "robot_api_server refuses to run as root" not in composition_cpp
    assert "NJRH_ALLOW_ROOT_API_SERVER" not in composition_cpp
    assert "::umask(0002)" in node_cpp
    assert "geometry_msgs::msg::Twist" in node_cpp
    assert "sensor_msgs::msg::BatteryState" in node_cpp
    assert "sensor_msgs::msg::LaserScan" in docking_runtime_cpp
    assert "sensor_msgs::msg::LaserScan" not in composition_cpp
    assert "tf2_msgs::msg::TFMessage" in node_cpp
    assert "/ws/v1/teleop" in teleop_module_cpp
    assert "/cmd_vel_api" in teleop_configuration_cpp
    assert "std_msgs::msg::Bool" in teleop_module_cpp
    assert "robot_interfaces::srv::SwitchFloor" in node_cpp
    assert "robot_interfaces::srv::TriggerLocalization" in node_cpp
    assert "geometry_msgs::msg::PoseWithCovarianceStamped" in node_cpp
    assert "/api/v1/status" in system_status_module_cpp
    assert "create_subscription<sensor_msgs::msg::BatteryState>" not in composition_cpp
    assert "teleop_stop_on_charging" in teleop_configuration_cpp
    assert "class TeleopConfigurationModule" in teleop_configuration_header
    assert "TeleopConfigurationModule::declare_parameters" in node_cpp
    assert "teleop_charging_current_min_a" in power_configuration_cpp
    assert "bms_charging_contact_voltage_min_v" in power_configuration_cpp
    assert "bms_charging_contact_voltage_max_v" in power_configuration_cpp
    assert "bms_full_soc_threshold_pct" in power_configuration_cpp
    assert "bms_full_soc_voltage_contact_enable" in power_configuration_cpp
    assert "battery_indicates_charging" not in composition_cpp
    assert "battery_charging_contact" not in composition_cpp
    assert '#include "robot_api_server/features/power/bms_contact.hpp"' in node_cpp
    assert '#include "robot_api_server/features/power/power_configuration_module.hpp"' in node_cpp
    assert '#include "robot_api_server/features/power/power_module.hpp"' in node_cpp
    assert "BatteryContactEvaluation" in bms_header
    assert "evaluate_battery_charging_contact" in bms_cpp
    assert "POWER_SUPPLY_STATUS_CHARGING" in bms_cpp
    assert "POWER_SUPPLY_STATUS_FULL" in bms_cpp
    assert "present_voltage_valid" in bms_cpp
    assert "full_soc_present_voltage_valid" in bms_cpp
    assert "current_above_threshold" in bms_cpp
    assert "class PowerModule" in power_module_header
    assert "class PowerConfigurationModule" in power_configuration_header
    assert "PowerConfigurationModule::declare_parameters" in node_cpp
    assert "create_subscription<sensor_msgs::msg::BatteryState>" in power_module_cpp
    assert "dependencies_.power->charging_contact_active()" in node_cpp
    assert "charging_guard_active" in teleop_module_cpp
    assert "charging detected; teleop command stopped" in teleop_module_cpp
    assert "\\\"bms\\\"" in system_status_module_cpp
    assert "\\\"soc\\\"" in system_status_module_cpp
    assert "\\\"soc_valid\\\"" in system_status_module_cpp
    assert "\\\"power_supply_status\\\"" in system_status_module_cpp
    assert "\\\"power_supply_health\\\"" in system_status_module_cpp
    assert "\\\"power_supply_technology\\\"" in system_status_module_cpp
    assert "\\\"present\\\"" in system_status_module_cpp
    assert "\\\"charging_contact\\\"" in system_status_module_cpp
    assert "\\\"charging_contact_reason\\\"" in system_status_module_cpp
    assert "\\\"inferred_docked\\\"" in system_status_module_cpp
    assert '#include "robot_api_server/features/system_status/robot_pose_model.hpp"' in node_cpp
    assert "struct RobotPoseSnapshot" in robot_pose_header
    assert "struct RobotPoseMapIdentity" in robot_pose_header
    assert "no_fresh_map_robot_pose_json" in robot_pose_header
    assert "robot_pose_json" in robot_pose_header
    assert "std::string no_fresh_map_robot_pose_json" in robot_pose_cpp
    assert "std::string robot_pose_json" in robot_pose_cpp
    assert "no fresh map-frame robot pose" in robot_pose_cpp
    assert "\\\"age_sec\\\":null" in robot_pose_cpp
    assert "\\\"map_id\\\":" in robot_pose_cpp
    assert "struct RobotPoseSnapshot" not in composition_cpp
    assert "std::string no_fresh_map_robot_pose_json()" not in composition_cpp
    assert "std::string no_fresh_map_robot_pose_json(const std::string & detail)" not in composition_cpp
    assert "class SystemStatusModule" in system_status_module_header
    assert "struct SystemStatusWiringDependencies" in system_status_wiring_header
    assert "make_system_status_module_ports" in system_status_wiring_cpp
    assert "/api/v1/robot/pose" in system_status_module_cpp
    assert "handle_robot_pose" in system_status_module_cpp
    assert "handle_robot_pose" not in composition_cpp
    assert "wait_for_current_robot_pose" in node_cpp
    assert "robot_pose_freshness_sec" in node_cpp
    assert "TfChainFreshnessSnapshot" in node_cpp
    assert "tf_chain_freshness_sec" in node_cpp
    assert "tf_chain_settle_timeout_sec" in node_cpp
    assert "wait_for_fresh_tf_chain" in node_cpp
    assert '"navigation goal"' in node_cpp
    assert '"docking predock navigation"' in node_cpp
    assert '"docking fine docking"' in node_cpp
    assert '#include "robot_api_server/features/localization/localization_result_model.hpp"' in node_cpp
    assert "localization_result_topic" in node_cpp
    assert "handle_localization_result" in node_cpp
    assert "trigger_localization_and_wait_for_result" in node_cpp
    assert "struct LocalizationResultSnapshot" in localization_result_header
    assert "localization_result_success_detail" in localization_result_header
    assert "localization_result_wait_failure_detail" in localization_result_header
    assert "localization_result_recent_fallback_detail" in localization_result_header
    assert "localization_result seq=" in localization_result_cpp
    assert "no new map-frame " in localization_result_cpp
    assert "because no newer result arrived after trigger" in localization_result_cpp
    assert "struct LocalizationResultSnapshot" not in composition_cpp
    assert '#include "lifecycle_msgs/srv/get_state.hpp"' in node_cpp
    assert "navigation_relocalize_before_goal" in node_cpp
    assert "navigation_relocalize_before_goal_always" in node_cpp
    assert "navigation_relocalize_before_goal_required" in node_cpp
    assert "navigation_relocalize_wait_sec" in node_cpp
    assert "force_pre_navigation_relocalization" in node_cpp
    assert "force_relocalize" in node_cpp
    assert "goal_admission_snapshot" in node_cpp
    assert "target floor differs from confirmed runtime map context" not in composition_cpp
    assert "requested pose target does not match confirmed runtime map context" in node_cpp
    assert "if (navigation_relocalize_before_goal_ && !pre_navigation_undock)" not in composition_cpp
    assert "navigation_lifecycle_check_timeout_sec" in node_cpp
    assert "lifecycle_snapshot" in node_cpp
    assert "navigation lifecycle inactive" in node_cpp
    assert "force_relocalize is no longer executed inside normal navigation goals" in node_cpp
    assert 'bridge_safe_for_goal_start("navigation goal"' in node_cpp
    assert "pre_navigation_relocalization_requested" in node_cpp
    assert "pre_navigation_relocalization_succeeded" in node_cpp
    assert "docking_relocalize_before_predock" in node_cpp
    assert "docking_relocalize_after_predock" in node_cpp
    assert "docking_relocalize_after_predock_required" in node_cpp
    assert "docking_relocalize_after_fine_docking" in node_cpp
    assert "docking_validate_predock_pose_after_relocalization" in node_cpp
    assert "docking_predock_pose_max_distance_m" in node_cpp
    assert "docking_predock_pose_max_yaw_rad" in node_cpp
    assert "docking_manual_predock_distance_check_enable" in node_cpp
    assert "docking_manual_predock_min_distance_m" in node_cpp
    assert "docking_manual_predock_max_distance_m" in node_cpp
    assert "docking_manual_predock_max_yaw_error_rad" in node_cpp
    assert "predock_pose_resolver_->validate" in node_cpp
    assert "manual predock pose" in docking_predock_pose_resolver_cpp
    assert "failed docking sanity check" in docking_predock_pose_resolver_cpp
    assert "distance_check=disabled" in docking_predock_pose_resolver_cpp
    assert "heading aligned to the charger" in docking_predock_pose_resolver_cpp
    assert "docking_relocalize_recent_result_max_age_sec" in node_cpp
    assert "undock_relocalize_after_success" in node_cpp
    assert "undock_relocalize_wait_sec" in node_cpp
    assert "localization_trigger_service_timeout_sec" in node_cpp
    assert "localization_bridge_acceptance_timeout_sec" in node_cpp
    assert "localization_bridge_acceptance_max_distance_m" in node_cpp
    assert "localization_bridge_acceptance_max_yaw_rad" in node_cpp
    assert "wait_for_localization_bridge_acceptance" in node_cpp
    assert "localization_result not accepted by map->odom bridge" in node_cpp
    assert "relocalize_after_predock" in node_cpp
    assert "docking_after_predock:" in node_cpp
    assert "relocalize after predock failed before fine docking" in node_cpp
    assert "post-predock localization pose is outside approach tolerance" in node_cpp
    assert "predock_pose_id" in node_cpp
    assert "approach_pose_id" in node_cpp
    assert "manual_predock_explicit" in docking_predock_pose_resolver_cpp
    assert "manual_predock_auto_id" in docking_predock_pose_resolver_cpp
    assert "manual_predock_auto_unique_type" in docking_predock_pose_resolver_cpp
    assert "dock_predock" in docking_predock_pose_resolver_cpp
    assert "dock_approach" in docking_predock_pose_resolver_cpp
    assert "computed_from_dock_pose" in node_cpp
    assert "multiple dock_predock poses found" in docking_predock_pose_resolver_cpp
    assert "relocalize_after_fine_docking" in node_cpp
    assert "docking_after_fine:" in docking_status_module_cpp
    assert "relocalized after fine docking" in docking_status_module_cpp
    assert 'starts_with(code, "not_found_")' in docking_status_cpp
    assert 'code.find("_not_found")' in docking_status_cpp
    assert "post_undock_relocalization_requested" in docking_status_module_cpp
    assert "post_undock_relocalization_succeeded" in docking_status_module_cpp
    assert "post_predock_relocalization_requested" in node_cpp
    assert "post_predock_relocalization_succeeded" in node_cpp
    assert "docking_cancel_active_goal_before_predock" in node_cpp
    assert "docking canceled before GS2 fine docking start" in node_cpp
    assert "docking_stop_service_wait_sec" in node_cpp
    assert "stop_client_->wait_for_service(stop_service_wait())" in node_cpp
    assert "runtime_map_context_file" in node_cpp
    assert "confirmed_runtime_map_manifest" in node_cpp
    assert "unique_active_map_manifest" in node_cpp
    assert "runtime map context is not confirmed yet" in node_cpp
    assert "navigation or docking is active but no runtime map context is recorded" in node_cpp
    assert "child_frame_id" in node_cpp
    assert "latest_pose_stamp_sec_" in node_cpp
    assert "/api/v1/maps" in node_cpp
    assert "/api/v1/mapping/2d/start" in node_cpp
    assert "/api/v1/mapping/2d/stop" in node_cpp
    assert "/api/v1/mapping/2d/save" in node_cpp
    assert "/api/v1/mapping/stop" in node_cpp
    assert "/api/v1/mapping/save" in node_cpp
    assert "/api/v1/maps/delete" in node_cpp
    assert "/api/v1/maps/semantic_layer" in node_cpp
    assert "handle_get_semantic_layer" in node_cpp
    assert "/api/v1/maps/poses" in node_cpp
    assert "handle_get_poses" in node_cpp
    assert "POST /api/v1/maps/poses" in node_cpp
    assert "PUT /api/v1/maps/poses/{pose_id}" in node_cpp
    assert "DELETE /api/v1/maps/poses/{pose_id}" in node_cpp
    assert "PUT /api/v1/maps/poses/batch" in node_cpp
    assert "/api/v1/maps/poses/save" in node_cpp
    assert "/api/v1/maps/poses/save_current" in node_cpp
    assert "handle_save_current_pose" in node_cpp
    assert "current_robot_pose" in node_cpp
    assert "handle_delete_pose" in node_cpp
    assert "handle_replace_poses_batch" in node_cpp
    assert "json_object_array_value" in node_cpp
    assert "/api/v1/maps/filters/keepout" in node_cpp
    assert "handle_get_keepout_filter" in node_cpp
    assert "/api/v1/maps/filters/keepout/save" in node_cpp
    assert "handle_save_keepout_filter" in node_cpp
    assert "keepout_semantic_layer.json" in semantic_cpp
    assert "/api/v1/navigation/goal" in node_cpp
    assert "/api/v1/navigation/cancel" in node_cpp
    assert "HttpResponse handle_goal" in navigation_module_cpp
    assert "HttpResponse handle_cancel" in navigation_module_cpp
    assert "NavigateToPose" in node_cpp
    assert "navigate_to_pose_action" in node_cpp
    assert "std::mutex action_mutex_;" in node_cpp
    assert "GoalHandle::SharedPtr active_goal_handle_;" in node_cpp
    assert '#include "robot_api_server/features/navigation/mission/navigation_goal_job.hpp"' in node_cpp
    assert "struct NavigationGoalJob" not in composition_cpp
    assert "mission_runtime_.json()" in node_cpp
    assert "navigation_goal_id" in node_cpp
    assert "NavigationGoalExecutor::run" in goal_executor_cpp
    assert "run_final_yaw_align" in node_cpp
    assert "runtime.terminal_publish_command(command)" in node_cpp
    assert "geometry_msgs::msg::Twist>::SharedPtr command_pub_" in node_cpp
    assert "navigation_goal_position_success_tolerance_m" in node_cpp
    assert "navigation_final_yaw_align_enable" in node_cpp
    assert "navigation_final_yaw_align_trigger_rad" in node_cpp
    assert "navigation_final_yaw_align_kp" in node_cpp
    assert "navigation_final_yaw_align_max_xy_drift_m" in node_cpp
    assert "navigation_final_yaw_align_cmd_topic" in node_cpp
    assert 'final_yaw_align_cmd_topic != "/cmd_vel_nav"' in node_cpp
    assert 'final_yaw_align_cmd_topic != "/cmd_vel_api"' in node_cpp
    assert 'final_yaw_align_cmd_topic = "/cmd_vel_api"' in node_cpp
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe"' not in composition_cpp
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel"' not in composition_cpp
    assert "final_pose_auditing" in goal_executor_cpp
    assert "position_reached_yaw_aligning" in goal_executor_cpp
    assert "final_pose_verifying" in navigation_terminal_cpp
    assert "final_pose_verified" in goal_executor_cpp
    assert "failed_final_pose_verify" in goal_executor_cpp
    assert "blocked_by_safety" in navigation_terminal_cpp
    assert "goal_completion_policy" in node_cpp
    assert "position_only" in node_cpp
    assert "pose_required" in node_cpp
    assert "dock_staging" in node_cpp
    assert "task_complete" in node_cpp
    assert "yaw_align_required" in node_cpp
    assert "yaw_align_active" in node_cpp
    assert "yaw_align_failed" in node_cpp
    assert "REPOSITION_AFTER_YAW_DRIFT" in node_cpp
    assert "run_reposition_after_yaw_drift" in node_cpp
    assert "navigation goal reached by commercial final verification" in goal_executor_cpp
    assert "commercial_final_verify=true" in goal_executor_cpp
    assert "degraded_final_pose_verify" in goal_executor_cpp
    assert "under position_only policy" in goal_executor_cpp
    assert "bool position_reached = pose_check.position_reached;" in goal_executor_cpp
    assert "bool position_reached = nav2_succeeded" not in goal_executor_cpp
    run_goal_block = goal_executor_cpp
    assert "Nav2 reported success but read-only final pose verification failed" not in run_goal_block
    assert '"failed_final_pose_verify"' in goal_executor_cpp
    assert '"nav2_failed"' not in run_goal_block
    assert '"position_not_reached"' not in goal_executor_cpp
    assert "final_yaw_align_blocked" in navigation_goal_job_cpp
    assert "final_yaw_align_attempted" in navigation_goal_job_cpp
    assert "final_yaw_align_blocked_reason" in navigation_goal_job_cpp
    assert "final_yaw_align_duration_sec" in navigation_goal_job_cpp
    assert "final_yaw_align_target_yaw_rad" in navigation_goal_job_cpp
    assert "final_yaw_align_initial_yaw_error_rad" in navigation_goal_job_cpp
    assert "final_yaw_align_final_yaw_error_rad" in navigation_goal_job_cpp
    assert "final_yaw_align_observed_xy_drift_m" in navigation_goal_job_cpp
    assert "final_yaw_align_bypass_collision_monitor" in navigation_goal_job_cpp
    assert "final_pose_verify_reason" in navigation_goal_job_cpp
    assert "request_navigation_goal_cancel" in node_cpp
    assert "navigation_goal_cancel_requested" in node_cpp
    assert "exception sending navigation goal" in node_cpp
    assert "async_cancel_goal" in node_cpp
    assert "async_cancel_all_goals" in node_cpp
    assert "cancel_goal_and_prove_terminal" in node_cpp
    assert "exact goal terminal state is unknown" in node_cpp
    assert "cancel_all_goals_with_evidence" in node_cpp
    assert "late cancel-all outcome is unknown" in node_cpp
    assert "navigation_cancel_action_wait_sec" in node_cpp
    assert "cancel_action_wait_" in node_cpp
    assert "unhandled API exception" in http_server_cpp
    assert "log_http_request" in api_gateway_cpp
    assert "log_http_request" not in composition_cpp
    assert "::listen(local_server_fd, 64)" in http_server_cpp
    assert "SO_RCVTIMEO" in http_server_cpp
    assert "SO_SNDTIMEO" in http_server_cpp
    assert "failed to send full HTTP response" in http_server_cpp
    assert "Taking data from action client but no ready event" in node_cpp
    assert "continuing after transient action client executor exception" in node_cpp
    assert "SingleThreadedExecutor executor" in node_cpp
    assert "set_close_on_exec(local_server_fd)" in http_server_cpp
    assert "prepare_child_process(" in node_cpp
    assert "timed out waiting for navigation stop command" in node_cpp
    assert "teleop->publish_zero_burst()" in node_cpp
    assert "zero_velocity_published" in node_cpp
    assert "action_available" in node_cpp
    assert "cancel_all_detail" in node_cpp
    assert "navigation_stop_command" in node_cpp
    assert "stop_runtime_stack" in node_cpp
    assert "cancel_requested" in node_cpp
    assert "navigation_stack_stopped" in navigation_cancel_cpp
    assert "stop_stack" in node_cpp
    assert "read_floor_poses" in node_cpp
    assert "write_floor_poses" in node_cpp
    assert "/api/v1/subscriptions/acquire" in subscription_module_cpp
    assert "/api/v1/subscriptions/release" in subscription_module_cpp
    assert "/api/v1/subscriptions/heartbeat" in subscription_module_cpp
    assert "set_resource_active" in subscription_module_cpp
    assert "Status and safety/floor inputs are process-resident" in subscription_module_cpp
    assert "subscriptions->handle_http(request)" in node_cpp
    assert "status->ensure_floor_status_subscription_active();" in node_cpp
    assert "create_subscription<std_msgs::msg::String>" in system_status_module_cpp
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()" in node_cpp
    assert "refresh_runtime_state(false);" in node_cpp
    assert 'transition_owner() != "mapping_start"' in node_cpp
    assert "runtime.mapping_active ||" in node_cpp
    assert "void refresh_runtime_state(bool probe_lifecycle = false);" in node_cpp
    assert "worker_loop" in http_server_cpp
    assert "client_queue_" in http_server_cpp
    assert ".detach()" not in (node_cpp + http_server_cpp)
    assert "live_map resource is not acquired" in mapping_module_cpp
    assert "ports_.release_subscriptions(websocket_client_id" in teleop_module_cpp
    assert "subscription_client_id_from_body" in subscription_module_cpp
    assert "subscription_client_id_from_body" not in composition_cpp
    assert "clientId" in subscription_api_cpp
    assert "lease_id" in subscription_module_cpp
    assert "http:compat-default" in subscription_api_cpp
    assert "resources_for_client(client_id)" in subscription_module_cpp
    assert "\\\"refreshed\\\":false" in subscription_module_cpp
    assert "/api/v1/mapping/2d/map" in mapping_module_cpp
    assert "image/png" in mapping_module_cpp
    assert "nav_msgs::msg::OccupancyGrid" in mapping_module_cpp
    assert "encode_grayscale_png" in mapping_module_cpp
    assert "handle_start_mapping_2d" in mapping_module_cpp
    assert "handle_stop_mapping_2d" in mapping_module_cpp
    assert "handle_save_mapping_2d" in mapping_module_cpp
    assert "ports_.clear_runtime_map_context();" in mapping_module_cpp
    start_mapping_section = mapping_module_cpp[
        mapping_module_cpp.index("HttpResponse handle_start_mapping_2d("):
        mapping_module_cpp.index("HttpResponse handle_stop_mapping_2d()")
    ]
    assert "ports_.cancel_navigation" not in start_mapping_section
    assert "ports_.stop_navigation_runtime" not in start_mapping_section
    assert "run_mapping_start_job_guarded" in start_mapping_section
    assert "start_worker_" in start_mapping_section
    assert "cannot start 2D mapping while navigation runtime is active; stop navigation runtime first" not in start_mapping_section
    mapping_start_worker_section = mapping_module_cpp[
        mapping_module_cpp.index("void run_mapping_start_job("):
        mapping_module_cpp.index("void run_mapping_start_job_guarded(")
    ]
    assert "ports_.cancel_navigation()" in mapping_start_worker_section
    assert "ports_.stop_navigation_runtime()" in mapping_start_worker_section
    assert "ports_.clear_runtime_map_context();" in mapping_start_worker_section
    assert "process_runtime_.start" in mapping_start_worker_section
    assert mapping_start_worker_section.index("ports_.cancel_navigation()") < mapping_start_worker_section.index("ports_.stop_navigation_runtime()")
    assert mapping_start_worker_section.index("ports_.stop_navigation_runtime()") < mapping_start_worker_section.index("ports_.clear_runtime_map_context();")
    assert mapping_start_worker_section.index("ports_.clear_runtime_map_context();") < mapping_start_worker_section.index("process_runtime_.start")
    assert "navigation goals are unavailable while 2D mapping is active or starting" in node_cpp
    assert "navigation cancel/stop is unavailable while 2D mapping is active or starting" in node_cpp
    assert "docking is unavailable while 2D mapping is active or starting" in node_cpp
    assert "start_job_.json()" in mapping_module_cpp
    assert "cancel_task_for_mode_switch" in node_cpp
    assert "navigation->module().cancel_task_for_mode_switch" in node_cpp
    assert "requires_manual_navigation_selection" in mapping_module_cpp
    assert "write_last_navigation_map_selection" in node_cpp
    assert "njrh.last_navigation_map.v1" in node_cpp
    assert "2D map saved and activated" not in composition_cpp
    assert "handle_delete_map" in node_cpp
    assert "runtime_map_asset_paths" in runtime_map_lookup_cpp
    assert "MapManifest" in node_cpp
    assert "generate_map_id" in map_catalog_cpp
    assert "display_name" in node_cpp
    assert "floor_maps" in node_cpp
    assert "map_info" in node_cpp
    assert "read_nav_map_info" in node_cpp
    assert "read_pgm_dimensions" not in composition_cpp
    assert "manifest.json" in node_cpp
    assert "maps/<map_id>" not in composition_cpp
    assert "\"maps\"" in map_catalog_cpp
    assert "\"current\"" in map_catalog_cpp
    assert "activate_map_manifest" in node_cpp
    assert "remove_current_map_entry" in node_cpp
    assert "refusing unsafe current map reset path" in node_cpp
    assert ".stale_current_" in node_cpp
    assert "quarantined stale current map entry" in node_cpp
    assert "delete by map_id only" in node_cpp
    assert "ensure_legacy_floor_map_manifest" in node_cpp
    assert "legacy_" in node_cpp
    assert "write_neutral_filter_assets" in map_asset_writer_cpp
    assert "robot_api_server_slam_toolbox_save" in map_asset_writer_cpp
    assert "MappingProcessRuntime::terminate_process_groups_locked" in mapping_runtime_cpp
    assert "discover_mapping_process_groups" in mapping_runtime_cpp
    assert "terminate_residual_processes" in mapping_runtime_cpp
    assert "discover_mapping_residual_processes" in mapping_runtime_cpp
    assert "mapping_was_active" in mapping_module_cpp
    assert "stopped_groups" in mapping_module_cpp
    stop_mapping_section = mapping_module_cpp[
        mapping_module_cpp.index("HttpResponse handle_stop_mapping_2d()"):
        mapping_module_cpp.index("void refresh_mapping_2d_runtime_state()")
    ]
    assert 'set_mapping_runtime_state(true, "stopping"' not in stop_mapping_section
    mapping_residual_section = mapping_runtime_cpp[
        mapping_runtime_cpp.index("bool is_mapping_2d_residual_process_command"):
        mapping_runtime_cpp.index("bool is_safe_mapping_lidar_rps_xps_path")
    ]
    assert '"fastlio_mapping_odom_bridge.py"' in mapping_residual_section
    assert '"/mapping/fastlio_odometry"' in mapping_residual_section
    assert '"/tf_slam2d"' in mapping_residual_section
    assert "return is_private_slam2d_fastlio_process(cmdline, environ);" in mapping_residual_section
    private_fastlio_section = mapping_runtime_cpp[
        mapping_runtime_cpp.index("bool is_private_slam2d_fastlio_process"):
        mapping_runtime_cpp.index("bool is_mapping_2d_residual_process_command")
    ]
    assert '"ros2 run fast_lio fastlio_mapping"' in private_fastlio_section
    assert '"fast_lio/lib/fast_lio/fastlio_mapping"' in private_fastlio_section
    assert '"fastlio_mapping --ros-args"' in private_fastlio_section
    assert "NJRH_SLAM2D_PRIVATE_FASTLIO=1" in private_fastlio_section
    assert '"fastlio_odom_bridge_node"' not in mapping_residual_section
    assert 'cmdline.find("nav_cloud_preprocessor")' in mapping_residual_section
    assert 'cmdline.find("pointcloud_to_laserscan_node")' in mapping_residual_section
    assert 'environ.find("NJRH_SLAM2D_MAPPING_PIPELINE=1")' in mapping_residual_section
    assert '"scan_republisher_node"' not in mapping_residual_section
    assert "mapping_2d_start_command" in node_cpp
    assert "navigation_resume_command" in node_cpp
    assert "handle_resume_floor_navigation" in node_cpp
    assert "run_navigation_runtime_services.sh" in node_cpp
    assert "run_projected_map.sh" in node_cpp
    assert "jt128_slam_toolbox_mapping.launch.py" in mapping_runtime_cpp
    assert "run_cancel_job_guarded" in navigation_module_cpp
    assert "navigation cancel worker exception" in navigation_module_cpp
    assert "job_executor_->run_guarded(job_id)" in node_cpp
    assert "docking worker exception" in node_cpp
    assert "live slam_toolbox /map" in mapping_module_cpp
    assert "newest_png_in_directory" in runtime_map_lookup_cpp
    assert "resolve_mapping_2d_png" in runtime_map_lookup_cpp
    assert "/api/v1/safety/stop" not in composition_cpp
    assert "/api/v1/safety/resume" not in composition_cpp
    assert "/api/v1/safety/stop" in safety_module_cpp
    assert "/api/v1/safety/resume" in safety_module_cpp
    assert "/api/v1/floors/switch" in node_cpp
    assert "/api/v1/localization/trigger" in node_cpp
    assert "endpoint is reserved but not wired to a ROS-native service/action yet" in application_router_cpp
    assert "api_token" in config
    assert "max_http_connections: 16" in config
    assert "max_http_connections: 16" in overlay_config
    assert "bms_state_topic: \"/battery_state\"" in config
    assert "bms_state_max_age_sec: 3.0" in config
    assert "teleop_stop_on_charging: true" in config
    assert "teleop_charging_current_min_a: 0.10" in config
    assert "bms_charging_contact_voltage_min_v: 40.0" in config
    assert "bms_charging_contact_voltage_max_v: 1000.0" in config
    assert "bms_full_soc_threshold_pct: 99.0" in config
    assert "bms_full_soc_voltage_contact_enable: true" in config
    assert "robot_pose_freshness_sec: 0.5" in config
    assert "tf_chain_freshness_sec: 0.30" in config
    assert "tf_chain_settle_timeout_sec: 2.0" in config
    assert "localization_result_topic: \"/localization_result\"" in config
    assert "navigation_relocalize_before_goal: false" in config
    assert "navigation_relocalize_before_goal_always: false" in config
    assert "navigation_relocalize_before_goal_required: false" in config
    assert "navigation_relocalize_wait_sec: 8.0" in config
    assert "navigation_lifecycle_check_timeout_sec: 1.5" in config
    assert "navigation_goal_result_timeout_sec: 600.0" in config
    assert "navigation_goal_position_success_tolerance_m: 0.06" in config
    assert "navigation_default_goal_completion_policy: \"pose_required\"" in config
    assert "navigation_delivery_point_goal_completion_policy: \"pose_required\"" in config
    assert "navigation_position_only_nav2_yaw_mode: \"approach_heading\"" in config
    assert "navigation_position_only_approach_heading_min_distance_m: 0.20" in config
    assert "navigation_max_reposition_after_yaw_retry: 0" in config
    assert "navigation_reposition_after_yaw_drift_timeout_sec: 30.0" in config
    assert "nav2_native_goal_completion_enabled: true" in config
    assert "nav2_rotation_shim_enabled: true" in config
    assert "api_final_yaw_align_fallback_enabled: true" in config
    assert "navigation_final_yaw_align_enable: true" in config
    assert "navigation_final_yaw_tolerance_rad: 0.05" in config
    assert "navigation_final_yaw_align_trigger_rad: 0.08" in config
    assert "navigation_final_yaw_align_success_tolerance_rad: 0.045" in config
    assert "navigation_final_yaw_align_speed_radps: 0.60" in config
    assert "navigation_final_yaw_align_min_speed_radps: 0.06" in config
    assert "navigation_final_yaw_align_kp: 1.2" in config
    assert "navigation_final_yaw_align_max_speed_radps: 0.60" in config
    assert "navigation_final_yaw_align_timeout_sec: 8.0" in config
    assert "navigation_final_yaw_align_max_xy_drift_m: 0.08" in config
    assert "navigation_final_yaw_align_require_fresh_pose: true" in config
    assert "navigation_final_yaw_align_cmd_topic: \"/cmd_vel_api\"" in config
    assert "navigation_final_yaw_align_cmd_topic: \"/cmd_vel_collision_checked\"" not in config
    assert "navigation_final_yaw_align_bypass_collision_monitor: true" in config
    assert "navigation_final_yaw_align_zero_cmd_count: 3" in config
    assert "navigation_pause_global_correction_during_final_yaw: true" in config
    assert "yaw_align_stop_lead_enabled: true" in config
    assert "yaw_align_stop_lead_time_sec: 0.125" in config
    assert "yaw_align_stop_lead_max_rad: 0.09" in config
    assert "docking_relocalize_before_predock: false" in config
    assert "docking_relocalize_after_predock: false" in config
    assert "docking_relocalize_after_predock_required: false" in config
    assert "docking_relocalize_after_fine_docking: false" in config
    assert "docking_relocalize_after_fine_docking_required: false" in config
    assert "docking_validate_predock_pose_after_relocalization: true" in config
    assert "docking_predock_pose_max_distance_m: 0.30" in config
    assert "docking_predock_pose_max_yaw_rad: 0.35" in config
    assert "docking_manual_predock_distance_check_enable: false" in config
    assert "docking_manual_predock_min_distance_m: 0.50" in config
    assert "docking_manual_predock_max_distance_m: 1.20" in config
    assert "docking_manual_predock_max_yaw_error_rad: 0.80" in config
    assert "docking_relocalize_wait_sec: 8.0" in config
    assert "docking_relocalize_recent_result_max_age_sec: 5.0" in config
    assert "localization_trigger_service_timeout_sec: 15.0" in config
    assert "localization_bridge_acceptance_timeout_sec: 3.0" in config
    assert "localization_bridge_acceptance_max_distance_m: 1.0" in config
    assert "localization_bridge_acceptance_max_yaw_rad: 0.35" in config
    assert "navigation_cancel_action_wait_sec: 0.75" in config
    assert "docking_stop_service_wait_sec: 3.0" in config
    assert "undock_relocalize_after_success: true" in config
    assert "undock_relocalize_wait_sec: 8.0" in config
    assert "docking_cancel_active_goal_before_predock: true" in config
    assert "runtime_map_context_file: \"/tmp/njrh_runtime_map_context.json\"" in config
    assert "last_navigation_map_file: \"/workspaces/njrh-v3/workspace1/maps_release/last_navigation_map.json\"" in config
    assert "navigate_to_pose_action: \"/navigate_to_pose\"" in config
    assert "mapping_2d_start_command" in config
    assert "run_projected_map.sh" in config
    assert "navigation_resume_command" in config
    assert "run_navigation_runtime_services.sh" in config
    assert "docking_undock_service: \"/docking/undock\"" in config
    assert "docking_pre_dock_distance_m: 0.60" in config
    assert "navigation_auto_undock_timeout_sec: 28.0" in config
    assert 'declare_parameter<double>("navigation_auto_undock_timeout_sec", 28.0)' in node_cpp
    assert "docking_undock_charging_retry_sec: 3.0" in config
    assert "mapping_2d_live_map_topic: \"/map\"" in config
    assert "refresh_mapping_2d_runtime_state();" in mapping_module_cpp
    assert "MappingProcessRuntime::recover_process_locked" in mapping_runtime_cpp
    assert "set_mapping_live_map_cache_active(true)" in mapping_module_cpp
    assert "live_map_mapping_cache_active_" in mapping_module_cpp
    assert "live_map_page_subscription_active_" in mapping_module_cpp
    assert "2D mapping process discovered" in mapping_module_cpp
    assert "\\\"live_map_available\\\"" in mapping_module_cpp
    assert "\\\"live_map_age_sec\\\"" in mapping_module_cpp
    assert "scan_topic: \"/scan\"" in config
    assert "tf_topic: \"/tf\"" in config
    assert "subscription_default_ttl_ms: 10000" in config
    assert "teleop_cmd_topic: \"/cmd_vel_api\"" in config
    assert "teleop_cmd_topic: \"/cmd_vel_collision_checked\"" not in config
    assert "teleop_reverse_enable_topic: \"/ranger_mini3/teleop_allow_reverse\"" in config
    assert "teleop_pose_topic: \"/local_state/odometry\"" in config
    assert "teleop_max_linear_x_mps: 1.00" in config
    assert "teleop_allow_reverse: true" in config
    assert "teleop_require_mapping_active: true" in config
    assert "teleop_watchdog_timeout_sec: 0.5" in config
    assert "teleop_socket_idle_timeout_sec: 5.0" in config
    assert "teleop_repeat_rate_hz: 20.0" in config
    assert "on_repeat_timer" in teleop_module_cpp
    assert "teleop_session_.session_active()" in teleop_module_cpp
    assert "active && config_.stop_on_charging && teleop_session_.session_active()" in teleop_module_cpp
    assert "if (!teleop_session_.session_active())" in teleop_module_cpp
    assert "store_teleop_command(twist)" in teleop_module_cpp
    assert "clear_command()" in teleop_module_cpp
    assert "set_socket_receive_timeout(client_fd, config_.socket_idle_timeout_sec)" in teleop_module_cpp
    assert "TeleopSessionState teleop_session_" in teleop_module_cpp
    assert "TeleopSessionState teleop_session_" not in composition_cpp
    assert "teleop_mutex_" not in composition_cpp
    assert "payload, \"linear\", \"x\"" in teleop_cpp
    assert "payload, \"angular\", \"z\"" in teleop_cpp
    assert "json_number_value(payload, \"linearX\")" in teleop_cpp
    assert "json_number_value(payload, \"angularZ\")" in teleop_cpp
    assert "class TeleopSessionState" in teleop_header
    assert "repeat_decision" in teleop_cpp
    assert "if (payload_length > 4096U)" in websocket_transport_cpp
    assert "if (masked && !recv_exact(" in websocket_transport_cpp
    assert "if (masked) {" in websocket_transport_cpp
    assert "if (!masked || payload_length > 4096U)" not in websocket_transport_cpp
    assert "read_websocket_frame" in websocket_transport_header
    assert "src/features/teleop/teleop_session.cpp" in cmake
    assert "src/features/teleop/teleop_configuration_module.cpp" in cmake
    assert "src/features/teleop/teleop_module.cpp" in cmake
    assert "src/infrastructure/http/api_gateway_configuration_module.cpp" in cmake
    assert "src/infrastructure/http/websocket_transport.cpp" in cmake
    assert "test_teleop_session" in cmake
    assert "test_teleop_configuration_module" in cmake
    assert "test_teleop_module" in cmake
    assert "test_websocket_transport" in cmake
    assert "class SafetyRosAdapter" in safety_adapter_header
    assert "Final command arbitration" in safety_adapter_header
    assert "reliable().transient_local()" in safety_adapter_cpp
    assert "evaluate_hard_block" in safety_state_header
    assert 'snapshot.status == "COMMAND_STALE"' in safety_state_cpp
    assert "class SafetyConfigurationModule" in safety_configuration_header
    assert "safety_estop_topic" in safety_configuration_cpp
    assert "class SafetyModule" in safety_module_header
    assert "dependencies_.safety->hard_blocked(detail)" in node_cpp
    assert "SafetyRosAdapterOptions safety_options" not in composition_cpp
    assert "safety_adapter_->hard_block_decision()" not in composition_cpp
    assert "latest_safety_status_" not in composition_cpp
    assert "latest_motion_allowed_" not in composition_cpp
    assert "src/features/safety/safety_configuration_module.cpp" in cmake
    assert "src/features/safety/safety_module.cpp" in cmake
    assert "src/features/safety/safety_ros_adapter.cpp" in cmake
    assert "src/features/safety/safety_state.cpp" in cmake
    assert "test_safety_configuration_module" in cmake
    assert "test_safety_module" in cmake
    assert "test_safety_state" in cmake
    assert "test_safety_ros_adapter" in cmake
    assert "/safety/estop" in config
    assert "/floor_manager/switch_floor" in config
    assert "/global_localization/trigger" in config
    assert "robot_api_server_node" in launch
    assert "test web dashboard is not used" in readme
    assert "api_token" in overlay_config
    assert "bms_state_topic: \"/battery_state\"" in overlay_config
    assert "bms_state_max_age_sec: 3.0" in overlay_config
    assert "teleop_stop_on_charging: true" in overlay_config
    assert "teleop_charging_current_min_a: 0.10" in overlay_config
    assert "bms_charging_contact_voltage_min_v: 40.0" in overlay_config
    assert "bms_charging_contact_voltage_max_v: 1000.0" in overlay_config
    assert "bms_full_soc_threshold_pct: 99.0" in overlay_config
    assert "bms_full_soc_voltage_contact_enable: true" in overlay_config
    assert "robot_pose_freshness_sec: 0.5" in overlay_config
    assert "tf_chain_freshness_sec: 0.30" in overlay_config
    assert "tf_chain_settle_timeout_sec: 2.0" in overlay_config
    assert "navigation_relocalize_before_goal: false" in overlay_config
    assert "navigation_relocalize_before_goal_always: false" in overlay_config
    assert "navigation_relocalize_before_goal_required: false" in overlay_config
    assert "navigation_relocalize_wait_sec: 8.0" in overlay_config
    assert "navigation_lifecycle_check_timeout_sec: 1.5" in overlay_config
    assert "navigation_goal_result_timeout_sec: 600.0" in overlay_config
    assert "navigation_goal_position_success_tolerance_m: 0.06" in overlay_config
    assert "navigation_default_goal_completion_policy: \"pose_required\"" in overlay_config
    assert "navigation_delivery_point_goal_completion_policy: \"pose_required\"" in overlay_config
    assert "navigation_position_only_nav2_yaw_mode: \"approach_heading\"" in overlay_config
    assert "navigation_position_only_approach_heading_min_distance_m: 0.20" in overlay_config
    assert "navigation_max_reposition_after_yaw_retry: 0" in overlay_config
    assert "navigation_reposition_after_yaw_drift_timeout_sec: 30.0" in overlay_config
    assert "nav2_native_goal_completion_enabled: true" in overlay_config
    assert "nav2_rotation_shim_enabled: true" in overlay_config
    assert "api_final_yaw_align_fallback_enabled: true" in overlay_config
    assert "navigation_final_yaw_align_enable: true" in overlay_config
    assert "navigation_final_yaw_tolerance_rad: 0.05" in overlay_config
    assert "navigation_final_yaw_align_trigger_rad: 0.08" in overlay_config
    assert "navigation_final_yaw_align_success_tolerance_rad: 0.045" in overlay_config
    assert "navigation_final_yaw_align_speed_radps: 0.60" in overlay_config
    assert "navigation_final_yaw_align_min_speed_radps: 0.06" in overlay_config
    assert "navigation_final_yaw_align_kp: 1.2" in overlay_config
    assert "navigation_final_yaw_align_max_speed_radps: 0.60" in overlay_config
    assert "navigation_final_yaw_align_timeout_sec: 8.0" in overlay_config
    assert "navigation_final_yaw_align_max_xy_drift_m: 0.08" in overlay_config
    assert "navigation_final_yaw_align_require_fresh_pose: true" in overlay_config
    assert "navigation_final_yaw_align_cmd_topic: \"/cmd_vel_api\"" in overlay_config
    assert "navigation_final_yaw_align_cmd_topic: \"/cmd_vel_collision_checked\"" not in overlay_config
    assert "navigation_final_yaw_align_bypass_collision_monitor: true" in overlay_config
    assert "navigation_final_yaw_align_zero_cmd_count: 3" in overlay_config
    assert "navigation_pause_global_correction_during_final_yaw: true" in overlay_config
    assert "yaw_align_stop_lead_enabled: true" in overlay_config
    assert "yaw_align_stop_lead_time_sec: 0.125" in overlay_config
    assert "yaw_align_stop_lead_max_rad: 0.09" in overlay_config
    assert "localization_result_topic: \"/localization_result\"" in overlay_config
    assert "docking_relocalize_before_predock: false" in overlay_config
    assert "docking_relocalize_after_predock: false" in overlay_config
    assert "docking_relocalize_after_predock_required: false" in overlay_config
    assert "docking_relocalize_wait_sec: 8.0" in overlay_config
    assert "docking_relocalize_recent_result_max_age_sec: 5.0" in overlay_config
    assert "localization_trigger_service_timeout_sec: 15.0" in overlay_config
    assert "localization_bridge_acceptance_timeout_sec: 3.0" in overlay_config
    assert "localization_bridge_acceptance_max_distance_m: 1.0" in overlay_config
    assert "localization_bridge_acceptance_max_yaw_rad: 0.35" in overlay_config
    assert "navigation_cancel_action_wait_sec: 0.75" in overlay_config
    assert "docking_stop_service_wait_sec: 3.0" in overlay_config
    assert "undock_relocalize_after_success: true" in overlay_config
    assert "undock_relocalize_wait_sec: 8.0" in overlay_config
    assert "docking_cancel_active_goal_before_predock: true" in overlay_config
    assert "docking_manual_predock_distance_check_enable: false" in overlay_config
    assert "docking_manual_predock_min_distance_m: 0.50" in overlay_config
    assert "docking_manual_predock_max_distance_m: 1.20" in overlay_config
    assert "docking_manual_predock_max_yaw_error_rad: 0.80" in overlay_config
    assert "runtime_map_context_file: \"/tmp/njrh_runtime_map_context.json\"" in overlay_config
    assert "navigation_resume_starting_context_ttl_sec: 300.0" in overlay_config
    assert "navigation_resume_starting_context_ttl_sec: 300.0" in config
    assert "last_navigation_map_file: \"/workspaces/njrh-v3/workspace1/maps_release/last_navigation_map.json\"" in overlay_config
    assert "navigate_to_pose_action: \"/navigate_to_pose\"" in overlay_config
    assert "scan_topic: \"/scan\"" in overlay_config
    assert "tf_topic: \"/tf\"" in overlay_config
    assert "subscription_default_ttl_ms: 10000" in overlay_config
    assert "mapping_2d_start_command" in overlay_config
    assert "navigation_resume_command" in overlay_config
    assert "run_navigation_runtime_services.sh" in overlay_config
    assert "docking_undock_service: \"/docking/undock\"" in overlay_config
    assert "docking_pre_dock_distance_m: 0.60" in overlay_config
    assert "navigation_auto_undock_timeout_sec: 28.0" in overlay_config
    assert "docking_undock_charging_retry_sec: 3.0" in overlay_config
    assert "teleop_cmd_topic: \"/cmd_vel_api\"" in overlay_config
    assert "teleop_cmd_topic: \"/cmd_vel_collision_checked\"" not in overlay_config
    assert "teleop_reverse_enable_topic: \"/ranger_mini3/teleop_allow_reverse\"" in overlay_config
    assert "teleop_max_linear_x_mps: 1.00" in overlay_config
    assert "teleop_socket_idle_timeout_sec: 5.0" in overlay_config
    assert "teleop_repeat_rate_hz: 20.0" in overlay_config
    assert 'std::getenv("ROBOT_API_TOKEN")' in api_gateway_cpp
    assert 'std::getenv("ROBOT_API_TOKEN")' not in composition_cpp
    assert 'api_token:="${ROBOT_API_TOKEN}"' not in overlay_script
    assert "ROBOT_API_TOKEN" not in overlay_script
    assert '"-e" "ROBOT_API_TOKEN"' in systemd_runtime_script
    assert 'ROBOT_API_TOKEN=${ROBOT_API_TOKEN' not in systemd_runtime_script
    assert '-e "ROBOT_API_TOKEN"' in container_runtime_script
    assert "ROBOT_API_TOKEN='${ROBOT_API_TOKEN" not in container_runtime_script
    assert 'export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"' in overlay_script
    assert 'export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"' in overlay_script
    assert overlay_script.index("AMENT_TRACE_SETUP_FILES") < overlay_script.index('source "${SCRIPT_DIR}/common_env.sh"')
    assert "robot_api_server_node" in overlay_script
    assert "cleanup_stale_api_processes()" in overlay_supervisor_script
    assert 'pkill -TERM -f "${pattern}"' in overlay_supervisor_script
    assert 'pkill -KILL -f "${pattern}"' in overlay_supervisor_script
    assert "ros2 run robot_api_server robot_api_server_node" in overlay_supervisor_script
    assert "refusing isolated restart" in overlay_supervisor_script
    assert "ROBOT_API_SERVER_RESTART_DELAY_SEC" not in overlay_supervisor_script
    assert 'kill -TERM "${PPID}"' in overlay_supervisor_script
    assert "verify_robot_api_server_common_health_or_exit" in (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh"
    ).read_text(encoding="utf-8")
    assert (
        "colcon build --packages-select robot_map_asset_identity robot_interfaces "
        "robot_elevator_manager robot_api_server"
        in overlay_script
    )
    assert "umask 0002" in overlay_script
    assert "compatibility-only and is blocked by default" in floor_navigation_script
    assert "sudo systemctl restart njrh-runtime.service" in floor_navigation_script
    assert "NJRH_ALLOW_TRANSIENT_NAVIGATION_OWNER" in floor_navigation_script
    assert "NJRH_ALLOW_MANUAL_NAV_OWNER" in floor_navigation_script
    assert "run_navigation_runtime_services.sh" in floor_navigation_script
    assert "NJRH_RUNTIME_MAP_CONTEXT_FILE" in commercial_runtime_helpers
    assert "runtime_map_context_matches_current_floor()" in commercial_runtime_helpers
    assert 'local context_map_id="${NJRH_MAP_ID:-${NJRH_NAV_MAP_ID:-}}"' in commercial_runtime_helpers
    assert 'export NJRH_MAP_ID="${context_map_id}"' in commercial_runtime_helpers
    assert "runtime map context skipped because no map_id is available" in commercial_runtime_helpers
    assert "navigation_map_source_diagnostics()" in commercial_runtime_helpers
    assert "map_server asset is not directly confirmable" in commercial_runtime_helpers
    assert "navigation readiness failed: map_server_asset" not in commercial_runtime_helpers
    assert "navigation readiness failed: map_topic" not in commercial_runtime_helpers
    assert "write_runtime_map_context \"ready\" \"true\"" in resident_runtime_script
    assert "STARTUP_STAGE" in resident_runtime_script
    assert (
        "resident navigation runtime ready after trigger wrapper, bridge map->odom, Nav2 activation, "
        "and AMCL tracking readiness"
    ) in resident_runtime_script
    assert "resident_navigation_ready()" in resident_runtime_script
    assert 'NJRH_NAV_REUSE_READY_CONTEXT:-true' in resident_runtime_script
    assert "navigation_runtime_ready_for_current_floor 3" not in resident_runtime_script
    assert "wait_for_resident_navigation_runtime_ready()" not in resident_runtime_script
    assert "ensure_navigation_layer_alive()" in resident_runtime_script
    assert 'wait_for_resident_navigation_runtime_ready "${NJRH_NAV_RUNTIME_READY_TIMEOUT:-300}"' not in resident_runtime_script
    assert "resident navigation runtime did not become ready within" not in resident_runtime_script
    assert "ensure_localization_stack_ready_for_navigation()" in resident_runtime_script
    assert "wait_for_nav2_layer_ready()" in resident_runtime_script
    assert 'wait_for_ros_service "/global_localization/trigger"' in resident_runtime_script
    assert 'wait_for_ros_service "/trigger_grid_search_localization"' in resident_runtime_script
    assert 'wait_for_flatscan_publisher_ready "${flatscan_timeout}"' in resident_runtime_script
    assert 'wait_for_topic_publisher_from_node "/flatscan" "laser_scan_to_flatscan"' in resident_runtime_script
    assert 'NJRH_INITIAL_LOCALIZATION_FLATSCAN_MAX_AGE_SEC' not in resident_runtime_script
    assert "wait_for_bridge_has_map_to_odom()" in resident_runtime_script
    assert "trigger_output_reports_map_to_odom_ready()" in resident_runtime_script
    assert "wrapper already verified bridge map->odom readiness" in resident_runtime_script
    assert "initial_localization_ready_from_bridge_after_wrapper_failure()" in resident_runtime_script
    assert (
        "global localization wrapper did not return accepted before timeout, but bridge map->odom and fresh TF are ready"
    ) in resident_runtime_script
    assert "bridge_status did not report has_map_to_odom before wrapper timeout fallback" in resident_runtime_script
    assert 'wait_for_fresh_tf_transform "map" "odom" "${tf_timeout}" "${max_tf_age_sec}"' in resident_runtime_script
    assert (
        'initial_localization_ready_from_bridge_after_wrapper_failure \\\n'
        '      "${bridge_timeout}" \\\n'
        '      "${tf_timeout}" \\\n'
        '      "${wrapper_code:-none}" \\\n'
        '      "${baseline_explicit_sequence}"'
    ) in resident_runtime_script
    assert "map->odom ready owner=robot_localization_bridge" in resident_runtime_script
    assert "bridge_status.has_map_to_odom=true was not observed" in resident_runtime_script
    assert "map->odom TF was not published after bridge acceptance" in resident_runtime_script
    assert "BRIDGE_ACCEPT_TIMEOUT" in resident_runtime_script
    assert "BRIDGE_REJECTED_RESULT" in resident_runtime_script
    assert "MAP_TO_ODOM_WRONG_OWNER" in resident_runtime_script
    assert 'wait_for_fresh_header_topic_message \\\n    "/local_state/odometry"' in resident_runtime_script
    assert 'wait_for_fresh_header_topic_message \\\n    "/lidar_points"' not in resident_runtime_script
    assert 'wait_for_tf_transform "map" "odom"' in resident_runtime_script
    assert 'wait_for_global_costmap_static "${costmap_timeout}"' in resident_runtime_script
    assert "trigger_global_localization_for_navigation()" in resident_runtime_script
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_RECHECK_TIMEOUT:-30" not in resident_runtime_script
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-90" in resident_runtime_script
    assert "resident_navigation_start:${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" in resident_runtime_script
    assert "resident localization layer already owns map/localizer loading" in resident_runtime_script
    assert "floor_manager source preflight is not repeated during runtime startup" in resident_runtime_script
    assert "/floor_manager/switch_floor" not in resident_runtime_script
    assert "resume_navigation: true}" not in resident_runtime_script
    resident_runtime_startup = resident_runtime_script.split('if resident_navigation_ready; then', 1)[1]
    assert "ensure_localization_stack_ready_for_navigation ||" in resident_runtime_startup
    assert 'ensure_map_server_active "${NAV2_MAP_YAML:-}" "${map_server_timeout}"' in resident_runtime_script
    assert "MAP_SERVER_NOT_ACTIVE" in resident_runtime_script
    assert 'log_startup_stage "waiting_for_localization"' in resident_runtime_script
    assert "wait_for_later_initial_localization || exit 1" in resident_runtime_script
    assert "start_initial_global_localization_background()" in resident_runtime_script
    assert "wait_for_initial_global_localization()" in resident_runtime_script
    assert "NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false" in resident_runtime_script
    assert (
        "initial global localization trigger is starting after full "
        "localization-stack and floor-context readiness"
    ) in resident_runtime_script
    assert resident_runtime_startup.index(
        "ensure_localization_stack_ready_for_navigation"
    ) < resident_runtime_startup.index('log_startup_stage "floor_asset_context_verified"')
    assert resident_runtime_startup.index(
        'log_startup_stage "floor_asset_context_verified"'
    ) < resident_runtime_startup.index("start_initial_global_localization_background")
    assert resident_runtime_startup.index(
        "wait_for_initial_global_localization"
    ) < resident_runtime_startup.index(
        'start_resident_navigation_layer "true" "nav2_layer_prestarted_held"'
    )
    assert "existing resident navigation context matches selected floor" in resident_runtime_script
    assert 'NJRH_NAV2_REUSE_READY_STACK:-false' in overlay_nav2_script
    assert 'map_server_ready_timeout_sec="${NJRH_NAV_MAP_SERVER_READY_TIMEOUT:-75}"' in overlay_nav2_script
    assert 'global_costmap_ready_timeout_sec="${NJRH_NAV_GLOBAL_COSTMAP_READY_TIMEOUT:-90}"' in overlay_nav2_script
    assert 'ensure_map_server_active "${NAV2_MAP_YAML:-}" "${map_server_ready_timeout_sec}"' not in overlay_nav2_script
    assert 'wait_for_global_costmap_static "${global_costmap_ready_timeout_sec}"' not in overlay_nav2_script
    assert "starting Nav2 without blocking map/topic/TF readiness probes" in overlay_nav2_script
    assert "blocking readiness probes are disabled" in overlay_nav2_script
    assert 'MAP_SERVER_READY_TIMEOUT="${NJRH_LOCALIZATION_MAP_SERVER_READY_TIMEOUT:-75}"' in occupancy_localization_script
    assert 'LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP="${NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP:-true}"' in occupancy_localization_script
    assert '"log_level:=${NJRH_LOCALIZATION_LOG_LEVEL:-warn}"' in occupancy_localization_script
    assert 'map_lifecycle_manager_enabled:=false' in occupancy_localization_script
    map_lifecycle_block = occupancy_localization_script[
        occupancy_localization_script.index("start_map_server_lifecycle_with_nav2_util()") :
        occupancy_localization_script.index("repair_jt128_navigation_points()")
    ]
    assert "nav2_lifecycle_sequence.py" in map_lifecycle_block
    assert "/opt/ros/humble/lib/nav2_util/lifecycle_bringup map_server" not in map_lifecycle_block
    assert "localization map_server repo lifecycle sequence: map_server active" in occupancy_localization_script
    assert "map_lifecycle_bringup_pid" in occupancy_localization_script
    assert 'export NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP="${NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP:-true}"' in resident_runtime_script
    assert "map_lifecycle_manager_enabled" in occupancy_localization_launch
    assert 'DeclareLaunchArgument("log_level", default_value="info")' in occupancy_localization_launch
    assert 'arguments=["--ros-args", "--log-level", log_level]' in occupancy_localization_launch
    assert "PythonExpression" in occupancy_localization_launch
    assert 'executable="component_container_isolated"' in occupancy_localization_launch
    assert 'executable="component_container_mt"' not in occupancy_localization_launch
    assert "respawn=True" in occupancy_localization_launch
    assert "respawn_delay=0.5" in occupancy_localization_launch
    assert 'ensure_map_server_active "${NAV2_MAP_YAML}" "${MAP_SERVER_READY_TIMEOUT}"' not in occupancy_localization_script
    assert "localization pointcloud startup probe disabled" in occupancy_localization_script
    assert "standard_nav_stack_ready()" in overlay_nav2_script
    assert "standard Nav2 navigation stack already ready; reusing existing stack" in overlay_nav2_script
    assert "NJRH_NAV_STOP_ZERO_TIMEOUT_SEC" in stop_navigation_script
    assert "NJRH_NAV_STOP_ZERO_TIMEOUT_SEC:-0.25s" in stop_navigation_script
    assert 'timeout --kill-after="${NJRH_NAV_STOP_ZERO_KILL_AFTER_SEC:-0.25s}"' in stop_navigation_script
    assert 'publish_zero_topic "${topic}" &' in stop_navigation_script
    assert "mapfile -t pids" in stop_navigation_script
    assert "matching_navigation_processes | awk '{print $1}'" in stop_navigation_script
    assert 'for pattern in "${patterns[@]}"' not in stop_navigation_script
    assert "final_navigation_cleanup()" in stop_navigation_script
    assert "NJRH_NAV_STOP_FINAL_GRACE_SEC:-2" in stop_navigation_script
    assert "NJRH_NAV_STOP_FINAL_KILL_WAIT_SEC:-2" in stop_navigation_script
    assert stop_navigation_script.index("stop_amcl_bounded\nfinal_navigation_cleanup") < stop_navigation_script.index(
        'lingering="$(matching_navigation_processes)"'
    )
    assert "NJRH_NAV_STOP_INT_WAIT_SEC:-1" in stop_navigation_script
    assert "NJRH_NAV_STOP_TERM_WAIT_SEC:-1" in stop_navigation_script
    assert "NJRH_NAV_STOP_KILL_WAIT_SEC:-1" in stop_navigation_script
    assert "NJRH_NAV_STOP_AMCL_TIMEOUT_SEC" in stop_navigation_script
    assert "NJRH_NAV_STOP_AMCL_KILL_AFTER_SEC" in stop_navigation_script
    assert "stop_amcl_bounded()" in stop_navigation_script
    assert "timeout --kill-after" in stop_navigation_script
    assert "run_navigation_runtime_services.sh" in stop_navigation_script
    assert "run_local_perception.sh" in stop_navigation_script
    assert "robot_local_perception/local_perception_node" in stop_navigation_script
    assert "nav2_amcl/amcl" in stop_navigation_script
    assert "amcl_scan_admission_node" in stop_navigation_script
    assert "amcl_scan_admission_relay.py" in stop_navigation_script
    assert "clear_runtime_map_context()" in stop_navigation_script
    assert "rm -f \"${context_file}\"" in stop_navigation_script
    assert "run_amcl_shadow_localization.sh\" --stop >/dev/null" not in stop_navigation_script
    assert stop_navigation_script.index("kill_navigation_patterns KILL") < stop_navigation_script.index(
        "publish_zero\nstop_amcl_bounded\nfinal_navigation_cleanup\nclear_runtime_map_context"
    )
    assert "The app should not join ROS 2 DDS directly" in app_doc
    assert "bms.soc" in app_doc
    assert "Ranger `/battery_state`" in app_doc
    assert "POST /api/v1/subscriptions/acquire" in app_doc
    assert "Page-Scoped Subscriptions" in app_doc
    assert "acquire `live_map`" in app_doc
    assert "must not bypass `robot_safety`" in app_doc
    assert "POST /api/v1/mapping/2d/start" in app_doc
    assert "mapping.start_job" in app_doc
    assert "mode_transition" in app_doc
    assert "stop_navigation_runtime" in app_doc
    assert "POST /api/v1/mapping/2d/stop" in app_doc
    assert "POST /api/v1/mapping/2d/save" in app_doc
    assert "POST /api/v1/mapping/stop" in app_doc
    assert "POST /api/v1/mapping/save" in app_doc
    assert "POST /api/v1/maps/delete" in app_doc
    assert "GET  /api/v1/robot/pose" in app_doc
    assert "no fresh map-frame robot pose" in app_doc
    assert "Request-body `yaw` or `theta` is ignored" in app_doc
    assert "GET  /api/v1/maps/semantic_layer" in app_doc
    assert "GET  /api/v1/maps/poses" in app_doc
    assert "GET  /api/v1/maps/filters/keepout" in app_doc
    assert "POST /api/v1/maps/poses" in app_doc
    assert "PUT  /api/v1/maps/poses/{pose_id}" in app_doc
    assert "DELETE /api/v1/maps/poses/{pose_id}" in app_doc
    assert "PUT  /api/v1/maps/poses/batch" in app_doc
    assert "POST /api/v1/maps/poses/save" in app_doc
    assert "POST /api/v1/maps/poses/save_current" in app_doc
    assert "POST /api/v1/maps/filters/keepout/save" in app_doc
    assert "POST /api/v1/navigation/goal" in app_doc
    assert "navigation_goal_id" in app_doc
    assert "goal_completion_policy" in app_doc
    assert "task_complete" in app_doc
    assert "final_yaw_align_blocked" in app_doc
    assert "Delivery completion is policy-driven" in readme
    assert "POST http://<robot-ip>:8080/api/v1/docking/undock" in app_doc
    assert "automatically performs controlled undocking first" in app_doc
    assert "The App must not use mapping teleop or direct velocity commands for docking" in app_doc
    assert "checking yaw sanity only by default" in app_doc
    assert "0.356m" in app_doc
    assert "docking_manual_predock_distance_check_enable` is `false" in readme
    assert "Manual pre-dock distance checking is disabled by default" in lifecycle_doc
    assert "NavigateToPose" in app_doc
    assert "The App must not send `/cmd_vel` for task navigation" in app_doc
    assert "mapping_active" in app_doc
    assert "building_id" in app_doc
    assert "floor_assets" in app_doc
    assert "localizer_map_png" in app_doc
    assert "GET  /api/v1/mapping/2d/map" in app_doc
    assert "Content-Type: image/png" in app_doc
    assert "current live `slam_toolbox` `/map`" in app_doc
    assert "returns JSON `409`/`404` instead of falling back" in app_doc
    assert "does not convert `PGM` to `PNG` during the request" in app_doc
    assert "WS   /ws/v1/teleop" in app_doc
    assert "Teleop stops automatically" in app_doc
    assert "/cmd_vel_api -> robot_safety -> /cmd_vel" in app_doc
    assert "/ranger_mini3/teleop_allow_reverse" in app_doc
    assert "mapping_state" in app_doc
    assert "resume_navigation" in app_doc
    assert "start the occupancy localization stack" in app_doc
    assert "startup waits for the trigger wrapper to report a bridge-accepted `map -> odom`" in app_doc
    assert "run_occupancy_grid_localization.sh" in resident_runtime_script
    assert "run_nav2_navigation.sh" in resident_runtime_script
    assert "/floor_manager/switch_floor" not in resident_runtime_script
    assert "global_localization_node" in global_localization_script
    assert "global_localization_node.py" not in global_localization_script
    assert "GLOBAL_LOCALIZATION_PARAMS_FILE" in global_localization_script
    assert "robot_global_localization/global_localization_node|" in nav_runtime_helpers
    assert "/trigger_grid_search_localization" in resident_runtime_script
    assert "trigger_global_localization_and_wait_for_result" not in resident_runtime_script
    assert "pre-armed localization_result subscribers" not in resident_runtime_script
    assert 'local call_timeout="${NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-90}"' in resident_runtime_script
    assert 'local bridge_timeout="${NJRH_INITIAL_LOCALIZATION_BRIDGE_ACCEPT_WAIT_SEC:-8}"' in resident_runtime_script
    assert 'local tf_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_ODOM_WAIT_SEC:-8}"' in resident_runtime_script
    assert "global localization wrapper accepted" in resident_runtime_script
    assert "trigger_output_reports_map_to_odom_ready" in resident_runtime_script
    assert "wrapper already verified bridge map->odom readiness" in resident_runtime_script
    assert "bridge_status.has_map_to_odom=true was not observed" in resident_runtime_script
    assert "map->odom TF was not published after bridge acceptance" in resident_runtime_script
    assert 'NJRH_GLOBAL_LOCALIZATION_TRIGGER_FALLBACK_TF_TIMEOUT:-20' not in resident_runtime_script
    assert "requesting global localization through wrapper and waiting for bridge/map->odom" in resident_runtime_script
    assert "start_resident_navigation_layer" in resident_runtime_script
    assert '"nav2_layer_prestarted"' in resident_runtime_script
    assert "runtime_ready=0" in resident_runtime_script
    assert "write_runtime_map_context \"failed\" \"false\"" in resident_runtime_script
    assert "resident navigation runtime failed; check" in resident_runtime_script
    assert '#include "std_srvs/srv/empty.hpp"' in global_localization_node
    assert "grid_search_trigger_service" in global_localization_node
    assert "MultiThreadedExecutor" in global_localization_node
    assert "async_send_request" in global_localization_node
    assert "wait_for_bridge_acceptance" in global_localization_node
    assert "wait_for_map_to_odom" in global_localization_node
    assert "map_odom_publish_gap_ms" in global_localization_node
    assert "map_to_odom_bridge_publish_healthy" in global_localization_node
    assert "map_to_odom_ready(latest)" in global_localization_node
    assert "draining pre-arm localization_result inside the same trigger transaction" in global_localization_node
    assert "failure_code=BRIDGE_ACCEPT_TIMEOUT" in global_localization_node
    assert "failure_code=MAP_TO_ODOM_TIMEOUT" in global_localization_node
    assert "done.wait" not in global_localization_node
    assert "mock_localizer_ready" in global_localization_node
    assert "localizer_waiting_for_grid_search" in global_localization_node
    assert "mock_mode: false" in global_localization_config
    assert "/trigger_grid_search_localization" in global_localization_config
    assert "<depend>rclcpp</depend>" in global_localization_package
    assert "<depend>std_srvs</depend>" in global_localization_package
    assert "<exec_depend>rclpy</exec_depend>" not in global_localization_package
    assert not (ROOT / "src" / "robot_global_localization" / "scripts" / "global_localization_node.py").exists()
    assert 'wait_for_flatscan_publisher_ready "${flatscan_timeout}"' in resident_runtime_script
    assert 'wait_for_topic_publisher_from_node "/flatscan" "laser_scan_to_flatscan"' in resident_runtime_script
    assert 'wait_for_topic_message "/flatscan"' not in resident_runtime_script
    assert "create_subscription(" not in resident_runtime_script
    assert '"/localization_result"' in resident_runtime_script
    assert 'wait_for_tf_transform "map" "odom"' in resident_runtime_script
    assert "NJRH_NAV_MAP_NAME" in floor_asset_helpers
    assert "NJRH_NAV_MAP_ID" in floor_asset_helpers
    assert 'export NJRH_MAP_ID="${nav_map_id:-${NJRH_MAP_ID:-}}"' in floor_asset_helpers
    assert 'export NJRH_MAP_DISPLAY_NAME="${nav_map_name:-${NJRH_MAP_DISPLAY_NAME:-}}"' in floor_asset_helpers
    assert 'export NJRH_MAP_CONTEXT_BUILDING_ID="${building_id}"' in floor_asset_helpers
    assert 'export NJRH_MAP_CONTEXT_FLOOR_ID="${floor_id}"' in floor_asset_helpers
    assert "asset_report.json" in floor_asset_helpers
    assert "wait_for_ros_service()" in nav_runtime_helpers
    assert 'runtime_readiness_probe service "${service_name}" "${timeout_sec}"' in nav_runtime_helpers
    assert "wait_for_ros_service_direct" not in nav_runtime_helpers
    assert "ros2 service type" not in nav_runtime_helpers
    assert "rosidl_runtime_py.utilities import get_message" not in nav_runtime_helpers
    assert "runtime_readiness_probe topic" in nav_runtime_helpers
    assert "create_generic_subscription" in runtime_probe_cpp
    assert "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT" in runtime_probe_cpp
    assert "timed out waiting for topic message: " in runtime_probe_cpp
    assert "helper_ready()" in nav_runtime_helpers
    assert "local_perception_runtime_config_ready()" not in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception' not in nav_runtime_helpers
    assert 'wait_for_topic_message "/scan"' in nav_runtime_helpers
    assert "wait_for_local_costmap_observation_ready()" in nav_runtime_helpers
    assert 'wait_for_topic_message "/local_costmap/costmap"' in nav_runtime_helpers
    assert 'wait_for_fresh_tf_transform "odom" "base_link"' in nav_runtime_helpers
    assert "local costmap observation ready from runtime health snapshot" not in nav_runtime_helpers
    assert "odom_base_tf_not_fresh" in nav_runtime_helpers
    assert "from tf2_msgs.msg import TFMessage" not in nav_runtime_helpers
    assert "tf2_msgs::msg::TFMessage" in runtime_probe_cpp
    assert 'create_subscription<tf2_msgs::msg::TFMessage>' in runtime_probe_cpp
    assert "source=/tf" in runtime_probe_cpp
    assert "wait_for_transformable_local_scan" in nav_runtime_helpers
    assert "transformable-scan" in nav_runtime_helpers
    assert "NJRH_LOCAL_COSTMAP_TF_BUFFER_WARMUP_SEC" in runtime_probe_cpp
    assert "tf_buffer_warmup" in runtime_probe_cpp
    assert "local_costmap_scan_tf_not_transformable" in nav_runtime_helpers
    assert 'wait_for_topic_message "/safety/status"' not in nav_runtime_helpers
    assert 'wait_for_ros_service "/floor_manager/switch_floor"' not in nav_runtime_helpers
    assert "existing ${helper_name} process is stale" not in nav_runtime_helpers
    assert "existing ${helper_name} process will be restarted" in nav_runtime_helpers
    assert "cleanup_stale_overlay_helper" in nav_runtime_helpers
    assert "wait_for_ros_node()" in map_server_helpers
    assert 'runtime_readiness_probe node "${node_name}" "${timeout_sec}"' in map_server_helpers
    assert "ros2 node info" not in map_server_helpers
    assert "ros2 node list" not in map_server_helpers
    assert "map_server_publishing_requested_map()" in map_server_helpers
    assert "map_topic_matches_yaml()" in map_server_helpers
    assert "runtime_readiness_probe map-topic-matches-yaml" in map_server_helpers
    assert "runtime_readiness_probe occupancy-grid" in map_server_helpers
    assert "NJRH_MAP_TOPIC_MATCH_TIMEOUT_SEC:-8.0" in map_server_helpers
    map_publishing_body = map_server_helpers.split("map_server_publishing_requested_map()", 1)[1].split("ensure_map_server_active()", 1)[0]
    assert "map_server_param_matches_yaml" not in map_publishing_body
    assert "requested map is already published on /map; continuing without waiting for /map_server discovery" in map_server_helpers
    assert "/map_server node discovery unavailable, but requested map is being published; continuing" in map_server_helpers
    assert "requested map is already published after /map_server discovery; continuing without lifecycle state probe" in map_server_helpers
    assert "timeout 3 ros2 lifecycle get /map_server" in map_server_helpers
    assert "localization_map_external_lifecycle_bringup_enabled()" in map_server_helpers
    assert "external lifecycle_bringup is managing /map_server" in map_server_helpers
    assert "lifecycle state unavailable, but requested map is selected; continuing" in map_server_helpers
    assert 'wait_for_ros_service "/global_localization/trigger" "${NJRH_GLOBAL_LOCALIZATION_SERVICE_WAIT_SEC:-75}"' not in resident_runtime_script
    assert 'wait_for_ros_service "/trigger_grid_search_localization" "${NJRH_ISAAC_TRIGGER_SERVICE_WAIT_SEC:-75}"' not in resident_runtime_script
    assert 'wait_for_ros_service "/floor_manager/switch_floor" "${NJRH_FLOOR_MANAGER_SERVICE_WAIT_SEC:-45}"' not in resident_runtime_script
    assert "wait_for_tf_transform()" in nav_runtime_helpers
    assert "lookupTransform(target, source, tf2::TimePointZero" in runtime_probe_cpp
    assert "buffer.can_transform" not in nav_runtime_helpers
    assert "refresh_runtime_state" in node_cpp
    assert "navigation runtime process exited before ready" in node_cpp
    assert 'context && !context->confirmed && context->state == "failed"' in node_cpp
    assert "navigation runtime failed before ready" in node_cpp
    assert "runtime_context_matches_resume_request" in node_cpp
    assert "runtime_context_same_resume_request" in node_cpp
    assert "runtime_context_starting_is_fresh" in node_cpp
    assert "navigation_resume_starting_context_ttl_sec" in node_cpp
    assert "runtime map context is not ready: state=" in node_cpp
    assert "startup_stage=" in node_cpp
    assert '\\"runtime_map_context\\":' in node_cpp
    assert '\\"startup_stage\\":' in runtime_context_cpp
    assert 'json_string_value(text, "startup_stage")' in runtime_context_cpp
    assert "navigation_runtime_starting_reused" in node_cpp
    assert "resident navigation runtime already starting for selected floor" in node_cpp
    assert '\\"external_runtime\\":' in node_cpp
    assert "navigation_runtime_reused" in node_cpp
    assert "reused" in node_cpp
    assert node_cpp.index("existing_resume_process_running") < node_cpp.index(
        "process_runtime_.terminate_managed_process();"
    )
    assert 'context->confirmed && context->state == "ready"' in node_cpp
    assert "action_server_is_ready" in node_cpp
    assert "navigation runtime ready" in node_cpp
    assert "const bool pre_nav_resume = request->resume_navigation" not in floor_manager_code
    assert "if (request->resume_navigation)" in floor_manager_code
    assert "LEGACY_RESUME_NAVIGATION_DISABLED" in floor_manager_code
    assert (
        "floor source preflight verified; API activation is still required: "
        in floor_manager_code
    )
    assert 'publish_status("switching:" + map_key)' not in floor_manager_code
    assert "create_client<nav2_msgs::srv::ClearEntireCostmap>" in floor_manager_code
    assert "create_client<std_srvs::srv::Empty>" not in floor_manager_code
    assert resident_runtime_script.index("run_occupancy_grid_localization.sh") < resident_runtime_script.index(
        'sleep "${NJRH_NAV_LOCALIZATION_START_SETTLE_SEC:-0.1}"'
    )
    assert resident_runtime_script.index(
        'sleep "${NJRH_NAV_LOCALIZATION_START_SETTLE_SEC:-0.1}"'
    ) < resident_runtime_script.index('log_startup_stage "floor_asset_context_verified"')
    assert resident_runtime_startup.index('log_startup_stage "floor_asset_context_verified"') < resident_runtime_startup.index(
        "wait_for_initial_global_localization"
    )
    assert resident_runtime_startup.index("wait_for_initial_global_localization") < resident_runtime_startup.index(
        'log_startup_stage "nav2_layer_started_after_initial_localization"'
    )
    assert resident_runtime_startup.index("wait_for_initial_global_localization") < resident_runtime_startup.index(
        'sleep "${NJRH_NAV_RUNTIME_READY_MARK_DELAY_SEC:-0.0}"'
    )
    assert "dashboard_server" not in composition_cpp
    assert "patch_dashboard_runtime" not in composition_cpp
    assert "system(" not in composition_cpp
    assert "popen(" not in composition_cpp
    assert "/api/v1/docking/undock" in node_cpp
    assert "handle_undock" in node_cpp
    assert "undock_before_navigation_if_needed" in node_cpp
    assert "class PreNavigationUndockModule" in pre_navigation_undock_header
    assert "bool PreNavigationUndockModule::start" in pre_navigation_undock_cpp
    assert "bool PreNavigationUndockModule::wait_for_completion" in pre_navigation_undock_cpp
    assert "call_undock_service_with_charging_retry" in node_cpp
    assert "DockingUndockServiceObservation" in node_cpp
    assert "call_trigger_service_observed" in node_cpp
    assert "docking_service_success" in node_cpp
    assert "docking_service_message" in node_cpp
    assert "docking_status_at_request" in node_cpp
    assert "docking_status_after_request" in node_cpp
    assert "undock_started_observed" in node_cpp
    assert "undock_cmd_count_observed" in node_cpp
    assert "undock_failure_reason" in node_cpp
    assert "service_success_without_undocking_status_observed_yet" in pre_navigation_undock_cpp
    assert "already running; no new /docking/undock service call" in node_cpp
    assert "complete_post_undock_relocalization" in docking_status_module_cpp
    assert "relocalize_after_undock" in docking_status_module_cpp
    assert "undock_after_success" in docking_status_module_cpp
    assert "waiting for docking manager charging state before undock" in node_cpp
    assert "pre_navigation_undock" in node_cpp
    assert "undock_client_" in node_cpp
    assert "docking_status_is_undocked" in node_cpp
    assert "undock requires docked state or live charging contact" in node_cpp


def test_ranger_chassis_core_owns_mode_switching_and_reliable_can_tx():
    package_root = ROOT / "src" / "ranger_base"
    sdk_root = ROOT / "external_sources" / "jetson_ugv_sdk_20260713" / "ugv_sdk"
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    messenger = (package_root / "src" / "ranger_messenger.cpp").read_text(encoding="utf-8")
    async_can = (sdk_root / "src" / "async_port" / "async_can.cpp").read_text(encoding="utf-8")
    async_can_header = (
        sdk_root / "include" / "ugv_sdk" / "details" / "async_port" / "async_can.hpp"
    ).read_text(encoding="utf-8")
    common_services = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh"
    ).read_text(encoding="utf-8")

    assert package_root.exists()
    assert (sdk_root / "COLCON_IGNORE").exists()
    assert not (ROOT / "src" / "ranger_mini3_mode_controller").exists()
    assert not (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_ranger_mini3_mode_controller.sh"
    ).exists()
    assert "RANGER_UGV_SDK_SOURCE_DIR" in cmake
    assert "add_subdirectory" in cmake
    assert "async_write_some" not in async_can
    assert "asio::async_write" in async_can
    assert "EnqueueFrame" in async_can
    assert "kMotionCommandCanId = 0x111" in async_can
    assert "kMotionModeCommandCanId = 0x141" in async_can
    assert "std::deque<struct can_frame> tx_queue_" in async_can_header
    assert "EnsureMotionModeReady" in messenger
    assert "latest_feedback_mode_changing_" in messenger
    assert "mode_switch_handshake_enabled" in messenger
    assert '"/ranger_base/status"' in messenger
    assert "robot_->SetMotionCommand(0.0, 0.0, 0.0)" in messenger
    assert "Ranger mode switch timeout" in messenger
    assert "run_ranger_mini3_mode_controller.sh" not in common_services


def test_occupancy_builder_package_exists():
    package_root = ROOT / "src" / "robot_occupancy_builder"
    assert (package_root / "package.xml").exists()
    assert (package_root / "config" / "live_draft.yaml").exists()
    assert (package_root / "config" / "release_rebuild.yaml").exists()
    assert (package_root / "scripts" / "occupancy_builder_live_node.py").exists()
    assert (package_root / "scripts" / "occupancy_builder_release_node.py").exists()
    assert (package_root / "scripts" / "occupancy_postprocess.py").exists()
    assert (ROOT / "docs" / "occupancy_builder_workflow.md").exists()


def test_occupancy_builder_contracts_are_repo_owned():
    live_cfg = (ROOT / "src" / "robot_occupancy_builder" / "config" / "live_draft.yaml").read_text(encoding="utf-8")
    release_cfg = (ROOT / "src" / "robot_occupancy_builder" / "config" / "release_rebuild.yaml").read_text(encoding="utf-8")
    fastlio_cfg = (ROOT / "src" / "robot_fastlio_mapping" / "config" / "fastlio.yaml").read_text(encoding="utf-8")
    pgo_script = (ROOT / "src" / "robot_pgo_mapping" / "scripts" / "pgo_wrapper_node.py").read_text(encoding="utf-8")
    assert "/mapping/frontend_pose" in live_cfg
    assert "/mapping/draft_map" in live_cfg
    assert "raw_bag_path" in release_cfg
    assert "optimized_trajectory_csv" in release_cfg
    assert "frontend_pose_topic" in fastlio_cfg
    assert "yaw" in pgo_script


def test_local_perception_node_ports_validated_base_link_filter_contract():
    cmake = (ROOT / "src" / "robot_local_perception" / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (ROOT / "src" / "robot_local_perception" / "package.xml").read_text(encoding="utf-8")
    node_script = (ROOT / "src" / "robot_local_perception" / "src" / "local_perception_node.cpp").read_text(encoding="utf-8")
    readme = (ROOT / "src" / "robot_local_perception" / "README.md").read_text(encoding="utf-8")
    local_cfg = (ROOT / "src" / "robot_local_perception" / "config" / "local_perception.yaml").read_text(encoding="utf-8")
    overlay_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_perception.yaml"
    ).read_text(encoding="utf-8")
    run_local = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_perception.sh"
    ).read_text(encoding="utf-8")
    assert "find_package(nav_msgs REQUIRED)" in cmake
    assert "nav_msgs" in cmake
    assert "<depend>nav_msgs</depend>" in package_xml
    assert 'declare_parameter<bool>("enabled", false)' in node_script
    assert 'declare_parameter<std::string>("input_topic", "")' in node_script
    assert 'declare_parameter<std::string>("output_topic", "")' in node_script
    assert 'declare_parameter<std::string>("clearing_output_topic", "")' in node_script
    assert 'declare_parameter<std::string>("status_topic", "")' in node_script
    assert "robot_local_perception is retired and disabled by default" in node_script
    assert "requires explicit input_topic, output_topic, and clearing_output_topic" in node_script
    assert 'enabled: false' in local_cfg
    assert 'enabled: false' in overlay_cfg
    assert 'input_topic: ""' in local_cfg
    assert 'input_topic: ""' in overlay_cfg
    assert 'output_topic: ""' in local_cfg
    assert 'output_topic: ""' in overlay_cfg
    assert 'clearing_output_topic: ""' in local_cfg
    assert 'clearing_output_topic: ""' in overlay_cfg
    assert "clearing.enabled: false" in local_cfg
    assert "clearing.enabled: false" in overlay_cfg
    assert 'status_topic: ""' in local_cfg
    assert 'status_topic: ""' in overlay_cfg
    assert "/perception/obstacle_points" not in local_cfg
    assert "/perception/obstacle_points" not in overlay_cfg
    assert "/perception/clearing_points" not in local_cfg
    assert "/perception/clearing_points" not in overlay_cfg
    assert "/perception/local_perception_status" not in local_cfg
    assert "/perception/local_perception_status" not in overlay_cfg
    assert "max_output_tf_stamp_age_sec: 0.25" in local_cfg
    assert "max_output_tf_stamp_age_sec: 0.25" in overlay_cfg
    assert "retired from the production navigation runtime" in readme
    assert "Nav2 `local_costmap` uses `nav2_costmap_2d::ObstacleLayer`" in readme
    assert "no production local PointCloud2 obstacle publisher" in readme
    assert "no production local PointCloud2 clearing publisher" in readme
    assert "robot_local_perception PointCloud2 obstacle pipeline has been removed" in run_local


def test_robot_safety_node_exports_stateful_final_cmd_vel_contract():
    node_script = (ROOT / "src" / "robot_safety" / "src" / "robot_safety_node.cpp").read_text(encoding="utf-8")
    config_text = (ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml").read_text(encoding="utf-8")
    overlay_config_text = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_safety.yaml"
    ).read_text(encoding="utf-8")
    lateral_watchdog_wrapper = (
        ROOT
        / "src"
        / "robot_safety"
        / "test"
        / "run_isolated_normal_lateral_watchdog_smoke.sh"
    ).read_text(encoding="utf-8")
    lateral_watchdog_smoke = (
        ROOT
        / "src"
        / "robot_safety"
        / "test"
        / "isolated_normal_lateral_watchdog_smoke.py"
    ).read_text(encoding="utf-8")
    api_node = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    navigation_terminal_runtime_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "navigation"
        / "terminal_control" / "navigation_terminal_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    docking_runtime_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "docking_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    teleop_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "teleop"
        / "teleop_module.cpp"
    ).read_text(encoding="utf-8")
    docking_node = (
        ROOT / "src" / "robot_docking_manager" / "src" / "docking_manager_node.cpp"
    ).read_text(encoding="utf-8")
    ranger_core = (
        ROOT / "src" / "ranger_base" / "src" / "ranger_messenger.cpp"
    ).read_text(encoding="utf-8")
    cmd_vel_stop_latency = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "record_cmd_vel_stop_latency.sh"
    ).read_text(encoding="utf-8")
    spin_imu_yaw = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_ranger_spin_imu_yaw_test.sh"
    ).read_text(encoding="utf-8")
    assert "enum class SafetyState" in node_script
    assert '"status_topic", "/safety/status"' in node_script
    assert '"motion_allowed_topic", "/safety/motion_allowed"' in node_script
    assert "COMMAND_STALE" in node_script
    assert "LOCALIZATION_INVALID" in node_script
    assert "status_topic: /safety/status" in config_text
    assert "motion_allowed_topic: /safety/motion_allowed" in config_text
    assert "cmd_vel_out_topic: /cmd_vel" in config_text
    assert "cmd_vel_mirror_topic: /cmd_vel_safe" in config_text
    assert "cmd_vel_qos_depth: 1" in config_text
    assert "zero_cmd_priority_enabled: true" in config_text
    assert "zero_cmd_priority_epsilon: 0.0001" in config_text
    assert "zero_cmd_priority_burst_sec: 0.25" in config_text
    assert "cmd_vel_qos_depth: 1" in overlay_config_text
    assert "zero_cmd_priority_enabled: true" in overlay_config_text
    assert "zero_cmd_priority_epsilon: 0.0001" in overlay_config_text
    assert "zero_cmd_priority_burst_sec: 0.25" in overlay_config_text
    assert "spin_to_drive_require_local_odom_stable: false" in config_text
    assert "spin_to_drive_local_odom_topic: /local_state/odometry" in config_text
    assert "spin_to_drive_local_wz_threshold_radps: 0.03" in config_text
    assert "spin_to_drive_local_stable_samples: 5" in config_text
    assert "spin_to_drive_local_stable_duration_sec: 0.30" in config_text
    assert "spin_to_drive_local_yaw_delta_threshold_rad: 0.005" in config_text
    assert "spin_to_drive_require_imu_stable: true" in config_text
    assert "spin_to_drive_imu_topic: /lidar_imu_bias_corrected" in config_text
    assert "spin_to_drive_imu_wz_threshold_radps: 0.035" in config_text
    assert "spin_to_drive_imu_stable_duration_sec: 0.30" in config_text
    assert "spin_to_drive_imu_max_age_sec: 0.10" in config_text
    assert "spin_to_drive_timeout_sec: 2.0" in config_text
    assert "mode_exit_guard_enabled: true" in config_text
    assert "mode_controller_status_topic: /ranger_base/status" in config_text
    assert "mode_exit_guard_probe_speed_mps: 0.06" in config_text
    assert "mode_exit_guard_timeout_sec: 1.0" in config_text
    assert "final_cmd_lateral_deadband_mps: 0.001" in config_text
    assert "allow_api_lateral_cmd: true" in config_text
    assert "api_lateral_max_mps: 0.10" in config_text
    assert "api_cmd_vel_in_topic: /cmd_vel_api" in config_text
    assert "enable_api_cmd_priority: true" in config_text
    assert "api_cmd_priority_timeout_sec: 0.25" in config_text
    assert "docking_cmd_vel_in_topic: /cmd_vel_docking" in config_text
    assert "enable_docking_cmd_priority: true" in config_text
    assert "docking_cmd_priority_timeout_sec: 0.25" in config_text
    assert 'declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel")' in node_script
    assert 'declare_parameter<std::string>("cmd_vel_mirror_topic", "/cmd_vel_safe")' in node_script
    assert 'declare_parameter<int>("cmd_vel_qos_depth", 1)' in node_script
    assert 'declare_parameter<bool>("zero_cmd_priority_enabled", true)' in node_script
    assert 'declare_parameter<double>("zero_cmd_priority_epsilon", 0.0001)' in node_script
    assert 'declare_parameter<double>("zero_cmd_priority_burst_sec", 0.25)' in node_script
    assert "rclcpp::QoS(rclcpp::KeepLast(cmd_vel_qos_depth_)).reliable()" in node_script
    assert "command_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>" in navigation_terminal_runtime_cpp
    assert "config_.command_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable()" in navigation_terminal_runtime_cpp
    assert "create_publisher<geometry_msgs::msg::Twist>" in teleop_module_cpp
    assert "config_.cmd_topic, command_qos" in teleop_module_cpp
    assert "config_.command_topic, command_qos" in docking_runtime_cpp
    assert "predock_yaw_align_cmd_pub_" not in api_node
    assert "cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable()" in docking_node
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()" in ranger_core
    assert "cmd_qos = QoSProfile(depth=1)" in cmd_vel_stop_latency
    assert "--discovery-timeout-sec" in cmd_vel_stop_latency
    assert "node.pub.get_subscription_count() >= 2" in cmd_vel_stop_latency
    assert "zero-command handshake did not reach /cmd_vel_safe and /cmd_vel" in cmd_vel_stop_latency
    assert "SingleThreadedExecutor" in cmd_vel_stop_latency
    assert "depth=1" in cmd_vel_stop_latency
    assert '"/lidar_imu_bias_corrected"' in cmd_vel_stop_latency
    assert 'metrics["wheel_yaw_tail_deg"]' in cmd_vel_stop_latency
    assert 'metrics["imu_yaw_tail_deg"]' in cmd_vel_stop_latency
    assert "cmd_qos = QoSProfile(depth=1)" in spin_imu_yaw
    assert "handle_zero_priority_command" in node_script
    assert "zero_cmd_priority_active()" in node_script
    assert "publish_stop_priority_command" in node_script
    assert "twist_near_zero(cmd, zero_cmd_priority_epsilon_)" in node_script
    assert 'declare_parameter<bool>("spin_to_drive_require_local_odom_stable", false)' in node_script
    assert 'declare_parameter<std::string>("spin_to_drive_local_odom_topic", "/local_state/odometry")' in node_script
    assert "on_spin_to_drive_local_odom" in node_script
    assert "on_spin_to_drive_imu" in node_script
    assert "spin_to_drive_local_odom_stable()" in node_script
    assert "spin_to_drive_imu_stable()" in node_script
    assert "spin_to_drive_actual_wz_stable() && spin_to_drive_local_odom_stable() &&" in node_script
    assert "spin_to_drive_imu_stable())" in node_script
    assert "spin_to_drive_linear_hold_started_time_" in node_script
    assert "spin_to_drive_local_stable_since_time_" in node_script
    assert "spin_to_drive_imu_stable_since_time_" in node_script
    assert 'declare_parameter<double>("spin_to_drive_local_stable_duration_sec", 0.30)' in node_script
    assert 'declare_parameter<bool>("spin_to_drive_require_imu_stable", true)' in node_script
    assert 'declare_parameter<std::string>("spin_to_drive_imu_topic", "/lidar_imu_bias_corrected")' in node_script
    assert 'declare_parameter<double>("spin_to_drive_imu_wz_threshold_radps", 0.035)' in node_script
    assert 'declare_parameter<double>("spin_to_drive_imu_stable_duration_sec", 0.30)' in node_script
    assert 'declare_parameter<double>("spin_to_drive_imu_max_age_sec", 0.10)' in node_script
    assert 'declare_parameter<double>("spin_to_drive_timeout_sec", 2.0)' in node_script
    assert "spin_to_drive settle timed out while holding linear drive" in node_script
    assert 'declare_parameter<bool>("mode_exit_guard_enabled", true)' in node_script
    assert 'declare_parameter<std::string>("mode_controller_status_topic", "/ranger_base/status")' in node_script
    assert "on_mode_controller_status" in node_script
    assert "parse_actual_motion_mode_code" in node_script
    assert "actual_motion_mode_code_ == 1 || actual_motion_mode_code_ == 3" in node_script
    assert "apply_mode_exit_guard" in node_script
    assert "sanitize_command_for_mode_contract" in node_script
    assert "allow_api_lateral_cmd" in node_script
    assert "api_lateral_max_mps" in node_script
    assert "source == CommandSource::DOCKING" in node_script
    assert "source == CommandSource::API && allow_api_lateral_cmd_" in node_script
    assert "sanitized.linear.y = 0.0" in node_script
    assert "std::clamp(" in node_script
    assert "mode_exit_guard releasing stale lateral motion_mode" in node_script
    assert "mode_exit_guard probing DUAL_ACKERMAN" in node_script
    assert "std::max(std::abs(cmd.linear.x), mode_exit_guard_probe_speed_mps_)" in node_script
    assert "bool normal_command_fresh() const" in node_script
    assert "actual_motion_mode_is_lateral() && !normal_command_fresh()" in node_script
    assert "ROS_DOMAIN_ID=182" in lateral_watchdog_wrapper
    assert "/lateral_guard_test/output" in lateral_watchdog_wrapper
    assert "fresh normal lateral stream was interrupted by safety-timer zeros" in lateral_watchdog_smoke
    assert "stale normal lateral stream did not produce watchdog zero" in lateral_watchdog_smoke
    assert 'declare_parameter<std::string>("api_cmd_vel_in_topic", "/cmd_vel_api")' in node_script
    assert 'declare_parameter<bool>("enable_api_cmd_priority", true)' in node_script
    assert 'declare_parameter<std::string>("docking_cmd_vel_in_topic", "/cmd_vel_docking")' in node_script
    assert 'declare_parameter<bool>("enable_docking_cmd_priority", true)' in node_script
    assert "api_command_fresh" in node_script
    assert "fresh_api_command_active()" in node_script
    assert "last_api_cmd_ = *msg" in node_script
    assert "have_last_api_cmd_ = true" in node_script
    assert "prepare_checked_command(last_api_cmd_, CommandSource::API)" in node_script
    assert "docking_command_fresh" in node_script
    assert "fresh_docking_command_active()" in node_script
    assert "last_docking_cmd_ = *msg" in node_script
    assert "have_last_docking_cmd_ = true" in node_script
    assert "prepare_checked_command(last_docking_cmd_, CommandSource::DOCKING)" in node_script
    assert "on_docking_cmd" in node_script
    assert "on_normal_cmd" in node_script

    sdk = (
        ROOT
        / "src"
        / "ranger_base"
        / "src"
        / "ranger_messenger.cpp"
    ).read_text(encoding="utf-8")
    assert "kCmdVelLateralDeadband" in sdk
    assert "msg->linear.y = 0.0;" in sdk
    assert "std::abs(msg->linear.y) > kCmdVelLateralDeadband" in sdk


def test_robot_bringup_wires_repo_owned_localization_and_navigation_launches():
    cmake = (ROOT / "src" / "robot_bringup" / "CMakeLists.txt").read_text(encoding="utf-8")
    localization_launch = (ROOT / "src" / "robot_bringup" / "launch" / "localization_bringup.launch.py").read_text(encoding="utf-8")
    navigation_launch = (ROOT / "src" / "robot_bringup" / "launch" / "navigation_bringup.launch.py").read_text(encoding="utf-8")
    standard_navigation_launch = (ROOT / "src" / "robot_bringup" / "launch" / "standard_navigation.launch.py").read_text(encoding="utf-8")
    local_costmap_debug_launch = (ROOT / "src" / "robot_bringup" / "launch" / "local_costmap_debug.launch.py").read_text(encoding="utf-8")
    bringup_cfg = (ROOT / "src" / "robot_bringup" / "config" / "bringup.yaml").read_text(encoding="utf-8")
    bringup_readme = (ROOT / "src" / "robot_bringup" / "README.md").read_text(encoding="utf-8")
    package_xml = (ROOT / "src" / "robot_bringup" / "package.xml").read_text(encoding="utf-8")
    assert "nav2_map_server" in localization_launch
    assert 'include("robot_floor_manager", "floor_manager.launch.py")' in localization_launch
    assert 'include("robot_safety", "robot_safety.launch.py")' in localization_launch
    assert "standard_navigation.launch.py" in navigation_launch
    assert 'package="nav2_controller"' in standard_navigation_launch
    assert 'package="nav2_velocity_smoother"' in standard_navigation_launch
    assert 'package="nav2_collision_monitor"' in standard_navigation_launch
    assert 'package="nav2_controller"' in local_costmap_debug_launch
    assert 'package="nav2_lifecycle_manager"' in local_costmap_debug_launch
    assert 'name="lifecycle_manager_local_costmap_debug"' in local_costmap_debug_launch
    assert '("cmd_vel", "cmd_vel_local_costmap_debug")' in local_costmap_debug_launch
    assert 'package="nav2_planner"' not in local_costmap_debug_launch
    assert 'package="nav2_bt_navigator"' not in local_costmap_debug_launch
    assert 'package="nav2_collision_monitor"' not in local_costmap_debug_launch
    assert '("cmd_vel", "cmd_vel_nav_raw")' in standard_navigation_launch
    assert '("cmd_vel_smoothed", "cmd_vel_nav")' in standard_navigation_launch
    assert '("cmd_vel", "cmd_vel_nav")' in standard_navigation_launch
    assert "load_map_server" in localization_launch
    assert "nav2_params_file" in bringup_cfg
    assert "navigation_bringup.launch.py" in bringup_readme
    assert "standard_navigation.launch.py" in bringup_readme
    assert "add_executable(runtime_readiness_probe" in cmake
    assert "src/runtime_readiness_probe.cpp" in cmake
    assert "install(TARGETS runtime_readiness_probe" in cmake
    assert "<depend>rclcpp</depend>" in package_xml
    assert "<depend>lifecycle_msgs</depend>" in package_xml
    assert "<depend>tf2_ros</depend>" in package_xml
    assert "<exec_depend>nav2_bringup</exec_depend>" in package_xml
    assert "<exec_depend>nav2_collision_monitor</exec_depend>" in package_xml
    assert "<exec_depend>robot_floor_manager</exec_depend>" in package_xml


def test_floor_manager_package_and_asset_contracts_exist():
    package_root = ROOT / "src" / "robot_floor_manager"
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
    node_cpp = (package_root / "src" / "floor_manager_node.cpp").read_text(encoding="utf-8")
    config = (package_root / "config" / "floor_manager.yaml").read_text(encoding="utf-8")
    map_toolkit = (ROOT / "src" / "robot_map_toolkit" / "scripts" / "map_toolkit_cli.py").read_text(encoding="utf-8")
    interfaces_cmake = (ROOT / "src" / "robot_interfaces" / "CMakeLists.txt").read_text(encoding="utf-8")
    switch_floor_srv = (
        ROOT / "src" / "robot_interfaces" / "srv" / "SwitchFloor.srv"
    ).read_text(encoding="utf-8")
    assert (package_root / "launch" / "floor_manager.launch.py").exists()
    assert "add_executable(floor_manager_node src/floor_manager_node.cpp)" in cmake
    assert "lifecycle_msgs" in cmake
    assert "<depend>lifecycle_msgs</depend>" in package_xml
    assert "nav2_msgs" in package_xml
    assert "SwitchFloor" in node_cpp
    assert "/floor_manager/switch_floor" in node_cpp
    assert "FloorSwitchAction" in node_cpp
    assert "/floor_manager/floor_switch" in node_cpp
    assert "FloorSwitchStatus" in node_cpp
    assert "/floor_manager/transition_status" in node_cpp
    assert "live_floor_switch_enabled: false" in config
    assert "LEGACY_RESUME_NAVIGATION_DISABLED" in node_cpp
    assert "nav2_msgs::srv::ClearEntireCostmap" in node_cpp
    assert "std_srvs::srv::Empty" not in node_cpp
    assert "rclcpp_action" in cmake
    assert "<depend>rclcpp_action</depend>" in package_xml
    assert "/map_server/load_map" in node_cpp
    assert "/global_localization/apply_floor_assets" in node_cpp
    assert "/global_localization/trigger" in node_cpp
    assert "clear_entirely_global_costmap" in node_cpp
    assert '"nav" / "nav_map.yaml"' in node_cpp
    assert '"localizer" / "localizer_map.png"' in node_cpp
    assert '"filters" / "keepout_mask.yaml"' in node_cpp
    assert "poses.yaml" in node_cpp
    assert "require_filter_assets: true" in config
    assert "srv/SwitchFloor.srv" in interfaces_cmake
    assert "string map_id" in switch_floor_srv
    assert "uint64 expected_asset_epoch" in switch_floor_srv
    assert "string expected_asset_digest" in switch_floor_srv
    assert "string selected_map_id" in switch_floor_srv
    assert "uint64 asset_epoch" in switch_floor_srv
    assert "string asset_digest" in switch_floor_srv
    assert "FloorAssetSnapshotLoader" in node_cpp
    assert "selected_map_id_ = assets.map_id" not in node_cpp
    assert 'publish_status(success ? "idle" : ("failed:" + message))' in node_cpp
    assert 'publish_status("switching:" + map_key)' not in node_cpp
    assert "floor source preflight verified; API activation is still required" in node_cpp
    assert "test_floor_manager_exact_selection" in cmake
    assert "action/FloorSwitch.action" in interfaces_cmake
    assert "msg/FloorSwitchStatus.msg" in interfaces_cmake
    assert "REQUIRED_RELATIVE_ASSETS" in map_toolkit
    assert "--flat-map-name" in map_toolkit
    assert "promote_flat_map" in map_toolkit
    assert "localizer/localizer_params.yaml" in map_toolkit


def test_live_floor_switch_retries_only_proven_pre_dispatch_localization_failures():
    floor_root = ROOT / "src" / "robot_floor_manager"
    policy_header = (
        floor_root
        / "include"
        / "robot_floor_manager"
        / "floor_transition_reconciliation.hpp"
    ).read_text(encoding="utf-8")
    policy_source = (
        floor_root / "src" / "floor_transition_reconciliation.cpp"
    ).read_text(encoding="utf-8")
    floor_node = (floor_root / "src" / "floor_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    policy_test = (
        floor_root / "test" / "test_floor_transition_reconciliation.cpp"
    ).read_text(encoding="utf-8")

    assert "classify_explicit_localization_failure" in policy_header
    for code in (
        "LOCALIZER_POST_RELOAD_NOT_READY",
        "LOCALIZER_OPERATION_BUSY",
        "LOCALIZER_INPUT_NOT_FRESH",
        "ISAAC_SERVICE_UNAVAILABLE",
        "BRIDGE_FORCE_ACCEPT_UNAVAILABLE",
        "BRIDGE_FORCE_ACCEPT_TIMEOUT",
    ):
        assert code in policy_source
        assert code in policy_test
    for code in (
        "FRESH_LOCALIZATION_RETRY_REQUIRED",
        "ISAAC_DISPATCH_TIMEOUT",
        "LOCALIZATION_RESULT_TIMEOUT",
        "BRIDGE_ACCEPT_TIMEOUT",
        "MAP_TO_ODOM_TIMEOUT",
    ):
        assert code not in policy_source
        assert code in policy_test
    assert "dispatch_state=not_dispatched" in policy_source
    assert "MAP_TO_ODOM_WRONG_OWNER" in policy_test
    assert "TF_HISTORY_MISSING" in policy_test

    trigger_body = floor_node.split(
        "bool trigger_localization(const FloorAssets & assets, std::string & error)", 1
    )[1].split("\n  bool clear_costmap(", 1)[0]
    deadline_index = trigger_body.index("const auto deadline =")
    retry_loop_index = trigger_body.index(
        "while (!shutting_down_.load() && !cancel_requested_.load())"
    )
    assert deadline_index < retry_loop_index
    assert trigger_body.count("const auto deadline =") == 1
    assert "classify_explicit_localization_failure(error)" in trigger_body
    assert "retryable explicit localization failure kept inside floor transaction" in trigger_body
    assert "std::chrono::milliseconds(50)" in trigger_body
    assert "s total retry budget after " in trigger_body


def test_floor_switch_pause_handoff_is_transaction_scoped_and_level_triggered():
    action = (
        ROOT / "src" / "robot_interfaces" / "action" / "FloorSwitch.action"
    ).read_text(encoding="utf-8")
    floor_header = (
        ROOT
        / "src"
        / "robot_floor_manager"
        / "include"
        / "robot_floor_manager"
        / "floor_transition_executor.hpp"
    ).read_text(encoding="utf-8")
    floor_source = (
        ROOT / "src" / "robot_floor_manager" / "src" / "floor_transition_executor.cpp"
    ).read_text(encoding="utf-8")
    floor_node = (
        ROOT / "src" / "robot_floor_manager" / "src" / "floor_manager_node.cpp"
    ).read_text(encoding="utf-8")
    api_source = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "elevator"
        / "execution"
        / "elevator_ros_runtime_port.cpp"
    ).read_text(encoding="utf-8")
    api_cmake = (
        ROOT / "src" / "robot_api_server" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")

    assert "string transaction_id" in action.split("---")[-1]
    assert "uint64 stage_sequence" in action.split("---")[-1]
    assert "bool caller_pause_handoff_ready" in action.split("---")[-1]
    assert "floor_transition_caller_pause_handoff_ready" in floor_header
    assert "FloorTransitionState::kReportingBeginReady ||" in floor_source
    assert "FloorTransitionState::kVerifyingPauseHandoff" in floor_source
    assert "feedback->transaction_id = request.transaction_id" in floor_node
    assert "feedback->stage_sequence = output.effect.sequence" in floor_node
    assert "feedback->caller_pause_handoff_ready" in floor_node
    assert "FloorSwitchHandoffTracker floor_handoff_tracker_" in api_source
    assert "feedback->stage == kVerifyPauseHandoffStage" in api_source
    assert "floor_handoff_tracker_.snapshot().ready" in api_source
    assert "src/features/floor_switch/floor_switch_handoff_tracker.cpp" in api_cmake
    assert "test_floor_switch_handoff_tracker" in api_cmake


def test_floor_switch_cold_start_nav_idle_requires_stable_action_graph():
    floor_root = ROOT / "src" / "robot_floor_manager"
    evidence_header = (
        floor_root
        / "include"
        / "robot_floor_manager"
        / "floor_transition_evidence_tracker.hpp"
    ).read_text(encoding="utf-8")
    evidence_source = (
        floor_root / "src" / "floor_transition_evidence_tracker.cpp"
    ).read_text(encoding="utf-8")
    floor_node = (floor_root / "src" / "floor_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    floor_config = (floor_root / "config" / "floor_manager.yaml").read_text(
        encoding="utf-8"
    )
    floor_test = (
        floor_root / "test" / "test_floor_transition_evidence_tracker.cpp"
    ).read_text(encoding="utf-8")
    runtime_launcher = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_floor_manager.sh"
    ).read_text(encoding="utf-8")

    assert "observe_nav_graph_ready" in evidence_header
    assert "cold_start_proves_idle" in evidence_source
    assert "nav_graph_ready_ &&" in evidence_source
    assert "nav_active_.received_steady_sec < 0.0" in evidence_source
    assert "action_server_is_ready()" in floor_node
    assert "count_publishers(navigate_to_pose_status_topic_) > 0U" in floor_node
    assert 'declare_parameter<double>("nav_idle_bootstrap_grace_sec", 2.0)' in floor_node
    assert "nav_idle_bootstrap_grace_sec: 2.0" in floor_config
    assert "NJRH_NAV_IDLE_BOOTSTRAP_GRACE_SEC:-2.0" in runtime_launcher
    assert "ProvesColdStartIdleAfterStableActionServerWithoutStatus" in floor_test
    assert "Any later active status overrides" in floor_test
    assert "Losing the action server invalidates" in floor_test


def test_manual_live_floor_switch_http_wraps_only_the_strict_action():
    api_root = ROOT / "src" / "robot_api_server"
    entry_source = (api_root / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    floor_switch_source = (
        api_root / "src" / "features" / "floor_switch" / "floor_switch_module.cpp"
    ).read_text(
        encoding="utf-8"
    )
    transaction_source = (
        api_root / "src" / "features" / "floor_switch" / "floor_switch_http_transaction.cpp"
    ).read_text(encoding="utf-8")
    api_cmake = (api_root / "CMakeLists.txt").read_text(encoding="utf-8")
    overlay_config = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")

    for method, endpoint in (
        ("POST", "/api/v1/floor-switch/start"),
        ("GET", "/api/v1/floor-switch/state"),
        ("POST", "/api/v1/floor-switch/cancel"),
    ):
        assert f'request.method == "{method}" && request.path == "{endpoint}"' in floor_switch_source
        assert endpoint not in entry_source
    live_body = floor_switch_source.split(
        "HttpResponse handle_live_start", 1
    )[1].split("HttpResponse handle_live_state", 1)[0]
    assert "live_client_->async_send_goal" in live_body
    assert "goal.expected_asset_epoch = manifest->asset_epoch" in live_body
    assert "goal.expected_asset_digest = manifest->asset_digest" in live_body
    assert "startNavigation" not in live_body
    assert "handle_offline_switch(" not in live_body
    assert "activate_map_manifest" not in live_body
    assert 'live_floor_switch_action: "/floor_manager/floor_switch"' in overlay_config
    assert "FLOOR_SWITCH_TRANSACTION_BUSY" in transaction_source
    assert "FLOOR_SWITCH_RECOVERY_REQUIRED" in transaction_source
    assert "runtime_context_valid" in transaction_source
    assert "src/features/floor_switch/floor_switch_http_transaction.cpp" in api_cmake
    assert "test_floor_switch_http_transaction" in api_cmake


def test_p6_floor_transition_negative_interlock_contract():
    api_root = ROOT / "src" / "robot_api_server"
    entry_code = (api_root / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    floor_switch_code = (
        api_root / "src" / "features" / "floor_switch" / "floor_switch_module.cpp"
    ).read_text(encoding="utf-8")
    docking_http_code = (
        api_root
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_http_module.cpp"
    ).read_text(encoding="utf-8")
    maps_code = (api_root / "src" / "features" / "maps" / "maps_module.cpp").read_text(
        encoding="utf-8"
    )
    feature_code = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted((api_root / "src" / "features").rglob("*.cpp"))
    )
    api_cmake = (api_root / "CMakeLists.txt").read_text(encoding="utf-8")
    api_config = (api_root / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    overlay_api_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    interlock_header = (
        api_root
        / "include"
        / "robot_api_server"
        / "features"
        / "floor_switch"
        / "floor_runtime_interlock.hpp"
    ).read_text(encoding="utf-8")
    interlock_source = (
        api_root / "src" / "features" / "floor_switch" / "floor_runtime_interlock.cpp"
    ).read_text(encoding="utf-8")
    interlock_test = (
        api_root / "test" / "features" / "floor_switch" / "test_floor_runtime_interlock.cpp"
    ).read_text(encoding="utf-8")
    floor_http_smoke = (
        api_root / "test" / "features" / "floor_switch" / "floor_runtime_http_smoke.py"
    ).read_text(encoding="utf-8")

    assert "robot_interfaces/msg/floor_switch_status.hpp" in floor_switch_code
    assert "robot_interfaces/msg/localization_health.hpp" in floor_switch_code
    assert "create_subscription<robot_interfaces::msg::FloorSwitchStatus>" in floor_switch_code
    assert "create_subscription<robot_interfaces::msg::LocalizationHealth>" in floor_switch_code
    assert "FLOOR_TRANSITION_BLOCKED" in floor_switch_code
    assert "floor_runtime_negative_interlock_enabled" in feature_code
    for operation in (
        "navigation_goal",
        "navigation_goal_submit",
        "mapping_start",
        "mapping_process_launch",
        "mapping_save",
        "map_delete",
        "keepout_update",
        "floor_switch",
        "manual_localization",
        "docking_start",
        "docking_undock",
        "safety_resume",
    ):
        assert f'"{operation}"' in feature_code
        assert f'"{operation}"' not in entry_code

    switch_body = floor_switch_code.split(
        "HttpResponse handle_offline_switch", 1
    )[1].split("rclcpp::Node & node_", 1)[0]
    assert "LIVE_FLOOR_SWITCH_DISABLED" in switch_body
    assert switch_body.index("if (resume_navigation)") < switch_body.index(
        "map_catalog_.find_map_by_id"
    )
    assert "return handle_resume_floor_navigation" not in switch_body
    assert switch_body.index("future.get()") < switch_body.index(
        "ports_.activate_map_manifest(*selected_map, *selection_transaction)"
    )
    assert "FLOOR_SELECTION_RUNTIME_BUSY" in floor_switch_code
    assert switch_body.count("offline_runtime_busy_detail()") >= 2
    assert "runtime_map_already_selected" in switch_body
    assert r'\"already_active\":true' in floor_switch_code
    assert r'\"runtime_unchanged\":true' in floor_switch_code
    assert r'\"selection_performed\":false' in floor_switch_code
    assert "floor_switch_noop_commit" in switch_body
    assert switch_body.index("clear_runtime_context()") < switch_body.index(
        "ports_.activate_map_manifest(*selected_map, *selection_transaction)"
    )
    assert 'environment["ROS_DOMAIN_ID"] = "229"' in floor_http_smoke
    assert 'environment["ROS_LOCALHOST_ONLY"] = "1"' in floor_http_smoke
    assert "ActionServer(" in floor_http_smoke
    assert "snapshot_file_tree" in floor_http_smoke
    assert "switch_floor_calls" in floor_http_smoke
    assert "same_runtime_map_selection_is_noop" in floor_http_smoke
    assert "different_runtime_map_selection_remains_blocked" in floor_http_smoke
    assert floor_http_smoke.count("last_navigation_map_file:=") >= 2
    assert 'last_navigation_map_path = root / "last_navigation_map.json"' in floor_http_smoke

    docking_body = docking_http_code.split(
        "HttpResponse handle_start", 1
    )[1].split("HttpResponse handle_cancel", 1)[0]
    assert "FLOOR_SWITCH_REQUIRED" in docking_http_code
    assert "maps_module_.runtime_context_matches" in docking_body
    assert "activate_map_manifest(*selected_map)" not in docking_body

    delete_body = maps_code.split(
        "HttpResponse handle_delete_map", 1
    )[1].split("HttpResponse handle_get_keepout_filter", 1)[0]
    assert "ACTIVE_MAP_DELETE_DISABLED" in delete_body
    assert "activate_map_manifest(remaining.front())" not in delete_body

    assert "FloorRuntimeInterlock" in interlock_header
    assert "FLOOR_RUNTIME_LEGACY_UNSCOPED" in interlock_header
    assert "FAILED_LOCKED" in interlock_source
    assert "PREFLIGHT" in interlock_test
    assert "BLOCKED" in interlock_test
    assert "MissingTypedEvidencePreservesLegacyRuntime" in interlock_test
    assert "test_floor_runtime_interlock" in api_cmake
    assert "src/features/floor_switch/floor_runtime_interlock.cpp" in api_cmake
    for config in (api_config, overlay_api_config):
        assert 'floor_transition_status_topic: "/floor_manager/transition_status"' in config
        assert "floor_runtime_negative_interlock_enabled: true" in config
        assert 'localization_floor_health_topic: "/localization/floor_health"' in config

    bridge_root = ROOT / "src" / "robot_localization_bridge"
    bridge_code = (bridge_root / "src" / "localization_bridge_node.cpp").read_text(
        encoding="utf-8"
    )
    bridge_config = (bridge_root / "config" / "localization_bridge.yaml").read_text(
        encoding="utf-8"
    )
    floor_manager_code = (
        ROOT / "src" / "robot_floor_manager" / "src" / "floor_manager_node.cpp"
    ).read_text(encoding="utf-8")
    overlay_bridge_config = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    assert "robot_interfaces::srv::BeginFloorTransition" in bridge_code
    assert "robot_interfaces::msg::LocalizationHealth" in bridge_code
    assert "OP_BEGIN" in bridge_code
    assert "OP_COMMIT" in bridge_code
    assert "OP_ABORT" in bridge_code
    # Failed transaction history is not a permanent bridge lock. Invalid map
    # context still gates localization/motion until a fresh switch establishes it.
    assert "!floor.runtime_context_valid" in bridge_code
    assert "FLOOR_CONTEXT_INVALID" in bridge_code
    assert "live_floor_transition_service_enabled" in bridge_code
    assert 'declare_parameter<bool>("live_floor_transition_service_enabled", false)' in bridge_code
    disabled_bridge_smoke = (
        bridge_root / "test" / "isolated_floor_transition_disabled_smoke.py"
    ).read_text(encoding="utf-8")
    assert "LIVE_FLOOR_TRANSITION_DISABLED" in disabled_bridge_smoke
    assert "runtime_context_remained_valid" in disabled_bridge_smoke
    for config in (bridge_config, overlay_bridge_config):
        assert "floor_health_topic: /localization/floor_health" in config
        assert "live_floor_transition_service_enabled: false" in config
        assert (
            "begin_floor_transition_service: "
            "/robot_localization_bridge/begin_floor_transition"
        ) in config

    assert ".detach()" not in floor_manager_code
    assert "floor_state_mutex_" in floor_manager_code
    assert "release_floor_switch_action" in floor_manager_code
    assert floor_manager_code.count("goal_handle->is_canceling()") >= 2


def test_map_toolkit_promotes_only_pgm_nav_map(tmp_path):
    module_path = ROOT / "src" / "robot_map_toolkit" / "scripts" / "map_toolkit_cli.py"
    spec = importlib.util.spec_from_file_location("map_toolkit_cli", module_path)
    assert spec is not None and spec.loader is not None
    map_toolkit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(map_toolkit)

    flat_maps = tmp_path / "maps"
    flat_maps.mkdir()
    map_toolkit.save_pgm_from_grayscale(flat_maps / "sample.pgm", 2, 2, bytes([1, 2, 3, 4]))
    map_toolkit.save_png_from_grayscale(flat_maps / "sample.png", 2, 2, bytes([0, 128, 205, 254]))
    map_toolkit.save_png_from_grayscale(flat_maps / "sample.localizer.png", 2, 2, bytes([0, 128, 205, 254]))
    (flat_maps / "sample.yaml").write_text(
        "\n".join(
            [
                "image: sample.pgm",
                "resolution: 0.050000",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "mode: trinary",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (flat_maps / "sample.localizer.yaml").write_text(
        (flat_maps / "sample.yaml").read_text(encoding="utf-8").replace("sample.pgm", "sample.localizer.png"),
        encoding="utf-8",
    )

    floor_root = tmp_path / "maps_release" / "building_1" / "floor_1"
    map_toolkit.promote_flat_map(flat_maps, "sample", floor_root, "building_1", "floor_1", 1, 1, 0.05)

    assert (floor_root / "nav" / "nav_map.yaml").read_text(encoding="utf-8").splitlines()[0] == "image: nav_map.pgm"
    assert (floor_root / "nav" / "nav_map.pgm").read_bytes().startswith(b"P5\n2 2\n255\n")
    assert map_toolkit.load_pgm_grayscale(floor_root / "nav" / "nav_map.pgm")[2] == bytes([1, 2, 3, 4])
    assert (floor_root / "localizer" / "localizer_params.yaml").read_text(encoding="utf-8").splitlines()[0] == (
        "image: localizer_map.png"
    )
    assert (floor_root / "localizer" / "localizer_map.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert map_toolkit.load_pgm_grayscale(floor_root / "filters" / "keepout_mask.pgm")[2] == bytes([254] * 4)
    assert map_toolkit.load_pgm_grayscale(floor_root / "filters" / "speed_mask.pgm")[2] == bytes([254] * 4)
    assert map_toolkit.load_pgm_grayscale(floor_root / "filters" / "binary_mask.pgm")[2] == bytes([254] * 4)

    bad_maps = tmp_path / "bad_maps"
    bad_maps.mkdir()
    map_toolkit.save_pgm_from_grayscale(bad_maps / "bad.pgm", 2, 2, bytes([1, 2, 3, 4]))
    map_toolkit.save_png_from_grayscale(bad_maps / "bad.png", 2, 2, bytes([0, 128, 205, 254]))
    map_toolkit.save_png_from_grayscale(bad_maps / "bad.localizer.png", 2, 2, bytes([0, 128, 205, 254]))
    (bad_maps / "bad.yaml").write_text(
        (flat_maps / "sample.yaml").read_text(encoding="utf-8").replace("sample.pgm", "bad.png"),
        encoding="utf-8",
    )
    (bad_maps / "bad.localizer.yaml").write_text(
        (flat_maps / "sample.localizer.yaml").read_text(encoding="utf-8").replace(
            "sample.localizer.png", "bad.localizer.png"
        ),
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="must reference bad.pgm"):
        map_toolkit.promote_flat_map(bad_maps, "bad", tmp_path / "bad_release", "building_1", "floor_1", 1, 1, 0.05)


def test_runtime_overlay_live_2d_mapping_uses_slam_toolbox():
    run_projected_map = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_projected_map.sh").read_text(encoding="utf-8")
    common_env = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "common_env.sh").read_text(encoding="utf-8")
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    slam_launch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_slam_toolbox_mapping.launch.py").read_text(encoding="utf-8")
    slam_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_slam_toolbox_mapping.yaml").read_text(encoding="utf-8")
    slam_scan_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_scan_slam2d.yaml").read_text(encoding="utf-8")
    preprocessor_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_nav_cloud_preprocessor.yaml").read_text(encoding="utf-8")
    fastlio_odom_bridge_cpp = (
        ROOT / "src" / "robot_fastlio_mapping" / "src" / "fastlio_odom_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    mapping_scan_tf_gate_cpp = (
        ROOT / "src" / "robot_fastlio_mapping" / "src" / "mapping_scan_tf_gate_node.cpp"
    ).read_text(encoding="utf-8")
    fastlio_mapping_cmake = (
        ROOT / "src" / "robot_fastlio_mapping" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    fastlio_mapping_package = (
        ROOT / "src" / "robot_fastlio_mapping" / "package.xml"
    ).read_text(encoding="utf-8")
    fastlio_pair_probe = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_mapping_fastlio_pair.py"
    ).read_text(encoding="utf-8")
    readiness_probe_cpp = (
        ROOT / "src" / "robot_bringup" / "src" / "runtime_readiness_probe.cpp"
    ).read_text(encoding="utf-8")
    stamped_scan_tf_smoke = (
        ROOT
        / "src"
        / "robot_bringup"
        / "test"
        / "run_isolated_stamped_scan_tf_smoke.sh"
    ).read_text(encoding="utf-8")
    scan_republisher_cpp = (ROOT / "src" / "robot_hesai_jt128" / "src" / "scan_republisher_node.cpp").read_text(encoding="utf-8")
    hesai_cmake = (ROOT / "src" / "robot_hesai_jt128" / "CMakeLists.txt").read_text(encoding="utf-8")
    hesai_package = (ROOT / "src" / "robot_hesai_jt128" / "package.xml").read_text(encoding="utf-8")
    assert "ros2 launch" in run_projected_map
    assert "jt128_slam_toolbox_mapping.launch.py" in run_projected_map
    assert "require_mapping_ros_package || exit 1" in run_projected_map
    assert 'get_package_prefix("robot_fastlio_mapping")' in run_projected_map
    assert run_projected_map.index("require_mapping_ros_package || exit 1") < (
        run_projected_map.index("stop_legacy_mapping_processes() {")
    )
    assert "import os" in slam_launch
    assert "os.environ.get" in slam_launch
    assert "require_resident_common_mapping_prereqs" in run_projected_map
    assert "run_robot_description.sh" not in run_projected_map
    assert "run_local_state.sh" not in run_projected_map
    assert "stop_existing_canonical_tf_publishers" not in run_projected_map
    assert "start_canonical_helper" not in run_projected_map
    assert 'common_mode="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}"' in run_projected_map
    assert "resident robot_local_state endpoint is not ready; start common services before mapping" in run_projected_map
    assert 'SLAM2D_ODOM_SOURCE="${NJRH_SLAM2D_ODOM_SOURCE:-fastlio}"' in run_projected_map
    assert 'SLAM2D_ALLOW_PRIVATE_FASTLIO="${NJRH_SLAM2D_ALLOW_PRIVATE_FASTLIO:-true}"' in run_projected_map
    assert 'SLAM2D_REUSE_EXISTING_FASTLIO="${NJRH_SLAM2D_REUSE_EXISTING_FASTLIO:-false}"' in run_projected_map
    assert 'FASTLIO_ODOM_TOPIC="${NJRH_SLAM2D_FASTLIO_ODOM_TOPIC:-/mapping/fastlio/odometry}"' in run_projected_map
    assert 'SLAM2D_PRIVATE_TF_TOPIC="${NJRH_SLAM2D_PRIVATE_TF_TOPIC:-/tf_slam2d}"' in run_projected_map
    assert 'SLAM2D_FASTLIO_ODOM_FRAME="${NJRH_SLAM2D_FASTLIO_ODOM_FRAME:-mapping_odom}"' in run_projected_map
    assert 'POINTS_TOPIC="${NJRH_SLAM2D_POINTS_TOPIC:-/mapping/fastlio/cloud_registered_body}"' in run_projected_map
    assert "FASTLIO_CONFIG_FILE" in run_projected_map
    assert 'LOCAL_ODOM_READY_TIMEOUT="${NJRH_SLAM2D_ODOM_READY_TIMEOUT:-30}"' in run_projected_map
    assert 'FASTLIO_POINTS_READY_TIMEOUT="${NJRH_SLAM2D_FASTLIO_POINTS_READY_TIMEOUT:-60}"' in run_projected_map
    assert 'FASTLIO_POINTS_MAX_AGE_SEC="${NJRH_SLAM2D_FASTLIO_POINTS_MAX_AGE_SEC:-1.0}"' in run_projected_map
    assert 'LOCAL_ODOM_MAX_AGE_SEC="${NJRH_SLAM2D_LOCAL_ODOM_MAX_AGE_SEC:-1.0}"' in run_projected_map
    assert 'LOCAL_ODOM_MAX_WHEEL_DIFF_M="${NJRH_SLAM2D_LOCAL_ODOM_MAX_WHEEL_DIFF_M:-25.0}"' in run_projected_map
    assert 'SLAM2D_LIDAR_RPS_XPS_ENABLED="${NJRH_SLAM2D_LIDAR_RPS_XPS_ENABLED:-true}"' in run_projected_map
    assert 'SLAM2D_LIDAR_RPS_XPS_INTERFACE="${NJRH_SLAM2D_LIDAR_RPS_XPS_INTERFACE:-${NJRH_LIDAR_INTERFACE:-eth1}}"' in run_projected_map
    assert 'SLAM2D_LIDAR_RPS_XPS_CPUSET="${NJRH_SLAM2D_LIDAR_RPS_XPS_CPUSET:-5}"' in run_projected_map
    assert "apply_slam2d_lidar_rps_xps" in run_projected_map
    assert "restore_slam2d_lidar_rps_xps" in run_projected_map
    assert 'export NJRH_CPUSET_FASTLIO_DESKEW="${NJRH_SLAM2D_FASTLIO_CPUSET:-${NJRH_CPUSET_MAPPING_BACKEND:-7}}"' in run_projected_map
    assert 'export NJRH_CPUSET_SLAM_TOOLBOX_MAPPING="${NJRH_SLAM2D_SLAM_TOOLBOX_CPUSET:-3,7}"' in run_projected_map
    assert "ros2 run fast_lio fastlio_mapping" in run_projected_map
    normalized_mapping_runner = re.sub(
        r"\s+", " ", re.sub(r"\\\s*\n\s*", " ", run_projected_map)
    )
    assert (
        "njrh_start_affined_background fastlio_deskew_pid "
        "fastlio_deskew env -u FASTDDS_BUILTIN_TRANSPORTS "
        'FASTRTPS_DEFAULT_PROFILES_FILE="${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE}" '
        'FASTDDS_DEFAULT_PROFILES_FILE="${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE}" '
        "NJRH_SLAM2D_FASTDDS_ROLE=large_cloud "
        "ros2 run fast_lio fastlio_mapping"
    ) in normalized_mapping_runner
    assert "njrh_run_affined fastlio_deskew ros2 run fast_lio fastlio_mapping" not in run_projected_map
    assert '-r /cloud_registered_body:="${POINTS_TOPIC}"' in run_projected_map
    assert '-r /Odometry:="${FASTLIO_ODOM_TOPIC}"' in run_projected_map
    assert "-r /tf:=/tf_fastlio_internal" in run_projected_map
    assert 'verify_mapping_fastlio_pair.py' in run_projected_map
    assert '"${POINTS_TOPIC}" "${FASTLIO_ODOM_TOPIC}"' in run_projected_map
    assert "get_publishers_info_by_topic" in fastlio_pair_probe
    assert "exactly one publisher" in fastlio_pair_probe
    assert "same ROS node" in fastlio_pair_probe
    assert 'reusing existing FAST-LIO2 mapping source: ${POINTS_TOPIC}' in run_projected_map
    assert "FAST-LIO2 is required for mapping and mapping-owned startup is disabled" in run_projected_map
    assert 'if [[ "${SLAM2D_ALLOW_PRIVATE_FASTLIO}" != "true" ]]; then' in run_projected_map
    initial_cleanup = run_projected_map.split("require_can_interface_up", 1)[0]
    pre_runtime_cleanup = initial_cleanup.split("stop_fastlio_deskew_sources()", 1)[0]
    assert '"fast_lio/lib/fast_lio/fastlio_mapping"' not in initial_cleanup
    assert '"fastlio_mapping --ros-args"' not in initial_cleanup
    assert '"laser_mapping"' not in initial_cleanup
    assert "NJRH_SLAM2D_PRIVATE_FASTLIO=1" in run_projected_map
    assert "fastlio_reused_for_slam2d" in run_projected_map
    assert "stop_fastlio_deskew_sources" in run_projected_map
    assert "stop_mapping_fastlio_processes" in run_projected_map
    private_fastlio_cleanup = run_projected_map.split("stop_fastlio_deskew_sources()", 1)[1].split(
        "terminate_child()", 1
    )[0]
    assert "SLAM2D_PRIVATE_FASTLIO_PID_FILE" in run_projected_map
    assert "NJRH_SLAM2D_PRIVATE_FASTLIO=1" in run_projected_map
    assert 'refusing to stop FAST-LIO2 pid=${pid}: missing slam2d private marker' in run_projected_map
    assert '"run_fastlio_tf.sh"' not in private_fastlio_cleanup
    assert '"laser_mapping"' not in private_fastlio_cleanup
    assert '"fastlio_mapping"' not in private_fastlio_cleanup
    assert '"ros2 run fast_lio fastlio_mapping"' not in private_fastlio_cleanup
    assert "terminate_child" in run_projected_map
    assert 'wait_for_fresh_header_topic_message "${POINTS_TOPIC}" "${FASTLIO_POINTS_READY_TIMEOUT}" "${FASTLIO_POINTS_MAX_AGE_SEC}" 0.25' in run_projected_map
    assert 'runtime_readiness_probe mapping-fastlio-ready' in run_projected_map
    assert run_projected_map.index('log_mapping_startup_stage "odom_bridge_started"') < run_projected_map.index(
        'wait_for_fresh_header_topic_message "${POINTS_TOPIC}" "${FASTLIO_POINTS_READY_TIMEOUT}"')
    assert 'bridge_bin="${NJRH_PROJECT_ROOT}/install/robot_fastlio_mapping/lib/robot_fastlio_mapping/fastlio_odom_bridge_node"' in run_projected_map
    assert (
        'njrh_start_affined_background fastlio_odom_bridge_pid '
        'fastlio_odom_bridge "${bridge_bin}"'
    ) in normalized_mapping_runner
    assert 'njrh_run_affined fastlio_odom_bridge "${bridge_bin}"' not in run_projected_map
    assert 'python3 "${bridge_script}"' not in run_projected_map
    assert '"${POINTS_TOPIC}" "${FASTLIO_ODOM_TOPIC}" /mapping/fastlio_odometry' in run_projected_map
    assert 'odom_frame:="${slam_odom_frame}"' in run_projected_map
    assert 'tf_topic:="${slam_tf_topic}"' in run_projected_map
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' not in run_projected_map
    assert "wait_for_ekf_local_state_inputs" not in run_projected_map
    assert "wait_for_ekf_local_state_ready" not in run_projected_map
    assert "restart_robot_local_state_for_mapping" not in run_projected_map
    assert "starting a fresh local-state stack" not in run_projected_map
    assert 'wait_for_topic_message "/local_state/imu_bias" "${bias_timeout}"' not in run_projected_map
    assert 'SLAM2D_COMPOSITE_PREFLIGHT_ENABLED="${NJRH_SLAM2D_COMPOSITE_PREFLIGHT_ENABLED:-true}"' in run_projected_map
    assert "require_resident_common_mapping_prereqs_legacy()" in run_projected_map
    primary_preflight = run_projected_map.split(
        "require_resident_common_mapping_prereqs() {", 1
    )[1].split("\n}\n\ncleanup()", 1)[0]
    assert "runtime_readiness_probe mapping-preflight" in primary_preflight
    assert primary_preflight.count("runtime_readiness_probe") == 1
    assert 'if [[ "${SLAM2D_COMPOSITE_PREFLIGHT_ENABLED}" != "true" ]]; then' in primary_preflight
    assert "require_resident_common_mapping_prereqs_legacy" in primary_preflight
    assert "wait_for_tf_edge" not in primary_preflight
    assert "wait_for_topic_publisher" not in primary_preflight
    assert "wait_for_fresh_header_topic_message" not in primary_preflight
    assert "check_local_state_odom_sane" not in primary_preflight
    assert "bool wait_for_mapping_preflight(" in readiness_probe_cpp
    assert 'command == "mapping-preflight"' in readiness_probe_cpp
    assert "mapping-preflight <scan_topic> <scan_owner_node>" in readiness_probe_cpp
    assert 'mapping-preflight)' in common_env
    assert 'tf_timeout="${7:-0}"' in common_env
    assert 'scan_timeout="${8:-0}"' in common_env
    assert 'odom_timeout="${9:-0}"' in common_env
    mapping_pipeline_cleanup = run_projected_map.split(
        "stop_mapping_pipeline_processes() {", 1
    )[1].split("\n}\n\nterminate_child()", 1)[0]
    assert 'mapfile -t candidates < <(pgrep -f "${candidate_pattern}"' in (
        mapping_pipeline_cleanup
    )
    assert "for proc in /proc/[0-9]*" not in mapping_pipeline_cleanup
    assert "process_has_exact_env_marker" in mapping_pipeline_cleanup
    assert "stop_legacy_mapping_processes() {" in run_projected_map
    legacy_mapping_cleanup = run_projected_map.split(
        "stop_legacy_mapping_processes() {", 1
    )[1].split("\n}\n\nrequire_can_interface_up", 1)[0]
    assert 'mapfile -t candidates < <(pgrep -f "${candidate_pattern}"' in (
        legacy_mapping_cleanup
    )
    assert "pkill" not in legacy_mapping_cleanup
    assert "for pattern in" not in legacy_mapping_cleanup
    assert legacy_mapping_cleanup.index(
        '[[ ${#pids[@]} -gt 0 ]] || return 0'
    ) < legacy_mapping_cleanup.index("sleep")
    mapping_fastlio_cleanup = run_projected_map.split(
        "stop_mapping_fastlio_processes() {", 1
    )[1].split("\n}\n\nstop_mapping_pipeline_processes()", 1)[0]
    assert 'mapfile -t candidates < <(pgrep -f "${candidate_pattern}"' in (
        mapping_fastlio_cleanup
    )
    assert "for proc in /proc/[0-9]*" not in mapping_fastlio_cleanup
    assert "process_has_exact_env_marker" in mapping_fastlio_cleanup
    private_fastlio_startup = run_projected_map.split(
        'log_mapping_startup_stage "fastlio_started"', 1
    )[1].split(
        'wait_for_fresh_header_topic_message "${POINTS_TOPIC}"', 1
    )[0]
    assert "sleep 2" not in private_fastlio_startup
    assert 'kill -0 "${fastlio_deskew_pid}"' in private_fastlio_startup
    for startup_stage in (
        "script_started",
        "legacy_processes_stopped",
        "mapping_pipeline_stopped",
        "mapping_preflight_ready",
        "mapping_fastlio_residuals_stopped",
        "fastlio_started",
        "fastlio_ready",
        "odom_bridge_ready",
        "stamped_scan_tf_ready",
        "slam_toolbox_started",
    ):
        assert f'log_mapping_startup_stage "{startup_stage}"' in run_projected_map
    assert 'local_odom_reference_topic="/fastlio/base_odometry"' in run_projected_map
    assert 'local_odom_reference_topic="/wheel/odom_ekf"' in run_projected_map
    assert 'refusing to start slam_toolbox mapping with unhealthy resident local odometry' in run_projected_map
    assert "slam_toolbox" in slam_launch
    assert 'DeclareLaunchArgument("scan_topic", default_value="/scan")' in slam_launch
    assert 'DeclareLaunchArgument("tf_topic", default_value="/tf")' in slam_launch
    assert '("/tf", tf_topic)' in slam_launch
    assert "f'image: {pgm_path.name}'," in dashboard_patch
    assert "'image': str(pgm_path)" in dashboard_patch
    assert "nav_cloud_preprocessor" in slam_launch
    assert "pointcloud_to_laserscan_node" in slam_launch
    assert "mapping_scan_tf_gate_node" not in slam_launch
    assert '"NJRH_SLAM2D_MAPPING_PIPELINE": "1"' in slam_launch
    assert slam_launch.count("env=cloud_transport_env") == 2
    assert slam_launch.count("additional_env=control_transport_env") == 1
    assert 'cloud_transport_env["NJRH_SLAM2D_FASTDDS_ROLE"] = "large_cloud"' in slam_launch
    assert '"NJRH_SLAM2D_FASTDDS_ROLE": "control_udp"' in slam_launch
    assert 'cloud_transport_env.pop("FASTDDS_BUILTIN_TRANSPORTS", None)' in slam_launch
    assert "stop_mapping_pipeline_processes" in run_projected_map
    assert "NJRH_SLAM2D_MAPPING_PIPELINE=1" in run_projected_map
    assert "mapping_scan_tf_gate_node|pointcloud_to_laserscan|nav_cloud_preprocessor|slam_toolbox" in run_projected_map
    assert "scan_republisher_node" not in run_projected_map
    assert "scan_republisher_node" not in slam_launch
    assert '"scan_topic": scan_topic' in slam_launch
    assert "canTransform(" in mapping_scan_tf_gate_cpp
    assert "tf_buffer_.setTransform" in mapping_scan_tf_gate_cpp
    assert 'declare_parameter<double>("max_wait_sec", 2.0)' in mapping_scan_tf_gate_cpp
    assert 'declare_parameter<double>("post_tf_settle_ms", 20.0)' in mapping_scan_tf_gate_cpp
    assert 'declare_parameter<bool>("preserve_stamp", true)' in mapping_scan_tf_gate_cpp
    assert "outgoing.header.stamp" not in mapping_scan_tf_gate_cpp
    assert '\\"preserve_stamp\\":true' in mapping_scan_tf_gate_cpp
    assert "mapping_scan_tf_gate_node src/mapping_scan_tf_gate_node.cpp" in fastlio_mapping_cmake
    assert "sensor_msgs" in fastlio_mapping_cmake
    assert "std_msgs" in fastlio_mapping_cmake
    assert "<depend>sensor_msgs</depend>" in fastlio_mapping_package
    assert "<depend>std_msgs</depend>" in fastlio_mapping_package
    assert "<build_type>ament_cmake</build_type>" in fastlio_mapping_package
    assert "motion_gate_enabled" not in slam_launch
    assert "motion_gate_enabled" not in scan_republisher_cpp
    assert "Dropping mapping scan" not in scan_republisher_cpp
    assert "tf2_msgs::msg::TFMessage" in fastlio_odom_bridge_cpp
    assert 'declare_parameter<std::string>("tf_topic", "/tf_slam2d")' in fastlio_odom_bridge_cpp
    assert 'declare_parameter<std::string>("output_odom_frame", "odom")' in fastlio_odom_bridge_cpp
    assert 'declare_parameter<bool>("publish_tf", false)' in fastlio_odom_bridge_cpp
    assert 'declare_parameter<bool>("input_reliable", false)' in fastlio_odom_bridge_cpp
    assert 'declare_parameter<bool>("output_reliable", true)' in fastlio_odom_bridge_cpp
    assert "lookupTransform(" in fastlio_odom_bridge_cpp
    assert "odom.header.frame_id = output_odom_frame_" in fastlio_odom_bridge_cpp
    assert "tf.header = odom.header" in fastlio_odom_bridge_cpp
    assert "nav_msgs/msg/odometry.hpp" not in scan_republisher_cpp
    assert "find_package(nav_msgs REQUIRED)" in hesai_cmake
    assert "<depend>nav_msgs</depend>" in hesai_package
    assert "preprocessor_params" in slam_launch
    assert "nav_points_topic" in slam_launch
    assert '"output_frame_id": "lidar_level_link"' in slam_launch
    assert 'overlay_root / "config" / "jt128_scan_slam2d.yaml"' in slam_launch
    assert "output_frame_id: lidar_level_link" in preprocessor_cfg
    assert "height_filter.min_z: -1.20" in preprocessor_cfg
    assert "transform_publish_period: 0.0" in slam_cfg
    assert "target_frame: lidar_level_link" in slam_scan_cfg
    assert "min_height: -0.75" in slam_scan_cfg
    assert "max_height: 0.35" in slam_scan_cfg
    assert "minimum_time_interval: 0.05" in slam_cfg
    assert "transform_timeout: 0.50" in slam_cfg
    assert "scan_topic: /scan" in slam_cfg
    assert "scan_queue_size: 1" in slam_cfg
    assert "minimum_travel_distance: 0.10" in slam_cfg
    assert "minimum_travel_heading: 0.035" in slam_cfg
    assert "scan_buffer_size: 100" in slam_cfg
    assert "scan_buffer_maximum_scan_distance: 10.0" in slam_cfg
    assert "use_scan_matching: true" in slam_cfg
    assert "ceres_loss_function: HuberLoss" in slam_cfg
    assert "do_loop_closing: true" in slam_cfg
    assert "loop_search_maximum_distance: 3.0" in slam_cfg
    assert "loop_match_minimum_chain_size: 15" in slam_cfg
    assert "loop_match_maximum_variance_coarse: 2.0" in slam_cfg
    assert "loop_match_minimum_response_coarse: 0.45" in slam_cfg
    assert "loop_match_minimum_response_fine: 0.55" in slam_cfg
    assert "loop_search_space_dimension: 6.0" in slam_cfg
    assert "correlation_search_space_dimension: 0.5" in slam_cfg
    assert "stamped-scan-tf <scan_topic> <tf_topic> <target_frame>" in readiness_probe_cpp
    assert 'command == "stamped-scan-tf"' in readiness_probe_cpp
    assert "stamped_scan_tf_fixture.py" in stamped_scan_tf_smoke
    assert "probe incorrectly accepted scans without dynamic TF" in stamped_scan_tf_smoke
    assert 'start_mapping_scan_observer \\' in run_projected_map
    assert '"${SLAM2D_SCAN_TOPIC}" "${slam_tf_topic}" "${slam_odom_frame}"' in run_projected_map
    assert run_projected_map.index('log_mapping_startup_stage "odom_bridge_ready"') < run_projected_map.index(
        'start_mapping_scan_observer \\')
    assert run_projected_map.index('wait_for_scan_owner_observer', run_projected_map.index('ros2 launch "${SLAM_LAUNCH_FILE}"')) < (
        run_projected_map.index('log_mapping_startup_stage "stamped_scan_tf_ready"'))
    assert "/mapping/scan" not in slam_launch
    assert "'topic': '/map'" in dashboard_patch
    assert "slam_toolbox_map_grid" in dashboard_patch
    assert "timeout=45.0" in dashboard_patch
    assert "scale(1, -1)" in dashboard_patch
    assert "y: lastDrawBounds.offsetY + lastDrawBounds.drawHeight - ((localSampleY + 0.5) / lastDrawBounds.sampleHeight) * lastDrawBounds.drawHeight" in dashboard_patch
    assert "save_map_2d(safe_name, 'slam')" in dashboard_patch
    assert "def _payload_ready() -> bool:" in dashboard_patch
    assert "timeout=12.0" in dashboard_patch
    assert "frame_id', '')).strip() == 'map'" in dashboard_patch
    assert "self.ros_state._refresh_dynamic_subscriptions()" in dashboard_patch
    assert "actions.extend(self._ensure_pgo_started())" in dashboard_patch
    assert "NJRH_LOCALIZER_PREPARE_SCRIPT" in dashboard_patch
    assert "save_map_2d(self, name: str, source: str = 'slam')" in dashboard_patch
    assert ".localizer.png" in dashboard_patch
    assert ".localizer.yaml" in dashboard_patch
    assert 'NEW_START_MAPPING_2D_BLOCK = """' in dashboard_patch
    mapping2d_section = dashboard_patch.split('NEW_START_MAPPING_2D_BLOCK = """', 1)[1]
    mapping2d_section = mapping2d_section.split('"""', 1)[0]
    assert "actions = self._ensure_driver_ready('mapping')" in mapping2d_section
    assert "actions.extend(self._ensure_projected_map_ready())" in mapping2d_section
    assert "self._ensure_driver_fastlio()" not in mapping2d_section
    assert "run_jt128_2d_mapping.sh" not in mapping2d_section
    assert "_cartographer_running()" not in mapping2d_section


def test_slam_toolbox_mapping_owns_canonical_scan_from_fastlio_slice():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    mapping_runner = (overlay / "scripts" / "run_projected_map.sh").read_text(
        encoding="utf-8"
    )
    ownership_helpers = (
        overlay / "scripts" / "scan_ownership_helpers.sh"
    ).read_text(encoding="utf-8")
    transport_helpers = (
        overlay / "scripts" / "mapping_fastdds_transport.sh"
    ).read_text(encoding="utf-8")
    throughput_probe = (
        overlay / "scripts" / "verify_mapping_scan_throughput.py"
    ).read_text(encoding="utf-8")
    launch = (overlay / "launch" / "jt128_slam_toolbox_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    mapping_config = (
        overlay / "config" / "jt128_slam_toolbox_mapping.yaml"
    ).read_text(encoding="utf-8")
    navigation_scan_config = (
        overlay / "config" / "pointcloud_accel_axis.yaml"
    ).read_text(encoding="utf-8")
    accel_core = (
        ROOT
        / "src"
        / "robot_hesai_jt128"
        / "src"
        / "pointcloud_accel_core.cpp"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("scan_topic", default_value="/scan")' in launch
    assert '"scan_topic": scan_topic' in launch
    assert "scan_topic: /scan" in mapping_config
    assert "scan_output_topic: /scan" in navigation_scan_config
    assert "nav_cloud_preprocessor" in launch
    assert "pointcloud_to_laserscan" in launch
    assert 'DeclareLaunchArgument("points_topic", default_value="/mapping/fastlio/cloud_registered_body")' in launch
    assert 'DeclareLaunchArgument("nav_points_topic", default_value="/mapping/points_nav")' in launch
    assert 'DeclareLaunchArgument("scan_topic", default_value="/scan")' in launch
    assert '("scan", scan_topic)' in launch
    assert "scan_republisher" not in launch
    assert "mapping_scan_tf_gate" not in launch
    assert "restamp" not in launch.lower()
    assert 'SLAM2D_SCAN_TOPIC="${NJRH_SLAM2D_SCAN_TOPIC:-/scan}"' in mapping_runner
    assert (
        'release_navigation_scan_owner "${SLAM2D_SCAN_TOPIC}"'
    ) in mapping_runner
    assert (
        'wait_for_scan_publisher_count "${SLAM2D_SCAN_TOPIC}" 0'
    ) not in mapping_runner
    # Start the graph-identity observer before the mapping publisher.  Fast DDS
    # can otherwise expose a unique endpoint with UNKNOWN node metadata to a
    # participant that joins after the launch graph is already active.
    assert 'SLAM2D_SCAN_OWNER_READY_TIMEOUT="${NJRH_SLAM2D_SCAN_OWNER_READY_TIMEOUT:-30}"' in mapping_runner
    assert 'SLAM2D_STAMPED_TF_READY_TIMEOUT="${NJRH_SLAM2D_STAMPED_TF_READY_TIMEOUT:-30}"' in mapping_runner
    assert 'SLAM2D_STAMPED_TF_REQUIRED_GOOD="${NJRH_SLAM2D_STAMPED_TF_REQUIRED_GOOD:-3}"' in mapping_runner
    assert 'pointcloud_to_laserscan "${SLAM2D_SCAN_MAX_AGE_SEC}"' in mapping_runner
    assert '"${SLAM2D_SCAN_OWNER_READY_TIMEOUT}"' in mapping_runner
    assert 'wait_for_scan_owner_observer' in mapping_runner
    observer_start = mapping_runner.index(
        'start_mapping_scan_observer \\\n'
    )
    launch_start = mapping_runner.index('ros2 launch "${SLAM_LAUNCH_FILE}"')
    observer_wait = mapping_runner.index('wait_for_scan_owner_observer', launch_start)
    assert observer_start < launch_start < observer_wait
    assert 'restore_navigation_scan_owner "${SLAM2D_SCAN_TOPIC}"' in mapping_runner
    assert 'runtime_readiness_probe scan-handoff ensure' in ownership_helpers
    assert 'runtime_readiness_probe scan-handoff release' in ownership_helpers
    assert "resident scan ownership request outcome is unknown" in ownership_helpers
    assert "resident_scan_output_matches_requested_state" in ownership_helpers
    assert ownership_helpers.count("ros2 service call") == 1
    assert "std_srvs::srv::SetBool" in accel_core
    assert '"~/set_scan_output_enabled"' in accel_core
    assert "scan_publisher_.reset()" in accel_core
    assert "create_scan_publisher" in accel_core
    assert 'source "${SCRIPT_DIR}/mapping_fastdds_transport.sh"' in mapping_runner
    assert "configure_mapping_fastdds_transport" in mapping_runner
    assert "cleanup_mapping_fastdds_transport" in mapping_runner
    assert "NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE" in mapping_runner
    assert "NJRH_SLAM2D_FASTDDS_BASE_PROFILE_FILE" in transport_helpers
    assert "134217728" in transport_helpers
    assert "<type>UDPv4</type>" in transport_helpers
    assert "<type>SHM</type>" in transport_helpers
    assert "NJRH_SLAM2D_FASTDDS_TRANSPORT:-scoped_shm" in transport_helpers
    assert "NJRH_SLAM2D_FASTDDS_TRANSPORT=inherit" in transport_helpers
    assert "mapping_fastdds_stop_profile_participants" in transport_helpers
    assert "mapping_fastdds_process_uses_profile" in transport_helpers
    assert "pgrep -f 'fastlio_mapping|nav_cloud_preprocessor|pointcloud_to_laserscan'" in transport_helpers
    assert "cloud_transport_env" in launch
    assert "control_transport_env" in launch
    assert 'cloud_transport_env["NJRH_SLAM2D_FASTDDS_ROLE"] = "large_cloud"' in launch
    assert '"NJRH_SLAM2D_FASTDDS_ROLE": "control_udp"' in launch
    assert "nav_cloud_preprocessor" in launch
    assert "pointcloud_to_scan" in launch
    assert "slam_toolbox_node" in launch
    assert "from sensor_msgs.msg import LaserScan" in throughput_probe
    assert "from sensor_msgs.msg import PointCloud2" not in throughput_probe
    assert 'default="/scan"' in throughput_probe
    assert "duplicate_stamp_count" in throughput_probe
    assert "regressed_stamp_count" in throughput_probe


def test_localization_bridge_uses_pose_receive_time_for_freshness():
    repo_bridge = (ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp").read_text(
        encoding="utf-8"
    )
    overlay_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_localization_bridge.sh"
    ).read_text(encoding="utf-8")
    assert "latest_pose_received_sec_" in repo_bridge
    assert "pose_received_sec = latest_pose_received_sec_" in repo_bridge
    assert "now_sec - pose_received_sec > timeout_sec_" in repo_bridge
    assert 'declare_parameter<double>("publish_rate_hz", 10.0)' in repo_bridge
    assert 'declare_parameter<double>("tf_future_stamp_offset_sec", 0.0)' in repo_bridge
    assert "rclcpp::Duration::from_seconds(tf_future_stamp_offset_sec_)" in repo_bridge
    assert "publish_rate_hz: 50.0" in (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    assert "tf_future_stamp_offset_sec: 0.0" in (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    assert "install/robot_localization_bridge/lib/robot_localization_bridge/localization_bridge_node" in overlay_runner
    assert "njrh_exec_affined robot_localization_bridge" in overlay_runner
    assert "Python fallback has been removed" in overlay_runner


def test_localization_bridge_runtime_requires_ros_graph_health():
    canonical_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "canonical_tf_helpers.sh"
    ).read_text(encoding="utf-8")
    occupancy_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh"
    ).read_text(encoding="utf-8")
    nav2_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_nav2_navigation.sh"
    ).read_text(encoding="utf-8")
    navigation_services = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")

    assert "localization_bridge_graph_ready" in canonical_helpers
    assert "localization_bridge_runtime_ready" in canonical_helpers
    assert 'wait_for_node_name "/robot_localization_bridge"' in canonical_helpers
    assert 'wait_for_topic_publisher_from_node "/localization/health" "robot_localization_bridge"' not in canonical_helpers
    assert 'wait_for_topic_publisher_from_node "/tf" "robot_localization_bridge"' in canonical_helpers
    assert 'wait_for_service_name "/robot_localization_bridge/force_accept_next_localization"' in canonical_helpers
    assert 'wait_for_topic_message "/localization/health"' not in canonical_helpers
    assert 'wait_for_tf_edge "map" "odom"' in canonical_helpers
    assert 'robot_localization_bridge*)' in canonical_helpers
    assert 'localization_bridge_graph_ready "${NJRH_LOCALIZATION_BRIDGE_REUSE_READY_TIMEOUT:-2}"' not in canonical_helpers
    assert 'localization_bridge_graph_ready "${NJRH_LOCALIZATION_BRIDGE_ENDPOINT_READY_TIMEOUT:-8}"' not in canonical_helpers

    assert "monitor_localization_bridge_graph" not in occupancy_runner
    assert "graph/runtime watchdog miss" not in occupancy_runner
    assert "NJRH_LOCALIZATION_BRIDGE_WATCHDOG_MAX_MISSES" not in occupancy_runner
    assert "NJRH_LOCALIZATION_BRIDGE_WATCHDOG_PERIOD_SEC" not in occupancy_runner

    assert 'wait_for_tf_transform "map" "odom" "${NJRH_NAV_MAP_ODOM_TF_READY_TIMEOUT:-12}"' not in nav2_runner
    assert "map->odom is not ready; refusing to start Nav2 blind" not in nav2_runner
    assert "blocking readiness probes are disabled" in nav2_runner

    assert "ensure_localization_layer_alive" in navigation_services
    assert "resident localization layer exited before Nav2 activation" in navigation_services


def test_dashboard_runtime_patches_nav2_params_to_repo_owned_config():
    patch_v2 = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime_v2.py"
    ).read_text(encoding="utf-8")
    assert "nav2 params path override" in patch_v2
    assert "return self.root_dir / 'config' / 'nav2.yaml'" in patch_v2
    assert "local_costmap subscription qos" in patch_v2
    assert "'topic': '/local_costmap/costmap'" in patch_v2
    assert "'qos': self._map_qos" in patch_v2
    assert "standard navigation kill pattern" in patch_v2
    assert "ros2 launch .*standard_navigation.launch.py" in patch_v2
    assert "nav2 navigation lifecycle helpers" in patch_v2
    assert "/lifecycle_manager_navigation/manage_nodes" in patch_v2
    assert "active [3]" in patch_v2
    assert "Nav2 lifecycle startup requested" in patch_v2
    assert "nav2 navigation lifecycle activation" in patch_v2
    assert "description='Nav2 navigation lifecycle activation'" in patch_v2
    assert "/api/local_costmap_debug/start" in patch_v2
    assert "/api/local_costmap_debug/stop" in patch_v2
    assert "startLocalCostmapDebugBtn" in patch_v2
    assert "stopLocalCostmapDebugBtn" in patch_v2
    assert "run_local_costmap_debug.sh" in patch_v2
    assert "local_costmap_debug.launch.py" in patch_v2
    assert "openMap2dPopup('', true, false, false, 'local_costmap')" in patch_v2
    assert "/api/floors/list" in patch_v2
    assert "/api/floors/promote" in patch_v2
    assert "/api/floors/select" in patch_v2
    assert "/api/floors/switch" in patch_v2
    assert "expected_asset_epoch" in patch_v2
    assert "expected_asset_digest" in patch_v2
    assert "safe_map_id" in patch_v2
    assert "resume_navigation: false" in patch_v2
    assert "resume_navigation: true" not in patch_v2
    assert "promote_map_to_floor.sh" in patch_v2
    assert "select_floor_assets.sh" in patch_v2
    assert "/floor_manager/switch_floor" in patch_v2
    assert "/workspaces/njrh-v3/workspace1" in patch_v2
    assert "listFloorAssetsTestBtn" in patch_v2
    assert "promoteFloorAssetTestBtn" in patch_v2
    assert "selectFloorAssetTestBtn" in patch_v2
    assert "switchFloorAssetTestBtn" in patch_v2
    assert "测试：归档地图到楼层" in patch_v2


def test_runtime_overlay_standard_navigation_uses_repo_owned_launch():
    overlay_nav = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_nav2_navigation.sh").read_text(encoding="utf-8")
    web_runner = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_web_dashboard.sh").read_text(encoding="utf-8")
    common_env = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "common_env.sh").read_text(encoding="utf-8")
    standard_navigation_launch = (ROOT / "src" / "robot_bringup" / "launch" / "standard_navigation.launch.py").read_text(encoding="utf-8")
    assert "standard_navigation.launch.py" in overlay_nav
    assert "require_upstream_script run_nav2_navigation.sh" not in overlay_nav
    assert 'params_file:="${NAV2_PARAMS_FILE}"' in overlay_nav
    assert 'PROJECT_ROOT="${NJRH_PROJECT_ROOT:-/workspaces/njrh-v3/workspace1}"' in common_env
    assert 'export NJRH_MAPS_DIR="${NJRH_MAPS_DIR:-${OVERLAY_ROOT}/maps}"' in common_env
    assert 'export NJRH_MAPS3D_DIR="${NJRH_MAPS3D_DIR:-${OVERLAY_ROOT}/maps3d}"' in common_env
    assert 'export NJRH_RELEASE_ASSETS_DIR="${NJRH_RELEASE_ASSETS_DIR:-${PROJECT_ROOT}/maps_release}"' in common_env
    assert 'export NJRH_WAYPOINTS_DIR="${NJRH_WAYPOINTS_DIR:-${OVERLAY_ROOT}/waypoints}"' in common_env
    assert 'NJRH_COMMON_ENV_SETUP_DONE' in common_env
    assert 'source "${ROOT_DIR}/scripts/common_env.sh"' in web_runner
    assert 'LAUNCH_FILE="${NJRH_PROJECT_ROOT}/src/robot_bringup/launch/standard_navigation.launch.py"' in overlay_nav
    assert "stop_existing_standard_nav_stack" in overlay_nav
    assert "use_respawn:=false" not in overlay_nav
    assert "use_composition:=false" not in overlay_nav
    assert 'DeclareLaunchArgument("use_respawn", default_value="False")' in standard_navigation_launch
    assert 'DeclareLaunchArgument("use_composition", default_value="False")' in standard_navigation_launch


def test_dashboard_runtime_uses_canonical_lidar_transform_checks():
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    assert "base_link -> lidar_level_link TF is unavailable" in dashboard_patch
    assert "has_transform('base_link', 'lidar_level_link')" in dashboard_patch
    assert "description='base_link -> lidar_level_link transform'" in dashboard_patch
    assert "mount_frame = 'lidar_mount_link'" in dashboard_patch


def test_dashboard_navigation_waits_for_map_pose_before_standard_nav():
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    assert 'NEW_START_NAVIGATION_BLOCK = """' in dashboard_patch
    nav_section = dashboard_patch.split('NEW_START_NAVIGATION_BLOCK = """', 1)[1]
    nav_section = nav_section.split('"""', 1)[0]
    assert "actions.extend(self._ensure_fastlio_ready())" not in nav_section
    assert "actions = self._ensure_driver_ready('mapping')" in nav_section
    assert "actions.extend(self._ensure_lidar_tf_ready())" not in nav_section
    assert "self._nav2_map_name = map_yaml.stem" in nav_section
    assert "with self._temporary_view_features(['pose_tracking'], source='internal:start_navigation_pose_wait'):" in nav_section
    assert "actions.append(self._schedule_grid_search_localization(" in nav_section
    assert "NEW_START_NAVIGATION_BLOCK = NEW_START_NAVIGATION_BLOCK.replace" in dashboard_patch
    assert "timeout=8.0" in dashboard_patch
    assert "localization_result not observed directly; using map->odom gate" in dashboard_patch
    assert "latest_localization_result_pose(max_age=0.0) is not None" in nav_section
    assert "description='Isaac localization_result'" in nav_section
    assert "actions.append('localization_result ready')" in nav_section
    assert "lambda: self.ros_state.current_map_pose() is not None" in nav_section
    assert "description='map -> odom stability'" in nav_section
    assert "actions.append('map -> odom ready')" in nav_section
    assert "actions.extend(self._start_navigation_nav_stack(nav_profile=nav_profile))" in nav_section
    servers_section = dashboard_patch.split('NEW_START_NAVIGATION_SERVERS_BLOCK = """', 1)[1]
    servers_section = servers_section.split('"""', 1)[0]
    assert "actions.extend(self._start_navigation_localization_stack(map_yaml))" in servers_section
    assert "actions.extend(self._ensure_lidar_tf_ready())" in servers_section
    assert "latest_localization_result_pose(max_age=0.0) is not None" not in servers_section
    assert "actions.extend(self._start_navigation_nav_stack(nav_profile=nav_profile))" not in servers_section


def test_restart_navigation_localization_does_not_restart_fastlio():
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    assert 'NEW_RESTART_NAVIGATION_LOCALIZATION_BLOCK = """' in dashboard_patch
    assert "source='restart_localization'" in dashboard_patch
    restart_section = dashboard_patch.split('NEW_RESTART_NAVIGATION_LOCALIZATION_BLOCK = """', 1)[1]
    restart_section = restart_section.split('"""', 1)[0]
    assert "actions.extend(self._ensure_fastlio_ready())" not in restart_section
    assert "actions.extend(self._ensure_driver_ready('mapping'))" in restart_section
    assert "actions.extend(self._start_navigation_localization_stack(map_yaml))" in restart_section
    assert "actions.extend(self._ensure_lidar_tf_ready())" in restart_section


def test_navigation_localization_uses_canonical_lidar_points_with_fastlio_odom_only():
    occupancy_stack = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization_stack.launch.py").read_text(encoding="utf-8")
    localization_sensing = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_localization_sensing.launch.py").read_text(encoding="utf-8")
    localization_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    assert 'POINTS_TOPIC="${NJRH_LOCALIZATION_POINTS_TOPIC:-/lidar_points_nav}"' in localization_script
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points_nav")' in occupancy_stack
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points_nav")' in localization_sensing
    assert "starting FAST-LIO2 deskew source for occupancy localization" not in localization_script
    assert "FASTLIO_CONFIG_FILE" not in localization_script
    assert "start_or_reuse_fastlio_for_local_state" not in localization_script
    assert "fastlio_runtime_topics_fresh()" not in localization_script
    assert "resident FAST-LIO runtime is stale" not in localization_script
    assert "FAST-LIO odometry did not become fresh for navigation odometry" not in localization_script
    assert "ros2 run fast_lio fastlio_mapping" not in localization_script
    assert "-r /tf:=/tf_fastlio_internal" not in localization_script
    assert "ensure_resident_fastlio_for_local_state()" in localization_script
    assert "ensure_resident_local_state_for_localization()" in localization_script
    assert "NJRH_LOCALIZATION_LOCAL_STATE_WAIT_SEC:-20" in localization_script
    assert 'wait_for_local_state_required_processes "${timeout_sec}"' in localization_script
    assert "managed FAST-LIO process exists for explicit fastlio local-state mode; startup topic probes are disabled" in localization_script
    assert "resident local_state ${NAV_LOCAL_STATE_MODE} process exists for occupancy localization; startup odom/TF probes are disabled" in localization_script
    assert "timed out waiting for localization pointcloud" not in localization_script
    assert "localization pointcloud startup probe disabled" in localization_script


def test_fastlio_uses_canonical_lidar_topics_by_default():
    fastlio_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "fastlio.yaml").read_text(encoding="utf-8")
    fastlio_qos_patch = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "patches" / "fast_lio_reliable_lidar_qos.patch"
    ).read_text(encoding="utf-8")
    fastlio_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_fastlio_tf.sh").read_text(encoding="utf-8")
    localization_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    wrapper_cfg = (ROOT / "src" / "robot_fastlio_mapping" / "config" / "fastlio.yaml").read_text(encoding="utf-8")
    fastlio_cmake = (ROOT / "src" / "robot_fastlio_mapping" / "CMakeLists.txt").read_text(encoding="utf-8")
    fastlio_package = (ROOT / "src" / "robot_fastlio_mapping" / "package.xml").read_text(encoding="utf-8")
    fastlio_odom_bridge = (
        ROOT / "src" / "robot_fastlio_mapping" / "src" / "fastlio_odom_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    assert "lid_topic: /lidar_points" in fastlio_cfg
    assert "lid_topic: /lidar_points_fastlio" not in fastlio_cfg
    assert "imu_topic: /lidar_imu" in fastlio_cfg
    assert "sensor_frame_id: lidar_link" in fastlio_cfg
    assert "point_filter_num: 1" in fastlio_cfg
    assert "max_iteration: 4" in fastlio_cfg
    assert "lidar_input_reliable: false" in fastlio_cfg
    assert "lidar_input_qos_depth: 1" in fastlio_cfg
    assert "path_en: false" in fastlio_cfg
    assert "map_en: false" in fastlio_cfg
    assert "lidar_input_reliable" in fastlio_qos_patch
    assert "lidar_input_qos_depth" in fastlio_qos_patch
    assert "create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, lidar_input_qos" in fastlio_qos_patch
    assert "Legacy Fast-LIO-only remap paths are no longer supported" in fastlio_script
    assert 'for pattern in "fastlio_mapping" "laser_mapping" "ros2 launch fast_lio" "hesai_lidar_state_publisher"; do' in fastlio_script
    assert 'source "${SCRIPT_DIR}/cpu_affinity.sh"' in fastlio_script
    assert "njrh_run_affined fastlio_mapping ros2 run fast_lio fastlio_mapping" in fastlio_script
    assert 'env LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}" bash "${SCRIPT_DIR}/run_local_state.sh"' in fastlio_script
    assert "pointcloud_axis_remap --ros-args" not in fastlio_script
    assert "imu_axis_remap --ros-args" not in fastlio_script
    assert '"imu_axis_remap"' not in fastlio_script
    assert '"pointcloud_axis_remap"' not in fastlio_script
    assert '"imu_axis_remap"' not in localization_script
    assert '"pointcloud_axis_remap"' not in localization_script
    assert "ros2 run fast_lio fastlio_mapping" not in localization_script
    assert "resident_fastlio_runtime_running()" in localization_script
    assert "ensure_resident_fastlio_for_local_state()" in localization_script
    assert "upstream_points_topic: /lidar_points" in wrapper_cfg
    assert "upstream_imu_topic: /lidar_imu" in wrapper_cfg
    assert "upstream_sensor_frame: lidar_link" in wrapper_cfg
    assert "upstream_send_odom_base_tf: false" in wrapper_cfg
    assert "fallback_points_topic" not in wrapper_cfg
    assert "fallback_imu_topic" not in wrapper_cfg
    assert "add_executable(fastlio_odom_bridge_node src/fastlio_odom_bridge_node.cpp)" in fastlio_cmake
    assert "ament_target_dependencies(fastlio_odom_bridge_node" in fastlio_cmake
    assert "<depend>geometry_msgs</depend>" in fastlio_package
    assert "<depend>nav_msgs</depend>" in fastlio_package
    assert "<depend>rclcpp</depend>" in fastlio_package
    assert "<depend>tf2_msgs</depend>" in fastlio_package
    assert "<depend>tf2_ros</depend>" in fastlio_package
    assert 'declare_parameter<std::string>("input_topic", "/Odometry")' in fastlio_odom_bridge
    assert 'declare_parameter<std::string>("output_topic", "/fastlio/base_odometry")' in fastlio_odom_bridge
    assert 'declare_parameter<bool>("publish_tf", false)' in fastlio_odom_bridge
    assert 'declare_parameter<bool>("restamp_output_to_now", false)' in fastlio_odom_bridge
    assert 'declare_parameter<double>("output_stamp_offset_sec", 0.0)' in fastlio_odom_bridge
    assert 'declare_parameter<bool>("input_reliable", false)' in fastlio_odom_bridge
    assert 'declare_parameter<std::int64_t>("input_qos_depth", 1)' in fastlio_odom_bridge
    assert "builtin_interfaces::msg::Time output_stamp" in fastlio_odom_bridge
    assert "get_clock()->now()" in fastlio_odom_bridge
    assert "lookupTransform(" in fastlio_odom_bridge
    assert "base_from_child" in fastlio_odom_bridge
    assert "odom.header.frame_id = output_odom_frame_" in fastlio_odom_bridge
    assert "odom.child_frame_id = output_base_frame_" in fastlio_odom_bridge


def test_jt128_driver_normalizes_vendor_raw_to_canonical_topics():
    driver_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_driver.sh").read_text(encoding="utf-8")
    jt128_cfg = (ROOT / "src" / "robot_hesai_jt128" / "config" / "jt128.yaml").read_text(encoding="utf-8")
    pointcloud_remap_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_canonical_pointcloud_remap.yaml"
    ).read_text(encoding="utf-8")
    pointcloud_local_branch_cfg_path = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "jt128_canonical_pointcloud_remap_local_branch.yaml"
    )
    local_profile_env = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_perception_input_profile.env"
    ).read_text(encoding="utf-8")
    local_profile_helper = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "local_perception_profile.sh"
    ).read_text(encoding="utf-8")
    pointcloud_remap_cpp = (
        ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_axis_remap_node.cpp"
    ).read_text(encoding="utf-8")
    pointcloud_downsample_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_pointcloud_downsample.yaml"
    ).read_text(encoding="utf-8")
    pointcloud_downsample_cpp = (
        ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_downsample_node.cpp"
    ).read_text(encoding="utf-8")
    imu_remap_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_canonical_imu_remap.yaml"
    ).read_text(encoding="utf-8")
    hesai_cmake = (ROOT / "src" / "robot_hesai_jt128" / "CMakeLists.txt").read_text(encoding="utf-8")
    local_perception_cmake = (ROOT / "src" / "robot_local_perception" / "CMakeLists.txt").read_text(encoding="utf-8")
    pointcloud_pipeline_launch = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "pointcloud_perception_pipeline.launch.py"
    ).read_text(encoding="utf-8")
    run_local_perception = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_perception.sh"
    ).read_text(encoding="utf-8")
    verify_pointcloud_rates = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_pointcloud_rates.sh"
    ).read_text(encoding="utf-8")
    verify_pointcloud_matrix = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_pointcloud_delivery_matrix.sh"
    ).read_text(encoding="utf-8")
    verify_lidar_trunk_jitter = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_lidar_trunk_jitter.sh"
    ).read_text(encoding="utf-8")
    diagnose_lidar_jitter = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "diagnose_lidar_points_jitter.sh"
    ).read_text(encoding="utf-8")
    pure_trunk_ab = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_lidar_trunk_pure_ab.sh"
    ).read_text(encoding="utf-8")
    process_freshness = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "check_runtime_process_freshness.sh"
    ).read_text(encoding="utf-8")
    topology_contract = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "check_pointcloud_topology_contract.sh"
    ).read_text(encoding="utf-8")
    inspect_pointcloud_subscribers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "inspect_pointcloud_subscribers.sh"
    ).read_text(encoding="utf-8")
    dds_transport_ab = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_pointcloud_dds_transport_ab.sh"
    ).read_text(encoding="utf-8")
    set_local_profile = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "set_local_perception_input_profile.sh"
    ).read_text(encoding="utf-8")
    inspect_cpu_affinity = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "inspect_pointcloud_cpu_affinity.sh"
    ).read_text(encoding="utf-8")
    nav_acceptance = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "record_pointcloud_nav_acceptance.sh"
    ).read_text(encoding="utf-8")
    assert 'export LIDAR_FRAME="${LIDAR_FRAME:-lidar_link}"' in driver_script
    assert 'export IMU_FRAME="${IMU_FRAME:-imu_link}"' in driver_script
    assert 'export POINTS_TOPIC="${NJRH_JT128_POINTS_TOPIC:-/lidar_points}"' in driver_script
    assert 'export IMU_TOPIC="${NJRH_JT128_IMU_TOPIC:-/lidar_imu}"' in driver_script
    assert 'export VENDOR_POINTS_TOPIC="${NJRH_JT128_VENDOR_POINTS_TOPIC:-/jt128/vendor/points_raw}"' in driver_script
    assert 'export VENDOR_IMU_TOPIC="${NJRH_JT128_VENDOR_IMU_TOPIC:-/jt128/vendor/imu_raw}"' in driver_script
    assert 'export POINTCLOUD_PIPELINE_LAUNCH="${NJRH_POINTCLOUD_PIPELINE_LAUNCH:-${NJRH_OVERLAY_ROOT}/launch/pointcloud_perception_pipeline.launch.py}"' in driver_script
    assert 'export NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER="${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER:-false}"' in driver_script
    assert 'export POINTCLOUD_REMAP_CPP_BIN="${NJRH_POINTCLOUD_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/pointcloud_axis_remap_node}"' in driver_script
    assert 'export NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE="${NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE:-false}"' in driver_script
    assert "POINTCLOUD_FASTLIO_REMAP_CONFIG" not in driver_script
    assert 'export IMU_REMAP_CPP_BIN="${NJRH_IMU_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/imu_axis_remap_node}"' in driver_script
    assert "ros_send_point_cloud_topic" in driver_script
    assert "ros_send_imu_topic" in driver_script
    assert "/jt128/vendor/points_raw" in driver_script
    assert "/jt128/vendor/imu_raw" in driver_script
    assert '[[ -x "${POINTCLOUD_REMAP_CPP_BIN}" ]]' in driver_script
    assert 'source "${SCRIPT_DIR}/local_perception_profile.sh"' in driver_script
    assert "njrh_load_local_perception_input_profile" in driver_script
    assert "njrh_print_local_perception_profile" in driver_script
    assert 'pointcloud_remap_args=(--ros-args --params-file "${POINTCLOUD_REMAP_CONFIG}")' in driver_script
    assert '-p "local_output_topic:=${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC}"' in driver_script
    assert '-p "local_output_stride:=${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE}"' in driver_script
    assert '-p "local_output_publish_every_n:=${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N}"' in driver_script
    assert '"${POINTCLOUD_REMAP_CPP_BIN}" "${pointcloud_remap_args[@]}" &' in driver_script
    assert "does not apply local branch CLI overrides" in driver_script
    assert "ros2 launch \"${POINTCLOUD_PIPELINE_LAUNCH}\"" in driver_script
    assert "pointcloud ingress publishes only canonical /lidar_points" in driver_script
    assert "[p]ointcloud_fastlio_remap" in driver_script
    assert '--params-file "${POINTCLOUD_FASTLIO_REMAP_CONFIG}" -r __node:=pointcloud_fastlio_remap &' not in driver_script
    assert '[[ -x "${IMU_REMAP_CPP_BIN}" ]]' in driver_script
    assert '"${IMU_REMAP_CPP_BIN}" --ros-args --params-file "${IMU_REMAP_CONFIG}" &' in driver_script
    assert "canonical_jt128_ingress_running()" in driver_script
    assert "incomplete JT128 ingress detected" in driver_script
    assert "stop_jt128_ingress_processes" in driver_script
    assert 'pointcloud_axis_remap.py' not in driver_script
    assert 'imu_axis_remap.py' not in driver_script
    assert "Python remap fallback has been removed" in driver_script
    assert "points_topic: /lidar_points" in jt128_cfg
    assert "nav_points_topic: /lidar_points_nav" in jt128_cfg
    assert 'local_perception_points_topic: ""' in jt128_cfg
    assert "imu_topic: /lidar_imu" in jt128_cfg
    assert "vendor_points_topic: /jt128/vendor/points_raw" in jt128_cfg
    assert "vendor_imu_topic: /jt128/vendor/imu_raw" in jt128_cfg
    assert 'export NJRH_HESAI_UPSTREAM_DRIVER_PROFILE="${NJRH_HESAI_UPSTREAM_DRIVER_PROFILE:-navigation}"' in driver_script
    assert 'UPSTREAM_DRIVER_PROFILE="${NJRH_HESAI_UPSTREAM_DRIVER_PROFILE}"' in driver_script
    assert 'use_timestamp_type: 1' in driver_script
    assert 'export DRIVER_PROFILE="${UPSTREAM_DRIVER_PROFILE}"' in driver_script
    assert 'njrh_run_large_cloud_affined hesai_ros_driver bash "$(require_upstream_script run_driver.sh)" &' in driver_script
    assert 'njrh_run_affined hesai_ros_driver bash "$(require_upstream_script run_driver.sh)" &' not in driver_script
    raw_to_canonical = (
        "rotation_matrix:\n"
        "      - 0.0\n"
        "      - 1.0\n"
        "      - 0.0\n"
        "      - -1.0\n"
        "      - 0.0\n"
        "      - 0.0\n"
        "      - 0.0\n"
        "      - 0.0\n"
        "      - 1.0"
    )
    identity_matrix = (
        "rotation_matrix:\n"
        "      - 1.0\n"
        "      - 0.0\n"
        "      - 0.0\n"
        "      - 0.0\n"
        "      - 1.0\n"
        "      - 0.0\n"
        "      - 0.0\n"
        "      - 0.0\n"
        "      - 1.0"
    )
    assert raw_to_canonical in pointcloud_remap_cfg
    assert raw_to_canonical in imu_remap_cfg
    assert "fast_path_neg_raw_y_neg_raw_x" in pointcloud_remap_cpp
    assert "void on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)" in pointcloud_remap_cpp
    assert "auto output = std::move(msg);" in pointcloud_remap_cpp
    assert "pointcloud_axis_remap_component" in hesai_cmake
    assert "find_package(std_msgs REQUIRED)" in hesai_cmake
    assert "std_msgs" in hesai_cmake
    assert 'rclcpp_components_register_nodes(pointcloud_axis_remap_component "PointCloudAxisRemapNode")' in hesai_cmake
    assert "local_perception_component" in local_perception_cmake
    assert 'rclcpp_components_register_nodes(local_perception_component "LocalPerceptionNode")' in local_perception_cmake
    assert "ComposableNodeContainer" in pointcloud_pipeline_launch
    assert 'plugin="PointCloudAxisRemapNode"' in pointcloud_pipeline_launch
    assert 'plugin="LocalPerceptionNode"' not in pointcloud_pipeline_launch
    assert "robot_local_perception" not in pointcloud_pipeline_launch
    assert '"use_intra_process_comms": True' in pointcloud_pipeline_launch
    assert "nav_output_topic: /lidar_points_nav" in pointcloud_remap_cfg
    assert "nav_output_stride: 4" in pointcloud_remap_cfg
    assert "nav_output_publish_every_n: 2" in pointcloud_remap_cfg
    assert 'local_output_topic: ""' in pointcloud_remap_cfg
    assert "local_output_stride: 1" in pointcloud_remap_cfg
    assert "local_output_publish_every_n: 1" in pointcloud_remap_cfg
    assert "output_qos_depth: 1" in pointcloud_remap_cfg
    assert "output_reliable: false" in pointcloud_remap_cfg
    assert "status_topic: /lidar/axis_remap_status" in pointcloud_remap_cfg
    assert "status_publish_period_sec: 1.0" in pointcloud_remap_cfg
    assert not pointcloud_local_branch_cfg_path.exists()
    assert "export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=disabled" in local_profile_env
    assert "Production local obstacle handling has moved to Nav2's standard" in local_profile_env
    assert "/scan" in local_profile_env
    assert "path. Keep the old PointCloud2 local-perception branch disabled" in local_profile_env
    assert "local_branch|trunk)" in local_profile_helper
    assert "forcing disabled because Nav2 uses /scan" in local_profile_helper
    assert "disabled)" in local_profile_helper
    assert 'RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC="${profile_input_topic}"' in local_profile_helper
    assert 'RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC="${profile_axis_local_output_topic}"' in local_profile_helper
    assert 'RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC="${ROBOT_LOCAL_PERCEPTION_INPUT_TOPIC}"' not in local_profile_helper
    assert 'RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC}"' not in local_profile_helper
    assert 'RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE}"' in local_profile_helper
    assert 'RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N}"' in local_profile_helper
    assert "selected local perception profile=" in local_profile_helper
    assert "robot_local_perception PointCloud2 obstacle pipeline has been removed" in run_local_perception
    assert '-p "input_topic:=${INPUT_TOPIC}"' not in run_local_perception
    assert 'declare_parameter<std::string>("status_topic", "/lidar/axis_remap_status")' in pointcloud_remap_cpp
    assert 'declare_parameter<int>("nav_output_publish_every_n", 1)' in pointcloud_remap_cpp
    assert 'declare_parameter<int>("local_output_publish_every_n", 1)' in pointcloud_remap_cpp
    assert "raw_input_hz" in pointcloud_remap_cpp
    assert "lidar_points_publish_hz" in pointcloud_remap_cpp
    assert "last_trunk_publish_duration_ms" in pointcloud_remap_cpp
    assert "last_branch_publish_duration_ms" in pointcloud_remap_cpp
    assert "last_total_publish_outputs_duration_ms" in pointcloud_remap_cpp
    assert "raw_interarrival_ms_avg" in pointcloud_remap_cpp
    assert "raw_interarrival_ms_max" in pointcloud_remap_cpp
    assert "lidar_points_publish_interval_ms_avg" in pointcloud_remap_cpp
    assert "lidar_points_publish_interval_ms_max" in pointcloud_remap_cpp
    assert "trunk_publish_gap_over_100ms_count" in pointcloud_remap_cpp
    assert "trunk_publish_gap_over_150ms_count" in pointcloud_remap_cpp
    assert "trunk_publish_gap_over_200ms_count" in pointcloud_remap_cpp
    assert "last_raw_callback_duration_ms" in pointcloud_remap_cpp
    assert "last_publish_outputs_start_to_end_ms" in pointcloud_remap_cpp
    assert "nav_branch_attempt_hz" in pointcloud_remap_cpp
    assert "local_branch_attempt_hz" in pointcloud_remap_cpp
    assert "nav_branch_publish_hz" in pointcloud_remap_cpp
    assert "nav_branch_skip_hz" in pointcloud_remap_cpp
    assert "nav_branch_last_publish_age_ms" in pointcloud_remap_cpp
    assert "nav_branch_last_publish_duration_ms" in pointcloud_remap_cpp
    assert "nav_branch_last_points" in pointcloud_remap_cpp
    assert "nav_branch_last_bytes" in pointcloud_remap_cpp
    assert "nav_branch_subscription_count" in pointcloud_remap_cpp
    assert "nav_output_publish_every_n" in pointcloud_remap_cpp
    assert "local_branch_publish_hz" in pointcloud_remap_cpp
    assert "local_branch_skip_hz" in pointcloud_remap_cpp
    assert "local_branch_last_publish_age_ms" in pointcloud_remap_cpp
    assert "local_branch_last_publish_duration_ms" in pointcloud_remap_cpp
    assert "local_branch_last_points" in pointcloud_remap_cpp
    assert "local_branch_last_bytes" in pointcloud_remap_cpp
    assert "local_branch_subscription_count" in pointcloud_remap_cpp
    assert "local_output_publish_every_n" in pointcloud_remap_cpp
    assert "output_subscription_count" in pointcloud_remap_cpp
    assert "dropped_or_skipped_count" in pointcloud_remap_cpp
    assert "/jt128/vendor/points_raw" in verify_pointcloud_rates
    assert "/lidar_points" in verify_pointcloud_rates
    assert "/lidar_points_nav" in verify_pointcloud_rates
    assert "MIN_LIDAR_HZ" in verify_pointcloud_rates
    assert "for topic in ${OPTIONAL_TOPICS}" in verify_pointcloud_rates
    assert "/lidar/axis_remap_status" in verify_pointcloud_matrix
    assert "/perception/local_perception_status" not in verify_pointcloud_matrix
    assert "/lidar/nav_cloud_preprocessor_status" in verify_pointcloud_matrix
    assert "CASE_A_MAIN_TRUNK_LOW" in verify_pointcloud_matrix
    assert "CASE_B_LOCAL_DDS_DELIVERY_LOW" not in verify_pointcloud_matrix
    assert "CASE_C_LOCAL_PROCESS_OR_PUBLISH_GATING" not in verify_pointcloud_matrix
    assert "CASE_D_FANOUT_PRESSURE" in verify_pointcloud_matrix
    assert "CASE_E_TOO_MANY_FULL_DENSITY_SUBSCRIBERS" in verify_pointcloud_matrix
    assert "CASE_F_LOCAL_BRANCH_EFFECTIVE" not in verify_pointcloud_matrix
    assert "CASE_G_DDS_TRANSPORT_SUSPECT" in verify_pointcloud_matrix
    assert "CASE_H_LOCAL_BRANCH_ENABLED_BUT_WEAK" not in verify_pointcloud_matrix
    assert "CASE_I_LOCAL_BRANCH_DRAGS_TRUNK" not in verify_pointcloud_matrix
    assert "NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ" in verify_pointcloud_matrix
    assert "topic_graph_counts" in verify_pointcloud_matrix
    assert "graph_subscription_count" in verify_pointcloud_matrix
    assert "FASTDDS_BUILTIN_TRANSPORTS" in verify_pointcloud_matrix
    assert "NJRH_FASTDDS_PROFILE_ENABLED" in verify_pointcloud_matrix
    assert "retired local PointCloud2 obstacle branch is not sampled" in verify_pointcloud_matrix
    assert "local_branch_enabled" not in verify_pointcloud_matrix
    assert "local_branch_publish_hz" not in verify_pointcloud_matrix
    assert "local_branch_subscription_count" not in verify_pointcloud_matrix
    assert "obstacle CLI hint: ros2 topic hz /perception/obstacle_points" not in verify_pointcloud_matrix
    assert "FAST-LIO2 navigation residue" in verify_pointcloud_matrix
    assert "does not start/stop Nav2" in verify_pointcloud_matrix
    assert "/lidar/axis_remap_status" in verify_lidar_trunk_jitter
    assert "/jt128/vendor/points_raw" in verify_lidar_trunk_jitter
    assert "ros2 topic hz \"${RAW_TOPIC}\"" in verify_lidar_trunk_jitter
    assert "ros2 topic hz /lidar_points" in verify_lidar_trunk_jitter
    assert "full-density subscriber" in verify_lidar_trunk_jitter
    assert "last_trunk_publish_duration_ms" in verify_lidar_trunk_jitter
    assert "last_branch_publish_duration_ms" in verify_lidar_trunk_jitter
    assert "last_total_publish_outputs_duration_ms" in verify_lidar_trunk_jitter
    assert "nav_output_publish_every_n" in verify_lidar_trunk_jitter
    assert "scan_publish_hz" in verify_lidar_trunk_jitter
    assert "published_obstacle_hz" not in verify_lidar_trunk_jitter
    assert "This default run does not subscribe to full-density /lidar_points" in diagnose_lidar_jitter
    assert "--include-cli-hz" in diagnose_lidar_jitter
    assert "CASE_A_AXIS_PUBLISH_LOW" in diagnose_lidar_jitter
    assert "CASE_B_CLI_DELIVERY_LOW_ONLY" in diagnose_lidar_jitter
    assert "CASE_C_RETIRED_LOCAL_POINTCLOUD_BRANCH_ENABLED" in diagnose_lidar_jitter
    assert "CASE_D_STALE_PROCESS_OR_OLD_BINARY" in diagnose_lidar_jitter
    assert "CASE_E_TOO_MANY_TRUNK_SUBSCRIBERS" in diagnose_lidar_jitter
    assert "CASE_F_LOCAL_BRANCH_OK_BUT_TRUNK_CLI_LOW" not in diagnose_lidar_jitter
    assert "CASE_G_AXIS_LOW_WITH_PURE_TRUNK_NEEDED" not in diagnose_lidar_jitter
    assert "ros2 topic hz /lidar_points" in diagnose_lidar_jitter
    assert "INCLUDE_CLI_HZ" in diagnose_lidar_jitter
    assert "trunk_publish_gap_over_100ms_count" in diagnose_lidar_jitter
    assert "raw_interarrival_ms_avg" in diagnose_lidar_jitter
    assert "Retired robot_local_perception PointCloud2 obstacle topics are not sampled" in diagnose_lidar_jitter
    assert "does not subscribe to full-density" in diagnose_lidar_jitter
    assert "run_lidar_trunk_pure_ab.sh [--duration-sec SECONDS]" in pure_trunk_ab
    assert "--execute" in pure_trunk_ab
    assert "RESTORE_REQUESTED=true" in pure_trunk_ab
    assert "restore_production_driver" in pure_trunk_ab
    assert 'NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC=""' in pure_trunk_ab
    assert 's|^([[:space:]]*nav_output_topic:[[:space:]]*).*|\\1""|' in pure_trunk_ab
    assert 's|^([[:space:]]*local_output_topic:[[:space:]]*).*|\\1""|' in pure_trunk_ab
    assert "It does not change local_perception_input_profile.env" in pure_trunk_ab
    assert "This A/B is diagnostic only" in pure_trunk_ab
    assert "check_runtime_process_freshness.sh" in str(
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "check_runtime_process_freshness.sh"
    )
    assert "process start >= binary mtime" in process_freshness
    assert "binary mtime >= source mtime" in process_freshness
    assert "/lidar/axis_remap_status" in process_freshness
    assert "raw_interarrival_ms_avg" in process_freshness
    assert "lidar_points_publish_interval_ms_avg" in process_freshness
    assert "nav_branch_attempt_hz" in process_freshness
    assert "jt128_canonical_pointcloud_remap.yaml" in topology_contract
    assert "require_value input_topic /jt128/vendor/points_raw" in topology_contract
    assert "require_value output_topic /lidar_points" in topology_contract
    assert "require_value local_output_topic '\"\"'" in topology_contract
    assert "require_int_ge nav_output_stride 2" in topology_contract
    assert "require_int_ge nav_output_publish_every_n 2" in topology_contract
    assert "baseline config keeps retired local PointCloud2 branch disabled" in topology_contract
    assert "local_branch profile derives /_internal/lidar_points_local" not in topology_contract
    assert "trunk profile disables local internal branch" not in topology_contract
    assert "/lidar_points graph publisher count is 1" in topology_contract
    assert "/scan graph has Nav2/collision subscribers" in topology_contract
    assert "/_internal/lidar_points_local graph publisher/subscriber count is 1/1" not in topology_contract
    assert "jt128_canonical_pointcloud_remap(_local_branch)?[.]yaml" not in topology_contract
    assert "output_reliable" in topology_contract
    assert "Observing ROS graph only" in inspect_pointcloud_subscribers
    assert "ros2 topic info -v" in inspect_pointcloud_subscribers
    assert "ros2 topic echo" not in inspect_pointcloud_subscribers
    assert "ros2 topic hz" not in inspect_pointcloud_subscribers
    assert "/lidar_points publisher count must be exactly 1" in inspect_pointcloud_subscribers
    assert "/lidar_points has ${subscription_count} subscribers" in inspect_pointcloud_subscribers
    assert "/_internal/lidar_points_local" not in inspect_pointcloud_subscribers
    assert "FAST-LIO appears attached to /lidar_points" in inspect_pointcloud_subscribers
    assert "--transport UDPv4|DEFAULT|LARGE_DATA" in dds_transport_ab
    assert "Runtime restart not requested; no production process was stopped." in dds_transport_ab
    assert "FASTDDS_BUILTIN_TRANSPORTS must be set before every ROS participant starts" in dds_transport_ab
    assert "NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ" in dds_transport_ab
    assert "PointCloud2 obstacle profiles have been removed" in set_local_profile
    assert "--profile local_branch|trunk" not in set_local_profile
    assert "NJRH_FORCE_RESTART_DRIVER=true" not in set_local_profile
    assert "ros2 topic hz /perception/obstacle_points" not in set_local_profile
    assert "pointcloud_axis_remap" in inspect_cpu_affinity
    assert "pid=%s tid=%s cpu=%s pcpu=%s" in inspect_cpu_affinity
    assert 'print_threads_for_pattern "robot_local_perception"' not in inspect_cpu_affinity
    assert "/perception/local_perception_status" not in inspect_cpu_affinity
    assert "nav_cloud_preprocessor" in inspect_cpu_affinity
    assert "pointcloud_to_laserscan" in inspect_cpu_affinity
    assert "scan_republisher" in inspect_cpu_affinity
    assert "global_localization/localizer" in inspect_cpu_affinity
    assert "NJRH_CPUSET_" in inspect_cpu_affinity
    assert "WARN local_branch is enabled but local input <10Hz" not in inspect_cpu_affinity
    assert "thermal and clock snapshot" in inspect_cpu_affinity
    assert "tegrastats" in inspect_cpu_affinity
    assert "nvpmodel" in inspect_cpu_affinity
    assert "jetson_clocks" in inspect_cpu_affinity
    assert "record_pointcloud_nav_acceptance.sh [--duration-sec SECONDS]" in nav_acceptance
    assert "/lidar/axis_remap_status" in nav_acceptance
    assert "/perception/local_perception_status" not in nav_acceptance
    assert "/perception/obstacle_points" not in nav_acceptance
    assert "/perception/clearing_points" not in nav_acceptance
    assert "/lidar/pointcloud_accel_status" in nav_acceptance
    assert "/scan" in nav_acceptance
    assert "/safety/status" in nav_acceptance
    assert "/local_state/odometry" in nav_acceptance
    assert "/localization/bridge_status" in nav_acceptance
    assert "NJRH_RECORD_HEAVY_POINTCLOUDS=true" not in nav_acceptance
    assert "axis lidar_points_publish_hz" in nav_acceptance
    assert "accel scan_publish_hz" in nav_acceptance
    assert "CASE_I_LOCAL_BRANCH_DRAGS_TRUNK seen" not in nav_acceptance
    assert "FAST-LIO2 navigation residue seen" in nav_acceptance
    assert "timeout 8 ros2 lifecycle get /bt_navigator" in nav_acceptance
    assert "input_topic: /lidar_points" in pointcloud_downsample_cfg
    assert "input_qos_depth: 1" in pointcloud_downsample_cfg
    assert "input_reliable: false" in pointcloud_downsample_cfg
    assert "output_frame_id: lidar_link" in pointcloud_downsample_cfg
    assert raw_to_canonical not in pointcloud_downsample_cfg
    assert identity_matrix in pointcloud_downsample_cfg
    assert "nav_output_topic: /lidar_points_nav" in pointcloud_downsample_cfg
    assert "nav_output_stride: 1" in pointcloud_downsample_cfg
    assert "nav_output_qos_depth: 1" in pointcloud_downsample_cfg
    assert 'local_output_topic: ""' in pointcloud_downsample_cfg
    assert "local_output_stride: 1" in pointcloud_downsample_cfg
    assert "fast_path_identity" in pointcloud_remap_cpp
    assert "input_reliable: false" in pointcloud_remap_cfg
    assert 'declare_parameter<bool>("input_reliable", false)' in pointcloud_remap_cpp
    assert "publish_downsample" in pointcloud_remap_cpp
    assert "result.subscription_count = publisher->get_subscription_count()" in pointcloud_remap_cpp
    assert "result.subscription_count == 0U" in pointcloud_remap_cpp
    assert "result.skipped = true" in pointcloud_remap_cpp
    assert "stride <= 1U" in pointcloud_remap_cpp
    assert "std::make_unique<sensor_msgs::msg::PointCloud2>(cloud)" in pointcloud_remap_cpp
    assert "publisher_->publish(*output)" in pointcloud_remap_cpp
    assert pointcloud_remap_cpp.index("publisher_->publish(*output)") < pointcloud_remap_cpp.index(
        "publish_downsample(local_publisher_"
    )
    assert pointcloud_remap_cpp.index("publish_downsample(local_publisher_") < pointcloud_remap_cpp.index(
        "publish_downsample(nav_publisher_"
    )
    assert "*output = std::move(*msg)" not in pointcloud_remap_cpp
    assert "pointcloud_downsample_node src/pointcloud_downsample_node.cpp" in hesai_cmake
    assert 'declare_parameter<std::string>("input_topic", "/jt128/vendor/points_raw")' in pointcloud_downsample_cpp
    assert 'declare_parameter<std::string>("nav_output_topic", "/lidar_points_nav")' in pointcloud_downsample_cpp
    assert 'declare_parameter<std::string>("local_output_topic", "")' in pointcloud_downsample_cpp
    assert 'declare_parameter<std::string>("output_frame_id", "lidar_link")' in pointcloud_downsample_cpp
    assert 'declare_parameter<bool>("input_reliable", false)' in pointcloud_downsample_cpp
    assert "rotate_point" in pointcloud_downsample_cpp
    assert "rotate_point_fast_xy" in pointcloud_downsample_cpp
    assert "fast_path_neg_raw_y_neg_raw_x" in pointcloud_downsample_cpp
    assert "PointCloudDownsampleNode" in pointcloud_downsample_cpp


def test_robot_api_server_keeps_tf_subscription_resident():
    entry_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    localization_feature_cpp = localization_feature_source()
    localization_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    system_status_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "system_status"
        / "system_status_module.cpp"
    ).read_text(encoding="utf-8")
    subscription_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "application"
        / "subscriptions"
        / "subscription_module.cpp"
    ).read_text(encoding="utf-8")

    assert "TF is a process-level localization input" in subscription_module_cpp
    tf_resident_index = subscription_module_cpp.index("// TF is a process-level localization input")
    assert "ports.ensure_tf_resident" in subscription_module_cpp[tf_resident_index:]
    assert (
        "module_->ensure_tf_subscription_active();"
        in localization_feature_cpp
    )
    assert "ensure_floor_status_subscription_active();" in system_status_cpp
    assert "floor_status_subscription.reset()" not in system_status_cpp
    assert "Lease transitions only" in subscription_module_cpp
    assert "bool tf_subscription_active()" not in entry_cpp + localization_cpp

    tf_function = subscription_module_cpp[
        subscription_module_cpp.index('resource == "tf"'):
        subscription_module_cpp.index('resource == "teleop"')
    ]
    assert "ports.ensure_tf_resident" in tf_function
    assert "tf_sub_.reset()" not in tf_function
    assert "have_pose_ = false" not in tf_function

    wait_function = localization_cpp[
        localization_cpp.index("RobotPoseSnapshot wait_for_current_robot_pose"):
        localization_cpp.index("TfChainFreshnessSnapshot tf_chain_freshness_snapshot")
    ]
    assert "ensure_tf_subscription_active();" in wait_function
    assert "tf_sub_.reset()" not in wait_function
    assert "was_active" not in wait_function


def test_mapping_stop_only_cleans_private_fastlio_residuals():
    entry_code = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    composition_code = application_composition_source()
    mapping_module_code = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "mapping"
        / "mapping_module.cpp"
    ).read_text(encoding="utf-8")
    mapping_configuration_code = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "mapping"
        / "mapping_configuration_module.cpp"
    ).read_text(encoding="utf-8")
    mapping_runtime_code = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "mapping"
        / "runtime"
        / "mapping_process_runtime.cpp"
    ).read_text(encoding="utf-8")
    api_config = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    runtime_utils = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "infrastructure"
        / "process"
        / "runtime_process_utils.cpp"
    ).read_text(
        encoding="utf-8"
    )
    mapping_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_projected_map.sh"
    ).read_text(encoding="utf-8")

    assert "read_proc_environ(process_id)" in mapping_runtime_code
    assert "is_private_slam2d_fastlio_process" in mapping_runtime_code
    assert "NJRH_SLAM2D_PRIVATE_FASTLIO=1" in mapping_runtime_code
    assert "read_proc_environ" in runtime_utils
    assert "run_mapping_start_job_guarded" in mapping_module_code
    assert "MappingConfigurationModule::declare_parameters" in composition_code
    assert "mapping_lidar_rps_xps_state_dir" in mapping_configuration_code
    assert "/tmp/njrh_slam2d_lidar_rps_xps" in mapping_configuration_code
    assert 'declare_parameter<std::string>("mapping_lidar_rps_xps_state_dir"' not in entry_code
    assert "restore_lidar_rps_xps_state()" in mapping_runtime_code
    assert "is_safe_mapping_lidar_rps_xps_path" in mapping_runtime_code
    assert "rps_xps.tsv" in mapping_runtime_code
    assert 'path.rfind("/sys/class/net/", 0U)' in mapping_runtime_code
    assert 'path.find("/queues/")' in mapping_runtime_code
    assert '"/rps_cpus"' in mapping_runtime_code
    assert '"/xps_cpus"' in mapping_runtime_code
    for config_text in (api_config, overlay_api_config):
        assert 'mapping_lidar_rps_xps_state_dir: "/tmp/njrh_slam2d_lidar_rps_xps"' in config_text

    residual_fn = mapping_runtime_code[
        mapping_runtime_code.index("bool is_mapping_2d_residual_process_command"):
        mapping_runtime_code.index("bool is_safe_mapping_lidar_rps_xps_path")
    ]
    fastlio_fn = mapping_runtime_code[
        mapping_runtime_code.index("bool is_private_slam2d_fastlio_process"):
        mapping_runtime_code.index("bool is_mapping_2d_residual_process_command")
    ]
    terminate_fn = mapping_runtime_code[
        mapping_runtime_code.index("std::size_t MappingProcessRuntime::terminate_process_groups_locked"):
        mapping_runtime_code.index("std::size_t MappingProcessRuntime::stop()")
    ]

    assert "nav_cloud_preprocessor" in residual_fn
    assert "pointcloud_to_laserscan_node" in residual_fn
    assert "NJRH_SLAM2D_MAPPING_PIPELINE=1" in residual_fn
    assert "scan_republisher_node" not in residual_fn
    assert "return is_private_slam2d_fastlio_process(cmdline, environ);" in residual_fn
    assert "ros2 run fast_lio fastlio_mapping" in fastlio_fn
    assert "environ.find(\"NJRH_SLAM2D_PRIVATE_FASTLIO=1\")" in fastlio_fn
    assert "nav_cloud_preprocessor" in mapping_script
    assert "pointcloud_to_laserscan" in mapping_script
    assert "scan_republisher_node" not in mapping_script
    assert "restore_lidar_rps_xps_state();" in terminate_fn
    assert "config_.graceful_stop_timeout_sec" in terminate_fn
    assert "scan_owner_restore_timeout_sec" in mapping_module_code
    assert "wait_for_mapping_scan_owner_restored" in mapping_module_code
    assert "ensure_mapping_scan_owner_restored" in mapping_module_code
    assert "mapping stop failed to restore canonical navigation /scan ownership" in mapping_module_code
    assert "set_mapping_runtime_state(false, \"failed\"" in mapping_module_code
    for config_text in (api_config, overlay_api_config):
        assert "mapping_graceful_stop_timeout_sec: 30.0" in config_text
        assert "mapping_scan_owner_restore_timeout_sec: 30.0" in config_text

    restore_fn = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "scan_ownership_helpers.sh"
    ).read_text(encoding="utf-8")
    restore_fn = restore_fn.split("restore_navigation_scan_owner() {", 1)[1]
    assert 'runtime_readiness_probe scan-handoff ensure' in restore_fn
    assert 'wait_for_scan_publisher_count "${topic}" 0 1' not in restore_fn
    handoff = (ROOT / "src/robot_bringup/src/mapping_scan_handoff.hpp").read_text(encoding="utf-8")
    assert 'fresh_disabled && publishers.empty()' in handoff
    assert 'acknowledged_ && publishers.empty()' in handoff
    assert 'wire.source_timestamp < started_ns_' in handoff


def test_localization_bridge_latches_one_shot_localizer_pose():
    bridge_code = (ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp").read_text(encoding="utf-8")
    bridge_cmake = (ROOT / "src" / "robot_localization_bridge" / "CMakeLists.txt").read_text(encoding="utf-8")
    bridge_package = (ROOT / "src" / "robot_localization_bridge" / "package.xml").read_text(encoding="utf-8")
    bridge_cfg = (ROOT / "src" / "robot_localization_bridge" / "config" / "localization_bridge.yaml").read_text(encoding="utf-8")
    overlay_bridge_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    entry_code = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    docking_status_code = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_status_module.cpp"
    ).read_text(encoding="utf-8")
    localization_code = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    pre_navigation_undock_code = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "pre_navigation_undock_module.cpp"
    ).read_text(encoding="utf-8")
    api_code = localization_code + pre_navigation_undock_code
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    overlay_api_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    assert "has_last_pose_stamp_used_" in bridge_code
    assert 'refresh_state("pose")' in bridge_code
    assert 'refresh_state("status_timer")' in bridge_code
    assert "publish_map_to_odom_from_state()" in bridge_code
    assert 'bridge waiting for localization_result' in bridge_code
    assert "else if (!has_map_to_odom_)" in bridge_code
    assert "if (!has_map_to_odom_)" in bridge_code
    assert "tf.transform.translation.x = state.transform.x" in bridge_code
    assert "tf.transform.rotation = quaternion_from_yaw(state.transform.yaw)" in bridge_code
    assert '#include "std_srvs/srv/trigger.hpp"' in bridge_code
    assert "forced_jump_threshold_m" in bridge_code
    assert "force_accept_next_pose_" in bridge_code
    assert "force accepting next localization_result" in bridge_code
    assert "bridge forced map->odom jump rejected" in bridge_code
    assert "create_service<std_srvs::srv::Trigger>" in bridge_code
    assert "find_package(std_srvs REQUIRED)" in bridge_cmake
    assert "std_srvs" in bridge_cmake
    assert "<depend>std_srvs</depend>" in bridge_package
    for cfg in (bridge_cfg, overlay_bridge_cfg):
        assert "forced_jump_threshold_m: 20.0" in cfg
        assert "force_accept_service: /robot_localization_bridge/force_accept_next_localization" in cfg
    assert "request_localization_bridge_force_accept" not in api_code
    assert "global localization wrapper owns bridge force-accept" in localization_code
    assert "trigger_localization_and_wait_for_result(" in api_code
    assert "&relocalization_sequence" in docking_status_code
    assert "post_undock_relocalization_succeeded" in docking_status_code
    assert "undocked before navigation; post-undock navigation readiness failed, Nav2 goal not sent" in api_code
    assert "trigger_localization_and_wait_for_result(" not in entry_code
    for cfg in (api_cfg, overlay_api_cfg):
        assert "localization_bridge_force_accept_service" not in cfg


def test_localization_sensing_and_mapping_scan_contracts_are_explicit():
    occupancy_stack = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization_stack.launch.py").read_text(encoding="utf-8")
    localization_sensing = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_localization_sensing.launch.py").read_text(encoding="utf-8")
    localization_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    slam_mapping = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_slam_toolbox_mapping.launch.py").read_text(encoding="utf-8")
    slam_scan_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_scan_slam2d.yaml").read_text(encoding="utf-8")
    preprocessor_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_nav_cloud_preprocessor.yaml").read_text(encoding="utf-8")
    preprocessor_qos_patch = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "patches" / "jt128_nav_tools_pointcloud_qos.patch"
    ).read_text(encoding="utf-8")
    flatscan_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_flatscan.yaml").read_text(encoding="utf-8")
    localizer_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_occupancy_grid_localizer.yaml"
    ).read_text(encoding="utf-8")
    assert "jt128_nav_sensing.launch.py" not in occupancy_stack
    assert "jt128_localization_sensing.launch.py" in occupancy_stack
    assert 'points_topic = LaunchConfiguration("points_topic")' in occupancy_stack
    assert '"points_topic": points_topic' in occupancy_stack
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points_nav")' in occupancy_stack
    assert 'overlay_root / "config" / "jt128_scan_slam2d.yaml"' in localization_sensing
    assert "jt128_flatscan.yaml" in localization_sensing
    assert "scan_republisher_node" in localization_sensing
    assert "scan_republisher_node" in localization_script
    assert "NJRH_NAV_LOCAL_STATE_MODE:-ekf" in localization_script
    assert 'POINTS_TOPIC="${NJRH_LOCALIZATION_POINTS_TOPIC:-/lidar_points_nav}"' in localization_script
    assert "FASTLIO_CONFIG_FILE" not in localization_script
    assert "start_or_reuse_fastlio_for_local_state" not in localization_script
    assert "fastlio_runtime_topics_fresh()" not in localization_script
    assert "resident FAST-LIO runtime is stale" not in localization_script
    assert "ros2 run fast_lio fastlio_mapping" not in localization_script
    assert "-r /tf:=/tf_fastlio_internal" not in localization_script
    assert "ensure_resident_fastlio_for_local_state()" in localization_script
    assert "ensure_resident_local_state_for_localization()" in localization_script
    assert "cleanup_canonical_helpers" not in localization_script
    assert "not owned by the" in localization_script
    assert "occupancy localization mode" in localization_script
    assert "ensure_localization_pointcloud_ready" in localization_script
    assert "repair_jt128_navigation_points" in localization_script
    assert "NJRH_LOCALIZATION_POINTS_DRIVER_REPAIR" in localization_script
    assert '"points_topic:=${POINTS_TOPIC}"' in localization_script
    assert 'env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"' not in localization_script
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' not in localization_script
    assert "nav_points_topic" in localization_sensing
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points_nav")' in localization_sensing
    assert '"output_frame_id": "lidar_level_link"' in localization_sensing
    assert "input_topic: /lidar_points_nav\n" in preprocessor_cfg
    assert "output_frame_id: lidar_level_link" in preprocessor_cfg
    assert "input_reliable: false" in preprocessor_cfg
    assert "input_qos_depth: 1" in preprocessor_cfg
    assert "output_reliable: false" in preprocessor_cfg
    assert "output_qos_depth: 1" in preprocessor_cfg
    assert "status_topic: /lidar/nav_cloud_preprocessor_status" in preprocessor_cfg
    assert "status_publish_period_sec: 1.0" in preprocessor_cfg
    assert "makePointCloudQos" in preprocessor_qos_patch
    assert "find_package(std_msgs REQUIRED)" in preprocessor_qos_patch
    assert "<depend>std_msgs</depend>" in preprocessor_qos_patch
    assert 'declare_parameter<bool>("input_reliable", false)' in preprocessor_qos_patch
    assert 'declare_parameter<bool>("output_reliable", false)' in preprocessor_qos_patch
    assert 'declare_parameter<std::string>("status_topic", "/lidar/nav_cloud_preprocessor_status")' in preprocessor_qos_patch
    assert "input_callback_hz" in preprocessor_qos_patch
    assert "input_accept_hz" in preprocessor_qos_patch
    assert "output_points_nav_hz" in preprocessor_qos_patch
    assert "skipped_transform" in preprocessor_qos_patch
    assert "skipped_empty" in preprocessor_qos_patch
    assert "skipped_filter_empty" in preprocessor_qos_patch
    assert "input_interarrival_ms_avg" in preprocessor_qos_patch
    assert "input_last_msg_age_ms" in preprocessor_qos_patch
    assert "processing_ms_avg" in preprocessor_qos_patch
    assert "lookup_timeout_sec" in preprocessor_qos_patch
    assert "target_frame" in preprocessor_qos_patch
    assert "height_filter_min" in preprocessor_qos_patch
    assert "subscriber_count" in preprocessor_qos_patch
    assert "publisher_subscription_count" in preprocessor_qos_patch
    assert "input_topic_, makePointCloudQos(input_qos_depth_, input_reliable_)" in preprocessor_qos_patch
    assert "output_topic_, makePointCloudQos(output_qos_depth_, output_reliable_)" in preprocessor_qos_patch
    assert "lookup_timeout_sec: 0.03" in preprocessor_cfg
    assert '("cloud_in", nav_points_topic)' in localization_sensing
    assert '("scan", "/scan_raw")' in localization_sensing
    assert '("scan", scan_topic)' in localization_sensing
    assert '("flatscan", flatscan_topic)' in localization_sensing
    assert 'overlay_root / "config" / "jt128_scan_slam2d.yaml"' in slam_mapping
    assert 'DeclareLaunchArgument("scan_topic", default_value="/scan")' in slam_mapping
    assert "target_frame: lidar_level_link" in slam_scan_cfg
    assert "min_height: -0.75" in slam_scan_cfg
    assert "max_height: 0.35" in slam_scan_cfg
    assert "drop_invalid_ranges: true" in flatscan_cfg
    assert "min_scan_fov_degrees: 115.0" in localizer_cfg
    assert "max_beam_error: 0.50" in localizer_cfg


def test_phase111_pointcloud_triage_contracts():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    config_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "config"
    local_node = (ROOT / "src" / "robot_local_perception" / "src" / "local_perception_node.cpp").read_text(
        encoding="utf-8"
    )
    nav_patch = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "patches" / "jt128_nav_tools_pointcloud_qos.patch"
    ).read_text(encoding="utf-8")
    local_cfg = (config_dir / "local_perception.yaml").read_text(encoding="utf-8")
    local_branch_cfg_path = config_dir / "jt128_canonical_pointcloud_remap_local_branch.yaml"
    local_profile = (config_dir / "local_perception_input_profile.env").read_text(encoding="utf-8")
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")

    required_scripts = [
        "diagnose_local_perception_pipeline.sh",
        "diagnose_nav_scan_pipeline.sh",
        "diagnose_pointcloud_cpu_pressure.sh",
        "run_pointcloud_cpu_affinity_ab.sh",
    ]
    for name in required_scripts:
        path = scripts_dir / name
        assert path.exists(), name
        text = path.read_text(encoding="utf-8")
        assert "killall -9" not in text
        assert "pkill -9" not in text
        assert "pkill -f" not in text
        assert "ros2 topic hz /lidar_points" not in text

    assert "timer_tick_hz" in local_node
    assert "no_new_hz" in local_node
    assert "skipped_empty_input" in local_node
    assert "skipped_empty_obstacle" in local_node
    assert "skipped_mode_gating" in local_node
    assert "skipped_publish_gating" in local_node
    assert "processing_ms_avg" in local_node
    assert "last_filtered_points" in local_node
    assert "last_obstacle_points" in local_node
    assert "active_profile_name" in local_node
    assert "clearing_publish_every_n" in local_node

    assert "input_accept_hz" in nav_patch
    assert "skipped_empty" in nav_patch
    assert "skipped_filter_empty" in nav_patch
    assert "input_interarrival_ms_avg" in nav_patch
    assert "input_last_msg_age_ms" in nav_patch
    assert "processing_ms_avg" in nav_patch
    assert "lookup_timeout_sec" in nav_patch
    assert "target_frame" in nav_patch
    assert "source_frame" in nav_patch
    assert "publisher_subscription_count" in nav_patch

    assert "enabled: false" in local_cfg
    assert 'input_topic: ""' in local_cfg
    assert 'output_topic: ""' in local_cfg
    assert 'clearing_output_topic: ""' in local_cfg
    assert 'status_topic: ""' in local_cfg
    assert "/perception/obstacle_points" not in local_cfg
    assert "/perception/clearing_points" not in local_cfg
    assert "input_topic: /_internal/lidar_points_local" not in local_cfg
    assert "export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=disabled" in local_profile
    assert not local_branch_cfg_path.exists()
    assert "input_reliable: false" in local_cfg
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert 'export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env
    assert (
        'planner_plugins: ["GridBased", "ElevatorScoped", '
        '"ElevatorHallScoped", "ElevatorReverseEntryStagingScoped", '
        '"ElevatorReverseDockingScoped"]'
    ) in nav2
    assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in nav2
    assert (
        'controller_plugins: ["FollowPath", "FollowPathFallback", '
        '"ElevatorFollowPath", "ElevatorHallFollowPath", '
        '"ElevatorReverseEntryStagingFollowPath", '
        '"ElevatorReverseDockingFollowPath"]'
    ) in nav2
    assert 'primary_controller: "robot_nav_config::RangerMPPIController"' in nav2
    assert 'plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"' in nav2


def test_phase112_cpu_irq_experiment_contracts():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    cpu_affinity = (scripts_dir / "cpu_affinity.sh").read_text(encoding="utf-8")

    scripts = {
        name: (scripts_dir / name).read_text(encoding="utf-8")
        for name in [
            "collect_cpu_irq_softirq_snapshot.sh",
            "identify_lidar_network_irq.sh",
            "run_cpu_core_allocation_ab.sh",
            "run_lidar_irq_affinity_ab.sh",
            "run_pointcloud_cpu_irq_experiment.sh",
        ]
    }

    for name, text in scripts.items():
        assert "pkill -9" not in text, name
        assert "killall" not in text, name
        assert "chrt" not in text, name
        assert "SCHED_FIFO" not in text, name
        assert "ros2 topic hz /lidar_points" not in text, name
        assert "rmw_cyclonedds_cpp" not in text, name

    assert "cpu_affinity_runtime_override.env" in cpu_affinity
    assert "NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE" in cpu_affinity

    snapshot = scripts["collect_cpu_irq_softirq_snapshot.sh"]
    assert "/proc/interrupts" in snapshot
    assert "/proc/softirqs" in snapshot
    assert "NET_RX" in snapshot
    assert "ksoftirqd/0" in snapshot
    assert "RPS / XPS" in snapshot
    assert "mode: read-only" in snapshot
    assert "ros2 " not in snapshot

    identify = scripts["identify_lidar_network_irq.sh"]
    assert "Read-only LiDAR network interface and IRQ detector" in identify
    assert "candidate_lidar_ips" in identify
    assert "ssh_interface_risk" in identify
    assert "--interface" in identify
    assert "smp_affinity_list" in identify

    cpu_ab = scripts["run_cpu_core_allocation_ab.sh"]
    assert "cpu_affinity_runtime_override.env" in cpu_ab
    assert "resolved_profile_overrides" in cpu_ab
    assert "split_local_nav_v1" in cpu_ab
    assert "split_local_nav_v2" in cpu_ab
    assert "njrh_apply_affinity_to_pids" in cpu_ab
    assert "live affinity reapplied; no process was killed" in cpu_ab
    assert 'matches="$(pgrep -f "$1" 2>/dev/null || true)"' in cpu_ab

    irq_ab = scripts["run_lidar_irq_affinity_ab.sh"]
    assert "irq_keep_default" in irq_ab
    assert "lidar_irq_cpu5" in irq_ab
    assert "lidar_irq_cpu7" in irq_ab
    assert "lidar_irq_split_5_7" in irq_ab
    assert "rps_xps_cpu5" in irq_ab
    assert "rps_xps_5_7" in irq_ab
    assert 'rps_xps_cpu5|rps_xps_5_7)' in irq_ab
    assert 'echo "unchanged"' in irq_ab
    assert "profile_changes_irq" in irq_ab
    assert "--allow-ssh-interface-risk" in irq_ab
    assert "refusing --apply without --allow-ssh-interface-risk" in irq_ab
    assert "backup captured" in irq_ab
    assert "restore_previous" in irq_ab
    assert "smp_affinity_list" in irq_ab
    assert "rps_cpus" in irq_ab
    assert "xps_cpus" in irq_ab
    assert "will_not_change=QoS,DDS,timestamps,Nav2_planner_controller,EKF,FAST-LIO2,ROS_processes" in irq_ab

    experiment = scripts["run_pointcloud_cpu_irq_experiment.sh"]
    assert "run_cpu_core_allocation_ab.sh" in experiment
    assert "run_lidar_irq_affinity_ab.sh" in experiment
    assert "collect_cpu_irq_softirq_snapshot.sh" in experiment
    assert "diagnose_local_perception_pipeline.sh" in experiment
    assert "diagnose_nav_scan_pipeline.sh" in experiment
    assert "verify_pointcloud_delivery_matrix.sh" in experiment
    assert "trap cleanup_restore EXIT" in experiment
    assert "--keep-applied" in experiment
    assert "This experiment does not change ROS QoS" in experiment


def test_phase113_pointcloud_accel_profile_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"
    config_dir = overlay / "config"
    launch_dir = overlay / "launch"
    hesai_overlay_dir = ROOT / "src" / "third_party" / "hesai_lidar_ros2_overlay"

    required_scripts = {
        "pointcloud_accel_profile.sh",
        "set_pointcloud_accel_profile.sh",
        "run_pointcloud_accel_pipeline.sh",
        "verify_pointcloud_accel_profile.sh",
        "run_pointcloud_accel_ab.sh",
        "check_isaac_ros_nitros_env.sh",
    }
    for name in required_scripts:
        assert (scripts_dir / name).exists(), name

    profile_env = (config_dir / "pointcloud_accel_profile.env").read_text(encoding="utf-8")
    ingress_env = (config_dir / "pointcloud_ingress_profile.env").read_text(encoding="utf-8")
    hesai_accel_cfg = (config_dir / "hesai_accel_driver.yaml").read_text(encoding="utf-8")
    accel_cfg = (config_dir / "pointcloud_accel_axis.yaml").read_text(encoding="utf-8")
    accel_profile = (scripts_dir / "pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    set_profile = (scripts_dir / "set_pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    run_pipeline = (scripts_dir / "run_pointcloud_accel_pipeline.sh").read_text(encoding="utf-8")
    verify_profile = (scripts_dir / "verify_pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    ab_runner = (scripts_dir / "run_pointcloud_accel_ab.sh").read_text(encoding="utf-8")
    nitros_check = (scripts_dir / "check_isaac_ros_nitros_env.sh").read_text(encoding="utf-8")
    run_driver = (scripts_dir / "run_driver.sh").read_text(encoding="utf-8")
    common_services = (scripts_dir / "run_common_services.sh").read_text(encoding="utf-8")
    nav2_navigation = (scripts_dir / "run_nav2_navigation.sh").read_text(encoding="utf-8")
    occupancy = (scripts_dir / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    local_costmap_debug = (scripts_dir / "run_local_costmap_debug.sh").read_text(encoding="utf-8")
    cpu_affinity = (config_dir / "cpu_affinity.env").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    accel_launch = (launch_dir / "pointcloud_accel_pipeline.launch.py").read_text(encoding="utf-8")
    hesai_cmake = (ROOT / "src" / "robot_hesai_jt128" / "CMakeLists.txt").read_text(encoding="utf-8")
    hesai_package = (ROOT / "src" / "robot_hesai_jt128" / "package.xml").read_text(encoding="utf-8")
    legacy_axis = (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_axis_remap_node.cpp").read_text(
        encoding="utf-8"
    )
    accel_axis_wrapper = (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_accel_axis_node.cpp").read_text(
        encoding="utf-8"
    )
    accel_core_header = (
        ROOT / "src" / "robot_hesai_jt128" / "include" / "robot_hesai_jt128" / "pointcloud_accel_core.hpp"
    ).read_text(encoding="utf-8")
    accel_core = (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_accel_core.cpp").read_text(
        encoding="utf-8"
    )
    hesai_accel_driver = (ROOT / "src" / "robot_hesai_jt128" / "src" / "hesai_accel_driver_node.cpp").read_text(
        encoding="utf-8"
    )
    hesai_overlay_cmake = (hesai_overlay_dir / "CMakeLists.txt").read_text(encoding="utf-8")
    hesai_overlay_package = (hesai_overlay_dir / "package.xml").read_text(encoding="utf-8")
    hesai_overlay_source_driver = (
        hesai_overlay_dir / "src" / "manager" / "source_driver_ros2.hpp"
    ).read_text(encoding="utf-8")
    hesai_overlay_node = (hesai_overlay_dir / "node" / "hesai_ros_driver_node.cc").read_text(encoding="utf-8")
    accel_axis = accel_axis_wrapper + "\n" + accel_core
    nitros_cmake = (ROOT / "src" / "robot_isaac_nitros_pointcloud" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    nitros_package = (ROOT / "src" / "robot_isaac_nitros_pointcloud" / "package.xml").read_text(
        encoding="utf-8"
    )

    assert 'export NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE:-ipc_worker}"' in profile_env
    assert 'export NJRH_POINTCLOUD_INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}"' in ingress_env
    assert "separate_process" in accel_profile
    assert "driver_integrated" in accel_profile
    assert "nitros" in accel_profile
    assert "ipc_worker" in accel_profile
    assert "--profile ipc_worker|nitros" in set_profile
    assert "legacy profile was removed" in set_profile
    assert "--ingress-profile separate_process|driver_integrated" in set_profile
    assert "NJRH_POINTCLOUD_ACCEL_RESTART=true" in set_profile
    assert "NITROS profile was not written" in set_profile
    assert "pkill -9" not in set_profile
    assert "killall" not in set_profile
    assert "DEFAULT_HESAI_CONFIG_FILE" in run_driver
    assert "REPO_HESAI_CONFIG_FILE" in run_driver
    assert 'src/third_party/hesai_lidar_ros2_overlay/config/config.yaml' in run_driver
    assert "hesai_driver_config_path: /workspaces/njrh-v3/workspace1/src/third_party/hesai_lidar_ros2_overlay/config/config.yaml" in hesai_accel_cfg
    assert "legacy)" in run_pipeline
    assert "FAIL legacy profile removed" in run_pipeline
    assert "ipc_worker)" in run_pipeline
    assert "nitros)" in run_pipeline
    assert "NITROS profile not started" in run_pipeline
    assert "NJRH_POINTCLOUD_INGRESS_PROFILE=driver_integrated" in run_pipeline
    assert "driver_integrated ingress selected" in run_pipeline
    assert "standalone hesai_ros_driver_node or pointcloud_accel_axis_node" in run_pipeline
    assert 'NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}"' in run_pipeline
    assert "run_local_perception.sh" not in run_pipeline
    assert "NJRH_POINTCLOUD_ACCEL_PROFILE=legacy" not in run_pipeline
    assert "jt128_localization_sensing.launch.py" not in run_pipeline
    assert "legacy scan chain already running; reusing" not in run_pipeline
    assert "pkill -TERM -f" in run_pipeline
    assert "laser_scan_to_flatscan" in run_pipeline
    assert "pointcloud_accel_axis_node scan worker publishes /scan" in run_pipeline
    assert "pkill -9" not in run_pipeline
    assert "killall" not in run_pipeline
    for protected_process in (
        "controller_server",
        "planner_server",
        "bt_navigator",
        "map_server",
        "robot_local_state",
        "robot_safety",
        "ranger_base_node",
        "robot_api_server",
        "robot_floor_manager",
    ):
        assert protected_process not in run_pipeline
    assert "pkill -9" not in run_driver
    assert "killall -9" not in run_driver
    assert "--duration-sec" in ab_runner
    assert "pointcloud_accel_ab_" in ab_runner
    assert "check_isaac_ros_nitros_env.sh" in run_pipeline
    assert "isaac_ros_nitros" in nitros_check
    assert "isaac_ros_managed_nitros" in nitros_check
    assert "isaac_ros_nitros_point_cloud_type" in nitros_check
    assert "NitrosPointCloud" in nitros_check
    assert "use NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker or legacy" not in nitros_check

    assert "pointcloud_accel_axis_node src/pointcloud_accel_axis_node.cpp" in hesai_cmake
    assert "pointcloud_accel_core src/pointcloud_accel_core.cpp" in hesai_cmake
    assert "hesai_accel_driver_node src/hesai_accel_driver_node.cpp" in hesai_cmake
    assert "install(DIRECTORY include/ DESTINATION include)" in hesai_cmake
    assert "install(TARGETS" in hesai_cmake
    assert "  pointcloud_accel_core" in hesai_cmake
    assert "ament_export_libraries(pointcloud_accel_core)" in hesai_cmake
    assert "ament_export_targets(export_pointcloud_accel_core HAS_LIBRARY_TARGET)" in hesai_cmake
    assert "  pointcloud_accel_axis_node" in hesai_cmake
    assert "  hesai_accel_driver_node" in hesai_cmake
    assert "find_package(tf2_ros REQUIRED)" in hesai_cmake
    assert "geometry_msgs" in hesai_cmake
    assert "<depend>tf2_ros</depend>" in hesai_package
    assert "<depend>geometry_msgs</depend>" in hesai_package
    assert "class PointCloudAccelCore" in accel_core_header
    assert "DecodedCloudView" in accel_core_header
    assert "PointCloudAccelCoreOptions" in accel_core_header
    assert "process_pointcloud2" in accel_core_header
    assert "process_decoded_points" in accel_core_header
    assert "PointCloudAccelCore" in accel_axis_wrapper
    assert "struct NormalizedPointView" not in accel_axis_wrapper
    assert "trunk_publisher_" not in accel_axis_wrapper
    assert "create_subscription<sensor_msgs::msg::PointCloud2>" in accel_axis_wrapper
    assert "core_->process_pointcloud2" in accel_axis_wrapper
    assert "DRIVER_INTEGRATED_UNAVAILABLE_REASON" in hesai_accel_driver
    assert "repo_owned_hesai_driver_overlay_not_available" in hesai_accel_driver
    assert "PointCloudAccelCoreOptions" in hesai_accel_driver
    assert "hesai_accel_driver_node" in hesai_overlay_cmake
    assert "HESAI_ACCEL_DRIVER_INTEGRATED" in hesai_overlay_cmake
    assert 'HESAI_ROS_DRIVER_NODE_NAME="hesai_accel_driver_node"' in hesai_overlay_cmake
    assert "find_package(robot_hesai_jt128 REQUIRED)" in hesai_overlay_cmake
    assert "ROBOT_HESAI_JT128_POINTCLOUD_ACCEL_CORE_LIBRARY" in hesai_overlay_cmake
    assert 'target_link_libraries(hesai_accel_driver_node' in hesai_overlay_cmake
    assert "<depend condition=\"$ROS_VERSION == 2\">robot_hesai_jt128</depend>" in hesai_overlay_package
    assert "PointCloudAccelCore" in hesai_overlay_source_driver
    assert "driver_callback_pointcloud2" in hesai_overlay_source_driver
    assert "vendor_raw_ros_hop_required = false" in hesai_overlay_source_driver
    assert "publish_vendor_raw_debug_" in hesai_overlay_source_driver
    assert "accel_core_->process_pointcloud2(std::move(cloud))" in hesai_overlay_source_driver
    assert "pub_->publish(cloud)" in hesai_overlay_source_driver
    assert "HESAI_ROS_DRIVER_NODE_NAME" in hesai_overlay_node
    assert 'declare_parameter<std::string>("input_topic", "/jt128/vendor/points_raw")' in accel_axis
    assert 'declare_parameter<std::string>("output_topic", "/lidar_points")' in accel_axis
    assert 'Node("pointcloud_accel_axis_node", options)' in accel_axis
    assert 'declare_parameter<bool>("input_reliable", false)' in accel_axis
    assert 'declare_parameter<bool>("output_reliable", false)' in accel_axis
    assert 'declare_parameter<bool>("local_worker_enabled", false)' in accel_axis
    assert "PointCloud2 local obstacle worker is disabled" in accel_axis
    assert 'declare_parameter<bool>("local_worker_restamp_to_now", true)' in accel_axis
    assert 'declare_parameter<std::string>("local_worker_stamp_source", "")' in accel_axis
    assert 'declare_parameter<std::string>("local_worker_stamp_odom_topic", "/local_state/odometry")' in accel_axis
    assert 'declare_parameter<double>("local_worker_stamp_max_odom_age_sec", 0.25)' in accel_axis
    assert "create_subscription<nav_msgs::msg::Odometry>" in accel_axis
    assert "resolve_local_worker_output_stamp" in accel_axis
    assert 'declare_parameter<bool>("scan_worker_enabled", true)' in accel_axis
    assert 'declare_parameter<std::string>("flatscan_output_topic", "/flatscan")' in accel_axis
    assert "local_worker_enabled=" in accel_axis
    assert "scan_worker_enabled=" in accel_axis
    assert "struct NormalizedPointView" in accel_axis
    assert "struct LatestNormalizedBuffer" in accel_axis
    assert "std::vector<NormalizedPointView> points" in accel_axis
    assert "trunk_publisher_->publish(*output)" in accel_axis
    assert "latest_normalized_buffer_ = normalized_buffer" in accel_axis
    assert accel_axis.index("trunk_publisher_->publish(*output)") < accel_axis.index(
        "latest_normalized_buffer_ = normalized_buffer"
    )
    assert "latest_cloud_ = cloud" not in accel_axis
    assert "auto cloud = std::shared_ptr<sensor_msgs::msg::PointCloud2>(std::move(output))" not in accel_axis
    assert "local_worker_loop" in accel_axis
    assert "scan_worker_loop" in accel_axis
    assert "set_thread_name(\"pc_accel_local\")" in accel_axis
    assert "set_thread_name(\"pc_accel_scan\")" in accel_axis
    assert "local_compact_fields" in accel_axis
    assert "nav_compact_fields" in accel_axis
    assert "msg.point_step = 12U" in accel_axis
    assert "msg.point_step = 16U" in accel_axis
    assert "fast_path_raw_input_hz" in accel_axis
    assert "trunk_publish_gap_over_150ms_count" in accel_axis
    assert "trunk_output_subscription_count" in accel_axis
    assert "local_worker_obstacle_publish_hz" in accel_axis
    assert "scan_worker_scan_publish_hz" in accel_axis
    assert "internal_zero_copy_profile=true" in accel_axis
    assert "latest_internal_buffer_points" in accel_axis
    assert "local_worker_full_cloud_copy_count" in accel_axis
    assert "scan_worker_full_cloud_copy_count" in accel_axis
    assert "local_worker_intermediate_pointcloud_build_count" in accel_axis
    assert "scan_worker_intermediate_pointcloud_build_count" in accel_axis
    assert "local_worker_lock_wait_ms_max" in accel_axis
    assert "scan_worker_lock_wait_ms_max" in accel_axis
    assert "publish_downsample" not in accel_axis
    assert "publisher_->publish(*output)" in legacy_axis
    assert "publish_downsample(local_publisher_" in legacy_axis

    assert "pointcloud_accel_axis_node:" in accel_cfg
    assert "input_topic: /jt128/vendor/points_raw" in accel_cfg
    assert "output_topic: /lidar_points" in accel_cfg
    assert "output_qos_depth: 1" in accel_cfg
    assert "output_reliable: false" in accel_cfg
    assert "flatscan_output_topic: /flatscan" in accel_cfg
    assert "local_worker_enabled: false" in accel_cfg
    assert "worker_local_enabled: false" in accel_cfg
    assert 'local_output_topic: ""' in accel_cfg
    assert "local_compact_enabled: false" in accel_cfg
    assert 'nav_output_topic: ""' in accel_cfg
    assert "nav_compact_enabled: false" in accel_cfg
    assert "scan_worker_enabled: true" in accel_cfg
    assert "local_compact_fields: xyzi" in accel_cfg
    assert "local_compact_stride: 4" in accel_cfg
    assert "local_compact_max_rate_hz: 12.0" in accel_cfg
    assert "nav_compact_fields: xyzi" in accel_cfg
    assert "nav_compact_stride: 4" in accel_cfg
    assert "nav_compact_max_rate_hz: 10.0" in accel_cfg
    assert "local_worker_restamp_to_now: true" in accel_cfg
    assert "local_worker_stamp_source: local_odom" in accel_cfg
    assert "local_worker_stamp_odom_topic: /local_state/odometry" in accel_cfg
    assert "local_worker_stamp_max_odom_age_sec: 0.25" in accel_cfg
    assert "scan_output_topic: /scan" in accel_cfg
    assert "/points_nav" not in accel_cfg
    assert "driver_integrated_available: true" in hesai_accel_cfg
    assert "vendor_raw_ros_hop_required: false" in hesai_accel_cfg
    assert "input_path: driver_callback_pointcloud2" in hesai_accel_cfg
    assert "publish_vendor_raw_debug: false" in hesai_accel_cfg
    assert "publish_vendor_imu_raw_debug: true" in hesai_accel_cfg
    assert "lidar_points_full_density_full_fields: true" in hesai_accel_cfg
    assert "local_worker_enabled: false" in hesai_accel_cfg
    assert "worker_local_enabled: false" in hesai_accel_cfg
    assert 'local_output_topic: ""' in hesai_accel_cfg
    assert "local_compact_enabled: false" in hesai_accel_cfg
    assert 'nav_output_topic: ""' in hesai_accel_cfg
    assert "nav_compact_enabled: false" in hesai_accel_cfg
    assert "local_worker_restamp_to_now: true" in hesai_accel_cfg
    assert "local_worker_stamp_source: local_odom" in hesai_accel_cfg
    assert "local_worker_stamp_odom_topic: /local_state/odometry" in hesai_accel_cfg
    assert "local_worker_stamp_max_odom_age_sec: 0.25" in hesai_accel_cfg

    assert 'source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"' in run_driver
    assert "njrh_load_pointcloud_accel_profile" in run_driver
    assert "njrh_load_pointcloud_ingress_profile" in run_driver
    assert "NJRH_POINTCLOUD_INGRESS_PROFILE" in run_driver
    assert "HESAI_ACCEL_DRIVER_CPP_BIN" in run_driver
    assert "hesai_accel_driver_node" in run_driver
    assert "driver_integrated JT128 ingress" in run_driver
    assert "standalone hesai_ros_driver_node" not in run_driver
    assert 'install/hesai_ros_driver/lib/hesai_ros_driver/hesai_accel_driver_node' in run_driver
    assert '-p "config_path:=${RUNTIME_CONFIG_FILE}"' in run_driver
    assert "starting canonical imu remap for driver_integrated ingress" in run_driver
    assert "pointcloud_accel_axis.yaml" in run_driver
    assert "pointcloud_accel_axis_node" in run_driver
    assert "pointcloud_axis_remap_node" in run_driver
    assert "pointcloud_accel_container" in run_driver
    assert '-p "accel_profile:=${NJRH_POINTCLOUD_ACCEL_PROFILE}"' in run_driver
    assert "NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE=false" in run_driver
    assert "pointcloud_axis_remap|pointcloud_accel_axis" in run_driver
    assert "canonical /lidar_points" in run_driver

    assert "pointcloud_accel_pipeline_aux_running" in common_services
    assert "run_pointcloud_accel_pipeline.sh" in common_services
    assert "local_perception_common disabled" in common_services
    assert "local_perception helper disabled" in nav2_navigation
    assert "local_perception debug helper disabled" in local_costmap_debug
    assert "occupancy_localization.launch.py" in occupancy
    assert "occupancy_localization_stack.launch.py" in occupancy
    assert "require_common_pointcloud_for_localization" in occupancy
    assert "ensure_pointcloud_accel_pipeline_for_localization" not in occupancy
    assert "common-owned pointcloud accel pipeline is healthy" in occupancy
    assert "flatscan_publisher_ready_for_localization()" in occupancy
    assert "NJRH_LOCALIZATION_FLATSCAN_READY_TIMEOUT_SEC:-75" in occupancy
    assert 'wait_for_topic_publisher_from_node "${LOCALIZER_FLATSCAN_TOPIC}" "laser_scan_to_flatscan"' in occupancy
    assert "nav2_lifecycle_sequence.py" in occupancy
    assert "localization map_server repo lifecycle sequence" in occupancy
    map_lifecycle_block = occupancy[
        occupancy.index("start_map_server_lifecycle_with_nav2_util()") :
        occupancy.index("repair_jt128_navigation_points()")
    ]
    assert "/opt/ros/humble/lib/nav2_util/lifecycle_bringup map_server" not in map_lifecycle_block
    assert "localization will not restart it" in occupancy
    assert "pointcloud_accel_pipeline_localization" not in occupancy
    assert "laser_scan_to_flatscan" in occupancy
    assert "pointcloud accel profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}" in occupancy
    occupancy_cleanup = occupancy[occupancy.index("patterns=(") : occupancy.index("for pattern in \"${patterns[@]}\"")]
    assert '"laser_scan_to_flatscan"' not in occupancy_cleanup

    assert "NJR H_CPUSET_POINTCLOUD_ACCEL" not in cpu_affinity
    assert "NJRH_CPUSET_POINTCLOUD_ACCEL_CONTAINER" in cpu_affinity
    assert "NJRH_CPUSET_POINTCLOUD_ACCEL_LOCAL_WORKER" in cpu_affinity
    assert "NJRH_CPUSET_POINTCLOUD_ACCEL_SCAN_WORKER" in cpu_affinity
    assert "NJRH_CPUSET_NITROS_POINTCLOUD_CONTAINER" in cpu_affinity

    assert "/perception/obstacle_points" not in nav2
    assert "/perception/clearing_points" not in nav2
    assert "observation_sources: scan" in nav2
    assert "data_type: LaserScan" in nav2
    assert "inf_is_valid: true" in nav2
    assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in nav2
    assert 'primary_controller: "robot_nav_config::RangerMPPIController"' in nav2
    assert 'plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"' in nav2
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert 'export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env

    assert "current NJRH_POINTCLOUD_ACCEL_PROFILE" in verify_profile
    assert "ingress_profile" in verify_profile
    assert "/jt128/vendor/points_raw publishers" in verify_profile
    assert "status_accel_ingress_profile" in verify_profile
    assert "status_input_path" in verify_profile
    assert "status_vendor_raw_ros_hop_required" in verify_profile
    assert "status_driver_integrated_process" in verify_profile
    assert "status_accel_core_process_pointcloud2_count" in verify_profile
    assert "status_accel_core_process_decoded_view_count" in verify_profile
    assert "lidar_driver_owner" in verify_profile
    assert "requested_profile" in verify_profile
    assert "resolved_profile" in verify_profile
    assert "/lidar_points publisher count" in verify_profile
    assert "axis publish hz" in verify_profile
    assert "retired local PointCloud2 obstacle output topics are not configured" in verify_profile
    assert "scan_hz" in verify_profile
    assert "flatscan_hz" in verify_profile
    assert "FAST-LIO2 residual" in verify_profile
    assert "local_costmap subscribes" in verify_profile
    assert "PointCloud2 QoS" in verify_profile
    assert "DDS transport env" in verify_profile
    assert "/lidar_points publisher_nodes" in verify_profile
    assert "actual trunk owner" in verify_profile
    assert "actual obstacle owner" not in verify_profile
    assert "actual clearing owner" not in verify_profile
    assert "actual points_nav owner" in verify_profile
    assert "actual scan owner" in verify_profile
    assert "actual flatscan owner" in verify_profile
    assert "trunk owner is pointcloud_accel_axis_node" in verify_profile
    assert "must not publish /perception/obstacle_points" not in verify_profile
    assert "must not publish /perception/clearing_points" not in verify_profile
    assert "local PointCloud2 worker disabled by config" in verify_profile
    assert "still using legacy pointcloud_axis_remap as trunk owner" in verify_profile
    assert "scan owner is accel core process" in verify_profile
    assert "local_worker_enabled must be false" in verify_profile
    assert "scan_worker_enabled must be true" in verify_profile
    assert "internal_zero_copy_profile must be true" in verify_profile
    assert "latest_internal_buffer_points must be present and nonzero" in verify_profile
    assert "worker full PointCloud2 copy counters must stay zero" in verify_profile
    assert "worker intermediate PointCloud2 build counters must stay zero" in verify_profile
    assert "NITROS_ENV_GUARDED_UNAVAILABLE" in verify_profile
    assert "production_hop_internal_local_branch=false" in verify_profile
    assert "production_hop_points_nav=false" in verify_profile
    assert "/points_nav still has a production publisher" in verify_profile
    assert "PROFILE_OWNER_CONTRACT_OK" in verify_profile
    assert "IPC_WORKER_OWNER_OK" in verify_profile
    assert "TRUNK_FULL_DENSITY_OK" in verify_profile
    assert "NAV2_COMPAT_TOPICS_OK" in verify_profile
    assert "INGRESS_PROFILE" in verify_profile

    assert "--profile ipc_worker|nitros" in ab_runner
    assert "legacy profile was removed" in ab_runner
    assert "--ingress-profile separate_process|driver_integrated" in ab_runner
    assert "--duration-sec" in ab_runner
    assert "--apply" in ab_runner
    assert "--restart" in ab_runner
    assert "--restore" in ab_runner
    assert "profile requested" in ab_runner
    assert "profile before run" in ab_runner
    assert "profile actually running" in ab_runner
    assert "ingress profile requested" in ab_runner
    assert "ingress profile actually running" in ab_runner
    assert "/jt128/vendor/points_raw publisher_count" in ab_runner
    assert "Socket Drop Snapshot" in ab_runner
    assert "status_vendor_raw_ros_hop_required" in ab_runner
    assert "binary actually running" in ab_runner
    assert "trunk owner" in ab_runner
    assert "obstacle owner" not in ab_runner
    assert "clearing owner" not in ab_runner
    assert "retired local PointCloud2 obstacle publishers are not part of production" in ab_runner
    assert "points_nav owner" in ab_runner
    assert "scan owner" in ab_runner
    assert "flatscan owner" in ab_runner
    assert "internal_zero_copy_profile" in ab_runner
    assert "latest_internal_buffer_points" in ab_runner
    assert "local_worker_full_cloud_copy_count" in ab_runner
    assert "scan_worker_full_cloud_copy_count" in ab_runner
    assert "/lidar_points subscriber_count" in ab_runner
    assert "/points_nav subscribers" in ab_runner
    assert "/scan subscribers" in ab_runner
    assert "/flatscan subscribers" in ab_runner
    assert "legacy scan chain recovered" not in ab_runner
    assert "ipc_worker no production /points_nav hop" in ab_runner
    assert "PASS/WARN/FAIL" in ab_runner
    assert "restoring prior profile" in ab_runner
    assert "pointcloud_accel_ab_" in ab_runner
    assert "cpu0" in ab_runner
    assert "cpu4" in ab_runner
    assert "cpu5" in ab_runner
    assert "cpu6" in ab_runner
    assert "cpu7" in ab_runner

    assert 'export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=disabled' in (
        config_dir / "local_perception_input_profile.env"
    ).read_text(encoding="utf-8")
    assert "robot_local_perception PointCloud2 obstacle pipeline has been removed" in (
        scripts_dir / "run_local_perception.sh"
    ).read_text(encoding="utf-8")

    assert "ComposableNodeContainer" not in accel_launch
    assert "pointcloud_accel_axis_node" in accel_launch
    assert "laser_scan_to_flatscan" in accel_launch
    assert "ROBOT_ISAAC_NITROS_POINTCLOUD_ENABLE" in nitros_cmake
    assert "OFF" in nitros_cmake
    assert "isaac_ros_nitros" in nitros_cmake
    assert "<depend>isaac_ros_nitros</depend>" not in nitros_package
    nitros_launch = (
        ROOT / "src" / "robot_isaac_nitros_pointcloud" / "launch" / "nitros_pointcloud_branch.launch.py"
    ).read_text(encoding="utf-8")
    assert "same-process component container" in nitros_launch
    assert "PointCloud2" not in accel_launch
    assert "sensor_msgs.msg" not in accel_launch
    assert "create_subscription" not in accel_launch
    assert "create_publisher" not in accel_launch


def test_phase_d1_pointcloud_driver_integrated_ingress_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"
    config_dir = overlay / "config"
    hesai_dir = ROOT / "src" / "robot_hesai_jt128"
    hesai_overlay_dir = ROOT / "src" / "third_party" / "hesai_lidar_ros2_overlay"

    ingress_env_path = config_dir / "pointcloud_ingress_profile.env"
    accel_core_header_path = hesai_dir / "include" / "robot_hesai_jt128" / "pointcloud_accel_core.hpp"
    accel_core_cpp_path = hesai_dir / "src" / "pointcloud_accel_core.cpp"
    hesai_accel_driver_path = hesai_dir / "src" / "hesai_accel_driver_node.cpp"
    hesai_overlay_source_driver_path = hesai_overlay_dir / "src" / "manager" / "source_driver_ros2.hpp"

    assert ingress_env_path.exists()
    assert accel_core_header_path.exists()
    assert accel_core_cpp_path.exists()
    assert hesai_accel_driver_path.exists()
    assert hesai_overlay_source_driver_path.exists()

    ingress_env = ingress_env_path.read_text(encoding="utf-8")
    accel_cfg = (config_dir / "pointcloud_accel_axis.yaml").read_text(encoding="utf-8")
    hesai_accel_cfg = (config_dir / "hesai_accel_driver.yaml").read_text(encoding="utf-8")
    accel_profile = (scripts_dir / "pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    run_driver = (scripts_dir / "run_driver.sh").read_text(encoding="utf-8")
    run_pipeline = (scripts_dir / "run_pointcloud_accel_pipeline.sh").read_text(encoding="utf-8")
    verify_profile = (scripts_dir / "verify_pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    ab_runner = (scripts_dir / "run_pointcloud_accel_ab.sh").read_text(encoding="utf-8")
    set_profile = (scripts_dir / "set_pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    hesai_cmake = (hesai_dir / "CMakeLists.txt").read_text(encoding="utf-8")
    hesai_overlay_cmake = (hesai_overlay_dir / "CMakeLists.txt").read_text(encoding="utf-8")
    hesai_overlay_package = (hesai_overlay_dir / "package.xml").read_text(encoding="utf-8")
    hesai_overlay_source_driver = hesai_overlay_source_driver_path.read_text(encoding="utf-8")
    accel_axis_wrapper = (hesai_dir / "src" / "pointcloud_accel_axis_node.cpp").read_text(encoding="utf-8")
    accel_core = accel_core_cpp_path.read_text(encoding="utf-8")
    hesai_accel_driver = hesai_accel_driver_path.read_text(encoding="utf-8")

    assert 'NJR H_POINTCLOUD_INGRESS_PROFILE' not in ingress_env
    assert 'export NJRH_POINTCLOUD_INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}"' in ingress_env
    assert "driver_integrated_available: true" in hesai_accel_cfg
    assert "hesai_accel_driver_node src/hesai_accel_driver_node.cpp" in hesai_cmake
    assert "pointcloud_accel_core src/pointcloud_accel_core.cpp" in hesai_cmake
    assert "ament_export_libraries(pointcloud_accel_core)" in hesai_cmake
    assert "ament_export_targets(export_pointcloud_accel_core HAS_LIBRARY_TARGET)" in hesai_cmake
    assert "hesai_accel_driver_node" in hesai_overlay_cmake
    assert "HESAI_ACCEL_DRIVER_INTEGRATED" in hesai_overlay_cmake
    assert "find_package(robot_hesai_jt128 REQUIRED)" in hesai_overlay_cmake
    assert "ROBOT_HESAI_JT128_POINTCLOUD_ACCEL_CORE_LIBRARY" in hesai_overlay_cmake
    assert 'target_link_libraries(hesai_accel_driver_node' in hesai_overlay_cmake
    assert "<depend condition=\"$ROS_VERSION == 2\">robot_hesai_jt128</depend>" in hesai_overlay_package
    assert "PointCloudAccelCore" in hesai_overlay_source_driver
    assert "driver_callback_pointcloud2" in hesai_overlay_source_driver
    assert "vendor_raw_ros_hop_required = false" in hesai_overlay_source_driver
    assert "accel_core_->process_pointcloud2(std::move(cloud))" in hesai_overlay_source_driver

    assert "PointCloudAccelCore" in accel_axis_wrapper
    assert "create_subscription<sensor_msgs::msg::PointCloud2>" in accel_axis_wrapper
    assert "core_->process_pointcloud2" in accel_axis_wrapper
    assert "struct LatestNormalizedBuffer" not in accel_axis_wrapper
    assert "trunk_publisher_->publish(*output)" not in accel_axis_wrapper
    assert "struct LatestNormalizedBuffer" in accel_core
    assert "trunk_publisher_->publish(*output)" in accel_core
    assert "process_decoded_points" in accel_core
    assert "accel_ingress_profile" in accel_core
    assert "input_path" in accel_core
    assert "vendor_raw_ros_hop_required" in accel_core
    assert "driver_integrated_process" in accel_core
    assert "accel_core_process_pointcloud2_count" in accel_core
    assert "accel_core_process_decoded_view_count" in accel_core
    assert 'declare_parameter<bool>("clearing_worker_virtual_rays_enabled", true)' in accel_core
    assert 'declare_parameter<double>("clearing_worker_virtual_ray_angle_resolution_deg", 1.0)' in accel_core
    assert 'declare_parameter<double>("clearing_worker_virtual_ray_min_angle_deg", -110.0)' in accel_core
    assert 'declare_parameter<double>("clearing_worker_virtual_ray_max_angle_deg", 110.0)' in accel_core
    assert 'declare_parameter<bool>("clearing_worker_virtual_rays_allow_self_mask_endpoints", true)' in accel_core
    assert 'declare_parameter<std::string>("local_worker_stamp_source", "")' in accel_core
    assert "local_worker_stamp_source_ == \"local_odom\"" in accel_core
    assert "LocalWorkerHeaderStampSource::LocalOdom" in accel_core
    assert "current_local_worker_header_stamp_source_label()" in accel_core
    assert "!clearing_worker_virtual_rays_allow_self_mask_endpoints_ && in_self_mask(endpoint)" in accel_core
    assert "struct ClearingRayBin" in accel_core
    assert "build_virtual_clearing_points()" in accel_core
    assert "update_clearing_virtual_ray_bin(point)" in accel_core

    for script in (run_driver, run_pipeline, verify_profile, ab_runner, set_profile):
        assert "NJRH_POINTCLOUD_INGRESS_PROFILE" in script

    assert "driver_integrated" in accel_profile
    assert "NJR H_POINTCLOUD_ACCEL_PROFILE" not in accel_profile
    assert "NJR H_POINTCLOUD_INGRESS_PROFILE" not in accel_profile
    assert "NJR H" not in accel_profile
    assert "NJR H" not in run_pipeline
    assert "NJR H" not in verify_profile
    assert "NJR H" not in ab_runner
    assert "NJR H" not in set_profile
    assert "NJRH_POINTCLOUD_ACCEL_PROFILE=legacy has been removed from production" in accel_profile
    assert "NJRH_POINTCLOUD_INGRESS_PROFILE=driver_integrated" in run_pipeline
    assert "driver_integrated ingress selected" in run_pipeline
    assert "HESAI_ACCEL_DRIVER_CPP_BIN" in run_driver
    assert "DEFAULT_HESAI_CONFIG_FILE" in run_driver
    assert "REPO_HESAI_CONFIG_FILE" in run_driver
    assert 'src/third_party/hesai_lidar_ros2_overlay/config/config.yaml' in run_driver
    assert run_driver.index('if [[ "${NJRH_POINTCLOUD_INGRESS_PROFILE}" == "driver_integrated" ]]') < run_driver.index('[[ -f "${POINTCLOUD_REMAP_CONFIG}" ]]')
    assert "hesai_accel_driver_node" in run_driver
    assert "hesai_ros_driver_node" in run_driver
    assert 'install/hesai_ros_driver/lib/hesai_ros_driver/hesai_accel_driver_node' in run_driver
    assert '-p "config_path:=${RUNTIME_CONFIG_FILE}"' in run_driver
    assert "starting canonical imu remap for driver_integrated ingress" in run_driver
    assert "both hesai_ros_driver_node and hesai_accel_driver_node" not in run_driver
    assert "driver_integrated must not run standalone hesai_ros_driver_node" in verify_profile
    assert "driver_integrated must not run standalone pointcloud_accel_axis_node" in verify_profile
    assert "driver_integrated must not require /jt128/vendor/points_raw subscribers" in verify_profile
    assert "vendor_raw_ros_hop_required=false" in verify_profile

    assert "/lidar_points is always full-density/full-fields" in accel_cfg
    assert "output_topic: /lidar_points" in accel_cfg
    assert "output_reliable: false" in accel_cfg
    assert "local_compact_stride: 4" in accel_cfg
    assert "nav_compact_stride: 4" in accel_cfg
    assert "lidar_points_full_density_full_fields: true" in hesai_accel_cfg
    assert "hesai_driver_config_path: /workspaces/njrh-v3/workspace1/src/third_party/hesai_lidar_ros2_overlay/config/config.yaml" in hesai_accel_cfg
    assert "publish_vendor_raw_debug: false" in hesai_accel_cfg
    assert "publish_vendor_imu_raw_debug: true" in hesai_accel_cfg
    assert "vendor_raw_ros_hop_required: false" in hesai_accel_cfg
    for cfg in (accel_cfg, hesai_accel_cfg):
        assert "local_worker_stamp_source: local_odom" in cfg
        assert "local_worker_stamp_odom_topic: /local_state/odometry" in cfg
        assert "local_worker_stamp_max_odom_age_sec: 0.25" in cfg
        assert "clearing_worker_virtual_rays_enabled: true" in cfg
        assert "clearing_worker_virtual_ray_angle_resolution_deg: 1.0" in cfg
        assert "clearing_worker_virtual_ray_min_angle_deg: -110.0" in cfg
        assert "clearing_worker_virtual_ray_max_angle_deg: 110.0" in cfg
        assert "clearing_worker_virtual_rays_allow_self_mask_endpoints: true" in cfg
        assert "clearing_worker_virtual_ray_range: 8.00" in cfg
        assert "clearing_worker_max_points: 30000" in cfg
        assert (
            "clearing_worker_virtual_ray_range_steps: "
            "[0.10, 0.15, 0.20, 0.35, 0.50, 0.75, 1.00, 1.50, 2.50, 4.00, 6.00, 8.00]"
        ) in cfg
        assert (
            "clearing_worker_virtual_ray_endpoint_z_values: "
            "[-0.10, 0.05, 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.40]"
        ) in cfg

    status_topics = {
        "/lidar/axis_remap_status",
        "/lidar/pointcloud_accel_status",
        "/perception/local_perception_status",
    }
    allowed_topics = {
        "/jt128/vendor/points_raw",
        "/jt128/vendor/imu_raw",
        "/lidar_points",
        "/_internal/lidar_points_local",
        "/lidar_points_nav",
        "/points_nav",
        "/perception/obstacle_points",
            "/perception/clearing_points",
            "/scan",
            "/scan-based",
            "/flatscan",
        }
    topic_prefixes = ("/jt128/", "/lidar", "/_internal/", "/points", "/perception/", "/scan", "/flatscan")
    for text in (accel_cfg, hesai_accel_cfg, accel_core, hesai_overlay_source_driver, verify_profile, ab_runner):
        for topic in re.findall(r"/[A-Za-z0-9_][A-Za-z0-9_./-]*", text):
            if not topic.startswith(topic_prefixes):
                continue
            if (
                topic.endswith(".sh")
                or topic.endswith(".info")
                or topic.endswith(".env")
                or topic.endswith(".")
                or topic.endswith("/")
            ):
                continue
            if topic in status_topics:
                continue
            assert topic in allowed_topics, topic

    assert "pkill -9" not in run_driver
    assert "killall -9" not in run_driver
    assert "pkill -9" not in run_pipeline
    assert "killall -9" not in run_pipeline
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert 'export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env
    assert 'export NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE:-ipc_worker}"' in (
        config_dir / "pointcloud_accel_profile.env"
    ).read_text(encoding="utf-8")
    assert "global_frame: odom" in local_costmap_config_block(nav2)
    assert "global_frame: base_link" not in local_costmap_config_block(nav2)
    assert "MPPIController" in nav2
    assert "SmacPlanner2D" in nav2


def test_phase115_flatscan_lifecycle_hardening_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"

    run_pipeline = (scripts_dir / "run_pointcloud_accel_pipeline.sh").read_text(encoding="utf-8")
    common_services = (scripts_dir / "run_common_services.sh").read_text(encoding="utf-8")
    commercial_ready = (scripts_dir / "check_commercial_runtime_ready.sh").read_text(encoding="utf-8")
    verify_profile = (scripts_dir / "verify_pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    nav_runtime = (scripts_dir / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    ab_runner = (scripts_dir / "run_pointcloud_accel_ab.sh").read_text(encoding="utf-8")
    nav2 = (overlay / "config" / "nav2.yaml").read_text(encoding="utf-8")

    combined = run_pipeline + verify_profile + nav_runtime + ab_runner
    for topic in ("/lidar_points", "/scan", "/flatscan"):
        assert topic in combined
    assert "/flatscan_debug" not in combined
    assert "/flatscan_status" not in combined

    assert "start_flatscan_helper()" in run_pipeline
    assert "flatscan_helper_running()" in run_pipeline
    assert "restart_flatscan_helper_if_allowed()" in run_pipeline
    assert "wait_for_scan_ready()" in run_pipeline
    assert "helper_bin=\"${helper_prefix}/lib/jt128_nav_tools/laser_scan_to_flatscan\"" in run_pipeline
    assert "ros2 run jt128_nav_tools laser_scan_to_flatscan" not in run_pipeline
    assert "wait_for_flatscan_ready()" in run_pipeline
    assert "supervise_flatscan_helper()" in run_pipeline
    assert "stop_flatscan_helper()" in run_pipeline
    assert "NJRH_FLATSCAN_HELPER_REQUIRED" in run_pipeline
    assert "NJRH_FLATSCAN_HELPER_RESTART" in run_pipeline
    assert "NJRH_FLATSCAN_HELPER_MAX_RESTARTS" in run_pipeline
    assert "NJRH_FLATSCAN_HELPER_RESTART_BACKOFF_SEC" in run_pipeline
    assert "NJRH_FLATSCAN_HELPER_MISSING_CONFIRMATIONS" in run_pipeline
    assert "NJRH_FLATSCAN_HELPER_RESTART_COOLDOWN_SEC" in run_pipeline
    assert "NJRH_FLATSCAN_HELPER_HEALTHY_RESET_SEC" in run_pipeline
    assert "NJRH_FLATSCAN_GRAPH_PROBE_TIMEOUT_SEC" in run_pipeline
    assert 'FLATSCAN_WAIT_SEC="${NJRH_FLATSCAN_WAIT_SEC:-30}"' in run_pipeline
    assert "NJRH_FLATSCAN_MIN_HZ" in run_pipeline
    assert "FLATSCAN_HELPER_RESTART_COUNT" in run_pipeline
    assert "FLATSCAN_HELPER_GRAPH_MISS_COUNT" in run_pipeline
    assert "FLATSCAN_HELPER_HEALTH_STATE" in run_pipeline
    assert "FLATSCAN_HELPER_RESTART_COOLDOWN_UNTIL_EPOCH" in run_pipeline
    assert "CASE_FLATSCAN_HELPER_DEAD" in run_pipeline
    assert "confirm_flatscan_stream_after_graph_misses()" in run_pipeline
    assert "graph misses confirmed but /flatscan messages are flowing; keeping helper" in run_pipeline
    assert "restart budget exhausted; keeping supervisor alive" in run_pipeline
    assert "stable health reset restart budget" in run_pipeline
    assert "independently confirmed no /flatscan flow while /scan publisher exists and helper pid=" in run_pipeline
    assert 'flatscan_helper_health_state="process_missing"' in run_pipeline
    assert 'restart_flatscan_helper_if_allowed "laser_scan_to_flatscan exited"' in run_pipeline
    assert "FlatScan fault candidate unconfirmed upstream; keeping helper alive" in run_pipeline
    assert "FAIL standalone scan chain lost /flatscan and /scan publisher is not ready" not in run_pipeline
    assert "legacy_scan_pid=$!" not in run_pipeline
    assert "jt128_localization_sensing.launch.py" not in run_pipeline
    assert 'flatscan_helper_mode="legacy_launch"' not in run_pipeline
    assert 'flatscan_helper_mode="standalone"' in run_pipeline
    assert "start_flatscan_helper" in run_pipeline[run_pipeline.index("ipc_worker)") :]
    legacy_block = run_pipeline[run_pipeline.index("legacy)") : run_pipeline.index("ipc_worker)")]
    assert "FAIL legacy profile removed" in legacy_block
    assert "jt128_localization_sensing.launch.py" not in legacy_block
    assert "start_flatscan_helper" not in legacy_block
    assert run_pipeline.rstrip().endswith("supervise_flatscan_helper")
    assert "pkill -9" not in run_pipeline
    assert "killall -9" not in run_pipeline

    assert "MIN_FLATSCAN_HZ" in verify_profile
    assert "FLATSCAN_OWNER_OK" in verify_profile
    assert "FLATSCAN_HZ_OK" in verify_profile
    assert "FLATSCAN_NAV_STARTUP_GATE_OK" in verify_profile
    assert "CASE_FLATSCAN_HELPER_DEAD" in verify_profile
    assert "flatscan_helper_pid" in verify_profile
    assert "flatscan_helper_restart_count" in verify_profile
    assert "flatscan owner is pointcloud_accel_axis_node direct publisher" in verify_profile

    assert "GLOBAL_LOCALIZATION_TRIGGER_SERVICE_MISSING" in nav_runtime
    assert "GRID_SEARCH_LOCALIZATION_SERVICE_MISSING" in nav_runtime
    assert "MAP_TOPIC_MISSING" in nav_runtime
    assert "FLATSCAN_MISSING" in nav_runtime
    assert "LOCALIZATION_RESULT_PUBLISHER_MISSING" in nav_runtime
    assert "scan_flatscan_admission_diagnostics" in nav_runtime
    assert "recover_flatscan_helper_for_navigation" in nav_runtime
    assert "start_flatscan_helper_for_navigation_repair" in nav_runtime
    assert "wait_for_flatscan_publisher_ready" in nav_runtime
    assert 'wait_for_topic_publisher_from_node "/flatscan" "laser_scan_to_flatscan"' in nav_runtime
    assert 'wait_for_topic_message "/flatscan"' not in nav_runtime
    assert "current_pointcloud_accel_profile" in nav_runtime
    assert "without restarting pointcloud accel profile" in nav_runtime
    assert "stopping stale laser_scan_to_flatscan process without touching pointcloud driver" in nav_runtime
    assert "NJRH_FLATSCAN_REPAIR_RESTART_POINTCLOUD:-false" in nav_runtime
    assert "set_pointcloud_accel_profile.sh\" --profile \"${profile}\" --restart" in nav_runtime
    assert "/flatscan repair fallback: restarting pointcloud accel profile=${profile}" in nav_runtime
    assert "/flatscan repair succeeded" in nav_runtime
    assert "laser_scan_to_flatscan_process" in nav_runtime
    assert "pointcloud_accel_profile" in nav_runtime
    assert "suggested_fix" in nav_runtime
    assert "pointcloud_accel_pipeline_aux_unique()" in common_services
    assert "pointcloud_accel_pipeline_aux_complete()" in common_services
    assert "stop_stale_pointcloud_accel_pipeline_processes()" in common_services
    assert 'process_count_for_pattern "[r]un_pointcloud_accel_pipeline.sh"' in common_services
    assert 'process_count_for_pattern "[l]aser_scan_to_flatscan"' in common_services
    assert "stopping stale pointcloud accel pipeline before restart" in common_services
    assert "stop_stale_pointcloud_accel_pipeline_processes" in common_services[
        common_services.index('if reuse_common_services_enabled && canonical_jt128_runtime_complete; then') :
        common_services.index('start_common_process "pointcloud_accel_pipeline"')
    ]

    assert "flatscan helper pid" in ab_runner
    assert "flatscan helper restart count" in ab_runner
    assert "flatscan case" in ab_runner
    assert "CASE_FLATSCAN_HELPER_DEAD" in ab_runner
    assert "check_topic_publisher_node()" in commercial_ready
    assert 'run_check check_topic_publisher_node "/flatscan" "laser_scan_to_flatscan" 10' in commercial_ready

    assert "global_frame: odom" in nav2
    assert "global_frame: base_link" not in nav2


def test_phase_a2_amcl_replaces_isaac_continuous_localization_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"
    bridge_dir = ROOT / "src" / "robot_localization_bridge"
    global_dir = ROOT / "src" / "robot_global_localization"

    accel_cfg = (config_dir / "pointcloud_accel_axis.yaml").read_text(encoding="utf-8")
    hesai_accel_cfg = (config_dir / "hesai_accel_driver.yaml").read_text(encoding="utf-8")
    mode_env = (config_dir / "isaac_localization_mode.env").read_text(encoding="utf-8")
    amcl_mode = (config_dir / "amcl_localization_profile.env").read_text(encoding="utf-8")
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    run_localizer = (scripts_dir / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    run_bridge = (scripts_dir / "run_localization_bridge.sh").read_text(encoding="utf-8")
    nav_runtime = (scripts_dir / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    commercial_helpers = (scripts_dir / "commercial_runtime_helpers.sh").read_text(encoding="utf-8")
    nav_helpers = (scripts_dir / "nav_runtime_helpers.sh").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    bridge_src_cfg = (bridge_dir / "config" / "localization_bridge.yaml").read_text(encoding="utf-8")
    bridge_cpp = (bridge_dir / "src" / "localization_bridge_node.cpp").read_text(encoding="utf-8")
    global_cfg = (global_dir / "config" / "global_localization.yaml").read_text(encoding="utf-8")
    global_cpp = (global_dir / "src" / "global_localization_node.cpp").read_text(encoding="utf-8")
    global_cmake = (global_dir / "CMakeLists.txt").read_text(encoding="utf-8")
    global_pkg = (global_dir / "package.xml").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")

    assert "scan_worker_rate_hz: 15.0" in accel_cfg
    assert "scan_worker_rate_hz: 15.0" in hesai_accel_cfg
    assert "output_topic: /lidar_points" in accel_cfg
    assert "local_worker_enabled: false" in accel_cfg
    assert "input_reliable: false" in accel_cfg
    assert "output_reliable: false" in accel_cfg
    assert "rmw_cyclonedds_cpp" not in common_env
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env

    assert "NJRH_ISAAC_LOCALIZATION_MODE:-triggered" in mode_env
    assert "NJRH_ISAAC_CONTINUOUS" not in mode_env
    assert "NJRH_AMCL_LOCALIZATION_MODE:-gated" in amcl_mode
    assert "NJRH_ISAAC_LOCALIZATION_MODE_FILE" in common_env
    assert "isaac_localization_mode.env" in common_env
    assert "amcl_localization_profile.env" in common_env
    assert "continuous_localization_mode: triggered" in bridge_cfg
    assert "continuous_localization_mode: triggered" in bridge_src_cfg
    assert "triggered_max_result_age_ms: 5000.0" in bridge_cfg
    assert "triggered_max_result_age_ms: 5000.0" in bridge_src_cfg
    assert "force_accept_min_pose_stamp_slack_sec: 1.0" in bridge_cfg
    assert "force_accept_min_pose_stamp_slack_sec: 1.0" in bridge_src_cfg
    assert "continuous_max_result_age_ms:" not in bridge_cfg
    assert "continuous_max_result_age_ms:" not in bridge_src_cfg
    assert "triggered_allow_large_correction: true" in bridge_cfg
    assert "continuous_allow_large_correction:" not in bridge_cfg

    assert not (scripts_dir / "continuous_flatscan_forwarder.py").exists()
    assert not (scripts_dir / "verify_isaac_continuous_localization.sh").exists()
    assert not (ROOT / "docs" / "phase_l1_isaac_continuous_localization.md").exists()
    assert "python3 \"${SCRIPT_DIR}/continuous_flatscan_forwarder.py\"" not in run_localizer
    assert "starting Isaac continuous localization forwarder" not in run_localizer
    assert "LOCALIZER_FLATSCAN_TOPIC" in run_localizer
    assert "NJRH_ISAAC_LOCALIZER_FLATSCAN_TOPIC:-/flatscan" in run_localizer
    assert "NJRH_ISAAC_CONTINUOUS_LOCALIZATION_INPUT_TOPIC" not in run_localizer
    assert "flatscan_topic:=${LOCALIZER_FLATSCAN_TOPIC}" in run_localizer
    assert "localizer_flatscan_topic=${LOCALIZER_FLATSCAN_TOPIC}" in run_localizer
    assert "expected triggered." in run_localizer
    assert 'DeclareLaunchArgument("flatscan_topic", default_value="/flatscan")' in (
        overlay / "launch" / "occupancy_localization.launch.py"
    ).read_text(encoding="utf-8")
    assert '("flatscan", flatscan_topic)' in (overlay / "launch" / "occupancy_localization.launch.py").read_text(
        encoding="utf-8"
    )
    assert "continuous_flatscan_forwarder.py" in nav_helpers
    assert 'continuous_localization_mode:=triggered' in run_bridge
    assert "AMCL_INPUT_ENABLED=\"true\"" in run_bridge
    assert "amcl_input_enabled:=${AMCL_INPUT_ENABLED}" in run_bridge
    assert "amcl_scan_admission_enabled:=${AMCL_SCAN_ADMISSION_ENABLED}" in run_bridge
    assert "amcl_scan_admission_status_topic:=${NJRH_AMCL_SCAN_ADMISSION_STATUS_TOPIC" in run_bridge

    assert "lookupTransform(odom_frame_, base_frame_, stamp" in bridge_cpp
    assert "lookupTransform(odom_frame_, base_frame_, latest_tf_time" in bridge_cpp
    assert "odom_base_latest_tf_stale_ms" in bridge_cpp
    assert "solve_correction(" in bridge_cpp
    assert "candidate.transform = solution.target_map_odom" in bridge_cpp
    assert "solution.base_translation_m" in bridge_cpp
    assert "continuous_localization_mode_" in bridge_cpp
    assert "triggered_max_result_age_ms_" in bridge_cpp
    assert "force_accept_min_pose_stamp_slack_sec_" in bridge_cpp
    assert "force_accept_armed_sec_" in bridge_cpp
    assert "should_ignore_force_accept_pretrigger_result" in bridge_cpp
    assert "force_accept_ignored_pretrigger_result_count" in bridge_cpp
    assert "last_force_accept_ignored_reason" in bridge_cpp
    assert "mark_latest_pose_stamp_used" in bridge_cpp
    assert "candidate_should_retry_later" in bridge_cpp
    assert "EXPLICIT_TRIGGERED_RELOCALIZATION" in bridge_cpp
    assert "force_accept_next_pose_" in bridge_cpp
    assert "map_to_odom_publisher_owner" in bridge_cpp
    assert "robot_localization_bridge" in bridge_cpp
    assert "isaac_background_correction_removed" in bridge_cpp
    assert "continuous_shadow" not in bridge_cpp
    assert "continuous_gated" not in bridge_cpp
    assert "isaac_continuous" not in bridge_cpp
    assert "gate_mode" in bridge_cpp
    assert "last_result_age_ms" in bridge_cpp
    assert "gate_result_age_limit_ms" in bridge_cpp
    assert "last_accept_reason" in bridge_cpp
    assert "last_result_used_original_stamp" in bridge_cpp
    assert "last_odom_tf_history_lookup_ok" in bridge_cpp
    assert "latest_odom_tf_age_ms" in bridge_cpp
    assert "triggered_result_count" in bridge_cpp
    assert "rejected_result_count" in bridge_cpp
    assert "last_reject_reason" in bridge_cpp
    assert "accepted_result_hz" in bridge_cpp
    assert "on_amcl_pose" in bridge_cpp
    assert "accept_amcl_candidate" in bridge_cpp
    assert "amcl_accept_corrections_while_moving" in bridge_cpp
    assert "AMCL_ROBOT_MOVING_OBSERVE_ONLY" in bridge_cpp

    assert "service_call_timeout_sec: 10.0" in global_cfg
    assert "result_wait_timeout_sec: 20.0" in global_cfg
    assert "bridge_accept_timeout_sec: 12.0" in global_cfg
    assert "transient_stale_bridge_accept_timeout_sec" not in global_cfg
    assert "localizer_input_freshness_enabled: true" in global_cfg
    assert "localizer_input_topic: /flatscan" in global_cfg
    assert "localizer_input_wait_timeout_sec: 1.0" in global_cfg
    assert "localizer_input_max_age_sec: 0.5" in global_cfg
    assert "localizer_input_min_fov_deg: 115.0" in global_cfg
    assert "result_allowed_pretrigger_age_sec: 1.0" in global_cfg
    assert "active_deadline = shortened_deadline" not in global_cpp
    assert "draining pre-arm localization_result inside the same trigger transaction" in global_cpp
    assert "force_accept_ignored_pretrigger_result_count" in global_cpp
    assert "last_force_accept_ignored_reason" in global_cpp
    assert "draining pre-arm localization_result inside the same trigger transaction" in global_cpp
    assert "create_subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>" in global_cpp
    assert "localizer_input_fov_deg" in global_cpp
    assert "localizer_input_min_fov_deg_" in global_cpp
    assert "wait_for_fresh_localizer_input" in global_cpp
    assert "failure_code=LOCALIZER_INPUT_NOT_FRESH" in global_cpp
    assert "localization_result_is_fresh_for_trigger" in global_cpp
    assert "force-accept arm time and deadline remain unchanged" in global_cpp
    assert "map_to_odom_wait_timeout_sec: 8.0" in global_cfg
    assert "bridge_status_topic: /localization/bridge_status" in global_cfg
    assert "bridge_force_accept_service: /robot_localization_bridge/force_accept_next_localization" in global_cfg
    assert "bridge_status_sub_" in global_cpp
    assert "localization_result_sub_" in global_cpp
    assert "find_package(tf2_ros REQUIRED)" in global_cmake
    assert "find_package(isaac_ros_pointcloud_interfaces REQUIRED)" in global_cmake
    assert "tf2_ros" in global_cmake
    assert "<depend>tf2_ros</depend>" in global_pkg
    assert "<depend>isaac_ros_pointcloud_interfaces</depend>" in global_pkg
    assert "wait_for_bridge_acceptance" in global_cpp
    assert "bridge_reject_is_transient_triggered_stale" in global_cpp
    assert "bridge_reject_is_expected_amcl_observe_only" in global_cpp
    assert "AMCL_ROBOT_MOVING_OBSERVE_ONLY" in global_cpp
    assert "amcl_suppressed_after_isaac_triggered" in global_cpp
    assert "AMCL_CORRECTION_TOO_LARGE" in global_cpp
    assert "bridge_explicit_trigger_accept_observed" in global_cpp
    assert "last_explicit_relocalization_sequence" in global_cpp
    assert "bridge accepted explicit triggered relocalization" in global_cpp
    assert global_cpp.index("bridge_explicit_trigger_accept_observed(initial, latest)") < global_cpp.index(
        "latest.rejected_result_count > initial.rejected_result_count"
    )
    assert "bridge ignoring AMCL observe-only reject while waiting for fresh" in global_cpp
    assert "isaac_triggered_pose_stale_ms" in global_cpp
    assert "wait_for_map_to_odom" in global_cpp
    assert "failure_code=ISAAC_SERVICE_UNAVAILABLE" in global_cpp
    assert "failure_code=ISAAC_DISPATCH_TIMEOUT" in global_cpp
    assert "failure_code=LOCALIZATION_RESULT_TIMEOUT" in global_cpp
    assert "failure_code=FRESH_LOCALIZATION_RETRY_REQUIRED" not in global_cpp
    assert "failure_code=BRIDGE_ACCEPT_TIMEOUT" in global_cpp
    assert "failure_code=MAP_TO_ODOM_TIMEOUT" in global_cpp
    assert "failure_code=MAP_TO_ODOM_WRONG_OWNER" in global_cpp
    assert "failure_code=TF_HISTORY_MISSING" in global_cpp
    assert "restamp" not in global_cpp.lower()

    assert "wait_for_bridge_has_map_to_odom" in nav_runtime
    assert "start_amcl_resident_if_enabled_for_navigation" in nav_runtime
    assert "complete_amcl_readiness_if_enabled_for_navigation" in nav_runtime
    assert "amcl_resident_runtime_status_ready_for_seed()" in nav_runtime
    assert "AMCL resident already warm from status file" in nav_runtime
    assert 'wait_for_fresh_header_topic_message \\\n    "/local_state/odometry"' in nav_runtime
    assert 'wait_for_fresh_header_topic_message \\\n    "/lidar_points"' not in nav_runtime
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-90" in nav_runtime
    assert "GLOBAL_LOCALIZATION_TRIGGER_TIMEOUT" in nav_runtime
    assert "LOCALIZATION_RESULT_TIMEOUT" in nav_runtime
    assert "trigger_output_reports_transient_amcl_pose_stale_reject" not in nav_runtime
    assert "AMCL_POSE_STALE" in nav_runtime
    assert "MAP_TO_ODOM_TIMEOUT" in nav_runtime
    assert "NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK" in nav_runtime
    assert "NJRH_RUNTIME_MAP_TO_ODOM_AGE_MS" in nav_runtime
    assert "failure_code" in commercial_helpers

    assert "MPPIController" in nav2
    assert "SmacPlanner2D" in nav2
    assert "global_frame: odom" in local_costmap_config_block(nav2)
    assert "global_frame: base_link" not in local_costmap_config_block(nav2)
    assert "FAST-LIO2" not in run_bridge


def test_phase_a1_amcl_shadow_localization_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"
    bridge_dir = ROOT / "src" / "robot_localization_bridge"

    amcl_cfg_path = config_dir / "amcl_shadow.yaml"
    amcl_mode_path = config_dir / "amcl_localization_profile.env"
    amcl_runner_path = scripts_dir / "run_amcl_shadow_localization.sh"
    amcl_verify_path = scripts_dir / "verify_amcl_shadow_localization.sh"
    amcl_scan_relay_path = scripts_dir / "amcl_scan_admission_relay.py"
    assert amcl_cfg_path.exists()
    assert amcl_mode_path.exists()
    assert amcl_runner_path.exists()
    assert amcl_verify_path.exists()
    assert amcl_scan_relay_path.exists()

    amcl_cfg = amcl_cfg_path.read_text(encoding="utf-8")
    amcl_mode = amcl_mode_path.read_text(encoding="utf-8")
    amcl_runner = amcl_runner_path.read_text(encoding="utf-8")
    amcl_verify = amcl_verify_path.read_text(encoding="utf-8")
    amcl_scan_relay = amcl_scan_relay_path.read_text(encoding="utf-8")
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    run_bridge = (scripts_dir / "run_localization_bridge.sh").read_text(encoding="utf-8")
    nav_runtime = (scripts_dir / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    stop_nav = (scripts_dir / "stop_floor_navigation.sh").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    bridge_src_cfg = (bridge_dir / "config" / "localization_bridge.yaml").read_text(encoding="utf-8")
    bridge_cpp = (bridge_dir / "src" / "localization_bridge_node.cpp").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")

    assert "tf_broadcast: false" in amcl_cfg
    assert "scan_topic: /scan_amcl" in amcl_cfg
    assert "scan_topic: /flatscan" not in amcl_cfg
    assert "map_topic: /map" in amcl_cfg
    assert "set_initial_pose: false" in amcl_cfg
    assert "nav2_amcl::OmniMotionModel" in amcl_cfg

    assert "NJRH_AMCL_LOCALIZATION_MODE:-gated" in amcl_mode
    assert "NJRH_AMCL_TF_WARMUP_SEC" in amcl_mode
    assert "NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC" in amcl_mode
    assert "NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC:-8" in amcl_mode
    assert "NJRH_AMCL_LIFECYCLE_POST_TRANSITION_STATE_WAIT_SEC:-2" in amcl_mode
    assert "NJRH_AMCL_PARAM_READY_TIMEOUT_SEC:-5" in amcl_mode
    assert "NJRH_AMCL_SEED_RETRY_COUNT" in amcl_mode
    assert "NJRH_AMCL_SCAN_ADMISSION_ENABLED" in amcl_mode
    assert "NJRH_AMCL_SCAN_OUTPUT_TOPIC" in amcl_mode
    assert "NJRH_AMCL_SCAN_MAX_AGE_MS:-1000.0" in amcl_mode
    assert "NJRH_AMCL_SCAN_WAIT_FOR_TF_TIMEOUT_MS:-20.0" in amcl_mode
    assert "NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_FRESH_WAIT:-true" in amcl_runner
    assert "NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_ADMISSION_READY_WAIT:-true" in amcl_runner
    assert "AMCL static standby fast seed" in amcl_runner
    assert "status readiness remains asynchronous" in amcl_runner
    readiness_start = amcl_runner.index("complete_amcl_readiness_sequence()")
    readiness_end = amcl_runner.index("start_amcl_node()", readiness_start)
    readiness_sequence = amcl_runner[readiness_start:readiness_end]
    assert readiness_sequence.index("AMCL static standby fast seed") < readiness_sequence.index(
        "wait_for_scan_admission_status_ready"
    )
    assert "disabled|shadow|gated" in amcl_runner
    assert "AMCL_BIN=" in amcl_runner
    assert '"${AMCL_BIN}" --ros-args' in amcl_runner
    assert "ros2 run nav2_amcl amcl --ros-args" not in amcl_runner
    assert "complete_amcl_readiness_sequence" in amcl_runner
    assert "wait_for_amcl_tf_warmup" in amcl_runner
    assert 'source "${SCRIPT_DIR}/map_server_helpers.sh"' in amcl_runner
    assert 'wait_for_occupancy_grid "/map" "${map_timeout}"' in amcl_runner
    assert 'wait_for_topic_message "/map" "${map_timeout}"' not in amcl_runner
    assert "AMCL_LIFECYCLE_HELPER=" in amcl_runner
    assert 'python3 "${AMCL_LIFECYCLE_HELPER}"' in amcl_runner
    assert "--per-node-timeout-sec" in amcl_runner
    assert "--change-state-response-timeout-sec" in amcl_runner
    assert "request_amcl_lifecycle_transition()" not in amcl_runner
    assert "amcl_lifecycle_transition_state_reached()" not in amcl_runner
    assert 'ros2 service call \\\n    "/${AMCL_NODE_NAME}/change_state"' not in amcl_runner
    assert 'ros2 lifecycle set "/${AMCL_NODE_NAME}" configure' not in amcl_runner
    assert 'ros2 lifecycle set "/${AMCL_NODE_NAME}" activate' not in amcl_runner
    assert "seed_amcl_initial_pose" in amcl_runner
    assert "start_scan_admission_relay" in amcl_runner
    assert "scan_max_age_ms" in amcl_runner
    assert '-p "max_age_ms:=${scan_max_age_ms}"' in amcl_runner
    assert '-p "wait_for_tf_timeout_ms:=${scan_wait_for_tf_timeout_ms}"' in amcl_runner
    assert "wait_for_amcl_pose_fresh" in amcl_runner
    assert "scan_topic:=$(effective_scan_topic)" in amcl_runner
    assert "tf_broadcast=false" in amcl_verify
    assert "AMCL has a /tf publisher endpoint" in amcl_verify
    assert "sensor_msgs/msg/LaserScan" in amcl_verify
    assert "/scan_amcl" in amcl_verify
    assert "--seed" in amcl_verify
    assert "--tf-warmup-sec" in amcl_verify
    assert "--scan-admission" in amcl_verify
    assert "--check-logs" in amcl_verify
    assert "verify_amcl_bridge_status_once" in amcl_verify
    assert "rclpy.spin_once" in amcl_verify
    assert "Please set the initial pose" in amcl_verify
    assert "Message Filter dropping message" in amcl_verify
    assert "run_amcl_shadow_localization.sh" in stop_nav
    assert "NJRH_NAV_STOP_AMCL_TIMEOUT_SEC" in stop_nav
    assert "stop_amcl_bounded()" in stop_nav
    assert "nav2_amcl/amcl" in stop_nav
    assert "amcl_scan_admission_node" in stop_nav
    assert "run_amcl_shadow_localization.sh\" --stop >/dev/null" not in stop_nav
    assert "killall -9" not in amcl_runner
    assert "killall -9" not in amcl_verify
    assert "pkill -9" not in amcl_runner
    assert "pkill -9" not in amcl_verify

    assert "amcl_localization_profile.env" in common_env
    assert "NJRH_AMCL_LOCALIZATION_MODE" in run_bridge
    assert "amcl_input_enabled:=${AMCL_INPUT_ENABLED}" in run_bridge
    assert "amcl_gate_mode:=${AMCL_GATE_MODE}" in run_bridge
    assert "amcl_scan_admission_enabled:=${AMCL_SCAN_ADMISSION_ENABLED}" in run_bridge
    assert "start_amcl_resident_if_enabled_for_navigation" in nav_runtime
    assert "complete_amcl_readiness_if_enabled_for_navigation" in nav_runtime
    assert "initial localization accepted" in nav_runtime
    assert "resident AMCL localization candidate" in nav_runtime

    for cfg in (bridge_cfg, bridge_src_cfg):
        assert "amcl_pose_topic: /amcl_pose" in cfg
        assert "amcl_input_enabled: false" in cfg
        assert "amcl_gate_mode: shadow" in cfg
        assert "amcl_max_result_age_ms: 1000.0" in cfg
        assert "amcl_small_correction_translation_m: 0.12" in cfg
        assert "amcl_small_correction_yaw_rad: 0.20" in cfg
        assert "amcl_medium_correction_translation_m: 0.28" in cfg
        assert "amcl_medium_correction_yaw_rad: 0.35" in cfg
        assert "amcl_medium_correction_consistency_count: 3" in cfg
        assert "amcl_hard_reject_translation_m: 0.60" in cfg
        assert "amcl_hard_reject_yaw_rad: 0.8" in cfg
        assert "amcl_max_xy_covariance: 0.15" in cfg
        assert "amcl_max_yaw_covariance: 0.10" in cfg
        assert "amcl_seed_service: /robot_localization_bridge/seed_amcl_initial_pose" in cfg
        assert "amcl_pose_max_age_ms: 1000.0" in cfg
        assert "amcl_initial_pose_publish_repetitions: 3" in cfg
        assert "amcl_scan_admission_enabled: false" in cfg
        assert "amcl_scan_admission_status_topic: /amcl_scan_admission/status" in cfg

    assert "on_amcl_pose" in bridge_cpp
    assert "amcl_pose_topic_" in bridge_cpp
    assert "accept_amcl_candidate" in bridge_cpp
    assert "AMCL_SMALL_CORRECTION" in bridge_cpp
    assert "AMCL_MEDIUM_CONSISTENT_CORRECTION" in bridge_cpp
    assert "AMCL_MEDIUM_CORRECTION_WAITING_FOR_CONSISTENCY" in bridge_cpp
    assert "AMCL_SHADOW_ONLY" in bridge_cpp
    assert "AMCL_CORRECTION_TOO_LARGE" in bridge_cpp
    assert "AMCL_NOT_SEEDED" in bridge_cpp
    assert "AMCL_POSE_STALE" in bridge_cpp
    assert "AMCL_SCAN_TF_UNAVAILABLE" in bridge_cpp
    assert "AMCL_TF_LOOKUP_FAILED" in bridge_cpp
    assert "AMCL_COVARIANCE_TOO_LARGE" in bridge_cpp
    assert "_covariance_rejected" in bridge_cpp
    assert "on_amcl_seed_request" in bridge_cpp
    assert "amcl_initial_pose_pub_" in bridge_cpp
    assert "amcl_seed_succeeded_" in bridge_cpp
    assert '",\\"amcl_initial_pose_seed_count\\":" << amcl_initial_pose_seed_count_' in bridge_cpp
    assert "amcl_ready" in bridge_cpp
    assert "amcl_scan_admission_hz" in bridge_cpp
    assert '",\\"last_accept_reason\\":\\"" << json_escape(last_accept_reason_)' in bridge_cpp
    assert "active_correction_source" in bridge_cpp
    assert "last_accepted_source" in bridge_cpp
    assert "last_rejected_source" in bridge_cpp
    assert "isaac_triggered" in bridge_cpp
    assert "amcl_gated" in bridge_cpp
    assert "amcl_shadow" in bridge_cpp
    assert "lookupTransform(odom_frame_, base_frame_, stamp" in bridge_cpp
    assert "tf_broadcaster_->sendTransform(tf)" in bridge_cpp

    assert "class AmclScanAdmissionRelay" in amcl_scan_relay
    assert "ParameterDescriptor(dynamic_typing=True)" in amcl_scan_relay
    assert "self.pub.publish(msg)" in amcl_scan_relay
    assert "Preserve the original header.stamp" in amcl_scan_relay
    assert "self.max_age_ms" in amcl_scan_relay
    assert "can_transform" in amcl_scan_relay
    assert "dropped_age_count" in amcl_scan_relay
    assert "dropped_tf_count" in amcl_scan_relay
    assert "message_filter_drop_detected" in amcl_scan_relay

    assert "MPPIController" in nav2
    assert "SmacPlanner2D" in nav2
    assert "global_frame: odom" in local_costmap_config_block(nav2)
    assert "global_frame: base_link" not in local_costmap_config_block(nav2)

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            subprocess.run([bash, "-n", str(amcl_runner_path)], check=True)
            subprocess.run([bash, "-n", str(amcl_verify_path)], check=True)


def test_phase_a13_amcl_scan_admission_cpu_affinity_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"

    affinity_cfg = (config_dir / "cpu_affinity.env").read_text(encoding="utf-8")
    amcl_runner_path = scripts_dir / "run_amcl_shadow_localization.sh"
    amcl_runner = amcl_runner_path.read_text(encoding="utf-8")
    inspect_path = scripts_dir / "inspect_runtime_cpu_affinity.sh"
    observe_path = scripts_dir / "observe_navigation_tf_jitter_180s.sh"
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    amcl_cfg = (config_dir / "amcl_shadow.yaml").read_text(encoding="utf-8")
    relay = (scripts_dir / "amcl_scan_admission_relay.py").read_text(encoding="utf-8")

    assert 'export NJRH_CPUSET_AMCL="${NJRH_CPUSET_AMCL:-${NJRH_CPUSET_LOCALIZATION}}"' in affinity_cfg
    assert (
        'export NJRH_CPUSET_AMCL_SCAN_ADMISSION="${NJRH_CPUSET_AMCL_SCAN_ADMISSION:-${NJRH_CPUSET_LOCALIZATION}}"'
        in affinity_cfg
    )
    assert 'source "${SCRIPT_DIR}/cpu_affinity.sh"' in amcl_runner
    assert "njrh_cpuset_for amcl" in amcl_runner
    assert 'export NJRH_CPUSET_AMCL="${amcl_cpuset}"' in amcl_runner
    assert 'taskset -c "${amcl_cpuset}" true' in amcl_runner
    assert 'nohup taskset -c "${amcl_cpuset}" "${AMCL_BIN}" --ros-args' in amcl_runner
    assert "njrh_apply_affinity_to_pids amcl" in amcl_runner
    assert "njrh_cpuset_for amcl_scan_admission" in amcl_runner
    assert 'export NJRH_CPUSET_AMCL_SCAN_ADMISSION="${relay_cpuset}"' in amcl_runner
    assert 'taskset -c "${relay_cpuset}" true' in amcl_runner
    assert 'nohup taskset -c "${relay_cpuset}" "${relay_cmd[@]}"' in amcl_runner
    assert 'nohup python3 "${SCAN_RELAY_SCRIPT}"' not in amcl_runner
    assert "AMCL scan admission relay started implementation=${SCAN_RELAY_IMPL}" in amcl_runner
    assert "input_topic=${scan_input_topic}" in amcl_runner
    assert "output_topic=${scan_output_topic}" in amcl_runner
    assert "rate_hz=${scan_rate_hz}" in amcl_runner
    assert "scan_relay_allowed_cpus" in amcl_runner
    assert "njrh_apply_affinity_to_pids amcl_scan_admission" in amcl_runner
    assert "killall -9" not in amcl_runner
    assert "pkill -9" not in amcl_runner

    assert inspect_path.exists()
    inspect = inspect_path.read_text(encoding="utf-8")
    assert "amcl_scan_admission_node" in inspect
    assert "amcl_scan_admission_relay.py" in inspect
    assert "Cpus_allowed_list" in inspect
    assert "allows_forbidden_${forbidden}" in inspect
    assert '"2,3,7"' in inspect
    assert "robot_localization_bridge" in inspect
    assert "robot_local_state_ekf" in inspect
    assert "wheel_odom_ekf_input" in inspect
    assert "controller_server" in inspect
    assert "hesai_accel_driver_node" in inspect
    assert "robot_safety" in inspect
    assert "ranger_base_node" in inspect

    assert observe_path.exists()
    observe = observe_path.read_text(encoding="utf-8")
    assert "does not send navigation goals" in observe
    assert "/local_state/odometry" in observe
    assert "/wheel/odom" in observe
    assert "/wheel/odom_ekf" in observe
    assert "/scan_amcl" in observe
    assert "/amcl_pose" in observe
    assert "/localization/bridge_status" in observe
    assert "tf:odom->base_link" in observe
    assert "tf:map->odom" in observe
    assert "tf:base_link->lidar_level_link" in observe
    assert "tf_future_extrapolation_count" in observe
    assert "message_filter_drop_count" in observe
    assert "failed_to_make_progress_count" in observe
    assert "admission_relay" in observe
    assert "amcl_scan_admission_node" in observe
    assert "PointCloud2" not in observe
    assert "ros2 action send_goal" not in observe
    assert "ros2 topic pub" not in observe

    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "tf_future_stamp_offset_sec: 0.0" in bridge_cfg
    assert "transform_tolerance: 0.10" in nav2
    assert "global_frame: odom" in local_costmap_config_block(nav2)
    assert "global_frame: base_link" not in local_costmap_config_block(nav2)
    assert "tf_broadcast: false" in amcl_cfg
    assert "scan_topic: /scan_amcl" in amcl_cfg
    assert "self.pub.publish(msg)" in relay
    assert "Preserve the original header.stamp" in relay
    assert "msg.header.stamp =" not in relay
    assert "MPPIController" in nav2
    assert "SmacPlanner2D" in nav2

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            subprocess.run([bash, "-n", str(amcl_runner_path)], check=True)
            subprocess.run([bash, "-n", str(inspect_path)], check=True)
            subprocess.run([bash, "-n", str(observe_path)], check=True)


def test_phase_a14_cpp_amcl_scan_admission_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"
    bridge_dir = ROOT / "src" / "robot_localization_bridge"

    cpp_path = bridge_dir / "src" / "amcl_scan_admission_node.cpp"
    assert cpp_path.exists()
    cpp = cpp_path.read_text(encoding="utf-8")
    cmake = (bridge_dir / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (bridge_dir / "package.xml").read_text(encoding="utf-8")
    profile = (config_dir / "amcl_localization_profile.env").read_text(encoding="utf-8")
    runner_path = scripts_dir / "run_amcl_shadow_localization.sh"
    runner = runner_path.read_text(encoding="utf-8")
    verify_path = scripts_dir / "verify_amcl_shadow_localization.sh"
    verify = verify_path.read_text(encoding="utf-8")
    inspect_path = scripts_dir / "inspect_runtime_cpu_affinity.sh"
    inspect = inspect_path.read_text(encoding="utf-8")
    observe_path = scripts_dir / "observe_navigation_tf_jitter_180s.sh"
    observe = observe_path.read_text(encoding="utf-8")
    amcl_cfg = (config_dir / "amcl_shadow.yaml").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    python_relay = (scripts_dir / "amcl_scan_admission_relay.py").read_text(encoding="utf-8")

    assert "add_executable(amcl_scan_admission_node src/amcl_scan_admission_node.cpp)" in cmake
    assert (
        "TARGETS correction_pause_arbiter localization_bridge_node amcl_scan_admission_node"
        in cmake
    )
    for dep in ("sensor_msgs", "tf2", "tf2_ros", "geometry_msgs", "rclcpp"):
        assert dep in cmake
        assert f"<depend>{dep}</depend>" in package

    assert 'export NJRH_AMCL_SCAN_ADMISSION_IMPL="${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}"' in profile
    assert 'SCAN_RELAY_IMPL="${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}"' in runner
    assert "SCAN_RELAY_CPP_BIN" in runner
    assert 'relay_cmd=("${SCAN_RELAY_CPP_BIN}" --ros-args' in runner
    assert "ros2 run robot_localization_bridge amcl_scan_admission_node --ros-args" not in runner
    assert "python3 \"${SCAN_RELAY_SCRIPT}\"" in runner
    assert "wait_for_fresh_amcl_scan_input()" in runner
    assert "runtime_readiness_probe \\" in runner
    assert "fresh-header-topic" in runner
    readiness_block = runner[
        runner.index("complete_amcl_readiness_sequence()") :
        runner.index("start_amcl_node()", runner.index("complete_amcl_readiness_sequence()"))
    ]
    assert readiness_block.index("wait_for_amcl_tf_warmup true") < readiness_block.index(
        "wait_for_fresh_amcl_scan_input"
    )
    assert readiness_block.index("wait_for_fresh_amcl_scan_input") < readiness_block.index(
        "start_scan_admission_relay"
    )
    assert "/scan did not become fresh before AMCL scan admission" in runner
    assert "explicitly set NJRH_AMCL_SCAN_ADMISSION_IMPL=python" in runner
    assert "implementation=${SCAN_RELAY_IMPL}" in runner
    assert "max_scan_age_ms=${scan_max_age_ms}" in runner
    assert "tf_wait_timeout_ms=${scan_wait_for_tf_timeout_ms}" in runner
    assert "nohup taskset -c \"${relay_cpuset}\" \"${relay_cmd[@]}\"" in runner
    assert "killall -9" not in runner
    assert "pkill -9" not in runner
    assert "amcl_heartbeat_process_pids()" in runner
    assert "amcl_nonstop_runner_process_pids()" in runner
    assert "stop_amcl_runner_processes()" in runner
    assert "stop_amcl_heartbeat_processes()" in runner
    assert "stopping remaining AMCL runner processes" in runner
    assert "AMCL runner processes ignored SIGTERM; killing exact pids" in runner
    assert "stop_amcl_runner_processes" in runner[runner.index("stop_amcl()") :]
    assert "stop_amcl_heartbeat_processes" in runner[runner.index("stop_amcl()") :]
    assert "/run_amcl_shadow_localization.sh/ && /--heartbeat/" in runner
    assert "/run_amcl_shadow_localization.sh/ && !/--stop/" in runner

    assert "declare_parameter<std::string>(\"input_topic\", \"/scan\")" in cpp
    assert "declare_parameter<std::string>(\"output_topic\", \"/scan_amcl\")" in cpp
    assert "declare_parameter<std::string>(\"target_frame\", \"odom\")" in cpp
    assert "declare_parameter<double>(\"max_rate_hz\", 5.0)" in cpp
    assert "declare_parameter<double>(\"max_scan_age_ms\", 1000.0)" in cpp
    assert "declare_parameter<double>(\"tf_wait_timeout_ms\", 20.0)" in cpp
    assert "declare_parameter<bool>(\"require_tf_available\", true)" in cpp
    assert "declare_parameter<bool>(\"preserve_stamp\", true)" in cpp
    assert "declare_parameter<bool>(\"drop_if_future_stamp\", true)" in cpp
    assert "declare_parameter<double>(\"max_future_stamp_ms\", 50.0)" in cpp
    assert "declare_parameter<double>(\"rate_hz\", max_rate_hz_)" in cpp
    assert "declare_parameter<double>(\"max_age_ms\", max_scan_age_ms_)" in cpp
    assert "declare_parameter<double>(\"wait_for_tf_timeout_ms\", tf_wait_timeout_ms_)" in cpp
    assert "declare_parameter<bool>(\"drop_if_tf_unavailable\", require_tf_available_)" in cpp
    assert "lookupTransform(\n        target_frame_, msg.header.frame_id, msg.header.stamp" in cpp
    assert "pub_->publish(*msg)" in cpp
    assert "header.stamp =" not in cpp
    assert "header.frame_id =" not in cpp
    assert "ranges" not in cpp
    assert "\\\"implementation\\\":\\\"cpp\\\"" in cpp
    assert "dropped_age_count" in cpp
    assert "dropped_tf_count" in cpp
    assert "dropped_rate_count" in cpp
    assert "dropped_future_count" in cpp
    assert "AMCL scan admission status implementation=cpp" in cpp

    assert "EXPECTED_SCAN_ADMISSION_IMPL=\"${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}\"" in verify
    assert "amcl_scan_admission_node" in verify
    assert "amcl_scan_admission_relay.py" in verify
    assert "Python AMCL scan admission fallback is running while expected impl=cpp" in verify
    assert "scan_amcl_age_ms" in verify
    assert "possible restamp" in verify
    assert "scan admission status has ${field}" in verify
    assert "tf_broadcast=false" in verify
    assert "bridge_status owner is robot_localization_bridge" in verify

    assert "expected_amcl_scan_impl=\"${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}\"" in inspect
    assert "amcl_scan_admission_node" in inspect
    assert "python_fallback_running_by_default" in inspect
    assert '"2,3,7"' in inspect

    assert "admission_relay" in observe
    assert "amcl_scan_admission_node" in observe
    assert "Cpus_allowed_list" in observe
    assert "pcpu" in observe
    assert "does not send navigation goals" in observe
    assert "PointCloud2" not in observe

    assert "scan_topic: /scan_amcl" in amcl_cfg
    assert "tf_broadcast: false" in amcl_cfg
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "tf_future_stamp_offset_sec: 0.0" in bridge_cfg
    assert "MPPIController" in nav2
    assert "SmacPlanner2D" in nav2
    assert "global_frame: odom" in local_costmap_config_block(nav2)
    assert "global_frame: base_link" not in local_costmap_config_block(nav2)

    assert "self.pub.publish(msg)" in python_relay
    assert "Preserve the original header.stamp" in python_relay

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            subprocess.run([bash, "-n", str(runner_path)], check=True)
            subprocess.run([bash, "-n", str(verify_path)], check=True)
            subprocess.run([bash, "-n", str(inspect_path)], check=True)
            subprocess.run([bash, "-n", str(observe_path)], check=True)


def test_phase_a2_always_on_amcl_runtime_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"

    profile = (config_dir / "amcl_localization_profile.env").read_text(encoding="utf-8")
    amcl_cfg = (config_dir / "amcl_shadow.yaml").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    bridge_cpp = (ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp").read_text(
        encoding="utf-8"
    )
    navigation_state_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_state.cpp"
    ).read_text(encoding="utf-8")
    navigation_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    localization_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    system_status_wiring_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "system_status"
        / "system_status_wiring.cpp"
    ).read_text(encoding="utf-8")
    api_observability_cpp = "\n".join(
        (
            navigation_state_cpp,
            navigation_module_cpp,
            localization_module_cpp,
            system_status_wiring_cpp,
            localization_configuration_source(),
        )
    )
    runtime_path = scripts_dir / "run_navigation_runtime_services.sh"
    runtime = runtime_path.read_text(encoding="utf-8")
    lifecycle_sequence = (scripts_dir / "nav2_lifecycle_sequence.py").read_text(encoding="utf-8")
    common_runner = (scripts_dir / "run_common_services.sh").read_text(encoding="utf-8")
    canonical_helpers = (scripts_dir / "canonical_tf_helpers.sh").read_text(encoding="utf-8")
    amcl_runner_path = scripts_dir / "run_amcl_shadow_localization.sh"
    amcl_runner = amcl_runner_path.read_text(encoding="utf-8")
    verify_path = scripts_dir / "verify_amcl_runtime_readiness.sh"
    verify_contract_path = scripts_dir / "verify_amcl_runtime_contract.sh"
    verify_status_path = scripts_dir / "verify_amcl_readiness_status.sh"
    verify_nomotion_path = scripts_dir / "verify_amcl_nomotion_readiness.sh"
    nomotion_probe_path = scripts_dir / "amcl_nomotion_update_probe.py"
    observe_path = scripts_dir / "observe_amcl_navigation_shadow_180s.sh"
    amcl_status = (ROOT / "src/robot_bringup/src/runtime_amcl_status_server.cpp").read_text(encoding="utf-8")
    amcl_status_header = (ROOT / "src/robot_bringup/include/robot_bringup/runtime_amcl_status.hpp").read_text(encoding="utf-8")
    verify = verify_path.read_text(encoding="utf-8")
    verify_contract = verify_contract_path.read_text(encoding="utf-8")
    verify_status = verify_status_path.read_text(encoding="utf-8")
    verify_nomotion = verify_nomotion_path.read_text(encoding="utf-8")
    nomotion_probe = nomotion_probe_path.read_text(encoding="utf-8")
    observe = observe_path.read_text(encoding="utf-8")

    assert 'export NJRH_AMCL_LOCALIZATION_MODE="${NJRH_AMCL_LOCALIZATION_MODE:-gated}"' in profile
    assert "shadow starts AMCL and lets robot_localization_bridge compute/report AMCL" in profile
    assert "gated starts AMCL and lets robot_localization_bridge accept only small" in profile
    assert "disabled|shadow|gated" in amcl_runner
    assert "--start-resident" in amcl_runner
    assert "--complete-readiness" in amcl_runner
    assert "--heartbeat" in amcl_runner
    assert "wait_for_amcl_tf_warmup false" in amcl_runner
    assert "wait_for_amcl_tf_warmup true" in amcl_runner
    assert "start_scan_admission_relay || return 2" in amcl_runner
    assert "wait_for_scan_admission_status_ready || return 3" in amcl_runner
    assert "NJRH_AMCL_SCAN_ADMISSION_READY_MIN_HZ:-0.5" in amcl_runner
    assert "blocking_errors = (" in amcl_runner
    assert '"AMCL_SCAN_TF_UNAVAILABLE"' in amcl_runner
    assert '"AMCL_SCAN_FRAME_MISMATCH"' in amcl_runner
    assert '"AMCL_SCAN_FUTURE_STAMP"' in amcl_runner
    assert '"AMCL_SCAN_WARMUP"' in amcl_runner
    assert "published_count = int(data.get(\"published_count\", 0) or 0)" in amcl_runner
    assert "hz >= min_ready_hz and published_count > 0" in amcl_runner
    assert "str(data.get(\"last_error\", \"none\")) in (\"\", \"none\")" not in amcl_runner
    assert "seed_amcl_initial_pose || return 4" in amcl_runner
    seed_fn = amcl_runner[
        amcl_runner.index("seed_amcl_initial_pose()") :
        amcl_runner.index("start_scan_admission_relay()", amcl_runner.index("seed_amcl_initial_pose()"))
    ]
    assert "client = node.create_client(Trigger, service)" in seed_fn
    assert "ros2 service call" not in seed_fn
    assert 'status_log_period_sec:=${NJRH_AMCL_SCAN_ADMISSION_STATUS_LOG_PERIOD_SEC:-1.0}' in amcl_runner
    assert "request_amcl_nomotion_update" in amcl_runner
    assert "request_amcl_nomotion_update_and_wait_for_pose" in amcl_runner
    assert "/request_nomotion_update" in amcl_runner
    assert "amcl_nomotion_update_probe.py" in amcl_runner
    assert "ros2 service call \"${service}\" std_srvs/srv/Empty" not in amcl_runner
    assert "wait_for_amcl_pose_fresh_or_nomotion_update" in amcl_runner
    assert "AMCL_RESIDENT mode=${MODE}" in amcl_runner
    assert "AMCL_READY mode=${MODE}" in amcl_runner
    assert "wait_for_amcl_tf_broadcast_false" in amcl_runner
    assert "NJRH_AMCL_PARAM_READY_TIMEOUT_SEC" in amcl_runner
    assert "amcl_cmdline_tf_broadcast_false" in amcl_runner
    assert "process launch argument is tf_broadcast:=false; continuing" in amcl_runner
    assert "did not become readable as false" in amcl_runner
    assert "NJRH_AMCL_RUNTIME_STATUS_FILE" in amcl_runner
    assert "/tmp/njrh_amcl_runtime_status.env" in amcl_runner
    assert "AMCL_EXIT_DEGRADED=10" in amcl_runner
    assert "AMCL_EXIT_GATED_NOT_READY=21" in amcl_runner
    assert "AMCL_EXIT_SCAN_ADMISSION_FAILED=22" in amcl_runner
    assert "AMCL_EXIT_LIFECYCLE_FAILED=23" in amcl_runner
    assert "AMCL_EXIT_SEED_FAILED=24" in amcl_runner
    assert "AMCL_EXIT_POSE_MISSING=25" in amcl_runner
    assert "write_amcl_runtime_status" in amcl_runner
    assert "source_status_file_if_valid" not in amcl_runner
    assert "status_file_value()" not in amcl_runner
    assert "load_existing_amcl_runtime_status" not in amcl_runner
    assert 'amcl_status_cli submit' in amcl_runner
    assert '--set "LIFECYCLE_VERIFIED=${AMCL_PROGRESS_LIFECYCLE:-false}"' in amcl_runner
    assert '--set "PROGRESS_KEY=${AMCL_PROGRESS_KEY:-}"' in amcl_runner
    assert 'local owner_pid="${NJRH_AMCL_STATUS_OWNER_PID:-${NJRH_STARTUP_OWNER_PID:-$PPID}}"' in amcl_runner
    assert '--owner-pid "${owner_pid}" --owner-generation "${owner_generation}"' in amcl_runner
    assert 'same_process(new_amcl, amcl)' in amcl_status
    assert 'mg == map_generation' in amcl_status
    assert 'og == owner_generation' in amcl_status
    assert 'pk == progress_key' in amcl_status
    assert 'AMCL_SEED_SUCCEEDED=true' in amcl_runner
    assert 'AMCL_STATIC_STANDBY_ACCEPTED=true' in amcl_runner
    assert 'mv -f "${tmp_file}" "${STATUS_FILE}"' not in amcl_runner
    assert 'atomic_write(path, env_document(f))' in amcl_status
    assert 'flock(lock.value, LOCK_EX | LOCK_NB)' in amcl_status
    assert 'SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC' in amcl_status
    assert 'SO_PEERCRED' in amcl_status
    assert 'AMCL_STATUS_WRITER"] = "runtime_health_guard"' in amcl_status
    assert "heartbeat_amcl_runtime_status" in amcl_runner
    assert 'amcl_status_cli ping >/dev/null' in amcl_runner
    assert "AMCL status heartbeat started" not in amcl_runner
    assert 'double refresh_sec{2.0}' in amcl_status_header
    assert 'double ttl_sec{5.0}' in amcl_status_header
    assert '!seed && result == "ready"' in amcl_status
    assert 'result = "waiting_seed"' in amcl_status
    assert "AMCL seed has not completed" in amcl_status
    assert 'finish_amcl_status waiting_seed false false "resident AMCL started; waiting for initial pose seed"' in amcl_runner
    assert 'finish_amcl_status ready true false ""' in amcl_runner
    assert 'write_amcl_runtime_status "${start_result}" "${ready}" "${degraded}" "${reason}" || return "${AMCL_EXIT_FAILED}"' in amcl_runner
    assert "AMCL_STATE" in amcl_status
    assert "AMCL_STATUS_STAMP_SEC" in amcl_status
    assert "AMCL_STATUS_STALE" in amcl_status
    assert 'yes(evidence, "AMCL_STATIC_STANDBY_ACCEPTED")' in amcl_status
    assert 'yes(evidence, "AMCL_NOMOTION_POSE_RECEIVED")' in amcl_status
    assert "AMCL_PROCESS_READY" in amcl_status
    assert "AMCL_SEED_RESPONSE_OK" in amcl_runner
    assert "AMCL_NOMOTION_PROBE_USED" in amcl_runner
    assert "AMCL_NOMOTION_POSE_RECEIVED" in amcl_runner
    assert "AMCL_STATIC_STANDBY_ACCEPTED" in amcl_runner
    assert "NJRH_AMCL_STATIC_STANDBY_WITHOUT_POSE_OK:-true" in amcl_runner
    assert "NJRH_AMCL_STATIC_STANDBY_SKIP_POSE_WAIT:-true" in amcl_runner
    assert "AMCL static standby accepted immediately after seed" in amcl_runner
    assert "AMCL static standby accepted after seed without a fresh/no-motion pose" in amcl_runner
    assert "NJRH_AMCL_STATUS_GRAPH_PROBE_ENABLED" not in amcl_runner
    assert "void StatusServer::observe_graph" in amcl_status
    assert "graph.available" in amcl_status
    assert "AMCL_STATIC_STANDBY" in amcl_status
    assert "AMCL_CORRECTION_READY" in amcl_status
    assert "AMCL_DEGRADED" in amcl_status
    assert "AMCL_FAILED" in amcl_status
    assert "AMCL_PID_STALE_CLEARED" in amcl_runner
    assert "SCAN_ADMISSION_PID_STALE_CLEARED" in amcl_runner
    assert "validated_pid_from_file" in amcl_runner
    assert "pid_cmdline_matches" in amcl_runner
    assert "finish_amcl_status degraded false true" in amcl_runner
    assert "finish_amcl_status failed false false" in amcl_runner
    assert "continuing triggered localization baseline\"\\n    return 0" not in amcl_runner
    assert 'AMCL lifecycle is active but not ready; continuing triggered localization baseline"\n    return 0' not in amcl_runner

    assert "start_amcl_resident_if_enabled_for_navigation" in runtime
    assert "start_amcl_resident_background_if_enabled_for_navigation" in runtime
    assert "start_amcl_readiness_background_if_enabled_for_navigation" in runtime
    assert "wait_for_amcl_readiness_background_if_running" in runtime
    assert "start_prestarted_nav2_lifecycle_background()" in runtime
    assert "wait_for_prestarted_nav2_lifecycle_background()" in runtime
    assert "wait_for_prestarted_nav2_launch_hold_ready()" in runtime
    assert "prestarted Nav2 held launch ready from" in runtime
    assert "NAV2_HOLD_READY_WRAPPER_PID" in runtime
    assert "NAV2_HOLD_READY_BASHPID" in runtime
    assert "NAV2_HOLD_READY_CONTROLLER_PID" in runtime
    assert "NJRH_NAV2_HOLD_READY_FILE" in runtime
    assert "NJRH_NAV2_PRESTART_HOLD_READY_TIMEOUT_SEC:-25" in runtime
    assert "NJRH_NAV2_PRESTART_HOLD_READY_MAX_AGE_SEC" not in runtime
    assert "NJRH_NAV2_LIFECYCLE_BACKGROUND_START:-false" in runtime
    assert "NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false" in runtime
    assert "starting prestarted Nav2 lifecycle background after localization stack readiness" in runtime
    assert "NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false" in runtime
    assert "NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false" in runtime
    assert "NJRH_NAV2_LIFECYCLE_PARALLEL_BT:-true" in runtime
    assert "NJRH_NAV2_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE:-true" in runtime
    assert "NJRH_INITIAL_LOCALIZATION_REQUIRE_RESULT_PUBLISHER:-false" in runtime
    assert "skipping /localization_result publisher pre-gate" in runtime
    assert "NJRH_INITIAL_LOCALIZATION_FLATSCAN_WAIT_SEC:-5" in runtime
    assert "NJRH_INITIAL_LOCALIZATION_FLATSCAN_REPAIR_WAIT_SEC:-20" in runtime
    assert 'bash -n "${NJRH_AMCL_RUNTIME_STATUS_FILE}"' not in runtime
    assert 'snapshot="$(amcl_status_client_for_navigation read)"' in runtime
    assert 'printf -v "${key}"' in runtime
    assert "failed to source AMCL runtime status file" not in runtime
    assert "run_nav2_lifecycle_sequence()" in runtime
    assert "run_nav2_lifecycle_sequence_until_active()" in runtime
    assert 'runtime_readiness_probe lifecycle-active "${node_name}"' in runtime
    assert "lifecycle nodes active; stopping lifecycle helper pid=" in runtime
    assert 'if wait "${nav2_lifecycle_bringup_pid}"; then' in runtime
    assert "prestarted Nav2 lifecycle helper exited rc=" in runtime
    assert "prestarted Nav2 lifecycle nodes active before background helper exit" in runtime
    assert "NJRH_NAV2_LIFECYCLE_BACKGROUND_ACTIVE_WAIT_SEC" in runtime
    assert "NJRH_NAV2_LIFECYCLE_BACKGROUND_JOIN_TIMEOUT_SEC" not in runtime
    assert "Nav2 lifecycle parallel core activation enabled" in runtime
    assert "Nav2 startup worker running in background" in runtime
    assert "--trust-change-state-response" in lifecycle_sequence
    assert "--configure-all-before-activate" in lifecycle_sequence
    assert "lifecycle sequence: configuring all managed nodes before activation" in lifecycle_sequence
    assert "lifecycle configure complete node=" in lifecycle_sequence
    assert "lifecycle activate complete node=" in lifecycle_sequence
    resident_lifecycle_block = runtime[
        runtime.index("activate_prestarted_nav2_lifecycle()") :
        runtime.index("ensure_navigation_layer_alive || return 1", runtime.index("activate_prestarted_nav2_lifecycle()"))
    ]
    assert "waypoint_follower" not in resident_lifecycle_block
    assert "smoother_server" in resident_lifecycle_block
    assert "behavior_server" in resident_lifecycle_block
    assert resident_lifecycle_block.index("behavior_server") < resident_lifecycle_block.index("smoother_server")
    assert resident_lifecycle_block.index("smoother_server") < resident_lifecycle_block.index("bt_navigator")
    assert resident_lifecycle_block.index("planner_server") < resident_lifecycle_block.index("controller_server")
    assert "start_amcl_status_heartbeat_if_enabled_for_navigation" in runtime
    assert "amcl_status_heartbeat_pid" not in runtime
    assert "amcl_status_client_for_navigation register" in runtime
    assert 'export NJRH_AMCL_STATUS_OWNER_PID="${NJRH_STARTUP_OWNER_PID}"' in runtime
    assert "amcl_status_client_for_navigation ping >/dev/null" in runtime
    assert "AMCL observer unavailable; do not classify an observer failure as a navigation producer failure" in runtime
    assert "cleanup_stale_amcl_runtime_status_owner" in common_runner
    assert "cleanup_stale_amcl_runtime_status_owner" in runtime
    assert "resident_navigation_layer_pids()" in common_runner
    assert "nav2_lifecycle_sequence.py" in common_runner
    assert "call_global_localization_trigger.py" in common_runner
    assert "resident_navigation_layers_running()" in common_runner
    assert "resolve_resident_navigation_autostart_selection()" in common_runner
    assert "start_resident_navigation_autostart_if_selected()" in common_runner
    assert "wait_for_resident_navigation_autostart_if_started()" in common_runner
    assert "COMMON_STARTUP_STAGE stage=" in common_runner
    assert 'log_common_startup_stage "helpers_loaded"' in common_runner
    assert 'log_common_startup_stage "local_state_ready"' in common_runner
    assert 'log_common_startup_stage "common_core_services_started"' in common_runner
    assert 'log_common_startup_stage "common_startup_finished"' in common_runner
    assert 'NJRH_LOCAL_STATE_START_READY_MODE="${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}"' in common_runner
    resident_autostart_wait_block = common_runner[
        common_runner.index("wait_for_resident_navigation_autostart_if_started()") :
        common_runner.index("stop_stale_pointcloud_accel_pipeline_processes()")
    ]
    assert "common services will not block on navigation readiness" in resident_autostart_wait_block
    assert "wait_for_resident_navigation_context_ready" not in resident_autostart_wait_block
    assert "NJRH_RESIDENT_NAVIGATION_EARLY_AUTOSTART:-true" in common_runner
    assert "no stale resident navigation layers found; skipping Nav2/localization/AMCL cleanup sweep" in common_runner
    assert "njrh_load_pointcloud_ingress_profile" in common_runner
    assert "hesai_accel_driver_node" in common_runner
    assert "NJRH_COMMON_PROCESS_START_SETTLE_SEC:-0.2" in common_runner
    assert "ensure_flatscan_ready_before_navigation_autostart()" in common_runner
    assert "NJRH_COMMON_FLATSCAN_READY_TIMEOUT_SEC:-45" in common_runner
    assert "NJRH_COMMON_FLATSCAN_REPAIR_TIMEOUT_SEC:-60" in common_runner
    assert "common_require_flatscan_before_resident_autostart()" in common_runner
    assert "NJRH_COMMON_REQUIRE_FLATSCAN_BEFORE_RESIDENT_AUTOSTART:-false" in common_runner
    assert "skipping common /flatscan precheck before resident navigation autostart" in common_runner
    assert 'wait_for_topic_publisher_from_node "/flatscan" "laser_scan_to_flatscan"' in common_runner
    assert 'wait_for_topic_message "/flatscan"' not in common_runner
    assert "NJRH_POINTCLOUD_ACCEL_STOP_KILL_WAIT_SEC:-0.2" in common_runner
    assert "stale pointcloud accel process ignored SIGTERM; killing exact pids" in common_runner
    assert "/flatscan missing before resident navigation; restarting pointcloud accel" in common_runner
    common_main_flow = common_runner[common_runner.index("require_can_interface_up") :]
    assert common_main_flow.index('start_common_canonical_helper_background "ranger_chassis_common"') > common_main_flow.index(
        'start_common_process "pointcloud_accel_pipeline"'
    )
    assert common_main_flow.index('start_common_canonical_helper_background "robot_description_static_tf_common"') < common_main_flow.index(
        'start_common_process "pointcloud_accel_pipeline"'
    )
    assert common_main_flow.index("start_robot_local_state_common_background_if_enabled") > common_main_flow.index(
        'start_common_process "pointcloud_accel_pipeline"'
    )
    assert common_main_flow.index("NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART") < common_main_flow.index(
        "wait_for_robot_local_state_common_background_if_started"
    )
    assert common_main_flow.index("NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE") < common_main_flow.index(
        "start_resident_navigation_autostart_if_selected"
    )
    assert common_main_flow.index("wait_for_robot_local_state_common_background_if_started") < common_main_flow.index(
        "wait_for_runtime_health_local_state_ready"
    )
    assert "wait_for_fresh_header_topic_message" in runtime
    assert 'wait_for_fresh_tf_transform "odom" "base_link"' in runtime
    autostart_block = common_runner[
        common_runner.index("start_resident_navigation_autostart_if_selected()") :
        common_runner.index("wait_for_resident_navigation_autostart_if_started()")
    ]
    assert (
        'NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK="${NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false}"'
        in autostart_block
    )
    assert (
        'NJRH_NAV2_LIFECYCLE_PARALLEL_CORE="${NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false}"'
        in autostart_block
    )
    assert autostart_block.index("prepare_resident_navigation_autostart") < autostart_block.index(
        "common_require_flatscan_before_resident_autostart"
    )
    assert autostart_block.index("common_require_flatscan_before_resident_autostart") < autostart_block.index(
        'start_common_process "resident_navigation_runtime"'
    )
    assert "LOCAL_STATE_START_READY_TIMEOUT_SEC:-12" in canonical_helpers
    assert "LOCAL_STATE_READY_RECHECK_TIMEOUT_SEC:-12" in canonical_helpers
    assert "NJRH_RESIDENT_NAVIGATION_STOP_INT_WAIT_SEC:-0.5" in common_runner
    assert "NJRH_RESIDENT_NAVIGATION_STOP_TERM_WAIT_SEC:-0.5" in common_runner
    assert "NJRH_RESIDENT_NAVIGATION_STOP_KILL_WAIT_SEC:-0.2" in common_runner
    assert "stale_amcl_seed_helper_pids()" in common_runner
    assert 'grep -F "/robot_localization_bridge/seed_amcl_initial_pose"' in common_runner
    assert "stopping stale AMCL seed helper before common startup" in common_runner
    assert "NJRH_AMCL_SEED_HELPER_STOP_TERM_WAIT_SEC" in common_runner
    assert "killing stale AMCL seed helper before common startup" in common_runner
    assert "kill -KILL ${pids}" in common_runner
    assert "/run_amcl_shadow_localization.sh/ && /--heartbeat/" in common_runner
    assert "/run_amcl_shadow_localization.sh/ && /--heartbeat/" in runtime
    assert "stopping stale AMCL runtime status heartbeat before common startup" in common_runner
    assert "stopping stale AMCL runtime status heartbeat before resident navigation startup" in runtime
    assert "complete_amcl_readiness_if_enabled_for_navigation" in runtime
    assert "complete_amcl_readiness_with_retries_for_navigation" in runtime
    assert "run_amcl_localization_step" in runtime
    assert 'if ! run_amcl_localization_step "${mode}" "start-resident" --start-resident; then' in runtime
    assert 'if ! run_amcl_localization_step "${mode}" "complete-readiness" --complete-readiness; then' in runtime
    background_block = runtime[
        runtime.index("start_amcl_readiness_background_if_enabled_for_navigation()") :
        runtime.index(
            "wait_for_amcl_readiness_background_if_running()",
            runtime.index("start_amcl_readiness_background_if_enabled_for_navigation()"),
        )
    ]
    assert "wait_for_amcl_resident_background_if_running" in background_block
    assert "complete_amcl_readiness_with_retries_for_navigation" in background_block
    assert "AMCL readiness completed in background" in background_block
    assert "set +e" in runtime
    assert "rc=$?" in runtime
    assert "AMCL_DEGRADED phase=${phase}" in runtime
    assert "AMCL_GATED_NOT_READY" in runtime
    assert "AMCL_SCAN_ADMISSION_FAILED" in runtime
    assert "AMCL_POSE_MISSING" in runtime
    main_flow = runtime[
        runtime.index('write_runtime_map_context "starting" "false" "resident navigation runtime starting"') :
    ]
    assert main_flow.index("wait_for_initial_global_localization") < main_flow.index(
        "start_amcl_readiness_background_if_enabled_for_navigation"
    )
    assert main_flow.index("ensure_localization_stack_ready_for_navigation") < main_flow.index(
        "wait_for_initial_global_localization"
    )
    assert main_flow.index("wait_for_initial_global_localization") < main_flow.index(
        'log_startup_stage "nav2_layer_started_after_initial_localization"'
    )
    assert main_flow.index('log_startup_stage "nav2_layer_ready"') < main_flow.index(
        "wait_for_amcl_readiness_background_if_running"
    )
    assert 'log_startup_stage "amcl_tracking_ready"' in main_flow
    assert main_flow.index('log_startup_stage "amcl_tracking_ready"') < main_flow.index(
        "start_amcl_status_heartbeat_if_enabled_for_navigation"
    )
    assert main_flow.index("start_amcl_status_heartbeat_if_enabled_for_navigation") < main_flow.index(
        'commit_runtime_ready_context "${ready_context_message}"'
    )
    assert main_flow.index("wait_for_amcl_readiness_background_if_running") < main_flow.index(
        'commit_runtime_ready_context "${ready_context_message}"'
    )
    assert "complete_amcl_readiness_if_enabled_for_navigation ||" not in main_flow
    assert "--start-resident" in runtime
    assert "--complete-readiness" in runtime
    assert "--mode \"${mode}\" --restart" not in runtime

    assert "amcl_input_enabled" in bridge_cpp
    assert "amcl_gate_mode" in bridge_cpp
    assert "amcl_source_name" in bridge_cpp
    assert "AMCL_NOT_SEEDED" in bridge_cpp
    assert "AMCL_SHADOW_ONLY" in bridge_cpp
    assert "AMCL_SMALL_CORRECTION" in bridge_cpp
    assert "AMCL_CORRECTION_TOO_LARGE" in bridge_cpp
    assert "amcl_shadow_ready" in bridge_cpp
    assert "amcl_gated_ready" in bridge_cpp
    assert "amcl_status_file_stale" in bridge_cpp
    assert "stale_file_ignored" in bridge_cpp
    assert "amcl_process_ready" in bridge_cpp
    assert "amcl_seeded" in bridge_cpp
    assert "amcl_seed_response_ok" in bridge_cpp
    assert "amcl_nomotion_pose_received" in bridge_cpp
    assert "amcl_static_standby" in bridge_cpp
    assert "amcl_tracking_ready" in bridge_cpp
    assert "amcl_correction_ready" in bridge_cpp
    assert "amcl_correction_pending" in bridge_cpp
    assert "amcl_pose_age_ms" in bridge_cpp
    assert "amcl_pose_fresh" in bridge_cpp
    assert "amcl_not_moving_no_update_ok" in bridge_cpp
    assert "amcl_runtime_static_standby" in bridge_cpp
    assert "amcl_runtime_status.static_standby" in bridge_cpp
    assert "amcl_runtime_status.not_moving_no_update_ok" in bridge_cpp
    assert "amcl_runtime_status.tracking_ready && !amcl_runtime_status.correction_ready" in bridge_cpp
    assert "amcl_runtime_waiting_seed_resolved" in bridge_cpp
    assert "amcl_degraded_reason.clear()" in bridge_cpp
    assert "localization_degraded" in bridge_cpp
    assert "amcl_runtime_status_file" in bridge_cpp
    assert "read_amcl_runtime_status_file" in bridge_cpp
    assert "amcl_process_alive" in bridge_cpp
    assert "amcl_node_exists" in bridge_cpp
    assert "amcl_lifecycle_active" in bridge_cpp
    assert "amcl_scan_admission_alive" in bridge_cpp
    assert "amcl_pose_publisher_count" in bridge_cpp
    assert "amcl_scan_admission_status_publisher_count" in bridge_cpp
    assert "AMCL_UPSTREAM_MISSING" in bridge_cpp
    assert "amcl_state" in bridge_cpp
    assert "expected_map_to_odom_owner" in bridge_cpp
    assert "candidate.source == \"isaac_triggered\"" in bridge_cpp
    assert "publish_amcl_initial_pose(candidate.map_base_pose, \"isaac_triggered_accept\")" in bridge_cpp
    assert "tf_broadcaster_->sendTransform(tf)" in bridge_cpp
    assert "amcl_runtime_status_file" in api_observability_cpp
    assert "read_amcl_runtime_status" in api_observability_cpp
    assert "amcl_state" in api_observability_cpp
    assert "amcl_status_file_stale" in api_observability_cpp
    assert "amcl_process_ready" in api_observability_cpp
    assert "amcl_seed_response_ok" in api_observability_cpp
    assert "amcl_nomotion_pose_received" in api_observability_cpp
    assert "amcl_static_standby" in api_observability_cpp
    assert "amcl_correction_ready" in api_observability_cpp
    assert "amcl_correction_pending" in api_observability_cpp
    assert "localization_degraded" in api_observability_cpp
    assert "localization_degraded_reason" in api_observability_cpp
    assert "using_triggered_baseline_only" in api_observability_cpp
    assert "amcl_scan_admission_status_publisher_count" in api_observability_cpp

    assert "tf_broadcast: false" in amcl_cfg
    assert "scan_topic: /scan_amcl" in amcl_cfg
    assert "map_topic: /map" in amcl_cfg
    assert "update_min_d: 0.05" in amcl_cfg
    assert "update_min_a: 0.05" in amcl_cfg
    assert "robot_model_type: nav2_amcl::OmniMotionModel" in amcl_cfg
    assert "amcl_input_enabled: false" in bridge_cfg
    assert "amcl_gate_mode: shadow" in bridge_cfg
    assert "amcl_small_correction_translation_m: 0.12" in bridge_cfg
    assert "amcl_small_correction_yaw_rad: 0.20" in bridge_cfg
    assert "amcl_medium_correction_translation_m: 0.28" in bridge_cfg
    assert "amcl_medium_correction_yaw_rad: 0.35" in bridge_cfg
    assert "amcl_medium_correction_consistency_count: 3" in bridge_cfg
    assert "amcl_accept_corrections_while_moving: true" in bridge_cfg
    assert "amcl_moving_linear_speed_mps: 0.02" in bridge_cfg
    assert "amcl_moving_angular_speed_radps: 0.02" in bridge_cfg
    assert "amcl_hard_reject_translation_m: 0.60" in bridge_cfg
    assert "amcl_hard_reject_yaw_rad: 0.8" in bridge_cfg
    assert "amcl_scan_admission_status_topic: /amcl_scan_admission/status" in bridge_cfg
    assert "amcl_runtime_status_file: /tmp/njrh_amcl_runtime_status.env" in bridge_cfg
    assert "amcl_runtime_status_ttl_sec: 5.0" in bridge_cfg
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "transform_tolerance: 0.10" in nav2
    assert "MPPIController" in nav2
    assert "SmacPlanner2D" in nav2
    assert "global_frame: odom" in local_costmap_config_block(nav2)

    assert verify_path.exists()
    assert verify_contract_path.exists()
    assert verify_status_path.exists()
    assert verify_nomotion_path.exists()
    assert nomotion_probe_path.exists()
    assert observe_path.exists()
    assert "--mode disabled|shadow|gated" in verify
    assert 'MODE="${NJRH_AMCL_LOCALIZATION_MODE:-shadow}"' in verify
    assert "--seed" in verify
    assert "--check-triggered" in verify
    assert "--check-owner" in verify
    assert "tf_broadcast=false" in verify
    assert "/scan_amcl hz" in verify
    assert "AMCL is not a /tf publisher" in verify
    assert "/global_localization/trigger" in verify
    assert "ros2 action send_goal" not in verify
    assert "ros2 topic pub" not in verify
    assert "PointCloud2" not in verify
    assert "--expect-ready" in verify_contract
    assert "--expect-degraded" in verify_contract
    assert "--expect-failed" in verify_contract
    assert "--kill-amcl-for-test" in verify_contract
    assert "AMCL_READY" in verify_contract
    assert "AMCL_DEGRADED" in verify_contract
    assert "AMCL_FAILED" in verify_contract
    assert "bridge reports amcl_ready=true while runtime AMCL_READY" in verify_contract
    assert "AMCL_PID_STALE_CLEARED" in verify_contract
    assert "amcl_scan_admission_status_publisher_count" in verify_contract
    assert "amcl_nomotion_update_probe.py" in verify_nomotion
    assert "--expect-static-standby" in verify_nomotion
    assert "pose received in request window" in verify_nomotion
    assert "ros2 service call" not in verify_nomotion
    assert "create_subscription(PoseWithCovarianceStamped" in nomotion_probe
    assert "client.call_async(Empty.Request())" in nomotion_probe
    assert nomotion_probe.index("create_subscription(PoseWithCovarianceStamped") < nomotion_probe.index(
        "client.call_async(Empty.Request())"
    )
    assert "--request-nomotion-update" in verify_status
    assert "--expect-static-standby" in verify_status
    assert "amcl_status_file_stale" in verify_status
    assert "stale_file_ignored" in verify_status
    assert "pose_received_after_request" in verify_status

    assert "does not send goals" in observe
    assert "observe_navigation_tf_jitter_180s.sh" in observe
    assert "reports/amcl_navigation_shadow_${timestamp}.md" in observe
    assert "amcl_shadow_ready" in observe
    assert "amcl_gated_ready" in observe
    assert "localization_degraded" in observe
    assert "ros2 action send_goal" not in observe
    assert "ros2 topic pub" not in observe
    assert "PointCloud2" not in observe

    for text in (amcl_runner, verify, verify_contract, verify_status, verify_nomotion, observe):
        assert "pkill -9" not in text
        assert "killall -9" not in text

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            for path in (
                runtime_path,
                amcl_runner_path,
                verify_path,
                verify_contract_path,
                verify_status_path,
                verify_nomotion_path,
                observe_path,
            ):
                subprocess.run([bash, "-n", str(path)], check=True)


def test_amcl_startup_uses_one_bounded_lifecycle_client_and_heartbeat_grace():
    scripts_dir = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
    )
    runner = (scripts_dir / "run_amcl_shadow_localization.sh").read_text(
        encoding="utf-8"
    )
    lifecycle_helper = (scripts_dir / "nav2_lifecycle_sequence.py").read_text(
        encoding="utf-8"
    )
    status = (ROOT / "src/robot_bringup/src/runtime_amcl_status_server.cpp").read_text(encoding="utf-8")
    status_header = (ROOT / "src/robot_bringup/include/robot_bringup/runtime_amcl_status.hpp").read_text(encoding="utf-8")

    activate_block = runner[
        runner.index("activate_amcl_lifecycle()") :
        runner.index("amcl_warn()", runner.index("activate_amcl_lifecycle()"))
    ]
    heartbeat_block = runner[
        runner.index("heartbeat_amcl_runtime_status()") :
        runner.index("wait_for_pid_exit()", runner.index("heartbeat_amcl_runtime_status()"))
    ]

    assert 'AMCL_LIFECYCLE_HELPER="${NJRH_AMCL_LIFECYCLE_HELPER:-' in runner
    assert "nav2_lifecycle_sequence.py" in runner
    assert '"${AMCL_LIFECYCLE_HELPER}"' in activate_block
    assert "--per-node-timeout-sec" in activate_block
    assert "--change-state-response-timeout-sec" in activate_block
    assert '"/${AMCL_NODE_NAME}"' in activate_block
    assert "ros2 lifecycle get" not in activate_block
    assert "ros2 service call" not in activate_block
    assert "request_amcl_lifecycle_transition()" not in runner
    assert "amcl_lifecycle_transition_state_reached()" not in runner
    assert "rclpy.init()" in lifecycle_helper
    assert "create_client(GetState" in lifecycle_helper
    assert "create_client(ChangeState" in lifecycle_helper
    assert lifecycle_helper.count("rclpy.init()") == 1

    assert '{"starting","AMCL_STARTING"}' in status
    assert "AMCL_STARTUP_EPOCH_SEC" in runner
    assert "double startup_grace_sec{45.0}" in status_header
    assert 'result == "starting" && m - registered_at <= options.startup_grace_sec' in status
    assert 'if (!same_context) registered_at = json::monotonic_now()' in status
    assert 'result != "stopped" && result != "failed"' in status
    assert "resident AMCL startup is still in progress" in status
    assert "AMCL_HEARTBEAT_PROCESS_NOT_ALIVE" in status
    assert "heartbeat_startup_epoch" not in heartbeat_block
    assert "while" not in heartbeat_block
    assert "sleep" not in heartbeat_block
    assert "amcl_status_cli ping >/dev/null" in heartbeat_block
    assert "AMCL_PROGRESS_LIFECYCLE=true" in activate_block
    assert '--set "LIFECYCLE_VERIFIED=${AMCL_PROGRESS_LIFECYCLE:-false}"' in runner


def test_phase_a22_amcl_readiness_stale_status_and_nomotion_race_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"

    amcl_runner = (scripts_dir / "run_amcl_shadow_localization.sh").read_text(encoding="utf-8")
    amcl_status = (ROOT / "src/robot_bringup/src/runtime_amcl_status_server.cpp").read_text(encoding="utf-8")
    verify_status_path = scripts_dir / "verify_amcl_readiness_status.sh"
    verify_nomotion_path = scripts_dir / "verify_amcl_nomotion_readiness.sh"
    nomotion_probe_path = scripts_dir / "amcl_nomotion_update_probe.py"
    verify_status = verify_status_path.read_text(encoding="utf-8")
    verify_nomotion = verify_nomotion_path.read_text(encoding="utf-8")
    nomotion_probe = nomotion_probe_path.read_text(encoding="utf-8")
    bridge_cpp = (ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp").read_text(
        encoding="utf-8"
    )
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    navigation_state_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_state.cpp"
    ).read_text(encoding="utf-8")
    api_observability_cpp = "\n".join(
        (api_cpp, navigation_state_cpp, localization_configuration_source())
    )
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    api_cfg = (config_dir / "robot_api_server.yaml").read_text(encoding="utf-8")

    assert "AMCL_STATUS_STAMP_SEC" in amcl_status
    assert "AMCL_STATUS_AGE_MS" in amcl_status
    assert "AMCL_STATUS_STALE" in amcl_status
    assert "AMCL_PROCESS_READY" in amcl_status
    assert "AMCL_SEEDED" in amcl_status
    assert "AMCL_STATIC_STANDBY" in amcl_status
    assert "AMCL_TRACKING_READY" in amcl_status
    assert "AMCL_CORRECTION_READY" in amcl_status
    assert "AMCL_NOMOTION_PROBE_USED" in amcl_runner
    assert "AMCL_NOMOTION_POSE_RECEIVED" in amcl_runner
    assert "AMCL_NOMOTION_UPDATE_ACCEPT_RECEIVED_AFTER_CALL" in amcl_runner
    assert "AMCL_SEED_READINESS_DO_NOT_REQUIRE_FRESH_HEADER_WHEN_STATIC" in amcl_runner
    assert "amcl_nomotion_update_probe.py" in amcl_runner
    assert "python3 \"${NOMOTION_PROBE}\"" in amcl_runner
    assert "ros2 service call \"${service}\" std_srvs/srv/Empty" not in amcl_runner
    assert "create_subscription(PoseWithCovarianceStamped" in nomotion_probe
    assert "client.call_async(Empty.Request())" in nomotion_probe
    assert nomotion_probe.index("create_subscription(PoseWithCovarianceStamped") < nomotion_probe.index(
        "client.call_async(Empty.Request())"
    )
    assert "--require-header-fresh \"${require_header_fresh}\"" in amcl_runner
    assert "wait_for_amcl_pose_fresh && return 0" in amcl_runner
    assert "request_amcl_nomotion_update_and_wait_for_pose" in amcl_runner
    assert "request_amcl_nomotion_update || return 1" not in amcl_runner

    assert "amcl_runtime_status_ttl_sec" in bridge_cpp
    assert "amcl_accept_corrections_while_moving" in bridge_cpp
    assert "amcl_robot_moving_now" in bridge_cpp
    assert "amcl_runtime_status_authoritative" in bridge_cpp
    assert "stale_file_ignored" in bridge_cpp
    assert "amcl_status_file_stale" in bridge_cpp
    assert "amcl_status_age_ms" in bridge_cpp
    assert "amcl_status_source" in bridge_cpp
    assert "amcl_process_ready" in bridge_cpp
    assert "amcl_seeded" in bridge_cpp
    assert "amcl_seed_response_ok" in bridge_cpp
    assert "amcl_nomotion_pose_received" in bridge_cpp
    assert "amcl_static_standby" in bridge_cpp
    assert "amcl_tracking_ready" in bridge_cpp
    assert "amcl_correction_ready" in bridge_cpp
    assert "amcl_upstream_ready &&\n      amcl_tracking_ready" in bridge_cpp
    assert "amcl_gated_ready = amcl_shadow_ready && amcl_gate_mode_ == \"gated\"" not in bridge_cpp
    assert "AMCL_TRACKING_NOT_READY" in bridge_cpp
    assert "(amcl_gate_mode_ == \"gated\" && !amcl_correction_ready)" not in bridge_cpp

    assert "amcl_runtime_status_ttl_sec" in api_observability_cpp
    assert "bridge_status.amcl_correction_ready" in api_observability_cpp
    assert "bridge_status.amcl_correction_pending" in api_observability_cpp
    assert "amcl_status_file_stale" in api_observability_cpp
    assert "amcl_seed_response_ok" in api_observability_cpp
    assert "amcl_nomotion_pose_received" in api_observability_cpp
    assert "amcl_status_source" in api_observability_cpp
    assert "stale_file_ignored" in api_observability_cpp
    assert "using_triggered_baseline_only" in api_observability_cpp

    assert "amcl_runtime_status_ttl_sec: 5.0" in bridge_cfg
    assert "amcl_accept_corrections_while_moving: true" in bridge_cfg
    assert "amcl_moving_linear_speed_mps: 0.02" in bridge_cfg
    assert "amcl_moving_angular_speed_radps: 0.02" in bridge_cfg
    assert "amcl_runtime_status_ttl_sec: 5.0" in api_cfg
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg

    assert verify_status_path.exists()
    assert verify_nomotion_path.exists()
    assert nomotion_probe_path.exists()
    assert "--request-nomotion-update" in verify_status
    assert "--expect-static-standby" in verify_nomotion
    assert "amcl_nomotion_update_probe.py" in verify_nomotion
    assert "pose received in request window" in verify_nomotion
    assert "pose_received_after_request" in verify_status
    assert "amcl_gated_ready=true with /amcl_pose publisher_count=0" in verify_status
    assert "ros2 action send_goal" not in verify_status
    assert "ros2 topic pub" not in verify_status
    assert "PointCloud2" not in verify_status
    assert "pkill -9" not in verify_status
    assert "killall -9" not in verify_status


def test_phase_c1_nav2_controller_cpu_profile_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"

    affinity_cfg = (config_dir / "cpu_affinity.env").read_text(encoding="utf-8")
    affinity_helper = (scripts_dir / "cpu_affinity.sh").read_text(encoding="utf-8")
    nav_runner_path = scripts_dir / "run_nav2_navigation.sh"
    nav_runner = nav_runner_path.read_text(encoding="utf-8")
    inspect_path = scripts_dir / "inspect_nav2_controller_threads.sh"
    observe_path = scripts_dir / "observe_controller_tf_backlog_180s.sh"
    ab_path = scripts_dir / "run_nav2_controller_cpu_ab.sh"
    cleanup_path = scripts_dir / "cleanup_stale_ros2_cli.sh"
    inspect = inspect_path.read_text(encoding="utf-8")
    observe = observe_path.read_text(encoding="utf-8")
    ab = ab_path.read_text(encoding="utf-8")
    cleanup = cleanup_path.read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    amcl_cfg = (config_dir / "amcl_shadow.yaml").read_text(encoding="utf-8")

    assert 'export NJRH_NAV2_CONTROLLER_CPU_PROFILE="${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current}"' in affinity_cfg
    assert 'export NJRH_CPUSET_NAV2_CONTROLLER_CURRENT="${NJRH_CPUSET_NAV2_CONTROLLER_CURRENT:-3}"' in affinity_cfg
    assert 'export NJRH_CPUSET_NAV2_CONTROLLER_WIDE="${NJRH_CPUSET_NAV2_CONTROLLER_WIDE:-3,5}"' in affinity_cfg
    assert "control_wide)" in affinity_cfg
    assert 'export NJRH_CPUSET_CONTROLLER_SERVER="${NJRH_CPUSET_NAV2_CONTROLLER_CURRENT}"' in affinity_cfg
    assert 'export NJRH_CPUSET_CONTROLLER_SERVER="${NJRH_CPUSET_NAV2_CONTROLLER_WIDE}"' in affinity_cfg
    assert 'export NJRH_CPUSET_LOCAL_COSTMAP="${NJRH_CPUSET_LOCAL_COSTMAP:-${NJRH_CPUSET_CONTROLLER_SERVER}}"' in affinity_cfg
    assert "njrh_resolve_nav2_controller_cpuset_profile" in affinity_helper

    assert "Nav2 controller CPU profile=${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current}" in nav_runner
    assert "wait_for_controller_server_affinity" in nav_runner
    assert "controller_threads_match_cpuset" in nav_runner
    assert "NJRH_SKIP_PRESTART_NAV2_STOP" in nav_runner
    assert "controller_server affinity check failed" in nav_runner

    for path in (inspect_path, observe_path, ab_path, cleanup_path):
        assert path.exists()
    assert "controller_server_pid" in inspect
    assert "Cpus_allowed_list" in inspect
    assert "ps -L -p" in inspect
    assert "local_costmap_same_pid=expected_yes_controller_server_hosts_local_costmap" in inspect
    assert "robot_localization_bridge" in inspect
    assert "robot_local_state_ekf" in inspect
    assert "amcl_scan_admission_node" in inspect
    assert "hesai_accel_driver_node" in inspect

    assert "does not send navigation goals" in observe
    assert "REQUESTED_LATEST_RE" in observe
    assert "Requested time" in observe
    assert "/cmd_vel_nav_raw" in observe
    assert "/cmd_vel_nav" in observe
    assert "/cmd_vel_collision_checked" in observe
    assert "/cmd_vel_safe" in observe
    assert "/cmd_vel" in observe
    assert "tf:map->odom" in observe
    assert "tf:odom->base_link" in observe
    assert "local_costmap_message_filter_drop" in observe
    assert "PointCloud2" not in observe
    assert "ros2 action send_goal" not in observe
    assert "ros2 topic pub" not in observe

    assert "--profile current|control_wide" in ab
    assert "--restart-nav2" in ab
    assert "--apply" in ab
    assert "safe_stop_nav2_stack" in ab
    assert "NJRH_SKIP_PRESTART_NAV2_STOP=true" in ab
    assert "changed_tf_gate: no" in ab
    assert "changed_nav2_controller_or_planner_params: no" in ab
    assert "changed_pointcloud_qos_or_dds: no" in ab
    assert "changed_fastlio: no" in ab
    assert "changed_ranger_odom_or_ekf: no" in ab
    assert "pkill -9" not in ab
    assert "killall -9" not in ab
    assert "kill -KILL" not in ab

    assert "Dry-run by default" in cleanup
    assert "lifecycle/doctor/bag and tf2_echo" in cleanup
    assert "lifecycle|doctor|bag" in cleanup
    assert "tf2_echo" in cleanup
    assert "ros2 run" in cleanup
    assert "ros2 launch" in cleanup
    assert "kill -TERM" in cleanup
    assert "kill -KILL" not in cleanup
    assert "pkill -9" not in cleanup
    assert "killall -9" not in cleanup

    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "tf_future_stamp_offset_sec: 0.0" in bridge_cfg
    assert "transform_tolerance: 0.10" in nav2
    assert "transform_tolerance: 0.15" in local_costmap_config_block(nav2)
    assert "tf_filter_tolerance" not in nav2
    assert "global_frame: odom" in local_costmap_config_block(nav2)
    assert "global_frame: base_link" not in local_costmap_config_block(nav2)
    assert "MPPIController" in nav2
    assert "SmacPlanner2D" in nav2
    assert "tf_broadcast: false" in amcl_cfg

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            for path in (nav_runner_path, inspect_path, observe_path, ab_path, cleanup_path):
                subprocess.run([bash, "-n", str(path)], check=True)


def test_phase_d3_docking_framework_state_machine_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"
    config_dir = overlay / "config"

    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    docking_feature_cpp = docking_feature_source()
    docking_executor_hpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.hpp"
    ).read_text(encoding="utf-8")
    docking_executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    docking_status_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_status_module.cpp"
    ).read_text(encoding="utf-8")
    docking_runtime_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "docking_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    docking_correction_pause_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "docking_correction_pause_module.cpp"
    ).read_text(encoding="utf-8")
    pre_navigation_undock_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "docking"
        / "lifecycle" / "pre_navigation_undock_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_feature_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "navigation"
        / "navigation_feature_module.cpp"
    ).read_text(encoding="utf-8")
    predock_control_hpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.hpp"
    ).read_text(encoding="utf-8")
    predock_control_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.cpp"
    ).read_text(encoding="utf-8")
    docking_cpp = "\n".join(
        (
            api_cpp,
            docking_executor_cpp,
            docking_status_module_cpp,
            docking_runtime_cpp,
            pre_navigation_undock_cpp,
            predock_control_cpp,
            navigation_feature_cpp,
            docking_feature_cpp,
        )
    )
    job_hpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_model.hpp"
    ).read_text(
        encoding="utf-8"
    )
    job_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_model.cpp"
    ).read_text(encoding="utf-8")
    predock_policy_hpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_alignment_policy.hpp"
    ).read_text(encoding="utf-8")
    predock_policy_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_alignment_policy.cpp"
    ).read_text(encoding="utf-8")
    bridge_cpp = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (config_dir / "robot_api_server.yaml").read_text(encoding="utf-8")
    docking_cfg = (config_dir / "docking.yaml").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    verify_path = scripts_dir / "verify_docking_framework_state_machine.sh"
    observe_path = scripts_dir / "observe_docking_predock_yaw_align.sh"
    ab_path = scripts_dir / "run_docking_framework_ab.sh"
    verify = verify_path.read_text(encoding="utf-8")
    observe = observe_path.read_text(encoding="utf-8")
    ab = ab_path.read_text(encoding="utf-8")
    phase_doc = (ROOT / "docs" / "phase_d3_docking_framework_state_machine.md").read_text(encoding="utf-8")

    for phase in (
        "DOCK_REQUESTED",
        "RESOLVE_DOCK_PROFILE",
        "BEFORE_PREDOCK_RELOCALIZE",
        "BEFORE_PREDOCK_SETTLE",
        "NAV_TO_STAGING_NATIVE_NAV2",
        "STAGING_NAV2_EARLY_HANDOFF",
        "STAGING_NAV2_GOAL_SUCCEEDED",
        "PREDOCK_POSE_VERIFY",
        "PREDOCK_NATIVE_GOAL_VERIFY_FAILED",
        "PREDOCK_ALIGNMENT_DEFERRED_FOR_BRIDGE_SETTLE",
        "PREDOCK_YAW_ALIGN_RECOVERY",
        "AFTER_PREDOCK_RELOCALIZE",
        "AFTER_PREDOCK_SETTLE",
        "GS2_DOCK_DETECT",
            "FINE_DOCKING_BRIDGE_SETTLE",
            "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE",
            "PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE",
            "PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE_VERIFY",
            "PREDOCK_LATERAL_ALIGN_AFTER_BRIDGE_SETTLE_VERIFY",
            "FINE_DOCKING_ENTRY_CHECK",
        "FINE_ALIGN",
        "RESTAGE_RETRY",
    ):
        assert phase in docking_cpp
        assert phase in verify

    for symbol in (
        "PredockAlignmentPolicy::expected_staging_yaw",
        "PredockAlignmentPolicy::predock_yaw_error",
        "PredockAlignmentPolicy::contact_yaw_error",
        "PredockAlignmentPolicy::normalize_yaw_error",
        "PredockAlignmentPolicy::pose_inside_xy_handoff_window",
        "PredockAlignmentPolicy::pose_allows_staging_recovery",
        "PredockAlignmentPolicy::yaw_angles_met",
        "PredockAlignmentPolicy::staging_target_met",
        "PredockAlignmentPolicy::forward_capture_window_met",
        "PredockAlignmentPolicy::lateral_capture_allowed",
        "PredockControlModule::run_yaw_align",
        "PredockControlModule::run_lateral_align",
        "PredockAlignmentPolicy::lateral_align_yaw_gate_met",
        "PredockControlModule::ensure_lateral_alignment",
        "PredockControlModule::evaluate_fine_entry",
    ):
        assert symbol in predock_policy_cpp + predock_control_cpp

    assert "class DockingJobExecutor" in docking_executor_hpp
    assert "DockingJobExecutor::run" in docking_executor_cpp
    assert "void run_docking_job(" not in api_cpp
    assert "job_executor_->run_guarded(job_id);" in docking_feature_cpp
    assert "src/features/docking/lifecycle/docking_job_executor.cpp" in (
        ROOT / "src" / "robot_api_server" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")

    assert "class PredockAlignmentPolicy" in predock_policy_hpp
    assert "PredockAlignmentPolicy::evaluate_pose" in predock_policy_cpp
    assert "PredockAlignmentPolicy::pose_inside_handoff_window" in predock_policy_cpp
    assert "PredockAlignmentPolicy::pose_allows_staging_recovery" in predock_policy_cpp
    assert "PredockAlignmentPolicy::staging_target_met" in predock_policy_cpp
    assert "PredockAlignmentPolicy::apply_pose_verification" in predock_policy_cpp
    assert "PredockAlignmentPolicy::apply_yaw_align_result" in predock_policy_cpp
    assert "PredockAlignmentPolicy::apply_lateral_align_result" in predock_policy_cpp
    assert "PredockAlignmentPolicy::apply_fine_entry_check" in predock_policy_cpp
    assert (
        "std::unique_ptr<predock_alignment::PredockAlignmentPolicy> predock_alignment_policy_;"
        in docking_feature_cpp
    )
    assert "src/features/docking/predock_alignment/predock_alignment_policy.cpp" in (
        ROOT / "src" / "robot_api_server" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")

    assert "class PredockControlModule" in predock_control_hpp
    assert "PredockAssessment probe_handoff" in predock_control_hpp
    assert "PredockAssessment verify_staging_pose" in predock_control_hpp
    assert "bool start_fine_docking_handoff" in predock_control_hpp
    assert (
        "std::unique_ptr<predock_alignment::PredockControlModule> predock_control_;"
        in docking_feature_cpp
    )
    assert "predock_control_.start_fine_docking_handoff(job_id, job)" in docking_executor_cpp

    assert 'config_.command_topic != "/cmd_vel_docking"' in docking_runtime_cpp
    assert "config_.command_topic, command_qos" in docking_runtime_cpp
    assert "predock_yaw_align_cmd_pub_" not in api_cpp
    assert "actual_motion_mode_code == 2" in predock_control_cpp
    assert (
        "Ranger feedback did not confirm SPINNING=2 before predock yaw command timeout"
        in predock_control_cpp
    )
    assert "PREDOCK_YAW_ALIGN_MODE_SWITCH_TIMEOUT" in predock_control_cpp
    assert "PREDOCK_YAW_ALIGN_NO_CONFIRMED_PHYSICAL_SPIN" in predock_control_cpp
    assert "docking-owned yaw/lateral recovery" in docking_cpp
    assert "retrying predock native Nav2 because current pose is outside docking recovery window" in docking_cpp
    docking_configuration_cpp = docking_configuration_source()
    assert "config.job_executor.predock_early_handoff_enabled" in docking_configuration_cpp
    assert '"docking_predock_early_handoff_enabled", false' in docking_configuration_cpp
    assert "config_.predock_early_handoff_enabled &&" in docking_executor_cpp
    assert "policy_.pose_allows_staging_recovery(result.check)" in predock_control_cpp
    assert "predock yaw exceeded lateral alignment gate" in predock_control_cpp
    assert (
        "predock lateral error diverged; reversing side-slip command direction once"
        in predock_control_cpp
    )
    assert (
        "direction_multiplier * std::copysign(command_speed, check.lateral_m)"
        in predock_control_cpp
    )
    assert "closed-loop predock staging capture verify cycle" in predock_control_cpp
    assert "recapturing predock yaw in closed-loop staging cycle" in predock_control_cpp
    assert "closed-loop lateral capture cycle" in predock_control_cpp
    post_bridge_start = predock_control_cpp.index(
        'job_store_.set_phase(job_id, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE")'
    )
    fine_entry_start = predock_control_cpp.index(
        'job_store_.set_phase(job_id, "FINE_DOCKING_ENTRY_CHECK")',
        post_bridge_start,
    )
    post_bridge_block = predock_control_cpp[post_bridge_start:fine_entry_start]
    assert "ensure_lateral_alignment(" in post_bridge_block
    assert "aligning staging lateral after bridge settle with map->odom frozen" in post_bridge_block
    staging_capture_block = predock_control_cpp[
        predock_control_cpp.index("bool PredockControlModule::ensure_lateral_alignment(") :
        predock_control_cpp.index(
            "bool PredockControlModule::evaluate_fine_entry",
            predock_control_cpp.index("bool PredockControlModule::ensure_lateral_alignment("),
        )
    ]
    assert "yaw_aligned = policy_.yaw_angles_met(check);" in staging_capture_block
    assert "yaw_aligned = policy_.yaw_target_met(check);" not in staging_capture_block
    assert "bool predock_yaw_aligned = false;" in docking_executor_cpp
    assert "predock staging capture cannot fix current centerline/forward window by side-slip" in docking_cpp
    assert "forward_capture_min=" in predock_control_cpp
    assert "forward_capture_max=" in predock_control_cpp
    assert "predock staging capture exhausted closed-loop cycles" in predock_control_cpp
    assert "ports_.publish_forced_mode(config_.lateral_align_forced_mode)" in predock_control_cpp
    assert "twist.linear.y" in predock_control_cpp
    assert '"mode_controller_status_topic", "/ranger_base/status"' in navigation_configuration_source()
    assert "config_.gs2_scan_topic" in docking_runtime_cpp
    assert "docking_gs2_scan_topic_" not in api_cpp
    assert 'job_id, true, "docking_staging_alignment", pause_detail' in predock_control_cpp
    assert "correction_pause_reason" in navigation_feature_cpp
    assert "bridge_has_docking_fine_pause" in docking_correction_pause_cpp
    assert "DockingCorrectionPauseModule::release_stale_if_needed" in docking_correction_pause_cpp
    assert "release_stale_docking_fine_pause_if_needed" not in api_cpp
    assert "PredockControlModule::bridge_safe_for_fine_entry" in predock_control_cpp
    assert "ports_.bridge_safe_for_goal_start(bridge, \"docking fine docking\", base_detail)" in predock_control_cpp
    assert "AMCL_NOT_TRACKING tolerated_for_fine_docking_after_predock_verified" in predock_control_cpp
    assert 'bridge.amcl_degraded_reason == "AMCL_NOT_TRACKING"' in predock_control_cpp
    assert "!bridge.correction_active" in predock_control_cpp
    assert "!bridge.amcl_correction_pending" in predock_control_cpp
    assert "std::fabs(bridge.remaining_translation_error_m) <= 0.02" in predock_control_cpp
    assert "std::fabs(bridge.remaining_yaw_error_rad) <= 0.05" in predock_control_cpp
    assert '"pre_navigation_undock_start"' in docking_cpp
    assert '"post_undock_relocalization_before_trigger"' in docking_status_module_cpp
    assert "POST_UNDOCK_STALE_DOCKING_FINE_PAUSE" in docking_status_module_cpp
    assert "docking_job_finished_" in docking_status_module_cpp
    localization_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    assert "std_srvs::srv::SetBool" in localization_cpp

    for code in (
        "DOCK_FAILED_PREDOCK_NAV",
        "DOCK_FAILED_PREDOCK_RELOCALIZATION",
        "DOCK_FAILED_PREDOCK_SETTLE",
        "PREDOCK_NATIVE_GOAL_VERIFY_FAILED",
            "PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2",
            "PREDOCK_POSE_DRIFTED_AFTER_BRIDGE_SETTLE",
            "PREDOCK_YAW_NOT_ALIGNED_AFTER_BRIDGE_SETTLE",
        "PREDOCK_YAW_NOT_ALIGNED",
        "PREDOCK_YAW_HARD_FAIL",
        "PREDOCK_YAW_ALIGN_TIMEOUT",
        "PREDOCK_YAW_ALIGN_NO_YAW_MOTION",
        "PREDOCK_LATERAL_NOT_ALIGNED",
        "PREDOCK_LATERAL_HARD_FAIL",
        "PREDOCK_LATERAL_ALIGN_TIMEOUT",
        "PREDOCK_LATERAL_ALIGN_NO_LATERAL_MOTION",
        "PREDOCK_LATERAL_ALIGN_DIVERGING",
        "PREDOCK_LATERAL_ALIGN_OWNER_CONFLICT",
        "GS2_DOCK_DETECT_TIMEOUT",
        "DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT",
        "FINE_DOCKING_ENTRY_CONDITION_FAILED",
        "FINE_DOCKING_REJECTED_YAW_TOO_LARGE",
        "FINE_DOCKING_REJECTED_LATERAL_TOO_LARGE",
        "FINE_DOCKING_TIMEOUT",
        "FINAL_INSERTION_NO_CONTACT",
        "DOCK_FAILED_SAFETY_BLOCKED",
    ):
        assert code in docking_cpp

    for field in (
        "dock_profile_id",
        "approach_direction",
        "contact_frame",
        "sensor_frame",
        "max_retries",
        "retry_count",
        "predock_yaw_verified_by_nav2",
        "reverse_yaw_offset_applied",
        "contact_frame_available",
        "predock_forward_m",
        "predock_lateral_m",
        "predock_lateral_abs_m",
        "predock_yaw_aligned",
        "predock_lateral_aligned",
        "predock_lateral_align_attempted",
        "predock_lateral_align_failure_code",
        "fine_bridge_settle_started",
        "fine_bridge_settle_complete",
        "fine_bridge_settle_failure_code",
        "fine_entry_checked",
        "global_correction_paused",
        "pause_reason",
        "display_pose_source",
    ):
        assert field in job_hpp
        assert field in job_cpp

    assert "correction_pause_service" in bridge_cpp
    assert "GLOBAL_CORRECTION_PAUSED" in bridge_cpp
    assert "global_correction_paused" in bridge_cpp
    assert "correction_pause_service: /robot_localization_bridge/set_correction_paused" in bridge_cfg

    for key in (
        "docking_framework_state_machine_enabled: true",
        "docking_predock_early_handoff_enabled: false",
            "predock_yaw_align_enabled: true",
            "predock_yaw_align_fallback_enabled: true",
        'predock_yaw_align_cmd_topic: "/cmd_vel_docking"',
        "predock_yaw_align_require_actual_spin: true",
        "predock_lateral_align_enabled: true",
        "predock_lateral_align_target_m: 0.03",
        "predock_lateral_align_max_correction_m: 0.25",
        "predock_lateral_align_yaw_slack_rad: 0.02",
        "predock_lateral_align_divergence_epsilon_m: 0.015",
        "predock_lateral_align_divergence_count: 2",
        "predock_lateral_align_auto_reverse_on_divergence: true",
        "predock_staging_capture_max_cycles: 6",
        "predock_forward_capture_min_m: -0.40",
        "predock_forward_capture_max_m: 0.55",
        'predock_lateral_align_forced_mode_topic: "/ranger_mini3/forced_mode"',
        'predock_lateral_align_forced_mode: "side_slip"',
        "predock_lateral_align_timeout_sec: 14.0",
        "predock_lateral_align_speed_mps: 0.04",
        "fine_docking_entry_require_gs2_fresh: true",
        "fine_docking_entry_require_predock_yaw_aligned: true",
        "fine_docking_entry_max_lateral_m: 0.08",
        "docking_fine_wait_for_bridge_smoothing_enabled: true",
        "docking_fine_bridge_smoothing_wait_timeout_ms: 5000",
        "docking_pause_global_correction_during_fine: true",
        'localization_bridge_correction_pause_service: "/robot_localization_bridge/set_correction_paused"',
        'mode_controller_status_topic: "/ranger_base/status"',
    ):
        assert key in api_cfg

    assert "dock_types:" in docking_cfg
    assert "gs2_rear_charging_dock:" in docking_cfg
    assert "staging_offset_m: 0.60" in docking_cfg
    assert "approach_direction: reverse" in docking_cfg

    for path in (verify_path, observe_path, ab_path):
        assert path.exists()
    assert "ros2 topic pub" not in observe
    assert "write_dds_env_log" in observe
    assert "setsid bash -c" in observe
    assert "timeout --kill-after" in observe
    assert "start_api_poll" in observe
    assert "select_docking_state" in observe
    assert 'bash -lc \'' in observe
    assert "timeout 1 ros2 topic echo /localization/bridge_status" not in observe
    assert 'printf \',"bridge_status":null}\\n\'' in observe
    assert "matches = re.findall" in observe
    assert "transformPose" in observe
    assert "Message Filter" in observe
    assert "ActionServer" in observe
    assert "Aborting" in observe
    assert "kill -KILL" in observe
    assert "ros2 topic pub" not in ab
    assert "opennav_docking" not in docking_cpp
    assert "robot_docking_manager" in phase_doc
    assert "/cmd_vel_docking -> robot_safety" in phase_doc

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            for path in (verify_path, observe_path, ab_path):
                subprocess.run([bash, "-n", str(path)], check=True)


def test_phase_v1_navigation_docking_validation_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"
    config_dir = overlay / "config"

    script_paths = {
        "pose": scripts_dir / "observe_pose_required_navigation.sh",
        "manual_relocalization": scripts_dir / "verify_manual_relocalization_api.sh",
        "predock_trace": scripts_dir / "observe_predock_yaw_alignment_trace.sh",
        "predock_probe": scripts_dir / "run_predock_yaw_alignment_probe.sh",
        "fine_gate": scripts_dir / "verify_fine_docking_entry_gate.sh",
        "v1_runner": scripts_dir / "run_v1_navigation_docking_validation.sh",
    }
    scripts = {}
    for name, path in script_paths.items():
        assert path.exists(), name
        scripts[name] = path.read_text(encoding="utf-8")

    combined_scripts = "\n".join(scripts.values())
    for forbidden in (
        "PointCloud2",
        "/lidar_points",
        "/perception/obstacle_points",
        "ros2 action send_goal",
        "pkill -9",
        "killall -9",
    ):
        assert forbidden not in combined_scripts

    assert "Default mode is observe-only" in scripts["pose"]
    assert "never sends navigation goals" in scripts["pose"]
    assert "velocity commands, relocalization triggers" in scripts["pose"]
    assert "/api/v1/navigation/state" in scripts["pose"]
    assert "/cmd_vel_collision_checked" in scripts["pose"]
    assert "/cmd_vel_safe" in scripts["pose"]
    assert "/follow_path/_action/status" in scripts["pose"]
    assert "task_complete" in scripts["pose"]
    assert "goal_completion_policy" in scripts["pose"]
    assert "/api/v1/localization/trigger" not in scripts["pose"]

    assert "WAIT_FOR_SETTLE=false" in scripts["manual_relocalization"]
    assert "POST /api/v1/localization/trigger" in scripts["manual_relocalization"]
    assert '{"wait_for_settle": wait_for_settle}' in scripts["manual_relocalization"]
    assert "post_relocalization_settle_requested" in scripts["manual_relocalization"]
    assert "default post_relocalization_settle_requested=false" in scripts["manual_relocalization"]
    assert "/cmd_vel" not in scripts["manual_relocalization"]

    assert "Default mode is read-only" in scripts["predock_trace"]
    assert "does not send docking requests" in scripts["predock_trace"]
    assert "/api/v1/docking/state" in scripts["predock_trace"]
    assert "/cmd_vel_docking" in scripts["predock_trace"]
    assert "/cmd_vel_safe" in scripts["predock_trace"]
    assert "/ranger_mini3_mode_controller/status" in scripts["predock_trace"]
    assert "predock_yaw_aligned" in scripts["predock_trace"]
    assert "/api/v1/localization/trigger" not in scripts["predock_trace"]

    assert "APPLY_SMALL_YAW_TEST=false" in scripts["predock_probe"]
    assert "Default mode is observe-only" in scripts["predock_probe"]
    assert "publishes a bounded angular command only to" in scripts["predock_probe"]
    assert 'create_publisher(Twist, "/cmd_vel_docking", 10)' in scripts["predock_probe"]
    assert "active navigation goal is present" in scripts["predock_probe"]
    assert "active docking or undocking is present" in scripts["predock_probe"]
    assert "command_published: `false`" in scripts["predock_probe"]
    assert "POST /api/v1/localization/trigger" not in scripts["predock_probe"]

    assert "Static/read-only contract verifier" in scripts["fine_gate"]
    assert "calls_docking_start" in scripts["fine_gate"]
    assert "sends_velocity" in scripts["fine_gate"]
    assert "false" in scripts["fine_gate"]
    assert "fine_docking_entry_require_predock_yaw_aligned_ && !predock_yaw_aligned" in scripts["fine_gate"]
    assert "global_correction_pause_applied" in scripts["fine_gate"]
    assert "post_predock_settle_complete" in scripts["fine_gate"]
    assert "FINE_DOCKING_BRIDGE_SETTLE" in scripts["fine_gate"]
    assert "DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT" in scripts["fine_gate"]

    assert "OBSERVE_ONLY=true" in scripts["v1_runner"]
    assert "INCLUDE_MANUAL_RELOCALIZATION=false" in scripts["v1_runner"]
    assert "INCLUDE_PREDOCK_YAW_PROBE=false" in scripts["v1_runner"]
    assert "APPLY_SMALL_YAW_TEST=false" in scripts["v1_runner"]
    assert "verify_goal_completion_semantics.sh" in scripts["v1_runner"]
    assert "verify_docking_framework_state_machine.sh" in scripts["v1_runner"]
    assert "verify_fine_docking_entry_gate.sh" in scripts["v1_runner"]
    assert "observe_pose_required_navigation.sh" in scripts["v1_runner"]
    assert "observe_predock_yaw_alignment_trace.sh" in scripts["v1_runner"]
    assert "verify_manual_relocalization_api.sh" in scripts["v1_runner"]
    assert "run_predock_yaw_alignment_probe.sh" in scripts["v1_runner"]
    assert "allowed_to_run_full_docking_test" in scripts["v1_runner"]
    assert "runtime_report_dirs=" in scripts["v1_runner"]
    assert "PREDOCK_YAW_ALIGN_OWNER_CONFLICT" in scripts["v1_runner"]
    assert "grep -R \"PREDOCK_YAW_ALIGN_OWNER_CONFLICT\" \"${OUTPUT_DIR}\"" not in scripts["v1_runner"]

    api_cfg = (config_dir / "robot_api_server.yaml").read_text(encoding="utf-8")
    amcl_cfg = (config_dir / "amcl_shadow.yaml").read_text(encoding="utf-8")
    nav2_cfg = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    navigation_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    localization_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    docking_executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    predock_control_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "predock_alignment"
        / "predock_control_module.cpp"
    ).read_text(encoding="utf-8")
    gateway_cpp = "\n".join(
        (
            api_cpp,
            navigation_configuration_source(),
            navigation_module_cpp,
            localization_module_cpp,
            docking_executor_cpp,
            predock_control_cpp,
        )
    )
    bridge_cpp = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")

    assert 'navigation_default_goal_completion_policy: "pose_required"' in api_cfg
    assert 'navigation_delivery_point_goal_completion_policy: "pose_required"' in api_cfg
    assert 'navigation_position_only_nav2_yaw_mode: "approach_heading"' in api_cfg
    assert 'predock_yaw_align_cmd_topic: "/cmd_vel_docking"' in api_cfg
    assert "predock_lateral_align_enabled: true" in api_cfg
    assert 'predock_lateral_align_forced_mode: "side_slip"' in api_cfg
    assert "fine_docking_entry_require_predock_yaw_aligned: true" in api_cfg
    assert "docking_fine_wait_for_bridge_smoothing_enabled: true" in api_cfg
    assert "docking_fine_bridge_smoothing_wait_timeout_ms: 5000" in api_cfg
    assert "docking_pause_global_correction_during_fine: true" in api_cfg
    assert "tf_broadcast: false" in amcl_cfg
    assert "global_frame: odom" in local_costmap_config_block(nav2_cfg)
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert bridge_cpp.count("tf_broadcaster_->sendTransform(tf)") == 1
    assert "map_to_odom_ = candidate.transform" not in bridge_cpp

    assert '"navigation_default_goal_completion_policy", "pose_required"' in gateway_cpp
    assert '"navigation_delivery_point_goal_completion_policy", "pose_required"' in gateway_cpp
    assert "resolve_nav2_goal_yaw" in gateway_cpp
    assert "nav2_goal_yaw_source" in gateway_cpp
    assert "goal_completion_policy=dock_staging is reserved for /api/v1/docking/start" in gateway_cpp
    assert 'json_bool_value(body, "wait_for_settle", false)' in gateway_cpp
    assert "post_relocalization_settle_requested" in gateway_cpp
    assert "PREDOCK_YAW_ALIGN_OWNER_CONFLICT" in gateway_cpp
    assert "PREDOCK_LATERAL_ALIGN" in gateway_cpp
    assert "config_.fine_entry_require_predock_yaw_aligned && !yaw_aligned" in gateway_cpp
    fine_entry_block = predock_control_cpp[
        predock_control_cpp.index("bool PredockControlModule::evaluate_fine_entry") :
        predock_control_cpp.index("bool PredockControlModule::start_fine_docking_handoff")
    ]
    assert (
        'if (current_policy != "dock_staging" || !handoff_ready ||'
        in fine_entry_block
    )
    assert "!predock_pose_verified" not in fine_entry_block
    assert "PredockControlModule::wait_for_bridge_smoothing" in predock_control_cpp
    assert "PredockControlModule::bridge_safe_for_fine_entry" in predock_control_cpp
    assert "AMCL_NOT_TRACKING tolerated_for_fine_docking_after_predock_verified" in predock_control_cpp
    assert "FINE_DOCKING_BRIDGE_SETTLE" in predock_control_cpp
    assert "DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT" in predock_control_cpp

    audit_reports = sorted((ROOT / "reports").glob("v1_runtime_config_audit_*.md"))
    assert audit_reports
    audit = audit_reports[-1].read_text(encoding="utf-8")
    for token in (
        "Phase V1 Runtime Config Audit",
        "navigation_default_goal_completion_policy",
        "/api/v1/localization/trigger",
        "PREDOCK_YAW_ALIGN",
        "PASS",
    ):
        assert token in audit

    phase_doc = ROOT / "docs" / "phase_v1_navigation_docking_validation.md"
    assert phase_doc.exists()
    phase_doc_text = phase_doc.read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for token in (
        "Phase V1",
        "observe_pose_required_navigation.sh",
        "verify_manual_relocalization_api.sh",
        "observe_predock_yaw_alignment_trace.sh",
        "run_predock_yaw_alignment_probe.sh",
        "verify_fine_docking_entry_gate.sh",
        "run_v1_navigation_docking_validation.sh",
        "bridge `map->odom` smoothing completion",
    ):
        assert token in phase_doc_text
        assert token in readme

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            for path in script_paths.values():
                subprocess.run([bash, "-n", str(path)], check=True)


def test_phase_l2_post_relocalization_settle_barrier_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"

    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    docking_feature_cpp = docking_feature_source()
    docking_executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    docking_status_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_status_module.cpp"
    ).read_text(encoding="utf-8")
    docking_execution_cpp = "\n".join(
        (api_cpp, docking_feature_cpp, docking_executor_cpp, docking_status_module_cpp)
    )
    localization_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    settle_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "post_relocalization_settle_module.cpp"
    ).read_text(encoding="utf-8")
    localization_feature_cpp = localization_feature_source()
    api_runtime_cpp = "\n".join(
        (docking_execution_cpp, localization_cpp, localization_feature_cpp, settle_cpp)
    )
    bridge_cpp = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_cfg = (config_dir / "robot_api_server.yaml").read_text(encoding="utf-8")
    nav2_cfg = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    bridge_src_cfg = (
        ROOT / "src" / "robot_localization_bridge" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    verify_script_path = scripts_dir / "verify_post_relocalization_settle_barrier.sh"
    observe_script_path = scripts_dir / "observe_relocalization_to_next_stage.sh"
    bridge_verify_script_path = scripts_dir / "verify_bridge_map_odom_publisher.sh"
    tf_stability_script_path = scripts_dir / "observe_tf_stability_after_relocalization.sh"
    verify_script = verify_script_path.read_text(encoding="utf-8")
    observe_script = observe_script_path.read_text(encoding="utf-8")
    bridge_verify_script = bridge_verify_script_path.read_text(encoding="utf-8")
    tf_stability_script = tf_stability_script_path.read_text(encoding="utf-8")

    for cfg in (api_cfg, overlay_api_cfg):
        assert "localization_bridge_status_topic: \"/localization/bridge_status\"" in cfg
        assert "local_costmap_topic: \"/local_costmap/costmap\"" in cfg
        assert "post_relocalization_settle_enabled: true" in cfg
        assert "post_relocalization_settle_min_ms: 800" in cfg
        assert "post_relocalization_settle_max_ms: 3000" in cfg
        assert "post_relocalization_stable_tf_samples: 5" in cfg
        assert "post_relocalization_tf_sample_period_ms: 100" in cfg
        assert "post_relocalization_required_local_costmap_updates: 2" in cfg
        assert "post_relocalization_map_odom_publish_gap_warn_ms: 100.0" in cfg
        assert "post_relocalization_map_odom_publish_gap_fail_ms: 250.0" in cfg
        assert "post_relocalization_large_correction_translation_m: 0.5" in cfg
        assert "post_relocalization_large_correction_yaw_rad: 0.3" in cfg
        assert "post_relocalization_large_correction_min_ms: 1500" in cfg

    for cfg in (bridge_cfg, bridge_src_cfg):
        assert "map_odom_publish_gap_warn_ms: 100.0" in cfg
        assert "map_odom_publish_gap_fail_ms: 250.0" in cfg

    assert 'candidate.source == "isaac_triggered" && candidate.explicit_trigger' in bridge_cpp
    assert "last_explicit_relocalization_sequence_" in bridge_cpp
    assert "last_explicit_relocalization_accept_time" in bridge_cpp
    assert "localization_settle_required" in bridge_cpp
    assert "struct MapOdomState" in bridge_cpp
    assert "update_map_odom_state_from_candidate" in bridge_cpp
    assert "publish_map_to_odom_from_state" in bridge_cpp
    assert "map_odom_publisher_callback_group_" in bridge_cpp
    assert "publisher_decoupled_from_correction" in bridge_cpp
    assert "map_odom_publish_loop_hz" in bridge_cpp
    assert bridge_cpp.count("tf_broadcaster_->sendTransform(tf)") == 1
    publish_start = bridge_cpp.index("void publish_map_to_odom_from_state")
    publish_fn = bridge_cpp[
        publish_start:
        bridge_cpp.index("void refresh_state", publish_start)
    ]
    assert "tf_broadcaster_->sendTransform(tf)" in publish_fn
    refresh_start = bridge_cpp.index("void refresh_state")
    refresh_fn = bridge_cpp[
        refresh_start:
        bridge_cpp.index("double rate_since_last_status", refresh_start)
    ]
    assert "tf_broadcaster_->sendTransform(tf)" not in refresh_fn

    assert "wait_for_settle(" in localization_feature_cpp
    assert '"post_undock"' in docking_status_module_cpp
    assert '"before_predock"' in docking_execution_cpp
    assert '"after_predock"' in docking_execution_cpp
    assert '"manual_before_navigation"' in api_runtime_cpp
    assert '"nav2_goal"' in api_runtime_cpp
    assert '"fine_docking"' in api_runtime_cpp
    assert "local_costmap_message_filter_drop_count()" in localization_feature_cpp
    assert "base_to_lidar_static_tf_ready()" in localization_feature_cpp
    assert "settle_state_json()" in localization_feature_cpp
    assert "dependencies_.teleop->publish_zero_burst()" in localization_feature_cpp
    assert "publish_final_yaw_align_zero_burst()" in docking_feature_cpp
    assert "map_odom_publish_gap_ms" in settle_cpp
    assert "publisher_decoupled_from_correction" in settle_cpp
    assert "map_odom_last_published_sequence" in settle_cpp

    for code in [
        "POST_RELOCALIZATION_SETTLE_TIMEOUT",
        "POST_RELOCALIZATION_STABLE_SAMPLE_TIMEOUT",
        "POST_RELOCALIZATION_CORRECTION_ACTIVE",
        "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH",
        "POST_RELOCALIZATION_ODOM_BASE_NOT_FRESH",
        "POST_RELOCALIZATION_TF_CHAIN_UNSTABLE",
        "POST_RELOCALIZATION_LOCAL_COSTMAP_NOT_UPDATED",
        "POST_RELOCALIZATION_LOCAL_COSTMAP_TF_DROPS",
        "POST_RELOCALIZATION_MAP_ODOM_PUBLISH_GAP",
        "POST_RELOCALIZATION_MAP_ODOM_PUBLISHER_NOT_DECOUPLED",
        "POST_RELOCALIZATION_MAP_ODOM_PUBLISH_SEQUENCE_LAG",
        "POST_RELOCALIZATION_SCAN_ADMISSION_TF_ERROR",
        "POST_RELOCALIZATION_WRONG_MAP_ODOM_OWNER",
        "POST_RELOCALIZATION_SEQUENCE_MISMATCH",
        "CANCELLED_BY_APP",
    ]:
        assert code in settle_cpp

    assert verify_script_path.exists()
    assert observe_script_path.exists()
    assert bridge_verify_script_path.exists()
    assert tf_stability_script_path.exists()
    assert "never sends" in verify_script
    assert "publisher_decoupled_from_correction" in verify_script
    assert "ros2 action send_goal" not in verify_script
    assert "ros2 topic pub" not in verify_script
    assert "does not send goals or velocity" in observe_script
    assert "ros2 action send_goal" not in observe_script
    assert "ros2 topic pub" not in observe_script
    assert "decoupled map->odom publisher" in bridge_verify_script
    assert "ros2 topic echo" not in bridge_verify_script
    assert "does not subscribe to pointcloud topics" in tf_stability_script
    assert "ros2 topic echo" not in tf_stability_script
    assert "PointCloud2" not in bridge_verify_script + tf_stability_script

    assert "transform_tolerance: 0.10" in nav2_cfg
    assert "transform_tolerance: 0.15" in local_costmap_config_block(nav2_cfg)
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "MPPIController" in nav2_cfg
    assert "SmacPlanner2D" in nav2_cfg
    assert "PointCloud2" not in api_cpp
    assert "FAST-LIO" not in api_cpp
    assert "pkill -9" not in verify_script + observe_script + bridge_verify_script + tf_stability_script
    assert "killall -9" not in verify_script + observe_script + bridge_verify_script + tf_stability_script

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            subprocess.run([bash, "-n", str(verify_script_path)], check=True)
            subprocess.run([bash, "-n", str(observe_script_path)], check=True)
            subprocess.run([bash, "-n", str(bridge_verify_script_path)], check=True)
            subprocess.run([bash, "-n", str(tf_stability_script_path)], check=True)


def test_phase_u1_post_undock_settle_before_pending_nav_goal_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"

    api_cpp_path = ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp"
    api_cpp = api_cpp_path.read_text(encoding="utf-8")
    docking_status_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_status_module.cpp"
    ).read_text(encoding="utf-8")
    pre_navigation_undock_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "pre_navigation_undock_module.cpp"
    ).read_text(encoding="utf-8")
    docking_job_store_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_store.cpp"
    ).read_text(encoding="utf-8")
    navigation_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_state_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_state.cpp"
    ).read_text(encoding="utf-8")
    navigation_executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "mission"
        / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    settle_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "post_relocalization_settle_module.cpp"
    ).read_text(encoding="utf-8")
    docking_job_hpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_model.hpp"
    ).read_text(encoding="utf-8")
    docking_job_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_model.cpp"
    ).read_text(
        encoding="utf-8"
    )
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_cfg = (overlay / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    nav2_cfg = (overlay / "config" / "nav2.yaml").read_text(encoding="utf-8")
    bridge_cfg = (overlay / "config" / "localization_bridge.yaml").read_text(encoding="utf-8")
    observe_path = scripts_dir / "observe_post_undock_to_nav_goal.sh"
    observe_script = observe_path.read_text(encoding="utf-8")

    for cfg in (api_cfg, overlay_api_cfg):
        assert "post_undock_relocalization_settle_enabled: true" in cfg
        assert "post_undock_relocalization_settle_min_ms: 800" in cfg
        assert "post_undock_relocalization_settle_max_ms: 5000" in cfg
        assert "post_undock_stable_tf_samples: 2" in cfg
        assert "post_undock_tf_sample_period_ms: 100" in cfg
        assert "post_undock_required_local_costmap_updates: 2" in cfg
        assert "post_undock_reject_if_new_message_filter_drop: true" in cfg
        assert "post_undock_zero_cmd_during_settle: true" in cfg

    complete_start = docking_status_cpp.index(
        "void DockingStatusModule::Impl::complete_post_undock_relocalization"
    )
    complete_end = docking_status_cpp.index(
        "DockingStatusModule::DockingStatusModule", complete_start
    )
    complete_block = docking_status_cpp[complete_start:complete_end]
    wait_start = pre_navigation_undock_cpp.index(
        "bool PreNavigationUndockModule::wait_for_completion"
    )
    wait_block = pre_navigation_undock_cpp[wait_start:]
    goal_start = navigation_module_cpp.index("HttpResponse handle_goal")
    goal_end = navigation_module_cpp.index("HttpResponse handle_cancel", goal_start)
    goal_handler = navigation_module_cpp[goal_start:goal_end]
    barrier_start = settle_cpp.index("  PostRelocalizationSettleResult wait_for_settle(")
    barrier_end = settle_cpp.index(
        "  PostRelocalizationSettleState state_snapshot() const", barrier_start
    )
    barrier_block = settle_cpp[barrier_start:barrier_end]
    barrier_idx = complete_block.find("ports_.wait_for_relocalization_settle(")
    finish_success_idx = complete_block.find(
        'ports_.finish_docking_job_locked(\n      true,\n      "undocked"'
    )
    assert barrier_idx != -1
    assert finish_success_idx != -1
    assert barrier_idx < finish_success_idx
    assert '"post_undock"' in complete_block
    assert '"nav2_goal"' in complete_block
    assert "post_undock_failure_code_from_settle_failure" in complete_block
    assert "record_post_undock_navigation_readiness_failure_locked" in complete_block
    assert "undock succeeded; post-undock navigation readiness failed, pending Nav2 goal held" in complete_block
    assert 'ports_.finish_docking_job_locked(\n      false,\n      "failed"' not in complete_block
    assert "post_undock_relocalization_succeeded" in wait_block
    assert "post-undock navigation readiness failed, Nav2 goal not sent" in wait_block
    assert "pending_goal_released_after_post_undock_settle = true" in wait_block
    assert "is_post_undock" in barrier_block
    assert "post-undock warning only: local_costmap updates below required=" in barrier_block
    assert "post-undock warning only: local_costmap MessageFilter drop:" in barrier_block
    assert "post-undock warning only: amcl scan admission not clean:" in barrier_block
    assert "POST_RELOCALIZATION_CORRECTION_ACTIVE" in barrier_block
    assert "post-undock settle warning: stable_samples=" in barrier_block
    assert "undock_before_navigation_if_needed" not in goal_handler
    assert "async_send_goal(goal)" not in goal_handler
    assert "goal-start readiness will be checked in navigation background job" in goal_handler
    assert "ports_.run_goal_job(" in goal_handler

    pre_send_start = navigation_executor_cpp.index(
        "bool NavigationGoalExecutor::run_pre_send_sequence"
    )
    pre_send_end = navigation_executor_cpp.index(
        "void NavigationGoalExecutor::run_guarded", pre_send_start
    )
    pre_send_block = navigation_executor_cpp[pre_send_start:pre_send_end]
    assert "undock_before_navigation_if_needed" in pre_send_block
    assert pre_send_block.find("undock_before_navigation_if_needed") < pre_send_block.find(
        "wait_for_goal_start_readiness"
    )
    assert "navigation requires post-undock localization readiness before goal start" in pre_send_block

    for field in [
        "post_undock_relocalization_started",
        "post_undock_relocalization_accepted",
        "post_undock_settle_started",
        "post_undock_settle_complete",
        "post_undock_settle_failure_reason",
        "post_undock_navigation_readiness_failed",
        "post_undock_navigation_readiness_failure_code",
        "post_undock_navigation_readiness_detail",
        "pending_goal_held_for_post_undock_settle",
        "pending_goal_released_after_post_undock_settle",
        "using_triggered_baseline_only",
        "amcl_ready",
        "localization_degraded",
    ]:
        assert field in docking_job_hpp
        assert field in docking_job_cpp
        assert field in api_cpp + docking_status_cpp + docking_job_store_cpp

    for code in [
        "POST_UNDOCK_RELOCALIZATION_FAILED",
        "POST_UNDOCK_SETTLE_TIMEOUT",
        "POST_UNDOCK_STABLE_SAMPLE_TIMEOUT",
        "POST_UNDOCK_CORRECTION_ACTIVE",
        "POST_UNDOCK_MAP_ODOM_NOT_FRESH",
        "POST_UNDOCK_ODOM_BASE_NOT_FRESH",
        "POST_UNDOCK_TF_CHAIN_UNSTABLE",
        "POST_UNDOCK_LOCAL_COSTMAP_NOT_UPDATED",
        "POST_UNDOCK_LOCAL_COSTMAP_TF_DROPS",
        "POST_UNDOCK_MAP_ODOM_PUBLISH_GAP",
        "POST_UNDOCK_MAP_ODOM_PUBLISHER_NOT_DECOUPLED",
        "POST_UNDOCK_MAP_ODOM_PUBLISH_SEQUENCE_LAG",
        "POST_UNDOCK_WRONG_MAP_ODOM_OWNER",
        "POST_UNDOCK_RELOCALIZATION_SEQUENCE_MISMATCH",
    ]:
        assert code in docking_status_cpp

    assert "post_undock_settle_json_locked()" in docking_job_store_cpp
    assert '\\"post_undock_settle\\":' in navigation_state_cpp
    assert observe_path.exists()
    assert "Read-only observer for Phase U1" in observe_script
    assert "does not send goals" in observe_script
    assert "does not subscribe to heavy point clouds" in observe_script
    assert "ros2 action send_goal" not in observe_script
    assert "ros2 topic pub" not in observe_script
    assert "PointCloud2" not in observe_script
    assert "reports/post_undock_to_nav_goal_" in observe_script

    assert "transform_tolerance: 0.10" in nav2_cfg
    assert "transform_tolerance: 0.15" in local_costmap_config_block(nav2_cfg)
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "MPPIController" in nav2_cfg
    assert "SmacPlanner2D" in nav2_cfg
    assert "PointCloud2" not in api_cpp
    assert "FAST-LIO" not in api_cpp
    assert "ranger_base_node" not in complete_block
    assert "pkill -9" not in observe_script
    assert "killall -9" not in observe_script

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            subprocess.run([bash, "-n", str(observe_path)], check=True)


def test_phase_r0_r2_runtime_force_accept_reduction_and_bridge_smoothing_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"
    navigation_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_goal_executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "mission"
        / "navigation_goal_executor.cpp"
    ).read_text(encoding="utf-8")
    localization_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "localization"
        / "localization_module.cpp"
    ).read_text(encoding="utf-8")
    navigation_state_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "navigation_state.cpp"
    ).read_text(encoding="utf-8")
    navigation_bridge_wait_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "navigation"
        / "runtime"
        / "navigation_bridge_wait.cpp"
    ).read_text(encoding="utf-8")
    system_status_module_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "system_status"
        / "system_status_module.cpp"
    ).read_text(encoding="utf-8")
    composition_cpp = application_composition_source()
    api_cpp = "\n".join(
        (
            navigation_module_cpp,
            navigation_state_cpp,
            navigation_bridge_wait_cpp,
            localization_module_cpp,
            system_status_module_cpp,
        )
    )
    bridge_cpp = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_cfg = (overlay / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    bridge_cfg = (
        ROOT / "src" / "robot_localization_bridge" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    overlay_bridge_cfg = (overlay / "config" / "localization_bridge.yaml").read_text(
        encoding="utf-8"
    )
    nav2_cfg = (overlay / "config" / "nav2.yaml").read_text(encoding="utf-8")
    amcl_cfg = (overlay / "config" / "amcl_shadow.yaml").read_text(encoding="utf-8")
    phase_r3_doc = (ROOT / "docs" / "phase_r3_explicit_relocalization_fast_smoothing.md").read_text(
        encoding="utf-8"
    )
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    audit_reports = sorted(
        (ROOT / "reports").glob("runtime_force_accept_and_bridge_smoothing_audit_*.md")
    )
    assert audit_reports
    audit_report = audit_reports[-1].read_text(encoding="utf-8")
    assert "Normal-path high-risk calls" in audit_report
    assert "Required Reduction" in audit_report

    goal_start = navigation_module_cpp.index("HttpResponse handle_goal")
    goal_end = navigation_module_cpp.index("bool request_goal_cancel", goal_start)
    goal_handler = navigation_module_cpp[goal_start:goal_end]
    assert "trigger_localization_and_wait_for_result(" not in goal_handler
    assert "request_localization_bridge_force_accept(" not in goal_handler
    assert "wait_for_post_relocalization_settle_barrier(" not in goal_handler
    assert "wait_for_localization_result_after(" not in goal_handler
    assert "/global_localization/trigger" not in goal_handler
    assert 'bridge_safe_for_goal_start("navigation goal"' in navigation_goal_executor_cpp
    assert "force_relocalize is no longer executed inside normal navigation goals" in goal_handler
    assert "LOCALIZATION_RECOVERY_REQUIRED" in goal_handler

    assert "trigger_localization_and_wait_for_result(" in api_cpp
    assert "request_localization_bridge_force_accept(" not in api_cpp
    assert "global localization wrapper owns bridge force-accept" in api_cpp
    assert "/api/v1/localization/trigger" in api_cpp
    assert "handle_trigger_localization" in api_cpp

    for cfg in (api_cfg, overlay_api_cfg):
        assert "navigation_relocalize_before_goal: false" in cfg
        assert "navigation_relocalize_before_goal_required: false" in cfg
        assert "navigation_relocalize_before_goal_always: false" in cfg
        assert "docking_relocalize_before_predock: false" in cfg
        assert "docking_relocalize_after_predock: false" in cfg
        assert "docking_relocalize_after_predock_required: false" in cfg
        assert "docking_relocalize_after_fine_docking: false" in cfg
        assert "docking_relocalize_after_fine_docking_required: false" in cfg

    for token in (
        "navigation_normal_path_relocalization_enabled",
        "docking_normal_path_relocalization_enabled",
        "localization_recovery_available",
        "force_accept_allowed_in_normal_path",
        "ordinary_navigation_triggered_relocalization",
        "docking_predock_triggered_relocalization",
        "removed_redundant_gates",
        "active_runtime_mode",
        "safe_for_goal_start",
        "localization_recovery_required",
        "bridge.amcl_seeded",
        "bridge.amcl_tracking_ready",
        "!bridge.amcl_correction_pending",
        "AMCL correction pending before",
        "AMCL correction not ready before",
        "amcl_static_pending_is_standby",
        "amcl_not_moving_no_update_ok",
        "bridge.amcl_correction_pending",
        "bridge.amcl_correction_ready",
        "goal_start_detail",
        "localization_transition_active",
        "LOCALIZATION_DEGRADED",
        "LOCALIZATION_TRANSITION_ACTIVE",
    ):
        assert token in api_cpp
    assert "bridge_status_safe_for_goal_start(bridge, context, detail)" in composition_cpp
    assert 'BridgeReadinessPurpose::kGoalStart,\n    "navigation state"' in navigation_state_cpp

    assert "current_transform" in bridge_cpp
    assert "target_transform" in bridge_cpp
    assert "advance_map_odom_state_locked" in bridge_cpp
    assert "current_map_to_odom_snapshot" in bridge_cpp
    assert "safe_for_goal_start" in bridge_cpp
    assert "correction_active" in bridge_cpp
    assert "smoothing_enabled" in bridge_cpp
    assert "explicit_relocalization_fast_smoothing_enabled_" in bridge_cpp
    assert "explicit_relocalization_immediate" in bridge_cpp
    assert "explicit_relocalization_fast_max_duration_sec_" in bridge_cpp
    assert "smoothing_policy" in bridge_cpp
    assert "online_correction_requires_recovery" in bridge_cpp
    assert "large_correction_rejected_count" in bridge_cpp
    apply_start = bridge_cpp.index("void apply_candidate(")
    apply_end = bridge_cpp.index("void fill_amcl_initial_pose_covariance", apply_start)
    apply_candidate = bridge_cpp[apply_start:apply_end]
    assert "map_to_odom_ = candidate.transform" not in apply_candidate
    assert "update_map_odom_state_from_candidate(candidate, initial_lock)" in apply_candidate
    assert bridge_cpp.count("tf_broadcaster_->sendTransform(tf)") == 1
    publish_start = bridge_cpp.index("void publish_map_to_odom_from_state")
    publish_end = bridge_cpp.index("void refresh_state", publish_start)
    publish_block = bridge_cpp[publish_start:publish_end]
    assert "advance_map_odom_state_locked(map_odom_state_, wall_sec)" in publish_block
    assert "tf_broadcaster_->sendTransform(tf)" in publish_block

    for cfg in (bridge_cfg, overlay_bridge_cfg):
        assert "amcl_input_enabled" in cfg
        assert "amcl_gate_mode" in cfg
        assert "map_odom_smoothing_enabled: true" in cfg
        assert "map_odom_smoothing_publish_rate_hz: 50.0" in cfg
        assert "map_odom_smoothing_translation_rate_mps: 0.20" in cfg
        assert "map_odom_smoothing_yaw_rate_radps: 0.25" in cfg
        assert "explicit_relocalization_fast_smoothing_enabled: true" in cfg
        assert "explicit_relocalization_fast_correction_translation_m: 1.0" in cfg
        assert "explicit_relocalization_fast_correction_yaw_rad: 0.35" in cfg
        assert "explicit_relocalization_fast_max_duration_sec: 3.0" in cfg
        assert "map_odom_large_correction_requires_recovery: true" in cfg
        assert "map_odom_online_hard_reject_translation_m: 0.80" in cfg
        assert "max_odom_tf_age_ms: 100.0" in cfg

    assert "tf_broadcast: false" in amcl_cfg
    assert "transform_tolerance: 0.10" in nav2_cfg
    assert "transform_tolerance: 0.15" in local_costmap_config_block(nav2_cfg)
    assert "MPPIController" in nav2_cfg
    assert "SmacPlanner2D" in nav2_cfg
    assert "Phase R3 Explicit Relocalization Fast Smoothing" in phase_r3_doc
    assert "explicit_relocalization_fast" in phase_r3_doc
    assert "Phase R3 keeps that smoothing model" in readme

    script_names = (
        "verify_runtime_force_accept_reduction.sh",
        "verify_bridge_map_odom_smoothing.sh",
        "observe_normal_navigation_minimal_path.sh",
    )
    for script_name in script_names:
        script_path = scripts_dir / script_name
        assert script_path.exists()
        script = script_path.read_text(encoding="utf-8")
        assert "ros2 topic pub" not in script
        assert "ros2 action send_goal" not in script
        assert "pkill -9" not in script
        assert "killall -9" not in script

    bash = shutil.which("bash")
    if bash:
        bash_probe = subprocess.run(
            [bash, "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if bash_probe.returncode == 0:
            for script_name in script_names:
                subprocess.run([bash, "-n", str(scripts_dir / script_name)], check=True)


def test_phase24a_local_costmap_timestamp_audit_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    scripts_dir = overlay / "scripts"
    config_dir = overlay / "config"
    script = scripts_dir / "verify_local_costmap_observation_timestamp_root_cause.sh"
    assert script.exists()

    script_text = script.read_text(encoding="utf-8")
    accel_axis_wrapper = (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_accel_axis_node.cpp").read_text(
        encoding="utf-8"
    )
    accel_core = (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_accel_core.cpp").read_text(
        encoding="utf-8"
    )
    accel_axis = accel_axis_wrapper + "\n" + accel_core
    local_perception = (
        ROOT / "src" / "robot_local_perception" / "src" / "local_perception_node.cpp"
    ).read_text(encoding="utf-8")
    nav2 = (config_dir / "nav2.yaml").read_text(encoding="utf-8")
    accel_cfg = (config_dir / "pointcloud_accel_axis.yaml").read_text(encoding="utf-8")
    local_perception_cfg = (config_dir / "local_perception.yaml").read_text(encoding="utf-8")
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    phase_doc = (ROOT / "docs" / "phase_2_4a_local_costmap_timestamp_audit.md").read_text(encoding="utf-8")

    assert "ros2 param set" not in script_text
    assert "set_pointcloud_accel_profile" not in script_text
    assert "ros2 topic pub" not in script_text
    assert "create_publisher" not in script_text
    assert "/lidar_points" not in script_text
    assert ".yaml" not in script_text
    assert "ros2 topic echo /lidar/pointcloud_accel_status" in script_text
    assert "ros2 topic echo /perception/local_perception_status" not in script_text
    assert "header_once /scan" in script_text
    assert "ros2 topic echo \"${topic}\" --once --field header" in script_text
    assert "ros2 param get" in script_text
    assert 'export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"' in script_text
    assert "docker logs NJRH-car" in script_text
    assert "local_costmap_timestamp_audit_" in script_text
    for case_name in (
        "CASE_A_RAW_STAMP_ALREADY_OLD",
        "CASE_B_INTERNAL_BUFFER_STALE",
        "CASE_C_OUTPUT_REUSES_OLD_SOURCE_STAMP",
        "CASE_D_TF_CACHE_TIME_AHEAD",
        "CASE_E_STARTUP_TF_CACHE_WARMUP",
        "CASE_F_FRAME_MISMATCH",
        "CASE_G_UNKNOWN_NEEDS_BAG",
    ):
        assert case_name in script_text
        assert case_name in phase_doc

    for field in (
        "raw_header_age_ms",
        "latest_internal_buffer_stamp_age_ms",
        "latest_internal_buffer_update_age_ms",
        "latest_internal_buffer_seq",
            "publish_time",
            "source_stamp",
            "scan_output_header_age_ms",
            "scan_output_source_age_ms",
            "scan_output_frame_id",
        ):
            assert field in accel_axis

    assert 'declare_parameter<bool>("enabled", false)' in local_perception
    assert "robot_local_perception is retired and disabled by default" in local_perception

    assert "global_frame: odom" in nav2
    assert "global_frame: base_link" not in nav2
    assert "sensor_frame: lidar_level_link" in nav2
    assert "sensor_frame: base_link" not in nav2
    assert "tf_filter_tolerance" not in nav2
    assert "observation_persistence: 0.0" in nav2
    assert "local_worker_enabled: false" in accel_cfg
    assert "worker_local_enabled: false" in accel_cfg
    assert 'local_output_topic: ""' in accel_cfg
    assert 'nav_output_topic: ""' in accel_cfg
    assert "enabled: false" in local_perception_cfg
    assert 'input_topic: ""' in local_perception_cfg
    assert 'output_topic: ""' in local_perception_cfg
    assert 'clearing_output_topic: ""' in local_perception_cfg
    assert "input_reliable: false" in accel_cfg
    assert "output_reliable: false" in accel_cfg
    assert "input_reliable: false" in local_perception_cfg
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env
    assert "stamp=now" not in phase_doc
    assert "does not restamp clouds" in phase_doc


def test_robot_api_health_uses_process_identity_not_pgrep_command_substrings():
    common = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_common_services.sh"
    ).read_text(encoding="utf-8")
    start = common.index("verify_robot_api_server_common_health_or_exit()")
    end = common.index("\n}\n", start)
    body = common[start:end]

    query_start = common.index("query_robot_api_process_ownership()")
    query_end = common.index("\n}\n", query_start)
    query = common[query_start:query_end]
    assert "query_robot_api_process_ownership" in body
    assert "process_count_for_executable()" not in common
    assert "process_count_for_executable_with_exact_argument()" not in common
    assert "process_count_for_pattern" not in body
    assert "pgrep" not in query and "readlink" not in query
    assert "/proc/[0-9]*/exe" not in common
    assert query.count('output="$("${binary}"') == 1
    assert "NJRH_RUNTIME_PROCESS_CHECK_BIN" in query
    assert '--supervisor-exe "${bash_executable}"' in query
    assert '--supervisor-arg "${SCRIPT_DIR}/run_robot_api_server_supervised.sh"' in query
    assert (
        '--api-exe "${PROJECT_ROOT}/install/robot_api_server/lib/robot_api_server/'
        'robot_api_server_node"'
    ) in query
    assert '"${supervisor_count}" == 1 && "${node_count}" == 1' in query
    assert '"${rc}" == 50 && "${status}" == ownership_fault' in query
    assert 'if [[ "${rc}" != 50 ]]; then' in body
    assert "does not authorize complete-chain recovery" in body

    native = (ROOT / "src/robot_bringup/src/runtime_process_check.cpp").read_text(encoding="utf-8")
    assert "canonical(options.supervisor_exe)" in native
    assert "canonical(options.api_exe)" in native
    assert '::readlinkat(directory, "exe"' in native
    assert "exe == supervisor_exe && exact_argument(" in native
    assert "const bool node = exe == api_exe;" in native
    assert "cmdline.find('\\0', start)" in native
    assert "cmdline.compare(start, end - start, expected) == 0" in native
    assert "supervisors == 1 && nodes == 1" in native
    assert "return unique ? 0 : 50;" in native
    # CTest must run the behavioral proc-fixture tests against the built CLI,
    # including argv decoys, duplicate/missing owners, and observer failures.
    cmake = (ROOT / "src/robot_bringup/CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_test(NAME runtime_process_cli" in cmake
    assert "NJRH_RUNTIME_PROCESS_CHECK_BIN=$<TARGET_FILE:runtime_process_check>" in cmake
    assert "test/test_runtime_process_check.py" in cmake
    assert "wait_for_robot_api_server_common_ready()" in common
    assert "NJRH_ROBOT_API_PROCESS_READY_TIMEOUT_SEC" in common
    assert 'sleep "${NJRH_COMMON_MAIN_HEALTH_PERIOD_SEC:-5}"' in common
    start_call = common.index('start_common_process "robot_api_server"')
    readiness_call = common.index("wait_for_robot_api_server_common_ready", start_call)
    ready_stage = common.index('log_common_startup_stage "robot_api_server_process_ready"', start_call)
    assert start_call < readiness_call < ready_stage
    deferred = common[common.index("update_common_deferred_startup() {"):common.index("require_can_interface_up\n")]
    assert deferred.index("common_api_http_ready") < deferred.index('log_common_startup_stage "robot_api_server_ready"')


def test_localization_layer_does_not_own_common_runtime_processes():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    localization = (scripts_dir / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")

    assert "require_common_ranger_chassis_for_localization()" in localization
    assert "ranger_chassis_liveness_ready" in localization
    assert "require_common_static_tf_for_localization()" in localization
    assert "require_common_pointcloud_for_localization()" in localization
    assert "flatscan_helper_status.env" in localization

    assert 'start_canonical_helper "ranger_chassis_localization"' not in localization
    assert 'start_canonical_helper "robot_description_static_tf_localization"' not in localization
    assert "set_pointcloud_accel_profile.sh" not in localization
    assert 'bash "${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh"' not in localization
    assert 'start_overlay_helper "pointcloud_accel_pipeline_localization"' not in localization


def test_startup_environment_and_floor_assets_are_reused_across_child_layers():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    floor_helpers = (scripts_dir / "floor_asset_helpers.sh").read_text(encoding="utf-8")
    common_services = (scripts_dir / "run_common_services.sh").read_text(encoding="utf-8")
    resident = (scripts_dir / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    localization = (scripts_dir / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    nav2 = (scripts_dir / "run_nav2_navigation.sh").read_text(encoding="utf-8")

    assert 'if [[ "${NJRH_COMMON_ENV_LOADED:-}" == "1" ]]; then' in common_env
    assert "NJRH_COMMON_ENV_LOADED=1" in common_env
    assert "export NJRH_COMMON_ENV_LOADED" not in common_env
    assert "export NJRH_COMMON_ENV_PARENT_READY=1" in common_env
    assert 'common_env_parent_ready="${NJRH_COMMON_ENV_PARENT_READY:-0}"' in common_env
    assert '[[ "${common_env_parent_ready}" != "1" ]]' in common_env
    assert common_env.index("NJRH_COMMON_ENV_LOADED") < common_env.index("configure_fastdds_interface_whitelist()")

    assert "floor_asset_context_ready_for()" in floor_helpers
    assert "resolve_floor_assets_if_needed()" in floor_helpers
    assert "export NJRH_FLOOR_ASSET_CONTEXT_READY=1" in floor_helpers
    assert 'source "${SCRIPT_DIR}/floor_asset_helpers.sh"' in common_services
    assert 'resolve_floor_assets_if_needed "${autostart_building_id}" "${autostart_floor_id}"' in common_services
    assert 'resolve_floor_assets_if_needed "${building_id}" "${floor_id}"' in resident
    assert "resolve_floor_assets_if_needed" in localization
    assert "resolve_floor_assets_if_needed" in nav2
    assert resident.index("startup_epoch_sec=") < resident.index('source "${SCRIPT_DIR}/commercial_runtime_helpers.sh"')


def test_startup_logs_are_bounded_and_lifecycle_readiness_uses_status_file():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    common_env = (scripts_dir / "common_env.sh").read_text(encoding="utf-8")
    common_services = (scripts_dir / "run_common_services.sh").read_text(encoding="utf-8")
    commercial = (scripts_dir / "commercial_runtime_helpers.sh").read_text(encoding="utf-8")
    resident = (scripts_dir / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    nav2 = (scripts_dir / "run_nav2_navigation.sh").read_text(encoding="utf-8")
    hesai_driver = (
        ROOT
        / "src"
        / "third_party"
        / "hesai_lidar_ros2_overlay"
        / "src"
        / "manager"
        / "source_driver_ros2.hpp"
    ).read_text(encoding="utf-8")

    assert "rotate_runtime_log()" in common_env
    assert "NJRH_RUNTIME_LOG_RETAIN_BYTES" in common_env
    assert 'rotate_runtime_log "${log_file}"' in common_services
    assert 'printf("%s frame:%d points:%u packet:%d' not in hesai_driver

    assert "nav2_lifecycle_ready_status_matches()" in commercial
    assert "NJRH_NAV2_LIFECYCLE_READY_STATUS_FILE" in commercial
    assert "NAV2_LIFECYCLE_READY_OWNER_PID" in commercial
    assert ".readlines()" not in commercial
    assert "write_nav2_lifecycle_ready_status" in resident
    assert "write_nav2_lifecycle_ready_status" in nav2


def test_global_localization_keeps_one_immutable_trigger_transaction():
    source = (
        ROOT / "src" / "robot_global_localization" / "src" / "global_localization_node.cpp"
    ).read_text(encoding="utf-8")
    config = (
        ROOT / "src" / "robot_global_localization" / "config" / "global_localization.yaml"
    ).read_text(encoding="utf-8")
    ignored_block = source[
        source.index("latest.force_accept_ignored_pretrigger_result_count >") :
        source.index("if (latest.rejected_result_count >", source.index("latest.force_accept_ignored_pretrigger_result_count >"))
    ]

    assert "draining pre-arm localization_result inside the same trigger transaction" in ignored_block
    assert "return false;" not in ignored_block
    assert "extend_active_deadline_for_transient_result" not in source
    assert "transient_stale_bridge_accept_timeout_sec" not in source
    assert "a new Isaac trigger is required" not in source
    assert "bridge_accept_timeout_sec: 12.0" in config
    assert "transient_stale_bridge_accept_timeout_sec" not in config

    bridge_processed = source[
        source.index("bool bridge_processed_after(") :
        source.index("bool arm_bridge_force_accept(")
    ]
    assert "bridge_explicit_trigger_accept_observed" in bridge_processed
    assert "rejected_result_count" not in bridge_processed
    assert "force_accept_ignored_pretrigger_result_count" not in bridge_processed

    arm_result_index = source.index("const bool force_accept_armed =")
    arm_failure_index = source.index("if (!force_accept_armed)", arm_result_index)
    direct_trigger_index = source.index(
        "grid_search_trigger_client_->async_send_request", arm_failure_index
    )
    assert arm_result_index < arm_failure_index < direct_trigger_index
    arm_failure_block = source[arm_failure_index:direct_trigger_index]
    assert "return;" in arm_failure_block
    arm_helper = source[
        source.index("bool arm_bridge_force_accept(") :
        source.index("bool wait_for_localization_result_or_bridge_processing(")
    ]
    assert "dispatch_state=not_dispatched" in arm_helper

    arm_index = source.index("arm_bridge_force_accept(")
    post_arm_input_index = source.index("wait_for_localizer_input_after_arm(", arm_index)
    trigger_started_index = source.index("const auto trigger_started_sec", post_arm_input_index)
    initial_result_index = source.index("const auto initial_result", trigger_started_index)
    direct_trigger_index = source.index("grid_search_trigger_client_->async_send_request", initial_result_index)
    assert arm_index < post_arm_input_index < trigger_started_index < initial_result_index < direct_trigger_index
    assert "snapshot.seq > baseline.seq" in source
    assert "snapshot.received_sec >= armed_sec" in source
    assert "snapshot.header_stamp_sec >= min_header_stamp_sec" in source
    assert "snapshot.fov_deg >= localizer_input_min_fov_deg_" in source


def test_explicit_isaac_result_requires_arm_and_uses_tf_history_instead_of_wall_age():
    bridge = (
        ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    packaged_config = (
        ROOT / "src" / "robot_localization_bridge" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    field_config = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")

    pose_callback = bridge[
        bridge.index("void on_pose(") : bridge.index("void on_amcl_scan_admission_status(")
    ]
    assert "kIgnoreUnarmed" in pose_callback
    assert "unarmed_isaac_result_ignored_count_" in pose_callback
    assert "refresh_state(\"pose\")" in pose_callback

    candidate = bridge[
        bridge.index("CandidateCorrection build_candidate(") :
        bridge.index("bool candidate_is_small(")
    ]
    assert "enforce_wall_age_limit(candidate.explicit_trigger)" in candidate
    assert "lookup_odom_base(pose.header.stamp" in candidate
    assert candidate.index("enforce_wall_age_limit(candidate.explicit_trigger)") < candidate.index(
        "lookup_odom_base(pose.header.stamp"
    )

    history_parameter_index = bridge.index(
        'declare_parameter<double>("odom_tf_history_duration_sec", 30.0)'
    )
    history_buffer_index = bridge.index(
        "tf2::durationFromSec(odom_tf_history_duration_sec_)"
    )
    assert history_parameter_index < history_buffer_index
    assert '\\"odom_tf_history_duration_sec\\":' in bridge
    for config in (packaged_config, field_config):
        assert "odom_tf_history_duration_sec: 30.0" in config


def test_global_localization_callers_retry_only_before_dispatch_and_api_does_not_prearm():
    resident = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    api = (
        ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp"
    ).read_text(encoding="utf-8")
    localization = (
        ROOT / "src" / "robot_api_server" / "src" / "features"
        / "localization" / "localization_module.cpp"
    ).read_text(encoding="utf-8")

    assert "trigger_output_reports_retryable_before_dispatch" in resident
    retry_classifier = resident[
        resident.index("trigger_output_reports_retryable_before_dispatch()") :
        resident.index("initial_localization_ready_from_bridge_after_wrapper_failure()")
    ]
    for code in (
        "LOCALIZER_OPERATION_BUSY",
        "LOCALIZER_POST_RELOAD_NOT_READY",
        "LOCALIZER_INPUT_NOT_FRESH",
        "ISAAC_SERVICE_UNAVAILABLE",
        "BRIDGE_FORCE_ACCEPT_UNAVAILABLE",
        "BRIDGE_FORCE_ACCEPT_TIMEOUT",
    ):
        assert code in retry_classifier
    for code in (
        "FRESH_LOCALIZATION_RETRY_REQUIRED",
        "LOCALIZATION_RESULT_TIMEOUT",
        "BRIDGE_ACCEPT_TIMEOUT",
        "MAP_TO_ODOM_TIMEOUT",
    ):
        assert code not in retry_classifier

    trigger_helper = localization[
        localization.index("  bool trigger_localization_and_wait_for_result(") :
        localization.index(
            "  bool request_bridge_correction_pause(",
            localization.index("  bool trigger_localization_and_wait_for_result("),
        )
    ]
    assert "request_localization_bridge_force_accept(" not in trigger_helper
    assert "global localization wrapper owns bridge force-accept" in trigger_helper


def test_global_localization_process_is_owned_by_localization_generation():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    nav_helpers = (scripts_dir / "nav_runtime_helpers.sh").read_text(encoding="utf-8")
    systemd_runtime = (
        ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh"
    ).read_text(encoding="utf-8")
    cleanup = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "stop_runtime_processes.sh"
    ).read_text(encoding="utf-8")

    global_start = nav_helpers[
        nav_helpers.index("start_overlay_helper()") :
        nav_helpers.index("cleanup_overlay_helpers()")
    ]
    assert "localization_generation_owned_helper()" in nav_helpers
    assert "starting a fresh localization generation" in global_start
    assert "disown" not in global_start
    assert "forget_overlay_helper_pid" not in nav_helpers

    assert 'source "${SCRIPT_DIR}/runtime_process_patterns.sh"' in cleanup
    process_patterns = (scripts_dir / "runtime_process_patterns.sh").read_text(encoding="utf-8")
    assert "robot_global_localization/global_localization_node" in process_patterns
    assert "/install/robot_global_localization/lib/robot_global_localization/global_localization_node" in process_patterns
    node_pattern = re.search(r'^NJRH_RUNTIME_NODE_PATTERN="([^"]+)"', process_patterns, re.M)
    assert node_pattern is not None
    for command in (
        "/install/robot_bringup/lib/robot_bringup/runtime_health_guard",
        "bash /scripts/run_runtime_health_guard.sh",
        "python3 /scripts/runtime_health_guard.py",
    ):
        assert re.search(node_pattern.group(1), command), command
    assert "/tmp/njrh_runtime_health.json" in cleanup
    assert "/tmp/njrh_nav2_lifecycle_ready.env" in cleanup


def test_localization_reuses_long_lived_health_for_ranger_admission():
    localization = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_occupancy_grid_localization.sh"
    ).read_text(encoding="utf-8")
    ranger_gate = localization[
        localization.index("require_common_ranger_chassis_for_localization()") :
        localization.index("require_common_static_tf_for_localization()")
    ]

    assert 'runtime_health_check "local_state_ready"' in ranger_gate
    assert 'runtime_health_check "local_state_topic_ready"' in ranger_gate
    assert 'canonical_helper_process_pattern "ranger_chassis"' in ranger_gate
    assert "common runtime health confirms the Ranger-to-local-state chain" in ranger_gate
    assert "ranger_chassis_liveness_ready" in ranger_gate


def test_systemd_boot_path_proves_complete_runtime_cleanup_before_restart():
    systemd_runtime = (
        ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh"
    ).read_text(encoding="utf-8")
    container = (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(encoding="utf-8")
    common = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh"
    ).read_text(encoding="utf-8")
    cleanup = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "stop_runtime_processes.sh"
    ).read_text(encoding="utf-8")
    process_patterns = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "runtime_process_patterns.sh"
    ).read_text(encoding="utf-8")

    assert "verify_container_runtime_processes_absent()" in systemd_runtime
    assert "Always run the idempotent complete-runtime sweep" in systemd_runtime
    assert "skipping container cleanup sweep" not in systemd_runtime
    assert "stop_runtime_processes.sh" in systemd_runtime
    assert "runtime_process_patterns.sh" in cleanup
    assert "NJRH_RUNTIME_ALL_PATTERN" in cleanup
    assert '"--check"' in cleanup
    assert "cleanup_failed=0" in cleanup
    assert 'exit "${cleanup_failed}"' in cleanup
    for required_owner in (
        "robot_docking_manager/docking_manager_node",
        "robot_floor_manager/floor_manager_node",
        "robot_safety/robot_safety_node",
        "__node:=controller_server",
        "__node:=behavior_server",
        "__node:=bt_navigator",
        "__node:=velocity_smoother",
        "__node:=collision_monitor",
        "run_projected_map[.]sh",
        "run_jt128_2d_mapping[.]sh",
        "jt128_slam_toolbox_mapping[.]launch[.]py",
        "slam_toolbox",
        "fastlio_mapping_odom_bridge[.]py",
        "/mapping/fastlio_odometry",
        "/tf_slam2d",
        "stop_floor_navigation[.]sh",
        "/pointcloud_perception_pipeline[.]launch[.]py([[:space:]]|$)",
        (
            "/component_container_mt([[:space:]]|$).*"
            "__node:=pointcloud_perception_pipeline([[:space:]]|$)"
        ),
        "/hesai_accel_driver_node([[:space:]]|$)",
        "/jt128_accel_driver_node([[:space:]]|$)",
        "/pointcloud_downsample_node([[:space:]]|$)",
        "__node:=pointcloud_fastlio_remap([[:space:]]|$)",
        "/smoother_server([[:space:]]|$)",
        "__node:=smoother_server([[:space:]]|$)",
        "/waypoint_follower([[:space:]]|$)",
        "__node:=waypoint_follower([[:space:]]|$)",
        "topic (echo|hz|info|pub)",
        "lifecycle (get|set)",
    ):
        assert required_owner in process_patterns
    assert "NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE:-false" in systemd_runtime
    assert "NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE:-true" in systemd_runtime
    assert 'cleanup_shell_pid="$$"' in cleanup
    assert 'cleanup_parent_pid="${PPID}"' in cleanup
    assert "$1 != cleanup_shell_pid" in cleanup
    assert "$1 != cleanup_parent_pid" in cleanup
    assert "NJRH_RUNTIME_NODE_PATTERN" not in systemd_runtime

    bash_candidates = []
    if sys.platform == "win32":
        bash_candidates.extend(
            (
                Path(r"C:\Program Files\Git\bin\bash.exe"),
                Path(r"C:\Program Files\Git\usr\bin\bash.exe"),
            )
        )
    discovered_bash = shutil.which("bash")
    if discovered_bash:
        bash_candidates.append(Path(discovered_bash))
    bash = next((candidate for candidate in bash_candidates if candidate.exists()), None)
    if bash is not None:
        match_harness = r"""
set -euo pipefail
source "$1"
matches() {
  local line="$1"
  local pattern="$2"
  awk -v pattern="${pattern}" '$0 ~ pattern {found=1} END {exit !found}' <<<"${line}"
}
matches \
  "101 /usr/bin/python3 /opt/ros/humble/bin/ros2 launch /runtime/pointcloud_perception_pipeline.launch.py pointcloud_params:=x" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"
matches \
  "102 /opt/ros/humble/lib/rclcpp_components/component_container_mt --ros-args -r __node:=pointcloud_perception_pipeline -r __ns:=/" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"
matches \
  "103 /runtime/install/hesai_ros_driver/lib/hesai_ros_driver/hesai_accel_driver_node --ros-args" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"
matches \
  "104 /runtime/install/hesai_ros_driver/lib/hesai_ros_driver/jt128_accel_driver_node --ros-args" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"
matches \
  "105 /runtime/install/robot_hesai_jt128/lib/robot_hesai_jt128/pointcloud_downsample_node --ros-args" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"
matches \
  "106 /runtime/pointcloud_axis_remap --ros-args -r __node:=pointcloud_fastlio_remap" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"
matches \
  "201 /opt/ros/humble/lib/nav2_smoother/smoother_server --ros-args -r __node:=smoother_server" \
  "${NJRH_RUNTIME_NAV2_NODE_PATTERN}"
matches \
  "202 /opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower --ros-args -r __node:=waypoint_follower" \
  "${NJRH_RUNTIME_NAV2_NODE_PATTERN}"
if matches \
  "301 /opt/ros/humble/lib/rclcpp_components/component_container_mt --ros-args -r __node:=unrelated_container" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"; then
  exit 31
fi
if matches \
  "302 /tmp/hesai_accel_driver_node_probe --ros-args" \
  "${NJRH_RUNTIME_POINTCLOUD_NODE_PATTERN}"; then
  exit 32
fi
if matches \
  "401 /tmp/waypoint_follower_test --ros-args" \
  "${NJRH_RUNTIME_NAV2_NODE_PATTERN}"; then
  exit 41
fi
"""
        match_result = subprocess.run(
            [
                str(bash),
                "-c",
                match_harness,
                "runtime-pattern-contract",
                str(
                    ROOT
                    / "scripts"
                    / "jetson"
                    / "runtime_overlay"
                    / "scripts"
                    / "runtime_process_patterns.sh"
                ),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        assert match_result.returncode == 0, match_result.stderr

    assert "prepare_running_container_runtime_once()" in container
    assert "wait_for_and_prepare_running_container" in container
    assert 'id "${NJRH_RUNTIME_USER_CHECK}"' in container
    assert "mkdir -p /tmp/isaac_ros_nitros/graphs" in container
    run_driver = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_driver.sh"
    ).read_text(encoding="utf-8")
    pointcloud = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_pointcloud_accel_pipeline.sh"
    ).read_text(encoding="utf-8")
    assert "unset NJRH_COMMON_ENV_SETUP_DONE NJRH_COMMON_ENV_PARENT_READY" in run_driver
    assert "unset NJRH_COMMON_ENV_SETUP_DONE NJRH_COMMON_ENV_PARENT_READY" in pointcloud
    assert 'mapfile -t candidates < <(pgrep -f "${pattern}"' in common
    assert "latch_safety_stop_for_runtime_shutdown" in common
    assert "/safety/estop std_msgs/msg/Bool" in common
    assert "__node:=controller_server|__node:=behavior_server" in common
    fastlio_cleanup = common[
        common.index("stop_non_mapping_fastlio_runtime_processes()") :
        common.index("start_fastlio_common()")
    ]
    assert "for proc in /proc/[0-9]*" not in fastlio_cleanup
    canonical_helpers = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "canonical_tf_helpers.sh"
    ).read_text(encoding="utf-8")
    assert "canonical_helper_final_recheck()" in canonical_helpers
    final_recheck = canonical_helpers[
        canonical_helpers.index("canonical_helper_final_recheck()") :
        canonical_helpers.index("kill_canonical_pattern()")
    ]
    assert 'LOCAL_STATE_PROCESS_START_TIMEOUT_SEC="${LOCAL_STATE_READY_RECHECK_TIMEOUT_SEC:-12}"' in final_recheck
    assert 'canonical_helper_start_ready "${helper_name}"' in final_recheck
    resident = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")
    localizer_priority = resident[
        resident.index('NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE:-true') :
        resident.index('start_resident_navigation_layer "true" "nav2_layer_prestarted_held"')
    ]
    assert "runtime_readiness_probe localization-prestart" in localizer_priority
    assert 'log_startup_stage "localizer_service_ready_for_nav2_prestart"' in localizer_priority
    occupancy = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_occupancy_grid_localization.sh"
    ).read_text(encoding="utf-8")
    local_state_index = occupancy.index("ensure_resident_local_state_for_localization || exit 1")
    bridge_index = occupancy.index('start_canonical_helper "robot_localization_bridge"', local_state_index)
    global_index = occupancy.index('start_overlay_helper "global_localization_localization"', bridge_index)
    launch_index = occupancy.index('ros2 launch "${LAUNCH_FILE}" "${launch_args[@]}" &', global_index)
    map_lifecycle_index = occupancy.index("start_map_server_lifecycle_with_nav2_util || exit 1", launch_index)
    assert local_state_index < bridge_index < global_index < launch_index < map_lifecycle_index


def test_systemd_boot_path_does_not_start_login_shells_in_running_container():
    systemd_runtime = (
        ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh"
    ).read_text(encoding="utf-8")
    container = (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(encoding="utf-8")

    prepare_block = container[
        container.index("prepare_running_container_runtime_once()") :
        container.index("wait_for_and_prepare_running_container()")
    ]
    run_block = systemd_runtime[
        systemd_runtime.index("  run)") :
        systemd_runtime.index("  stop)")
    ]
    assert '"${CONTAINER_NAME}" /bin/bash -c' in prepare_block
    assert '"${CONTAINER_NAME}" /bin/bash -lc' not in prepare_block
    assert '/bin/bash -c "cd \'${OVERLAY_CONTAINER}\'' in run_block
    assert '/bin/bash -lc "cd \'${OVERLAY_CONTAINER}\'' not in run_block


def test_nav2_lifecycle_lost_response_uses_short_fallback_then_state_confirmation():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    sequence = (scripts_dir / "nav2_lifecycle_sequence.py").read_text(encoding="utf-8")
    resident = (scripts_dir / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    nav2 = (scripts_dir / "run_nav2_navigation.sh").read_text(encoding="utf-8")
    localization = (scripts_dir / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")

    assert "--change-state-response-timeout-sec" in sequence
    assert 'NJRH_NAV2_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC", "5.0"' in sequence
    assert "response_deadline = min(" in sequence
    assert "State.TRANSITION_STATE_CONFIGURING" in sequence
    assert "State.TRANSITION_STATE_ACTIVATING" in sequence
    assert "wait_for_state(node, node_name, State.PRIMARY_STATE_INACTIVE" in sequence
    assert "wait_for_state(node, node_name, State.PRIMARY_STATE_ACTIVE" in sequence
    assert "os._exit(exit_code)" in sequence
    assert "node.destroy_node()" not in sequence
    assert "rclpy.shutdown()" not in sequence

    for runner in (resident, nav2, localization):
        assert "--change-state-response-timeout-sec" in runner
        assert "NJRH_NAV2_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC:-5" in runner


def test_resident_runtime_ready_context_commit_is_fail_closed():
    runtime = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_navigation_runtime_services.sh"
    ).read_text(encoding="utf-8")

    def shell_function(name: str) -> str:
        match = re.search(
            rf"(?ms)^{re.escape(name)}\(\) \{{\n.*?^\}}\n",
            runtime,
        )
        assert match is not None, name
        return match.group(0)

    commit_ready = shell_function("commit_runtime_ready_context")
    bridge_status_once = shell_function("bridge_status_once")
    context_sequence = shell_function(
        "runtime_context_explicit_relocalization_sequence"
    )
    wait_for_sequence_after = shell_function(
        "wait_for_bridge_relocalization_sequence_after"
    )
    wait_for_sequence = shell_function(
        "wait_for_bridge_explicit_relocalization_sequence"
    )
    trigger_output_sequence = shell_function(
        "trigger_output_explicit_relocalization_sequence"
    )
    on_exit = shell_function("on_exit")

    assert "wait_for_bridge_relocalization_sequence_after" in commit_ready
    assert "bridge_status_once" not in commit_ready
    assert 'NJR H_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK' not in commit_ready
    assert 'NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK:-' in commit_ready
    assert "ensure_localization_layer_alive" in commit_ready
    assert 'wait_for_fresh_tf_transform "map" "odom"' in commit_ready
    assert "using transaction-scoped sequence plus live map->odom" in commit_ready
    assert 'local expected_sequence="${NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE:-}"' in (
        commit_ready
    )
    assert "runtime_context_explicit_relocalization_sequence" in commit_ready
    assert '"$((expected_sequence - 1))"' in commit_ready
    assert '"${explicit_sequence}" != "${expected_sequence}"' in commit_ready
    assert "does not exactly match transaction sequence=" in commit_ready
    assert '"true"' in commit_ready
    assert 'data.get("confirmed") is True' in context_sequence
    assert 'data.get("state") == "ready"' in context_sequence
    assert "value > 0" in context_sequence
    assert "--qos-reliability reliable" in bridge_status_once
    assert "--qos-durability volatile" in bridge_status_once
    assert "std_msgs/msg/String" in bridge_status_once
    assert "awk '/^\\{/ {print; exit}'" in bridge_status_once
    assert 'local timeout_sec="${1:-${NJRH_RUNTIME_READY_BRIDGE_STATUS_WAIT_SEC:-15}}"' in (
        wait_for_sequence_after
    )
    assert 'bridge_status_once "${remaining}"' in wait_for_sequence_after
    assert "if (( remaining <= 0 )); then" in wait_for_sequence_after
    assert wait_for_sequence_after.index("if (( remaining <= 0 )); then") < (
        wait_for_sequence_after.index('bridge_status_once "${remaining}"')
    )
    assert "while (( SECONDS < deadline ))" in wait_for_sequence_after
    assert "explicit_sequence > minimum_sequence" in wait_for_sequence_after
    assert 'bridge_status_field "${bridge_status}" has_map_to_odom' in (
        wait_for_sequence_after
    )
    assert 'bridge_status_field "${bridge_status}" map_to_odom_publisher_owner' in (
        wait_for_sequence_after
    )
    assert (
        'wait_for_bridge_relocalization_sequence_after "${1:-15}" 0 true'
        in wait_for_sequence
    )
    assert "grep -Eo 'explicit_sequence=[1-9][0-9]*'" in trigger_output_sequence
    assert (
        'export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE="${accepted_explicit_sequence}"'
        in runtime
    )
    assert runtime.index(
        'export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE="${accepted_explicit_sequence}"'
    ) < runtime.index(
        'echo "[runtime-overlay] global localization wrapper accepted: ${trigger_output}"'
    )
    assert "captured pre-trigger bridge explicit sequence baseline=" in runtime
    assert "BRIDGE_ACCEPT_SEQUENCE_NOT_FRESH" in runtime
    assert "BRIDGE_ACCEPT_SEQUENCE_MISSING" in runtime
    assert (
        'export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE="${joined_explicit_sequence}"'
        in runtime
    )
    assert commit_ready.index('write_runtime_map_context "ready" "true" "${message}"') < (
        commit_ready.index("runtime_map_context_matches_current_floor")
    )
    assert commit_ready.index("runtime_map_context_matches_current_floor") < (
        commit_ready.index("runtime_ready=1")
    )
    assert runtime.count("runtime_ready=1") == 1
    assert runtime.count("commit_runtime_ready_context ") == 2
    assert '[[ "${stage}" != "script_start" &&' in runtime
    reuse_block = runtime[
        runtime.index("if resident_navigation_ready; then") :
        runtime.index(
            'echo "[runtime-overlay] navigation start source=',
            runtime.index("if resident_navigation_ready; then"),
        )
    ]
    assert reuse_block.index(
        'commit_runtime_ready_context "existing resident navigation context matches selected floor"'
    ) < reuse_block.index('log_startup_stage "resident_context_reused"')
    assert 'if ! write_runtime_map_context "failed" "false"' in on_exit
    assert on_exit.index('if ! write_runtime_map_context "failed" "false"') < (
        on_exit.index("cleanup")
    )

    bash_candidates = []
    if sys.platform == "win32":
        bash_candidates.extend(
            (
                Path(r"C:\Program Files\Git\bin\bash.exe"),
                Path(r"C:\Program Files\Git\usr\bin\bash.exe"),
            )
        )
    discovered_bash = shutil.which("bash")
    if discovered_bash:
        bash_candidates.append(Path(discovered_bash))
    bash = next((candidate for candidate in bash_candidates if candidate.exists()), None)
    if bash is None:
        pytest.skip("bash is required for the runtime-ready write failure injection")

    harness = f"""
set -euo pipefail
runtime_ready=0
floor_handoff_guard() {{ return 0; }}
cleanup_marker="$(mktemp)"
probe_count_file="$(mktemp)"
write_count_file="$(mktemp)"
rm -f "${{cleanup_marker}}"
printf '%s\n' "0" >"${{probe_count_file}}"
printf '%s\n' "0" >"${{write_count_file}}"
export NJRH_RUNTIME_READY_BRIDGE_STATUS_POLL_SEC=0.01
export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE="7"
bridge_status_once() {{
  local count
  count="$(cat "${{probe_count_file}}")"
  count=$((count + 1))
  printf '%s\n' "${{count}}" >"${{probe_count_file}}"
  if [[ "${{count}}" -eq 1 ]]; then
    printf '%s\n' '{{}}'
  else
    printf '%s\n' '{{"last_explicit_relocalization_sequence":7,"has_map_to_odom":true,"map_to_odom_publisher_owner":"robot_localization_bridge"}}'
  fi
}}
bridge_status_field() {{
  local status="$1"
  local field="$2"
  case "${{field}}" in
    last_explicit_relocalization_sequence)
      if [[ "${{status}}" == *'"last_explicit_relocalization_sequence":8'* ]]; then
        printf '%s\n' "8"
      elif [[ "${{status}}" == *'"last_explicit_relocalization_sequence":7'* ]]; then
        printf '%s\n' "7"
      elif [[ "${{status}}" == *'"last_explicit_relocalization_sequence":0'* ]]; then
        printf '%s\n' "0"
      else
        printf '%s\n' ""
      fi
      ;;
    has_map_to_odom)
      [[ "${{status}}" == *'"has_map_to_odom":true'* ]] && printf '%s\n' "true" || printf '%s\n' "false"
      ;;
    map_to_odom_publisher_owner)
      [[ "${{status}}" == *'"map_to_odom_publisher_owner":"robot_localization_bridge"'* ]] &&
        printf '%s\n' "robot_localization_bridge" || printf '%s\n' ""
      ;;
  esac
}}
write_runtime_map_context() {{
  local count
  count="$(cat "${{write_count_file}}")"
  printf '%s\n' "$((count + 1))" >"${{write_count_file}}"
  return 73
}}
runtime_map_context_matches_current_floor() {{
  return 0
}}
cleanup() {{
  printf '%s\n' "cleaned" >"${{cleanup_marker}}"
}}
{trigger_output_sequence}
{wait_for_sequence_after}
{wait_for_sequence}
{commit_ready}
{on_exit}

parsed_sequence="$(
  trigger_output_explicit_relocalization_sequence \
    "accepted: true bridge accepted explicit_sequence=11 current_sequence=17"
)"
[[ "${{parsed_sequence}}" == "11" ]]

set +e
commit_runtime_ready_context "injected write failure"
commit_rc=$?
set -e
[[ "${{commit_rc}}" -ne 0 ]]
[[ "${{runtime_ready}}" -eq 0 ]]
[[ "$(cat "${{probe_count_file}}")" -ge 2 ]]
[[ "$(cat "${{write_count_file}}")" -eq 1 ]]

bridge_status_once() {{
  printf '%s\n' '{{"last_explicit_relocalization_sequence":0,"has_map_to_odom":true,"map_to_odom_publisher_owner":"robot_localization_bridge"}}'
}}
export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE="7"
export NJRH_RUNTIME_READY_BRIDGE_STATUS_WAIT_SEC=1
set +e
commit_runtime_ready_context "cached sequence must not bypass live bridge" >/dev/null 2>&1
stale_commit_rc=$?
set -e
[[ "${{stale_commit_rc}}" -ne 0 ]]
[[ "$(cat "${{write_count_file}}")" -eq 1 ]]

bridge_status_once() {{
  printf '%s\n' '{{"last_explicit_relocalization_sequence":7,"has_map_to_odom":true,"map_to_odom_publisher_owner":"robot_localization_bridge"}}'
}}
set +e
wait_for_bridge_relocalization_sequence_after 1 7 true >/dev/null
same_sequence_rc=$?
set -e
[[ "${{same_sequence_rc}}" -ne 0 ]]

bridge_status_once() {{
  printf '%s\n' '{{"last_explicit_relocalization_sequence":8,"has_map_to_odom":true,"map_to_odom_publisher_owner":"robot_localization_bridge"}}'
}}
[[ "$(wait_for_bridge_relocalization_sequence_after 1 7 true)" == "8" ]]
set +e
commit_runtime_ready_context "concurrent sequence must fail closed" >/dev/null 2>&1
concurrent_commit_rc=$?
set -e
[[ "${{concurrent_commit_rc}}" -ne 0 ]]
[[ "$(cat "${{write_count_file}}")" -eq 1 ]]

set +e
(
  false
  on_exit
)
exit_rc=$?
set -e
[[ "${{exit_rc}}" -eq 1 ]]
[[ "$(cat "${{cleanup_marker}}")" == "cleaned" ]]
rm -f "${{cleanup_marker}}" "${{probe_count_file}}" "${{write_count_file}}"
"""
    result = subprocess.run(
        [str(bash), "-s"],
        input=harness,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_global_localization_trigger_process_has_bounded_os_timeout():
    scripts_dir = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
    )
    runtime = scripts_dir / "run_navigation_runtime_services.sh"
    verifier = scripts_dir / "verify_global_localization_trigger_timeout.sh"

    assert verifier.exists()
    verifier_text = verifier.read_text(encoding="utf-8")
    assert "timeout --signal=TERM --kill-after=1s 1s" in verifier_text
    assert "trap \"\" TERM; sleep 30" in verifier_text

    bash_candidates = []
    if sys.platform == "win32":
        bash_candidates.extend(
            (
                Path(r"C:\Program Files\Git\bin\bash.exe"),
                Path(r"C:\Program Files\Git\usr\bin\bash.exe"),
            )
        )
    discovered_bash = shutil.which("bash")
    if discovered_bash:
        bash_candidates.append(Path(discovered_bash))
    bash = next((candidate for candidate in bash_candidates if candidate.exists()), None)
    if bash is None:
        pytest.skip("bash is required for the global-localization process timeout regression")

    result = subprocess.run(
        [str(bash), str(verifier), str(runtime)],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr


def test_ranger_motion_tests_fail_closed_on_stale_latest_odom_feedback():
    scripts_dir = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"

    for filename in (
        "run_ranger_spin_odom_test.sh",
        "run_ranger_straight_odom_test.sh",
    ):
        script = (scripts_dir / filename).read_text(encoding="utf-8")
        assert "SingleThreadedExecutor" in script
        assert "QoSProfile(depth=1)" in script
        assert "wheel_odom_received_at" in script
        assert "wheel_odom_stale_age_" in script
        assert "--feedback-max-age-sec" in script
        assert "wheel_odom_receive_age_sec" in script
        assert "rclpy.spin_once" not in script


def test_predock_navigation_has_stable_path_and_contact_stop_contract():
    predock_bt_path = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "behavior_trees"
        / "navigate_to_predock.xml"
    )
    assert predock_bt_path.exists()
    predock_bt = predock_bt_path.read_text(encoding="utf-8")
    assert predock_bt.count("<ComputePathToPose") == 1
    assert predock_bt.count("<FollowPath") == 1
    assert "<Sequence" in predock_bt
    assert "RateController" not in predock_bt
    assert "PipelineSequence" not in predock_bt

    api_cpp = (
        ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp"
    ).read_text(encoding="utf-8")
    executor_cpp = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    api_cfg = (
        ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    overlay_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    verify = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_docking_framework_state_machine.sh"
    ).read_text(encoding="utf-8")

    docking_configuration_cpp = docking_configuration_source()
    assert '"docking_predock_behavior_tree"' in docking_configuration_cpp
    assert (
        "config.job_executor.predock_behavior_tree = node.declare_parameter<std::string>("
        in docking_configuration_cpp
    )
    assert "goal.behavior_tree = config_.predock_behavior_tree;" in executor_cpp
    for config in (api_cfg, overlay_cfg):
        assert "docking_predock_behavior_tree:" in config
        assert "/behavior_trees/navigate_to_predock.xml" in config

    loop_start = executor_cpp.index("while (result_future.wait_for(200ms)")
    loop_end = executor_cpp.index("const auto result = result_future.get();", loop_start)
    predock_wait_loop = executor_cpp[loop_start:loop_end]
    contact_guard = predock_wait_loop.index("bms_charging_contact_snapshot()")
    handoff_check = predock_wait_loop.index("predock_control_.probe_handoff(job_id, job)")
    assert contact_guard < handoff_check
    for required in (
        "contact_stable",
        "cancel_active_navigation_goal",
        "port_.publish_teleop_zero_burst",
        "wait_for_terminal_actual_stop",
        "predock_nav_contact_detected",
    ):
        assert required in predock_wait_loop
    assert 'finish_docking_job(job_id, true, "charging"' in predock_wait_loop
    for required in (
        "navigate_to_predock.xml",
        "PREDOCK_CONTACT_STOP",
        "DOCK_FAILED_PREDOCK_CONTACT_DROPPED",
        "predock BT must keep one stable path per Nav2 action attempt",
    ):
        assert required in verify


def test_rotation_shim_is_armed_once_per_navigation_goal():
    package = ROOT / "src" / "robot_nav_config"
    package_xml = (package / "package.xml").read_text(encoding="utf-8")
    plugin_xml_path = package / "robot_nav_config_controller_plugins.xml"
    plugin_header_path = (
        package
        / "include"
        / "robot_nav_config"
        / "goal_scoped_rotation_shim_controller.hpp"
    )
    plugin_source_path = package / "src" / "goal_scoped_rotation_shim_controller.cpp"

    assert plugin_xml_path.exists()
    assert plugin_header_path.exists()
    assert plugin_source_path.exists()
    assert "<build_type>ament_cmake</build_type>" in package_xml

    plugin_xml = plugin_xml_path.read_text(encoding="utf-8")
    plugin_header = plugin_header_path.read_text(encoding="utf-8")
    plugin_source = plugin_source_path.read_text(encoding="utf-8")
    assert "robot_nav_config::GoalScopedRotationShimController" in plugin_xml
    assert "nav2_core::Controller" in plugin_xml
    assert "GoalScopedRotationShimController" in plugin_header
    assert "same_goal_replan" in plugin_source
    assert "startup_alignment_consumed" in plugin_source
    assert "limit_terminal_rotation_speed" in plugin_source
    assert "terminal_goal_yaw_error" in plugin_source
    assert "PLUGINLIB_EXPORT_CLASS" in plugin_source

    for config_path in (
        package / "config" / "nav2.yaml",
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml",
    ):
        config = config_path.read_text(encoding="utf-8")
        assert 'plugin: "robot_nav_config::GoalScopedRotationShimController"' in config
        assert "rotate_to_heading_once: true" in config
        assert "goal_change_xy_threshold: 0.01" in config
        assert "goal_change_yaw_threshold: 0.01" in config
        assert "terminal_rotation_braking_enabled: true" in config
        assert "closed_loop: true" in config

    ordinary_bt = (package / "behavior_trees" / "navigate_to_pose.xml").read_text(
        encoding="utf-8"
    )
    assert '<RateController hz="1.0">' in ordinary_bt


def test_keepout_editor_has_raster_transaction_and_runtime_effect_proof():
    api_root = ROOT / "src" / "robot_api_server"
    entry_cpp = (api_root / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    maps_cpp = (api_root / "src" / "features" / "maps" / "maps_module.cpp").read_text(
        encoding="utf-8"
    )
    maps_feature_cpp = (
        api_root / "src" / "features" / "maps" / "maps_feature_module.cpp"
    ).read_text(encoding="utf-8")
    module_cpp = (api_root / "src" / "features" / "maps" / "keepout" / "keepout_layer.cpp").read_text(
        encoding="utf-8"
    )
    module_header = (
        api_root
        / "include"
        / "robot_api_server"
        / "features"
        / "maps"
        / "keepout"
        / "keepout_layer.hpp"
    ).read_text(encoding="utf-8")
    cmake = (api_root / "CMakeLists.txt").read_text(encoding="utf-8")
    source_config = (api_root / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    overlay_nav = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_nav2_navigation.sh"
    ).read_text(encoding="utf-8")
    mask_stager = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "ensure_costmap_filter_masks.py"
    ).read_text(encoding="utf-8")

    assert "class KeepoutLayerModule" in module_header
    assert "KeepoutRuntimePort" in module_header
    assert "world_to_local" in module_cpp
    assert "map.origin[2]" in module_cpp
    assert "map.height - 1U - row_from_bottom" in module_cpp
    assert "KEEP_OUT_MASK_ALL_BLOCKED" in module_cpp
    assert "UNMANAGED_NON_NEUTRAL_MASK" in module_cpp
    assert "restore_asset_snapshots" in module_cpp
    assert "runtime_proof_complete" in module_cpp
    assert "KeepoutRuntimeMutationState" in module_header
    assert "expected_revision" in module_header
    assert "REVISION_CONFLICT" in module_cpp
    assert "KEEP_OUT_INTEGRITY_MISMATCH" in module_cpp
    assert "canonical_document_json" in module_cpp
    assert "keepout_commit.json" in module_cpp
    assert "KeepoutMaskSummary KeepoutLayerModule::inspect" in module_cpp
    assert "cleared_costmap_samples" in module_cpp
    assert "feature does not cover a free nav-map cell" in module_cpp
    assert "kMaxRasterWorkUnits" in module_cpp
    assert "::fsync(" in module_cpp
    assert "::rename(" in module_cpp
    assert "handle_save_keepout_filter" in maps_cpp
    assert "NAVIGATION_BUSY" in maps_cpp
    assert "ROBOT_NOT_STATIONARY" in maps_cpp
    assert "keepout_integrity_degraded_" in maps_cpp
    assert "snapshot.nav2_goal_active" in maps_feature_cpp
    assert "runtime_context_matches_map" in maps_cpp
    assert "keepout_mask_observation_matches" in maps_cpp
    assert "global_costmap_matches_keepout_samples" in maps_cpp
    assert "global_costmap_content_matches" in maps_cpp
    assert "costmap_samples_cleared" in maps_cpp
    assert "global_costmap_clear_client_" in maps_cpp
    assert "resolve_keepout_runtime_projection" in maps_cpp
    assert "runtime_effective" in maps_cpp
    assert "effective_on_next_activation" in maps_cpp
    assert "handle_save_keepout_filter" not in entry_cpp
    assert "src/features/maps/keepout/keepout_layer.cpp" in cmake
    assert "test_keepout_layer" in cmake
    assert "keepout_http_smoke" in cmake
    for config in (source_config, overlay_config):
        assert 'keepout_mask_load_service: "/keepout_filter_mask_server/load_map"' in config
        assert (
            'keepout_mask_get_parameters_service: '
            '"/keepout_filter_mask_server/get_parameters"'
        ) in config
        assert (
            'keepout_filter_info_state_service: '
            '"/keepout_costmap_filter_info_server/get_state"'
        ) in config
        assert 'keepout_mask_topic: "/keepout_filter_mask"' in config
        assert (
            'global_costmap_clear_service: '
            '"/global_costmap/clear_entirely_global_costmap"'
        ) in config
        assert 'global_costmap_topic: "/global_costmap/costmap"' in config
    assert "NJRH_NAV2_DISABLE_NEUTRAL_COSTMAP_FILTERS:-false" in overlay_nav
    assert "refusing to replace a selected but invalid keepout mask" in mask_stager


def test_mode_manager_is_a_resident_runtime_dependency_for_elevator_execution():
    scripts = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
    runner = (scripts / "run_mode_manager.sh").read_text(encoding="utf-8")
    common = (scripts / "run_common_services.sh").read_text(encoding="utf-8")
    nav = (scripts / "run_navigation_runtime_services.sh").read_text(
        encoding="utf-8"
    )
    helpers = (scripts / "nav_runtime_helpers.sh").read_text(encoding="utf-8")
    readiness = (scripts / "check_commercial_runtime_ready.sh").read_text(
        encoding="utf-8"
    )
    stop_runtime = (scripts / "stop_runtime_processes.sh").read_text(
        encoding="utf-8"
    )
    runtime_patterns = (scripts / "runtime_process_patterns.sh").read_text(
        encoding="utf-8"
    )
    config = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "mode_manager.yaml"
    ).read_text(encoding="utf-8")

    assert "mode_manager_node" in runner
    assert "--params-file" in runner
    assert 'start_overlay_helper "mode_manager_common"' in common
    assert 'ensure_helper_process_no_probe "mode_manager"' in nav
    assert "robot_mode_manager/mode_manager_node" in helpers
    assert 'check_node "/robot_mode_manager"' in readiness
    assert 'source "${SCRIPT_DIR}/runtime_process_patterns.sh"' in stop_runtime
    assert "robot_mode_manager/mode_manager_node" in runtime_patterns
    assert "heartbeat_period_sec: 0.20" in config


def test_mapping_is_one_deep_module_behind_the_composition_root():
    package = ROOT / "src" / "robot_api_server"
    entry_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    composition_cpp = application_composition_source()
    application_router_wiring_cpp = (
        package / "src" / "application" / "routing" / "application_router_wiring.cpp"
    ).read_text(encoding="utf-8")
    system_status_wiring_cpp = (
        package / "src" / "features" / "system_status" / "system_status_wiring.cpp"
    ).read_text(encoding="utf-8")
    mapping_header = (
        package
        / "include"
        / "robot_api_server"
        / "features"
        / "mapping"
        / "mapping_module.hpp"
    ).read_text(encoding="utf-8")
    mapping_cpp = (
        package / "src" / "features" / "mapping" / "mapping_module.cpp"
    ).read_text(encoding="utf-8")
    teleop_feature_cpp = (
        package / "src" / "features" / "teleop" / "teleop_feature_module.cpp"
    ).read_text(encoding="utf-8")
    elevator_feature_cpp = (
        package / "src" / "features" / "elevator" / "elevator_feature_module.cpp"
    ).read_text(encoding="utf-8")
    subscription_feature_cpp = (
        package
        / "src"
        / "application"
        / "subscriptions"
        / "subscription_feature_module.cpp"
    ).read_text(encoding="utf-8")

    assert "class MappingModule" in mapping_header
    assert "handle_http" in mapping_header
    assert "status_json" in mapping_header
    assert "snapshot" in mapping_header
    assert "set_live_map_page_active" in mapping_header
    assert "shutdown" in mapping_header

    for behavior in (
        "handle_start_mapping_2d",
        "handle_stop_mapping_2d",
        "handle_save_mapping_2d",
        "handle_live_mapping_2d_map_png",
        "handle_saved_mapping_2d_map_png",
        "run_mapping_start_job",
        "ensure_mapping_scan_owner_restored",
        "refresh_mapping_2d_runtime_state",
    ):
        assert behavior in mapping_cpp
        assert behavior not in entry_cpp

    assert "mapping->handle_http(request, motion_admission_epoch)" in application_router_wiring_cpp
    assert "dependencies.mapping->status_json" in system_status_wiring_cpp
    assert "std::unique_ptr<MappingFeatureModule> mapping_feature_module_;" in composition_cpp
    assert "mapping_feature_module_ = std::make_unique<MappingFeatureModule>" in composition_cpp
    assert "mapping_feature_module_->module().status_json" not in composition_cpp
    assert "dependencies_.mapping->snapshot" in teleop_feature_cpp
    assert "dependencies_.mapping->snapshot" in elevator_feature_cpp
    assert "mapping->set_live_map_page_active(active)" in subscription_feature_cpp


def test_teleop_is_one_deep_module_behind_the_composition_root():
    package = ROOT / "src" / "robot_api_server"
    entry_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    composition_cpp = application_composition_source()
    docking_feature_cpp = docking_feature_source()
    teleop_header = (
        package
        / "include"
        / "robot_api_server"
        / "features"
        / "teleop"
        / "teleop_module.hpp"
    ).read_text(encoding="utf-8")
    teleop_cpp = (
        package / "src" / "features" / "teleop" / "teleop_module.cpp"
    ).read_text(encoding="utf-8")
    teleop_feature_cpp = (
        package / "src" / "features" / "teleop" / "teleop_feature_module.cpp"
    ).read_text(encoding="utf-8")

    assert "class TeleopModule" in teleop_header
    assert "handle_socket" in teleop_header
    assert "clear_command" in teleop_header
    assert "publish_zero_burst" in teleop_header
    assert "active" in teleop_header
    assert "idle" in teleop_header

    for behavior in (
        "handle_teleop_websocket",
        "publish_teleop_command",
        "teleop_state_json",
        "on_repeat_timer",
        "teleop_session_allowed",
        "TeleopSessionState teleop_session_",
        "teleop_cmd_pub_",
        "teleop_reverse_enable_pub_",
        "teleop_repeat_timer_",
    ):
        assert behavior in teleop_cpp
        assert behavior not in entry_cpp

    assert "std::unique_ptr<TeleopFeatureModule> teleop_feature_module_;" in composition_cpp
    assert "teleop_feature_module_->handle_socket" in composition_cpp
    assert "std::unique_ptr<TeleopModule> module_;" in teleop_feature_cpp
    assert "impl_->module_->handle_socket" in teleop_feature_cpp
    assert "impl_->module_->shutdown();" in teleop_feature_cpp

    destructor = composition_cpp[
        composition_cpp.index("~Impl()") :
        composition_cpp.index("private:", composition_cpp.index("~Impl()"))
    ]
    assert destructor.index("api_gateway_module_->stop();") < destructor.index(
        "elevator_feature_module_.reset();"
    )
    assert destructor.index("elevator_feature_module_.reset();") < destructor.index(
        "docking_feature_module_->shutdown();"
    )
    assert destructor.index("docking_feature_module_->shutdown();") < destructor.index(
        "teleop_feature_module_.reset();"
    )
    assert docking_feature_cpp.index("runtime_->shutdown();") < docking_feature_cpp.index(
        "status_->shutdown();"
    )


def test_docking_http_surface_is_one_module_behind_the_composition_root():
    package = ROOT / "src" / "robot_api_server"
    node_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    module_hpp = (
        package
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_http_module.hpp"
    ).read_text(encoding="utf-8")
    module_cpp = (
        package
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_http_module.cpp"
    ).read_text(encoding="utf-8")
    application_router_wiring_cpp = (
        package / "src" / "application" / "routing" / "application_router_wiring.cpp"
    ).read_text(encoding="utf-8")
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "class DockingHttpModule" in module_hpp
    assert "DockingHttpModule::handle_http" in module_cpp
    for route in (
        "/api/v1/docking/state",
        "/api/v1/docking/start",
        "/api/v1/docking/undock",
        "/api/v1/docking/confirm_docked",
        "/api/v1/docking/clear_docked_latch",
        "/api/v1/docking/cancel",
        "/api/v1/docking/stop",
    ):
        assert route in module_cpp

    for old_handler in (
        "handle_docking_state",
        "handle_docking_start",
        "handle_docking_undock",
        "handle_docking_confirm_docked",
        "handle_docking_clear_latch",
        "handle_docking_cancel",
    ):
        assert old_handler not in node_cpp

    assert "docking->handle_http(request, motion_admission_epoch)" in application_router_wiring_cpp
    assert "src/features/docking/lifecycle/docking_http_module.cpp" in cmake
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel"' not in module_cpp
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe"' not in module_cpp


def test_docking_status_lifecycle_is_one_module_behind_the_composition_root():
    package = ROOT / "src" / "robot_api_server"
    node_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    module_hpp = (
        package
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_status_module.hpp"
    ).read_text(encoding="utf-8")
    module_cpp = (
        package
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_status_module.cpp"
    ).read_text(encoding="utf-8")
    runtime_cpp = (
        package
        / "src"
        / "features"
        / "docking"
        / "lifecycle"
        / "docking_runtime_module.cpp"
    ).read_text(encoding="utf-8")
    docking_feature_cpp = docking_feature_source()
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "class DockingStatusModule" in module_hpp
    assert "struct DockingStatusPorts" in module_hpp
    assert "DockingStatusModule::handle_status" in module_cpp
    assert "complete_post_fine_docking_relocalization" in module_cpp
    assert "complete_post_undock_relocalization" in module_cpp
    assert "ports_.post_deferred_work" in module_cpp
    assert "docking_job_finished_" in module_cpp
    assert "POST_UNDOCK_STALE_DOCKING_FINE_PAUSE" in module_cpp
    assert "post_undock_failure_code_from_settle_failure" in module_cpp
    assert "pending Nav2 goal held" in module_cpp

    assert "status_->handle_status(status)" in docking_feature_cpp
    assert "ports_.on_status(msg->data)" in runtime_cpp
    assert "status_->shutdown()" in docking_feature_cpp
    assert "DockingStatusModule>" not in node_cpp
    assert "void handle_docking_status(" not in node_cpp
    assert "complete_post_fine_docking_relocalization" not in node_cpp
    assert "complete_post_undock_relocalization" not in node_cpp
    assert "docking_relocalization_worker_" not in node_cpp
    assert "src/features/docking/lifecycle/docking_status_module.cpp" in cmake
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel"' not in module_cpp
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe"' not in module_cpp


def test_docking_job_state_has_one_canonical_store_behind_the_composition_root():
    package = ROOT / "src" / "robot_api_server"
    entry_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    composition_cpp = application_composition_source()
    lifecycle_include = (
        package / "include" / "robot_api_server" / "features" / "docking" / "lifecycle"
    )
    lifecycle_source = package / "src" / "features" / "docking" / "lifecycle"
    store_hpp = (lifecycle_include / "docking_job_store.hpp").read_text(encoding="utf-8")
    store_cpp = (lifecycle_source / "docking_job_store.cpp").read_text(encoding="utf-8")
    http_hpp = (lifecycle_include / "docking_http_module.hpp").read_text(encoding="utf-8")
    status_hpp = (lifecycle_include / "docking_status_module.hpp").read_text(encoding="utf-8")
    docking_feature_cpp = docking_feature_source()
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "class DockingJobStore" in store_hpp
    assert "mutable std::mutex mutex_;" in store_hpp
    assert "DockingJob job_;" in store_hpp
    assert "std::uint64_t sequence_{0U};" in store_hpp
    assert "DockingJobStore::finish_locked" in store_cpp
    assert "DockingJobStore::finish_with_code" in store_cpp
    assert '"docking_job_finished_" + final_state' in store_cpp
    assert '"docking_job_finished_" + code' in store_cpp

    assert "DockingJobStore & job_store" in http_hpp
    assert "DockingJobStore & job_store" in status_hpp
    assert "std::unique_ptr<DockingJobStore> job_store_;" in docking_feature_cpp
    assert "DockingJobStore>" not in entry_cpp
    assert "std::mutex docking_job_mutex_;" not in entry_cpp
    assert "DockingJob docking_job_;" not in entry_cpp
    assert "std::uint64_t docking_job_seq_" not in entry_cpp
    assert composition_cpp.index("docking_feature_module_ = std::make_unique<DockingFeatureModule>") < composition_cpp.index(
        "navigation_feature_module_ = std::make_unique<NavigationFeatureModule>"
    )
    assert "job_store_ = std::make_unique<DockingJobStore>" in docking_feature_cpp

    assert "src/features/docking/lifecycle/docking_job_store.cpp" in cmake
    assert "test_docking_job_store" in cmake
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel"' not in store_cpp
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe"' not in store_cpp


def test_predock_control_is_one_deep_module_behind_semantic_executor_calls():
    package = ROOT / "src" / "robot_api_server"
    node_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    executor_cpp = (
        package / "src" / "features" / "docking" / "lifecycle" / "docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    predock_dir = package / "src" / "features" / "docking" / "predock_alignment"
    predock_include = (
        package
        / "include"
        / "robot_api_server"
        / "features"
        / "docking"
        / "predock_alignment"
    )
    module_hpp = (predock_include / "predock_control_module.hpp").read_text(encoding="utf-8")
    module_cpp = (predock_dir / "predock_control_module.cpp").read_text(encoding="utf-8")
    docking_feature_cpp = docking_feature_source()
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    verifier = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_docking_framework_state_machine.sh"
    ).read_text(encoding="utf-8")

    assert "class PredockControlModule" in module_hpp
    for semantic_call in (
        "probe_handoff",
        "verify_staging_pose",
        "validate_current_pose_near_approach",
        "start_fine_docking_handoff",
        "stop_and_release_motion",
    ):
        assert semantic_call in module_hpp

    for owned_logic in (
        "PredockControlModule::wait_for_bridge_smoothing",
        "PredockControlModule::run_yaw_align",
        "PredockControlModule::run_lateral_align",
        "PredockControlModule::ensure_lateral_alignment",
        "PredockControlModule::evaluate_fine_entry",
        "PREDOCK_YAW_ALIGN_MODE_SWITCH_TIMEOUT",
        "predock lateral error diverged; reversing side-slip command direction once",
        "FINE_DOCKING_BRIDGE_SETTLE",
        "ports_.start_fine_docking(service_detail)",
    ):
        assert owned_logic in module_cpp

    assert "predock_control_.probe_handoff(job_id, job)" in executor_cpp
    assert "predock_control_.verify_staging_pose(job_id, job)" in executor_cpp
    assert "predock_control_.start_fine_docking_handoff(job_id, job)" in executor_cpp
    assert "PredockControlModule::run_yaw_align" not in node_cpp + executor_cpp
    assert "PredockControlModule::run_lateral_align" not in node_cpp + executor_cpp
    assert "PREDOCK_YAW_ALIGN_MODE_SWITCH_TIMEOUT" not in node_cpp + executor_cpp

    assert (
        "std::unique_ptr<predock_alignment::PredockControlModule> predock_control_;"
        in docking_feature_cpp
    )
    assert (
        "predock_control_ = std::make_unique<predock_alignment::PredockControlModule>"
        in docking_feature_cpp
    )
    assert "PredockControlModule>" not in node_cpp
    assert "src/features/docking/predock_alignment/predock_control_module.cpp" in cmake
    assert "test_predock_control_module" in cmake
    assert 'predock_control_cpp="${WORKSPACE_ROOT}/src/robot_api_server/src/features/docking/' in verifier
    assert "require_text_any" in verifier
    assert 'require_text "${predock_control_cpp}" "PredockControlModule::run_yaw_align"' in verifier
    assert "create_publisher<" not in module_cpp
    assert "create_subscription<" not in module_cpp
    assert "create_client<" not in module_cpp


def test_power_module_uniquely_owns_battery_subscription_state_and_timing():
    package = ROOT / "src" / "robot_api_server"
    entry_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    composition_cpp = application_composition_source()
    power_include = package / "include" / "robot_api_server" / "features" / "power"
    power_source = package / "src" / "features" / "power"
    module_hpp = (power_include / "power_module.hpp").read_text(encoding="utf-8")
    module_cpp = (power_source / "power_module.cpp").read_text(encoding="utf-8")
    feature_cpp = (power_source / "power_feature_module.cpp").read_text(encoding="utf-8")
    docking_feature_cpp = docking_feature_source()
    module_test = (package / "test" / "features" / "power" / "test_power_module.cpp").read_text(
        encoding="utf-8"
    )
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "class PowerModule" in module_hpp
    assert "BmsChargingContactSnapshot snapshot() const" in module_hpp
    assert "bool charging_contact_active() const" in module_hpp
    assert "std::function<void(const BatteryContactEvaluation &, bool, double)>" in module_hpp

    assert "create_subscription<sensor_msgs::msg::BatteryState>" in module_cpp
    assert "evaluate_battery_charging_contact(" in module_cpp
    assert "current.contact_stable_duration_sec >= config.contact_stable_required_sec" in module_cpp
    assert 'result.reason = "stale_bms_state"' in module_cpp
    stale_block = module_cpp[
        module_cpp.index("if (!result.fresh)") : module_cpp.index(
            "return result;", module_cpp.index("if (!result.fresh)")
        )
    ]
    assert "result.contact = false" in stale_block
    assert "result.contact_stable = false" in stale_block
    assert module_cpp.index("ports.on_contact_evidence(") < module_cpp.index(
        "ports.on_charging_contact("
    )

    assert "std::unique_ptr<PowerFeatureModule> power_feature_module_;" in composition_cpp
    assert "power_feature_module_ = std::make_unique<PowerFeatureModule>" in composition_cpp
    assert "std::unique_ptr<PowerModule> module_;" in feature_cpp
    assert "module_ = std::make_unique<PowerModule>" in feature_cpp
    assert "void handle_bms_state" not in entry_cpp
    assert "bms_state_sub_" not in entry_cpp
    for removed_raw_state in (
        "have_bms_state_",
        "have_bms_soc_",
        "latest_bms_soc_",
        "latest_bms_charging_contact_",
        "latest_bms_received_at_",
    ):
        assert removed_raw_state not in entry_cpp
    assert "return dependencies_.power->snapshot();" in docking_feature_cpp
    assert (
        "docking->contact_interlock().on_bms_contact_evidence("
        in feature_cpp
    )
    assert "teleop->on_charging_contact(contact)" in feature_cpp

    assert "OwnsCommittedContactEvidenceCallbacksAndFreshness" in module_test
    assert "callback_order[0]" in module_test
    assert "stale.contact" in module_test
    assert "src/features/power/power_module.cpp" in cmake
    assert "test_power_module" in cmake
    assert "geometry_msgs::msg::Twist" not in module_cpp
    assert "publish_estop" not in module_cpp


def test_system_status_module_owns_read_only_routes_and_floor_status_lifecycle():
    package = ROOT / "src" / "robot_api_server"
    entry_cpp = (package / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    composition_cpp = application_composition_source()
    application_router_wiring_cpp = (
        package / "src" / "application" / "routing" / "application_router_wiring.cpp"
    ).read_text(encoding="utf-8")
    module_hpp = (
        package
        / "include"
        / "robot_api_server"
        / "features"
        / "system_status"
        / "system_status_module.hpp"
    ).read_text(encoding="utf-8")
    module_cpp = (
        package / "src" / "features" / "system_status" / "system_status_module.cpp"
    ).read_text(encoding="utf-8")
    wiring_hpp = (
        package
        / "include"
        / "robot_api_server"
        / "features"
        / "system_status"
        / "system_status_wiring.hpp"
    ).read_text(encoding="utf-8")
    wiring_cpp = (
        package / "src" / "features" / "system_status" / "system_status_wiring.cpp"
    ).read_text(encoding="utf-8")
    module_test = (
        package
        / "test"
        / "features"
        / "system_status"
        / "test_system_status_module.cpp"
    ).read_text(encoding="utf-8")
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "class SystemStatusModule" in module_hpp
    assert "struct SystemStatusSnapshot" in module_hpp
    assert "struct RobotPoseRuntimeContextSnapshot" in module_hpp
    assert "std::optional<HttpResponse> handle_http" in module_hpp
    assert 'request.path == "/api/v1/status"' in module_cpp
    assert 'request.path == "/api/v1/robot/pose"' in module_cpp
    assert "HttpResponse handle_status()" in module_cpp
    assert "HttpResponse handle_robot_pose()" in module_cpp
    assert "create_subscription<std_msgs::msg::String>" in module_cpp
    assert "ensure_floor_status_subscription_active();" in module_cpp
    assert "floor_status_subscription.reset()" not in module_cpp
    assert "ports.status_snapshot()" in module_cpp
    assert "ports.wait_for_robot_pose(error)" in module_cpp
    assert "ports.robot_pose_identity()" in module_cpp
    assert "struct SystemStatusWiringDependencies" in wiring_hpp
    assert "make_system_status_module_ports" in wiring_cpp
    assert "snapshot.mapping_status_json" in wiring_cpp
    assert "snapshot.post_undock_settle_json" in wiring_cpp

    assert "std::unique_ptr<SystemStatusModule> system_status_module_;" in composition_cpp
    assert "system_status_module_ = std::make_unique<SystemStatusModule>" in composition_cpp
    assert "system_status->handle_http(request)" in application_router_wiring_cpp
    assert "SystemStatusSnapshot snapshot;" not in entry_cpp
    assert "RobotPoseRuntimeContextSnapshot snapshot;" not in entry_cpp
    assert "HttpResponse handle_status()" not in entry_cpp
    assert "HttpResponse handle_robot_pose(" not in entry_cpp
    assert "latest_floor_status_" not in entry_cpp
    assert "floor_status_sub_" not in entry_cpp
    assert "set_status_subscriptions_active" not in entry_cpp

    assert "OwnsReadOnlyRoutesAndResidentFloorStatus" in module_test
    assert "publisher->get_subscription_count()" in module_test
    assert "runtime map context is not ready" in module_test
    assert "src/features/system_status/system_status_module.cpp" in cmake
    assert "src/features/system_status/system_status_wiring.cpp" in cmake
    assert "test_system_status_module" in cmake
    assert "create_publisher<" not in module_cpp
    assert "create_client<" not in module_cpp
    assert "geometry_msgs::msg::Twist" not in module_cpp

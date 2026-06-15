import importlib.util
import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]


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
    assert 'plugin: "nav2_controller::PoseProgressChecker"' in nav2_yaml
    assert 'plugin: "nav2_controller::SimpleProgressChecker"' not in nav2_yaml
    assert "required_movement_radius: 0.10" in nav2_yaml
    assert "required_movement_angle: 0.10" in nav2_yaml
    assert "movement_time_allowance: 12.0" in nav2_yaml
    assert (
        "        obstacle_points:\n"
        "          topic: /perception/obstacle_points\n"
        "          sensor_frame: base_link\n"
    ) in local_costmap
    assert (
        "        clearing_points:\n"
        "          topic: /perception/clearing_points\n"
        "          sensor_frame: base_link\n"
    ) in local_costmap


def test_reports_exist():
    for name in (
        "car_project_reuse_report.md",
        "tf_audit_report.md",
        "third_party_resolution_report.md",
        "occupancy_builder_design.md",
    ):
        assert (ROOT / "reports" / name).exists(), name


def test_nav_defaults_are_fixed():
    nav2 = (ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml").read_text(encoding="utf-8")
    overlay_nav2 = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml").read_text(
        encoding="utf-8"
    )
    nav2_local_costmap = local_costmap_config_block(nav2)
    overlay_local_costmap = local_costmap_config_block(overlay_nav2)
    assert "SmacPlanner2D" in nav2
    assert "MPPIController" in nav2
    assert "RegulatedPurePursuitController" in nav2
    assert 'motion_model: "Ackermann"' in nav2
    assert "AckermannConstraints:" in nav2
    assert "min_turning_r: 0.81" in nav2
    assert "vx_std: 0.45" in nav2
    assert "vy_std: 0.0" in nav2
    assert "wz_std: 0.45" in nav2
    assert "vx_max: 1.20" in nav2
    assert "vx_min: 0.0" in nav2
    assert "vy_max: 0.0" in nav2
    assert "wz_max: 0.70" in nav2
    assert "desired_linear_vel: 0.80" in nav2
    assert "max_velocity: [1.20, 0.0, 1.00]" in nav2
    assert "nav2_rotation_shim_controller::RotationShimController" in nav2
    assert 'primary_controller: "nav2_mppi_controller::MPPIController"' in nav2
    assert "angular_dist_threshold: 1.20" in nav2
    assert "angular_dist_threshold: 3.20" not in nav2
    assert 'plugin: "nav2_controller::PoseProgressChecker"' in nav2
    assert 'plugin: "nav2_controller::SimpleProgressChecker"' not in nav2
    assert "required_movement_radius: 0.10" in nav2
    assert "required_movement_angle: 0.10" in nav2
    assert "movement_time_allowance: 12.0" in nav2
    assert "movement_time_allowance: 20.0" not in nav2
    assert "rotate_to_goal_heading: false" in nav2
    assert "rotate_to_goal_heading: true" not in nav2
    assert "use_rotate_to_heading: false" in nav2
    assert "/perception/obstacle_points" in nav2
    assert "/perception/clearing_points" in nav2
    assert "origin_z: -0.20" in nav2
    assert "z_voxels: 16" in nav2
    assert "min_obstacle_height: -0.20" in nav2
    assert "max_obstacle_height: 1.40" in nav2
    assert "global_frame: odom" in nav2_local_costmap
    assert "global_frame: odom" in overlay_local_costmap
    assert "global_frame: base_link" not in nav2_local_costmap
    assert "global_frame: base_link" not in overlay_local_costmap
    assert "robot_base_frame: base_link" in nav2_local_costmap
    assert "robot_base_frame: base_link" in overlay_local_costmap
    assert "sensor_frame: base_link" in nav2
    assert "sensor_frame: base_link" in overlay_nav2
    assert "clearing: true" in nav2
    assert "clearing: false" in nav2_local_costmap
    assert "clearing: false" in overlay_local_costmap
    assert "observation_persistence: 0.0" in nav2
    assert "controller_frequency: 12.0" in nav2
    assert "bt_loop_duration: 50" in nav2
    assert "default_server_timeout: 1000" in nav2
    assert "wait_for_service_timeout: 2000" in nav2
    assert "bt_loop_duration: 50" in overlay_nav2
    assert "default_server_timeout: 1000" in overlay_nav2
    assert "wait_for_service_timeout: 2000" in overlay_nav2
    assert "transform_tolerance: 0.10" in nav2
    assert "transform_tolerance: 0.15" in nav2_local_costmap
    assert "transform_tolerance: 0.15" in overlay_local_costmap
    assert "tf_filter_tolerance" not in nav2
    assert "tf_filter_tolerance" not in overlay_nav2
    assert "transform_tolerance: 0.00" not in nav2
    assert "transform_tolerance: 0.50" not in nav2
    assert "transform_tolerance: 0.5" not in nav2
    assert "speed_limit_topic: /speed_limit" in nav2
    assert "time_steps: 44" in nav2
    assert "model_dt: 0.0833333333" in nav2
    assert (
        "default_nav_to_pose_bt_xml: "
        "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/robot_nav_config/behavior_trees/navigate_to_pose.xml"
    ) in nav2
    assert (
        "default_nav_to_pose_bt_xml: "
        "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/robot_nav_config/behavior_trees/navigate_to_pose.xml"
    ) in overlay_nav2
    assert "navigate_to_pose_w_replanning_and_recovery.xml" not in nav2
    assert "navigate_to_pose_w_replanning_and_recovery.xml" not in overlay_nav2
    assert "batch_size: 1200" in nav2
    assert "width: 10" in nav2
    assert "height: 10" in nav2
    assert "obstacle_max_range: 5.50" in nav2
    assert "raytrace_max_range: 8.00" in nav2
    assert "repulsion_weight: 2.0" in nav2
    assert "critical_weight: 20.0" in nav2
    assert "collision_margin_distance: 0.08" in nav2
    assert "inflation_radius: 0.35" in nav2
    assert 'inflation_layer_name: "local_inflation_layer"' in nav2
    assert "max_path_occupancy_ratio: 0.05" in nav2
    assert "source_timeout: 1.5" in nav2
    assert "stop_pub_timeout: 0.3" in nav2
    assert "/local_state/odometry" in nav2
    assert "/cmd_vel_collision_checked" in nav2
    assert "collision_monitor" in nav2
    assert "keepout_filter_mask_server" in nav2
    assert "keepout_costmap_filter_info_server" in nav2
    assert "speed_filter_mask_server" in nav2
    assert "speed_costmap_filter_info_server" in nav2
    assert 'filters: ["keepout_filter", "speed_filter"]' in nav2
    assert 'plugin: "nav2_costmap_2d::KeepoutFilter"' in nav2
    assert "filter_info_topic: /costmap_filter_info/keepout" in nav2
    assert 'plugin: "nav2_costmap_2d::SpeedFilter"' in nav2
    assert "filter_info_topic: /costmap_filter_info/speed" in nav2
    assert "yaw_goal_tolerance: 0.15" in nav2
    assert "yaw_goal_tolerance: 0.15" in overlay_nav2
    assert "yaw_goal_tolerance: 0.05" not in nav2
    assert "yaw_goal_tolerance: 0.05" not in overlay_nav2
    assert 'plugin: "nav2_controller::PoseProgressChecker"' in overlay_nav2
    assert 'plugin: "nav2_controller::SimpleProgressChecker"' not in overlay_nav2
    assert "required_movement_angle: 0.10" in overlay_nav2
    assert "rotate_to_goal_heading: false" in overlay_nav2
    assert "rotate_to_goal_heading: true" not in overlay_nav2

    local_perception = (
        ROOT / "src" / "robot_local_perception" / "config" / "local_perception.yaml"
    ).read_text(encoding="utf-8")
    overlay_local_perception = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_perception.yaml"
    ).read_text(encoding="utf-8")
    assert "input_topic: /lidar_points\n" in local_perception
    assert "input_topic: /_internal/lidar_points_local" not in local_perception
    assert "input_topic: /jt128/vendor/points_raw" not in local_perception
    assert "input_reliable: false" in local_perception
    assert "input_qos_depth: 1" in local_perception
    assert "input_transform_use_latest: true" in local_perception
    assert "clearing.publish_every_n: 4" in local_perception
    assert "status_topic: /perception/local_perception_status" in local_perception
    assert "status_publish_period_sec: 2.0" in local_perception
    assert "input_reliable: false" in overlay_local_perception
    assert "input_qos_depth: 1" in overlay_local_perception
    assert "input_topic: /lidar_points\n" in overlay_local_perception
    assert "input_topic: /_internal/lidar_points_local" not in overlay_local_perception
    assert "input_topic: /jt128/vendor/points_raw" not in overlay_local_perception
    assert "input_transform_use_latest: true" in overlay_local_perception
    assert "clearing.publish_every_n: 4" in overlay_local_perception
    assert "status_topic: /perception/local_perception_status" in overlay_local_perception
    assert "status_publish_period_sec: 2.0" in overlay_local_perception


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
    assert "ros2 topic info -v /perception/obstacle_points" in script
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
    assert "required_movement_angle=0.10" in ab
    assert "movement_time_allowance=12.0" in ab
    assert "pointcloud, DDS/RMW, EKF, FAST-LIO2, App API" in ab
    assert "pkill -9" not in ab
    assert "kill -9" not in ab

    assert "Default mode only records existing topics" in diagnose
    assert "--execute-goal" in diagnose
    assert "--bag" in diagnose
    assert "reports/nav2_zero_linear_progress_" in diagnose
    assert "/cmd_vel_nav_raw" in diagnose
    assert "/cmd_vel_nav" in diagnose
    assert "/cmd_vel_collision_checked" in diagnose
    assert "/cmd_vel_safe" in diagnose
    assert "/cmd_vel" in diagnose
    assert "/wheel/odom" in diagnose
    assert "/local_state/odometry" in diagnose
    assert "ros2 topic hz /perception/obstacle_points" in diagnose
    assert "ros2 topic hz /local_costmap/costmap" in diagnose
    assert "CASE_A_CONTROLLER_ZERO_LINEAR" in diagnose
    assert "CASE_B_COLLISION_ZERO_LINEAR" in diagnose
    assert "CASE_C_SAFETY_ZERO_LINEAR" in diagnose
    assert "CASE_D_MODE_CONTROLLER_OR_CHASSIS_NOT_EXECUTING" in diagnose
    assert "CASE_E_ODOM_NOT_REFLECTING_MOTION" in diagnose
    assert "CASE_F_ROTATION_PROGRESS_ONLY" in diagnose
    assert "CASE_G_ROTATION_STALL" in diagnose

    assert "progress_checker.plugin" in verify
    assert "progress_checker.required_movement_angle" in verify
    assert "FollowPath.rotate_to_goal_heading" in verify
    assert "FollowPath.angular_dist_threshold" in verify
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
    guard = (scripts_root / "runtime_health_guard.py").read_text(encoding="utf-8")
    helpers = (scripts_root / "runtime_health_helpers.sh").read_text(encoding="utf-8")
    runner = (scripts_root / "run_runtime_health_guard.sh").read_text(encoding="utf-8")
    common = (scripts_root / "run_common_services.sh").read_text(encoding="utf-8")
    canonical = (scripts_root / "canonical_tf_helpers.sh").read_text(encoding="utf-8")
    nav_helpers = (scripts_root / "nav_runtime_helpers.sh").read_text(encoding="utf-8")
    container_tool = (ROOT / "scripts" / "jetson" / "njrh_container.sh").read_text(encoding="utf-8")
    cpu_affinity = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "cpu_affinity.env"
    ).read_text(encoding="utf-8")

    assert 'super().__init__("runtime_health_guard")' in guard
    assert '"/local_state/odometry"' in guard
    assert '"/fastlio/base_odometry"' in guard
    assert '"/perception/obstacle_points"' in guard
    assert "NJRH_RUNTIME_HEALTH_OBSERVE_HEAVY_TOPICS" in guard
    assert "if self.observe_heavy_topics:" in guard
    assert 'NJRH_RUNTIME_HEALTH_GRAPH_PERIOD_SEC", 2.0' in guard
    assert 'NJRH_RUNTIME_HEALTH_WRITE_PERIOD_SEC", 1.0' in guard
    assert guard.count('self._make_subscription(TFMessage, "/tf"') == 1
    assert '"/tf"' in guard
    assert '"schema": "njrh.runtime_health.v1"' in guard
    assert "os.replace(tmp_name, self.output_path)" in guard
    assert "--once" in guard
    assert "runtime_health_check()" in helpers
    assert "runtime_health_fresh_tf_ready()" in helpers
    assert "runtime_health_topic_message_ready()" in helpers
    assert 'item.get("last_received_at") is None' in helpers
    assert "runtime_health_tf_seen()" in helpers
    assert "NJRH_RUNTIME_HEALTH_TF_SEEN_MAX_AGE_SEC" in helpers
    assert 'njrh_exec_affined runtime_health_guard python3 "${guard_script}"' in runner
    assert 'exec njrh_exec_affined runtime_health_guard' not in runner
    assert "NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART:-false" in common
    assert 'start_common_process "runtime_health_guard"' in common
    assert common.index('start_canonical_helper \\\n  "robot_local_state_common"') < common.index(
        "NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART"
    )
    assert common.index("NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART") < common.index(
        'start_overlay_helper "local_perception_common"'
    )
    assert 'source "${SCRIPT_DIR}/runtime_health_helpers.sh"' in canonical
    assert 'source "${SCRIPT_DIR}/runtime_health_helpers.sh"' in nav_helpers
    assert 'runtime_health_check "local_state_fastlio_endpoint"' in canonical
    assert 'runtime_health_check "local_state_ready"' not in canonical
    assert 'runtime_health_topic_message_ready "${topic}"' in nav_helpers
    assert 'runtime_health_fresh_tf_ready "${target_frame}" "${source_frame}" "${max_age_sec}"' in nav_helpers
    assert "local costmap observation ready from runtime health snapshot" not in nav_helpers
    assert "wait_for_transformable_obstacle_points" in nav_helpers
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
    assert "lidar_rpy: [0.0, -0.3490658503988659, 3.141592653589793]" in sensors
    assert "imu_rpy: [0.0, -0.3490658503988659, 3.141592653589793]" in sensors
    assert "charge_contact_xyz: [0.398, 0.0, 0.255]" in sensors
    assert "ranger_base_frame: ranger_base_link" in sensors
    assert "gs2_frame: gs2_link" in overlay_sensors
    assert "charge_contact_frame: charge_contact_link" in overlay_sensors
    assert "ranger_base_frame: ranger_base_link" in overlay_sensors
    assert "gs2_x: 0.36" in overlay_sensors
    assert "gs2_z: 0.290" in overlay_sensors
    assert "lidar_yaw: 3.141592653589793" in overlay_sensors
    assert "lidar_axis_yaw: 0.0" in overlay_sensors
    assert "imu_yaw: 3.141592653589793" in overlay_sensors
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
        assert "cmd_vel_topic: /cmd_vel_docking" in cfg
        assert "status_topic: /docking/status" in cfg
        assert "start_service: /docking/start" in cfg
        assert "stop_service: /docking/stop" in cfg
        assert "undock_service: /docking/undock" in cfg
        assert "forced_mode_topic: /ranger_mini3/forced_mode" in cfg
        assert "park_topic: /ranger_mini3/park" in cfg
        assert "reverse_enable_topic: /ranger_mini3/docking_allow_reverse" in cfg
        assert "use_crab_mode: true" in cfg
        assert "gs2_z_m: 0.290" in cfg
        assert "charge_contact_x_m: 0.398" in cfg
        assert "gs2_to_contact_x_m: 0.038" in cfg
        assert "housing_lateral_length_m: 0.235" in cfg
        assert "housing_vertical_width_m: 0.080" in cfg
        assert "electrode_lateral_length_m: 0.185" in cfg
        assert "electrode_vertical_width_m: 0.030" in cfg
        assert "positive_electrode_position: upper" in cfg
        assert "min_points: 8" in cfg
        assert "lateral_gate_m: 0.20" in cfg
        assert "stable_frames_required: 3" in cfg
        assert "filter_alpha: 0.25" in cfg
        assert "use_yaw_fit: true" in cfg
        assert "pre_dock_distance_m: 0.60" in cfg
        assert "distance_m: 0.60" in cfg
        assert "speed_mps: 0.06" in cfg
        assert "min_clear_distance_m: 0.45" in cfg
        assert "timeout_s: 20.0" in cfg
        assert "odom_topic: /local_state/odometry" in cfg
        assert "odom_timeout_s: 0.50" in cfg
        assert "odom_start_timeout_s: 2.0" in cfg
        assert "command_settle_s: 0.5" in cfg
        assert "motion_start_timeout_s: 6.0" in cfg
        assert "no_progress_timeout_s: 2.0" in cfg
        assert "progress_epsilon_m: 0.005" in cfg
        assert yaml_number(cfg, "motion_start_timeout_s") >= 5.0
        assert yaml_number(cfg, "no_progress_timeout_s") == 2.0
        assert yaml_number(cfg, "speed_mps") == 0.06
        timeout_budget = (
            yaml_number(cfg, "command_settle_s")
            + yaml_number(cfg, "motion_start_timeout_s")
            + yaml_number(cfg, "distance_m") / yaml_number(cfg, "speed_mps")
            + 2.0
        )
        assert yaml_number(cfg, "timeout_s") >= timeout_budget
        assert "max_angular_speed_radps: 0.12" in cfg
        assert "ky: 0.55" in cfg
        assert "ky_lateral: 0.70" in cfg
        assert "lateral_command_sign: -1.0" in cfg
        assert "kyaw: -0.70" in cfg
        assert "lateral_deadband_m: 0.005" in cfg
        assert "min_align_speed_mps: 0.035" in cfg
        assert "min_lateral_speed_mps: 0.025" in cfg
        assert "max_lateral_speed_mps: 0.04" in cfg
        assert "max_forward_while_lateral_mps: 0.000" in cfg
        assert "lock_lateral_during_final_insert: true" in cfg
        assert "max_command_steering_rad: 0.35" in cfg
        assert "contact_crawl_speed_mps: 0.025" in cfg
        assert "lateral_soft_limit_m: 0.015" in cfg
        assert "lateral_hard_limit_m: 0.050" in cfg
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
    readme = (ROOT / "src" / "robot_docking_manager" / "README.md").read_text(encoding="utf-8")
    gs2_doc = (ROOT / "docs" / "gs2_docking_lidar.md").read_text(encoding="utf-8")

    assert "add_executable(docking_manager_node src/docking_manager_node.cpp)" in cmake
    assert "find_package(nav_msgs REQUIRED)" in cmake
    assert "nav_msgs" in cmake
    assert "<name>robot_docking_manager</name>" in package_xml
    assert "<depend>nav_msgs</depend>" in package_xml
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
    assert "create_service<std_srvs::srv::Trigger>" in node
    assert "create_subscription<sensor_msgs::msg::LaserScan>" in node
    assert "create_subscription<sensor_msgs::msg::BatteryState>" in node
    assert "create_subscription<nav_msgs::msg::Odometry>" in node
    assert "State::BlindApproach" in node
    assert "State::ContactVerify" in node
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
    assert "docked_stop(\"docked_charging_detected\")" in node
    assert 'declare_parameter<bool>("detector.use_yaw_fit", false)' in node
    assert "estimate_yaw_error" in node
    assert "filter_detection" in node
    assert "limit_yaw_rate_for_ackermann" in node
    assert "valid_detection_streak_" in node
    assert "min_align_speed_mps_" in node
    assert "lateral_command_sign_ * ky_lateral_ * lateral_error" in node
    assert "cmd.angular.z = clamp(kyaw_ * yaw_error" in node
    assert "min_lateral_speed_mps_" in node
    assert "lock_lateral_during_final_insert_" in node
    assert "cmd.linear.y = 0.0;" in node
    assert "if (!final_insert_locked && (!lateral_ok || !yaw_ok" in node
    assert "release_docking_motion_mode(park_on_docked_)" in node
    assert "if (!distance_ok && distance_error > 0.0)" in node
    assert "FindPackageShare(\"robot_nav_config\")" in launch
    assert "install/robot_docking_manager/lib/robot_docking_manager/docking_manager_node" in runner
    assert "Python fallback has been removed" in runner
    assert "/cmd_vel_docking" in readme
    assert "/cmd_vel_collision_checked" in readme
    assert "controller.kyaw=-0.70" in readme
    assert "yaw tolerance is `2deg`" in readme
    assert "/docking/undock" in readme
    assert "/local_state/odometry" in readme
    assert "elapsed command time is not treated as distance" in readme
    assert "/ranger_mini3/docking_allow_reverse=true" in readme
    assert "undock.speed_mps=0.06" in readme
    assert "undock.motion_start_timeout_s" in readme
    assert "undock.no_progress_timeout_s" in readme
    assert "robot_safety" in readme
    assert "rosbag" in readme
    assert "Do not publish docking control directly to `/cmd_vel_safe`" in gs2_doc
    assert "POST /api/v1/docking/undock" in gs2_doc
    assert "Undocking completion is odometry-confirmed" in gs2_doc

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
    assert "speed_mps: 0.06" in (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "docking.yaml"
    ).read_text(encoding="utf-8")
    assert yaml_number(nav2, "vx_min") >= 0.0
    assert "CASE_DOCKING_MANAGER_NO_CMD" in diagnose
    assert "status_has_waiting_first_motion_with_cmd" in diagnose
    assert "CASE_DOCKING_MANAGER_WAITING_FIRST_MOTION_WITH_CMD" in diagnose


def test_bms_charging_contact_truth_table_for_full_dock_contact():
    package_root = ROOT / "src" / "robot_api_server"
    bms_header = (package_root / "include" / "robot_api_server" / "bms_contact.hpp").read_text(
        encoding="utf-8"
    )
    bms_cpp = (package_root / "src" / "bms_contact.cpp").read_text(encoding="utf-8")

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
    bms_cpp = (package_root / "src" / "bms_contact.cpp").read_text(encoding="utf-8")
    script = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "verify_pre_navigation_undock_gate.sh"
    ).read_text(encoding="utf-8")

    assert "GET /api/v1/navigation/pre_goal_check" in node_cpp
    assert "handle_navigation_pre_goal_check" in node_cpp
    assert "struct PreNavigationDockCheck" in node_cpp
    assert "pre_navigation_dock_check_snapshot" in node_cpp
    assert "pre_navigation_dock_check_json" in node_cpp
    assert "api_bms_charging_contact" in node_cpp
    assert "api_bms_charging_contact_reason" in node_cpp
    assert "final_is_docked_or_charging" in node_cpp
    assert "final_auto_undock_required" in node_cpp
    assert "can_auto_undock" in node_cpp
    assert "auto_undock_reason" in node_cpp
    assert "docking_status_charging" in node_cpp
    assert "docking_status_docked" in node_cpp
    assert "runtime_docking_state_docked" in node_cpp
    assert "bms_charging_contact:" in node_cpp

    assert "POWER_SUPPLY_STATUS_CHARGING" in bms_cpp
    assert "POWER_SUPPLY_STATUS_FULL" in bms_cpp
    assert "current_above_threshold" in bms_cpp
    assert "present_voltage_valid" in bms_cpp
    assert "full_soc_present_voltage_valid" in bms_cpp

    goal_block = node_cpp[
        node_cpp.index("HttpResponse handle_navigation_goal") : node_cpp.index(
            "bool undock_before_navigation_if_needed"
        )
    ]
    assert "const bool by_pose_id = !pose_id.empty();" in goal_block
    assert '"navigation_goal"' in goal_block
    assert "pre_navigation_dock_check_payload" in goal_block
    assert "undock_before_navigation_if_needed(" in goal_block
    assert goal_block.index("undock_before_navigation_if_needed(") < goal_block.index("NavigateToPose::Goal goal")
    assert "navigation requires successful undock first" in goal_block
    assert goal_block.index("navigation requires successful undock first") < goal_block.index(
        "NavigateToPose::Goal goal"
    )
    assert "pre_navigation_undock" in goal_block
    assert "pre_navigation_undock_detail" in goal_block
    assert "pre_navigation_dock_check" in goal_block
    assert "navigation_goal_error_response" in goal_block

    precheck_block = node_cpp[
        node_cpp.index("HttpResponse handle_navigation_pre_goal_check") : node_cpp.index(
            "HttpResponse navigation_goal_error_response"
        )
    ]
    assert "read_only" in precheck_block
    assert "find_floor_catalog_pose(*map_catalog_, building_id, floor_id, pose_id)" in precheck_block
    assert "map_catalog_->find_map_by_id(map_id)" in precheck_block
    assert 'query_number_value(request, "x")' in precheck_block
    assert 'query_number_value(request, "theta")' in precheck_block
    assert "navigation_lifecycle_snapshot()" in precheck_block
    assert "start_pre_navigation_undock" not in precheck_block
    assert "async_send_goal" not in precheck_block
    assert "call_undock_service_with_charging_retry" not in precheck_block

    assert "pre_navigation_dock_check" in script
    assert "/api/v1/navigation/pre_goal_check" in script
    assert "--execute-goal" in script
    assert "Read-only by default" in script
    assert "POST /api/v1/navigation/goal" in script


def test_phase25_docked_motion_interlock_contract():
    api_root = ROOT / "src" / "robot_api_server"
    api_cpp = (api_root / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
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

    assert "struct DockContactLatchSnapshot" in api_cpp
    assert "docking_contact_latch_file" in api_cpp
    assert "read_dock_contact_latch" in api_cpp
    assert "update_dock_contact_latch" in api_cpp
    assert "dock_latch_indicates_docked" in api_cpp
    assert "dock_contact_snapshot" in api_cpp
    assert "dock_contact_latch:" in api_cpp
    assert "DOCKED_OR_CHARGING_CONTACT" in api_cpp
    assert "blocked_by_docked_contact" in api_cpp
    assert "auto_undock_required" in api_cpp
    assert "safety" in api_cpp
    assert "normal_motion_blocked_reason" in api_cpp
    assert "pre_navigation dock gate" in api_cpp
    assert "pre_navigation_dock_check" in api_cpp
    assert "start_pre_navigation_undock" in api_cpp
    assert "NavigateToPose::Goal goal" in api_cpp
    goal_block = api_cpp[
        api_cpp.index("HttpResponse handle_navigation_goal") : api_cpp.index(
            "bool undock_before_navigation_if_needed"
        )
    ]
    assert goal_block.index("undock_before_navigation_if_needed(") < goal_block.index(
        "NavigateToPose::Goal goal"
    )
    gate_block = api_cpp[
        api_cpp.index("PreNavigationDockCheck pre_navigation_dock_check_snapshot") : api_cpp.index(
            "std::string bms_charging_contact_snapshot_json"
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
    assert "publish_checked_command(*msg, true)" in safety_cpp
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
    bms_cpp = (ROOT / "src" / "robot_api_server" / "src" / "bms_contact.cpp").read_text(
        encoding="utf-8"
    )

    assert "latched_docked" in api_cpp
    assert "docked_state_class" in api_cpp
    assert "DOCKED_CONFIRMED" in api_cpp
    assert "DOCKED_LATCHED" in api_cpp
    assert "DOCKED_CANDIDATE" not in api_cpp or "current_robot_pose_snapshot" not in api_cpp[
        api_cpp.index("PreNavigationDockCheck pre_navigation_dock_check_snapshot") : api_cpp.index(
            "std::string bms_charging_contact_snapshot_json"
        )
    ]
    assert "docked_evidence" in api_cpp
    assert "docked_warnings" in api_cpp
    assert "handle_docking_confirm_docked" in api_cpp
    assert "handle_docking_clear_latch" in api_cpp
    assert '"/api/v1/docking/confirm_docked"' in api_cpp
    assert '"/api/v1/docking/clear_docked_latch"' in api_cpp
    assert "sent_velocity" in api_cpp
    assert "maintenance_only" in api_cpp
    assert "fs::rename" in api_cpp
    assert ".tmp" in api_cpp

    gate_block = api_cpp[
        api_cpp.index("PreNavigationDockCheck pre_navigation_dock_check_snapshot") : api_cpp.index(
            "std::string bms_charging_contact_snapshot_json"
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
    assert "publish_checked_command(*msg, true)" in safety_cpp
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
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
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

    for text in (api_cpp, api_cfg, overlay_api_cfg):
        assert "dock_contact_latch_bms_ttl_sec" in text
        assert "dock_contact_latch_bms_require_contact_sec" in text
        assert "dock_contact_latch_bms_clear_no_contact_sec" in text
        assert "dock_contact_latch_allow_bms_stale_auto_undock" in text
        assert "dock_contact_latch_clear_when_live_undocked_no_contact" in text
        assert "dock_contact_latch_max_age_warn_sec" in text

    gate_block = api_cpp[
        api_cpp.index("PreNavigationDockCheck pre_navigation_dock_check_snapshot") : api_cpp.index(
            "std::string bms_charging_contact_snapshot_json"
        )
    ]
    assert "check.strong_live_docked" in gate_block
    assert "check.latch_valid_for_auto_undock" in gate_block
    assert "check.final_is_docked_or_charging = check.strong_live_docked || check.latch_valid_for_auto_undock" in gate_block
    assert "stale_bms_latch_cleared_live_undocked_no_contact" in gate_block
    assert "check.dock_contact_latch_contradicted_by_live_state" in gate_block
    assert "check.live_bms_charging_contact_stable" in gate_block
    assert "check.bms.contact ||" not in gate_block

    handle_bms_block = api_cpp[
        api_cpp.index("void handle_bms_state") : api_cpp.index("void handle_localization_result")
    ]
    assert "maybe_update_bms_dock_contact_latch" in api_cpp
    assert "contact_stable_duration_sec >= dock_contact_latch_bms_require_contact_sec_" in handle_bms_block
    assert 'update_dock_contact_latch(\n        true,\n        "bms"' not in handle_bms_block
    assert "runtime.navigation_active && navigation_goal_job_running()" in api_cpp

    assert "dock_contact_latch_age_sec" in api_cpp
    assert "dock_contact_latch_stale" in api_cpp
    assert "dock_contact_latch_contradicted_by_live_state" in api_cpp
    assert "dock_contact_latch_auto_cleared" in api_cpp
    assert "strong_live_docked" in api_cpp
    assert "latch_valid_for_auto_undock" in api_cpp
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
    overlay_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_only_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf_wheel_only.yaml"
    ).read_text(encoding="utf-8")
    overlay_wheel_odom_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_wheel_odom_ekf.yaml"
    ).read_text(encoding="utf-8")
    source_wheel_odom_cfg = (
        ROOT / "src" / "robot_local_state" / "config" / "local_state_wheel_odom_ekf.yaml"
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
    readme = (ROOT / "src" / "robot_local_state" / "README.md").read_text(encoding="utf-8")

    assert "find_package(sensor_msgs REQUIRED)" in cmake
    assert "add_executable(imu_gyro_bias_filter_node" in cmake
    assert "target_compile_features(imu_gyro_bias_filter_node PUBLIC cxx_std_17)" in cmake
    assert "sensor_msgs" in cmake
    assert "<exec_depend>robot_localization</exec_depend>" in package_xml
    assert "<depend>sensor_msgs</depend>" in package_xml
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
    assert "ros2 run robot_localization ekf_node" in overlay_runner
    assert "-r __node:=robot_local_state" in overlay_runner
    assert "LOCAL_STATE_WHEEL_ODOM_EKF_PARAMS_FILE" in overlay_runner
    assert "-r __node:=wheel_odom_ekf_input" in overlay_runner
    assert "LOCAL_STATE_EKF_PROFILE" in overlay_runner
    assert "NJRH_LOCAL_STATE_EKF_PROFILE" in overlay_runner
    assert "local_state_ekf_wheel_only.yaml" in overlay_runner
    assert 'invalid LOCAL_STATE_EKF_PROFILE=${EKF_PROFILE}; expected wheel_imu or wheel_only' in overlay_runner
    assert 'LOCAL_STATE_EKF_PROFILE=wheel_only; skipping IMU gyro bias filter' in overlay_runner
    assert "LOCAL_STATE_IMU_BIAS_FILTER_PARAMS_FILE" in overlay_runner
    assert "imu_gyro_bias_filter_node" in overlay_runner
    assert "-r __node:=imu_gyro_bias_filter" in overlay_runner
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
    assert "LOCAL_STATE_EKF_PROFILE=wheel_imu" in readme
    assert "LOCAL_STATE_MODE=passthrough" in readme
    assert 'declare_parameter<double>("odom_yaw_offset_rad", 0.0)' in local_state_node
    assert 'declare_parameter<bool>("rotate_odom_position_with_yaw_offset", true)' in local_state_node
    assert 'declare_parameter<bool>("anchor_pose_to_first_sample", false)' in local_state_node
    assert 'declare_parameter<bool>("apply_pose_covariance_floor", false)' in local_state_node
    assert 'declare_parameter<double>("pose_covariance_floor_yaw", 0.0)' in local_state_node
    assert 'declare_parameter<std::string>("input_base_frame", "base_link")' in local_state_node
    assert 'declare_parameter<bool>("publish_on_callback", false)' in local_state_node
    assert 'declare_parameter<bool>("republish_latest", true)' in local_state_node
    assert 'declare_parameter<double>("republish_latest_max_age_sec", 0.5)' in local_state_node
    assert "apply_pose_anchor(local_odom)" in local_state_node
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
    assert "fastlio_odom_bridge_process_running && local_state_node_process_running" in overlay_tf_helpers
    assert 'canonical_helper_start_ready "${helper_name}"' in overlay_tf_helpers
    assert "helper child process did not become ready" in overlay_tf_helpers
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
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom_ekf" in cfg
        assert "odom0_config: [true, true, false," in cfg
        assert "false, false, true," in cfg
        assert "imu0: /lidar_imu_bias_corrected" in cfg
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
    for imu_cfg in (source_imu_bias_cfg, overlay_imu_bias_cfg):
        assert "imu_topic: /lidar_imu" in imu_cfg
        assert "odom_topic: /wheel/odom_ekf" in imu_cfg
        assert "cmd_vel_topic: /cmd_vel_safe" in imu_cfg
        assert "output_imu_topic: /lidar_imu_bias_corrected" in imu_cfg
        assert "bias_topic: /local_state/imu_bias" in imu_cfg
        assert "stationary_required_sec: 1.0" in imu_cfg
        assert "accumulator_alpha: 0.02" in imu_cfg
        assert "zero_output_when_stationary: true" in imu_cfg
        assert "corrected_output_rate_hz: 100.0" in imu_cfg
        assert "bias_publish_rate_hz: 10.0" in imu_cfg
        assert "corrected_output_latest_on_timer: true" in imu_cfg
        assert "corrected_output_preserve_source_stamp: true" in imu_cfg
        assert "corrected_output_max_source_age_sec: 0.20" in imu_cfg
        assert "bias_publish_preserve_source_stamp: true" in imu_cfg
        assert yaml_number(imu_cfg, "corrected_output_rate_hz") <= 100.0
        assert yaml_number(imu_cfg, "bias_publish_rate_hz") <= 10.0
    assert "lid_topic: /lidar_points" in mapping_fastlio_cfg
    assert "imu_topic: /lidar_imu" in mapping_fastlio_cfg
    assert "imu_topic: /lidar_imu_bias_corrected" not in mapping_fastlio_cfg
    assert 'declare_parameter<std::string>("output_imu_topic", "/lidar_imu_bias_corrected")' in imu_bias_node
    assert 'declare_parameter<std::string>("bias_topic", "/local_state/imu_bias")' in imu_bias_node
    assert 'declare_parameter<double>("corrected_output_rate_hz", 100.0)' in imu_bias_node
    assert 'declare_parameter<double>("bias_publish_rate_hz", 10.0)' in imu_bias_node
    assert 'declare_parameter<bool>("corrected_output_latest_on_timer", true)' in imu_bias_node
    assert 'declare_parameter<bool>("corrected_output_preserve_source_stamp", true)' in imu_bias_node
    assert 'declare_parameter<double>("corrected_output_max_source_age_sec", 0.20)' in imu_bias_node
    assert 'declare_parameter<bool>("bias_publish_preserve_source_stamp", true)' in imu_bias_node
    assert "stationary_confirmed" in imu_bias_node
    assert "sample_is_safe_for_bias_update" in imu_bias_node
    assert "create_publisher<sensor_msgs::msg::Imu>(" in imu_bias_node
    assert "rclcpp::QoS(100)" in imu_bias_node
    assert "corrected_output_timer_" in imu_bias_node
    assert "bias_publish_timer_" in imu_bias_node
    assert "latest_corrected_imu_" in imu_bias_node
    assert "corrected_source_age_sec" in imu_bias_node
    assert "IMU bias filter rates input=" in imu_bias_node
    assert "corrected.angular_velocity.z = 0.0" in imu_bias_node
    assert "corrected.angular_velocity.z -= bias_.z" in imu_bias_node
    assert "TransformBroadcaster" not in imu_bias_node
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env
    assert "LOCAL_STATE_EKF_PROFILE_FILE" in common_env
    assert "local_state_ekf_profile.env" in common_env
    assert 'source "${LOCAL_STATE_EKF_PROFILE_FILE}"' in common_env
    assert 'export NJRH_LOCAL_STATE_EKF_PROFILE="${NJRH_LOCAL_STATE_EKF_PROFILE:-wheel_only}"' in local_state_ekf_profile_env
    assert "wheel_imu" in local_state_ekf_profile_env
    assert "record_rate /lidar_imu " in verify_local_state_rates
    assert "record_rate /lidar_imu_bias_corrected " in verify_local_state_rates
    assert "record_rate /local_state/imu_bias " in verify_local_state_rates
    assert "LOCAL_STATE_EKF_PROFILE=wheel_only skips IMU rate checks" in verify_local_state_rates
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
    assert 'include("robot_local_perception"' in launch_file
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


def test_commercial_runtime_architecture_contract_is_documented():
    doc = (ROOT / "docs" / "commercial_runtime_architecture.md").read_text(encoding="utf-8")
    readiness = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "check_commercial_runtime_ready.sh"
    ).read_text(encoding="utf-8")
    commercial_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "commercial_runtime_helpers.sh"
    ).read_text(encoding="utf-8")
    resident_runtime = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_navigation_runtime_services.sh"
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
    assert "check_topic \"/perception/obstacle_points\"" in readiness
    assert 'check_lifecycle_active "/collision_monitor" 12' in readiness
    assert 'check_tf "odom" "base_link" 10' in readiness
    assert 'check_topic "/perception/obstacle_points" 10' in readiness
    assert 'wait_for_global_costmap_static 10' in readiness
    assert "navigation_runtime_ready_for_current_floor()" in commercial_helpers
    assert "write_runtime_map_context()" in commercial_helpers
    assert "os.getpid()" in commercial_helpers
    assert 'tmp_path = f"{path}.tmp"' not in commercial_helpers
    assert 'runtime_readiness_probe lifecycle-active "${node_name}" "${timeout_sec}"' in commercial_helpers
    assert "from lifecycle_msgs.srv import GetState" not in commercial_helpers
    assert 'service_name = f"{node_name}/get_state"' not in commercial_helpers
    assert "ros2 lifecycle get" not in commercial_helpers
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
    assert "resident_navigation_ready" in resident_runtime
    assert "localization stack ready for initial relocalization" in resident_runtime
    assert "initial localization accepted: bridge_status.has_map_to_odom=true and map->odom are ready" in resident_runtime
    assert "Nav2 lifecycle and global costmap are ready" in resident_runtime
    assert "navigation_runtime_ready_for_current_floor 3" not in resident_runtime
    assert "wait_for_resident_navigation_runtime_ready" not in resident_runtime
    assert "floor_id is required for resident navigation runtime" in resident_runtime


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
    assert 'if [[ "${NJRH_COMMON_ENV_SETUP_DONE:-}" != "1" ]] || project_overlay_missing; then' in common_env
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
    autostart_installer = (ROOT / "scripts" / "jetson" / "install_njrh_autostart.sh").read_text(encoding="utf-8")
    autostart_doc = (ROOT / "docs" / "autostart_systemd.md").read_text(encoding="utf-8")
    powershell_helper = (ROOT / "scripts" / "jetson" / "Invoke-NJRHJetson.ps1").read_text(encoding="utf-8")
    assert "mode_topic" in local_perception
    assert "input_topic: /lidar_points\n" in local_perception
    assert "input_topic: /_internal/lidar_points_local" not in local_perception
    assert "input_topic: /jt128/vendor/points_raw" not in local_perception
    assert "input_reliable: false" in local_perception
    assert "input_qos_depth: 1" in local_perception
    assert "input_frame_id_override: lidar_link" in local_perception
    assert "input_transform_use_latest: true" in local_perception
    assert "input_rotation_matrix:" in local_perception
    assert "output_topic: /perception/obstacle_points" in local_perception
    assert "clearing_output_topic: /perception/clearing_points" in local_perception
    assert "processing_rate_hz: 15.0" in local_perception
    assert "process_on_callback: false" in local_perception
    assert "point_sample_stride: 1" in local_perception
    assert "restamp_to_now: false" in local_perception
    assert "restamp_to_latest_tf: false" in local_perception
    assert "require_output_stamp_tf: false" in local_perception
    assert "output_stamp_tf_target_frame: odom" in local_perception
    assert "max_output_tf_stamp_age_sec: 0.25" in local_perception
    assert "output_stamp_tf_backoff_sec: 0.0" in local_perception
    assert "output_stamp_forward_sec: 0.0" in local_perception
    assert "require_startup_tf_ready: true" in local_perception
    assert "startup_tf_warmup_sec: 1.0" in local_perception
    assert "clearing.enabled: true" in local_perception
    assert "clearing.virtual_rays.enabled: true" in local_perception
    assert "clearing.virtual_rays.angular_resolution_deg: 1.0" in local_perception
    assert "clearing.virtual_rays.range: 8.00" in local_perception
    assert "clearing.virtual_rays.range_steps: [0.50, 1.00, 2.00, 3.50, 5.50, 8.00]" in local_perception
    assert "clearing.max_points: 15000" in local_perception
    assert "status_topic: /perception/local_perception_status" in local_perception
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
    assert "cmd_vel_out_topic: /cmd_vel_safe" in robot_safety
    assert "run_local_perception.sh" in overlay_nav
    assert "run_floor_manager.sh" in overlay_nav
    assert "run_robot_safety.sh" in overlay_nav
    assert "run_ranger_mini3_mode_controller.sh" in overlay_nav
    assert "ensure_resident_overlay_helper_process()" in overlay_nav
    assert 'ensure_resident_overlay_helper_process "robot_safety" "robot_safety"' in overlay_nav
    assert (
        'ensure_resident_overlay_helper_process "ranger_mini3_mode_controller" '
        '"ranger_mini3_mode_controller"'
    ) in overlay_nav
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
    assert "install/robot_local_perception/lib/robot_local_perception/local_perception_node" in overlay_local_perception
    assert "Python fallback has been removed" in overlay_local_perception
    assert 'wait_for_topic_message "/local_state/odometry"' not in overlay_local_perception
    assert 'wait_for_tf_transform "base_link" "lidar_link"' not in overlay_local_perception
    assert 'wait_for_fresh_tf_transform \\' not in overlay_local_perception
    assert "local perception TF prerequisites ready" not in overlay_local_perception
    assert "starting local perception without startup topic/TF probes" in overlay_local_perception
    assert "src/robot_local_perception/scripts/local_perception_node.py" not in overlay_nav_helpers
    assert "python3 .*local_perception_node.py" not in overlay_nav_helpers
    assert "robot_local_perception/local_perception_node" in overlay_nav_helpers
    assert "local_perception_runtime_config_ready()" in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception restamp_to_now' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception restamp_to_latest_tf' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception require_output_stamp_tf' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception input_reliable' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception input_qos_depth' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception input_transform_use_latest' in overlay_nav_helpers
    assert '[[ "${restamp_to_now}" == *"False"* ]]' in overlay_nav_helpers
    assert '[[ "${restamp_to_latest_tf}" == *"False"* ]]' in overlay_nav_helpers
    assert '[[ "${require_output_stamp_tf}" == *"False"* ]]' in overlay_nav_helpers
    assert '[[ "${input_reliable}" == *"False"* ]]' in overlay_nav_helpers
    assert '[[ "${input_qos_depth}" == *"1"* ]]' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception max_output_tf_stamp_age_sec' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception output_stamp_tf_backoff_sec' in overlay_nav_helpers
    assert 'ros2 param get /robot_local_perception output_stamp_forward_sec' in overlay_nav_helpers
    assert "robot_floor_manager/floor_manager_node" in overlay_nav_helpers
    assert "install/robot_safety/lib/robot_safety/robot_safety_node" in overlay_robot_safety
    assert "Python fallback has been removed" in overlay_robot_safety
    assert "ranger_mini3_mode_controller" in overlay_nav_helpers
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
    assert 'keepout_mask_yaml:="${NAV2_KEEP_OUT_MASK_YAML}"' in overlay_nav
    assert 'speed_mask_yaml:="${NAV2_SPEED_MASK_YAML}"' in overlay_nav
    assert "costmap_filter_info_server" in standard_navigation_launch
    assert "cpu_affinity_prefix" in standard_navigation_launch
    assert "TimerAction" in standard_navigation_launch
    assert 'DeclareLaunchArgument(\n                "nav_lifecycle_start_delay"' in standard_navigation_launch
    assert 'NJRH_NAV_LIFECYCLE_START_DELAY_SEC", "18.0"' in standard_navigation_launch
    assert "filter_lifecycle_nodes = [" in standard_navigation_launch
    assert "navigation_lifecycle_nodes = [" in standard_navigation_launch
    assert 'name="lifecycle_manager_costmap_filters"' in standard_navigation_launch
    assert 'name="lifecycle_manager_navigation"' in standard_navigation_launch
    assert '{"node_names": filter_lifecycle_nodes}' in standard_navigation_launch
    assert '{"node_names": navigation_lifecycle_nodes}' in standard_navigation_launch
    assert '{"bond_timeout": 0.0}' in standard_navigation_launch
    assert 'with_cpu_affinity("controller_server", node_kwargs)' in standard_navigation_launch
    assert 'with_cpu_affinity("velocity_smoother", node_kwargs)' in standard_navigation_launch
    assert 'with_cpu_affinity("collision_monitor", node_kwargs)' in standard_navigation_launch
    assert standard_navigation_launch.index('"velocity_smoother",') < standard_navigation_launch.index('"bt_navigator",')
    assert standard_navigation_launch.index('"collision_monitor",') < standard_navigation_launch.index('"bt_navigator",')
    assert nav2_yaml.index("- velocity_smoother") < nav2_yaml.index("- bt_navigator")
    assert nav2_yaml.index("- collision_monitor") < nav2_yaml.index("- bt_navigator")
    assert "keepout_filter_mask_server" in standard_navigation_launch
    assert "speed_filter_mask_server" in standard_navigation_launch
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
    assert "colcon build --packages-select robot_interfaces robot_floor_manager" in overlay_floor_manager
    assert "--flat-map-name" in overlay_promote_floor
    assert "robot_map_toolkit/scripts/map_toolkit_cli.py" in overlay_promote_floor
    assert "local_costmap_debug.launch.py" in overlay_local_costmap_debug
    assert 'source "${SCRIPT_DIR}/cpu_affinity.sh"' in overlay_local_costmap_debug
    assert "run_local_perception.sh" in overlay_local_costmap_debug
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
    assert '"laser_scan_to_flatscan"' not in overlay_stop_navigation
    assert '"laser_scan_to_flatscan"' not in overlay_nav_runtime_helpers

    assert "__node:=lifecycle_manager_costmap_filters" in overlay_nav_runtime_helpers
    assert "__node:=lifecycle_manager_navigation" in overlay_nav_runtime_helpers
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
    assert 'env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"' in overlay_common_services
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' in overlay_common_services
    assert "run_local_perception.sh" in overlay_common_services
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
    assert "no valid last navigation map; common services stay alive in NO_MAP mode" in overlay_common_services
    assert "last_navigation_map.json" in overlay_common_services
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
    assert 'deadline=$((SECONDS + ROBOT_API_READY_TIMEOUT_SEC))' in container_script
    assert "seq 1 30" not in container_script
    start_runtime_case = container_script.split("start-runtime)", 1)[1].split(";;", 1)[0]
    assert "start_container" in start_runtime_case
    assert "start_dashboard" not in start_runtime_case
    assert "'start-common'" in powershell_helper
    assert "'start-debug-runtime'" in powershell_helper
    assert "bash scripts/jetson/njrh_container.sh start" in autostart_runner
    assert "bash scripts/jetson/njrh_container.sh start-runtime" not in autostart_runner
    assert "prepare_container_permissions" in autostart_runner
    assert "exec bash scripts/run_common_services.sh" in autostart_runner
    assert "robot_eai_gs2/gs2_driver_node" in autostart_runner
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
        ("NJRH_CPUSET_NAV_PLANNING", "0"),
        ("NJRH_CPUSET_LIDAR_PERCEPTION", "4,5"),
        ("NJRH_CPUSET_LIDAR_DRIVER", "4"),
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
    assert 'NJRH_CPUSET_SLAM_TOOLBOX_MAPPING="${NJRH_CPUSET_SLAM_TOOLBOX_MAPPING:-${NJRH_CPUSET_MAPPING_BACKEND}}"' in affinity_cfg
    assert 'NJRH_CPUSET_PGO_MAPPING="${NJRH_CPUSET_PGO_MAPPING:-${NJRH_CPUSET_MAPPING_BACKEND}}"' in affinity_cfg
    assert "njrh_exec_affined()" in affinity_helper
    assert "njrh_run_affined()" in affinity_helper
    assert "njrh_start_affined_background()" in affinity_helper
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
        "run_local_perception.sh",
        "run_robot_safety.sh",
        "run_nav2_navigation.sh",
        "run_fastlio_tf.sh",
        "run_pgo.sh",
    ):
        text = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / runner).read_text(encoding="utf-8")
        assert 'source "${SCRIPT_DIR}/cpu_affinity.sh"' in text


def test_robot_api_server_is_cpp_gateway_not_dashboard_backend():
    package_root = ROOT / "src" / "robot_api_server"
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
    api_time_header = (package_root / "include" / "robot_api_server" / "api_time_utils.hpp").read_text(
        encoding="utf-8"
    )
    api_time_cpp = (package_root / "src" / "api_time_utils.cpp").read_text(encoding="utf-8")
    bms_header = (package_root / "include" / "robot_api_server" / "bms_contact.hpp").read_text(
        encoding="utf-8"
    )
    bms_cpp = (package_root / "src" / "bms_contact.cpp").read_text(encoding="utf-8")
    docking_job_header = (
        package_root / "include" / "robot_api_server" / "docking_job_model.hpp"
    ).read_text(encoding="utf-8")
    docking_job_cpp = (package_root / "src" / "docking_job_model.cpp").read_text(encoding="utf-8")
    docking_status_header = (
        package_root / "include" / "robot_api_server" / "docking_status_utils.hpp"
    ).read_text(encoding="utf-8")
    docking_status_cpp = (package_root / "src" / "docking_status_utils.cpp").read_text(
        encoding="utf-8"
    )
    file_utils_header = (package_root / "include" / "robot_api_server" / "file_utils.hpp").read_text(
        encoding="utf-8"
    )
    file_utils_cpp = (package_root / "src" / "file_utils.cpp").read_text(encoding="utf-8")
    floor_asset_header = (
        package_root / "include" / "robot_api_server" / "floor_asset_resolver.hpp"
    ).read_text(encoding="utf-8")
    floor_asset_cpp = (package_root / "src" / "floor_asset_resolver.cpp").read_text(encoding="utf-8")
    http_header = (package_root / "include" / "robot_api_server" / "http_common.hpp").read_text(
        encoding="utf-8"
    )
    http_cpp = (package_root / "src" / "http_common.cpp").read_text(encoding="utf-8")
    localization_result_header = (
        package_root / "include" / "robot_api_server" / "localization_result_model.hpp"
    ).read_text(encoding="utf-8")
    localization_result_cpp = (
        package_root / "src" / "localization_result_model.cpp"
    ).read_text(encoding="utf-8")
    storage_header = (package_root / "include" / "robot_api_server" / "storage_models.hpp").read_text(
        encoding="utf-8"
    )
    storage_cpp = (package_root / "src" / "storage_models.cpp").read_text(encoding="utf-8")
    map_asset_header = (package_root / "include" / "robot_api_server" / "map_asset_io.hpp").read_text(
        encoding="utf-8"
    )
    map_asset_cpp = (package_root / "src" / "map_asset_io.cpp").read_text(encoding="utf-8")
    map_asset_writer_header = (
        package_root / "include" / "robot_api_server" / "map_asset_writer.hpp"
    ).read_text(encoding="utf-8")
    map_asset_writer_cpp = (package_root / "src" / "map_asset_writer.cpp").read_text(encoding="utf-8")
    map_catalog_header = (
        package_root / "include" / "robot_api_server" / "map_catalog.hpp"
    ).read_text(encoding="utf-8")
    map_catalog_cpp = (package_root / "src" / "map_catalog.cpp").read_text(encoding="utf-8")
    manifest_io_header = (package_root / "include" / "robot_api_server" / "map_manifest_io.hpp").read_text(
        encoding="utf-8"
    )
    manifest_io_cpp = (package_root / "src" / "map_manifest_io.cpp").read_text(encoding="utf-8")
    navigation_cancel_header = (
        package_root / "include" / "robot_api_server" / "navigation_cancel_job_model.hpp"
    ).read_text(encoding="utf-8")
    navigation_cancel_cpp = (package_root / "src" / "navigation_cancel_job_model.cpp").read_text(
        encoding="utf-8"
    )
    poses_io_header = (package_root / "include" / "robot_api_server" / "poses_io.hpp").read_text(
        encoding="utf-8"
    )
    poses_io_cpp = (package_root / "src" / "poses_io.cpp").read_text(encoding="utf-8")
    runtime_context_header = (
        package_root / "include" / "robot_api_server" / "runtime_map_context_io.hpp"
    ).read_text(encoding="utf-8")
    runtime_context_cpp = (package_root / "src" / "runtime_map_context_io.cpp").read_text(encoding="utf-8")
    runtime_map_lookup_header = (
        package_root / "include" / "robot_api_server" / "runtime_map_lookup.hpp"
    ).read_text(encoding="utf-8")
    runtime_map_lookup_cpp = (package_root / "src" / "runtime_map_lookup.cpp").read_text(encoding="utf-8")
    robot_pose_header = (
        package_root / "include" / "robot_api_server" / "robot_pose_model.hpp"
    ).read_text(encoding="utf-8")
    robot_pose_cpp = (package_root / "src" / "robot_pose_model.cpp").read_text(encoding="utf-8")
    runtime_process_header = (
        package_root / "include" / "robot_api_server" / "runtime_process_utils.hpp"
    ).read_text(encoding="utf-8")
    runtime_process_cpp = (package_root / "src" / "runtime_process_utils.cpp").read_text(encoding="utf-8")
    semantic_header = (
        package_root / "include" / "robot_api_server" / "semantic_layer_io.hpp"
    ).read_text(encoding="utf-8")
    semantic_cpp = (package_root / "src" / "semantic_layer_io.cpp").read_text(encoding="utf-8")
    subscription_api_header = (
        package_root / "include" / "robot_api_server" / "subscription_api.hpp"
    ).read_text(encoding="utf-8")
    subscription_api_cpp = (package_root / "src" / "subscription_api.cpp").read_text(encoding="utf-8")
    subscription_header = (
        package_root / "include" / "robot_api_server" / "subscription_manager.hpp"
    ).read_text(encoding="utf-8")
    subscription_cpp = (package_root / "src" / "subscription_manager.cpp").read_text(encoding="utf-8")
    tf_pose_header = (package_root / "include" / "robot_api_server" / "tf_pose_utils.hpp").read_text(
        encoding="utf-8"
    )
    tf_pose_cpp = (package_root / "src" / "tf_pose_utils.cpp").read_text(encoding="utf-8")
    node_cpp = (package_root / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
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
    assert "src/api_time_utils.cpp" in cmake
    assert "src/bms_contact.cpp" in cmake
    assert "src/docking_job_model.cpp" in cmake
    assert "src/docking_status_utils.cpp" in cmake
    assert "src/file_utils.cpp" in cmake
    assert "src/floor_asset_resolver.cpp" in cmake
    assert "src/http_common.cpp" in cmake
    assert "src/localization_result_model.cpp" in cmake
    assert "src/map_asset_io.cpp" in cmake
    assert "src/map_asset_writer.cpp" in cmake
    assert "src/map_catalog.cpp" in cmake
    assert "src/map_manifest_io.cpp" in cmake
    assert "src/navigation_cancel_job_model.cpp" in cmake
    assert "src/poses_io.cpp" in cmake
    assert "src/runtime_map_context_io.cpp" in cmake
    assert "src/runtime_map_lookup.cpp" in cmake
    assert "src/robot_pose_model.cpp" in cmake
    assert "src/runtime_process_utils.cpp" in cmake
    assert "src/semantic_layer_io.cpp" in cmake
    assert "src/storage_models.cpp" in cmake
    assert "src/subscription_api.cpp" in cmake
    assert "src/subscription_manager.cpp" in cmake
    assert "src/tf_pose_utils.cpp" in cmake
    assert "target_include_directories(robot_api_server_node PRIVATE include)" in cmake
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
    assert '#include "robot_api_server/api_time_utils.hpp"' in node_cpp
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
    assert "std::string utc_timestamp_compact() const" not in node_cpp
    assert "std::string utc_timestamp_iso8601() const" not in node_cpp
    assert "double wall_time_seconds() const" not in node_cpp
    assert "std::string generated_current_pose_id" not in node_cpp
    assert '#include "robot_api_server/file_utils.hpp"' in node_cpp
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
    assert "std::string read_text_file(const fs::path" not in node_cpp
    assert "std::string read_optional_text_file(const fs::path" not in node_cpp
    assert "void write_text_file(const fs::path" not in node_cpp
    assert "void write_pgm_file(" not in node_cpp
    assert "std::string yaml_with_image_file" not in node_cpp
    assert '#include "robot_api_server/floor_asset_resolver.hpp"' in node_cpp
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
    assert "struct FloorAssetPaths" not in node_cpp
    assert "bool resolve_floor_asset_paths" not in node_cpp
    assert "fs::path poses_yaml_path" not in node_cpp
    assert "std::optional<StoredPose> find_floor_pose" not in node_cpp
    assert '#include "robot_api_server/navigation_cancel_job_model.hpp"' in node_cpp
    assert "struct NavigationCancelJob" in navigation_cancel_header
    assert "navigation_cancel_job_json" in navigation_cancel_header
    assert "std::string navigation_cancel_job_json" in navigation_cancel_cpp
    assert "cancel_all_detail" in navigation_cancel_cpp
    assert "navigation_stack_stopped" in navigation_cancel_cpp
    assert "struct NavigationCancelJob" not in node_cpp
    assert "std::string navigation_cancel_job_json(const NavigationCancelJob" not in node_cpp
    assert '#include "robot_api_server/docking_job_model.hpp"' in node_cpp
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
    assert "struct DockingJob" not in node_cpp
    assert "std::string docking_job_json(const DockingJob" not in node_cpp
    assert "socket(AF_INET, SOCK_STREAM" in node_cpp
    assert "Sec-WebSocket-Accept" in node_cpp
    assert "websocket_accept_key" in node_cpp
    assert '#include "robot_api_server/http_common.hpp"' in node_cpp
    assert "struct HttpRequest" in http_header
    assert "struct HttpResponse" in http_header
    assert "struct WebSocketFrame" in http_header
    assert "parse_http_request" in http_cpp
    assert "content_length_from_headers" in http_cpp
    assert "reason_phrase" in http_cpp
    assert "websocket_accept_key" in http_cpp
    assert "json_string_value" in http_cpp
    assert "json_object_array_value" in http_cpp
    assert "struct HttpRequest" not in node_cpp
    assert "std::optional<HttpRequest> parse_http_request" not in node_cpp
    assert '#include "robot_api_server/storage_models.hpp"' in node_cpp
    assert "struct StoredPose" in storage_header
    assert "struct MapManifest" in storage_header
    assert "struct RuntimeMapContext" in storage_header
    assert "struct MapYamlInfo" in storage_header
    assert "safe_pose_id" in storage_cpp
    assert "safe_asset_id" in storage_cpp
    assert "valid_display_map_name" in storage_cpp
    assert "safe_file_stem_from_display_name" in storage_cpp
    assert "fnv1a64" in storage_cpp
    assert "fixed_hex" in storage_cpp
    assert "struct StoredPose" not in node_cpp
    assert "struct MapManifest" not in node_cpp
    assert "bool safe_pose_id" not in node_cpp
    assert "bool safe_asset_id" not in node_cpp
    assert '#include "robot_api_server/map_asset_io.hpp"' in node_cpp
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
    assert "std::optional<MapYamlInfo> read_nav_map_info" not in node_cpp
    assert "std::optional<std::pair<std::uint32_t, std::uint32_t>> read_pgm_dimensions" not in node_cpp
    assert '#include "robot_api_server/map_asset_writer.hpp"' in node_cpp
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
    assert "std::uint8_t occupancy_to_gray" not in node_cpp
    assert "std::vector<std::uint8_t> occupancy_grid_to_image_pixels" not in node_cpp
    assert "std::string map_yaml_text" not in node_cpp
    assert '#include "robot_api_server/map_catalog.hpp"' in node_cpp
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
    assert "std::vector<MapManifest> read_floor_map_manifests" not in node_cpp
    assert "std::vector<MapManifest> read_all_map_manifests" not in node_cpp
    assert "std::optional<MapManifest> find_map_by_id" not in node_cpp
    assert "std::optional<MapManifest> active_floor_map" not in node_cpp
    assert "std::optional<MapManifest> unique_active_map_manifest" not in node_cpp
    assert "fs::path floor_root_path" not in node_cpp
    assert '#include "robot_api_server/map_manifest_io.hpp"' in node_cpp
    assert "fill_manifest_paths" in manifest_io_header
    assert "map_manifest_json" in manifest_io_header
    assert "read_map_manifest" in manifest_io_header
    assert "write_map_manifest" in manifest_io_header
    assert "manifest.nav_map_yaml" in manifest_io_cpp
    assert "manifest.localizer_map_png" in manifest_io_cpp
    assert "manifest.asset_report_json" in manifest_io_cpp
    assert "json_string_value" in manifest_io_cpp
    assert "json_bool_value" in manifest_io_cpp
    assert "std::optional<MapManifest> read_map_manifest" not in node_cpp
    assert "void write_map_manifest" not in node_cpp
    assert "std::string map_manifest_json" not in node_cpp
    assert "void fill_manifest_paths" not in node_cpp
    assert '#include "robot_api_server/runtime_map_context_io.hpp"' in node_cpp
    assert "write_runtime_map_context_file" in runtime_context_header
    assert "read_runtime_map_context_file" in runtime_context_header
    assert "njrh.runtime_map_context.v1" in runtime_context_cpp
    assert "json_string(state)" in runtime_context_cpp
    assert 'json_bool_value(text, "confirmed", false)' in runtime_context_cpp
    assert "safe_asset_id(*map_id)" in runtime_context_cpp
    assert "write_runtime_map_context_file(" in node_cpp
    assert "read_runtime_map_context_file(" in node_cpp
    assert "njrh.runtime_map_context.v1" not in node_cpp
    assert '#include "robot_api_server/runtime_map_lookup.hpp"' in node_cpp
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
    assert "std::optional<fs::path> resolve_mapping_2d_png" not in node_cpp
    assert "std::optional<fs::path> newest_png_in_directory" not in node_cpp
    assert "std::vector<fs::path> runtime_map_asset_paths" not in node_cpp
    assert '#include "robot_api_server/runtime_process_utils.hpp"' in node_cpp
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
    assert "void set_close_on_exec" not in node_cpp
    assert "void close_inherited_fds" not in node_cpp
    assert "std::string read_proc_cmdline" not in node_cpp
    assert "bool process_group_has_live_process" not in node_cpp
    assert '#include "robot_api_server/semantic_layer_io.hpp"' in node_cpp
    assert "keepout_semantic_json_path" in semantic_header
    assert "json_raw_or_null" in semantic_header
    assert "keepout_semantic_payload_json" in semantic_header
    assert "keepout_filter_json" in semantic_header
    assert '"keepout_semantic_layer.json"' in semantic_cpp
    assert 'json_object_value(semantic_json, "keepout")' in semantic_cpp
    assert "keepout_mask_yaml" in semantic_cpp
    assert "keepout_payload" in semantic_cpp
    assert "std::string json_raw_or_null" not in node_cpp
    assert "std::string keepout_semantic_payload_json" not in node_cpp
    assert "std::string keepout_filter_json" not in node_cpp
    assert '#include "robot_api_server/subscription_api.hpp"' in node_cpp
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
    assert "bool safe_client_id" not in node_cpp
    assert "int subscription_ttl_ms_from_body" not in node_cpp
    assert "std::vector<std::string> subscription_resources_from_body" not in node_cpp
    assert "std::pair<std::string, std::string> subscription_client_id_from_body" not in node_cpp
    assert "std::string resource_list_json" not in node_cpp
    assert '#include "robot_api_server/poses_io.hpp"' in node_cpp
    assert "poses_json_array" in poses_io_header
    assert "read_floor_poses" in poses_io_header
    assert "write_floor_poses" in poses_io_header
    assert "find_floor_pose" in poses_io_header
    assert "apply_pose_yaml_field" in poses_io_cpp
    assert 'yaml << "poses:\\n"' in poses_io_cpp
    assert "json_string(pose.id)" in poses_io_cpp
    assert "normalize_angle(pose.yaw)" in poses_io_cpp
    assert "std::vector<StoredPose> read_floor_poses" not in node_cpp
    assert "void write_floor_poses" not in node_cpp
    assert "std::string poses_json_array" not in node_cpp
    assert "void apply_pose_yaml_field" not in node_cpp
    assert '#include "robot_api_server/subscription_manager.hpp"' in node_cpp
    assert "class SubscriptionManager" in subscription_header
    assert "validate_resources" in subscription_header
    assert "snapshot_json" in subscription_header
    assert "leases_" in subscription_header
    assert "SubscriptionManager::acquire" in subscription_cpp
    assert "SubscriptionManager::expire" in subscription_cpp
    assert "json_string(resource)" in subscription_cpp
    assert "class SubscriptionManager" not in node_cpp
    assert "std::map<std::string, std::map<std::string, Clock::time_point>> leases_" not in node_cpp
    assert '#include "robot_api_server/tf_pose_utils.hpp"' in node_cpp
    assert '#include "robot_api_server/tf_pose_utils.hpp"' in poses_io_cpp
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
    assert "std::string normalized_frame_id(std::string frame) const" not in node_cpp
    assert "double normalize_angle(const double angle) const" not in node_cpp
    assert "double stamp_to_seconds(const builtin_interfaces::msg::Time" not in node_cpp
    assert "double older_nonzero_stamp(const double lhs" not in node_cpp
    assert "double quaternion_yaw(const double x" not in node_cpp
    assert '#include "robot_api_server/docking_status_utils.hpp"' in node_cpp
    assert "docking_status_is_success" in docking_status_header
    assert "docking_status_is_failure" in docking_status_header
    assert "docking_status_is_undocking" in docking_status_header
    assert "docking_status_is_undocked" in docking_status_header
    assert "docking_status_is_undock_failed" in docking_status_header
    assert "docking_status_is_stopped" in docking_status_header
    assert 'starts_with(status, "docked")' in docking_status_cpp
    assert 'starts_with(status, "charging")' in docking_status_cpp
    assert 'starts_with(status, "undocking")' in docking_status_cpp
    assert 'starts_with(status, "undocked")' in docking_status_cpp
    assert 'starts_with(status, "undock_failed")' in docking_status_cpp
    assert 'status.find("stopped")' in docking_status_cpp
    assert "bool docking_status_is_success(const std::string & status) const" not in node_cpp
    assert "bool docking_status_is_failure(const std::string & status) const" not in node_cpp
    assert "bool docking_status_is_undocking(const std::string & status) const" not in node_cpp
    assert "bool docking_status_is_undocked(const std::string & status) const" not in node_cpp
    assert "bool docking_status_is_stopped(const std::string & status) const" not in node_cpp
    assert "configure_runtime_permissions" in node_cpp
    assert "robot_api_server refuses to run as root" not in node_cpp
    assert "NJRH_ALLOW_ROOT_API_SERVER" not in node_cpp
    assert "::umask(0002)" in node_cpp
    assert "geometry_msgs::msg::Twist" in node_cpp
    assert "sensor_msgs::msg::BatteryState" in node_cpp
    assert "sensor_msgs::msg::LaserScan" in node_cpp
    assert "tf2_msgs::msg::TFMessage" in node_cpp
    assert "/ws/v1/teleop" in node_cpp
    assert "/cmd_vel_collision_checked" in node_cpp
    assert "std_msgs::msg::Bool" in node_cpp
    assert "robot_interfaces::srv::SwitchFloor" in node_cpp
    assert "robot_interfaces::srv::TriggerLocalization" in node_cpp
    assert "geometry_msgs::msg::PoseWithCovarianceStamped" in node_cpp
    assert "/api/v1/status" in node_cpp
    assert "bms_state_topic" in node_cpp
    assert "teleop_stop_on_charging" in node_cpp
    assert "teleop_charging_current_min_a" in node_cpp
    assert "bms_charging_contact_voltage_min_v" in node_cpp
    assert "bms_charging_contact_voltage_max_v" in node_cpp
    assert "bms_full_soc_threshold_pct" in node_cpp
    assert "bms_full_soc_voltage_contact_enable" in node_cpp
    assert "battery_indicates_charging" in node_cpp
    assert "battery_charging_contact" in node_cpp
    assert '#include "robot_api_server/bms_contact.hpp"' in node_cpp
    assert "BatteryContactEvaluation" in bms_header
    assert "evaluate_battery_charging_contact" in bms_cpp
    assert "POWER_SUPPLY_STATUS_CHARGING" in bms_cpp
    assert "POWER_SUPPLY_STATUS_FULL" in bms_cpp
    assert "present_voltage_valid" in bms_cpp
    assert "full_soc_present_voltage_valid" in bms_cpp
    assert "current_above_threshold" in bms_cpp
    assert "bms_charging_contact_active" in node_cpp
    assert "teleop_charging_guard_active" in node_cpp
    assert "charging detected; teleop command stopped" in node_cpp
    assert "\\\"bms\\\"" in node_cpp
    assert "\\\"soc\\\"" in node_cpp
    assert "\\\"soc_valid\\\"" in node_cpp
    assert "\\\"power_supply_status\\\"" in node_cpp
    assert "\\\"power_supply_health\\\"" in node_cpp
    assert "\\\"power_supply_technology\\\"" in node_cpp
    assert "\\\"present\\\"" in node_cpp
    assert "\\\"charging_contact\\\"" in node_cpp
    assert "\\\"charging_contact_reason\\\"" in node_cpp
    assert "\\\"inferred_docked\\\"" in node_cpp
    assert '#include "robot_api_server/robot_pose_model.hpp"' in node_cpp
    assert "struct RobotPoseSnapshot" in robot_pose_header
    assert "struct RobotPoseMapIdentity" in robot_pose_header
    assert "no_fresh_map_robot_pose_json" in robot_pose_header
    assert "robot_pose_json" in robot_pose_header
    assert "std::string no_fresh_map_robot_pose_json" in robot_pose_cpp
    assert "std::string robot_pose_json" in robot_pose_cpp
    assert "no fresh map-frame robot pose" in robot_pose_cpp
    assert "\\\"age_sec\\\":null" in robot_pose_cpp
    assert "\\\"map_id\\\":" in robot_pose_cpp
    assert "struct RobotPoseSnapshot" not in node_cpp
    assert "std::string no_fresh_map_robot_pose_json()" not in node_cpp
    assert "std::string no_fresh_map_robot_pose_json(const std::string & detail)" not in node_cpp
    assert "/api/v1/robot/pose" in node_cpp
    assert "handle_robot_pose" in node_cpp
    assert "wait_for_current_robot_pose" in node_cpp
    assert "robot_pose_freshness_sec" in node_cpp
    assert "TfChainFreshnessSnapshot" in node_cpp
    assert "tf_chain_freshness_sec" in node_cpp
    assert "tf_chain_settle_timeout_sec" in node_cpp
    assert "wait_for_fresh_tf_chain" in node_cpp
    assert '"navigation goal"' in node_cpp
    assert '"docking predock navigation"' in node_cpp
    assert '"docking fine docking"' in node_cpp
    assert '#include "robot_api_server/localization_result_model.hpp"' in node_cpp
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
    assert "struct LocalizationResultSnapshot" not in node_cpp
    assert '#include "lifecycle_msgs/srv/get_state.hpp"' in node_cpp
    assert "navigation_relocalize_before_goal" in node_cpp
    assert "navigation_relocalize_before_goal_always" in node_cpp
    assert "navigation_relocalize_before_goal_required" in node_cpp
    assert "navigation_relocalize_wait_sec" in node_cpp
    assert "navigation_goal_relocalization_decision" in node_cpp
    assert "force_relocalize" in node_cpp
    assert "confirmed runtime map context and fresh map-frame pose" in node_cpp
    assert "target floor differs from confirmed runtime map context" not in node_cpp
    assert "requested pose target does not match confirmed runtime map context" in node_cpp
    assert "if (navigation_relocalize_before_goal_ && !pre_navigation_undock)" not in node_cpp
    assert "navigation_lifecycle_check_timeout_sec" in node_cpp
    assert "navigation_lifecycle_snapshot" in node_cpp
    assert "navigation lifecycle inactive" in node_cpp
    assert "navigation requires fresh localization before goal" in node_cpp
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
    assert "validate_manual_docking_predock_pose" in node_cpp
    assert "manual predock pose" in node_cpp
    assert "failed docking sanity check" in node_cpp
    assert "distance_check=disabled" in node_cpp
    assert "heading aligned to the charger" in node_cpp
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
    assert "manual_predock_explicit" in node_cpp
    assert "manual_predock_auto_id" in node_cpp
    assert "manual_predock_auto_unique_type" in node_cpp
    assert "dock_predock" in node_cpp
    assert "dock_approach" in node_cpp
    assert "computed_from_dock_pose" in node_cpp
    assert "multiple dock_predock poses found" in node_cpp
    assert "relocalize_after_fine_docking" in node_cpp
    assert "docking_after_fine:" in node_cpp
    assert "relocalized after fine docking" in node_cpp
    assert 'status.find("not_found")' in docking_status_cpp
    assert "post_undock_relocalization_requested" in node_cpp
    assert "post_undock_relocalization_succeeded" in node_cpp
    assert "post_predock_relocalization_requested" in node_cpp
    assert "post_predock_relocalization_succeeded" in node_cpp
    assert "docking_cancel_active_goal_before_predock" in node_cpp
    assert "docking canceled before GS2 fine docking start" in node_cpp
    assert "docking_stop_service_wait_sec" in node_cpp
    assert "docking_stop_client_->wait_for_service(docking_stop_service_wait())" in node_cpp
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
    assert "handle_navigation_goal" in node_cpp
    assert "handle_navigation_cancel" in node_cpp
    assert "NavigateToPose" in node_cpp
    assert "navigate_to_pose_action" in node_cpp
    assert "navigate_action_mutex_" in node_cpp
    assert "active_nav_goal_handle_" in node_cpp
    assert "struct NavigationGoalJob" in node_cpp
    assert "navigation_goal_job_json_locked" in node_cpp
    assert "navigation_goal_id" in node_cpp
    assert "run_navigation_goal_job" in node_cpp
    assert "run_final_yaw_align" in node_cpp
    assert "publish_final_yaw_align_command" in node_cpp
    assert "navigation_final_yaw_cmd_pub_" in node_cpp
    assert "navigation_goal_position_success_tolerance_m" in node_cpp
    assert "navigation_final_yaw_align_enable" in node_cpp
    assert "navigation_final_yaw_align_trigger_rad" in node_cpp
    assert "navigation_final_yaw_align_kp" in node_cpp
    assert "navigation_final_yaw_align_max_xy_drift_m" in node_cpp
    assert "navigation_final_yaw_align_cmd_topic" in node_cpp
    assert 'navigation_final_yaw_align_cmd_topic_ != "/cmd_vel_nav"' in node_cpp
    assert 'navigation_final_yaw_align_cmd_topic_ != "/cmd_vel_collision_checked"' in node_cpp
    assert 'navigation_final_yaw_align_cmd_topic_ = "/cmd_vel_collision_checked"' in node_cpp
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe"' not in node_cpp
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel"' not in node_cpp
    assert "position_reached_yaw_warning" in node_cpp
    assert "position_reached_verifying" in node_cpp
    assert "position_reached_yaw_aligning" in node_cpp
    assert "final_pose_verifying" in node_cpp
    assert "final_pose_verified" in node_cpp
    assert "failed_final_pose_verify" in node_cpp
    assert "blocked_by_safety" in node_cpp
    assert "navigation position reached; final yaw alignment warning" in node_cpp
    assert "bool position_reached = pose_check.position_reached;" in node_cpp
    assert "bool position_reached = nav2_succeeded" not in node_cpp
    assert "navigation reported success but final position is outside tolerance" in node_cpp
    assert '"failed_position"' in node_cpp
    assert '"position_not_reached"' not in node_cpp[node_cpp.index("void run_navigation_goal_job("):]
    assert "final_yaw_align_blocked" in node_cpp
    assert "final_yaw_align_attempted" in node_cpp
    assert "final_yaw_align_blocked_reason" in node_cpp
    assert "final_yaw_align_duration_sec" in node_cpp
    assert "final_yaw_align_target_yaw_rad" in node_cpp
    assert "final_yaw_align_initial_yaw_error_rad" in node_cpp
    assert "final_yaw_align_final_yaw_error_rad" in node_cpp
    assert "final_yaw_align_observed_xy_drift_m" in node_cpp
    assert "final_yaw_align_bypass_collision_monitor" in node_cpp
    assert "final_pose_verify_reason" in node_cpp
    assert "request_navigation_goal_cancel" in node_cpp
    assert "navigation_goal_cancel_requested" in node_cpp
    assert "exception sending navigation goal" in node_cpp
    assert "async_cancel_goal" in node_cpp
    assert "async_cancel_all_goals" in node_cpp
    assert "exception canceling cached goal handle" in node_cpp
    assert "exception canceling navigation goals" in node_cpp
    assert "navigation_cancel_action_wait_sec" in node_cpp
    assert "navigation_cancel_action_wait()" in node_cpp
    assert "unhandled API exception" in node_cpp
    assert "log_http_request" in node_cpp
    assert "::listen(server_fd_, 64)" in node_cpp
    assert "SO_RCVTIMEO" in node_cpp
    assert "SO_SNDTIMEO" in node_cpp
    assert "failed to send full HTTP response" in node_cpp
    assert "Taking data from action client but no ready event" in node_cpp
    assert "continuing after transient action client executor exception" in node_cpp
    assert "SingleThreadedExecutor executor" in node_cpp
    assert "set_close_on_exec(server_fd_)" in node_cpp
    assert "prepare_child_process(" in node_cpp
    assert "timed out waiting for navigation stop command" in node_cpp
    assert "publish_teleop_zero_burst" in node_cpp
    assert "zero_velocity_published" in node_cpp
    assert "action_available" in node_cpp
    assert "cancel_all_detail" in node_cpp
    assert "navigation_stop_command" in node_cpp
    assert "stop_navigation_runtime_stack" in node_cpp
    assert "cancel_requested" in node_cpp
    assert "navigation_stack_stopped" in navigation_cancel_cpp
    assert "stop_stack" in node_cpp
    assert "read_floor_poses" in node_cpp
    assert "write_floor_poses" in node_cpp
    assert "/api/v1/subscriptions/acquire" in node_cpp
    assert "/api/v1/subscriptions/release" in node_cpp
    assert "/api/v1/subscriptions/heartbeat" in node_cpp
    assert "set_subscription_resource_active" in node_cpp
    assert "Safety and floor health are process-level health inputs" in node_cpp
    assert "set_status_subscriptions_active(true);" in node_cpp
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()" in node_cpp
    assert "refresh_navigation_resume_runtime_state(false);" in node_cpp
    assert "void refresh_navigation_resume_runtime_state(const bool probe_lifecycle = false)" in node_cpp
    assert "http_worker_loop" in node_cpp
    assert "http_client_queue_" in node_cpp
    assert ".detach()" not in node_cpp
    assert "live_map resource is not acquired" in node_cpp
    assert "subscription_manager_->release(websocket_client_id" in node_cpp
    assert "subscription_client_id_from_body" in node_cpp
    assert "clientId" in subscription_api_cpp
    assert "lease_id" in node_cpp
    assert "http:compat-default" in subscription_api_cpp
    assert "resources_for_client(client_id)" in node_cpp
    assert "\\\"refreshed\\\":false" in node_cpp
    assert "/api/v1/mapping/2d/map" in node_cpp
    assert "image/png" in node_cpp
    assert "nav_msgs::msg::OccupancyGrid" in node_cpp
    assert "encode_grayscale_png" in node_cpp
    assert "handle_start_mapping_2d" in node_cpp
    assert "handle_stop_mapping_2d" in node_cpp
    assert "handle_save_mapping_2d" in node_cpp
    assert "clear_runtime_map_context();" in node_cpp
    start_mapping_section = node_cpp[
        node_cpp.index("HttpResponse handle_start_mapping_2d()"):
        node_cpp.index("HttpResponse handle_stop_mapping_2d()")
    ]
    assert "cancel_navigation_task_for_mode_switch" not in start_mapping_section
    assert "stop_navigation_runtime_stack" not in start_mapping_section
    assert "canceling active navigation task before 2D mapping" not in start_mapping_section
    assert "paused_for_mapping" not in start_mapping_section
    assert "cannot start 2D mapping while navigation runtime is active; stop navigation runtime first" in start_mapping_section
    assert "cancel_navigation_task_for_mode_switch" in node_cpp
    assert 'if (!runtime_mode_snapshot().navigation_active)' in node_cpp
    assert "requires_manual_navigation_selection" in node_cpp
    assert "write_last_navigation_map_selection" in node_cpp
    assert "njrh.last_navigation_map.v1" in node_cpp
    assert "2D map saved and activated" not in node_cpp
    assert "handle_delete_map" in node_cpp
    assert "runtime_map_asset_paths" in runtime_map_lookup_cpp
    assert "MapManifest" in node_cpp
    assert "generate_map_id" in map_catalog_cpp
    assert "display_name" in node_cpp
    assert "floor_maps" in node_cpp
    assert "map_info" in node_cpp
    assert "read_nav_map_info" in node_cpp
    assert "read_pgm_dimensions" not in node_cpp
    assert "manifest.json" in node_cpp
    assert "maps/<map_id>" not in node_cpp
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
    assert "terminate_mapping_2d_process_groups_locked" in node_cpp
    assert "discover_mapping_2d_process_groups" in node_cpp
    assert "terminate_mapping_2d_residual_processes" in node_cpp
    assert "discover_mapping_2d_residual_processes" in node_cpp
    assert "mapping_was_active" in node_cpp
    assert "stopped_groups" in node_cpp
    stop_mapping_section = node_cpp[
        node_cpp.index("HttpResponse handle_stop_mapping_2d()"):
        node_cpp.index("bool navigation_resume_process_running_locked()")
    ]
    assert 'set_mapping_runtime_state(true, "stopping"' not in stop_mapping_section
    mapping_residual_section = node_cpp[
        node_cpp.index("bool is_mapping_2d_residual_process_command"):
        node_cpp.index("std::set<pid_t> discover_mapping_2d_process_groups")
    ]
    assert '"fastlio_mapping"' not in mapping_residual_section
    assert '"laser_mapping"' not in mapping_residual_section
    assert '"run_fastlio_tf.sh"' not in mapping_residual_section
    assert '"fastlio_mapping_odom_bridge.py"' in mapping_residual_section
    assert '"/mapping/fastlio_odometry"' in mapping_residual_section
    assert '"/tf_slam2d"' in mapping_residual_section
    assert '"ros2 run fast_lio fastlio_mapping"' not in mapping_residual_section
    private_fastlio_section = node_cpp[
        node_cpp.index("bool is_private_slam2d_fastlio_process"):
        node_cpp.index("bool is_mapping_2d_residual_process_command")
    ]
    assert '"ros2 run fast_lio fastlio_mapping"' in private_fastlio_section
    assert '"fast_lio/lib/fast_lio/fastlio_mapping"' in private_fastlio_section
    assert '"fastlio_mapping --ros-args"' in private_fastlio_section
    assert "NJRH_SLAM2D_PRIVATE_FASTLIO=1" in private_fastlio_section
    assert '"fastlio_odom_bridge_node"' not in mapping_residual_section
    assert '"nav_cloud_preprocessor"' not in mapping_residual_section
    assert '"pointcloud_to_laserscan_node"' not in mapping_residual_section
    assert '"scan_republisher_node"' not in mapping_residual_section
    assert "mapping_2d_start_command" in node_cpp
    assert "navigation_resume_command" in node_cpp
    assert "handle_resume_floor_navigation" in node_cpp
    assert "run_navigation_runtime_services.sh" in node_cpp
    assert "run_projected_map.sh" in node_cpp
    assert "jt128_slam_toolbox_mapping.launch.py" in node_cpp
    assert "run_navigation_cancel_job_guarded" in node_cpp
    assert "navigation cancel worker exception" in node_cpp
    assert "run_docking_job_guarded" in node_cpp
    assert "docking worker exception" in node_cpp
    assert "live slam_toolbox /map" in node_cpp
    assert "newest_png_in_directory" in runtime_map_lookup_cpp
    assert "resolve_mapping_2d_png" in runtime_map_lookup_cpp
    assert "/api/v1/safety/stop" in node_cpp
    assert "/api/v1/safety/resume" in node_cpp
    assert "/api/v1/floors/switch" in node_cpp
    assert "/api/v1/localization/trigger" in node_cpp
    assert "endpoint is reserved but not wired to a ROS-native service/action yet" in node_cpp
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
    assert "navigation_relocalize_before_goal: true" in config
    assert "navigation_relocalize_before_goal_always: false" in config
    assert "navigation_relocalize_before_goal_required: true" in config
    assert "navigation_relocalize_wait_sec: 8.0" in config
    assert "navigation_lifecycle_check_timeout_sec: 0.35" in config
    assert "navigation_goal_result_timeout_sec: 600.0" in config
    assert "navigation_goal_position_success_tolerance_m: 0.20" in config
    assert "navigation_final_yaw_align_enable: true" in config
    assert "navigation_final_yaw_tolerance_rad: 0.05" in config
    assert "navigation_final_yaw_align_trigger_rad: 0.08" in config
    assert "navigation_final_yaw_align_speed_radps: 0.25" in config
    assert "navigation_final_yaw_align_min_speed_radps: 0.06" in config
    assert "navigation_final_yaw_align_kp: 1.2" in config
    assert "navigation_final_yaw_align_max_speed_radps: 0.25" in config
    assert "navigation_final_yaw_align_timeout_sec: 8.0" in config
    assert "navigation_final_yaw_align_max_xy_drift_m: 0.08" in config
    assert "navigation_final_yaw_align_require_fresh_pose: true" in config
    assert "navigation_final_yaw_align_cmd_topic: \"/cmd_vel_collision_checked\"" in config
    assert "navigation_final_yaw_align_bypass_collision_monitor: true" in config
    assert "navigation_final_yaw_align_zero_cmd_count: 3" in config
    assert "docking_relocalize_before_predock: true" in config
    assert "docking_relocalize_after_predock: true" in config
    assert "docking_relocalize_after_predock_required: true" in config
    assert "docking_relocalize_after_fine_docking: true" in config
    assert "docking_relocalize_after_fine_docking_required: false" in config
    assert "docking_validate_predock_pose_after_relocalization: true" in config
    assert "docking_predock_pose_max_distance_m: 0.35" in config
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
    assert "refresh_mapping_2d_runtime_state();" in node_cpp
    assert "recover_mapping_2d_process_locked" in node_cpp
    assert "set_mapping_live_map_cache_active(true)" in node_cpp
    assert "live_map_mapping_cache_active_" in node_cpp
    assert "live_map_page_subscription_active_" in node_cpp
    assert "2D mapping process discovered" in node_cpp
    assert "\\\"live_map_available\\\"" in node_cpp
    assert "\\\"live_map_age_sec\\\"" in node_cpp
    assert "scan_topic: \"/scan\"" in config
    assert "tf_topic: \"/tf\"" in config
    assert "subscription_default_ttl_ms: 10000" in config
    assert "teleop_cmd_topic: \"/cmd_vel_collision_checked\"" in config
    assert "teleop_reverse_enable_topic: \"/ranger_mini3/teleop_allow_reverse\"" in config
    assert "teleop_pose_topic: \"/local_state/odometry\"" in config
    assert "teleop_max_linear_x_mps: 1.00" in config
    assert "teleop_allow_reverse: true" in config
    assert "teleop_require_mapping_active: true" in config
    assert "teleop_watchdog_timeout_sec: 0.5" in config
    assert "teleop_socket_idle_timeout_sec: 5.0" in config
    assert "teleop_repeat_rate_hz: 20.0" in config
    assert "on_teleop_repeat_timer" in node_cpp
    assert "teleop_session_active()" in node_cpp
    assert "charging_contact.contact && teleop_stop_on_charging_ && teleop_session_active()" in node_cpp
    assert "if (!teleop_session_active())" in node_cpp
    assert "store_teleop_command(twist)" in node_cpp
    assert "clear_teleop_command()" in node_cpp
    assert "set_socket_receive_timeout(client_fd, teleop_socket_idle_timeout_sec_)" in node_cpp
    assert "json_nested_number_value(payload, \"linear\", \"x\")" in node_cpp
    assert "json_nested_number_value(payload, \"angular\", \"z\")" in node_cpp
    assert "json_number_value(payload, \"linearX\")" in node_cpp
    assert "json_number_value(payload, \"angularZ\")" in node_cpp
    assert "if (payload_length > 4096U)" in node_cpp
    assert "if (masked && !recv_exact(client_fd, mask, sizeof(mask)))" in node_cpp
    assert "if (masked) {" in node_cpp
    assert "if (!masked || payload_length > 4096U)" not in node_cpp
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
    assert "navigation_relocalize_before_goal: true" in overlay_config
    assert "navigation_relocalize_before_goal_always: false" in overlay_config
    assert "navigation_relocalize_before_goal_required: true" in overlay_config
    assert "navigation_relocalize_wait_sec: 8.0" in overlay_config
    assert "navigation_lifecycle_check_timeout_sec: 0.35" in overlay_config
    assert "navigation_goal_result_timeout_sec: 600.0" in overlay_config
    assert "navigation_goal_position_success_tolerance_m: 0.20" in overlay_config
    assert "navigation_final_yaw_align_enable: true" in overlay_config
    assert "navigation_final_yaw_tolerance_rad: 0.05" in overlay_config
    assert "navigation_final_yaw_align_trigger_rad: 0.08" in overlay_config
    assert "navigation_final_yaw_align_speed_radps: 0.25" in overlay_config
    assert "navigation_final_yaw_align_min_speed_radps: 0.06" in overlay_config
    assert "navigation_final_yaw_align_kp: 1.2" in overlay_config
    assert "navigation_final_yaw_align_max_speed_radps: 0.25" in overlay_config
    assert "navigation_final_yaw_align_timeout_sec: 8.0" in overlay_config
    assert "navigation_final_yaw_align_max_xy_drift_m: 0.08" in overlay_config
    assert "navigation_final_yaw_align_require_fresh_pose: true" in overlay_config
    assert "navigation_final_yaw_align_cmd_topic: \"/cmd_vel_collision_checked\"" in overlay_config
    assert "navigation_final_yaw_align_bypass_collision_monitor: true" in overlay_config
    assert "navigation_final_yaw_align_zero_cmd_count: 3" in overlay_config
    assert "localization_result_topic: \"/localization_result\"" in overlay_config
    assert "docking_relocalize_before_predock: true" in overlay_config
    assert "docking_relocalize_after_predock: true" in overlay_config
    assert "docking_relocalize_after_predock_required: true" in overlay_config
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
    assert "teleop_cmd_topic: \"/cmd_vel_collision_checked\"" in overlay_config
    assert "teleop_reverse_enable_topic: \"/ranger_mini3/teleop_allow_reverse\"" in overlay_config
    assert "teleop_max_linear_x_mps: 1.00" in overlay_config
    assert "teleop_socket_idle_timeout_sec: 5.0" in overlay_config
    assert "teleop_repeat_rate_hz: 20.0" in overlay_config
    assert "ROBOT_API_TOKEN" in overlay_script
    assert 'export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"' in overlay_script
    assert 'export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"' in overlay_script
    assert overlay_script.index("AMENT_TRACE_SETUP_FILES") < overlay_script.index('source "${SCRIPT_DIR}/common_env.sh"')
    assert "robot_api_server_node" in overlay_script
    assert "cleanup_stale_api_processes()" in overlay_supervisor_script
    assert 'pkill -TERM -f "${pattern}"' in overlay_supervisor_script
    assert 'pkill -KILL -f "${pattern}"' in overlay_supervisor_script
    assert "ros2 run robot_api_server robot_api_server_node" in overlay_supervisor_script
    assert "colcon build --packages-select robot_interfaces robot_api_server" in overlay_script
    assert "umask 0002" in overlay_script
    assert "compatibility wrapper" in floor_navigation_script
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
    assert "resident navigation runtime ready after trigger wrapper, bridge map->odom, and Nav2 activation" in resident_runtime_script
    assert "resident_navigation_ready()" in resident_runtime_script
    assert "navigation_runtime_ready_for_current_floor 3" not in resident_runtime_script
    assert "wait_for_resident_navigation_runtime_ready()" not in resident_runtime_script
    assert "ensure_navigation_layer_alive()" in resident_runtime_script
    assert 'wait_for_resident_navigation_runtime_ready "${NJRH_NAV_RUNTIME_READY_TIMEOUT:-300}"' not in resident_runtime_script
    assert "resident navigation runtime did not become ready within" not in resident_runtime_script
    assert "ensure_localization_stack_ready_for_navigation()" in resident_runtime_script
    assert "wait_for_nav2_layer_ready()" in resident_runtime_script
    assert 'wait_for_ros_service "/global_localization/trigger"' in resident_runtime_script
    assert 'wait_for_ros_service "/trigger_grid_search_localization"' in resident_runtime_script
    assert 'wait_for_topic_message "/flatscan" "${flatscan_timeout}"' in resident_runtime_script
    assert 'NJRH_INITIAL_LOCALIZATION_FLATSCAN_MAX_AGE_SEC' not in resident_runtime_script
    assert "wait_for_bridge_has_map_to_odom()" in resident_runtime_script
    assert "trigger_output_reports_map_to_odom_ready()" in resident_runtime_script
    assert "wrapper already verified bridge map->odom readiness" in resident_runtime_script
    assert "map->odom ready owner=robot_localization_bridge" in resident_runtime_script
    assert "bridge_status.has_map_to_odom=true was not observed" in resident_runtime_script
    assert "map->odom TF was not published after bridge acceptance" in resident_runtime_script
    assert "BRIDGE_ACCEPT_TIMEOUT" in resident_runtime_script
    assert "BRIDGE_REJECTED_RESULT" in resident_runtime_script
    assert "MAP_TO_ODOM_WRONG_OWNER" in resident_runtime_script
    assert 'wait_for_fresh_header_topic_message \\' not in resident_runtime_script
    assert 'wait_for_tf_transform "map" "odom"' in resident_runtime_script
    assert 'wait_for_global_costmap_static "${costmap_timeout}"' in resident_runtime_script
    assert "trigger_global_localization_for_navigation()" in resident_runtime_script
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_RECHECK_TIMEOUT:-30" not in resident_runtime_script
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-60" in resident_runtime_script
    assert "resident_navigation_start:${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" in resident_runtime_script
    assert "resident localization layer already owns map/localizer loading" in resident_runtime_script
    assert (
        "payload=\"{building_id: '${NJRH_BUILDING_ID}', floor_id: '${NJRH_FLOOR_ID}', "
        "resume_navigation: false}\""
    ) in resident_runtime_script
    assert "resume_navigation: true}" not in resident_runtime_script
    resident_runtime_startup = resident_runtime_script.split('write_runtime_map_context "starting"', 1)[1]
    assert "ensure_localization_stack_ready_for_navigation ||" in resident_runtime_startup
    assert 'ensure_map_server_active "${NAV2_MAP_YAML:-}" "${map_server_timeout}"' in resident_runtime_script
    assert "MAP_SERVER_NOT_ACTIVE" in resident_runtime_script
    assert "initial global localization did not pass trigger wrapper, bridge, and map->odom gates" in resident_runtime_script
    assert resident_runtime_startup.index("/floor_manager/switch_floor") < resident_runtime_startup.index(
        "trigger_global_localization_for_navigation"
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
    assert 'ensure_map_server_active "${NAV2_MAP_YAML}" "${MAP_SERVER_READY_TIMEOUT}"' not in occupancy_localization_script
    assert "localization pointcloud startup probe disabled" in occupancy_localization_script
    assert "standard_nav_stack_ready()" in overlay_nav2_script
    assert "standard Nav2 navigation stack already ready; reusing existing stack" in overlay_nav2_script
    assert "NJRH_NAV_STOP_ZERO_TIMEOUT_SEC" in stop_navigation_script
    assert "NJRH_NAV_STOP_ZERO_TIMEOUT_SEC:-0.25s" in stop_navigation_script
    assert "NJRH_NAV_STOP_INT_WAIT_SEC:-1" in stop_navigation_script
    assert "NJRH_NAV_STOP_TERM_WAIT_SEC:-1" in stop_navigation_script
    assert "NJRH_NAV_STOP_KILL_WAIT_SEC:-1" in stop_navigation_script
    assert "run_navigation_runtime_services.sh" in stop_navigation_script
    assert "run_local_perception.sh" in stop_navigation_script
    assert "robot_local_perception/local_perception_node" in stop_navigation_script
    assert "clear_runtime_map_context()" in stop_navigation_script
    assert "rm -f \"${context_file}\"" in stop_navigation_script
    assert "The app should not join ROS 2 DDS directly" in app_doc
    assert "bms.soc" in app_doc
    assert "Ranger `/battery_state`" in app_doc
    assert "POST /api/v1/subscriptions/acquire" in app_doc
    assert "Page-Scoped Subscriptions" in app_doc
    assert "acquire `live_map`" in app_doc
    assert "must not bypass `robot_safety`" in app_doc
    assert "POST /api/v1/mapping/2d/start" in app_doc
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
    assert "position_reached_yaw_warning" in app_doc
    assert "final_yaw_align_blocked" in app_doc
    assert "Delivery success is position-first" in readme
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
    assert "/cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe" in app_doc
    assert "/ranger_mini3/teleop_allow_reverse" in app_doc
    assert "mapping_state" in app_doc
    assert "resume_navigation" in app_doc
    assert "start the occupancy localization stack" in app_doc
    assert "startup waits for the trigger wrapper to report a bridge-accepted `map -> odom`" in app_doc
    assert "run_occupancy_grid_localization.sh" in resident_runtime_script
    assert "run_nav2_navigation.sh" in resident_runtime_script
    assert "/floor_manager/switch_floor" in resident_runtime_script
    assert "global_localization_node" in global_localization_script
    assert "global_localization_node.py" not in global_localization_script
    assert "GLOBAL_LOCALIZATION_PARAMS_FILE" in global_localization_script
    assert "robot_global_localization/global_localization_node|" in nav_runtime_helpers
    assert "/trigger_grid_search_localization" in resident_runtime_script
    assert "trigger_global_localization_and_wait_for_result" not in resident_runtime_script
    assert "pre-armed localization_result subscribers" not in resident_runtime_script
    assert 'local call_timeout="${NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-60}"' in resident_runtime_script
    assert 'local bridge_timeout="${NJRH_INITIAL_LOCALIZATION_BRIDGE_ACCEPT_WAIT_SEC:-25}"' in resident_runtime_script
    assert 'local tf_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_ODOM_WAIT_SEC:-20}"' in resident_runtime_script
    assert "global localization wrapper accepted" in resident_runtime_script
    assert "trigger_output_reports_map_to_odom_ready" in resident_runtime_script
    assert "wrapper already verified bridge map->odom readiness" in resident_runtime_script
    assert "bridge_status.has_map_to_odom=true was not observed" in resident_runtime_script
    assert "map->odom TF was not published after bridge acceptance" in resident_runtime_script
    assert 'NJRH_GLOBAL_LOCALIZATION_TRIGGER_FALLBACK_TF_TIMEOUT:-20' not in resident_runtime_script
    assert "requesting global localization through wrapper and waiting for bridge/map->odom" in resident_runtime_script
    assert "starting resident Nav2 layer" in resident_runtime_script
    assert "runtime_ready=0" in resident_runtime_script
    assert "write_runtime_map_context \"failed\" \"false\"" in resident_runtime_script
    assert "resident navigation runtime failed; check" in resident_runtime_script
    assert '#include "std_srvs/srv/empty.hpp"' in global_localization_node
    assert "grid_search_trigger_service" in global_localization_node
    assert "MultiThreadedExecutor" in global_localization_node
    assert "async_send_request" in global_localization_node
    assert "wait_for_bridge_acceptance" in global_localization_node
    assert "wait_for_map_to_odom" in global_localization_node
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
    assert 'wait_for_topic_message "/flatscan" "${flatscan_timeout}"' in resident_runtime_script
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
    assert "local_perception_runtime_config_ready()" in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception restamp_to_now' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception restamp_to_latest_tf' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception require_output_stamp_tf' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception input_reliable' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception input_qos_depth' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception input_transform_use_latest' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception max_output_tf_stamp_age_sec' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception output_stamp_tf_backoff_sec' in nav_runtime_helpers
    assert 'ros2 param get /robot_local_perception output_stamp_forward_sec' in nav_runtime_helpers
    assert 'wait_for_topic_message "/perception/obstacle_points"' in nav_runtime_helpers
    assert "wait_for_local_costmap_observation_ready()" in nav_runtime_helpers
    assert 'wait_for_topic_message "/local_costmap/costmap"' in nav_runtime_helpers
    assert 'wait_for_fresh_tf_transform "odom" "base_link"' in nav_runtime_helpers
    assert "local costmap observation ready from runtime health snapshot" not in nav_runtime_helpers
    assert "odom_base_tf_not_fresh" in nav_runtime_helpers
    assert "from tf2_msgs.msg import TFMessage" not in nav_runtime_helpers
    assert "tf2_msgs::msg::TFMessage" in runtime_probe_cpp
    assert 'create_subscription<tf2_msgs::msg::TFMessage>' in runtime_probe_cpp
    assert "source=/tf" in runtime_probe_cpp
    assert "wait_for_transformable_obstacle_points" in nav_runtime_helpers
    assert "NJRH_LOCAL_COSTMAP_TF_BUFFER_WARMUP_SEC" in runtime_probe_cpp
    assert "tf_buffer_warmup" in runtime_probe_cpp
    assert "local_costmap_observation_tf_not_transformable" in nav_runtime_helpers
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
    assert "timeout 3 ros2 lifecycle get /map_server" in map_server_helpers
    assert "lifecycle state unavailable, but requested map is selected; continuing" in map_server_helpers
    assert 'wait_for_ros_service "/global_localization/trigger" "${NJRH_GLOBAL_LOCALIZATION_SERVICE_WAIT_SEC:-75}"' not in resident_runtime_script
    assert 'wait_for_ros_service "/trigger_grid_search_localization" "${NJRH_ISAAC_TRIGGER_SERVICE_WAIT_SEC:-75}"' not in resident_runtime_script
    assert 'wait_for_ros_service "/floor_manager/switch_floor" "${NJRH_FLOOR_MANAGER_SERVICE_WAIT_SEC:-45}"' not in resident_runtime_script
    assert "wait_for_tf_transform()" in nav_runtime_helpers
    assert "lookupTransform(target, source, tf2::TimePointZero" in runtime_probe_cpp
    assert "buffer.can_transform" not in nav_runtime_helpers
    assert "refresh_navigation_resume_runtime_state" in node_cpp
    assert "navigation runtime process exited before ready" in node_cpp
    assert 'context && !context->confirmed && context->state == "failed"' in node_cpp
    assert "navigation runtime failed before ready" in node_cpp
    assert "runtime_context_matches_resume_request" in node_cpp
    assert "navigation_runtime_reused" in node_cpp
    assert "reused" in node_cpp
    assert node_cpp.index("existing_resume_process_running") < node_cpp.index("terminate_navigation_resume_process_locked();")
    assert 'context->confirmed && context->state == "ready"' in node_cpp
    assert "action_server_is_ready" in node_cpp
    assert "navigation runtime ready" in node_cpp
    assert "const bool pre_nav_resume = request->resume_navigation" in floor_manager_code
    assert 'floor assets selected for next navigation: ' in floor_manager_code
    assert floor_manager_code.index('if (!pre_nav_resume) {') < floor_manager_code.index('if (!load_nav_map')
    assert "if (!pre_nav_resume && !load_filter_masks" in floor_manager_code
    assert "if (!pre_nav_resume && clear_costmaps_after_switch_)" in floor_manager_code
    assert resident_runtime_script.index("run_occupancy_grid_localization.sh") < resident_runtime_script.index(
        'sleep "${NJRH_NAV_LOCALIZATION_START_SETTLE_SEC:-3}"'
    )
    assert resident_runtime_script.index('sleep "${NJRH_NAV_LOCALIZATION_START_SETTLE_SEC:-3}"') < resident_runtime_script.index(
        "/floor_manager/switch_floor"
    )
    assert resident_runtime_startup.index("/floor_manager/switch_floor") < resident_runtime_startup.index(
        "trigger_global_localization_for_navigation"
    )
    assert resident_runtime_startup.index("trigger_global_localization_for_navigation") < resident_runtime_startup.index(
        "starting resident Nav2 layer"
    )
    assert resident_runtime_script.index("starting resident Nav2 layer") < resident_runtime_script.index(
        'sleep "${NJRH_NAV_RUNTIME_READY_MARK_DELAY_SEC:-1}"'
    )
    assert "dashboard_server" not in node_cpp
    assert "patch_dashboard_runtime" not in node_cpp
    assert "system(" not in node_cpp
    assert "popen(" not in node_cpp
    assert "/api/v1/docking/undock" in node_cpp
    assert "handle_docking_undock" in node_cpp
    assert "undock_before_navigation_if_needed" in node_cpp
    assert "start_pre_navigation_undock" in node_cpp
    assert "wait_for_pre_navigation_undock" in node_cpp
    assert "call_undock_service_with_charging_retry" in node_cpp
    assert "TriggerServiceObservation" in node_cpp
    assert "call_docking_trigger_service_observed" in node_cpp
    assert "docking_service_success" in node_cpp
    assert "docking_service_message" in node_cpp
    assert "docking_status_at_request" in node_cpp
    assert "docking_status_after_request" in node_cpp
    assert "undock_started_observed" in node_cpp
    assert "undock_cmd_count_observed" in node_cpp
    assert "undock_failure_reason" in node_cpp
    assert "service_success_without_undocking_status_observed_yet" in node_cpp
    assert "already running; no new /docking/undock service call" in node_cpp
    assert "complete_post_undock_relocalization" in node_cpp
    assert "relocalize_after_undock" in node_cpp
    assert "undock_after_success" in node_cpp
    assert "waiting for docking manager charging state before undock" in node_cpp
    assert "pre_navigation_undock" in node_cpp
    assert "docking_undock_client_" in node_cpp
    assert "docking_status_is_undocked" in node_cpp
    assert "undock requires docked state or live charging contact" in node_cpp


def test_ranger_mini3_mode_controller_is_cpp_and_rejects_lateral_reverse():
    package_root = ROOT / "src" / "ranger_mini3_mode_controller"
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
    mode_header = (
        package_root / "include" / "ranger_mini3_mode_controller" / "ranger_motion_mode.hpp"
    ).read_text(encoding="utf-8")
    node_cpp = (package_root / "src" / "mode_controller_node.cpp").read_text(encoding="utf-8")
    config = (package_root / "config" / "ranger_mini3_mode_controller.yaml").read_text(encoding="utf-8")
    overlay_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "ranger_mini3_mode_controller.yaml"
    ).read_text(encoding="utf-8")
    overlay_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_ranger_mini3_mode_controller.sh"
    ).read_text(encoding="utf-8")
    set_profile = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "set_ranger_mode_controller_profile.sh"
    ).read_text(encoding="utf-8")
    verify_passthrough = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "verify_ranger_official_passthrough.sh"
    ).read_text(encoding="utf-8")
    ab_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_ranger_official_passthrough_ab.sh"
    ).read_text(encoding="utf-8")
    assert "ament_cmake" in cmake
    assert "rclcpp" in package_xml
    assert "ranger_msgs" in package_xml
    assert "ament_python" not in package_xml
    assert "find_package(ranger_msgs REQUIRED)" in cmake
    assert "add_executable(mode_controller_node src/mode_controller_node.cpp)" in cmake
    assert "target_include_directories(mode_controller_node PRIVATE include)" in cmake
    assert "ament_target_dependencies(mode_controller_node geometry_msgs ranger_msgs rclcpp std_msgs)" in cmake
    assert "enum class RangerMotionMode" in mode_header
    assert "DUAL_ACKERMAN = 0" in mode_header
    assert "PARALLEL = 1" in mode_header
    assert "SPINNING = 2" in mode_header
    assert "SIDE_SLIP = 3" in mode_header
    assert "UNKNOWN = 255" in mode_header
    assert "ranger_motion_mode_from_code" in mode_header
    assert "ranger_motion_mode_json" in mode_header
    assert "mode_controller_profile: official_passthrough" in config
    assert "mode_controller_profile: official_passthrough" in overlay_config
    assert 'declare_parameter<std::string>("mode_controller_profile", "official_passthrough")' in node_cpp
    assert 'mode_controller_profile_ == "official_passthrough"' in node_cpp
    assert 'mode_controller_profile_ == "custom"' in node_cpp
    assert "computeOfficialPassthrough" in node_cpp
    assert "cmdVelPassthrough()" in node_cpp
    assert "customAckermannEnabled()" in node_cpp
    assert "custom_ackermann_enabled" in node_cpp
    assert "custom_ackermann_diagnostic_only" in node_cpp
    assert "cmd_vel_passthrough" in node_cpp
    assert "passthrough_preserves_twist" in node_cpp
    assert "output_diff_from_input" in node_cpp
    assert "diff_reason" in node_cpp
    assert "predicted_from_cmd_vel_safe" in node_cpp
    assert "reverse_not_allowed" in node_cpp
    assert "lateral_not_allowed" in node_cpp
    assert "park_requested" in node_cpp
    assert "timeout_zero" in node_cpp
    assert "lateral_policy: reject" in config
    assert "max_lateral_mps: 0.08" in config
    assert "max_crab_yaw_radps: 0.15" in config
    assert "allow_reverse: false" in config
    assert "reverse_enable_topic: /ranger_mini3/allow_reverse" in config
    assert "docking_reverse_enable_topic: /ranger_mini3/docking_allow_reverse" in config
    assert "teleop_reverse_enable_topic: /ranger_mini3/teleop_allow_reverse" in config
    assert "reverse_enable_timeout_s: 0.75" in config
    assert "spin_steering_threshold_rad: 0.698" in config
    assert "spin_enter_steering_threshold_rad: 0.698" in config
    assert "auto_spin_max_linear_mps: 0.08" in config
    assert "spin_on_high_curvature_while_moving: false" in config
    assert "desired_mode_topic: /ranger_mini3/desired_motion_mode" in config
    assert "actual_motion_state_topic: /motion_state" in config
    assert "actual_system_state_topic: /system_state" in config
    assert "actual_motion_mode_max_age_sec: 0.5" in config
    assert "mode_alignment_warn_after_sec: 0.25" in config
    assert "lateral_policy: reject" in overlay_config
    assert "max_lateral_mps: 0.08" in overlay_config
    assert "max_crab_yaw_radps: 0.15" in overlay_config
    assert "allow_reverse: false" in overlay_config
    assert "reverse_enable_topic: /ranger_mini3/allow_reverse" in overlay_config
    assert "docking_reverse_enable_topic: /ranger_mini3/docking_allow_reverse" in overlay_config
    assert "teleop_reverse_enable_topic: /ranger_mini3/teleop_allow_reverse" in overlay_config
    assert "spin_enter_steering_threshold_rad: 0.698" in overlay_config
    assert "auto_spin_max_linear_mps: 0.08" in overlay_config
    assert "spin_on_high_curvature_while_moving: false" in overlay_config
    assert "desired_mode_topic: /ranger_mini3/desired_motion_mode" in overlay_config
    assert "actual_motion_state_topic: /motion_state" in overlay_config
    assert "actual_system_state_topic: /system_state" in overlay_config
    assert "actual_motion_mode_max_age_sec: 0.5" in overlay_config
    assert "mode_alignment_warn_after_sec: 0.25" in overlay_config
    assert "Lateral / crab commands are disabled" in node_cpp
    assert '#include "ranger_msgs/msg/motion_state.hpp"' in node_cpp
    assert '#include "ranger_msgs/msg/system_state.hpp"' in node_cpp
    assert "create_subscription<ranger_msgs::msg::MotionState>" in node_cpp
    assert "create_subscription<ranger_msgs::msg::SystemState>" in node_cpp
    assert "desiredRangerMode" in node_cpp
    assert "RangerMotionMode::SPINNING" in node_cpp
    assert "ranger_motion_mode_json(desired_mode" in node_cpp
    assert "desired_motion_mode" in node_cpp
    assert "actual_motion_mode" in node_cpp
    assert "mode_aligned" in node_cpp
    assert "waiting_actual_motion_mode" in node_cpp
    assert "odom guards must use actual motion_mode" in node_cpp
    assert "forced_policy_ = \"crab\"" in node_cpp
    assert "msg.linear.y = command.lateral_mps" in node_cpp
    assert "spin_on_high_curvature_while_moving_ || low_speed_spin_allowed" in node_cpp
    assert "auto_spin_max_linear_mps_" in node_cpp
    assert node_cpp.index("computeOfficialPassthrough") < node_cpp.index("RangerCommand computeCommand")
    official_branch = node_cpp[
        node_cpp.index("PublishDecision computeOfficialPassthrough"):
        node_cpp.index("RangerCommand computeCommand")
    ]
    assert "dualAckermannInnerAngleFromTwist" not in official_branch
    assert "dualAckermannVirtualAngleFromTwist" not in official_branch
    assert "maxInnerAckermannYawRate" not in official_branch
    assert "maxVirtualAckermannYawRate" not in official_branch
    assert "speedLimitForAngle" not in official_branch
    assert "rawAckermannAngle" not in official_branch
    assert "effectiveAllowReverse" in node_cpp
    assert "reverse_permits_by_source_" in node_cpp
    assert "subscribeReversePermit(\"docking\"" in node_cpp
    assert "subscribeReversePermit(\"teleop\"" in node_cpp
    assert "vx = std::max(0.0, vx)" in node_cpp
    assert "return makeSpin(wz)" in node_cpp
    assert "python3" not in overlay_runner
    assert "set +u" in overlay_runner
    assert 'source "${REPO_ROOT}/install/local_setup.bash"' in overlay_runner
    assert "set -u" in overlay_runner
    assert "colcon build --packages-select ranger_mini3_mode_controller" in overlay_runner
    assert "--profile official_passthrough|custom" in set_profile
    assert "kill -TERM" in set_profile
    assert "killall -9" not in set_profile
    assert "pkill -9" not in set_profile
    assert "run_ranger_mini3_mode_controller.sh" in set_profile
    assert "/cmd_vel_safe" in set_profile
    assert "/cmd_vel" in set_profile
    assert "--compare-cmd" in verify_passthrough
    assert "mode_controller_profile" in verify_passthrough
    assert "custom_ackermann_enabled" in verify_passthrough
    assert "cmd_vel_passthrough" in verify_passthrough
    assert "ranger_base_node" in verify_passthrough
    assert "/ranger_mini3/docking_allow_reverse" in verify_passthrough
    assert "/ranger_mini3/teleop_allow_reverse" in verify_passthrough
    assert "--duration-sec" in ab_runner
    assert "reports/ranger_official_passthrough_ab_" in ab_runner
    assert "/wheel/odom" in ab_runner
    assert "/localization/bridge_status" in ab_runner


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
    assert "find_package(nav_msgs REQUIRED)" in cmake
    assert "nav_msgs" in cmake
    assert "<depend>nav_msgs</depend>" in package_xml
    assert "pcl_conversions" in node_script
    assert '#include "nav_msgs/msg/odometry.hpp"' in node_script
    assert "TransformListener" in node_script
    assert "lookupTransform(" in node_script
    assert 'declare_parameter<std::string>("output_frame_id", "base_link")' in node_script
    assert 'declare_parameter<std::string>("input_topic", "/lidar_points")' in node_script
    assert 'declare_parameter<std::string>("clearing_output_topic", "/perception/clearing_points")' in node_script
    assert "clearing.virtual_rays.enabled" in node_script
    assert "buildVirtualClearingPoints" in node_script
    assert "updateClearingBin" in node_script
    assert "FusedTransform3x4" in node_script
    assert "makeFusedTransform" in node_script
    assert "transformInputPoint(fused_transform" in node_script
    assert "std::unordered_map<std::uint64_t, std::uint16_t> counts" in node_script
    assert "packedVoxelKey" in node_script
    assert "makePointCloud2FromPoints(filtered_points" in node_script
    assert "std::optional<PendingClearingJob> pending_clearing_job_" in node_script
    assert "clearingWorkerLoop" in node_script
    assert "enqueueClearingJob" in node_script
    assert "pending_clearing_job_ = std::move(job)" in node_script
    assert 'declare_parameter<std::string>("status_topic", "/perception/local_perception_status")' in node_script
    assert 'declare_parameter<double>("status_publish_period_sec", 2.0)' in node_script
    assert "publishStatus" in node_script
    assert "input_callback_hz" in node_script
    assert "input_cloud_accept_hz" in node_script
    assert "input_interarrival_ms_avg" in node_script
    assert "input_subscription_qos_reliable" in node_script
    assert "processed_cloud_hz" in node_script
    assert "published_obstacle_hz" in node_script
    assert "pcl::toROSMsg" not in node_script
    assert "std::map<std::tuple" not in node_script
    assert "outputStampForCostmap" in node_script
    assert 'declare_parameter<std::string>("output_stamp_tf_target_frame", "odom")' in node_script
    assert 'declare_parameter<std::string>("output_stamp_odom_topic", "/local_state/odometry")' in node_script
    assert 'declare_parameter<bool>("restamp_to_latest_tf", false)' in node_script
    assert 'declare_parameter<bool>("require_output_stamp_tf", false)' in node_script
    assert 'declare_parameter<double>("max_output_tf_stamp_age_sec", 0.45)' in node_script
    assert 'declare_parameter<double>("output_stamp_tf_backoff_sec", 0.0)' in node_script
    assert 'declare_parameter<double>("output_stamp_forward_sec", 0.0)' in node_script
    assert 'declare_parameter<bool>("require_startup_tf_ready", true)' in node_script
    assert 'declare_parameter<double>("startup_tf_warmup_sec", 1.0)' in node_script
    assert "startupTfGateReady" in node_script
    assert "Skipping local perception cloud while TF listener warms" in node_script
    assert "local perception startup TF gate passed" in node_script
    assert "create_subscription<nav_msgs::msg::Odometry>" in node_script
    assert "odomStampCallback" in node_script
    assert "latest_odom_stamp_" in node_script
    assert "costmapOutputStamp()" in node_script
    assert "shiftedStamp(nowMsg(), output_stamp_forward_sec_)" in node_script
    assert "Gate output on a fresh odom<-base_link TF" in node_script
    assert "Skipping local perception cloud because latest" in node_script
    assert "local costmap stamping requires latest %s <- %s TF" in node_script
    assert "return std::nullopt" in node_script
    assert "lookupTransform(\n        output_stamp_tf_target_frame_, output_frame_id_, tf2::TimePointZero" in node_script
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()" in node_script
    assert "best_effort" in node_script
    assert "applyVoxelOutlierFilter" in node_script
    assert "\"ELEVATOR_WAIT\"" in node_script
    assert "\"DOORWAY\"" in node_script
    assert "output_stamp_odom_topic: /local_state/odometry" in local_cfg
    assert "output_stamp_odom_topic: /local_state/odometry" in overlay_cfg
    assert "max_output_tf_stamp_age_sec: 0.25" in local_cfg
    assert "max_output_tf_stamp_age_sec: 0.25" in overlay_cfg
    assert "require_startup_tf_ready: true" in local_cfg
    assert "require_startup_tf_ready: true" in overlay_cfg
    assert "startup_tf_warmup_sec: 1.0" in local_cfg
    assert "startup_tf_warmup_sec: 1.0" in overlay_cfg
    assert "output_stamp_tf_backoff_sec: 0.0" in local_cfg
    assert "output_stamp_tf_backoff_sec: 0.0" in overlay_cfg
    assert "output_stamp_forward_sec: 0.0" in local_cfg
    assert "output_stamp_forward_sec: 0.0" in overlay_cfg
    assert "status_topic: /perception/local_perception_status" in local_cfg
    assert "status_topic: /perception/local_perception_status" in overlay_cfg
    assert "status_publish_period_sec: 2.0" in local_cfg
    assert "status_publish_period_sec: 2.0" in overlay_cfg
    assert "preserve the original pointcloud acquisition stamp" in readme
    assert "/perception/local_perception_status" in readme
    assert "cold TF listener" in readme


def test_robot_safety_node_exports_stateful_final_cmd_vel_contract():
    node_script = (ROOT / "src" / "robot_safety" / "src" / "robot_safety_node.cpp").read_text(encoding="utf-8")
    config_text = (ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml").read_text(encoding="utf-8")
    assert "enum class SafetyState" in node_script
    assert '"status_topic", "/safety/status"' in node_script
    assert '"motion_allowed_topic", "/safety/motion_allowed"' in node_script
    assert "COMMAND_STALE" in node_script
    assert "LOCALIZATION_INVALID" in node_script
    assert "status_topic: /safety/status" in config_text
    assert "motion_allowed_topic: /safety/motion_allowed" in config_text
    assert "cmd_vel_out_topic: /cmd_vel_safe" in config_text
    assert "docking_cmd_vel_in_topic: /cmd_vel_docking" in config_text
    assert "enable_docking_cmd_priority: true" in config_text
    assert "docking_cmd_priority_timeout_sec: 0.25" in config_text
    assert 'declare_parameter<std::string>("docking_cmd_vel_in_topic", "/cmd_vel_docking")' in node_script
    assert 'declare_parameter<bool>("enable_docking_cmd_priority", true)' in node_script
    assert "docking_command_fresh" in node_script
    assert "fresh_docking_command_active()" in node_script
    assert "last_docking_cmd_ = *msg" in node_script
    assert "have_last_docking_cmd_ = true" in node_script
    assert "publish_command(last_docking_cmd_, snapshot)" in node_script
    assert "on_docking_cmd" in node_script
    assert "on_normal_cmd" in node_script


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
    assert (package_root / "launch" / "floor_manager.launch.py").exists()
    assert "add_executable(floor_manager_node src/floor_manager_node.cpp)" in cmake
    assert "lifecycle_msgs" in cmake
    assert "<depend>lifecycle_msgs</depend>" in package_xml
    assert "nav2_msgs" in package_xml
    assert "SwitchFloor" in node_cpp
    assert "/floor_manager/switch_floor" in node_cpp
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
    assert "REQUIRED_RELATIVE_ASSETS" in map_toolkit
    assert "--flat-map-name" in map_toolkit
    assert "promote_flat_map" in map_toolkit
    assert "localizer/localizer_params.yaml" in map_toolkit


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
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    slam_launch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_slam_toolbox_mapping.launch.py").read_text(encoding="utf-8")
    slam_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_slam_toolbox_mapping.yaml").read_text(encoding="utf-8")
    slam_scan_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_scan_slam2d.yaml").read_text(encoding="utf-8")
    preprocessor_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_nav_cloud_preprocessor.yaml").read_text(encoding="utf-8")
    fastlio_odom_bridge_cpp = (
        ROOT / "src" / "robot_fastlio_mapping" / "src" / "fastlio_odom_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    scan_republisher_cpp = (ROOT / "src" / "robot_hesai_jt128" / "src" / "scan_republisher_node.cpp").read_text(encoding="utf-8")
    hesai_cmake = (ROOT / "src" / "robot_hesai_jt128" / "CMakeLists.txt").read_text(encoding="utf-8")
    hesai_package = (ROOT / "src" / "robot_hesai_jt128" / "package.xml").read_text(encoding="utf-8")
    assert "ros2 launch" in run_projected_map
    assert "jt128_slam_toolbox_mapping.launch.py" in run_projected_map
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
    assert 'FASTLIO_ODOM_TOPIC="${NJRH_SLAM2D_FASTLIO_ODOM_TOPIC:-/Odometry}"' in run_projected_map
    assert 'SLAM2D_PRIVATE_TF_TOPIC="${NJRH_SLAM2D_PRIVATE_TF_TOPIC:-/tf_slam2d}"' in run_projected_map
    assert 'SLAM2D_FASTLIO_ODOM_FRAME="${NJRH_SLAM2D_FASTLIO_ODOM_FRAME:-mapping_odom}"' in run_projected_map
    assert 'POINTS_TOPIC="${NJRH_SLAM2D_POINTS_TOPIC:-/cloud_registered_body}"' in run_projected_map
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
    assert 'export NJRH_CPUSET_SLAM_TOOLBOX_MAPPING="${NJRH_SLAM2D_SLAM_TOOLBOX_CPUSET:-${NJRH_CPUSET_MAPPING_BACKEND:-7}}"' in run_projected_map
    assert "ros2 run fast_lio fastlio_mapping" in run_projected_map
    assert "njrh_run_affined fastlio_deskew ros2 run fast_lio fastlio_mapping" in run_projected_map
    assert "-r /tf:=/tf_fastlio_internal" in run_projected_map
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
    assert 'wait_for_topic_message "${FASTLIO_ODOM_TOPIC}" "${FASTLIO_ODOM_READY_TIMEOUT}"' in run_projected_map
    assert 'bridge_bin="${NJRH_PROJECT_ROOT}/install/robot_fastlio_mapping/lib/robot_fastlio_mapping/fastlio_odom_bridge_node"' in run_projected_map
    assert 'njrh_run_affined fastlio_odom_bridge "${bridge_bin}"' in run_projected_map
    assert 'python3 "${bridge_script}"' not in run_projected_map
    assert 'wait_for_topic_message "/mapping/fastlio_odometry" 10' in run_projected_map
    assert 'odom_frame:="${slam_odom_frame}"' in run_projected_map
    assert 'tf_topic:="${slam_tf_topic}"' in run_projected_map
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' not in run_projected_map
    assert "wait_for_ekf_local_state_inputs" not in run_projected_map
    assert "wait_for_ekf_local_state_ready" not in run_projected_map
    assert "restart_robot_local_state_for_mapping" not in run_projected_map
    assert "starting a fresh local-state stack" not in run_projected_map
    assert 'wait_for_topic_message "/local_state/imu_bias" "${bias_timeout}"' not in run_projected_map
    assert 'wait_for_tf_edge "base_link" "lidar_level_link" 10' in run_projected_map
    assert 'wait_for_topic_publisher "/local_state/odometry" "${LOCAL_ODOM_READY_TIMEOUT}"' in run_projected_map
    assert 'wait_for_fresh_header_topic_message "/local_state/odometry" "${LOCAL_ODOM_READY_TIMEOUT}" "${LOCAL_ODOM_MAX_AGE_SEC}" 0.25' in run_projected_map
    assert "check_local_state_odom_sane" in run_projected_map
    assert 'local_odom_reference_topic="/fastlio/base_odometry"' in run_projected_map
    assert 'local_odom_reference_topic="/wheel/odom_ekf"' in run_projected_map
    assert 'refusing to start slam_toolbox mapping with unhealthy resident local odometry' in run_projected_map
    assert "slam_toolbox" in slam_launch
    assert 'DeclareLaunchArgument("points_topic", default_value="/cloud_registered_body")' in slam_launch
    assert 'DeclareLaunchArgument("tf_topic", default_value="/tf")' in slam_launch
    assert '("/tf", tf_topic)' in slam_launch
    assert "f'image: {pgm_path.name}'," in dashboard_patch
    assert "'image': str(pgm_path)" in dashboard_patch
    assert "nav_cloud_preprocessor" in slam_launch
    assert "pointcloud_to_laserscan_node" in slam_launch
    assert "scan_republisher_node" in slam_launch
    assert "scan_republisher_node" not in run_projected_map
    assert 'restamp_scan_to_now = env_bool("NJRH_SLAM2D_RESTAMP_SCAN_TO_NOW", False)' in slam_launch
    assert '"restamp_to_now": restamp_scan_to_now' in slam_launch
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
    assert "find_package(nav_msgs REQUIRED)" not in hesai_cmake
    assert "rclcpp nav_msgs sensor_msgs" not in hesai_cmake
    assert "<depend>nav_msgs</depend>" not in hesai_package
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
    assert "scan_queue_size: 30" in slam_cfg
    assert "minimum_travel_heading: 0.02" in slam_cfg
    assert "scan_buffer_size: 30" in slam_cfg
    assert "do_loop_closing: true" in slam_cfg
    assert "loop_search_maximum_distance: 3.0" in slam_cfg
    assert "loop_match_minimum_chain_size: 10" in slam_cfg
    assert "loop_match_maximum_variance_coarse: 2.5" in slam_cfg
    assert "loop_match_minimum_response_coarse: 0.40" in slam_cfg
    assert "loop_match_minimum_response_fine: 0.50" in slam_cfg
    assert "loop_search_space_dimension: 6.0" in slam_cfg
    assert "correlation_search_space_dimension: 0.5" in slam_cfg
    assert "scan_republisher_node" in slam_launch
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
    assert "tf_future_stamp_offset_sec: 0.10" in (
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
    pointcloud_local_branch_cfg = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "jt128_canonical_pointcloud_remap_local_branch.yaml"
    ).read_text(encoding="utf-8")
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
    assert 'njrh_run_affined hesai_ros_driver bash "$(require_upstream_script run_driver.sh)" &' in driver_script
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
    assert 'plugin="LocalPerceptionNode"' in pointcloud_pipeline_launch
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
    assert "local_output_topic: /_internal/lidar_points_local" in pointcloud_local_branch_cfg
    assert "local_output_stride: 2" in pointcloud_local_branch_cfg
    assert "local_output_publish_every_n: 1" in pointcloud_local_branch_cfg
    assert "local_output_qos_depth: 1" in pointcloud_local_branch_cfg
    assert "output_reliable: false" in pointcloud_local_branch_cfg
    assert "Branch publishers are fixed best-effort/depth 1" in pointcloud_local_branch_cfg
    assert "export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=local_branch" in local_profile_env
    assert "local_branch)" in local_profile_helper
    assert "trunk)" in local_profile_helper
    assert 'RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC="${ROBOT_LOCAL_PERCEPTION_INPUT_TOPIC}"' in local_profile_helper
    assert 'RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC}"' in local_profile_helper
    assert 'RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE}"' in local_profile_helper
    assert 'RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N}"' in local_profile_helper
    assert "selected local perception profile=" in local_profile_helper
    assert 'INPUT_TOPIC="${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}"' in run_local_perception
    assert "njrh_print_local_perception_profile" in run_local_perception
    assert '-p "input_topic:=${INPUT_TOPIC}"' in run_local_perception
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
    assert "/perception/local_perception_status" in verify_pointcloud_matrix
    assert "/lidar/nav_cloud_preprocessor_status" in verify_pointcloud_matrix
    assert "CASE_A_MAIN_TRUNK_LOW" in verify_pointcloud_matrix
    assert "CASE_B_LOCAL_DDS_DELIVERY_LOW" in verify_pointcloud_matrix
    assert "CASE_C_LOCAL_PROCESS_OR_PUBLISH_GATING" in verify_pointcloud_matrix
    assert "CASE_D_FANOUT_PRESSURE" in verify_pointcloud_matrix
    assert "CASE_E_TOO_MANY_FULL_DENSITY_SUBSCRIBERS" in verify_pointcloud_matrix
    assert "CASE_F_LOCAL_BRANCH_EFFECTIVE" in verify_pointcloud_matrix
    assert "CASE_G_DDS_TRANSPORT_SUSPECT" in verify_pointcloud_matrix
    assert "CASE_H_LOCAL_BRANCH_ENABLED_BUT_WEAK" in verify_pointcloud_matrix
    assert "CASE_I_LOCAL_BRANCH_DRAGS_TRUNK" in verify_pointcloud_matrix
    assert "NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ" in verify_pointcloud_matrix
    assert "topic_graph_counts" in verify_pointcloud_matrix
    assert "graph_subscription_count" in verify_pointcloud_matrix
    assert "FASTDDS_BUILTIN_TRANSPORTS" in verify_pointcloud_matrix
    assert "NJRH_FASTDDS_PROFILE_ENABLED" in verify_pointcloud_matrix
    assert "current local perception profile=" in verify_pointcloud_matrix
    assert "resolved axis local_output_stride=" in verify_pointcloud_matrix
    assert "resolved axis local_output_publish_every_n=" in verify_pointcloud_matrix
    assert "local_branch_enabled" in verify_pointcloud_matrix
    assert "local_branch_publish_hz" in verify_pointcloud_matrix
    assert "local_branch_subscription_count" in verify_pointcloud_matrix
    assert "obstacle CLI hint: ros2 topic hz /perception/obstacle_points" in verify_pointcloud_matrix
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
    assert "input_callback_hz" in verify_lidar_trunk_jitter
    assert "published_obstacle_hz" in verify_lidar_trunk_jitter
    assert "This default run does not subscribe to full-density /lidar_points" in diagnose_lidar_jitter
    assert "--include-cli-hz" in diagnose_lidar_jitter
    assert "CASE_A_AXIS_PUBLISH_LOW" in diagnose_lidar_jitter
    assert "CASE_B_CLI_DELIVERY_LOW_ONLY" in diagnose_lidar_jitter
    assert "CASE_C_PROFILE_REGRESSION" in diagnose_lidar_jitter
    assert "CASE_D_STALE_PROCESS_OR_OLD_BINARY" in diagnose_lidar_jitter
    assert "CASE_E_TOO_MANY_TRUNK_SUBSCRIBERS" in diagnose_lidar_jitter
    assert "CASE_F_LOCAL_BRANCH_OK_BUT_TRUNK_CLI_LOW" in diagnose_lidar_jitter
    assert "CASE_G_AXIS_LOW_WITH_PURE_TRUNK_NEEDED" in diagnose_lidar_jitter
    assert "ros2 topic hz /lidar_points" in diagnose_lidar_jitter
    assert "INCLUDE_CLI_HZ" in diagnose_lidar_jitter
    assert "trunk_publish_gap_over_100ms_count" in diagnose_lidar_jitter
    assert "raw_interarrival_ms_avg" in diagnose_lidar_jitter
    assert "local_branch production default" in diagnose_lidar_jitter
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
    assert "/_internal/lidar_points_local" in topology_contract
    assert "local_branch profile derives /_internal/lidar_points_local" in topology_contract
    assert "trunk profile disables local internal branch" in topology_contract
    assert "/lidar_points graph publisher count is 1" in topology_contract
    assert "/_internal/lidar_points_local graph publisher/subscriber count is 1/1" in topology_contract
    assert "jt128_canonical_pointcloud_remap(_local_branch)?[.]yaml" in topology_contract
    assert "output_reliable" in topology_contract
    assert "Observing ROS graph only" in inspect_pointcloud_subscribers
    assert "ros2 topic info -v" in inspect_pointcloud_subscribers
    assert "ros2 topic echo" not in inspect_pointcloud_subscribers
    assert "ros2 topic hz" not in inspect_pointcloud_subscribers
    assert "/lidar_points publisher count must be exactly 1" in inspect_pointcloud_subscribers
    assert "/lidar_points has ${subscription_count} subscribers" in inspect_pointcloud_subscribers
    assert "/_internal/lidar_points_local" in inspect_pointcloud_subscribers
    assert "FAST-LIO appears attached to /lidar_points" in inspect_pointcloud_subscribers
    assert "--transport UDPv4|DEFAULT|LARGE_DATA" in dds_transport_ab
    assert "Runtime restart not requested; no production process was stopped." in dds_transport_ab
    assert "FASTDDS_BUILTIN_TRANSPORTS must be set before every ROS participant starts" in dds_transport_ab
    assert "NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ" in dds_transport_ab
    assert "--profile local_branch|trunk" in set_local_profile
    assert "--restart not requested; no process was restarted." in set_local_profile
    assert "NJRH_FORCE_RESTART_DRIVER=true" in set_local_profile
    assert "run_local_perception_local_profile.log" in set_local_profile
    assert "inspect_pointcloud_subscribers.sh" in set_local_profile
    assert "verify_pointcloud_delivery_matrix.sh" in set_local_profile
    assert "ros2 topic hz /perception/obstacle_points" in set_local_profile
    assert "pointcloud_axis_remap" in inspect_cpu_affinity
    assert "pid=%s tid=%s cpu=%s pcpu=%s" in inspect_cpu_affinity
    assert "robot_local_perception" in inspect_cpu_affinity
    assert "nav_cloud_preprocessor" in inspect_cpu_affinity
    assert "pointcloud_to_laserscan" in inspect_cpu_affinity
    assert "scan_republisher" in inspect_cpu_affinity
    assert "global_localization/localizer" in inspect_cpu_affinity
    assert "NJRH_CPUSET_" in inspect_cpu_affinity
    assert "WARN local_branch is enabled but local input <10Hz" in inspect_cpu_affinity
    assert "thermal and clock snapshot" in inspect_cpu_affinity
    assert "tegrastats" in inspect_cpu_affinity
    assert "nvpmodel" in inspect_cpu_affinity
    assert "jetson_clocks" in inspect_cpu_affinity
    assert "record_pointcloud_nav_acceptance.sh [--duration-sec SECONDS]" in nav_acceptance
    assert "/lidar/axis_remap_status" in nav_acceptance
    assert "/perception/local_perception_status" in nav_acceptance
    assert "/lidar/nav_cloud_preprocessor_status" in nav_acceptance
    assert "/safety/status" in nav_acceptance
    assert "/local_state/odometry" in nav_acceptance
    assert "/localization_result" in nav_acceptance
    assert "NJRH_RECORD_HEAVY_POINTCLOUDS=true" in nav_acceptance
    assert "axis lidar_points_publish_hz" in nav_acceptance
    assert "axis local_branch_publish_hz" in nav_acceptance
    assert "CASE_I_LOCAL_BRANCH_DRAGS_TRUNK seen" in nav_acceptance
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
    node_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "TF is a process-level localization input" in node_cpp
    assert "set_status_subscriptions_active(true);\n    // TF is a process-level localization input" in node_cpp
    assert "page leases only report interest and must not tear it down" in node_cpp
    assert "tf_subscription_active()" not in node_cpp

    tf_function = node_cpp[
        node_cpp.index("void set_tf_subscription_active"):
        node_cpp.index("RobotPoseSnapshot current_robot_pose_snapshot")
    ]
    assert "if (!active) {\n      return;\n    }" in tf_function
    assert "tf_sub_.reset()" not in tf_function
    assert "have_pose_ = false" not in tf_function

    wait_function = node_cpp[
        node_cpp.index("RobotPoseSnapshot wait_for_current_robot_pose"):
        node_cpp.index("void handle_bms_state")
    ]
    assert "set_tf_subscription_active(true);" in wait_function
    assert "set_tf_subscription_active(false)" not in wait_function
    assert "was_active" not in wait_function


def test_mapping_stop_only_cleans_private_fastlio_residuals():
    api_code = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
    )
    api_config = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(
        encoding="utf-8"
    )
    overlay_api_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    runtime_utils = (ROOT / "src" / "robot_api_server" / "src" / "runtime_process_utils.cpp").read_text(
        encoding="utf-8"
    )
    mapping_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_projected_map.sh"
    ).read_text(encoding="utf-8")

    assert "read_proc_environ(pid)" in api_code
    assert "is_private_slam2d_fastlio_process" in api_code
    assert "NJRH_SLAM2D_PRIVATE_FASTLIO=1" in api_code
    assert "read_proc_environ" in runtime_utils
    assert "cannot start 2D mapping while navigation runtime is active; stop navigation runtime first" in api_code
    assert "mapping_lidar_rps_xps_state_dir" in api_code
    assert "/tmp/njrh_slam2d_lidar_rps_xps" in api_code
    assert "restore_mapping_lidar_rps_xps_state()" in api_code
    assert "is_safe_mapping_lidar_rps_xps_path" in api_code
    assert "rps_xps.tsv" in api_code
    assert 'starts_with(path, "/sys/class/net/")' in api_code
    assert 'path.find("/queues/")' in api_code
    assert '"/rps_cpus"' in api_code
    assert '"/xps_cpus"' in api_code
    for config_text in (api_config, overlay_api_config):
        assert 'mapping_lidar_rps_xps_state_dir: "/tmp/njrh_slam2d_lidar_rps_xps"' in config_text

    residual_fn = api_code[
        api_code.index("bool is_mapping_2d_residual_process_command"):
        api_code.index("std::set<pid_t> discover_mapping_2d_process_groups")
    ]
    fastlio_fn = api_code[
        api_code.index("bool is_private_slam2d_fastlio_process"):
        api_code.index("bool is_mapping_2d_residual_process_command")
    ]
    terminate_fn = api_code[
        api_code.index("std::size_t terminate_mapping_2d_process_groups_locked"):
        api_code.index("HttpResponse handle_start_mapping_2d")
    ]

    assert "ros2 run fast_lio fastlio_mapping" not in residual_fn
    assert "fast_lio/lib/fast_lio/fastlio_mapping" not in residual_fn
    assert "fastlio_mapping --ros-args" not in residual_fn
    assert "nav_cloud_preprocessor" not in residual_fn
    assert "pointcloud_to_laserscan_node" not in residual_fn
    assert "scan_republisher_node" not in residual_fn
    assert "ros2 run fast_lio fastlio_mapping" in fastlio_fn
    assert "environ.find(\"NJRH_SLAM2D_PRIVATE_FASTLIO=1\")" in fastlio_fn
    assert "nav_cloud_preprocessor" not in mapping_script
    assert "pointcloud_to_laserscan_node" not in mapping_script
    assert "scan_republisher_node" not in mapping_script
    assert "restore_mapping_lidar_rps_xps_state();" in terminate_fn


def test_localization_bridge_latches_one_shot_localizer_pose():
    bridge_code = (ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp").read_text(encoding="utf-8")
    bridge_cmake = (ROOT / "src" / "robot_localization_bridge" / "CMakeLists.txt").read_text(encoding="utf-8")
    bridge_package = (ROOT / "src" / "robot_localization_bridge" / "package.xml").read_text(encoding="utf-8")
    bridge_cfg = (ROOT / "src" / "robot_localization_bridge" / "config" / "localization_bridge.yaml").read_text(encoding="utf-8")
    overlay_bridge_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "localization_bridge.yaml"
    ).read_text(encoding="utf-8")
    api_code = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    api_cfg = (ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml").read_text(encoding="utf-8")
    overlay_api_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "robot_api_server.yaml"
    ).read_text(encoding="utf-8")
    assert "has_last_pose_stamp_used_" in bridge_code
    assert 'refresh_state("pose")' in bridge_code
    assert 'refresh_state("timer")' in bridge_code
    assert 'bridge waiting for localization_result' in bridge_code
    assert "else if (!has_map_to_odom_)" in bridge_code
    assert "if (!has_map_to_odom_)" in bridge_code
    assert "tf.transform.translation.x = map_to_odom_.x" in bridge_code
    assert "tf.transform.rotation = quaternion_from_yaw(map_to_odom_.yaw)" in bridge_code
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
    assert "request_localization_bridge_force_accept" in api_code
    assert "trigger_localization_and_wait_for_result(" in api_code
    assert "&relocalization_sequence" in api_code
    assert "post_undock_relocalization_succeeded" in api_code
    assert "undocked before navigation but post-undock relocalization did not complete" in api_code
    for cfg in (api_cfg, overlay_api_cfg):
        assert 'localization_bridge_force_accept_service: "/robot_localization_bridge/force_accept_next_localization"' in cfg


def test_localization_sensing_reuses_slam2d_scan_contract():
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
    assert "target_frame: lidar_level_link" in slam_scan_cfg
    assert "min_height: -0.75" in slam_scan_cfg
    assert "max_height: 0.35" in slam_scan_cfg
    assert "drop_invalid_ranges: true" in flatscan_cfg
    assert "min_scan_fov_degrees: 115.0" in localizer_cfg


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
    local_branch_cfg = (config_dir / "jt128_canonical_pointcloud_remap_local_branch.yaml").read_text(
        encoding="utf-8"
    )
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

    assert "input_topic: /lidar_points\n" in local_cfg
    assert "input_topic: /_internal/lidar_points_local" not in local_cfg
    assert "export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=local_branch" in local_profile
    assert "local_output_topic: /_internal/lidar_points_local" in local_branch_cfg
    assert "local_output_stride: 2" in local_branch_cfg
    assert "local_output_stride: 1" not in local_branch_cfg
    assert "input_reliable: false" in local_cfg
    assert "input_reliable: false" in local_branch_cfg
    assert "output_reliable: false" in local_branch_cfg
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert 'export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env
    assert 'planner_plugins: ["GridBased"]' in nav2
    assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in nav2
    assert 'controller_plugins: ["FollowPath", "FollowPathFallback"]' in nav2
    assert 'primary_controller: "nav2_mppi_controller::MPPIController"' in nav2
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

    assert 'export NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE:-legacy}"' in profile_env
    assert 'export NJRH_POINTCLOUD_INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}"' in ingress_env
    assert "separate_process" in accel_profile
    assert "driver_integrated" in accel_profile
    assert "nitros" in accel_profile
    assert "ipc_worker" in accel_profile
    assert "--profile legacy|ipc_worker|nitros" in set_profile
    assert "--ingress-profile separate_process|driver_integrated" in set_profile
    assert "NJRH_POINTCLOUD_ACCEL_RESTART=true" in set_profile
    assert "NITROS profile was not written" in set_profile
    assert "pkill -9" not in set_profile
    assert "killall" not in set_profile
    assert "legacy)" in run_pipeline
    assert "ipc_worker)" in run_pipeline
    assert "nitros)" in run_pipeline
    assert "NITROS profile not started" in run_pipeline
    assert "NJRH_POINTCLOUD_INGRESS_PROFILE=driver_integrated" in run_pipeline
    assert "driver_integrated ingress selected" in run_pipeline
    assert "standalone hesai_ros_driver_node or pointcloud_accel_axis_node" in run_pipeline
    assert 'NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}"' in run_pipeline
    assert "run_local_perception.sh" in run_pipeline
    assert "NJRH_POINTCLOUD_ACCEL_PROFILE=legacy" in run_pipeline
    assert "jt128_localization_sensing.launch.py" in run_pipeline
    assert "legacy_scan_pid" in run_pipeline
    assert "legacy_scan_chain_running()" in run_pipeline
    assert "legacy_scan_chain_partial_running()" in run_pipeline
    assert "legacy scan chain already running; reusing" in run_pipeline
    assert "preprocessor_params:=" in run_pipeline
    assert "scan_params:=" in run_pipeline
    assert "flatscan_params:=" in run_pipeline
    assert "points_topic:=" in run_pipeline
    assert "nav_points_topic:=" in run_pipeline
    assert "scan_topic:=" in run_pipeline
    assert "flatscan_topic:=" in run_pipeline
    assert "pkill -TERM -f" in run_pipeline
    assert "laser_scan_to_flatscan" in run_pipeline
    assert "pointcloud_accel_axis_node workers publish /perception/* and /scan" in run_pipeline
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
    assert "use NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker or legacy" in nitros_check

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
    assert 'declare_parameter<bool>("local_worker_enabled", true)' in accel_axis
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
    assert "local_worker_enabled: true" in accel_cfg
    assert "scan_worker_enabled: true" in accel_cfg
    assert "local_compact_fields: xyzi" in accel_cfg
    assert "local_compact_stride: 4" in accel_cfg
    assert "local_compact_max_rate_hz: 12.0" in accel_cfg
    assert "nav_compact_fields: xyzi" in accel_cfg
    assert "nav_compact_stride: 4" in accel_cfg
    assert "nav_compact_max_rate_hz: 10.0" in accel_cfg
    assert "obstacle_output_topic: /perception/obstacle_points" in accel_cfg
    assert "clearing_output_topic: /perception/clearing_points" in accel_cfg
    assert "scan_output_topic: /scan" in accel_cfg
    assert "/points_nav" not in accel_cfg
    assert "driver_integrated_available: true" in hesai_accel_cfg
    assert "vendor_raw_ros_hop_required: false" in hesai_accel_cfg
    assert "input_path: driver_callback_pointcloud2" in hesai_accel_cfg
    assert "publish_vendor_raw_debug: false" in hesai_accel_cfg
    assert "publish_vendor_imu_raw_debug: true" in hesai_accel_cfg
    assert "lidar_points_full_density_full_fields: true" in hesai_accel_cfg

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
    assert "local_perception is owned by pointcloud accel profile" in common_services
    assert "local_perception is owned by pointcloud accel profile" in nav2_navigation
    assert "local_perception is owned by pointcloud accel profile" in local_costmap_debug
    assert "occupancy_localization.launch.py" in occupancy
    assert "occupancy_localization_stack.launch.py" in occupancy
    assert "ensure_pointcloud_accel_pipeline_for_localization" in occupancy
    assert "pointcloud accel pipeline already running" in occupancy
    assert "pointcloud_accel_pipeline_localization" in occupancy
    assert "laser_scan_to_flatscan" in occupancy
    assert "pointcloud accel profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}" in occupancy

    assert "NJR H_CPUSET_POINTCLOUD_ACCEL" not in cpu_affinity
    assert "NJRH_CPUSET_POINTCLOUD_ACCEL_CONTAINER" in cpu_affinity
    assert "NJRH_CPUSET_POINTCLOUD_ACCEL_LOCAL_WORKER" in cpu_affinity
    assert "NJRH_CPUSET_POINTCLOUD_ACCEL_SCAN_WORKER" in cpu_affinity
    assert "NJRH_CPUSET_NITROS_POINTCLOUD_CONTAINER" in cpu_affinity

    assert "/perception/obstacle_points" in nav2
    assert "/perception/clearing_points" in nav2
    assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in nav2
    assert 'primary_controller: "nav2_mppi_controller::MPPIController"' in nav2
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
    assert "obstacle_hz" in verify_profile
    assert "scan_hz" in verify_profile
    assert "flatscan_hz" in verify_profile
    assert "FAST-LIO2 residual" in verify_profile
    assert "local_costmap subscribes" in verify_profile
    assert "PointCloud2 QoS" in verify_profile
    assert "DDS transport env" in verify_profile
    assert "/lidar_points publisher_nodes" in verify_profile
    assert "actual trunk owner" in verify_profile
    assert "actual obstacle owner" in verify_profile
    assert "actual clearing owner" in verify_profile
    assert "actual points_nav owner" in verify_profile
    assert "actual scan owner" in verify_profile
    assert "actual flatscan owner" in verify_profile
    assert "legacy trunk owner is pointcloud_axis_remap" in verify_profile
    assert "trunk owner is pointcloud_accel_axis_node" in verify_profile
    assert "obstacle owner is accel core process" in verify_profile
    assert "clearing owner is accel core process" in verify_profile
    assert "legacy points_nav owner is nav_cloud_preprocessor" in verify_profile
    assert "legacy missing /scan publisher from scan_republisher" in verify_profile
    assert "legacy missing /flatscan publisher from laser_scan_to_flatscan" in verify_profile
    assert "still using legacy pointcloud_axis_remap as trunk owner" in verify_profile
    assert "scan owner is accel core process" in verify_profile
    assert "local_worker_enabled must be true" in verify_profile
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
    assert "LEGACY_SCAN_CHAIN_OK" in verify_profile
    assert "IPC_WORKER_OWNER_OK" in verify_profile
    assert "TRUNK_FULL_DENSITY_OK" in verify_profile
    assert "NAV2_COMPAT_TOPICS_OK" in verify_profile
    assert "INGRESS_PROFILE" in verify_profile

    assert "--profile legacy|ipc_worker|nitros" in ab_runner
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
    assert "obstacle owner" in ab_runner
    assert "clearing owner" in ab_runner
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
    assert "legacy scan chain recovered" in ab_runner
    assert "ipc_worker no production /points_nav hop" in ab_runner
    assert "PASS/WARN/FAIL" in ab_runner
    assert "restoring prior profile" in ab_runner
    assert "pointcloud_accel_ab_" in ab_runner
    assert "cpu0" in ab_runner
    assert "cpu4" in ab_runner
    assert "cpu5" in ab_runner
    assert "cpu6" in ab_runner
    assert "cpu7" in ab_runner

    assert 'export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=local_branch' in (
        config_dir / "local_perception_input_profile.env"
    ).read_text(encoding="utf-8")
    assert "INPUT_TOPIC=\"${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}\"" in (
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
    assert "struct ClearingRayBin" in accel_core
    assert "build_virtual_clearing_points()" in accel_core
    assert "update_clearing_virtual_ray_bin(point)" in accel_core

    for script in (run_driver, run_pipeline, verify_profile, ab_runner, set_profile):
        assert "NJRH_POINTCLOUD_INGRESS_PROFILE" in script

    assert "driver_integrated" in accel_profile
    assert "forcing pointcloud ingress separate_process for legacy" in accel_profile
    assert "NJRH_POINTCLOUD_INGRESS_PROFILE=driver_integrated" in run_pipeline
    assert "driver_integrated ingress selected" in run_pipeline
    assert "HESAI_ACCEL_DRIVER_CPP_BIN" in run_driver
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
    assert "publish_vendor_raw_debug: false" in hesai_accel_cfg
    assert "publish_vendor_imu_raw_debug: true" in hesai_accel_cfg
    assert "vendor_raw_ros_hop_required: false" in hesai_accel_cfg
    for cfg in (accel_cfg, hesai_accel_cfg):
        assert "clearing_worker_virtual_rays_enabled: true" in cfg
        assert "clearing_worker_virtual_ray_angle_resolution_deg: 1.0" in cfg
        assert "clearing_worker_virtual_ray_range: 8.00" in cfg
        assert "clearing_worker_virtual_ray_range_steps: [0.50, 1.00, 2.00, 3.50, 5.50, 8.00]" in cfg
        assert (
            "clearing_worker_virtual_ray_endpoint_z_values: "
            "[-0.10, 0.05, 0.20, 0.40, 0.60, 0.85, 1.10, 1.30]"
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
    assert 'export NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE:-legacy}"' in (
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
    verify_profile = (scripts_dir / "verify_pointcloud_accel_profile.sh").read_text(encoding="utf-8")
    nav_runtime = (scripts_dir / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    ab_runner = (scripts_dir / "run_pointcloud_accel_ab.sh").read_text(encoding="utf-8")
    nav2 = (overlay / "config" / "nav2.yaml").read_text(encoding="utf-8")

    combined = run_pipeline + verify_profile + nav_runtime + ab_runner
    for topic in ("/lidar_points", "/scan", "/flatscan", "/perception/obstacle_points", "/perception/clearing_points"):
        assert topic in combined
    assert "/flatscan_debug" not in combined
    assert "/flatscan_status" not in combined

    assert "start_flatscan_helper()" in run_pipeline
    assert "flatscan_helper_running()" in run_pipeline
    assert "restart_flatscan_helper_or_fail()" in run_pipeline
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
    assert 'FLATSCAN_WAIT_SEC="${NJRH_FLATSCAN_WAIT_SEC:-30}"' in run_pipeline
    assert "NJRH_FLATSCAN_MIN_HZ" in run_pipeline
    assert "FLATSCAN_HELPER_RESTART_COUNT" in run_pipeline
    assert "CASE_FLATSCAN_HELPER_DEAD" in run_pipeline
    assert "standalone /scan exists but /flatscan publisher is missing while laser_scan_to_flatscan pid=" in run_pipeline
    assert "legacy_scan_pid=$!" in run_pipeline
    assert 'flatscan_helper_mode="legacy_launch"' in run_pipeline
    assert 'flatscan_helper_mode="standalone"' in run_pipeline
    assert "start_flatscan_helper" in run_pipeline[run_pipeline.index("ipc_worker)") :]
    legacy_block = run_pipeline[run_pipeline.index("legacy)") : run_pipeline.index("ipc_worker)")]
    assert "jt128_localization_sensing.launch.py" in legacy_block
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
    assert "current_pointcloud_accel_profile" in nav_runtime
    assert "set_pointcloud_accel_profile.sh\" --profile \"${profile}\" --restart" in nav_runtime
    assert "/flatscan repair succeeded after pointcloud accel restart" in nav_runtime
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
    assert "obstacle_output_topic: /perception/obstacle_points" in accel_cfg
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
    assert "map_to_odom_x" in bridge_cpp
    assert "map_to_odom_y" in bridge_cpp
    assert "continuous_localization_mode_" in bridge_cpp
    assert "triggered_max_result_age_ms_" in bridge_cpp
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

    assert "service_call_timeout_sec: 10.0" in global_cfg
    assert "result_wait_timeout_sec: 20.0" in global_cfg
    assert "bridge_accept_timeout_sec: 8.0" in global_cfg
    assert "map_to_odom_wait_timeout_sec: 8.0" in global_cfg
    assert "bridge_status_topic: /localization/bridge_status" in global_cfg
    assert "bridge_force_accept_service: /robot_localization_bridge/force_accept_next_localization" in global_cfg
    assert "bridge_status_sub_" in global_cpp
    assert "localization_result_sub_" in global_cpp
    assert "find_package(tf2_ros REQUIRED)" in global_cmake
    assert "tf2_ros" in global_cmake
    assert "<depend>tf2_ros</depend>" in global_pkg
    assert "wait_for_bridge_acceptance" in global_cpp
    assert "wait_for_map_to_odom" in global_cpp
    assert "failure_code=ISAAC_SERVICE_TIMEOUT" in global_cpp
    assert "failure_code=LOCALIZATION_RESULT_TIMEOUT" in global_cpp
    assert "failure_code=BRIDGE_ACCEPT_TIMEOUT" in global_cpp
    assert "failure_code=MAP_TO_ODOM_TIMEOUT" in global_cpp
    assert "failure_code=MAP_TO_ODOM_WRONG_OWNER" in global_cpp
    assert "failure_code=TF_HISTORY_MISSING" in global_cpp
    assert "restamp" not in global_cpp.lower()

    assert "wait_for_bridge_has_map_to_odom" in nav_runtime
    assert "start_amcl_resident_if_enabled_for_navigation" in nav_runtime
    assert "complete_amcl_readiness_if_enabled_for_navigation" in nav_runtime
    assert "wait_for_fresh_header_topic_message" not in nav_runtime
    assert "NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-60" in nav_runtime
    assert "GLOBAL_LOCALIZATION_TRIGGER_TIMEOUT" in nav_runtime
    assert "LOCALIZATION_RESULT_TIMEOUT" in nav_runtime
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
    assert "nav2_amcl::DifferentialMotionModel" in amcl_cfg

    assert "NJRH_AMCL_LOCALIZATION_MODE:-gated" in amcl_mode
    assert "NJRH_AMCL_TF_WARMUP_SEC" in amcl_mode
    assert "NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC" in amcl_mode
    assert "NJRH_AMCL_SEED_RETRY_COUNT" in amcl_mode
    assert "NJRH_AMCL_SCAN_ADMISSION_ENABLED" in amcl_mode
    assert "NJRH_AMCL_SCAN_OUTPUT_TOPIC" in amcl_mode
    assert "NJRH_AMCL_SCAN_MAX_AGE_MS:-250.0" in amcl_mode
    assert "NJRH_AMCL_SCAN_WAIT_FOR_TF_TIMEOUT_MS:-20.0" in amcl_mode
    assert "disabled|shadow|gated" in amcl_runner
    assert "ros2 run nav2_amcl amcl" in amcl_runner
    assert "complete_amcl_readiness_sequence" in amcl_runner
    assert "wait_for_amcl_tf_warmup" in amcl_runner
    assert 'source "${SCRIPT_DIR}/map_server_helpers.sh"' in amcl_runner
    assert 'wait_for_occupancy_grid "/map" "${map_timeout}"' in amcl_runner
    assert 'wait_for_topic_message "/map" "${map_timeout}"' not in amcl_runner
    assert '"/${AMCL_NODE_NAME}/change_state"' in amcl_runner
    assert "lifecycle_msgs/srv/ChangeState" in amcl_runner
    assert '"/${AMCL_NODE_NAME}/get_state"' in amcl_runner
    assert "lifecycle_msgs/srv/GetState" in amcl_runner
    assert '[[ "${state}" != active* && "${state}" != inactive* ]]' in amcl_runner
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
        assert "amcl_max_result_age_ms: 500.0" in cfg
        assert "amcl_small_correction_translation_m: 0.30" in cfg
        assert "amcl_small_correction_yaw_rad: 0.20" in cfg
        assert "amcl_medium_correction_translation_m: 0.70" in cfg
        assert "amcl_medium_correction_yaw_rad: 0.20" in cfg
        assert "amcl_medium_correction_consistency_count: 3" in cfg
        assert "amcl_hard_reject_translation_m: 1.20" in cfg
        assert "amcl_max_xy_covariance: 1.0" in cfg
        assert "amcl_max_yaw_covariance: 0.5" in cfg
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
    assert 'nohup taskset -c "${amcl_cpuset}" ros2 run nav2_amcl amcl' in amcl_runner
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
    assert "tf_future_stamp_offset_sec: 0.10" in bridge_cfg
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
    assert "install(TARGETS localization_bridge_node amcl_scan_admission_node" in cmake
    for dep in ("sensor_msgs", "tf2", "tf2_ros", "geometry_msgs", "rclcpp"):
        assert dep in cmake
        assert f"<depend>{dep}</depend>" in package

    assert 'export NJRH_AMCL_SCAN_ADMISSION_IMPL="${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}"' in profile
    assert 'SCAN_RELAY_IMPL="${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}"' in runner
    assert "SCAN_RELAY_CPP_BIN" in runner
    assert "ros2 run robot_localization_bridge amcl_scan_admission_node" in runner
    assert "python3 \"${SCAN_RELAY_SCRIPT}\"" in runner
    assert "explicitly set NJRH_AMCL_SCAN_ADMISSION_IMPL=python" in runner
    assert "implementation=${SCAN_RELAY_IMPL}" in runner
    assert "max_scan_age_ms=${scan_max_age_ms}" in runner
    assert "tf_wait_timeout_ms=${scan_wait_for_tf_timeout_ms}" in runner
    assert "nohup taskset -c \"${relay_cpuset}\" \"${relay_cmd[@]}\"" in runner
    assert "killall -9" not in runner
    assert "pkill -9" not in runner

    assert "declare_parameter<std::string>(\"input_topic\", \"/scan\")" in cpp
    assert "declare_parameter<std::string>(\"output_topic\", \"/scan_amcl\")" in cpp
    assert "declare_parameter<std::string>(\"target_frame\", \"odom\")" in cpp
    assert "declare_parameter<double>(\"max_rate_hz\", 5.0)" in cpp
    assert "declare_parameter<double>(\"max_scan_age_ms\", 250.0)" in cpp
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
    assert "tf_future_stamp_offset_sec: 0.10" in bridge_cfg
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
    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(encoding="utf-8")
    runtime_path = scripts_dir / "run_navigation_runtime_services.sh"
    runtime = runtime_path.read_text(encoding="utf-8")
    amcl_runner_path = scripts_dir / "run_amcl_shadow_localization.sh"
    amcl_runner = amcl_runner_path.read_text(encoding="utf-8")
    verify_path = scripts_dir / "verify_amcl_runtime_readiness.sh"
    verify_contract_path = scripts_dir / "verify_amcl_runtime_contract.sh"
    verify_status_path = scripts_dir / "verify_amcl_readiness_status.sh"
    verify_nomotion_path = scripts_dir / "verify_amcl_nomotion_readiness.sh"
    nomotion_probe_path = scripts_dir / "amcl_nomotion_update_probe.py"
    observe_path = scripts_dir / "observe_amcl_navigation_shadow_180s.sh"
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
    assert "wait_for_amcl_tf_warmup false" in amcl_runner
    assert "wait_for_amcl_tf_warmup true" in amcl_runner
    assert "start_scan_admission_relay || return 2" in amcl_runner
    assert "wait_for_scan_admission_status_ready || return 3" in amcl_runner
    assert "seed_amcl_initial_pose || return 4" in amcl_runner
    assert 'status_log_period_sec:=${NJRH_AMCL_SCAN_ADMISSION_STATUS_LOG_PERIOD_SEC:-1.0}' in amcl_runner
    assert "request_amcl_nomotion_update" in amcl_runner
    assert "request_amcl_nomotion_update_and_wait_for_pose" in amcl_runner
    assert "/request_nomotion_update" in amcl_runner
    assert "amcl_nomotion_update_probe.py" in amcl_runner
    assert "ros2 service call \"${service}\" std_srvs/srv/Empty" not in amcl_runner
    assert "wait_for_amcl_pose_fresh_or_nomotion_update" in amcl_runner
    assert "AMCL_RESIDENT mode=${MODE}" in amcl_runner
    assert "AMCL_READY mode=${MODE}" in amcl_runner
    assert "NJRH_AMCL_RUNTIME_STATUS_FILE" in amcl_runner
    assert "/tmp/njrh_amcl_runtime_status.env" in amcl_runner
    assert "AMCL_EXIT_DEGRADED=10" in amcl_runner
    assert "AMCL_EXIT_GATED_NOT_READY=21" in amcl_runner
    assert "AMCL_EXIT_SCAN_ADMISSION_FAILED=22" in amcl_runner
    assert "AMCL_EXIT_LIFECYCLE_FAILED=23" in amcl_runner
    assert "AMCL_EXIT_SEED_FAILED=24" in amcl_runner
    assert "AMCL_EXIT_POSE_MISSING=25" in amcl_runner
    assert "write_amcl_runtime_status" in amcl_runner
    assert "AMCL_STATE" in amcl_runner
    assert "AMCL_STATUS_STAMP_SEC" in amcl_runner
    assert "AMCL_STATUS_STALE" in amcl_runner
    assert "AMCL_PROCESS_READY" in amcl_runner
    assert "AMCL_SEED_RESPONSE_OK" in amcl_runner
    assert "AMCL_NOMOTION_PROBE_USED" in amcl_runner
    assert "AMCL_NOMOTION_POSE_RECEIVED" in amcl_runner
    assert "AMCL_STATIC_STANDBY" in amcl_runner
    assert "AMCL_CORRECTION_READY" in amcl_runner
    assert "AMCL_DEGRADED" in amcl_runner
    assert "AMCL_FAILED" in amcl_runner
    assert "AMCL_PID_STALE_CLEARED" in amcl_runner
    assert "SCAN_ADMISSION_PID_STALE_CLEARED" in amcl_runner
    assert "validated_pid_from_file" in amcl_runner
    assert "pid_cmdline_matches" in amcl_runner
    assert "finish_amcl_status degraded false true" in amcl_runner
    assert "finish_amcl_status failed false false" in amcl_runner
    assert "continuing triggered localization baseline\"\\n    return 0" not in amcl_runner
    assert 'AMCL lifecycle is active but not ready; continuing triggered localization baseline"\n    return 0' not in amcl_runner

    assert "start_amcl_resident_if_enabled_for_navigation" in runtime
    assert "complete_amcl_readiness_if_enabled_for_navigation" in runtime
    assert "run_amcl_localization_step" in runtime
    assert "set +e" in runtime
    assert "rc=$?" in runtime
    assert "AMCL_DEGRADED phase=${phase}" in runtime
    assert "AMCL_GATED_NOT_READY" in runtime
    assert "AMCL_SCAN_ADMISSION_FAILED" in runtime
    assert "AMCL_POSE_MISSING" in runtime
    main_flow = runtime.split('if resident_navigation_ready; then', 1)[1]
    assert main_flow.index("start_amcl_resident_if_enabled_for_navigation") < main_flow.index(
        "trigger_global_localization_for_navigation"
    )
    assert main_flow.index("trigger_global_localization_for_navigation") < main_flow.index(
        "complete_amcl_readiness_if_enabled_for_navigation"
    )
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
    assert "amcl_pose_age_ms" in bridge_cpp
    assert "amcl_pose_fresh" in bridge_cpp
    assert "amcl_not_moving_no_update_ok" in bridge_cpp
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
    assert "amcl_runtime_status_file" in api_cpp
    assert "read_amcl_runtime_status" in api_cpp
    assert "amcl_state" in api_cpp
    assert "amcl_status_file_stale" in api_cpp
    assert "amcl_process_ready" in api_cpp
    assert "amcl_seed_response_ok" in api_cpp
    assert "amcl_nomotion_pose_received" in api_cpp
    assert "amcl_static_standby" in api_cpp
    assert "amcl_correction_ready" in api_cpp
    assert "localization_degraded" in api_cpp
    assert "localization_degraded_reason" in api_cpp
    assert "using_triggered_baseline_only" in api_cpp
    assert "amcl_scan_admission_status_publisher_count" in api_cpp

    assert "tf_broadcast: false" in amcl_cfg
    assert "scan_topic: /scan_amcl" in amcl_cfg
    assert "map_topic: /map" in amcl_cfg
    assert "update_min_d: 0.05" in amcl_cfg
    assert "update_min_a: 0.05" in amcl_cfg
    assert "robot_model_type: nav2_amcl::DifferentialMotionModel" in amcl_cfg
    assert "amcl_input_enabled: false" in bridge_cfg
    assert "amcl_gate_mode: shadow" in bridge_cfg
    assert "amcl_small_correction_translation_m: 0.30" in bridge_cfg
    assert "amcl_medium_correction_translation_m: 0.70" in bridge_cfg
    assert "amcl_medium_correction_consistency_count: 3" in bridge_cfg
    assert "amcl_hard_reject_translation_m: 1.20" in bridge_cfg
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
    assert 'MODE="${NJRH_AMCL_LOCALIZATION_MODE:-gated}"' in verify
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


def test_phase_a22_amcl_readiness_stale_status_and_nomotion_race_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"

    amcl_runner = (scripts_dir / "run_amcl_shadow_localization.sh").read_text(encoding="utf-8")
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
    bridge_cfg = (config_dir / "localization_bridge.yaml").read_text(encoding="utf-8")
    api_cfg = (config_dir / "robot_api_server.yaml").read_text(encoding="utf-8")

    assert "AMCL_STATUS_STAMP_SEC" in amcl_runner
    assert "AMCL_STATUS_AGE_MS" in amcl_runner
    assert "AMCL_STATUS_STALE" in amcl_runner
    assert "AMCL_PROCESS_READY" in amcl_runner
    assert "AMCL_SEEDED" in amcl_runner
    assert "AMCL_STATIC_STANDBY" in amcl_runner
    assert "AMCL_TRACKING_READY" in amcl_runner
    assert "AMCL_CORRECTION_READY" in amcl_runner
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
    assert 'amcl_gate_mode_ == "gated" && amcl_correction_ready' in bridge_cpp
    assert "amcl_gated_ready = amcl_shadow_ready && amcl_gate_mode_ == \"gated\"" not in bridge_cpp
    assert "AMCL_STATIC_STANDBY_NO_CORRECTION" in bridge_cpp
    assert "AMCL_CORRECTION_NOT_READY" in bridge_cpp

    assert "amcl_runtime_status_ttl_sec" in api_cpp
    assert "bridge_status.amcl_correction_ready" in api_cpp
    assert "amcl_status_file_stale" in api_cpp
    assert "amcl_seed_response_ok" in api_cpp
    assert "amcl_nomotion_pose_received" in api_cpp
    assert "amcl_status_source" in api_cpp
    assert "stale_file_ignored" in api_cpp
    assert "using_triggered_baseline_only" in api_cpp

    assert "amcl_runtime_status_ttl_sec: 5.0" in bridge_cfg
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
    assert "ros2 run" in cleanup
    assert "ros2 launch" in cleanup
    assert "kill -TERM" in cleanup
    assert "kill -KILL" not in cleanup
    assert "pkill -9" not in cleanup
    assert "killall -9" not in cleanup

    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "tf_future_stamp_offset_sec: 0.10" in bridge_cfg
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


def test_phase_l2_post_relocalization_settle_barrier_contracts():
    overlay = ROOT / "scripts" / "jetson" / "runtime_overlay"
    config_dir = overlay / "config"
    scripts_dir = overlay / "scripts"

    api_cpp = (ROOT / "src" / "robot_api_server" / "src" / "robot_api_server_node.cpp").read_text(
        encoding="utf-8"
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
    verify_script_path = scripts_dir / "verify_post_relocalization_settle_barrier.sh"
    observe_script_path = scripts_dir / "observe_relocalization_to_next_stage.sh"
    verify_script = verify_script_path.read_text(encoding="utf-8")
    observe_script = observe_script_path.read_text(encoding="utf-8")

    for cfg in (api_cfg, overlay_api_cfg):
        assert "localization_bridge_status_topic: \"/localization/bridge_status\"" in cfg
        assert "local_costmap_topic: \"/local_costmap/costmap\"" in cfg
        assert "post_relocalization_settle_enabled: true" in cfg
        assert "post_relocalization_settle_min_ms: 800" in cfg
        assert "post_relocalization_settle_max_ms: 3000" in cfg
        assert "post_relocalization_stable_tf_samples: 5" in cfg
        assert "post_relocalization_tf_sample_period_ms: 100" in cfg
        assert "post_relocalization_required_local_costmap_updates: 2" in cfg
        assert "post_relocalization_large_correction_translation_m: 0.5" in cfg
        assert "post_relocalization_large_correction_yaw_rad: 0.3" in cfg
        assert "post_relocalization_large_correction_min_ms: 1500" in cfg

    assert 'candidate.source == "isaac_triggered" && candidate.explicit_trigger' in bridge_cpp
    assert "last_explicit_relocalization_sequence_" in bridge_cpp
    assert "last_explicit_relocalization_accept_time" in bridge_cpp
    assert "localization_settle_required" in bridge_cpp

    assert "wait_for_post_relocalization_settle_barrier(" in api_cpp
    assert '"post_undock"' in api_cpp
    assert '"before_predock"' in api_cpp
    assert '"after_predock"' in api_cpp
    assert '"manual_before_navigation"' in api_cpp
    assert '"nav2_goal"' in api_cpp
    assert '"fine_docking"' in api_cpp
    assert "local_costmap_message_filter_drop_count()" in api_cpp
    assert "base_to_lidar_static_tf_ready()" in api_cpp
    assert "post_relocalization_settle_state_json()" in api_cpp
    assert "publish_teleop_zero_burst()" in api_cpp
    assert "publish_final_yaw_align_zero_burst()" in api_cpp

    for code in [
        "POST_RELOCALIZATION_SETTLE_TIMEOUT",
        "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH",
        "POST_RELOCALIZATION_ODOM_BASE_NOT_FRESH",
        "POST_RELOCALIZATION_TF_CHAIN_UNSTABLE",
        "POST_RELOCALIZATION_LOCAL_COSTMAP_NOT_UPDATED",
        "POST_RELOCALIZATION_LOCAL_COSTMAP_TF_DROPS",
        "POST_RELOCALIZATION_SCAN_ADMISSION_TF_ERROR",
        "POST_RELOCALIZATION_WRONG_MAP_ODOM_OWNER",
        "POST_RELOCALIZATION_SEQUENCE_MISMATCH",
        "CANCELLED_BY_APP",
    ]:
        assert code in api_cpp

    assert verify_script_path.exists()
    assert observe_script_path.exists()
    assert "never sends" in verify_script
    assert "ros2 action send_goal" not in verify_script
    assert "ros2 topic pub" not in verify_script
    assert "does not send goals or velocity" in observe_script
    assert "ros2 action send_goal" not in observe_script
    assert "ros2 topic pub" not in observe_script

    assert "transform_tolerance: 0.10" in nav2_cfg
    assert "transform_tolerance: 0.15" in local_costmap_config_block(nav2_cfg)
    assert "max_odom_tf_age_ms: 100.0" in bridge_cfg
    assert "MPPIController" in nav2_cfg
    assert "SmacPlanner2D" in nav2_cfg
    assert "PointCloud2" not in api_cpp
    assert "FAST-LIO" not in api_cpp
    assert "pkill -9" not in verify_script + observe_script
    assert "killall -9" not in verify_script + observe_script

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
    assert "ros2 topic echo /perception/local_perception_status" in script_text
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
        "local_worker_start_source_age_ms",
        "local_worker_end_source_age_ms",
        "obstacle_output_header_age_ms",
        "obstacle_output_source_age_ms",
        "obstacle_output_publish_delay_ms",
        "obstacle_output_frame_id",
        "obstacle_output_header_stamp_source=source_stamp",
        "clearing_output_header_age_ms",
        "clearing_output_source_age_ms",
        "clearing_output_publish_delay_ms",
        "clearing_output_frame_id",
        "clearing_output_header_stamp_source=source_stamp",
        "scan_output_header_age_ms",
        "scan_output_source_age_ms",
        "scan_output_frame_id",
        "tf_drop_suspect_obstacle_header_age_over_100ms_count",
        "tf_drop_suspect_obstacle_header_age_over_200ms_count",
    ):
        assert field in accel_axis

    for field in (
        "input_header_age_ms",
        "input_receive_age_ms",
        "obstacle_output_header_age_ms",
        "obstacle_output_source_age_ms",
        "obstacle_output_frame_id",
        "clearing_output_header_age_ms",
        "clearing_output_source_age_ms",
        "clearing_output_frame_id",
    ):
        assert field in local_perception

    assert "global_frame: odom" in nav2
    assert "global_frame: base_link" not in nav2
    assert "sensor_frame: base_link" in nav2
    assert "tf_filter_tolerance" not in nav2
    assert "observation_persistence: 0.0" in nav2
    assert "obstacle_output_stamp_policy" not in accel_cfg
    assert "obstacle_output_stamp_policy" not in local_perception_cfg
    assert "restamp_to_now: false" in local_perception_cfg
    assert "restamp_to_latest_tf: false" in local_perception_cfg
    assert "input_reliable: false" in accel_cfg
    assert "output_reliable: false" in accel_cfg
    assert "input_reliable: false" in local_perception_cfg
    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"' in common_env
    assert "rmw_cyclonedds_cpp" not in common_env
    assert "stamp=now" not in phase_doc
    assert "does not restamp clouds" in phase_doc

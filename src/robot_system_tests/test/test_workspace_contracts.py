import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]


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
    assert "SmacPlanner2D" in nav2
    assert "MPPIController" in nav2
    assert "RegulatedPurePursuitController" in nav2
    assert 'motion_model: "Ackermann"' in nav2
    assert "AckermannConstraints:" in nav2
    assert "min_turning_r: 0.81" in nav2
    assert "vx_std: 0.28" in nav2
    assert "vy_std: 0.0" in nav2
    assert "wz_std: 0.45" in nav2
    assert "vx_max: 0.55" in nav2
    assert "vx_min: 0.0" in nav2
    assert "vy_max: 0.0" in nav2
    assert "wz_max: 0.70" in nav2
    assert "nav2_rotation_shim_controller::RotationShimController" in nav2
    assert 'primary_controller: "nav2_mppi_controller::MPPIController"' in nav2
    assert "angular_dist_threshold: 0.85" in nav2
    assert "use_rotate_to_heading: false" in nav2
    assert "/perception/obstacle_points" in nav2
    assert "/perception/clearing_points" in nav2
    assert "origin_z: -0.20" in nav2
    assert "z_voxels: 16" in nav2
    assert "min_obstacle_height: -0.20" in nav2
    assert "max_obstacle_height: 1.40" in nav2
    assert "sensor_frame: lidar_link" in nav2
    assert "clearing: true" in nav2
    assert "observation_persistence: 0.0" in nav2
    assert "controller_frequency: 12.0" in nav2
    assert "speed_limit_topic: /speed_limit" in nav2
    assert "time_steps: 44" in nav2
    assert "model_dt: 0.09" in nav2
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
    assert "source_timeout: 0.6" in nav2
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


def test_tf_policy_is_canonical():
    tf_policy = (ROOT / "src" / "robot_nav_config" / "config" / "tf_policy.yaml").read_text(encoding="utf-8")
    assert "robot_localization_bridge" in tf_policy
    assert "robot_local_state" in tf_policy
    assert "suppress_third_party_tf: true" in tf_policy


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
        assert "cmd_vel_topic: /cmd_vel_collision_checked" in cfg
        assert "status_topic: /docking/status" in cfg
        assert "start_service: /docking/start" in cfg
        assert "stop_service: /docking/stop" in cfg
        assert "undock_service: /docking/undock" in cfg
        assert "forced_mode_topic: /ranger_mini3/forced_mode" in cfg
        assert "park_topic: /ranger_mini3/park" in cfg
        assert "reverse_enable_topic: /ranger_mini3/allow_reverse" in cfg
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
        assert "use_yaw_fit: false" in cfg
        assert "pre_dock_distance_m: 0.60" in cfg
        assert "distance_m: 0.60" in cfg
        assert "speed_mps: 0.06" in cfg
        assert "min_clear_distance_m: 0.45" in cfg
        assert "timeout_s: 12.0" in cfg
        assert "max_angular_speed_radps: 0.12" in cfg
        assert "ky: 0.55" in cfg
        assert "ky_lateral: 0.70" in cfg
        assert "lateral_command_sign: -1.0" in cfg
        assert "kyaw: 0.00" in cfg
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
    assert "<name>robot_docking_manager</name>" in package_xml
    assert "<exec_depend>robot_nav_config</exec_depend>" in package_xml
    assert 'declare_parameter<std::string>("gs2_scan_topic", "/dock/gs2_scan")' in node
    assert 'declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_collision_checked")' in node
    assert 'declare_parameter<std::string>("start_service", "/docking/start")' in node
    assert 'declare_parameter<std::string>("stop_service", "/docking/stop")' in node
    assert 'declare_parameter<std::string>("undock_service", "/docking/undock")' in node
    assert 'declare_parameter<std::string>("mode.forced_mode_topic", "/ranger_mini3/forced_mode")' in node
    assert 'declare_parameter<std::string>("mode.reverse_enable_topic", "/ranger_mini3/allow_reverse")' in node
    assert 'declare_parameter<bool>("mode.use_crab_mode", true)' in node
    assert "create_service<std_srvs::srv::Trigger>" in node
    assert "create_subscription<sensor_msgs::msg::LaserScan>" in node
    assert "create_subscription<sensor_msgs::msg::BatteryState>" in node
    assert "State::BlindApproach" in node
    assert "State::ContactVerify" in node
    assert "State::Undocking" in node
    assert "start_undocking" in node
    assert "handle_undocking" in node
    assert "publish_reverse_enable(true)" in node
    assert "cmd.linear.x = -speed" in node
    assert "POWER_SUPPLY_STATUS_CHARGING" in node
    assert "battery_indicates_charging" in node
    assert "docked_stop(\"docked_charging_detected\")" in node
    assert 'declare_parameter<bool>("detector.use_yaw_fit", false)' in node
    assert "filter_detection" in node
    assert "limit_yaw_rate_for_ackermann" in node
    assert "valid_detection_streak_" in node
    assert "min_align_speed_mps_" in node
    assert "lateral_command_sign_ * ky_lateral_ * lateral_error" in node
    assert "min_lateral_speed_mps_" in node
    assert "lock_lateral_during_final_insert_" in node
    assert "cmd.linear.y = 0.0;" in node
    assert "if (!final_insert_locked && (!lateral_ok || !yaw_ok" in node
    assert "release_docking_motion_mode(park_on_docked_)" in node
    assert "if (!distance_ok && distance_error > 0.0)" in node
    assert "FindPackageShare(\"robot_nav_config\")" in launch
    assert "install/robot_docking_manager/lib/robot_docking_manager/docking_manager_node" in runner
    assert "Python fallback has been removed" in runner
    assert "/cmd_vel_collision_checked" in readme
    assert "/docking/undock" in readme
    assert "/ranger_mini3/allow_reverse=true" in readme
    assert "robot_safety" in readme
    assert "rosbag" in readme
    assert "Do not publish docking control directly to `/cmd_vel_safe`" in gs2_doc
    assert "POST /api/v1/docking/undock" in gs2_doc


def test_local_state_uses_robot_localization_ekf_with_system_time_driver():
    package_xml = (ROOT / "src" / "robot_local_state" / "package.xml").read_text(encoding="utf-8")
    launch_file = (ROOT / "src" / "robot_local_state" / "launch" / "local_state.launch.py").read_text(
        encoding="utf-8"
    )
    ekf_cfg = (ROOT / "src" / "robot_local_state" / "config" / "local_state_ekf.yaml").read_text(
        encoding="utf-8"
    )
    overlay_ekf_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state_ekf.yaml"
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
    overlay_passthrough_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "local_state.yaml"
    ).read_text(encoding="utf-8")
    ranger_chassis_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_ranger_chassis.sh"
    ).read_text(encoding="utf-8")
    readme = (ROOT / "src" / "robot_local_state" / "README.md").read_text(encoding="utf-8")

    assert "<exec_depend>robot_localization</exec_depend>" in package_xml
    assert 'package="robot_localization"' in launch_file
    assert 'executable="ekf_node"' in launch_file
    assert "local_state_ekf.yaml" in launch_file
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
    assert "-r /odometry/filtered:=/local_state/odometry" in overlay_runner
    assert "LOCAL_STATE_MODE=passthrough" in readme
    assert 'declare_parameter<double>("odom_yaw_offset_rad", 0.0)' in local_state_node
    assert 'declare_parameter<bool>("rotate_odom_position_with_yaw_offset", true)' in local_state_node
    assert 'declare_parameter<std::string>("input_base_frame", "ranger_base_link")' in local_state_node
    assert "apply_canonical_odom_transform(local_odom)" in local_state_node
    assert 'export BASE_FRAME="${BASE_FRAME:-ranger_base_link}"' in ranger_chassis_runner
    assert "input_base_frame: ranger_base_link" in overlay_passthrough_cfg
    assert "odom_yaw_offset_rad: 0.0" in overlay_passthrough_cfg
    assert "rotate_odom_position_with_yaw_offset: false" in overlay_passthrough_cfg
    assert "robot_localization/ekf_node" in overlay_tf_helpers
    assert "ekf_node --ros-args.*__node:=robot_local_state" in overlay_tf_helpers
    assert "local_state_node --ros-args" in overlay_tf_helpers
    for cfg in (ekf_cfg, overlay_ekf_cfg):
        assert "two_d_mode: true" in cfg
        assert "publish_tf: true" in cfg
        assert "world_frame: odom" in cfg
        assert "base_link_frame: base_link" in cfg
        assert "odom0: /wheel/odom" in cfg
        assert "imu0: /lidar_imu" in cfg
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
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_web_dashboard.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_perception.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_local_costmap_debug.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_robot_safety.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "stop_floor_navigation.sh").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_common_services.sh").exists()
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
    assert "robot_global_localization/global_localization_node.py" in stop_navigation
    assert "ranger_base_node" not in stop_navigation
    assert "hesai_ros_driver" not in stop_navigation
    assert "robot_api_server" not in stop_navigation
    assert "robot_safety" not in stop_navigation
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "prepare_localizer_map.py").exists()
    assert (ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_axis_remap_node.cpp").exists()
    assert (ROOT / "src" / "robot_hesai_jt128" / "src" / "imu_axis_remap_node.cpp").exists()
    assert (ROOT / "src" / "robot_hesai_jt128" / "src" / "scan_republisher_node.cpp").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "frontend_pose_from_odometry.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "release_rebuild_compat.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization.launch.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization_stack.launch.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_localization_sensing.launch.py").exists()
    assert (ROOT / "src" / "robot_bringup" / "launch" / "local_costmap_debug.launch.py").exists()
    assert (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "fastlio.yaml").exists()
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
    assert "mode: trinary" in ensure_masks
    assert "free_thresh: 0.196" in ensure_masks
    assert "mode: trinary" in neutral_keepout_yaml
    assert "free_thresh: 0.196" in neutral_keepout_yaml
    assert "mode: trinary" in neutral_speed_yaml
    assert "free_thresh: 0.196" in neutral_speed_yaml
    assert neutral_keepout_pgm.endswith("254\n")
    assert neutral_speed_pgm.endswith("254\n")


def test_runtime_overlay_lidar_view_defaults_to_base_link():
    dashboard_patch = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "patch_dashboard_runtime.py").read_text(encoding="utf-8")
    run_web_dashboard = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_web_dashboard.sh").read_text(encoding="utf-8")
    assert "--lidar-view-html" in dashboard_patch
    assert "DEFAULT_DISPLAY_FRAME = 'base_link'" in dashboard_patch
    assert "小车底盘坐标 base_link" in dashboard_patch
    assert "原始传感器坐标（仅调试）" in dashboard_patch
    assert "cp \"${UPSTREAM_WEB}/lidar_view.html\"" in run_web_dashboard
    assert "--lidar-view-html" in run_web_dashboard
    assert "ReliabilityPolicy.BEST_EFFORT" in dashboard_patch
    assert "depth=10" in dashboard_patch
    assert "def _driver_stack_running(self) -> bool:" in dashboard_patch
    assert "def _probe_lidar_topic_external(self, timeout: float = 6.0) -> bool:" in dashboard_patch
    assert "ros2 topic hz /lidar_points" in dashboard_patch
    assert "external lidar probe succeeded via ros2 topic hz /lidar_points" in dashboard_patch
    assert "driver live lidar confirmed by external ROS CLI probe; dashboard lidar cache remained stale" in dashboard_patch
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


def test_project_runtime_helpers_are_wired():
    local_perception = (ROOT / "src" / "robot_local_perception" / "config" / "local_perception.yaml").read_text(encoding="utf-8")
    robot_safety = (ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml").read_text(encoding="utf-8")
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
    assert "input_topic: /lidar_points" in local_perception
    assert "output_topic: /perception/obstacle_points" in local_perception
    assert "clearing_output_topic: /perception/clearing_points" in local_perception
    assert "restamp_to_latest_tf: true" in local_perception
    assert "require_output_stamp_tf: true" in local_perception
    assert "output_stamp_tf_target_frame: odom" in local_perception
    assert "clearing.enabled: true" in local_perception
    assert "clearing.virtual_rays.enabled: true" in local_perception
    assert "clearing.virtual_rays.angular_resolution_deg: 0.75" in local_perception
    assert "clearing.virtual_rays.range: 8.00" in local_perception
    assert "clearing.virtual_rays.range_steps: [0.50, 1.00, 2.00, 3.50, 5.50, 8.00]" in local_perception
    assert "clearing.max_points: 30000" in local_perception
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
    assert "install/robot_local_perception/lib/robot_local_perception/local_perception_node" in overlay_local_perception
    assert "Python fallback has been removed" in overlay_local_perception
    assert "src/robot_local_perception/scripts/local_perception_node.py" not in overlay_nav_helpers
    assert "python3 .*local_perception_node.py" not in overlay_nav_helpers
    assert "robot_local_perception/local_perception_node" in overlay_nav_helpers
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
    assert 'keepout_mask_yaml:="${NAV2_KEEP_OUT_MASK_YAML}"' in overlay_nav
    assert 'speed_mask_yaml:="${NAV2_SPEED_MASK_YAML}"' in overlay_nav
    assert "costmap_filter_info_server" in standard_navigation_launch
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
    assert "NJRH_NAV_LOCAL_STATE_MODE:-passthrough" in overlay_common_services
    assert 'env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"' in overlay_common_services
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' in overlay_common_services
    assert "run_local_perception.sh" in overlay_common_services
    assert "run_robot_safety.sh" in overlay_common_services
    assert "run_robot_api_server.sh" in overlay_common_services
    assert "run_gs2_driver.sh" in overlay_common_services
    assert "NJRH_GS2_AUTOSTART" in overlay_common_services
    assert "robot_eai_gs2/gs2_driver_node" in overlay_common_services
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


def test_robot_api_server_is_cpp_gateway_not_dashboard_backend():
    package_root = ROOT / "src" / "robot_api_server"
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
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
    floor_navigation_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_floor_navigation.sh"
    ).read_text(encoding="utf-8")
    floor_asset_helpers = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "floor_asset_helpers.sh"
    ).read_text(encoding="utf-8")
    floor_manager_code = (ROOT / "src" / "robot_floor_manager" / "src" / "floor_manager_node.cpp").read_text(
        encoding="utf-8"
    )
    global_localization_node = (
        ROOT / "src" / "robot_global_localization" / "scripts" / "global_localization_node.py"
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
    global_localization_script = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_global_localization.sh"
    ).read_text(encoding="utf-8")
    app_doc = (ROOT / "docs" / "android_app_api.md").read_text(encoding="utf-8")

    assert "add_executable(robot_api_server_node" in cmake
    assert "geometry_msgs" in cmake
    assert "nav_msgs" in cmake
    assert "robot_interfaces" in cmake
    assert "sensor_msgs" in cmake
    assert "std_msgs" in cmake
    assert "tf2_msgs" in cmake
    assert "<depend>geometry_msgs</depend>" in package_xml
    assert "<depend>nav_msgs</depend>" in package_xml
    assert "<depend>robot_interfaces</depend>" in package_xml
    assert "<depend>sensor_msgs</depend>" in package_xml
    assert "<depend>tf2_msgs</depend>" in package_xml
    assert "socket(AF_INET, SOCK_STREAM" in node_cpp
    assert "Sec-WebSocket-Accept" in node_cpp
    assert "websocket_accept_key" in node_cpp
    assert "configure_runtime_permissions" in node_cpp
    assert "robot_api_server refuses to run as root" not in node_cpp
    assert "NJRH_ALLOW_ROOT_API_SERVER" not in node_cpp
    assert "::umask(0002)" in node_cpp
    assert "geometry_msgs::msg::Twist" in node_cpp
    assert "sensor_msgs::msg::BatteryState" in node_cpp
    assert "sensor_msgs::msg::LaserScan" in node_cpp
    assert "tf2_msgs::msg::TFMessage" in node_cpp
    assert "class SubscriptionManager" in node_cpp
    assert "/ws/v1/teleop" in node_cpp
    assert "/cmd_vel_collision_checked" in node_cpp
    assert "std_msgs::msg::Bool" in node_cpp
    assert "robot_interfaces::srv::SwitchFloor" in node_cpp
    assert "robot_interfaces::srv::TriggerLocalization" in node_cpp
    assert "/api/v1/status" in node_cpp
    assert "bms_state_topic" in node_cpp
    assert "teleop_stop_on_charging" in node_cpp
    assert "teleop_charging_current_min_a" in node_cpp
    assert "battery_indicates_charging" in node_cpp
    assert "teleop_charging_guard_active" in node_cpp
    assert "charging detected; teleop command stopped" in node_cpp
    assert "\\\"bms\\\"" in node_cpp
    assert "\\\"soc\\\"" in node_cpp
    assert "\\\"soc_valid\\\"" in node_cpp
    assert "/api/v1/robot/pose" in node_cpp
    assert "handle_robot_pose" in node_cpp
    assert "wait_for_current_robot_pose" in node_cpp
    assert "robot_pose_freshness_sec" in node_cpp
    assert "no fresh map-frame robot pose" in node_cpp
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
    assert "keepout_semantic_layer.json" in node_cpp
    assert "/api/v1/navigation/goal" in node_cpp
    assert "/api/v1/navigation/cancel" in node_cpp
    assert "handle_navigation_goal" in node_cpp
    assert "handle_navigation_cancel" in node_cpp
    assert "NavigateToPose" in node_cpp
    assert "navigate_to_pose_action" in node_cpp
    assert "navigate_action_mutex_" in node_cpp
    assert "active_nav_goal_handle_" in node_cpp
    assert "exception sending navigation goal" in node_cpp
    assert "async_cancel_goal" in node_cpp
    assert "async_cancel_all_goals" in node_cpp
    assert "exception canceling cached goal handle" in node_cpp
    assert "exception canceling navigation goals" in node_cpp
    assert "unhandled API exception" in node_cpp
    assert "log_http_request" in node_cpp
    assert "::listen(server_fd_, 64)" in node_cpp
    assert "SO_RCVTIMEO" in node_cpp
    assert "SO_SNDTIMEO" in node_cpp
    assert "failed to send full HTTP response" in node_cpp
    assert "Taking data from action client but no ready event" in node_cpp
    assert "continuing after transient action client executor exception" in node_cpp
    assert "set_close_on_exec(server_fd_)" in node_cpp
    assert "close_inherited_fds()" in node_cpp
    assert "timed out waiting for navigation stop command" in node_cpp
    assert "publish_teleop_zero_burst" in node_cpp
    assert "zero_velocity_published" in node_cpp
    assert "action_available" in node_cpp
    assert "cancel_all_detail" in node_cpp
    assert "navigation_stop_command" in node_cpp
    assert "stop_navigation_runtime_stack" in node_cpp
    assert "cancel_requested" in node_cpp
    assert "navigation_stack_stopped" in node_cpp
    assert "stop_stack" in node_cpp
    assert "read_floor_poses" in node_cpp
    assert "write_floor_poses" in node_cpp
    assert "/api/v1/subscriptions/acquire" in node_cpp
    assert "/api/v1/subscriptions/release" in node_cpp
    assert "/api/v1/subscriptions/heartbeat" in node_cpp
    assert "set_subscription_resource_active" in node_cpp
    assert "live_map resource is not acquired" in node_cpp
    assert "subscription_manager_->release(websocket_client_id" in node_cpp
    assert "subscription_client_id_from_body" in node_cpp
    assert "clientId" in node_cpp
    assert "lease_id" in node_cpp
    assert "http:compat-default" in node_cpp
    assert "resources_for_client(client_id)" in node_cpp
    assert "\\\"refreshed\\\":false" in node_cpp
    assert "/api/v1/mapping/2d/map" in node_cpp
    assert "image/png" in node_cpp
    assert "nav_msgs::msg::OccupancyGrid" in node_cpp
    assert "encode_grayscale_png" in node_cpp
    assert "handle_start_mapping_2d" in node_cpp
    assert "handle_stop_mapping_2d" in node_cpp
    assert "handle_save_mapping_2d" in node_cpp
    assert "handle_delete_map" in node_cpp
    assert "runtime_map_asset_paths" in node_cpp
    assert "MapManifest" in node_cpp
    assert "generate_map_id" in node_cpp
    assert "display_name" in node_cpp
    assert "floor_maps" in node_cpp
    assert "map_info" in node_cpp
    assert "read_nav_map_info" in node_cpp
    assert "read_pgm_dimensions" in node_cpp
    assert "manifest.json" in node_cpp
    assert "maps/<map_id>" not in node_cpp
    assert "\"maps\"" in node_cpp
    assert "\"current\"" in node_cpp
    assert "activate_map_manifest" in node_cpp
    assert "remove_current_map_entry" in node_cpp
    assert "refusing unsafe current map reset path" in node_cpp
    assert ".stale_current_" in node_cpp
    assert "quarantined stale current map entry" in node_cpp
    assert "delete by map_id only" in node_cpp
    assert "ensure_legacy_floor_map_manifest" in node_cpp
    assert "legacy_" in node_cpp
    assert "write_neutral_filter_assets" in node_cpp
    assert "254U" in node_cpp
    assert "robot_api_server_slam_toolbox_save" in node_cpp
    assert "terminate_mapping_2d_process_groups_locked" in node_cpp
    assert "discover_mapping_2d_process_groups" in node_cpp
    assert "terminate_mapping_2d_residual_processes" in node_cpp
    assert "discover_mapping_2d_residual_processes" in node_cpp
    assert "mapping_was_active" in node_cpp
    assert "stopped_groups" in node_cpp
    assert '"fastlio_mapping"' in node_cpp
    assert '"laser_mapping"' in node_cpp
    assert '"nav_cloud_preprocessor"' in node_cpp
    assert '"scan_republisher_node"' in node_cpp
    assert "mapping_2d_start_command" in node_cpp
    assert "navigation_resume_command" in node_cpp
    assert "handle_resume_floor_navigation" in node_cpp
    assert "run_floor_navigation.sh" in node_cpp
    assert "run_projected_map.sh" in node_cpp
    assert "jt128_slam_toolbox_mapping.launch.py" in node_cpp
    assert "live slam_toolbox /map" in node_cpp
    assert "newest_png_in_directory" in node_cpp
    assert "resolve_mapping_2d_png" in node_cpp
    assert "/api/v1/safety/stop" in node_cpp
    assert "/api/v1/safety/resume" in node_cpp
    assert "/api/v1/floors/switch" in node_cpp
    assert "/api/v1/localization/trigger" in node_cpp
    assert "endpoint is reserved but not wired to a ROS-native service/action yet" in node_cpp
    assert "api_token" in config
    assert "bms_state_topic: \"/battery_state\"" in config
    assert "bms_state_max_age_sec: 3.0" in config
    assert "teleop_stop_on_charging: true" in config
    assert "teleop_charging_current_min_a: 0.10" in config
    assert "robot_pose_freshness_sec: 0.5" in config
    assert "runtime_map_context_file: \"/tmp/njrh_runtime_map_context.json\"" in config
    assert "navigate_to_pose_action: \"/navigate_to_pose\"" in config
    assert "mapping_2d_start_command" in config
    assert "run_projected_map.sh" in config
    assert "navigation_resume_command" in config
    assert "run_floor_navigation.sh" in config
    assert "docking_undock_service: \"/docking/undock\"" in config
    assert "docking_pre_dock_distance_m: 0.60" in config
    assert "navigation_auto_undock_timeout_sec: 18.0" in config
    assert "mapping_2d_live_map_topic: \"/map\"" in config
    assert "scan_topic: \"/scan\"" in config
    assert "tf_topic: \"/tf\"" in config
    assert "subscription_default_ttl_ms: 10000" in config
    assert "teleop_cmd_topic: \"/cmd_vel_collision_checked\"" in config
    assert "teleop_reverse_enable_topic: \"/ranger_mini3/allow_reverse\"" in config
    assert "teleop_pose_topic: \"/local_state/odometry\"" in config
    assert "teleop_allow_reverse: true" in config
    assert "teleop_require_mapping_active: true" in config
    assert "teleop_watchdog_timeout_sec: 0.5" in config
    assert "teleop_socket_idle_timeout_sec: 5.0" in config
    assert "teleop_repeat_rate_hz: 20.0" in config
    assert "on_teleop_repeat_timer" in node_cpp
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
    assert "robot_pose_freshness_sec: 0.5" in overlay_config
    assert "runtime_map_context_file: \"/tmp/njrh_runtime_map_context.json\"" in overlay_config
    assert "navigate_to_pose_action: \"/navigate_to_pose\"" in overlay_config
    assert "scan_topic: \"/scan\"" in overlay_config
    assert "tf_topic: \"/tf\"" in overlay_config
    assert "subscription_default_ttl_ms: 10000" in overlay_config
    assert "mapping_2d_start_command" in overlay_config
    assert "navigation_resume_command" in overlay_config
    assert "run_floor_navigation.sh" in overlay_config
    assert "docking_undock_service: \"/docking/undock\"" in overlay_config
    assert "docking_pre_dock_distance_m: 0.60" in overlay_config
    assert "navigation_auto_undock_timeout_sec: 18.0" in overlay_config
    assert "teleop_cmd_topic: \"/cmd_vel_collision_checked\"" in overlay_config
    assert "teleop_reverse_enable_topic: \"/ranger_mini3/allow_reverse\"" in overlay_config
    assert "teleop_socket_idle_timeout_sec: 5.0" in overlay_config
    assert "teleop_repeat_rate_hz: 20.0" in overlay_config
    assert "ROBOT_API_TOKEN" in overlay_script
    assert 'export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"' in overlay_script
    assert 'export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"' in overlay_script
    assert overlay_script.index("AMENT_TRACE_SETUP_FILES") < overlay_script.index('source "${SCRIPT_DIR}/common_env.sh"')
    assert "robot_api_server_node" in overlay_script
    assert "colcon build --packages-select robot_interfaces robot_api_server" in overlay_script
    assert "umask 0002" in overlay_script
    assert "NJRH_RUNTIME_MAP_CONTEXT_FILE" in floor_navigation_script
    assert "write_runtime_map_context \"ready\" \"true\"" in floor_navigation_script
    assert "map->odom and Nav2 global costmap are ready" in floor_navigation_script
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
    assert "POST http://<robot-ip>:8080/api/v1/docking/undock" in app_doc
    assert "automatically performs controlled undocking first" in app_doc
    assert "The App must not use mapping teleop or direct velocity commands for docking" in app_doc
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
    assert "/ranger_mini3/allow_reverse" in app_doc
    assert "mapping_state" in app_doc
    assert "resume_navigation" in app_doc
    assert "start the occupancy localization stack" in app_doc
    assert "treats `map -> odom` as the authoritative localization-ready signal" in app_doc
    assert "run_occupancy_grid_localization.sh" in floor_navigation_script
    assert "run_nav2_navigation.sh" in floor_navigation_script
    assert "/floor_manager/switch_floor" in floor_navigation_script
    assert "global_localization_node.py" in global_localization_script
    assert "GLOBAL_LOCALIZATION_PARAMS_FILE" in global_localization_script
    assert "robot_global_localization/global_localization_node.py" in nav_runtime_helpers
    assert "/global_localization/apply_floor_assets" in floor_navigation_script
    assert "/trigger_grid_search_localization" in floor_navigation_script
    assert "localization_result was not observed directly; checking map->odom bridge state" in floor_navigation_script
    assert "map->odom ready; starting standard Nav2 navigation stack" in floor_navigation_script
    assert "from std_srvs.srv import Empty" in global_localization_node
    assert "grid_search_trigger_service" in global_localization_node
    assert "MultiThreadedExecutor" in global_localization_node
    assert "/trigger_grid_search_localization" in global_localization_config
    assert "<exec_depend>std_srvs</exec_depend>" in global_localization_package
    assert 'wait_for_topic_message "/flatscan"' in floor_navigation_script
    assert 'wait_for_topic_message "/localization_result"' in floor_navigation_script
    assert 'wait_for_tf_transform "map" "odom"' in floor_navigation_script
    assert "NJRH_NAV_MAP_NAME" in floor_asset_helpers
    assert "NJRH_NAV_MAP_ID" in floor_asset_helpers
    assert "asset_report.json" in floor_asset_helpers
    assert "wait_for_ros_service()" in nav_runtime_helpers
    assert "wait_for_tf_transform()" in nav_runtime_helpers
    assert "const bool pre_nav_resume = request->resume_navigation" in floor_manager_code
    assert 'floor assets selected for next navigation: ' in floor_manager_code
    assert floor_manager_code.index('if (!pre_nav_resume) {') < floor_manager_code.index('if (!load_nav_map')
    assert "if (!pre_nav_resume && !load_filter_masks" in floor_manager_code
    assert "if (!pre_nav_resume && clear_costmaps_after_switch_)" in floor_manager_code
    assert floor_navigation_script.index("run_occupancy_grid_localization.sh") < floor_navigation_script.index(
        'wait_for_ros_service "/global_localization/apply_floor_assets"'
    )
    assert floor_navigation_script.index('wait_for_ros_service "/global_localization/apply_floor_assets"') < floor_navigation_script.index(
        'wait_for_occupancy_grid "/map"'
    )
    assert floor_navigation_script.index('wait_for_occupancy_grid "/map"') < floor_navigation_script.index(
        'wait_for_topic_message "/flatscan"'
    )
    assert floor_navigation_script.index('wait_for_topic_message "/flatscan"') < floor_navigation_script.index(
        "/floor_manager/switch_floor"
    )
    assert floor_navigation_script.index("/floor_manager/switch_floor") < floor_navigation_script.index(
        'wait_for_topic_message "/localization_result"'
    )
    assert floor_navigation_script.index('wait_for_topic_message "/localization_result"') < floor_navigation_script.index(
        'wait_for_tf_transform "map" "odom"'
    )
    assert floor_navigation_script.index('wait_for_tf_transform "map" "odom"') < floor_navigation_script.index(
        'bash "${SCRIPT_DIR}/run_nav2_navigation.sh" &'
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
    assert "pre_navigation_undock" in node_cpp
    assert "docking_undock_client_" in node_cpp
    assert "docking_status_is_undocked" in node_cpp
    assert "undock requires docked state or live charging contact" in node_cpp


def test_ranger_mini3_mode_controller_is_cpp_and_rejects_lateral_reverse():
    package_root = ROOT / "src" / "ranger_mini3_mode_controller"
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
    node_cpp = (package_root / "src" / "mode_controller_node.cpp").read_text(encoding="utf-8")
    config = (package_root / "config" / "ranger_mini3_mode_controller.yaml").read_text(encoding="utf-8")
    overlay_config = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "ranger_mini3_mode_controller.yaml"
    ).read_text(encoding="utf-8")
    overlay_runner = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_ranger_mini3_mode_controller.sh"
    ).read_text(encoding="utf-8")
    assert "ament_cmake" in cmake
    assert "rclcpp" in package_xml
    assert "ament_python" not in package_xml
    assert "add_executable(mode_controller_node src/mode_controller_node.cpp)" in cmake
    assert "lateral_policy: reject" in config
    assert "max_lateral_mps: 0.08" in config
    assert "max_crab_yaw_radps: 0.15" in config
    assert "allow_reverse: false" in config
    assert "reverse_enable_topic: /ranger_mini3/allow_reverse" in config
    assert "reverse_enable_timeout_s: 0.75" in config
    assert "spin_steering_threshold_rad: 0.698" in config
    assert "spin_enter_steering_threshold_rad: 0.698" in config
    assert "lateral_policy: reject" in overlay_config
    assert "max_lateral_mps: 0.08" in overlay_config
    assert "max_crab_yaw_radps: 0.15" in overlay_config
    assert "allow_reverse: false" in overlay_config
    assert "reverse_enable_topic: /ranger_mini3/allow_reverse" in overlay_config
    assert "spin_enter_steering_threshold_rad: 0.698" in overlay_config
    assert "Lateral / crab commands are disabled" in node_cpp
    assert "forced_policy_ = \"crab\"" in node_cpp
    assert "msg.linear.y = command.lateral_mps" in node_cpp
    assert "effectiveAllowReverse" in node_cpp
    assert "vx = std::max(0.0, vx)" in node_cpp
    assert "return makeSpin(wz)" in node_cpp
    assert "python3" not in overlay_runner
    assert "set +u" in overlay_runner
    assert 'source "${REPO_ROOT}/install/local_setup.bash"' in overlay_runner
    assert "set -u" in overlay_runner
    assert "colcon build --packages-select ranger_mini3_mode_controller" in overlay_runner


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
    node_script = (ROOT / "src" / "robot_local_perception" / "src" / "local_perception_node.cpp").read_text(encoding="utf-8")
    assert "pcl_conversions" in node_script
    assert "TransformListener" in node_script
    assert "lookupTransform(" in node_script
    assert 'declare_parameter<std::string>("output_frame_id", "base_link")' in node_script
    assert 'declare_parameter<std::string>("input_topic", "/lidar_points")' in node_script
    assert 'declare_parameter<std::string>("clearing_output_topic", "/perception/clearing_points")' in node_script
    assert "clearing.virtual_rays.enabled" in node_script
    assert "buildVirtualClearingCloud" in node_script
    assert "updateClearingBin" in node_script
    assert "outputStampForCostmap" in node_script
    assert 'declare_parameter<std::string>("output_stamp_tf_target_frame", "odom")' in node_script
    assert 'declare_parameter<bool>("restamp_to_latest_tf", true)' in node_script
    assert 'declare_parameter<bool>("require_output_stamp_tf", true)' in node_script
    assert "return std::nullopt" in node_script
    assert "lookupTransform(\n        output_stamp_tf_target_frame_, output_frame_id_, tf2::TimePointZero" in node_script
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()" in node_script
    assert "best_effort" in node_script
    assert "applyVoxelOutlierFilter" in node_script
    assert "\"ELEVATOR_WAIT\"" in node_script
    assert "\"DOORWAY\"" in node_script


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


def test_robot_bringup_wires_repo_owned_localization_and_navigation_launches():
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
    assert "ros2 launch" in run_projected_map
    assert "jt128_slam_toolbox_mapping.launch.py" in run_projected_map
    assert "import os" in slam_launch
    assert "os.environ.get" in slam_launch
    assert "run_robot_description.sh" in run_projected_map
    assert "run_local_state.sh" in run_projected_map
    assert "NJRH_SLAM2D_LOCAL_STATE_MODE:-passthrough" in run_projected_map
    assert 'POINTS_TOPIC="${NJRH_SLAM2D_POINTS_TOPIC:-/cloud_registered_body}"' in run_projected_map
    assert "FASTLIO_CONFIG_FILE" in run_projected_map
    assert "ros2 run fast_lio fastlio_mapping" in run_projected_map
    assert "-r /tf:=/tf_fastlio_internal" in run_projected_map
    assert 'reusing existing FAST-LIO2 deskew source: ${POINTS_TOPIC}' in run_projected_map
    initial_cleanup = run_projected_map.split("require_can_interface_up", 1)[0]
    pre_runtime_cleanup = initial_cleanup.split("stop_fastlio_deskew_sources()", 1)[0]
    assert '"fastlio_mapping"' not in pre_runtime_cleanup
    assert "fastlio_reused_for_slam2d" in run_projected_map
    assert "stop_fastlio_deskew_sources" in run_projected_map
    assert '"run_fastlio_tf.sh"' in run_projected_map
    assert 'wait_for_topic_message "${POINTS_TOPIC}" 20' in run_projected_map
    assert 'env LOCAL_STATE_MODE="${SLAM2D_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"' in run_projected_map
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' in run_projected_map
    assert 'wait_for_tf_edge "base_link" "lidar_level_link" 10' in run_projected_map
    assert 'wait_for_topic_message "/local_state/odometry" 12' in run_projected_map
    assert "slam_toolbox" in slam_launch
    assert 'DeclareLaunchArgument("points_topic", default_value="/cloud_registered_body")' in slam_launch
    assert "f'image: {pgm_path.name}'," in dashboard_patch
    assert "'image': str(pgm_path)" in dashboard_patch
    assert "nav_cloud_preprocessor" in slam_launch
    assert "pointcloud_to_laserscan_node" in slam_launch
    assert "scan_republisher_node" in slam_launch
    assert "scan_republisher_node" in run_projected_map
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
    assert "transform_timeout: 0.35" in slam_cfg
    assert "minimum_travel_heading: 0.02" in slam_cfg
    assert "scan_buffer_size: 30" in slam_cfg
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
    assert "install/robot_localization_bridge/lib/robot_localization_bridge/localization_bridge_node" in overlay_runner
    assert "Python fallback has been removed" in overlay_runner


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


def test_navigation_localization_uses_raw_lidar_without_fastlio():
    occupancy_stack = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization_stack.launch.py").read_text(encoding="utf-8")
    localization_sensing = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_localization_sensing.launch.py").read_text(encoding="utf-8")
    localization_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    assert 'POINTS_TOPIC="${NJRH_LOCALIZATION_POINTS_TOPIC:-/lidar_points}"' in localization_script
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points")' in occupancy_stack
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points")' in localization_sensing
    assert "starting FAST-LIO2 deskew source for occupancy localization" not in localization_script
    assert "FASTLIO_CONFIG_FILE" not in localization_script
    assert "ros2 run fast_lio fastlio_mapping" not in localization_script
    assert "-r /tf:=/tf_fastlio_internal" not in localization_script
    assert "timed out waiting for localization pointcloud" in localization_script


def test_fastlio_uses_canonical_lidar_topics_by_default():
    fastlio_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "fastlio.yaml").read_text(encoding="utf-8")
    fastlio_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_fastlio_tf.sh").read_text(encoding="utf-8")
    localization_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    wrapper_cfg = (ROOT / "src" / "robot_fastlio_mapping" / "config" / "fastlio.yaml").read_text(encoding="utf-8")
    assert "lid_topic: /lidar_points" in fastlio_cfg
    assert "imu_topic: /lidar_imu" in fastlio_cfg
    assert "sensor_frame_id: lidar_link" in fastlio_cfg
    assert "point_filter_num: 4" in fastlio_cfg
    assert "max_iteration: 3" in fastlio_cfg
    assert "path_en: false" in fastlio_cfg
    assert "map_en: false" in fastlio_cfg
    assert "Legacy Fast-LIO-only remap paths are no longer supported" in fastlio_script
    assert "pointcloud_axis_remap --ros-args" not in fastlio_script
    assert "imu_axis_remap --ros-args" not in fastlio_script
    assert '"imu_axis_remap"' not in fastlio_script
    assert '"pointcloud_axis_remap"' not in fastlio_script
    assert '"imu_axis_remap"' not in localization_script
    assert '"pointcloud_axis_remap"' not in localization_script
    assert "ros2 run fast_lio fastlio_mapping" not in localization_script
    assert "upstream_points_topic: /lidar_points" in wrapper_cfg
    assert "upstream_imu_topic: /lidar_imu" in wrapper_cfg
    assert "upstream_sensor_frame: lidar_link" in wrapper_cfg
    assert "upstream_send_odom_base_tf: false" in wrapper_cfg
    assert "fallback_points_topic" not in wrapper_cfg
    assert "fallback_imu_topic" not in wrapper_cfg


def test_jt128_driver_normalizes_vendor_raw_to_canonical_topics():
    driver_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_driver.sh").read_text(encoding="utf-8")
    jt128_cfg = (ROOT / "src" / "robot_hesai_jt128" / "config" / "jt128.yaml").read_text(encoding="utf-8")
    pointcloud_remap_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_canonical_pointcloud_remap.yaml"
    ).read_text(encoding="utf-8")
    pointcloud_remap_cpp = (
        ROOT / "src" / "robot_hesai_jt128" / "src" / "pointcloud_axis_remap_node.cpp"
    ).read_text(encoding="utf-8")
    imu_remap_cfg = (
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_canonical_imu_remap.yaml"
    ).read_text(encoding="utf-8")
    assert 'export LIDAR_FRAME="${LIDAR_FRAME:-lidar_link}"' in driver_script
    assert 'export IMU_FRAME="${IMU_FRAME:-imu_link}"' in driver_script
    assert 'export POINTS_TOPIC="${NJRH_JT128_POINTS_TOPIC:-/lidar_points}"' in driver_script
    assert 'export IMU_TOPIC="${NJRH_JT128_IMU_TOPIC:-/lidar_imu}"' in driver_script
    assert 'export VENDOR_POINTS_TOPIC="${NJRH_JT128_VENDOR_POINTS_TOPIC:-/jt128/vendor/points_raw}"' in driver_script
    assert 'export VENDOR_IMU_TOPIC="${NJRH_JT128_VENDOR_IMU_TOPIC:-/jt128/vendor/imu_raw}"' in driver_script
    assert 'export POINTCLOUD_REMAP_CPP_BIN="${NJRH_POINTCLOUD_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/pointcloud_axis_remap_node}"' in driver_script
    assert 'export IMU_REMAP_CPP_BIN="${NJRH_IMU_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/imu_axis_remap_node}"' in driver_script
    assert "ros_send_point_cloud_topic" in driver_script
    assert "ros_send_imu_topic" in driver_script
    assert "/jt128/vendor/points_raw" in driver_script
    assert "/jt128/vendor/imu_raw" in driver_script
    assert '[[ -x "${POINTCLOUD_REMAP_CPP_BIN}" ]]' in driver_script
    assert '"${POINTCLOUD_REMAP_CPP_BIN}" --ros-args --params-file "${POINTCLOUD_REMAP_CONFIG}" &' in driver_script
    assert '[[ -x "${IMU_REMAP_CPP_BIN}" ]]' in driver_script
    assert '"${IMU_REMAP_CPP_BIN}" --ros-args --params-file "${IMU_REMAP_CONFIG}" &' in driver_script
    assert "canonical_jt128_ingress_running()" in driver_script
    assert "incomplete JT128 ingress detected" in driver_script
    assert "stop_jt128_ingress_processes" in driver_script
    assert 'pointcloud_axis_remap.py' not in driver_script
    assert 'imu_axis_remap.py' not in driver_script
    assert "Python remap fallback has been removed" in driver_script
    assert "points_topic: /lidar_points" in jt128_cfg
    assert "imu_topic: /lidar_imu" in jt128_cfg
    assert "vendor_points_topic: /jt128/vendor/points_raw" in jt128_cfg
    assert "vendor_imu_topic: /jt128/vendor/imu_raw" in jt128_cfg
    assert 'export NJRH_HESAI_UPSTREAM_DRIVER_PROFILE="${NJRH_HESAI_UPSTREAM_DRIVER_PROFILE:-navigation}"' in driver_script
    assert 'UPSTREAM_DRIVER_PROFILE="${NJRH_HESAI_UPSTREAM_DRIVER_PROFILE}"' in driver_script
    assert 'use_timestamp_type: 1' in driver_script
    assert 'DRIVER_PROFILE="${UPSTREAM_DRIVER_PROFILE}" bash "$(require_upstream_script run_driver.sh)" &' in driver_script
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
    assert raw_to_canonical in pointcloud_remap_cfg
    assert raw_to_canonical in imu_remap_cfg
    assert "fast_path_neg_raw_y_neg_raw_x" in pointcloud_remap_cpp


def test_localization_bridge_latches_one_shot_localizer_pose():
    bridge_code = (ROOT / "src" / "robot_localization_bridge" / "src" / "localization_bridge_node.cpp").read_text(encoding="utf-8")
    assert "has_last_pose_stamp_used_" in bridge_code
    assert 'refresh_state("pose")' in bridge_code
    assert 'refresh_state("timer")' in bridge_code
    assert 'bridge waiting for localization_result' in bridge_code
    assert "else if (!has_map_to_odom_)" in bridge_code
    assert "if (!has_map_to_odom_)" in bridge_code
    assert "tf.transform.translation.x = map_to_odom_.x" in bridge_code
    assert "tf.transform.rotation = quaternion_from_yaw(map_to_odom_.yaw)" in bridge_code


def test_localization_sensing_reuses_slam2d_scan_contract():
    occupancy_stack = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "occupancy_localization_stack.launch.py").read_text(encoding="utf-8")
    localization_sensing = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_localization_sensing.launch.py").read_text(encoding="utf-8")
    localization_script = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts" / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    slam_mapping = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "launch" / "jt128_slam_toolbox_mapping.launch.py").read_text(encoding="utf-8")
    slam_scan_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_scan_slam2d.yaml").read_text(encoding="utf-8")
    preprocessor_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_nav_cloud_preprocessor.yaml").read_text(encoding="utf-8")
    flatscan_cfg = (ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "jt128_flatscan.yaml").read_text(encoding="utf-8")
    assert "jt128_nav_sensing.launch.py" not in occupancy_stack
    assert "jt128_localization_sensing.launch.py" in occupancy_stack
    assert 'points_topic = LaunchConfiguration("points_topic")' in occupancy_stack
    assert '"points_topic": points_topic' in occupancy_stack
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points")' in occupancy_stack
    assert 'overlay_root / "config" / "jt128_scan_slam2d.yaml"' in localization_sensing
    assert "jt128_flatscan.yaml" in localization_sensing
    assert "scan_republisher_node" in localization_sensing
    assert "scan_republisher_node" in localization_script
    assert "NJRH_NAV_LOCAL_STATE_MODE:-passthrough" in localization_script
    assert 'POINTS_TOPIC="${NJRH_LOCALIZATION_POINTS_TOPIC:-/lidar_points}"' in localization_script
    assert "FASTLIO_CONFIG_FILE" not in localization_script
    assert "ros2 run fast_lio fastlio_mapping" not in localization_script
    assert "-r /tf:=/tf_fastlio_internal" not in localization_script
    assert 'wait_for_topic_message "${POINTS_TOPIC}" 20' in localization_script
    assert '"points_topic:=${POINTS_TOPIC}"' in localization_script
    assert 'env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"' in localization_script
    assert 'kill_canonical_pattern "robot_localization/ekf_node"' in localization_script
    assert "nav_points_topic" in localization_sensing
    assert 'DeclareLaunchArgument("points_topic", default_value="/lidar_points")' in localization_sensing
    assert '"output_frame_id": "lidar_level_link"' in localization_sensing
    assert "output_frame_id: lidar_level_link" in preprocessor_cfg
    assert '("cloud_in", nav_points_topic)' in localization_sensing
    assert '("scan", "/scan_raw")' in localization_sensing
    assert '("scan", scan_topic)' in localization_sensing
    assert '("flatscan", flatscan_topic)' in localization_sensing
    assert 'overlay_root / "config" / "jt128_scan_slam2d.yaml"' in slam_mapping
    assert "target_frame: lidar_level_link" in slam_scan_cfg
    assert "min_height: -0.75" in slam_scan_cfg
    assert "max_height: 0.35" in slam_scan_cfg
    assert "drop_invalid_ranges: true" in flatscan_cfg

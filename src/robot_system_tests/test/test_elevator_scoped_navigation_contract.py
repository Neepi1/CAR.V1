from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]


def _yaml_block(text: str, key: str) -> str:
    lines = text.splitlines()
    marker = f"    {key}:"
    start = lines.index(marker)
    block = [lines[start]]
    for line in lines[start + 1 :]:
        if line and len(line) - len(line.lstrip()) <= 4:
            break
        block.append(line)
    return "\n".join(block)


def test_only_cabin_panel_lateral_speed_is_reduced():
    nav = (ROOT / "src/robot_nav_config/config/nav2.yaml").read_text(encoding="utf-8")
    direct = _yaml_block(nav, "ElevatorCabinEntryDirectFollowPath")
    panel = _yaml_block(nav, "ElevatorCabinPanelFollowPath")
    assert panel.replace("ElevatorCabinPanelFollowPath", "ElevatorCabinEntryDirectFollowPath").replace(
        "lateral_max_speed_mps: 0.20", "lateral_max_speed_mps: 0.40"
    ) == direct
    assert "lateral_max_speed_mps: 0.20" in panel
    assert "lateral_max_speed_mps: 0.40" in direct
    trees = ROOT / "src/robot_nav_config/behavior_trees"
    direct_tree = (trees / "navigate_elevator_cabin_entry_direct.xml").read_text()
    panel_tree = (trees / "navigate_elevator_cabin_panel.xml").read_text()
    assert panel_tree.replace("navigate_elevator_cabin_panel", "navigate_elevator_cabin_entry_direct").replace(
        "ElevatorCabinPanelFollowPath", "ElevatorCabinEntryDirectFollowPath"
    ) == direct_tree
    runtime = (ROOT / "src/robot_api_server/src/features/elevator/execution/elevator_ros_runtime_port.cpp").read_text(
        encoding="utf-8"
    )
    clients = runtime[runtime.index("for (const auto * controller_id : {") :]
    clients = clients[:clients.index("nav_client_ =")]
    assert '"ElevatorCabinPanelFollowPath"' in clients
    assert "goal.behavior_tree = elevator_cabin_behavior_tree_for_role(" in runtime


def test_elevator_scoped_plugins_are_transaction_selected_only():
    source_nav = (
        ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml"
    ).read_text(encoding="utf-8")
    overlay_nav = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "nav2.yaml"
    ).read_text(encoding="utf-8")
    assert source_nav == overlay_nav

    assert (
        'planner_plugins: ["GridBased", "ElevatorScoped", '
        '"ElevatorHallScoped", "ElevatorReverseEntryStagingScoped", '
        '"ElevatorReverseDockingScoped", "ElevatorCabinEntryDirect"]'
    ) in source_nav
    assert (
        'controller_plugins: ["FollowPath", "FollowPathFallback", '
        '"ElevatorFollowPath", "ElevatorHallFollowPath", '
        '"ElevatorReverseEntryStagingFollowPath", '
        '"ElevatorReverseDockingFollowPath", '
        '"ElevatorCabinEntryDirectFollowPath", "ElevatorCabinPanelFollowPath"]'
    ) in source_nav
    assert 'plugin: "robot_nav_config::ElevatorScopedPlanner"' in source_nav
    assert 'plugin: "robot_nav_config::ElevatorScopedController"' in source_nav
    assert "    ElevatorHallScoped:" in source_nav
    assert "    ElevatorHallFollowPath:" in source_nav
    assert "    ElevatorReverseEntryStagingScoped:" in source_nav
    assert "    ElevatorReverseEntryStagingFollowPath:" in source_nav
    assert "    ElevatorReverseDockingScoped:" in source_nav
    assert "    ElevatorReverseDockingFollowPath:" in source_nav
    assert "    ElevatorCabinEntryDirect:" in source_nav
    assert "      unchecked_direct_path: true" in source_nav
    assert "    ElevatorCabinEntryDirectFollowPath:" in source_nav
    assert "      command_clearance_check_enabled: false" in source_nav
    assert source_nav.count("      prefer_clear_startup_spin: true") == 2
    assert source_nav.count("      reverse_entry_staging_sequence: true") == 2
    assert source_nav.count("      reverse_docking_sequence: true") == 2
    assert "      search_grid_step_m: 0.05" in source_nav
    assert "      clearance_preference_cost_ratio: 1.10" in source_nav
    assert "      maximum_search_time_sec: 1.00" in source_nav
    assert "      route_revision_enabled: true" in source_nav
    assert "      - nav2_recovery_node_bt_node" in source_nav
    assert "      - nav2_wait_action_bt_node" in source_nav
    assert 'FollowPath:\n      plugin: "robot_nav_config::GoalScopedRotationShimController"' in source_nav
    assert "      vy_max: 0.0" in source_nav

    # Both schema-v2 SOURCE_LANDING and schema-v3 REVERSE_ENTRY_STAGING are
    # the post-call move to the source landing/entry-staging pose. They must
    # use the same obstacle-unchecked contract as the later elevator legs,
    # while the pre-call hall approach remains obstacle checked.
    for planner in ("ElevatorScoped", "ElevatorReverseEntryStagingScoped"):
        assert "unchecked_direct_path: true" in _yaml_block(source_nav, planner)
    for controller in (
        "ElevatorFollowPath",
        "ElevatorReverseEntryStagingFollowPath",
    ):
        block = _yaml_block(source_nav, controller)
        assert "command_clearance_check_enabled: false" in block
        assert "route_revision_enabled: false" in block
        assert "localization_replan_enabled: false" in block
    assert "unchecked_direct_path: true" not in _yaml_block(
        source_nav, "ElevatorHallScoped"
    )
    assert "command_clearance_check_enabled: false" not in _yaml_block(
        source_nav, "ElevatorHallFollowPath"
    )

    behavior_tree = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "behavior_trees"
        / "navigate_elevator_scoped_motion.xml"
    ).read_text(encoding="utf-8")
    assert 'planner_id="ElevatorScoped"' in behavior_tree
    assert 'controller_id="ElevatorFollowPath"' in behavior_tree

    scoped_root = ET.fromstring(behavior_tree)
    scoped_sequence = scoped_root.find("./BehaviorTree/Sequence")
    assert scoped_sequence is not None
    assert [child.tag for child in scoped_sequence] == ["RecoveryNode", "FollowPath"]
    scoped_retry = scoped_sequence[0]
    assert scoped_retry.attrib["number_of_retries"] == "3"
    assert [child.tag for child in scoped_retry] == ["ComputePathToPose", "Wait"]
    assert scoped_retry[0].attrib["planner_id"] == "ElevatorScoped"
    assert scoped_retry[1].attrib["wait_duration"] == "0.5"
    assert scoped_sequence[1].attrib["controller_id"] == "ElevatorFollowPath"

    hall_behavior_tree = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "behavior_trees"
        / "navigate_elevator_hall_call_scoped_motion.xml"
    ).read_text(encoding="utf-8")
    assert 'planner_id="ElevatorHallScoped"' in hall_behavior_tree
    assert 'controller_id="ElevatorHallFollowPath"' in hall_behavior_tree

    hall_root = ET.fromstring(hall_behavior_tree)
    hall_sequence = hall_root.find("./BehaviorTree/Sequence")
    assert hall_sequence is not None
    assert [child.tag for child in hall_sequence] == ["RecoveryNode", "FollowPath"]
    hall_retry = hall_sequence[0]
    assert hall_retry.attrib["number_of_retries"] == "3"
    assert [child.tag for child in hall_retry] == ["ComputePathToPose", "Wait"]
    assert hall_retry[0].attrib["planner_id"] == "ElevatorHallScoped"
    assert hall_retry[1].attrib["wait_duration"] == "0.5"
    assert hall_sequence[1].attrib["controller_id"] == "ElevatorHallFollowPath"

    reverse_behavior_tree = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "behavior_trees"
        / "navigate_elevator_reverse_docking.xml"
    ).read_text(encoding="utf-8")
    reverse_root = ET.fromstring(reverse_behavior_tree)
    reverse_sequence = reverse_root.find("./BehaviorTree/Sequence")
    assert reverse_sequence is not None
    assert [child.tag for child in reverse_sequence] == [
        "RecoveryNode",
        "FollowPath",
    ]
    reverse_retry = reverse_sequence[0]
    assert reverse_retry.attrib["number_of_retries"] == "60"
    assert [child.tag for child in reverse_retry] == ["ComputePathToPose", "Wait"]
    assert reverse_retry[0].attrib["planner_id"] == "ElevatorReverseDockingScoped"
    assert reverse_retry[1].attrib["wait_duration"] == "0.5"
    assert (
        reverse_sequence[1].attrib["controller_id"]
        == "ElevatorReverseDockingFollowPath"
    )

    staging_behavior_tree = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "behavior_trees"
        / "navigate_elevator_reverse_entry_staging.xml"
    ).read_text(encoding="utf-8")
    staging_root = ET.fromstring(staging_behavior_tree)
    staging_sequence = staging_root.find("./BehaviorTree/Sequence")
    assert staging_sequence is not None
    assert [child.tag for child in staging_sequence] == [
        "RecoveryNode",
        "FollowPath",
    ]
    staging_retry = staging_sequence[0]
    assert staging_retry.attrib["number_of_retries"] == "60"
    assert [child.tag for child in staging_retry] == ["ComputePathToPose", "Wait"]
    assert (
        staging_retry[0].attrib["planner_id"]
        == "ElevatorReverseEntryStagingScoped"
    )
    assert staging_retry[1].attrib["wait_duration"] == "0.5"
    assert (
        staging_sequence[1].attrib["controller_id"]
        == "ElevatorReverseEntryStagingFollowPath"
    )

    ranger_profile = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "config"
        / "planner_profiles"
        / "ranger_mini3_lattice.yaml"
    ).read_text(encoding="utf-8")
    assert (
        'planner_plugins: ["GridBased", "ElevatorScoped", '
        '"ElevatorHallScoped", "ElevatorReverseEntryStagingScoped", '
        '"ElevatorReverseDockingScoped", "ElevatorCabinEntryDirect"]'
    ) in ranger_profile
    assert 'plugin: "robot_nav_config::RangerMini3LatticePlanner"' in ranger_profile
    assert 'plugin: "robot_nav_config::ElevatorScopedPlanner"' in ranger_profile
    assert "    ElevatorHallScoped:" in ranger_profile
    assert "      prefer_clear_startup_spin: true" in ranger_profile
    assert "    ElevatorReverseEntryStagingScoped:" in ranger_profile
    assert "      reverse_entry_staging_sequence: true" in ranger_profile
    assert "    ElevatorReverseDockingScoped:" in ranger_profile
    assert "      reverse_docking_sequence: true" in ranger_profile
    assert "    ElevatorCabinEntryDirect:" in ranger_profile
    assert "      unchecked_direct_path: true" in ranger_profile


def test_runtime_adapter_owns_the_scoped_behavior_tree_selection():
    runtime = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features" / "elevator" / "execution"
        / "elevator_ros_runtime_port.cpp"
    ).read_text(encoding="utf-8")
    assert "resolve_elevator_navigation_request" in runtime
    assert "select_elevator_navigation_profile" in runtime
    assert "options_.current_map_pose_probe" in runtime
    assert "nearby_hall_call_scoped_distance_m" in runtime
    assert "ElevatorNavigationProfile::kElevatorScoped" in runtime
    assert "ElevatorNavigationProfile::kElevatorReverseEntryStaging" in runtime
    assert "ElevatorNavigationProfile::kElevatorReverseDocking" in runtime
    assert "ElevatorNavigationProfile::kElevatorCabinDirect" in runtime
    assert "options_.elevator_hall_call_scoped_behavior_tree" in runtime
    assert "options_.elevator_scoped_behavior_tree" in runtime
    assert "options_.elevator_reverse_entry_staging_behavior_tree" in runtime
    assert "options_.elevator_reverse_docking_behavior_tree" in runtime
    assert "options_.elevator_cabin_entry_direct_behavior_tree" in runtime

    for config in (
        ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml",
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "robot_api_server.yaml",
    ):
        text = config.read_text(encoding="utf-8")
        assert (
            "elevator_scoped_behavior_tree: "
            '"/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/'
            'robot_nav_config/behavior_trees/navigate_elevator_scoped_motion.xml"'
        ) in text
        assert (
            "elevator_hall_call_scoped_behavior_tree: "
            '"/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/'
            "robot_nav_config/behavior_trees/"
            'navigate_elevator_hall_call_scoped_motion.xml"'
        ) in text
        assert (
            "elevator_reverse_entry_staging_behavior_tree: "
            '"/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/'
            "robot_nav_config/behavior_trees/"
            'navigate_elevator_reverse_entry_staging.xml"'
        ) in text
        assert (
            "elevator_reverse_docking_behavior_tree: "
            '"/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/'
            "robot_nav_config/behavior_trees/"
            'navigate_elevator_reverse_docking.xml"'
        ) in text

    module = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features" / "elevator" / "elevator_module.cpp"
    ).read_text(encoding="utf-8")
    feature = (
        ROOT / "src" / "robot_api_server" / "src" / "features" / "elevator"
        / "elevator_feature_module.cpp"
    ).read_text(encoding="utf-8")
    assert "options.current_map_pose_probe = ports_.current_map_pose_probe" in module
    assert "pose.age_sec > dependencies_.robot_pose_freshness_sec" in feature
    assert "return ElevatorMapPose{" in feature
    assert "pose.x, pose.y, pose.yaw, pose.stamp_sec, pose.age_sec" in " ".join(feature.split())


def test_scoped_limits_preserve_the_elevator_motion_envelope_only():
    nav = (
        ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml"
    ).read_text(encoding="utf-8")
    scoped = nav[nav.index("    ElevatorFollowPath:") : nav.index("    FollowPath:")]
    assert "      max_distance_m: 2.5" in scoped
    assert "      goal_xy_tolerance_m: 0.06" in scoped
    assert "      goal_yaw_tolerance_rad: 0.05" in scoped
    assert scoped.count("      yaw_max_speed_radps: 0.50") == 6
    assert scoped.count("      lateral_max_speed_mps: 0.40") == 5
    assert scoped.count("      lateral_max_speed_mps: 0.20") == 1
    assert scoped.count("      forward_max_speed_mps: 0.40") == 6
    assert scoped.count("      reverse_max_speed_mps: 0.40") == 6
    assert "      blocked_timeout_sec: 3.0" in scoped
    assert "      route_revision_enabled: true" in scoped
    assert "      route_revision_interval_sec: 0.5" in scoped
    assert "      route_revision_maximum_expansions: 60000" in scoped
    assert "      route_revision_time_limit_sec: 1.00" in scoped
    assert "local_rejoin_" not in scoped
    assert "      localization_replan_enabled: true" in scoped
    assert "      localization_replan_translation_trigger_m: 0.08" in scoped
    assert "      localization_replan_yaw_trigger_rad: 0.08" in scoped
    assert "      localization_replan_stable_duration_sec: 0.60" in scoped
    assert (
        "      lateral_permit_topic: "
        "/ranger_mini3/nav_terminal_lateral_enable"
    ) in scoped
    assert (
        "      reverse_permit_topic: "
        "/ranger_mini3/nav_terminal_reverse_enable"
    ) in scoped

    # The smoother must admit the requested elevator velocities, but ordinary
    # navigation keeps its own controller and terminal-handoff caps.
    assert "    max_velocity: [1.20, 0.40, 0.70]" in nav
    assert "    min_velocity: [-0.40, -0.40, -0.70]" in nav
    ordinary = nav[nav.index("    FollowPath:") : nav.index("    FollowPathFallback:")]
    assert "      terminal_handoff_yaw_max_speed_radps: 0.30" in ordinary
    assert "      terminal_handoff_lateral_max_speed_mps: 0.05" in ordinary
    assert "      terminal_handoff_forward_max_speed_mps: 0.10" in ordinary
    assert "      terminal_handoff_reverse_max_speed_mps: 0.08" in ordinary

    for safety_config in (
        ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml",
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "robot_safety.yaml",
    ):
        safety = safety_config.read_text(encoding="utf-8")
        assert "normal_navigation_reverse_max_mps: 0.08" in safety
        assert "normal_navigation_lateral_max_mps: 0.05" in safety
        assert "elevator_navigation_reverse_max_mps: 0.40" in safety
        assert "elevator_navigation_lateral_max_mps: 0.40" in safety


def test_scoped_progress_counts_fine_motion_without_weakening_ordinary_navigation():
    source_nav = (
        ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml"
    ).read_text(encoding="utf-8")
    overlay_nav = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "nav2.yaml"
    ).read_text(encoding="utf-8")
    assert source_nav == overlay_nav

    progress = source_nav[
        source_nav.index("    progress_checker:") :
        source_nav.index("    goal_checker:")
    ]
    assert 'plugin: "robot_nav_config::ElevatorAwareProgressChecker"' in progress
    assert "      required_movement_radius: 0.03" in progress
    assert "      required_movement_angle: 0.05" in progress
    assert "      movement_time_allowance: 12.0" in progress
    assert "      elevator_required_movement_radius: 0.015" in progress
    assert "      elevator_required_movement_angle: 0.015" in progress
    assert "      elevator_movement_time_allowance: 20.0" in progress
    assert "      elevator_progress_state_timeout: 0.50" in progress
    assert (
        "      elevator_progress_state_topic: "
        "/ranger_mini3/nav_elevator_scoped_progress_state"
    ) in progress
    assert source_nav.count(
        "      progress_state_topic: "
        "/ranger_mini3/nav_elevator_scoped_progress_state"
    ) == 6


def test_scoped_clearance_uses_filled_footprint_without_mutating_inflation():
    source_nav = (
        ROOT / "src" / "robot_nav_config" / "config" / "nav2.yaml"
    ).read_text(encoding="utf-8")
    overlay_nav = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "nav2.yaml"
    ).read_text(encoding="utf-8")
    ranger_profile = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "config"
        / "planner_profiles"
        / "ranger_mini3_lattice.yaml"
    ).read_text(encoding="utf-8")
    planner = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_planner.cpp"
    ).read_text(encoding="utf-8")
    controller = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_controller.cpp"
    ).read_text(encoding="utf-8")
    replan_worker = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_replan_worker.cpp"
    ).read_text(encoding="utf-8")
    search = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_search.cpp"
    ).read_text(encoding="utf-8")

    assert source_nav == overlay_nav
    assert "maximum_center_cost" not in source_nav
    assert "maximum_center_cost" not in ranger_profile
    # Existing global/local/MPPI values plus matched local post-keepout inflation.
    assert source_nav.count("inflation_radius: 0.60") == 4
    assert "inflation_radius: 0.35" not in source_nav
    assert "search_elevator_scoped_path(" in planner
    assert "evaluate_elevator_scoped_path_clearance(" in controller
    assert "schedule_route_revision(" in controller
    assert "search_elevator_scoped_path(" not in controller
    assert "search_elevator_scoped_path(" in replan_worker
    assert "search_elevator_scoped_local_repair(" not in replan_worker
    assert "evaluate_elevator_scoped_clearance(" in search
    assert "evaluate_elevator_scoped_path_clearance(" in search


def test_scoped_controller_revises_the_full_route_from_changed_live_evidence():
    controller = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_controller.cpp"
    ).read_text(encoding="utf-8")
    worker = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_replan_worker.cpp"
    ).read_text(encoding="utf-8")
    blockage = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "include"
        / "robot_nav_config"
        / "elevator_scoped_blockage.hpp"
    ).read_text(encoding="utf-8")
    replan_progress = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "include"
        / "robot_nav_config"
        / "elevator_scoped_replan_progress.hpp"
    ).read_text(encoding="utf-8")
    assert "search_elevator_scoped_path(" in worker
    assert "remaining_path" not in worker
    assert "remaining in WAIT_CLEAR" in controller
    assert "reference_path_" not in controller
    assert "remaining_path" not in controller
    assert "replan_progress_.should_attempt_replan(request_signature)" in controller
    assert "replan_progress_.observe_replan_attempt(request_signature)" in controller
    assert "replan_progress_.try_commit_route_revision(" in controller
    assert "reference_path_start_index_" not in replan_progress
    assert "pending_repair_" not in replan_progress
    assert "attempted_signatures_" in replan_progress
    assert "local_repair_committed_" not in blockage
    assert "one-repair-per-plan" not in controller
    assert "elevator scoped local footprint remained blocked" not in controller


def test_scoped_controller_executes_the_planned_axis_before_final_pose_correction():
    route = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "include"
        / "robot_nav_config"
        / "elevator_scoped_route.hpp"
    ).read_text(encoding="utf-8")
    controller = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_controller.cpp"
    ).read_text(encoding="utf-8")

    assert "ElevatorScopedRoutePhase::kPose" in route
    assert "make_elevator_scoped_segment_error(" in route
    assert "input.error = make_elevator_scoped_segment_error(" in controller


def test_scoped_tracks_continuously_across_material_localization_corrections():
    controller = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_controller.cpp"
    ).read_text(encoding="utf-8")
    assert "canonical_goal_in_control_frame()" in controller
    assert "localization_replan_gate_.consume_correction_event()" in controller
    assert "target is being transformed continuously" in controller
    assert "ElevatorScopedReplanReason::kLocalizationCorrection" not in controller
    assert "localization_replan_gate_.mark_replan_submitted()" not in controller
    assert "localization_replan_gate_.mark_replan_accepted" not in controller
    assert "material map->odom correction invalidates" not in controller


def test_scoped_same_goal_refresh_is_non_disruptive():
    controller = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_controller.cpp"
    ).read_text(encoding="utf-8")
    update_policy = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "include"
        / "robot_nav_config"
        / "elevator_scoped_plan_update.hpp"
    ).read_text(encoding="utf-8")

    assert "decide_elevator_scoped_plan_update(" in controller
    assert "kHotSwapPreserveMotion" in controller
    assert "active motion preserved" in controller
    assert "kResetForNewGoal" in update_policy
    assert "kKeepActivePlan" in update_policy
    assert "active.direction == replacement.direction" in update_policy


def test_scoped_final_pose_uses_the_canonical_map_goal_after_route_revision():
    controller = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "src"
        / "elevator_scoped_controller.cpp"
    ).read_text(encoding="utf-8")

    assert "canonical_goal = final_goal_" in controller
    assert "ElevatorScopedRoutePhase::kPose" in controller
    assert "goal = canonical_goal" in controller
    assert "replanned_path.header.frame_id = route_frame" in controller
    assert "replanned_path.header.frame_id = completed->costmap_frame" not in controller


def test_all_cabin_transit_intents_use_one_unchecked_direct_nav2_chain():
    api_runtime = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features" / "elevator" / "execution"
        / "elevator_ros_runtime_port.cpp"
    ).read_text(encoding="utf-8")
    safety_runtime = (
        ROOT / "src" / "robot_safety" / "src" / "robot_safety_node.cpp"
    ).read_text(encoding="utf-8")
    policy = (
        ROOT
        / "src"
        / "robot_api_server"
        / "src"
        / "features" / "elevator" / "execution"
        / "elevator_runtime_policy.cpp"
    ).read_text(encoding="utf-8")
    direct_bt = (
        ROOT
        / "src"
        / "robot_nav_config"
        / "behavior_trees"
        / "navigate_elevator_cabin_entry_direct.xml"
    ).read_text(encoding="utf-8")

    for intent in (
        "kEnterCabin",
        "kReverseEnterCabin",
        "kCabinPanelApproach",
        "kReturnCabinCenter",
        "kTargetLanding",
    ):
        assert f"Intent::{intent}" in policy
    assert policy.count("profile = ElevatorNavigationProfile::kElevatorCabinDirect;") == 6
    assert (
        "navigation_profile == ElevatorNavigationProfile::kElevatorCabinDirect"
        in api_runtime
    )
    assert 'planner_id="ElevatorCabinEntryDirect"' in direct_bt
    assert 'controller_id="ElevatorCabinEntryDirectFollowPath"' in direct_bt

    assert "elevator_navigation_bypasses_collision_monitor" in api_runtime
    assert "publish_elevator_entry_collision_bypass_permit" in api_runtime
    assert "runtime_effect.effect.transaction_id" in api_runtime
    assert "clear_bypass_permit();" in api_runtime
    assert "elevator_entry_collision_bypass_authorized" in safety_runtime
    assert '"elevator_entry_cmd_vel_in_topic", "/cmd_vel_nav"' in safety_runtime
    assert "on_elevator_entry_cmd" in safety_runtime
    assert "if (elevator_entry_collision_bypass_authorized_now())" in safety_runtime

    for config in (
        ROOT / "src" / "robot_api_server" / "config" / "robot_api_server.yaml",
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "robot_api_server.yaml",
    ):
        text = config.read_text(encoding="utf-8")
        assert (
            "elevator_entry_collision_bypass_permit_topic: "
            '"/ranger_mini3/elevator_entry_collision_bypass"'
        ) in text
        assert "elevator_entry_collision_bypass_refresh_sec: 0.20" in text

    for config in (
        ROOT / "src" / "robot_safety" / "config" / "robot_safety.yaml",
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "config"
        / "robot_safety.yaml",
    ):
        text = config.read_text(encoding="utf-8")
        assert "elevator_entry_cmd_vel_in_topic: /cmd_vel_nav" in text
        assert (
            "elevator_entry_collision_bypass_permit_topic: "
            "/ranger_mini3/elevator_entry_collision_bypass"
        ) in text
        assert "elevator_entry_collision_bypass_permit_timeout_sec: 0.75" in text
        assert "elevator_entry_cmd_timeout_sec: 0.25" in text

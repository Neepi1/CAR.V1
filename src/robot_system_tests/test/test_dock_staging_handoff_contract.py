#!/usr/bin/env python3

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_predock_requires_the_normal_xy_and_yaw_goal_checker():
    config_paths = (
        ROOT / "src/robot_nav_config/config/nav2.yaml",
        ROOT / "scripts/jetson/runtime_overlay/config/nav2.yaml",
    )
    for config_path in config_paths:
        config = config_path.read_text(encoding="utf-8")
        assert 'goal_checker_plugins: ["goal_checker", "dock_staging_goal_checker"]' in config
        ordinary_checker = config[config.index("    goal_checker:\n"):]
        ordinary_checker = ordinary_checker[:ordinary_checker.index("    dock_staging_goal_checker:\n")]
        assert 'plugin: "nav2_controller::SimpleGoalChecker"' in ordinary_checker
        assert "stateful: false" in ordinary_checker
        assert "xy_goal_tolerance: 0.06" in ordinary_checker
        assert "yaw_goal_tolerance: 0.05" in ordinary_checker

    bt_root = ROOT / "src/robot_nav_config/behavior_trees"
    predock = (bt_root / "navigate_to_predock.xml").read_text(encoding="utf-8")
    assert 'goal_checker_id="goal_checker"' in predock
    assert 'goal_checker_id="dock_staging_goal_checker"' not in predock

    for name in (
        "navigate_to_pose.xml",
        "navigate_through_poses.xml",
        "navigate_to_pose_ranger_lattice.xml",
        "navigate_elevator_scoped_motion.xml",
        "navigate_elevator_hall_call_scoped_motion.xml",
        "navigate_elevator_reverse_entry_staging.xml",
        "navigate_elevator_reverse_docking.xml",
        "navigate_elevator_cabin_entry_direct.xml",
    ):
        text = (bt_root / name).read_text(encoding="utf-8")
        assert 'goal_checker_id="goal_checker"' in text, name
        assert 'goal_checker_id="dock_staging_goal_checker"' not in text, name


def test_predock_nav2_terminal_is_proven_stopped_before_near_field_handoff():
    executor = (
        ROOT
        / "src/robot_api_server/src/features/docking/lifecycle/docking_job_executor.cpp"
    ).read_text(encoding="utf-8")
    stop_phase = executor.index('"PREDOCK_NAV2_STOP_VERIFY"')
    pose_verify = executor.index('"PREDOCK_POSE_VERIFY"')
    fine_handoff = executor.index("start_fine_docking_handoff")

    assert stop_phase < pose_verify < fine_handoff
    stop_block = executor[stop_phase:pose_verify]
    assert "reset_terminal_actual_stop_stability()" in stop_block
    assert "wait_for_terminal_actual_stop(" in stop_block
    assert "DOCK_FAILED_PREDOCK_NAV_STOP_UNPROVEN" in stop_block

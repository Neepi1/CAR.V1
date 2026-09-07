from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_terminal_handoff_stays_inside_follow_path_task():
    controller = read(
        "src/robot_nav_config/src/goal_scoped_rotation_shim_controller.cpp"
    )
    behavior_tree = read(
        "src/robot_nav_config/behavior_trees/navigate_to_pose_ranger_lattice.xml"
    )

    assert "should_start_terminal_handoff" in controller
    assert "terminal_handoff_command(pose, velocity, *pose_error)" in controller
    assert "Ranger terminal handoff started inside FollowPath" in controller
    assert "terminal_lateral_permit_pub_->on_activate()" in controller
    assert "terminal_reverse_permit_pub_->on_activate()" in controller
    assert "terminal_lateral_permit_pub_->on_deactivate()" in controller
    assert "terminal_reverse_permit_pub_->on_deactivate()" in controller
    assert '<FollowPath path="{path}" controller_id="FollowPath"/>' in behavior_tree
    assert "Fallback" not in behavior_tree.split("<FollowPath", 1)[1]

    api_config = read("scripts/jetson/runtime_overlay/config/robot_api_server.yaml")
    assert "navigation_near_goal_stalled_handoff_enabled: false" in api_config


def test_observed_hairpin_is_covered_without_widening_goal_acceptance():
    nav2 = read("scripts/jetson/runtime_overlay/config/nav2.yaml")
    unit_test = read(
        "src/robot_nav_config/test/test_terminal_pose_handoff.cpp"
    )

    assert "path.chord_m = 0.4031" in unit_test
    assert "path.length_m = 1.3703" in unit_test
    assert "path.max_cross_track_m = 0.2722" in unit_test
    assert "terminal_handoff_max_distance_m: 0.40" in nav2
    assert "terminal_handoff_path_length_ratio: 1.80" in nav2
    assert "terminal_handoff_path_cross_track_m: 0.15" in nav2
    assert "xy_goal_tolerance: 0.06" in nav2
    assert "yaw_goal_tolerance: 0.05" in nav2


def test_live_pure_lateral_residual_does_not_require_a_hairpin_plan():
    trigger = read(
        "src/robot_nav_config/include/robot_nav_config/terminal_pose_handoff.hpp"
    )
    unit_test = read("src/robot_nav_config/test/test_terminal_pose_handoff.cpp")

    assert "LivePureLateralResidualTriggersWithoutHairpin" in unit_test
    assert "error.forward_m = -0.0253" in unit_test
    assert "error.lateral_m = 0.1329" in unit_test
    assert "lateral_residual_is_kinematically_unsuitable" in trigger
    assert (
        "lateral_residual_is_kinematically_unsuitable || "
        "path_is_geometrically_unsuitable"
    ) in trigger


def test_mppi_remains_ackermann_and_terminal_lateral_is_permit_gated():
    nav2 = read("scripts/jetson/runtime_overlay/config/nav2.yaml")
    safety_config = read("scripts/jetson/runtime_overlay/config/robot_safety.yaml")
    safety = read("src/robot_safety/src/robot_safety_node.cpp")

    assert 'motion_model: "Ackermann"' in nav2
    assert "vy_max: 0.0" in nav2
    assert "max_velocity: [1.20, 0.40, 0.70]" in nav2
    assert "min_velocity: [-0.40, -0.40, -0.70]" in nav2
    assert (
        "nav_terminal_lateral_enable_topic: "
        "/ranger_mini3/nav_terminal_lateral_enable"
    ) in safety_config
    assert "reverse_permit_fresh(nav_terminal_lateral_permit_)" in safety
    assert "normal_navigation_lateral_max_mps: 0.05" in safety_config
    assert "elevator_navigation_lateral_max_mps: 0.40" in safety_config


def test_terminal_axis_commands_are_mutually_exclusive():
    controller = read(
        "src/robot_nav_config/include/robot_nav_config/terminal_pose_handoff.hpp"
    )
    yaw = controller[
        controller.index("if (phase_ == TerminalControlPhase::kYaw)") :
        controller.index("if (phase_ == TerminalControlPhase::kLateral)")
    ]
    lateral = controller[
        controller.index("if (phase_ == TerminalControlPhase::kLateral)") :
        controller.index("if (phase_ == TerminalControlPhase::kForward)")
    ]
    forward_start = controller.index("if (phase_ == TerminalControlPhase::kForward)")
    forward = controller[
        forward_start : controller.index("\n    }\n\n    return make_output();", forward_start)
    ]

    assert "command.angular_z" in yaw
    assert "command.linear_x" not in yaw
    assert "command.linear_y" not in yaw
    assert "command.linear_y" in lateral
    assert "command.linear_x" not in lateral
    assert "command.angular_z" not in lateral
    assert "command.linear_x" in forward
    assert "command.linear_y" not in forward
    assert "command.angular_z" not in forward


def test_terminal_critic_handoff_keeps_path_alignment_until_35cm():
    nav2 = read("scripts/jetson/runtime_overlay/config/nav2.yaml")

    assert "GoalAngleCritic:" in nav2
    assert "PathAlignCritic:" in nav2
    assert "PathFollowCritic:" in nav2
    assert "PathAngleCritic:" in nav2
    assert nav2.count("threshold_to_consider: 0.35") >= 3
    goal_angle = nav2[
        nav2.index("GoalAngleCritic:") : nav2.index("ObstaclesCritic:")
    ]
    assert "threshold_to_consider: 0.45" in goal_angle


def test_terminal_handoff_keeps_six_centimetre_minimum_lateral_trigger():
    nav2 = read("scripts/jetson/runtime_overlay/config/nav2.yaml")
    ordinary_start = nav2.index("    FollowPath:\n")
    fallback_start = nav2.index("    FollowPathFallback:\n", ordinary_start)
    ordinary = nav2[ordinary_start:fallback_start]

    assert "terminal_handoff_minimum_abs_lateral_m: 0.06" in ordinary

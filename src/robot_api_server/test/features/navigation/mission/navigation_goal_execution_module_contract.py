#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "navigation"
    / "mission"
    / "navigation_goal_execution_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "navigation"
    / "mission"
    / "navigation_goal_execution_module.cpp"
)
FEATURE_SOURCE = (
    PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_feature_module.cpp"
)


def main() -> None:
    assert HEADER.exists(), "NavigationGoalExecutionModule public seam is missing"
    assert SOURCE.exists(), "NavigationGoalExecutionModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    feature = FEATURE_SOURCE.read_text(encoding="utf-8")

    assert "class NavigationGoalExecutionModule" in header
    assert "public NavigationGoalExecutionPort" in header
    assert "public NavigationBridgeWaitRuntimePort" in header
    assert "struct NavigationGoalExecutionConfig" in header
    assert "struct NavigationGoalExecutionPorts" in header
    assert "try_acquire_predock_motion_owner" in header
    assert "ordinary_final_yaw_align_active" in header

    for behavior in (
        "near-goal Nav2 handoff watch",
        "post_nav2_final_verify_retry",
        "global correction paused for final_yaw_align",
        "PREDOCK_YAW_ALIGN owns /cmd_vel_docking",
        "failed_send_nav2_goal",
        "AMCL no-motion update before final verify skipped",
        "REPOSITION_AFTER_YAW_DRIFT",
    ):
        assert behavior in source, behavior

    assert 'features/navigation/mission/navigation_goal_execution_module.hpp' in feature
    assert "std::unique_ptr<NavigationGoalExecutionModule> goal_execution_;" in feature
    assert "*goal_execution_" in feature
    assert "std::unique_ptr<NavigationGoalExecutionModule>" not in node
    assert "public NavigationGoalExecutionPort" not in node
    assert "public NavigationBridgeWaitRuntimePort" not in node

    for removed_owner in (
        "  bool maybe_navigation_near_goal_stalled_handoff(",
        "  NavigationRepositionResult run_post_nav2_final_verify_retry(",
        "  FinalYawAlignResult run_final_yaw_align(",
        "  NavigationRepositionResult run_reposition_after_yaw_drift(",
        "  bool send_initial_navigation_goal_to_nav2(",
        "  NavigationBridgeWaitRuntimePort::TimePoint bridge_wait_now(",
        "yaw_align_owner_mutex_",
        "ordinary_final_yaw_align_active_",
        "predock_yaw_align_active_",
    ):
        assert removed_owner not in node, removed_owner


if __name__ == "__main__":
    main()

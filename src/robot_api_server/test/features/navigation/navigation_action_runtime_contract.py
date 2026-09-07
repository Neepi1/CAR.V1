#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
RUNTIME_HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "navigation"
    / "runtime"
    / "navigation_action_runtime.hpp"
)
RUNTIME_SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "navigation"
    / "runtime"
    / "navigation_action_runtime.cpp"
)
NAVIGATION_MODULE_SOURCE = (
    PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_module.cpp"
)
NAVIGATION_FEATURE_SOURCE = (
    PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_feature_module.cpp"
)


def main() -> None:
    root = ROOT_NODE.read_text(encoding="utf-8")
    header = RUNTIME_HEADER.read_text(encoding="utf-8")
    source = RUNTIME_SOURCE.read_text(encoding="utf-8")
    navigation_module = NAVIGATION_MODULE_SOURCE.read_text(encoding="utf-8")
    navigation_feature = NAVIGATION_FEATURE_SOURCE.read_text(encoding="utf-8")

    assert "class NavigationActionRuntime" in header
    for seam in (
        "track_goal(",
        "status_snapshot()",
        "mark_terminal_unknown(",
        "mark_terminal_proven(",
        "cancel_goal_and_prove_terminal(",
        "cancel_all_goals_with_evidence(",
    ):
        assert seam in header, f"navigation action runtime seam missing: {seam}"

    for ownership in (
        "rclcpp_action::create_client<NavigateToPose>",
        "create_subscription<action_msgs::msg::GoalStatusArray>",
        "active_goal_handle_",
        "terminal_unknown_",
        "async_cancel_all_goals",
    ):
        assert ownership in source, f"navigation action runtime does not own: {ownership}"

    for forbidden in (
        "navigate_to_pose_client_",
        "navigate_to_pose_status_sub_",
        "active_nav_goal_handle_",
        "active_nav_goal_terminal_unknown_",
        "navigate_to_pose_status_mutex_",
    ):
        assert forbidden not in root, f"composition root still owns action state: {forbidden}"

    assert "NavigationActionRuntime action_runtime_;" in navigation_module
    assert "module_->action_runtime()" in navigation_feature
    assert "NavigationActionRuntime * navigation_action_runtime_{nullptr};" not in root


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
AGGREGATE = PACKAGE_ROOT / "src" / "features" / "docking" / "docking_feature_module.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "docking"
    / "lifecycle"
    / "pre_navigation_undock_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "lifecycle"
    / "pre_navigation_undock_module.cpp"
)


def main() -> None:
    assert HEADER.exists(), "PreNavigationUndockModule public seam is missing"
    assert SOURCE.exists(), "PreNavigationUndockModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    aggregate = AGGREGATE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "class PreNavigationUndockModule" in header
    assert "struct PreNavigationUndockRequest" in header
    assert "bool run_if_needed(" in header
    assert "bool start(" in header
    assert "bool wait_for_completion(" in header

    for responsibility in (
        "auto_undock_before_navigation",
        "undocking already active",
        "prepare_controlled_undock",
        "pre_navigation_undock_start",
        "service_success_without_undocking_status_observed_yet",
        "pending_goal_held_for_post_undock_settle",
        "pending_goal_released_after_post_undock_settle",
        "timed out waiting for undock before navigation",
    ):
        assert responsibility in source, responsibility

    assert 'features/docking/lifecycle/pre_navigation_undock_module.hpp' in aggregate
    assert "std::unique_ptr<PreNavigationUndockModule> pre_navigation_undock_;" in aggregate
    assert "PreNavigationUndockModule>" not in node

    for removed_owner in (
        "  bool start_pre_navigation_undock(",
        "  bool wait_for_pre_navigation_undock(",
        "navigation_auto_undock_timeout_sec_",
    ):
        assert removed_owner not in node, removed_owner


if __name__ == "__main__":
    main()

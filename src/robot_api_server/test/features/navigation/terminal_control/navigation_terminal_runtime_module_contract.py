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
    / "terminal_control"
    / "navigation_terminal_runtime_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "navigation"
    / "terminal_control"
    / "navigation_terminal_runtime_module.cpp"
)
FEATURE_SOURCE = (
    PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_feature_module.cpp"
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    require(HEADER.is_file(), "navigation terminal runtime module header is missing")
    require(SOURCE.is_file(), "navigation terminal runtime module source is missing")

    node = NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    feature = FEATURE_SOURCE.read_text(encoding="utf-8")

    require(
        "public NavigationTerminalRuntimePort" not in node,
        "composition root must not implement the terminal runtime port",
    )
    require(
        "std::unique_ptr<NavigationTerminalRuntimeModule> terminal_runtime_;" in feature,
        "navigation feature must own the terminal runtime module",
    )
    require(
        "std::unique_ptr<NavigationTerminalRuntimeModule>" not in node,
        "composition root must not own the terminal runtime module",
    )
    for legacy_member in (
        "navigation_terminal_speed_limit_pub_",
        "post_nav2_final_verify_reverse_enable_pub_",
        "mode_controller_status_sub_",
        "yaw_align_actual_stop_odom_sub_",
        "local_costmap_sub_",
        "rosout_sub_",
        "latest_terminal_actual_vx_mps_",
        "latest_local_costmap_",
    ):
        require(
            legacy_member not in node,
            f"composition root still owns terminal runtime state: {legacy_member}",
        )

    require(
        "class NavigationTerminalRuntimeModule" in header
        and "public NavigationTerminalRuntimePort" in header,
        "terminal runtime module must implement NavigationTerminalRuntimePort",
    )
    for responsibility in (
        "publish_speed_limit_for_goal",
        "update_reverse_permit_for_goal",
        "wait_for_actual_stop",
        "local_costmap_update_count",
        "local_costmap_message_filter_drop_count",
    ):
        require(
            responsibility in header or responsibility in source,
            f"terminal runtime responsibility is missing: {responsibility}",
        )


if __name__ == "__main__":
    main()

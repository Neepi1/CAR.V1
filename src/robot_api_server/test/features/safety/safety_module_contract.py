#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "safety"
    / "safety_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "safety" / "safety_module.cpp"
ROUTER_WIRING = (
    PACKAGE_ROOT / "src" / "application" / "routing" / "application_router_wiring.cpp"
)
NAVIGATION_FEATURE = (
    PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_feature_module.cpp"
)


def main() -> None:
    root = ROOT_NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    router_wiring = ROUTER_WIRING.read_text(encoding="utf-8")
    navigation_feature = NAVIGATION_FEATURE.read_text(encoding="utf-8")

    for token in (
        "struct SafetyModulePorts",
        "class SafetyModule",
        "handle_http",
        "latch_stop_for_unproven_navigation_terminal",
        "run_admitted_operation",
    ):
        assert token in header
    for token in (
        'request.path == "/api/v1/safety/stop"',
        'request.path != "/api/v1/safety/resume"',
        'floor_runtime_interlock_response("safety_resume")',
        '"safety_resume"',
        "ros.publish_estop(true)",
        "ports.clear_teleop_command()",
        "ports.publish_teleop_zero_burst()",
        "ports.publish_terminal_zero_burst()",
        "ports.stop_predock_motion()",
    ):
        assert token in source
    for legacy in (
        'request.path == "/api/v1/safety/stop"',
        'request.path == "/api/v1/safety/resume"',
        "HttpResponse publish_estop",
        "void latch_safety_stop_for_unproven_navigation_terminal",
        "bool safety_motion_hard_blocked_snapshot",
        "SafetyRosAdapterOptions safety_options",
        "std::unique_ptr<SafetyRosAdapter>",
    ):
        assert legacy not in root, f"legacy safety logic remains in root: {legacy}"
    assert "safety->handle_http(request, motion_admission_epoch)" in router_wiring
    assert "dependencies_.safety->latch_stop_for_unproven_navigation_terminal" in navigation_feature

    print("PASS: safety module owns both HTTP routes and emergency stop sequencing")


if __name__ == "__main__":
    main()

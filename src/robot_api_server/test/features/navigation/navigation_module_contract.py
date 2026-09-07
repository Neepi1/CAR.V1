#!/usr/bin/env python3

"""Lock the complete navigation HTTP slice outside the API composition root."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
MODULE_HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "navigation"
    / "navigation_module.hpp"
)
MODULE_SOURCE = PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_module.cpp"
STATE_SOURCE = PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_state.cpp"
ROUTER_WIRING = (
    PACKAGE_ROOT / "src" / "application" / "routing" / "application_router_wiring.cpp"
)


def main() -> None:
    root = ROOT_NODE.read_text(encoding="utf-8")
    header = MODULE_HEADER.read_text(encoding="utf-8")
    module = MODULE_SOURCE.read_text(encoding="utf-8")
    state = STATE_SOURCE.read_text(encoding="utf-8")
    router_wiring = ROUTER_WIRING.read_text(encoding="utf-8")

    assert "std::optional<HttpResponse> handle_http(" in header
    assert "navigation->handle_http(request, motion_admission_epoch)" in router_wiring

    for route in (
        "/api/v1/navigation/start",
        "/api/v1/navigation/state",
        "/api/v1/navigation/pre_goal_check",
        "/api/v1/navigation/goal",
        "/api/v1/navigation/cancel",
        "/api/v1/navigation/stop_runtime",
    ):
        assert route in module, f"NavigationModule route missing: {route}"

    for forbidden in (
        "HttpResponse handle_navigation_state(",
        "navigation_module_->handle_start(",
        "navigation_module_->handle_pre_goal_check(",
        "navigation_module_->handle_goal(",
        "navigation_module_->handle_cancel(",
        "effective_amcl_correction_pending",
        "\"navigation_goal\":" + " << navigation_goal_job_json_locked()",
    ):
        assert forbidden not in root, f"composition root still owns navigation HTTP detail: {forbidden}"

    for ownership in (
        "navigation_state_response(state_config, snapshot)",
        "snapshot.navigation_goal_json = mission_runtime_.json()",
        "snapshot.navigation_cancel_json = cancel_runtime_.json()",
    ):
        assert ownership in module, f"NavigationModule state ownership missing: {ownership}"

    for field in (
        "safe_for_goal_start",
        "localization_recovery_required",
        "pre_navigation_dock_check",
        "post_relocalization_settle",
        "post_undock_settle",
        "navigation_goal",
        "navigation_cancel",
    ):
        assert field in state, f"navigation state response field missing: {field}"


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
NODE = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "composition"
    / "application_composition_module.cpp"
)
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "application"
    / "routing"
    / "application_router_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "application" / "routing" / "application_router_module.cpp"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"


def main() -> None:
    assert HEADER.exists(), "application router public boundary is missing"
    assert SOURCE.exists(), "application router implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    compact_node = "".join(node.split())
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    assert "class ApplicationRouterModule" in header
    assert "capture_motion_admission_epoch" in header
    assert "elevator_interlock" in header
    assert "gateway_token_configured" in header
    assert "HttpResponse ApplicationRouterModule::route" in source
    assert "endpoint is reserved but not wired" in source
    assert "endpoint not found:" in source
    assert "const bool elevator_request" in source
    assert "if (elevator_request)" in source

    # Legacy signatures are ABI slots, not permission to reacquire the test lock.
    for relative in (
        "mapping/mapping_module.cpp", "navigation/navigation_module.cpp",
        "docking/lifecycle/docking_http_module.cpp", "maps/maps_module.cpp",
        "floor_switch/floor_switch_module.cpp", "teleop/teleop_module.cpp",
        "safety/safety_feature_module.cpp", "localization/localization_feature_module.cpp",
    ):
        feature = (PACKAGE_ROOT / "src/features" / relative).read_text(encoding="utf-8")
        assert ".acquire_motion_admission(" not in feature, relative
        assert "->acquire_motion_admission(" not in feature, relative
        assert "ports_.elevator_interlock()" not in feature, relative

    ordered_markers = (
        "ports_.elevator_interlock(request)",
        "ports_.system_status(request)",
        "ports_.maps(request, motion_admission_epoch)",
        "ports_.elevator(",
        "ports_.mapping(request, motion_admission_epoch)",
        "ports_.gateway_metadata(request)",
        "ports_.subscriptions(request)",
        "ports_.safety(request, motion_admission_epoch)",
        "ports_.floor_switch(request, motion_admission_epoch)",
        "ports_.localization(request, motion_admission_epoch)",
        "ports_.navigation(request, motion_admission_epoch)",
        "ports_.docking(request, motion_admission_epoch)",
    )
    positions = [source.index(marker) for marker in ordered_markers]
    assert positions == sorted(positions), "commercial route precedence changed"

    assert "application/routing/application_router_module.hpp" in node
    assert "std::unique_ptr<ApplicationRouterModule> application_router_module_;" in node
    assert (
        "application_router_module_->route(request,maintenance_peer_is_loopback)"
        in compact_node
    )
    assert "HttpResponse route(" not in node
    assert "HttpResponse not_wired(" not in node
    assert "endpoint is reserved but not wired" not in node
    assert "src/application/routing/application_router_module.cpp" in cmake


if __name__ == "__main__":
    main()

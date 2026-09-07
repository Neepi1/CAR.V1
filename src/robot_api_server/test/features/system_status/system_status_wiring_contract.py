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
    / "features"
    / "system_status"
    / "system_status_wiring.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "system_status" / "system_status_wiring.cpp"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"


def main() -> None:
    node = NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    assert "struct SystemStatusWiringDependencies" in header
    assert "make_system_status_module_ports" in header
    assert "make_system_status_module_ports" in source
    for projection in (
        "snapshot.mapping_status_json",
        "snapshot.safety",
        "snapshot.bms",
        "snapshot.runtime",
        "snapshot.dock",
        "snapshot.localization",
        "snapshot.amcl",
        "snapshot.bridge",
        "snapshot.navigation_goal_json",
        "snapshot.post_relocalization_settle_json",
        "snapshot.post_undock_settle_json",
        "snapshot.floor_runtime_interlock",
        "snapshot.keepout_integrity_degraded",
        "snapshot.delayed_side_effect_unknown_count",
        "snapshot.subscriptions_json",
        "snapshot.http_active_connections",
    ):
        assert projection in source, projection

    assert "make_system_status_module_ports(" in node
    assert "SystemStatusSnapshot snapshot;" not in node
    assert "RobotPoseRuntimeContextSnapshot snapshot;" not in node
    assert "RobotPoseIdentitySnapshot snapshot;" not in node
    assert "src/features/system_status/system_status_wiring.cpp" in cmake


if __name__ == "__main__":
    main()

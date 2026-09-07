#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
COMPOSITION = (
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
    / "floor_switch"
    / "floor_switch_feature_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT / "src" / "features" / "floor_switch" / "floor_switch_feature_module.cpp"
)


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    for token in (
        "struct FloorSwitchFeatureModuleDependencies",
        "class FloorSwitchFeatureModule",
        "FloorSwitchModule & module()",
        "const std::string & floor_status_topic()",
        "void shutdown()",
    ):
        assert token in header, token

    assert "std::unique_ptr<FloorSwitchModule> module_" in source
    assert (
        "std::unique_ptr<FloorSwitchFeatureModule> floor_switch_feature_module_;"
        in composition
    )
    assert "FloorSwitchFeatureModule" not in root
    for leaked in (
        "FloorSwitchModulePorts ",
        "std::unique_ptr<FloorSwitchModule>",
        "FloorSwitchRuntimeSnapshot snapshot;",
    ):
        assert leaked not in composition, leaked

    print("PASS: floor-switch aggregate owns floor-switch port wiring")


if __name__ == "__main__":
    main()

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
    / "maps"
    / "maps_feature_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "maps" / "maps_feature_module.cpp"


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    for token in (
        "struct MapsFeatureModuleDependencies",
        "class MapsFeatureModule",
        "MapsModule & module()",
        "MapRuntimeStateStore & runtime_state_store()",
        "std::mutex & cross_asset_commit_mutex()",
    ):
        assert token in header, token

    assert "std::unique_ptr<MapRuntimeStateStore> runtime_state_store_" in source
    assert "std::unique_ptr<MapsModule> module_" in source
    assert "std::unique_ptr<MapsFeatureModule> maps_feature_module_;" in composition
    assert "MapsFeatureModule" not in root
    for leaked in (
        "MapsModulePorts ",
        "std::unique_ptr<MapRuntimeStateStore>",
        "std::unique_ptr<MapsModule>",
        "std::mutex elevator_asset_mutation_mutex_",
    ):
        assert leaked not in composition, leaked

    print("PASS: maps aggregate owns maps runtime state and port wiring")


if __name__ == "__main__":
    main()

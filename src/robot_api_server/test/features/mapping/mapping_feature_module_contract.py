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
    / "mapping"
    / "mapping_feature_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "mapping" / "mapping_feature_module.cpp"


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    for token in (
        "struct MappingFeatureModuleDependencies",
        "class MappingFeatureModule",
        "MappingModule & module()",
        "void shutdown()",
    ):
        assert token in header, token

    assert "std::unique_ptr<MappingModule> module_" in source
    assert (
        "std::unique_ptr<MappingFeatureModule> mapping_feature_module_;" in composition
    )
    assert "MappingFeatureModule" not in root
    for leaked in (
        "MappingModulePorts ",
        "std::unique_ptr<MappingModule>",
        "MappingNavigationActionResult{",
    ):
        assert leaked not in composition, leaked

    print("PASS: mapping aggregate owns mapping and navigation handoff wiring")


if __name__ == "__main__":
    main()

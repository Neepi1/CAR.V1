#!/usr/bin/env python3

from pathlib import Path
import re


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
    / "elevator"
    / "elevator_feature_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "elevator" / "elevator_feature_module.cpp"


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "struct ElevatorFeatureModuleDependencies" in header
    assert "class ElevatorFeatureModule" in header
    assert re.search(r"ElevatorModule\s*&\s*module\(\)", header)
    assert "ElevatorModulePorts ports;" in source
    assert "std::unique_ptr<ElevatorModule> module_;" in source
    assert (
        "std::unique_ptr<ElevatorFeatureModule> elevator_feature_module_;"
        in composition
    )
    assert "ElevatorFeatureModule" not in root
    for leaked in (
        "ElevatorModulePorts elevator_ports;",
        "std::unique_ptr<ElevatorModule> elevator_module_;",
        "std::make_unique<ElevatorModule>(",
    ):
        assert leaked not in composition, leaked


if __name__ == "__main__":
    main()

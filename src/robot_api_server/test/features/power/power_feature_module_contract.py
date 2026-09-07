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
    / "power"
    / "power_feature_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "power" / "power_feature_module.cpp"


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "struct PowerFeatureModuleDependencies" in header
    assert "class PowerFeatureModule" in header
    assert re.search(r"PowerModule\s*&\s*module\(\)", header)
    assert "PowerModulePorts ports;" in source
    assert "std::unique_ptr<PowerModule> module_;" in source
    assert "std::unique_ptr<PowerFeatureModule> power_feature_module_;" in composition
    assert "PowerFeatureModule" not in root
    for leaked in (
        "PowerModulePorts power_module_ports;",
        "std::unique_ptr<PowerModule> power_module_;",
        "std::make_unique<PowerModule>(",
    ):
        assert leaked not in composition, leaked


if __name__ == "__main__":
    main()

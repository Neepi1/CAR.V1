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
    / "teleop"
    / "teleop_feature_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "teleop" / "teleop_feature_module.cpp"


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "struct TeleopFeatureModuleDependencies" in header
    assert "class TeleopFeatureModule" in header
    assert re.search(r"TeleopModule\s*&\s*module\(\)", header)
    assert "void shutdown()" in header
    assert "TeleopModulePorts ports;" in source
    assert "std::unique_ptr<TeleopModule> module_;" in source
    assert "std::unique_ptr<TeleopFeatureModule> teleop_feature_module_;" in composition
    assert "TeleopFeatureModule" not in root
    for leaked in (
        "TeleopModulePorts teleop_ports;",
        "std::unique_ptr<TeleopModule> teleop_module_;",
        "std::make_unique<TeleopModule>(",
    ):
        assert leaked not in composition, leaked


if __name__ == "__main__":
    main()

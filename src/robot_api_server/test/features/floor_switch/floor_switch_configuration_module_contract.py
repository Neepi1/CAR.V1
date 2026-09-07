#!/usr/bin/env python3

import re
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = (
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
    / "floor_switch_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "floor_switch"
    / "floor_switch_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "floor_status_topic",
    "floor_transition_status_topic",
    "floor_runtime_negative_interlock_enabled",
    "floor_switch_service",
    "live_floor_switch_action",
    "live_floor_switch_timeout_sec",
}


def declared_parameters(text: str) -> list[str]:
    return re.findall(r'declare_parameter<[^>]+>\s*\(\s*"([^"]+)"', text)


def main() -> None:
    assert HEADER.is_file(), "floor-switch configuration header is missing"
    assert SOURCE.is_file(), "floor-switch configuration source is missing"
    root = ROOT_NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    declarations = declared_parameters(source)

    assert len(EXPECTED_PARAMETERS) == 6
    assert len(declarations) == 6, declarations
    assert len(declarations) == len(set(declarations))
    assert set(declarations) == EXPECTED_PARAMETERS
    for parameter in EXPECTED_PARAMETERS:
        assert not re.search(
            rf'declare_parameter<[^>]+>\s*\(\s*"{re.escape(parameter)}"', root
        ), f"composition root still declares floor-switch parameter: {parameter}"

    for token in (
        "struct FloorSwitchConfigurationInputs",
        "struct FloorSwitchConfiguration",
        "FloorSwitchModuleConfig module",
        "std::string floor_status_topic",
    ):
        assert token in header
    for projection in (
        "module.maps_root = inputs.maps_root",
        "module.runtime_map_context_file = inputs.runtime_map_context_file",
        "module.localization_health_topic = inputs.localization_health_topic",
        "module.service_timeout = inputs.service_timeout",
    ):
        assert projection in source
    assert "FloorSwitchConfigurationModule::declare_parameters" in root
    assert "std::string floor_status_topic_" not in root

    print(
        "PASS: floor-switch configuration owns all 6 ROS parameters and all "
        "neighbor projections"
    )


if __name__ == "__main__":
    main()

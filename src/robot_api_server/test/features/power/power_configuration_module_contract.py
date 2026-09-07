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
    / "power"
    / "power_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "power"
    / "power_configuration_module.cpp"
)
DOCKING_CONFIGURATION_SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "configuration"
    / "docking_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "bms_state_topic",
    "bms_state_max_age_sec",
    "teleop_charging_current_min_a",
    "bms_charging_contact_voltage_min_v",
    "bms_charging_contact_voltage_max_v",
    "bms_full_soc_threshold_pct",
    "bms_full_soc_voltage_contact_enable",
    "dock_contact_latch_bms_require_contact_sec",
}


def declared_parameters(text: str) -> list[str]:
    return re.findall(r'declare_parameter<[^>]+>\s*\(\s*"([^"]+)"', text)


def main() -> None:
    for path in (HEADER, SOURCE, ROOT_NODE, DOCKING_CONFIGURATION_SOURCE):
        assert path.is_file(), f"missing required file: {path}"

    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    root = ROOT_NODE.read_text(encoding="utf-8")
    docking_source = DOCKING_CONFIGURATION_SOURCE.read_text(encoding="utf-8")
    declarations = declared_parameters(source)

    assert len(EXPECTED_PARAMETERS) == 8
    assert len(declarations) == 8, declarations
    assert len(declarations) == len(set(declarations))
    assert set(declarations) == EXPECTED_PARAMETERS
    for parameter in EXPECTED_PARAMETERS:
        assert not re.search(
            rf'declare_parameter<[^>]+>\s*\(\s*"{re.escape(parameter)}"', root
        ), f"composition root still declares power parameter: {parameter}"
    assert "dock_contact_latch_bms_require_contact_sec" not in docking_source
    assert "teleop_stop_on_charging" not in source

    for token in (
        "struct PowerConfiguration",
        "PowerModuleConfig module",
        "class PowerConfigurationModule",
    ):
        assert token in header
    assert "PowerConfigurationModule::declare_parameters" in root
    assert "PowerModuleConfig power_module_config" not in root

    print(
        "PASS: power configuration owns all 8 BMS evidence parameters without "
        "absorbing the teleop stop policy"
    )


if __name__ == "__main__":
    main()

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
    / "mapping"
    / "mapping_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "mapping"
    / "mapping_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "mapping_2d_start_command",
    "mapping_2d_log_file",
    "mapping_lidar_rps_xps_state_dir",
    "mapping_graceful_stop_timeout_sec",
    "mapping_scan_owner_restore_timeout_sec",
    "mapping_scan_owner_topic",
    "mapping_navigation_scan_owner_node",
    "mapping_resident_scan_control_service",
    "mapping_2d_live_map_topic",
    "mapping_2d_live_map_max_age_sec",
}


def declared_parameters(text: str) -> list[str]:
    return re.findall(r'declare_parameter<[^>]+>\s*\(\s*"([^"]+)"', text)


def main() -> None:
    assert HEADER.is_file(), "mapping configuration header is missing"
    assert SOURCE.is_file(), "mapping configuration source is missing"
    root = ROOT_NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    declarations = declared_parameters(source)

    assert len(EXPECTED_PARAMETERS) == 10
    assert len(declarations) == 10, declarations
    assert len(declarations) == len(set(declarations))
    assert set(declarations) == EXPECTED_PARAMETERS
    for parameter in EXPECTED_PARAMETERS:
        assert not re.search(
            rf'declare_parameter<[^>]+>\s*\(\s*"{re.escape(parameter)}"', root
        ), f"composition root still declares mapping parameter: {parameter}"

    assert "struct MappingConfigurationInputs" in header
    assert "MappingModuleConfig declare_parameters" in header
    assert "config.maps_root = inputs.maps_root" in source
    assert "config.runtime_maps_dir = inputs.runtime_maps_dir" in source
    assert "MappingConfigurationModule::declare_parameters" in root

    print(
        "PASS: mapping configuration owns all 10 ROS parameters and both maps "
        "path projections"
    )


if __name__ == "__main__":
    main()

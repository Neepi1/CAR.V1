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
    / "maps"
    / "maps_configuration_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "maps" / "maps_configuration_module.cpp"

EXPECTED_PARAMETERS = {
    "maps_root",
    "runtime_maps_dir",
    "runtime_map_context_file",
    "last_navigation_map_file",
    "keepout_mask_load_service",
    "keepout_mask_state_service",
    "keepout_filter_info_state_service",
    "keepout_mask_get_parameters_service",
    "keepout_runtime_stage_root",
    "keepout_mask_topic",
    "keepout_filter_info_topic",
    "global_costmap_get_parameters_service",
    "global_costmap_clear_service",
    "global_costmap_topic",
    "keepout_runtime_apply_timeout_sec",
}


def declared_parameters(text: str) -> list[str]:
    return re.findall(r'declare_parameter<[^>]+>\s*\(\s*"([^"]+)"', text)


def main() -> None:
    assert HEADER.is_file(), "maps configuration header is missing"
    assert SOURCE.is_file(), "maps configuration source is missing"
    root = ROOT_NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    declarations = declared_parameters(source)

    assert len(EXPECTED_PARAMETERS) == 15
    assert len(declarations) == 15, declarations
    assert len(declarations) == len(set(declarations))
    assert set(declarations) == EXPECTED_PARAMETERS
    for parameter in EXPECTED_PARAMETERS:
        assert not re.search(
            rf'declare_parameter<[^>]+>\s*\(\s*"{re.escape(parameter)}"', root
        ), f"composition root still declares maps parameter: {parameter}"

    for token in (
        "struct MapsRuntimePaths",
        "struct MapsConfiguration",
        "MapsModuleConfig module",
        "MapsRuntimePaths runtime_paths",
    ):
        assert token in header
    for projection in (
        "module.maps_root = paths.maps_root",
        "module.runtime_maps_dir = paths.runtime_maps_dir",
        "module.runtime_map_context_file = paths.runtime_map_context_file",
    ):
        assert projection in source
    for legacy_scalar in (
        "std::string maps_root_",
        "std::string runtime_maps_dir_",
        "std::string runtime_map_context_file_",
        "std::string last_navigation_map_file_",
    ):
        assert legacy_scalar not in root, f"legacy maps scalar remains: {legacy_scalar}"
    assert "MapsConfigurationModule::declare_parameters" in root

    print(
        "PASS: maps configuration owns all 15 ROS parameters and emits one "
        "shared immutable path projection"
    )


if __name__ == "__main__":
    main()

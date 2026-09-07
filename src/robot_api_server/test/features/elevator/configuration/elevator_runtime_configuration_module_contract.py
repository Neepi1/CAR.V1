#!/usr/bin/env python3

import re
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
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
    / "elevator"
    / "configuration"
    / "elevator_runtime_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "elevator"
    / "configuration"
    / "elevator_runtime_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "elevator_runtime_adapter_enabled",
    "elevator_arm_button_control_enabled",
    "elevator_arm_service_host",
    "elevator_arm_service_port",
    "elevator_arm_request_timeout_sec",
    "elevator_arm_task_timeout_sec",
    "elevator_arm_poll_interval_sec",
    "elevator_scoped_behavior_tree",
    "elevator_hall_call_scoped_behavior_tree",
    "elevator_reverse_entry_staging_behavior_tree",
    "elevator_reverse_docking_behavior_tree",
    "elevator_cabin_entry_direct_behavior_tree",
    "elevator_entry_collision_bypass_permit_topic",
    "elevator_entry_collision_bypass_refresh_sec",
    "elevator_recovery_hold_release_service",
}


def declared_parameters(text: str) -> list[str]:
    return re.findall(r'declare_parameter<[^>]+>\s*\(\s*"([^"]+)"', text)


def main() -> None:
    assert HEADER.is_file(), "elevator runtime configuration header is missing"
    assert SOURCE.is_file(), "elevator runtime configuration source is missing"
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
        ), f"composition root still declares elevator runtime parameter: {parameter}"

    assert "struct ElevatorRuntimeConfigurationInputs" in header
    assert "ElevatorModuleConfig declare_parameters" in header
    for projection in (
        "config.maps_root = inputs.maps_root",
        "config.runtime_map_context_file = inputs.runtime_map_context_file",
        "config.navigate_to_pose_action = inputs.navigate_to_pose_action",
        "config.floor_switch_action = inputs.floor_switch_action",
        "config.motion_allowed_topic = inputs.motion_allowed_topic",
        "config.safety_status_topic = inputs.safety_status_topic",
        "config.navigation_status_topic = inputs.navigation_status_topic",
    ):
        assert projection in source
    assert "ElevatorRuntimeConfigurationModule::declare_parameters" in root

    print(
        "PASS: elevator runtime configuration owns all 15 ROS parameters and "
        "projects all neighbor inputs"
    )


if __name__ == "__main__":
    main()

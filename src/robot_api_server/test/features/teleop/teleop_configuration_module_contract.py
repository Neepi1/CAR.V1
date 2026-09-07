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
    / "teleop"
    / "teleop_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "teleop"
    / "teleop_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "teleop_stop_on_charging",
    "teleop_cmd_topic",
    "teleop_reverse_enable_topic",
    "teleop_pose_topic",
    "teleop_max_linear_x_mps",
    "teleop_max_angular_z_radps",
    "teleop_allow_reverse",
    "teleop_require_mapping_active",
    "teleop_watchdog_timeout_sec",
    "teleop_socket_idle_timeout_sec",
    "teleop_repeat_rate_hz",
}


def declared_parameters(text: str) -> list[str]:
    return re.findall(r'declare_parameter<[^>]+>\s*\(\s*"([^"]+)"', text)


def main() -> None:
    for path in (HEADER, SOURCE, ROOT_NODE):
        assert path.is_file(), f"missing required file: {path}"

    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    root = ROOT_NODE.read_text(encoding="utf-8")
    declarations = declared_parameters(source)

    assert len(EXPECTED_PARAMETERS) == 11
    assert len(declarations) == 11, declarations
    assert len(declarations) == len(set(declarations))
    assert set(declarations) == EXPECTED_PARAMETERS
    for parameter in EXPECTED_PARAMETERS:
        assert not re.search(
            rf'declare_parameter<[^>]+>\s*\(\s*"{re.escape(parameter)}"', root
        ), f"composition root still declares teleop parameter: {parameter}"

    for token in (
        "struct TeleopConfigurationInputs",
        "int subscription_max_ttl_ms",
        "struct TeleopConfiguration",
        "TeleopModuleConfig module",
        "class TeleopConfigurationModule",
    ):
        assert token in header
    assert "TeleopConfigurationModule::declare_parameters" in root
    assert "inputs.subscription_max_ttl_ms" in source
    assert "subscription_default_ttl_ms" not in source
    assert "subscription_max_ttl_ms\"" not in source
    assert "TeleopModuleConfig teleop_module_config" not in root

    print(
        "PASS: teleop configuration owns all 11 parameters and consumes the "
        "normalized subscription TTL projection"
    )


if __name__ == "__main__":
    main()

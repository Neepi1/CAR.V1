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
    / "application"
    / "runtime_configuration"
    / "runtime_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "runtime_configuration"
    / "runtime_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "navigate_to_pose_action",
    "navigate_to_pose_status_topic",
    "service_timeout_sec",
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

    assert len(declarations) == 3, declarations
    assert len(declarations) == len(set(declarations))
    assert set(declarations) == EXPECTED_PARAMETERS
    assert not declared_parameters(root), "composition root still declares ROS parameters"

    for token in (
        "struct RuntimeConfiguration",
        "std::string navigate_to_pose_action",
        "std::string navigate_to_pose_status_topic",
        "double service_timeout_sec",
        "std::chrono::nanoseconds service_timeout() const",
        "class RuntimeConfigurationModule",
    ):
        assert token in header
    assert "RuntimeConfigurationModule::declare_parameters" in root
    for legacy_member in (
        "navigate_to_pose_action_",
        "navigate_to_pose_status_topic_",
        "service_timeout_sec_",
    ):
        assert legacy_member not in root

    assert root.count("runtime_configuration.navigate_to_pose_action") >= 4
    assert root.count("runtime_configuration.navigate_to_pose_status_topic") >= 2
    assert root.count("runtime_configuration.service_timeout_sec") >= 3
    assert "runtime_configuration.service_timeout()" in root

    print(
        "PASS: shared runtime configuration owns the final 3 parameters and "
        "the composition root declares none"
    )


if __name__ == "__main__":
    main()

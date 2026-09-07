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
    / "subscriptions"
    / "subscription_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "subscriptions"
    / "subscription_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "scan_topic",
    "scan_max_age_sec",
    "subscription_default_ttl_ms",
    "subscription_max_ttl_ms",
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

    assert len(declarations) == 4, declarations
    assert len(declarations) == len(set(declarations))
    assert set(declarations) == EXPECTED_PARAMETERS
    for parameter in EXPECTED_PARAMETERS:
        assert not re.search(
            rf'declare_parameter<[^>]+>\s*\(\s*"{re.escape(parameter)}"', root
        ), f"composition root still declares subscription parameter: {parameter}"

    assert "class SubscriptionConfigurationModule" in header
    assert "SubscriptionModuleConfig declare_parameters" in header
    assert "SubscriptionConfigurationModule::declare_parameters" in root
    assert "SubscriptionModuleConfig subscription_module_config" not in root

    print("PASS: application subscriptions configuration owns all 4 ROS parameters")


if __name__ == "__main__":
    main()

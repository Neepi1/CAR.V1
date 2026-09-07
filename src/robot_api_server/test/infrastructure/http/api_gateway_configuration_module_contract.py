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
    / "infrastructure"
    / "http"
    / "api_gateway_configuration_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "infrastructure"
    / "http"
    / "api_gateway_configuration_module.cpp"
)

EXPECTED_PARAMETERS = {
    "host",
    "port",
    "api_token",
    "max_http_connections",
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
        ), f"composition root still declares gateway parameter: {parameter}"

    assert "class ApiGatewayConfigurationModule" in header
    assert "ApiGatewayModuleConfig declare_parameters" in header
    assert "ApiGatewayConfigurationModule::declare_parameters" in root
    assert "ApiGatewayModuleConfig api_gateway_config" not in root
    assert "ROBOT_API_TOKEN" not in source

    print("PASS: HTTP gateway configuration owns all 4 ROS parameters")


if __name__ == "__main__":
    main()

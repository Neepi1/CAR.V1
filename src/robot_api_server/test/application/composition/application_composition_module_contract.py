#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "application"
    / "composition"
    / "application_composition_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "composition"
    / "application_composition_module.cpp"
)


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "class ApplicationCompositionModule" in header
    assert "struct Impl;" in header
    assert "std::unique_ptr<Impl> impl_;" in header
    assert "ApplicationCompositionModule(rclcpp::Node &node)" in header

    assert "ApplicationCompositionModule::Impl" in source
    for owned_module in (
        "SafetyFeatureModule",
        "PowerFeatureModule",
        "SubscriptionFeatureModule",
        "MapsFeatureModule",
        "FloorSwitchFeatureModule",
        "MappingFeatureModule",
        "TeleopFeatureModule",
        "LocalizationFeatureModule",
        "DockingFeatureModule",
        "ElevatorFeatureModule",
        "NavigationFeatureModule",
        "ApiGatewayModule",
        "SystemStatusModule",
        "ApplicationRouterModule",
        "RuntimeModeCoordinator",
    ):
        assert owned_module in source, owned_module
        assert owned_module not in root, owned_module

    for leaked_assembly in (
        "declare_parameters(",
        "make_application_router_ports(",
        "make_system_status_module_ports(",
        "api_gateway_module_->start()",
    ):
        assert leaked_assembly in source, leaked_assembly
        assert leaked_assembly not in root, leaked_assembly


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

"""Lock the complete navigation runtime family behind one feature boundary."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
COMPOSITION = (
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
    / "navigation"
    / "navigation_feature_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "navigation" / "navigation_feature_module.cpp"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"


def main() -> None:
    assert HEADER.exists(), "NavigationFeatureModule public boundary is missing"
    assert SOURCE.exists(), "NavigationFeatureModule implementation is missing"

    root = ROOT_NODE.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    assert "class NavigationFeatureModule" in header
    assert "struct NavigationFeatureModuleDependencies" in header
    for accessor in (
        "NavigationModule & module()",
        "NavigationActionRuntime & action_runtime()",
        "NavigationMissionRuntime & mission_runtime()",
        "NavigationTerminalRuntimeModule & terminal_runtime()",
        "NavigationGoalExecutionModule & goal_execution()",
        "void shutdown()",
    ):
        assert accessor in header, accessor

    for owned_wiring in (
        "NavigationActionRuntimePorts action_ports;",
        "NavigationModulePorts module_ports;",
        "NavigationTerminalRuntimePorts terminal_ports;",
        "NavigationGoalExecutionPorts execution_ports;",
        "pre_goal_dock_snapshot(",
        "undock_before_navigation(",
        "BridgeReadinessSnapshot bridge_readiness_snapshot(",
        "std::unique_ptr<NavigationGoalExecutor> goal_executor_;",
    ):
        assert owned_wiring in source, owned_wiring

    assert (
        "std::unique_ptr<NavigationFeatureModule> navigation_feature_module_;"
        in composition
    )
    assert "std::make_unique<NavigationFeatureModule>" in composition
    assert "NavigationFeatureModule" not in root
    for removed_root_owner in (
        "NavigationActionRuntimePorts navigation_action_ports;",
        "NavigationModulePorts navigation_module_ports;",
        "NavigationTerminalRuntimePorts navigation_terminal_runtime_ports;",
        "NavigationGoalExecutionPorts navigation_goal_execution_ports;",
        "std::unique_ptr<NavigationModule> navigation_module_;",
        "NavigationActionRuntime * navigation_action_runtime_",
        "NavigationMissionRuntime * navigation_mission_runtime_",
        "std::unique_ptr<NavigationTerminalRuntimeModule>",
        "std::unique_ptr<NavigationGoalExecutionModule>",
        "std::unique_ptr<NavigationGoalExecutor>",
        "BridgeReadinessSnapshot bridge_readiness_snapshot(",
        "bool undock_before_navigation_if_needed(",
    ):
        assert removed_root_owner not in composition, removed_root_owner

    assert "src/features/navigation/navigation_feature_module.cpp" in cmake


if __name__ == "__main__":
    main()

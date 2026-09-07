#!/usr/bin/env python3

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
    / "docking"
    / "docking_feature_module.hpp"
)
SOURCE = PACKAGE_ROOT / "src" / "features" / "docking" / "docking_feature_module.cpp"


def main() -> None:
    root = ROOT_NODE.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    for token in (
        "struct DockingFeatureModuleDependencies",
        "struct DockingFeatureLateDependencies",
        "class DockingFeatureModule",
        "void complete(DockingFeatureLateDependencies dependencies)",
        "void prepare_shutdown()",
        "void shutdown()",
    ):
        assert token in header, token

    for owned_type in (
        "PredockAlignmentPolicy",
        "DockContactInterlockModule",
        "DockingJobStore",
        "DockingCorrectionPauseModule",
        "DockingRuntimeModule",
        "PreNavigationUndockModule",
        "PredockControlModule",
        "DockingJobExecutionModule",
        "DockingJobExecutor",
        "DockingPredockPoseResolver",
        "DockingHttpModule",
        "DockingStatusModule",
    ):
        assert f"unique_ptr<{owned_type}>" in source or f"unique_ptr<configuration::{owned_type}>" in source or f"unique_ptr<predock_alignment::{owned_type}>" in source, owned_type
    assert "DeferredWorkQueue service_work_queue_;" in source

    assert (
        "std::unique_ptr<DockingFeatureModule> docking_feature_module_;"
        in composition
    )
    assert "docking_feature_module_->complete(" in composition
    assert composition.index("docking_feature_module_->complete(") < composition.index(
        "api_gateway_module_->start();"
    )
    assert "DockingFeatureModule" not in root

    for leaked_wiring in (
        "DockContactInterlockPorts ",
        "DockingJobStorePorts ",
        "DockingCorrectionPausePorts ",
        "DockingRuntimePorts ",
        "PreNavigationUndockPorts ",
        "PredockControlPorts ",
        "DockingJobExecutionModulePorts ",
        "DockingPredockPoseResolverPorts ",
        "DockingHttpPorts ",
        "DockingStatusPorts ",
        "std::unique_ptr<DockingRuntimeModule>",
        "std::unique_ptr<DockingHttpModule>",
        "std::unique_ptr<DockingStatusModule>",
        "DeferredWorkQueue docking_service_work_queue_",
        "std::mutex docking_start_mutex_",
    ):
        assert leaked_wiring not in composition, leaked_wiring

    print("PASS: docking feature aggregate owns the complete docking runtime family")


if __name__ == "__main__":
    main()

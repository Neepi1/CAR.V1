#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
AGGREGATE = PACKAGE_ROOT / "src" / "features" / "docking" / "docking_feature_module.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "docking"
    / "configuration"
    / "docking_predock_pose_resolver.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "configuration"
    / "docking_predock_pose_resolver.cpp"
)


def main() -> None:
    assert HEADER.exists(), "DockingPredockPoseResolver public seam is missing"
    assert SOURCE.exists(), "DockingPredockPoseResolver implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    aggregate = AGGREGATE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "class DockingPredockPoseResolver" in header
    assert "struct DockingPredockPoseResolverConfig" in header
    assert "struct DockingPredockPoseResolverPorts" in header
    assert "std::optional<StoredPose> resolve(" in header
    assert "bool validate(" in header

    for behavior in (
        "manual_predock_explicit",
        "manual_predock_auto_id",
        "manual_predock_auto_name",
        "manual_predock_auto_unique_type",
        "multiple matching dock predock poses found",
        "multiple dock_predock poses found",
        "distance_check=disabled",
        "heading aligned to the charger",
    ):
        assert behavior in source, behavior

    assert 'features/docking/configuration/docking_predock_pose_resolver.hpp' in aggregate
    assert (
        "std::unique_ptr<configuration::DockingPredockPoseResolver> predock_pose_resolver_;"
        in aggregate
    )
    assert "DockingPredockPoseResolver>" not in node
    for removed_owner in (
        "  bool is_docking_predock_pose_type(",
        "  std::vector<std::string> docking_predock_pose_id_candidates(",
        "  std::optional<StoredPose> resolve_docking_predock_pose(",
        "  bool validate_manual_docking_predock_pose(",
        "docking_manual_predock_distance_check_enable_",
        "docking_manual_predock_min_distance_m_",
        "docking_manual_predock_max_distance_m_",
        "docking_manual_predock_max_yaw_error_rad_",
    ):
        assert removed_owner not in node, removed_owner


if __name__ == "__main__":
    main()

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
    / "lifecycle"
    / "docking_job_execution_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "lifecycle"
    / "docking_job_execution_module.cpp"
)


def main() -> None:
    assert HEADER.exists(), "DockingJobExecutionModule public seam is missing"
    assert SOURCE.exists(), "DockingJobExecutionModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    aggregate = AGGREGATE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "class DockingJobExecutionModule" in header
    assert "public DockingJobExecutionPort" in header
    assert "struct DockingJobExecutionModuleConfig" in header
    assert "struct DockingJobExecutionModulePorts" in header

    for behavior in (
        "PendingSideEffectEvidence",
        "async_send_goal(goal)",
        "timed out sending predock navigation goal",
        "exception sending predock goal",
        "mark_navigation_goal_sent",
    ):
        assert behavior in source, behavior

    assert 'features/docking/lifecycle/docking_job_execution_module.hpp' in aggregate
    assert "std::unique_ptr<DockingJobExecutionModule> job_execution_;" in aggregate
    assert "*job_execution_" in aggregate
    assert "DockingJobExecutionModule>" not in node
    assert "public DockingJobExecutionPort" not in node

    for removed_owner in (
        "class PendingSideEffectEvidence",
        "  std::mutex & docking_job_mutex()",
        "  DockingJob & docking_job_unsafe()",
        "  bool resume_navigation_runtime_for_docking(",
        "  DockingRelocalizationSettleResult wait_for_docking_relocalization_settle_barrier(",
        "  rclcpp::Time docking_goal_stamp()",
        "  bool send_predock_navigation_goal(",
        "  bool ordinary_final_yaw_align_active()",
        "  void record_docking_cmd_owner_conflict()",
        "  BmsChargingContactSnapshot bms_charging_contact_snapshot()",
        "  void mark_docking_nav_goal_sent(",
    ):
        assert removed_owner not in node, removed_owner


if __name__ == "__main__":
    main()

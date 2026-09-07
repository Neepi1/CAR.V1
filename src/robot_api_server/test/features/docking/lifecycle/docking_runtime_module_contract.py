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
    / "docking_runtime_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "lifecycle"
    / "docking_runtime_module.cpp"
)


def main() -> None:
    assert HEADER.exists(), "DockingRuntimeModule public seam is missing"
    assert SOURCE.exists(), "DockingRuntimeModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    aggregate = AGGREGATE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "class DockingRuntimeModule" in header
    assert "DockingObservationSnapshot observation_snapshot() const" in header
    assert "bool ensure_manager_running(std::string & detail)" in header
    assert "bool start_fine_docking(std::string & detail)" in header
    assert "bool call_undock_with_charging_retry(" in header
    assert "bool launch_worker(" in header
    assert "void shutdown()" in header

    for marker in (
        "create_subscription<std_msgs::msg::String>",
        "create_subscription<robot_interfaces::msg::DockTargetObservation>",
        "create_subscription<sensor_msgs::msg::LaserScan>",
        "create_publisher<geometry_msgs::msg::Twist>",
        "create_client<std_srvs::srv::Trigger>",
        "prepare_child_process",
        "call_trigger_service_observed",
        "docking manager ready; log_file=",
        "waiting for docking manager charging state before undock",
    ):
        assert marker in source, marker

    assert 'features/docking/lifecycle/docking_runtime_module.hpp' in aggregate
    assert "std::unique_ptr<DockingRuntimeModule> runtime_;" in aggregate
    assert "DockingRuntimeModule>" not in node

    # Port member names remain part of neighboring modules' public adapters;
    # forbid concrete implementations and ROS/process resources in the root,
    # while allowing those composition-only assignments.
    for ownership_marker in (
        "  bool docking_manager_process_running_locked(",
        "  bool ensure_docking_manager_running(",
        "  bool call_docking_trigger_service(",
        "  DockingUndockServiceObservation call_docking_trigger_service_observed(",
        "  bool call_undock_service_with_charging_retry(",
        "  void handle_docking_gs2_scan(",
        "  double docking_gs2_scan_age_sec(",
        "  void handle_docking_target_observation(",
        "  double docking_target_observation_age_sec(",
        "  double docking_observation_age_sec(",
        "  bool docking_observation_usable(",
        "  std::string docking_observation_detail(",
        "  void record_undock_status_observation(",
        "  void join_docking_worker(",
        "docking_manager_pid_",
        "docking_gs2_scan_mutex_",
        "docking_target_observation_mutex_",
        "docking_worker_",
        "predock_yaw_align_cmd_pub_",
        "predock_lateral_align_forced_mode_pub_",
        "docking_status_sub_",
        "docking_gs2_scan_sub_",
        "docking_target_observation_sub_",
        "docking_start_client_",
        "docking_stop_client_",
        "docking_undock_client_",
    ):
        assert ownership_marker not in node, ownership_marker


if __name__ == "__main__":
    main()

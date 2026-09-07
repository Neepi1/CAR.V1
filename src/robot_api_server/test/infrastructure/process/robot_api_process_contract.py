#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "infrastructure"
    / "process"
    / "robot_api_process.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "infrastructure"
    / "process"
    / "robot_api_process.cpp"
)


def main() -> None:
    root = ROOT.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    compact_root = "".join(root.split())

    assert "int run_robot_api_server_process(int argc, char **argv);" in header
    assert "run_robot_api_server_process(argc,argv)" in compact_root

    for runtime_policy in (
        "class RobotApiServerNode",
        "ApplicationCompositionModule",
        "rclcpp::init(argc, argv)",
        "SingleThreadedExecutor",
        "is_transient_action_client_exception",
        "continuing after transient action client executor exception",
        "rclcpp::shutdown()",
    ):
        assert runtime_policy in source, runtime_policy
        assert runtime_policy not in root, runtime_policy

    assert root.count("int main(") == 1
    assert len(root.splitlines()) <= 10


if __name__ == "__main__":
    main()

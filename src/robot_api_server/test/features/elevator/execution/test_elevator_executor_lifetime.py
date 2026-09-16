"""Resource ordering contract; complements executable stop/race tests."""
import argparse
from pathlib import Path


def check(source: str) -> None:
    owner = source[source.index("class ElevatorRosRuntimePort::Implementation"):]
    worker = owner.index("std::unique_ptr<ElevatorRosExecutor> ros_worker_;")
    for dependent in (
        "std::condition_variable evidence_cv_;",
        "std::condition_variable floor_goal_cv_;",
        "local_odom_sub_;",
        "std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;",
    ):
        assert owner.index(dependent) < worker, (
            "worker must be destroyed before its notification targets and ROS resources: "
            + dependent
        )
    destructor = owner[owner.index("~Implementation()"):owner.index("template<typename Operation>")]
    operations = [destructor.index(operation) for operation in (
        "ros_worker_->request_stop()", "mode_keepalive_->stop()",
        "ros_worker_->stop()", "executor_->remove_node(node_)",
    )]
    assert operations == sorted(operations), "stop/wake/join/remove order changed"
    assert "detach(" not in owner
    assert "rclcpp::shutdown(" not in owner
    assert "spin_until_future_complete(" not in owner
    assert "MultiThreadedExecutor" not in owner


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-source", type=Path)
    args = parser.parse_args()
    source_path = args.runtime_source or (
        Path(__file__).resolve().parents[4]
        / "src/features/elevator/execution/elevator_ros_runtime_port.cpp")
    check(source_path.read_text(encoding="utf-8"))
    print("elevator executor ownership/lifetime contract: PASS")

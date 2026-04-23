#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PgoWrapperNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_pgo_mapping")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("frontend_artifact_dir", "mapping_result/frontend_result")
        self.declare_parameter("output_dir", "mapping_result")
        self.declare_parameter("loop_report_name", "loop_report.json")
        self.declare_parameter("local_config", "")
        self.status_pub = self.create_publisher(String, "/mapping/backend/status", 10)
        self._initialized = False
        self.create_timer(1.0, self.on_timer)

    def on_timer(self) -> None:
        if not self._initialized:
            output_dir = Path(self.get_parameter("output_dir").value)
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "optimized_map.pcd").write_text("# mock optimized map\n", encoding="utf-8")
            (output_dir / "optimized_trajectory.csv").write_text(
                "t,x,y,z,yaw\n0.0,0.0,0.0,0.0,0.0\n",
                encoding="utf-8",
            )
            (output_dir / "metadata.yaml").write_text("backend: robot_pgo_mapping\n", encoding="utf-8")
            loop_report = {"producer": "robot_pgo_mapping", "loops_detected": 0}
            (output_dir / self.get_parameter("loop_report_name").value).write_text(
                json.dumps(loop_report, indent=2), encoding="utf-8"
            )
            self._initialized = True
        self.status_pub.publish(String(data="mapping_result_ready"))


def main() -> None:
    rclpy.init()
    node = PgoWrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

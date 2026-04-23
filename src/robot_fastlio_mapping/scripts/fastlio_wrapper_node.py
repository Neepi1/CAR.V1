#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FastlioWrapperNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_fastlio_mapping")
        for name, value in {
            "mock_mode": True,
            "navigation_mode": False,
            "publish_tf": False,
            "upstream_points_topic": "/lidar_points",
            "upstream_imu_topic": "/lidar_imu",
            "upstream_sensor_frame": "lidar_link",
            "upstream_send_odom_base_tf": False,
            "frontend_pose_topic": "/mapping/frontend_pose",
            "frontend_status_topic": "/mapping/frontend/status",
            "artifact_dir": "mapping_result/frontend_result",
            "local_config": "",
        }.items():
            self.declare_parameter(name, value)
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter("frontend_pose_topic").value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter("frontend_status_topic").value, 10)
        self._initialized = False
        self._tick = 0
        self.create_timer(1.0, self.on_timer)

    def on_timer(self) -> None:
        if not self._initialized:
            artifact_dir = Path(self.get_parameter("artifact_dir").value)
            artifact_dir.mkdir(parents=True, exist_ok=True)
            result = {
                "producer": "robot_fastlio_mapping",
                "publish_tf": bool(self.get_parameter("publish_tf").value),
                "navigation_mode": bool(self.get_parameter("navigation_mode").value),
                "frontend_pose_topic": self.get_parameter("frontend_pose_topic").value,
                "upstream_points_topic": self.get_parameter("upstream_points_topic").value,
                "upstream_imu_topic": self.get_parameter("upstream_imu_topic").value,
                "upstream_sensor_frame": self.get_parameter("upstream_sensor_frame").value,
                "upstream_send_odom_base_tf": bool(self.get_parameter("upstream_send_odom_base_tf").value),
            }
            (artifact_dir / "frontend_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
            self._initialized = True
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = float(self._tick) * 0.05
        self.pose_pub.publish(pose)
        self._tick += 1
        self.status_pub.publish(String(data="frontend_result_ready"))


def main() -> None:
    rclpy.init()
    node = FastlioWrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FrontendPoseFromOdometry(Node):
    def __init__(self) -> None:
        super().__init__("frontend_pose_from_odometry")
        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/mapping/frontend_pose")
        self.declare_parameter("frame_override", "")
        self._frame_override = str(self.get_parameter("frame_override").value)
        self._pub = self.create_publisher(PoseStamped, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("input_topic").value), self._on_odom, 20)

    def _on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if self._frame_override:
            pose.header.frame_id = self._frame_override
        pose.pose = msg.pose.pose
        self._pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = FrontendPoseFromOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

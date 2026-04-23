#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ChassisBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_chassis_bridge")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_out_topic", "/platform/cmd_vel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.mock_mode = self.get_parameter("mock_mode").value
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("wheel_odom_topic").value, 10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_out_topic").value, 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Twist, self.get_parameter("cmd_vel_in_topic").value, self.on_cmd_vel, 10)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self.on_timer)

    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_pub.publish(msg)

    def on_timer(self) -> None:
        if self.mock_mode:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = self.get_parameter("odom_frame").value
            odom.child_frame_id = self.get_parameter("base_frame").value
            self.odom_pub.publish(odom)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = [DiagnosticStatus(level=DiagnosticStatus.OK, name="robot_chassis_bridge", message="mock_ok")]
        self.diagnostics_pub.publish(diag)


def main() -> None:
    rclpy.init()
    node = ChassisBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from __future__ import annotations

import copy
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "on")


class ScanFlipRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("scan_flip_republisher")
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # Default to pass-through. A full scan reversal changes the angular
        # winding order and mirrors the 2D map orientation, so keep it opt-in
        # for field debugging only.
        self._flip_scan = _env_bool("NJRH_SLAM2D_FLIP_SCAN", False)
        self._sub = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(LaserScan, "/scan", pub_qos)

    def _on_scan(self, msg: LaserScan) -> None:
        outgoing = copy.copy(msg)
        outgoing.header.stamp = self.get_clock().now().to_msg()
        if self._flip_scan:
            outgoing.ranges = list(reversed(msg.ranges))
            outgoing.intensities = list(reversed(msg.intensities))
            outgoing.angle_min = -float(msg.angle_max)
            outgoing.angle_max = -float(msg.angle_min)
            outgoing.angle_increment = float(msg.angle_increment)
        self._pub.publish(outgoing)


def main() -> None:
    rclpy.init()
    node = ScanFlipRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

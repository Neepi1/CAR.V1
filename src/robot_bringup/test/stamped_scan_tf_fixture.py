#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def make_transform(stamp, parent: str, child: str) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.rotation.w = 1.0
    return transform


def spin_for(node: Node, duration_sec: float) -> None:
    deadline = time.monotonic() + duration_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-topic", required=True)
    parser.add_argument("--tf-topic", required=True)
    parser.add_argument("--include-dynamic-tf", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Node("stamped_scan_tf_fixture")
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    tf_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    static_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    scan_pub = node.create_publisher(LaserScan, args.scan_topic, sensor_qos)
    tf_pub = node.create_publisher(TFMessage, args.tf_topic, tf_qos)
    static_pub = node.create_publisher(TFMessage, "/tf_static", static_qos)

    try:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                node.count_subscribers(args.scan_topic) >= 1
                and node.count_subscribers(args.tf_topic) >= 1
                and node.count_subscribers("/tf_static") >= 1
            ):
                break
        else:
            print("probe subscriptions did not become visible", file=sys.stderr)
            return 1

        static_stamp = node.get_clock().now().to_msg()
        static_pub.publish(
            TFMessage(
                transforms=[
                    make_transform(static_stamp, "base_link", "lidar_level_link")
                ]
            )
        )
        spin_for(node, 0.15)

        for _ in range(6):
            stamp = node.get_clock().now().to_msg()
            if args.include_dynamic_tf:
                tf_pub.publish(
                    TFMessage(
                        transforms=[
                            make_transform(stamp, "mapping_odom", "base_link")
                        ]
                    )
                )
                spin_for(node, 0.05)

            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = "lidar_level_link"
            scan.angle_min = -0.1
            scan.angle_max = 0.1
            scan.angle_increment = 0.1
            scan.range_min = 0.05
            scan.range_max = 10.0
            scan.ranges = [1.0, 2.0, 3.0]
            scan_pub.publish(scan)
            spin_for(node, 0.10)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

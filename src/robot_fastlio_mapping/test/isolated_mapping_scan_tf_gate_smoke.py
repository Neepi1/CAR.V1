#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


SCAN_IN = "/scan_tf_gate_test/scan_raw"
SCAN_OUT = "/scan_tf_gate_test/scan"
TF_TOPIC = "/scan_tf_gate_test/tf"
TF_STATIC_TOPIC = "/scan_tf_gate_test/tf_static"
STATUS_TOPIC = "/scan_tf_gate_test/status"


def spin_until(node: Node, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return bool(predicate())


def make_transform(stamp, parent: str, child: str) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.rotation.w = 1.0
    return transform


def main() -> int:
    rclpy.init()
    node = Node("isolated_mapping_scan_tf_gate_smoke")
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    reliable_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    static_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    status_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    scan_pub = node.create_publisher(LaserScan, SCAN_IN, sensor_qos)
    tf_pub = node.create_publisher(TFMessage, TF_TOPIC, sensor_qos)
    tf_static_pub = node.create_publisher(TFMessage, TF_STATIC_TOPIC, static_qos)
    outputs: list[LaserScan] = []
    statuses: list[dict] = []
    scan_sub = node.create_subscription(LaserScan, SCAN_OUT, outputs.append, reliable_qos)

    def on_status(msg: String) -> None:
        statuses.append(json.loads(msg.data))

    status_sub = node.create_subscription(String, STATUS_TOPIC, on_status, status_qos)

    try:
        graph_ready = spin_until(
            node,
            lambda: (
                node.count_subscribers(SCAN_IN) == 1
                and node.count_subscribers(TF_TOPIC) == 1
                and node.count_subscribers(TF_STATIC_TOPIC) == 1
                and node.count_publishers(SCAN_OUT) == 1
            ),
            5.0,
        )
        if not graph_ready:
            print("isolated gate graph did not become ready", file=sys.stderr)
            return 1

        stamp = (node.get_clock().now() - Duration(seconds=0.1)).to_msg()
        tf_static_pub.publish(TFMessage(transforms=[make_transform(stamp, "base_link", "lidar_level_link")]))
        spin_until(node, lambda: False, 0.1)

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

        spin_until(node, lambda: bool(outputs), 0.25)
        if outputs:
            print("gate released scan before original-stamp dynamic TF arrived", file=sys.stderr)
            return 1

        dynamic_tf = TFMessage(transforms=[make_transform(stamp, "mapping_odom", "base_link")])
        for _ in range(3):
            tf_pub.publish(dynamic_tf)
            rclpy.spin_once(node, timeout_sec=0.03)

        if not spin_until(node, lambda: bool(outputs), 2.0):
            print("gate did not release scan after matching TF arrived", file=sys.stderr)
            return 1
        outgoing = outputs[0]
        if outgoing.header.stamp != scan.header.stamp:
            print("gate changed LaserScan.header.stamp", file=sys.stderr)
            return 1
        if len(outgoing.ranges) != 3 or any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-6)
            for actual, expected in zip(outgoing.ranges, scan.ranges)
        ):
            print("gate changed LaserScan payload", file=sys.stderr)
            return 1

        if not spin_until(node, lambda: bool(statuses), 2.0):
            print("gate status was not published", file=sys.stderr)
            return 1
        status = statuses[-1]
        if not (
            status.get("preserve_stamp") is True
            and status.get("published_count", 0) >= 1
            and status.get("last_input_stamp_ns") == status.get("last_output_stamp_ns")
            and status.get("dropped_tf_timeout_count") == 0
            and status.get("dropped_queue_overflow_count") == 0
        ):
            print(f"unexpected gate status: {status}", file=sys.stderr)
            return 1

        print(
            "PASS scan held before TF, released after exact-stamp TF, stamp/payload preserved, "
            "no timeout or overflow"
        )
        return 0
    finally:
        node.destroy_subscription(scan_sub)
        node.destroy_subscription(status_sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC="${1:-5.0}"

python3 - "${DURATION_SEC}" <<'PY'
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class OdomProbe(Node):
    def __init__(self):
        super().__init__("wheel_odom_direction_probe")
        self.samples = {"/wheel/odom": [], "/local_state/odometry": []}
        for topic in self.samples:
            self.create_subscription(Odometry, topic, lambda msg, t=topic: self.samples[t].append(msg), 20)


def summarize(topic, samples):
    if not samples:
        print(f"{topic}: no samples")
        return
    first = samples[0]
    last = samples[-1]
    p0 = first.pose.pose.position
    p1 = last.pose.pose.position
    y0 = yaw_from_quat(first.pose.pose.orientation)
    y1 = yaw_from_quat(last.pose.pose.orientation)
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    forward = math.cos(y0) * dx + math.sin(y0) * dy
    left = -math.sin(y0) * dx + math.cos(y0) * dy
    twist = last.twist.twist
    print(f"{topic}: samples={len(samples)} frame={last.header.frame_id}->{last.child_frame_id}")
    print(f"  start: x={p0.x:.4f} y={p0.y:.4f} yaw={math.degrees(y0):.2f}deg")
    print(f"  end:   x={p1.x:.4f} y={p1.y:.4f} yaw={math.degrees(y1):.2f}deg")
    print(f"  delta_in_start_base: forward={forward:.4f}m left={left:.4f}m yaw={math.degrees(angle_diff(y1, y0)):.2f}deg")
    print(f"  last_twist: vx={twist.linear.x:.4f} vy={twist.linear.y:.4f} wz={twist.angular.z:.4f}")
    if abs(forward) < 0.02 and abs(left) < 0.02:
        print("  verdict: displacement too small; drive forward 0.10-0.30m while this script runs")
    elif forward > 0.0:
        print("  verdict: positive forward displacement")
    else:
        print("  verdict: NEGATIVE forward displacement; this odom source is front/back reversed")


def main():
    duration = float(sys.argv[1])
    rclpy.init()
    node = OdomProbe()
    deadline = node.get_clock().now().nanoseconds / 1e9 + duration
    while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    for topic, samples in node.samples.items():
        summarize(topic, samples)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
PY

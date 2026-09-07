#!/usr/bin/env python3
"""Lightweight mapping scan acceptance probe; never subscribes to PointCloud2."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def parse_status(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in text.split():
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value
    return result


def endpoint_name(endpoint: object) -> str:
    namespace = str(getattr(endpoint, "node_namespace", "") or "/").strip("/")
    name = str(getattr(endpoint, "node_name", "") or "<unknown>").strip("/")
    return "/".join(part for part in (namespace, name) if part)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--status-topic", default="/lidar/nav_cloud_preprocessor_status")
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--minimum-rate-hz", type=float, default=10.0)
    parser.add_argument("--expected-owner", default="pointcloud_to_laserscan")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = arguments()
    duration = max(1.0, args.duration_sec)
    scan_count = 0
    duplicate_stamp_count = 0
    regressed_stamp_count = 0
    last_stamp_ns: int | None = None
    latest_status: dict[str, str] = {}

    rclpy.init(args=None)
    node = rclpy.create_node(f"mapping_scan_throughput_probe_{os.getpid()}")
    scan_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    def on_scan(message: LaserScan) -> None:
        nonlocal scan_count, duplicate_stamp_count, regressed_stamp_count, last_stamp_ns
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if last_stamp_ns is not None:
            duplicate_stamp_count += int(stamp_ns == last_stamp_ns)
            regressed_stamp_count += int(stamp_ns < last_stamp_ns)
        last_stamp_ns = stamp_ns
        scan_count += 1

    def on_status(message: String) -> None:
        nonlocal latest_status
        latest_status = parse_status(message.data)

    subscriptions = [
        node.create_subscription(LaserScan, args.scan_topic, on_scan, scan_qos),
        node.create_subscription(String, args.status_topic, on_status, 10),
    ]
    del subscriptions

    start = time.monotonic()
    deadline = start + duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        elapsed = time.monotonic() - start
        publishers = list(node.get_publishers_info_by_topic(args.scan_topic))
        owners = [endpoint_name(item) for item in publishers]
        rate = scan_count / elapsed if elapsed > 0.0 else 0.0
        failures: list[str] = []
        if rate < args.minimum_rate_hz:
            failures.append(f"scan_rate_hz={rate:.3f} below {args.minimum_rate_hz:.3f}")
        if duplicate_stamp_count:
            failures.append(f"duplicate_stamp_count={duplicate_stamp_count}")
        if regressed_stamp_count:
            failures.append(f"regressed_stamp_count={regressed_stamp_count}")
        if len(publishers) != 1:
            failures.append(f"publisher_count={len(publishers)} owners={owners}")
        elif args.expected_owner not in owners[0]:
            failures.append(f"unexpected_owner={owners[0]}")

        report = {
            "passed": not failures,
            "duration_sec": duration,
            "elapsed_sec": elapsed,
            "scan_topic": args.scan_topic,
            "scan_count": scan_count,
            "scan_rate_hz": rate,
            "minimum_rate_hz": args.minimum_rate_hz,
            "duplicate_stamp_count": duplicate_stamp_count,
            "regressed_stamp_count": regressed_stamp_count,
            "publisher_count": len(publishers),
            "publisher_owners": owners,
            "preprocessor_status_topic": args.status_topic,
            "preprocessor_status": latest_status,
            "failures": failures,
        }
        rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered + "\n", encoding="utf-8")
        print(rendered)
        return 0 if not failures else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

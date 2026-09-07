·#!/usr/bin/env python3
"""One-shot ROS graph proof for the mapping-owned FAST-LIO cloud/odom pair."""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Iterable

import rclpy


def endpoint_owner(endpoint: object) -> tuple[str, str]:
    namespace = str(getattr(endpoint, "node_namespace", "") or "/")
    name = str(getattr(endpoint, "node_name", "") or "<unknown>")
    return namespace, name


def describe(endpoints: Iterable[object]) -> str:
    owners = ["/".join(part.strip("/") for part in endpoint_owner(item) if part.strip("/")) for item in endpoints]
    return ",".join(owners) if owners else "none"


def validate_pair(
    points_topic: str,
    points_publishers: list[object],
    odom_topic: str,
    odom_publishers: list[object],
) -> tuple[bool, str]:
    if len(points_publishers) != 1:
        return (
            False,
            f"{points_topic} must have exactly one publisher; "
            f"count={len(points_publishers)} owners={describe(points_publishers)}",
        )
    if len(odom_publishers) != 1:
        return (
            False,
            f"{odom_topic} must have exactly one publisher; "
            f"count={len(odom_publishers)} owners={describe(odom_publishers)}",
        )

    points_owner = endpoint_owner(points_publishers[0])
    odom_owner = endpoint_owner(odom_publishers[0])
    if points_owner != odom_owner:
        return (
            False,
            "mapping cloud and odom must come from the same ROS node; "
            f"points_owner={points_owner} odom_owner={odom_owner}",
        )
    return True, f"paired owner namespace={points_owner[0]} node={points_owner[1]}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Require exactly one publisher on each mapping FAST-LIO output and prove "
            "both endpoints belong to the same ROS node."
        )
    )
    parser.add_argument("points_topic")
    parser.add_argument("odom_topic")
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timeout_sec = max(0.2, args.timeout_sec)
    rclpy.init(args=None)
    node = rclpy.create_node(f"mapping_fastlio_pair_probe_{os.getpid()}")
    deadline = time.monotonic() + timeout_sec
    last_detail = "ROS graph has not converged"
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            points_publishers = list(node.get_publishers_info_by_topic(args.points_topic))
            odom_publishers = list(node.get_publishers_info_by_topic(args.odom_topic))
            valid, last_detail = validate_pair(
                args.points_topic,
                points_publishers,
                args.odom_topic,
                odom_publishers,
            )
            if valid:
                print(f"[mapping-fastlio-pair] PASS {last_detail}", file=sys.stderr)
                return 0
            time.sleep(0.05)

        print(f"[mapping-fastlio-pair] FAIL {last_detail}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Fail-fast continuity probe for the lightweight docking observation stream."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import DockTargetObservation


def _stamp_sec(message: DockTargetObservation) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


class ObservationProbe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("orbbec_docking_stream_probe")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.message_count = 0
        self.sensor_healthy_count = 0
        self.first_received_monotonic: float | None = None
        self.last_received_monotonic: float | None = None
        self.last_stamp_sec: float | None = None
        self.max_gap_sec = 0.0
        self.source = ""
        self.subscription = self.create_subscription(
            DockTargetObservation,
            topic,
            self._on_observation,
            qos,
        )

    def _on_observation(self, message: DockTargetObservation) -> None:
        now = time.monotonic()
        if self.first_received_monotonic is None:
            self.first_received_monotonic = now
        if self.last_received_monotonic is not None:
            self.max_gap_sec = max(self.max_gap_sec, now - self.last_received_monotonic)
        self.last_received_monotonic = now
        self.last_stamp_sec = _stamp_sec(message)
        self.message_count += 1
        self.sensor_healthy_count += int(message.sensor_healthy)
        self.source = message.source


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify that the Orbbec docking observation stream never stalls"
    )
    parser.add_argument("--topic", default="/dock/target_observation")
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=5.0)
    parser.add_argument("--max-gap-sec", type=float, default=1.0)
    parser.add_argument("--max-stamp-age-sec", type=float, default=1.0)
    parser.add_argument("--min-rate-hz", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration_sec <= 0.0 or args.startup_timeout_sec <= 0.0:
        raise SystemExit("duration and startup timeout must be positive")
    if args.max_gap_sec <= 0.0 or args.max_stamp_age_sec <= 0.0:
        raise SystemExit("freshness limits must be positive")

    rclpy.init()
    node = ObservationProbe(args.topic)
    started = time.monotonic()
    deadline = started + args.duration_sec
    failure = ""
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.first_received_monotonic is None:
                if now - started > args.startup_timeout_sec:
                    failure = "first_message_timeout"
                    break
                continue
            assert node.last_received_monotonic is not None
            live_gap = now - node.last_received_monotonic
            node.max_gap_sec = max(node.max_gap_sec, live_gap)
            if live_gap > args.max_gap_sec:
                failure = "stream_gap_exceeded"
                break
    except KeyboardInterrupt:
        failure = "interrupted"
    finally:
        elapsed = max(1.0e-9, time.monotonic() - started)
        ros_now = node.get_clock().now().nanoseconds * 1.0e-9
        stamp_age_sec = (
            ros_now - node.last_stamp_sec if node.last_stamp_sec is not None else math.inf
        )
        rate_hz = node.message_count / elapsed
        if not failure and rate_hz < args.min_rate_hz:
            failure = "rate_below_minimum"
        if not failure and not (-0.25 <= stamp_age_sec <= args.max_stamp_age_sec):
            failure = "stamp_not_fresh"
        summary = {
            "ok": not failure,
            "failure": failure,
            "topic": args.topic,
            "elapsed_sec": round(elapsed, 3),
            "message_count": node.message_count,
            "rate_hz": round(rate_hz, 3),
            "max_gap_sec": round(node.max_gap_sec, 3),
            "stamp_age_sec": round(stamp_age_sec, 3) if math.isfinite(stamp_age_sec) else None,
            "sensor_healthy_ratio": round(
                node.sensor_healthy_count / node.message_count, 4
            ) if node.message_count else 0.0,
            "source": node.source,
        }
        print(json.dumps(summary, ensure_ascii=True, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if not failure else 1


if __name__ == "__main__":
    raise SystemExit(main())

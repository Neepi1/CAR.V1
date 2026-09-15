#!/usr/bin/env python3
"""Read final startup service/bridge evidence on one short-lived ROS node; no RPCs."""

import argparse
import json
import math
import os
import re
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String


def floor_handoff_requested():
    from floor_startup_handoff import request_pending
    return request_pending(os.environ.get(
        "NJRH_FLOOR_STARTUP_HANDOFF_FILE", "/tmp/njrh_floor_startup_handoff.json"))


def log_step(phase, started, result):
    print(f"[runtime-overlay] CONTEXT_STEP phase={phase} "
          f"elapsed_sec={time.monotonic() - started:.3f} result={result}",
          file=sys.stderr, flush=True)


def observe(node, service_wait_sec, bridge_wait_sec, minimum_sequence):
    started = time.monotonic()
    deadline = started + service_wait_sec
    service_ready = False
    while rclpy.ok() and time.monotonic() < deadline:
        if floor_handoff_requested():
            return 20
        # Same existence criterion as the original native service probe.
        service_ready = any(name == "/global_localization/trigger" and types
                            for name, types in node.get_service_names_and_types())
        if service_ready:
            break
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
    log_step("wrapper_service", started, "ready" if service_ready else "missing")
    if not service_ready:
        print("service_ready: false", flush=True)
        return 2
    print("service_ready: true", flush=True)
    if floor_handoff_requested():
        return 20

    sequence = None

    def on_status(message):
        nonlocal sequence
        try:
            status = json.loads(message.data)
            if isinstance(status, str):
                status = json.loads(status)
            candidate = status.get("last_explicit_relocalization_sequence")
            # Preserve the former shell field extraction's numeric-string
            # compatibility, but never interpret booleans/floats as sequences.
            if isinstance(candidate, str) and re.fullmatch(r"0|[1-9][0-9]*", candidate):
                candidate = int(candidate)
            has_map_to_odom = status.get("has_map_to_odom")
            if (type(candidate) is int and candidate > minimum_sequence
                    and (has_map_to_odom is True or has_map_to_odom == "true")
                    and status.get("map_to_odom_publisher_owner") == "robot_localization_bridge"):
                sequence = candidate
        except (ValueError, TypeError, AttributeError):
            pass

    # New volatile reader after service discovery: no pre-service DDS queue or
    # previous startup's cached status can prove the final context.
    started = time.monotonic()
    subscription = node.create_subscription(
        String, "/localization/bridge_status", on_status,
        # Match ros2 topic echo's sensor_data preset plus the original explicit
        # reliable/volatile overrides; retain its KEEP_LAST depth of five.
        QoSProfile(depth=qos_profile_sensor_data.depth, history=qos_profile_sensor_data.history,
                   reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE))
    deadline = started + bridge_wait_sec
    while rclpy.ok() and sequence is None and time.monotonic() < deadline:
        if floor_handoff_requested():
            return 20
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
    if floor_handoff_requested():
        return 20
    log_step("bridge_sequence", started, "ready" if sequence is not None else "missing")
    if sequence is None:
        return 3
    print(f"explicit_sequence: {sequence}", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service-wait-sec", type=float, default=10.0)
    parser.add_argument("--bridge-wait-sec", type=float, default=15.0)
    parser.add_argument("--minimum-sequence", type=int, required=True)
    args = parser.parse_args()
    if (not all(math.isfinite(value) and value >= 0 for value in
                (args.service_wait_sec, args.bridge_wait_sec)) or args.minimum_sequence < -1):
        parser.error("wait budgets must be finite and non-negative; minimum sequence must be >= -1")
    node = None
    try:
        if floor_handoff_requested():
            return 20
        started = time.monotonic()
        rclpy.init(args=None)
        node = rclpy.create_node("startup_context_observer", enable_rosout=False,
                                 start_parameter_services=False)
        log_step("observer_start", started, "ready")
        return observe(node, args.service_wait_sec, args.bridge_wait_sec, args.minimum_sequence)
    except Exception as exc:
        print(f"[runtime-overlay] startup context observation failed: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())

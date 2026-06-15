#!/usr/bin/env python3
"""Probe AMCL no-motion update without racing the one-shot /amcl_pose."""

import argparse
import json
import sys
from typing import Any, Dict

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty


EXIT_OK = 0
EXIT_NO_POSE = 10
EXIT_SERVICE_UNAVAILABLE = 20
EXIT_SERVICE_FAILED = 21
EXIT_RCLPY_ERROR = 30


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-topic", default="/amcl_pose")
    parser.add_argument("--service", default="/request_nomotion_update")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--pre-subscribe-warmup-sec", type=float, default=0.2)
    parser.add_argument("--require-header-fresh", default="false")
    parser.add_argument("--max-header-age-sec", type=float, default=1.0)
    args = parser.parse_args()

    result: Dict[str, Any] = {
        "service_available": False,
        "service_call_ok": False,
        "pose_received": False,
        "pose_count": 0,
        "pose_received_after_request_start": False,
        "pose_receive_delay_sec": None,
        "pose_header_age_at_receive_sec": None,
        "request_start_time": None,
        "request_end_time": None,
        "error": "",
    }
    require_header_fresh = str(args.require_header_fresh).lower() in {"1", "true", "yes", "on"}

    try:
        rclpy.init()
        node = rclpy.create_node("amcl_nomotion_update_probe")

        def now_sec() -> float:
            return node.get_clock().now().nanoseconds * 1.0e-9

        def on_pose(msg: PoseWithCovarianceStamped) -> None:
            receive_time = now_sec()
            result["pose_count"] += 1
            header_age = receive_time - stamp_to_sec(msg.header.stamp)
            if result["request_start_time"] is not None and receive_time >= result["request_start_time"]:
                if not result["pose_received"]:
                    result["pose_received"] = True
                    result["pose_received_after_request_start"] = True
                    result["pose_receive_delay_sec"] = receive_time - result["request_start_time"]
                    result["pose_header_age_at_receive_sec"] = header_age

        node.create_subscription(PoseWithCovarianceStamped, args.pose_topic, on_pose, 10)

        warmup_deadline = now_sec() + max(0.0, args.pre_subscribe_warmup_sec)
        while rclpy.ok() and now_sec() < warmup_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

        client = node.create_client(Empty, args.service)
        if not client.wait_for_service(timeout_sec=max(0.0, args.timeout_sec)):
            result["error"] = "service_unavailable"
            print(json.dumps(result, sort_keys=True))
            return EXIT_SERVICE_UNAVAILABLE
        result["service_available"] = True

        result["request_start_time"] = now_sec()
        future = client.call_async(Empty.Request())
        deadline = result["request_start_time"] + max(0.0, args.timeout_sec)
        service_error = ""
        while rclpy.ok() and now_sec() <= deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if future.done() and not result["service_call_ok"]:
                try:
                    future.result()
                    result["service_call_ok"] = True
                    result["request_end_time"] = now_sec()
                except Exception as exc:  # noqa: BLE001
                    service_error = str(exc)
                    result["error"] = f"service_call_failed: {service_error}"
                    print(json.dumps(result, sort_keys=True))
                    return EXIT_SERVICE_FAILED
            if result["service_call_ok"] and result["pose_received"]:
                break

        if not result["service_call_ok"]:
            result["error"] = "service_call_timeout"
            print(json.dumps(result, sort_keys=True))
            return EXIT_SERVICE_FAILED

        if not result["pose_received"]:
            result["error"] = "pose_not_received"
            print(json.dumps(result, sort_keys=True))
            return EXIT_NO_POSE

        if require_header_fresh:
            age = result["pose_header_age_at_receive_sec"]
            if age is None or age < 0.0 or age > args.max_header_age_sec:
                result["error"] = "pose_header_not_fresh"
                print(json.dumps(result, sort_keys=True))
                return EXIT_NO_POSE

        print(json.dumps(result, sort_keys=True))
        return EXIT_OK
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"rclpy_error: {exc}"
        print(json.dumps(result, sort_keys=True))
        return EXIT_RCLPY_ERROR
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Small ROS client for the startup global localization trigger service."""

import argparse
import sys

import rclpy
from robot_interfaces.srv import TriggerLocalization


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", default="/global_localization/trigger")
    parser.add_argument("--reason", required=True)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    args = parser.parse_args()

    timeout_sec = max(float(args.timeout_sec), 1.0)
    rclpy.init(args=None)
    node = rclpy.create_node("startup_global_localization_trigger_client")
    try:
        client = node.create_client(TriggerLocalization, args.service)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            print(f"accepted: false")
            print(f"message: failure_code=SERVICE_UNAVAILABLE service is not available: {args.service}")
            return 2

        request = TriggerLocalization.Request()
        request.reason = args.reason
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            print("accepted: false")
            print(
                "message: failure_code=GLOBAL_LOCALIZATION_TRIGGER_TIMEOUT "
                f"request did not complete within {timeout_sec:.1f}s"
            )
            return 124

        response = future.result()
        print(f"accepted: {'true' if response.accepted else 'false'}")
        print(f"message: {response.message}")
        return 0
    except Exception as exc:
        print("accepted: false")
        print(f"message: failure_code=GLOBAL_LOCALIZATION_TRIGGER_CLIENT_ERROR {exc}")
        return 1
    finally:
        try:
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

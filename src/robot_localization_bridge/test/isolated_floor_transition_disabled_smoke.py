#!/usr/bin/env python3
"""Verify the production-default bridge floor-transition gate is fail closed."""

import json

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import BeginFloorTransition


def main() -> None:
    rclpy.init()
    node = Node("p6_floor_transition_disabled_smoke")
    try:
        client = node.create_client(
            BeginFloorTransition,
            "/robot_localization_bridge/begin_floor_transition",
        )
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("floor-transition service unavailable")

        request = BeginFloorTransition.Request()
        request.transaction_id = "floor-disabled-smoke"
        request.building_id = "B1"
        request.floor_id = "F2"
        request.map_id = "map_f2"
        request.asset_epoch = 1
        request.asset_digest = (
            "sha256:"
            "0123456789abcdef0123456789abcdef"
            "0123456789abcdef0123456789abcdef"
        )
        request.operation = BeginFloorTransition.Request.OP_BEGIN
        request.command_sequence = 1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("floor-transition service timed out")
        response = future.result()
        if response.success:
            raise AssertionError(f"default-disabled bridge accepted BEGIN: {response}")
        if "LIVE_FLOOR_TRANSITION_DISABLED" not in response.message:
            raise AssertionError(f"unexpected disabled-gate response: {response}")
        if not response.runtime_context_valid:
            raise AssertionError(f"disabled BEGIN invalidated runtime context: {response}")

        print(
            json.dumps(
                {
                    "default_begin_rejected": True,
                    "runtime_context_remained_valid": True,
                    "message": response.message,
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

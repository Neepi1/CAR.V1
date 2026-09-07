#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import LocalizationHealth
from robot_interfaces.srv import BeginFloorTransition, SetCorrectionPause


class FloorTransitionSmoke(Node):
    def __init__(self) -> None:
        super().__init__("p6_floor_transition_smoke")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.health: LocalizationHealth | None = None
        self.create_subscription(
            LocalizationHealth,
            "/localization/floor_health",
            self._on_health,
            qos,
        )
        self.pause_client = self.create_client(
            SetCorrectionPause,
            "/robot_localization_bridge/set_correction_pause_lease",
        )
        self.transition_client = self.create_client(
            BeginFloorTransition,
            "/robot_localization_bridge/begin_floor_transition",
        )

    def _on_health(self, health: LocalizationHealth) -> None:
        self.health = health

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service call timed out: {client.srv_name}")
        return future.result()

    def wait_health(self, predicate, timeout_sec: float = 3.0) -> LocalizationHealth:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.health is not None and predicate(self.health):
                return self.health
        raise AssertionError(f"timed out waiting for floor health: {self.health}")


def transition_request(
    operation: int, command_sequence: int
) -> BeginFloorTransition.Request:
    request = BeginFloorTransition.Request()
    request.transaction_id = "floor-smoke-17"
    request.building_id = "B10"
    request.floor_id = "F2"
    request.map_id = "map_f2"
    request.asset_epoch = 23
    request.asset_digest = (
        "sha256:"
        "0123456789abcdef0123456789abcdef"
        "0123456789abcdef0123456789abcdef"
    )
    request.source_building_id = "B10"
    request.source_floor_id = "F1"
    request.source_map_id = "map_f1"
    request.source_asset_epoch = 22
    request.source_asset_digest = (
        "sha256:"
        "abcdef0123456789abcdef0123456789"
        "abcdef0123456789abcdef0123456789"
    )
    request.operation = operation
    request.command_sequence = command_sequence
    return request


def main() -> None:
    rclpy.init()
    node = FloorTransitionSmoke()
    try:
        without_pause = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_BEGIN, 1),
        )
        if without_pause.success or not without_pause.runtime_context_valid:
            raise AssertionError(f"BEGIN bypassed pause ownership: {without_pause}")

        acquire = SetCorrectionPause.Request()
        acquire.owner = "robot_floor_manager"
        acquire.transaction_id = "floor-smoke-17"
        acquire.reason = "floor_transition"
        acquire.operation = SetCorrectionPause.Request.OP_ACQUIRE
        acquire.command_sequence = 1
        acquired = node.call(node.pause_client, acquire)
        if (
            not acquired.success
            or acquired.applied_sequence != acquire.command_sequence
            or not acquired.state.paused
        ):
            raise AssertionError(f"floor pause acquire failed: {acquired}")

        begun = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_BEGIN, 2),
        )
        if begun.success or not begun.runtime_context_valid:
            raise AssertionError(
                f"unseeded bridge accepted BEGIN or invalidated context: {begun}"
            )
        if "PREMUTATION_UNPROVEN" not in begun.message:
            raise AssertionError(f"unexpected unseeded-source rejection: {begun}")

        release = SetCorrectionPause.Request()
        release.owner = acquire.owner
        release.transaction_id = acquire.transaction_id
        release.reason = acquire.reason
        release.operation = SetCorrectionPause.Request.OP_RELEASE
        release.command_sequence = 2
        released = node.call(node.pause_client, release)
        if (
            not released.success
            or released.applied_sequence != release.command_sequence
            or released.state.paused
        ):
            raise AssertionError(f"floor pause release failed: {released}")

        print(
            json.dumps(
                {
                    "begin_without_pause_rejected": True,
                    "unseeded_source_begin_rejected": True,
                    "runtime_context_remained_valid": begun.runtime_context_valid,
                    "final_pause_released": not released.state.paused,
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

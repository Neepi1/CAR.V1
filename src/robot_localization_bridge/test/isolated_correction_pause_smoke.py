#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import CorrectionPauseState
from robot_interfaces.srv import SetCorrectionPause
from std_srvs.srv import SetBool


class PauseSmoke(Node):
    def __init__(self) -> None:
        super().__init__("p6_correction_pause_smoke")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state: CorrectionPauseState | None = None
        self.create_subscription(
            CorrectionPauseState,
            "/localization/correction_pause_state",
            self._on_state,
            qos,
        )
        self.lease_client = self.create_client(
            SetCorrectionPause,
            "/robot_localization_bridge/set_correction_pause_lease",
        )
        self.legacy_client = self.create_client(
            SetBool,
            "/robot_localization_bridge/set_correction_paused",
        )

    def _on_state(self, state: CorrectionPauseState) -> None:
        self.state = state

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service call timed out: {client.srv_name}")
        return future.result()


def main() -> None:
    rclpy.init()
    node = PauseSmoke()
    try:
        acquire = SetCorrectionPause.Request()
        acquire.owner = "robot_elevator_manager"
        acquire.transaction_id = "elevator-isolated"
        acquire.reason = "elevator_ride"
        acquire.operation = SetCorrectionPause.Request.OP_ACQUIRE
        acquired = node.call(node.lease_client, acquire)
        if not acquired.success or not acquired.state.paused:
            raise AssertionError(f"lease acquire failed: {acquired}")

        for owner, transaction_id in (
            ("legacy_set_bool", "reserved-owner-probe"),
            ("reserved-transaction-probe", "legacy_set_bool"),
        ):
            reserved = SetCorrectionPause.Request()
            reserved.owner = owner
            reserved.transaction_id = transaction_id
            reserved.reason = "reserved_identifier_probe"
            reserved.operation = SetCorrectionPause.Request.OP_ACQUIRE
            rejected = node.call(node.lease_client, reserved)
            if rejected.success or not rejected.state.paused:
                raise AssertionError(
                    f"reserved legacy identifier was accepted: {rejected}"
                )

        legacy_release = SetBool.Request()
        legacy_release.data = False
        legacy = node.call(node.legacy_client, legacy_release)
        if not legacy.success:
            raise AssertionError(f"legacy release failed: {legacy}")
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.state is None or not node.state.paused:
            raise AssertionError("legacy false released another owner's pause")
        if "robot_elevator_manager:elevator-isolated" not in node.state.lease_keys:
            raise AssertionError(f"elevator pause lease disappeared: {node.state}")

        release = SetCorrectionPause.Request()
        release.owner = acquire.owner
        release.transaction_id = acquire.transaction_id
        release.reason = acquire.reason
        release.operation = SetCorrectionPause.Request.OP_RELEASE
        released = node.call(node.lease_client, release)
        if not released.success or released.state.paused:
            raise AssertionError(f"exact release failed: {released}")

        print(
            json.dumps(
                {
                    "acquire_generation": acquired.state.generation,
                    "reserved_legacy_identifiers_rejected": True,
                    "legacy_false_preserved_elevator_lease": True,
                    "release_generation": released.state.generation,
                    "final_paused": released.state.paused,
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

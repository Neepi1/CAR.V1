#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import LocalizationHealth
from robot_interfaces.srv import BeginFloorTransition, SetCorrectionPause
from std_srvs.srv import Trigger


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
        self.force_client = self.create_client(
            Trigger,
            "/robot_localization_bridge/force_accept_next_localization",
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


def transition_request(operation: int) -> BeginFloorTransition.Request:
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
    request.operation = operation
    return request


def main() -> None:
    rclpy.init()
    node = FloorTransitionSmoke()
    try:
        without_pause = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_BEGIN),
        )
        if without_pause.success or not without_pause.runtime_context_valid:
            raise AssertionError(f"BEGIN bypassed pause ownership: {without_pause}")

        acquire = SetCorrectionPause.Request()
        acquire.owner = "robot_floor_manager"
        acquire.transaction_id = "floor-smoke-17"
        acquire.reason = "floor_transition"
        acquire.operation = SetCorrectionPause.Request.OP_ACQUIRE
        acquired = node.call(node.pause_client, acquire)
        if not acquired.success or not acquired.state.paused:
            raise AssertionError(f"floor pause acquire failed: {acquired}")

        begun = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_BEGIN),
        )
        if not begun.success or begun.runtime_context_valid or begun.safe_for_goal_start:
            raise AssertionError(f"BEGIN did not invalidate runtime context: {begun}")
        begin_health = node.wait_health(
            lambda health: health.transition_active
            and not health.runtime_context_valid
            and health.map_id == "map_f2"
        )
        if begin_health.localizer_ready or begin_health.tf_unique:
            raise AssertionError(f"health overstated unproven readiness: {begin_health}")

        aborted = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_ABORT),
        )
        if not aborted.success or aborted.runtime_context_valid or aborted.safe_for_goal_start:
            raise AssertionError(f"ABORT did not retain invalid context: {aborted}")
        failed_health = node.wait_health(
            lambda health: not health.transition_active
            and not health.runtime_context_valid
            and health.detail == "FAILED_LOCKED"
        )

        force = node.call(node.force_client, Trigger.Request())
        if force.success:
            raise AssertionError("FAILED_LOCKED bridge accepted force-localization arm")

        release = SetCorrectionPause.Request()
        release.owner = acquire.owner
        release.transaction_id = acquire.transaction_id
        release.reason = acquire.reason
        release.operation = SetCorrectionPause.Request.OP_RELEASE
        released = node.call(node.pause_client, release)
        if not released.success or released.state.paused:
            raise AssertionError(f"floor pause release failed: {released}")

        print(
            json.dumps(
                {
                    "begin_without_pause_rejected": True,
                    "begin_context_invalid": not begin_health.runtime_context_valid,
                    "localizer_ready_overstated": begin_health.localizer_ready,
                    "tf_unique_overstated": begin_health.tf_unique,
                    "abort_detail": failed_health.detail,
                    "force_arm_after_abort_rejected": True,
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

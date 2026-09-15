#!/usr/bin/env python3

import json
import os
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
            Trigger, "/robot_localization_bridge/force_accept_next_localization"
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
    # Match FloorManager when no healthy source localization has been observed.
    request.operation = operation
    request.command_sequence = command_sequence
    return request


def main() -> None:
    if (
        not os.environ.get("ROS_DOMAIN_ID", "").isdigit()
        or int(os.environ["ROS_DOMAIN_ID"]) == 0
        or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
    ):
        raise RuntimeError("isolated smoke requires a non-zero ROS domain and localhost-only")
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
        if not begun.success or begun.runtime_context_valid:
            raise AssertionError(f"unlocalized source blocked target BEGIN: {begun}")

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

        unproven_commit = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_COMMIT, 3),
        )
        if unproven_commit.success or unproven_commit.runtime_context_valid:
            raise AssertionError(f"target committed without localization: {unproven_commit}")
        health = node.wait_health(lambda value: value.transition_active)
        if health.runtime_context_valid or health.bridge_ready:
            raise AssertionError(f"BEGIN invented target readiness: {health}")

        aborted = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_ABORT, 4),
        )
        if not aborted.success or aborted.runtime_context_valid or aborted.safe_for_goal_start:
            raise AssertionError(f"ABORT did not preserve invalid localization: {aborted}")
        ended = node.wait_health(
            lambda value: not value.transition_active
            and value.detail.startswith("ABORTED_CONTEXT_INVALID")
        )
        if ended.runtime_context_valid:
            raise AssertionError(f"ABORT invented ready localization: {ended}")
        force = node.call(node.force_client, Trigger.Request())
        if force.success or "FLOOR_CONTEXT_INVALID" not in force.message:
            raise AssertionError(f"force-accept bypassed invalid floor context: {force}")

        replacement = transition_request(BeginFloorTransition.Request.OP_BEGIN, 1)
        replacement.transaction_id = "floor-smoke-new-after-abort"
        acquire.transaction_id = replacement.transaction_id
        acquire.command_sequence = 1
        if not node.call(node.pause_client, acquire).success:
            raise AssertionError("new transaction could not acquire its own floor pause")
        restarted = node.call(node.transition_client, replacement)
        if not restarted.success or restarted.runtime_context_valid:
            raise AssertionError(f"new switch blocked by historical failure: {restarted}")
        late_abort = node.call(
            node.transition_client,
            transition_request(BeginFloorTransition.Request.OP_ABORT, 100),
        )
        if late_abort.success:
            raise AssertionError("old transaction ABORT was allowed to end the new switch")
        node.wait_health(lambda value: value.transition_active)
        replacement.operation = BeginFloorTransition.Request.OP_ABORT
        replacement.command_sequence = 2
        if not node.call(node.transition_client, replacement).success:
            raise AssertionError("new transaction could not end itself")
        release.transaction_id = replacement.transaction_id
        release.command_sequence = 2
        released = node.call(node.pause_client, release)
        if not released.success or released.state.paused:
            raise AssertionError("test left its floor pause held")

        print(
            json.dumps(
                {
                    "begin_without_pause_rejected": True,
                    "unlocalized_source_begin_accepted": True,
                    "unlocalized_target_commit_rejected": True,
                    "runtime_context_remained_invalid": not begun.runtime_context_valid,
                    "final_pause_released": not released.state.paused,
                    "aborted_transaction_ended_without_permanent_lock": True,
                    "invalid_floor_context_force_accept_rejected": True,
                    "new_transaction_begin_accepted_after_abort": True,
                    "old_transaction_cannot_abort_replacement": True,
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

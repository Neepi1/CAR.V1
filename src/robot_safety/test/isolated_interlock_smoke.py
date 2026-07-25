#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MotionInterlockState, OperatingModeState
from robot_interfaces.srv import SetExecutionLease, SetMotionHold


class InterlockSmoke(Node):
    def __init__(self) -> None:
        super().__init__("p6_interlock_smoke")
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.outputs: list[float] = []
        self.output_twists: list[Twist] = []
        self.state: MotionInterlockState | None = None
        self.input_pub = self.create_publisher(Twist, "/p6_test/input", 1)
        self.api_input_pub = self.create_publisher(Twist, "/p6_test/api", 1)
        self.docking_input_pub = self.create_publisher(
            Twist, "/p6_test/docking", 1
        )
        self.mode_pub = self.create_publisher(
            OperatingModeState, "/p6_test/mode", state_qos
        )
        self.mode_state: OperatingModeState | None = None
        self.create_subscription(Twist, "/p6_test/output", self._on_output, 10)
        self.create_subscription(
            MotionInterlockState,
            "/safety/motion_interlock_state",
            self._on_state,
            state_qos,
        )
        self.hold_client = self.create_client(SetMotionHold, "/safety/set_motion_hold")
        self.execution_client = self.create_client(
            SetExecutionLease, "/safety/set_execution_lease"
        )

    def _on_output(self, message: Twist) -> None:
        self.outputs.append(float(message.linear.x))
        self.output_twists.append(message)

    def _on_state(self, message: MotionInterlockState) -> None:
        self.state = message

    def wait_ready(self) -> None:
        if not self.hold_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("motion hold service unavailable")
        if not self.execution_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("execution lease service unavailable")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("interlock service call timed out")
        return future.result()

    def pump(
        self,
        duration_sec: float,
        linear_x: float | None = None,
        use_api_source: bool = False,
        use_docking_source: bool = False,
    ) -> None:
        deadline = time.monotonic() + duration_sec
        next_publish = 0.0
        next_mode_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if self.mode_state is not None and now >= next_mode_publish:
                self.mode_pub.publish(self.mode_state)
                next_mode_publish = now + 0.10
            if linear_x is not None and now >= next_publish:
                command = Twist()
                command.linear.x = linear_x
                if use_docking_source:
                    publisher = self.docking_input_pub
                elif use_api_source:
                    publisher = self.api_input_pub
                else:
                    publisher = self.input_pub
                publisher.publish(command)
                next_publish = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.02)

    def pump_twist(
        self,
        duration_sec: float,
        command: Twist,
        source: str,
    ) -> None:
        publishers = {
            "normal": self.input_pub,
            "api": self.api_input_pub,
            "docking": self.docking_input_pub,
        }
        publisher = publishers[source]
        deadline = time.monotonic() + duration_sec
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                publisher.publish(command)
                next_publish = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_output(self, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while not self.output_twists and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not self.output_twists:
            raise AssertionError(f"timed out waiting for {label}")


def assert_finite_zero(messages: list[Twist], label: str) -> None:
    if not messages:
        raise AssertionError(f"{label} produced no output")
    for message in messages:
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if any(not math.isfinite(value) or abs(value) > 1.0e-6 for value in values):
            raise AssertionError(f"{label} leaked invalid/nonzero Twist: {values}")


def main() -> None:
    rclpy.init()
    node = InterlockSmoke()
    summary: dict[str, object] = {}
    try:
        node.wait_ready()

        acquire = SetMotionHold.Request()
        acquire.owner = "robot_elevator_manager"
        acquire.transaction_id = "tx-isolated"
        acquire.reason = "isolated_gate_test"
        acquire.operation = SetMotionHold.Request.OP_ACQUIRE
        acquired = node.call(node.hold_client, acquire)
        if not acquired.success or not acquired.state.motion_blocked:
            raise AssertionError(f"hold acquisition failed: {acquired}")

        node.outputs.clear()
        node.pump(0.6, linear_x=0.2)
        if not node.outputs or any(abs(value) > 1.0e-6 for value in node.outputs):
            raise AssertionError(f"hold leaked nonzero output: {node.outputs}")
        summary["held_output_max_abs"] = max(abs(value) for value in node.outputs)

        release = SetMotionHold.Request()
        release.owner = acquire.owner
        release.transaction_id = acquire.transaction_id
        release.reason = acquire.reason
        release.operation = SetMotionHold.Request.OP_RELEASE
        released = node.call(node.hold_client, release)
        if not released.success or released.state.motion_blocked:
            raise AssertionError(f"hold release failed: {released}")

        node.pump(0.35)
        node.outputs.clear()
        node.pump(0.7, linear_x=0.2)
        passed_values = [value for value in node.outputs if math.isclose(value, 0.2, abs_tol=1.0e-6)]
        if not passed_values:
            raise AssertionError(f"fresh post-release command did not pass: {node.outputs}")
        summary["post_release_nonzero_seen"] = True

        node.pump(0.35)
        node.outputs.clear()
        node.pump(0.4, linear_x=0.05, use_docking_source=True)
        if not any(math.isclose(value, 0.05, abs_tol=1.0e-6) for value in node.outputs):
            raise AssertionError(
                f"isolated docking source did not pass forward control: {node.outputs}"
            )
        node.outputs.clear()
        node.pump(0.4, linear_x=-0.05, use_docking_source=True)
        if not node.outputs or any(abs(value) > 1.0e-6 for value in node.outputs):
            raise AssertionError(
                "global allow_reverse bypassed the docking-specific permit: "
                f"{node.outputs}"
            )
        summary["docking_global_reverse_blocked"] = True

        invalid_commands: list[tuple[str, Twist]] = []
        normal_nan = Twist()
        normal_nan.linear.x = float("nan")
        invalid_commands.append(("normal", normal_nan))
        api_inf = Twist()
        api_inf.angular.z = float("inf")
        invalid_commands.append(("api", api_inf))
        docking_nan = Twist()
        docking_nan.linear.y = float("nan")
        invalid_commands.append(("docking", docking_nan))
        for source, command in invalid_commands:
            node.pump(0.4)
            node.output_twists.clear()
            node.pump_twist(0.35, command, source)
            assert_finite_zero(node.output_twists, f"{source} non-finite command")
        summary["non_finite_all_sources_blocked"] = True

        mode_state = OperatingModeState()
        mode_state.generation = 1
        mode_state.mode = "DOORWAY"
        mode_state.owner = "robot_elevator_manager"
        mode_state.mission_id = "mission-isolated"
        mode_state.lease_id = "mode-isolated"
        mode_state.lease_remaining.sec = 5
        mode_state.lease_active = True
        mode_state.transition_reason = "isolated_test"
        node.mode_state = mode_state
        node.pump(0.2)
        node.mode_state = None
        node.spin_for(0.1)
        node.output_twists.clear()
        node.wait_for_output(1.3, "low-rate safety timer")
        assert_finite_zero(
            node.output_twists,
            "low-rate pre-lease timer synchronization",
        )
        node.mode_state = mode_state
        node.pump(0.05)
        node.output_twists.clear()

        lease = SetExecutionLease.Request()
        lease.owner = "robot_elevator_manager"
        lease.mission_id = "mission-isolated"
        lease.transaction_id = "tx-isolated"
        lease.lease_id = "execution-isolated"
        lease.lease_duration.sec = 3
        lease.lease_duration.nanosec = 0
        lease.recovery = False
        lease.operation = SetExecutionLease.Request.OP_SET
        leased = node.call(node.execution_client, lease)
        if not leased.success or not leased.state.execution_lease_active:
            raise AssertionError(f"execution lease acquisition failed: {leased}")
        node.spin_for(0.15)
        assert_finite_zero(
            node.output_twists,
            "execution transition callback",
        )
        summary["execution_transition_immediate_zero"] = True

        node.outputs.clear()
        node.pump(0.3, linear_x=0.25, use_api_source=True)
        if not node.outputs or any(abs(value) > 1.0e-6 for value in node.outputs):
            raise AssertionError(
                f"healthy execution session leaked API command: {node.outputs}"
            )
        if (
            node.state is None
            or not node.state.execution_mode_contract_valid
            or not node.state.normal_source_only
            or node.state.interlock_effective_motion_blocked
        ):
            raise AssertionError(
                f"healthy execution state diagnostics inconsistent: {node.state}"
            )
        summary["execution_api_output_max_abs"] = max(
            abs(value) for value in node.outputs
        )

        node.pump(0.35)
        node.outputs.clear()
        node.pump(0.3, linear_x=0.2)
        if not any(math.isclose(value, 0.2, abs_tol=1.0e-6) for value in node.outputs):
            raise AssertionError(
                f"healthy execution session blocked normal Nav2 source: {node.outputs}"
            )
        summary["execution_normal_source_passed"] = True

        node.outputs.clear()
        node.pump(0.3, linear_x=-0.05)
        if not node.outputs or any(abs(value) > 1.0e-6 for value in node.outputs):
            raise AssertionError(
                "execution session accepted reverse through the global "
                f"allow_reverse setting: {node.outputs}"
            )
        summary["execution_global_reverse_blocked"] = True

        node.outputs.clear()
        node.mode_state.mission_id = "wrong-mission"
        node.mode_state.generation = 2
        node.pump(0.2, linear_x=0.2)
        if not node.outputs or any(abs(value) > 1.0e-6 for value in node.outputs):
            raise AssertionError(
                f"mode/execution owner mismatch leaked normal source: {node.outputs}"
            )
        summary["mode_owner_mismatch_output_max_abs"] = max(
            abs(value) for value in node.outputs
        )
        node.mode_state.mission_id = "mission-isolated"
        node.mode_state.generation = 3
        node.pump(0.1)

        node.mode_state.lease_remaining.sec = 0
        node.mode_state.lease_remaining.nanosec = 100_000_000
        node.mode_state.generation = 4
        node.pump(0.03)
        node.mode_state = None
        node.pump(0.12)
        node.outputs.clear()
        node.pump(0.2, linear_x=0.2)
        if not node.outputs or any(abs(value) > 1.0e-6 for value in node.outputs):
            raise AssertionError(
                f"expired mode lease leaked normal source before heartbeat timeout: "
                f"{node.outputs}"
            )
        if (
            node.state is None
            or node.state.execution_mode_contract_valid
            or not node.state.interlock_effective_motion_blocked
            or node.state.interlock_effective_block_reason
            != "execution_mode_invalid"
        ):
            raise AssertionError(
                f"expired mode diagnostics inconsistent: {node.state}"
            )
        summary["mode_near_expiry_output_max_abs"] = max(
            abs(value) for value in node.outputs
        )

        mode_state.lease_remaining.sec = 5
        mode_state.lease_remaining.nanosec = 0
        mode_state.generation = 5
        node.mode_state = mode_state
        node.pump(0.25)
        expiry_deadline = time.monotonic() + 2.0
        while (
            (
                node.state is None
                or node.state.execution_lease_active
                or not node.state.motion_blocked
            )
            and time.monotonic() < expiry_deadline
        ):
            node.pump(0.1)
        if (
            node.state is None
            or not node.state.execution_session_engaged
            or node.state.execution_lease_active
            or not node.state.motion_blocked
        ):
            raise AssertionError(f"execution expiry did not lock motion: {node.state}")

        node.outputs.clear()
        node.pump(0.5, linear_x=0.2)
        if not node.outputs or any(abs(value) > 1.0e-6 for value in node.outputs):
            raise AssertionError(f"expired execution lease leaked motion: {node.outputs}")
        summary["expired_output_max_abs"] = max(abs(value) for value in node.outputs)
        summary["expiry_reason"] = node.state.transition_reason
        print(json.dumps(summary, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

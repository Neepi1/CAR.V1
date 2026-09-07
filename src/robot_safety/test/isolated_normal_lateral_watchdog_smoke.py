#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _is_zero(message: Twist, epsilon: float = 1.0e-6) -> bool:
    values = (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )
    return all(math.isfinite(value) and abs(value) <= epsilon for value in values)


class NormalLateralWatchdogSmoke(Node):
    def __init__(self) -> None:
        super().__init__("normal_lateral_watchdog_smoke")
        self.outputs: list[Twist] = []
        self.input_pub = self.create_publisher(Twist, "/lateral_guard_test/input", 1)
        self.permit_pub = self.create_publisher(
            Bool, "/lateral_guard_test/nav_terminal_lateral_enable", 10
        )
        self.mode_pub = self.create_publisher(
            String, "/lateral_guard_test/ranger_status", 10
        )
        self.create_subscription(
            Twist, "/lateral_guard_test/output", self._on_output, 100
        )

    def _on_output(self, message: Twist) -> None:
        self.outputs.append(message)

    def wait_for_graph(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if (
                self.input_pub.get_subscription_count() > 0
                and self.permit_pub.get_subscription_count() > 0
                and self.mode_pub.get_subscription_count() > 0
            ):
                return
        raise RuntimeError("isolated robot_safety subscriptions did not become ready")

    def pump(
        self,
        duration_sec: float,
        *,
        publish_command: bool,
        command_period_sec: float = 0.05,
    ) -> None:
        deadline = time.monotonic() + duration_sec
        next_command = 0.0
        next_support = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_support:
                permit = Bool()
                permit.data = True
                self.permit_pub.publish(permit)

                status = String()
                status.data = (
                    '{"actual_motion_mode":{"available":true,"fresh":true,'
                    '"code":1},"actual_motion_mode_source":"isolated_test"}'
                )
                self.mode_pub.publish(status)
                next_support = now + 0.05

            if publish_command and now >= next_command:
                command = Twist()
                command.linear.y = -0.05
                self.input_pub.publish(command)
                next_command = now + command_period_sec

            rclpy.spin_once(self, timeout_sec=0.005)


def main() -> None:
    rclpy.init()
    node = NormalLateralWatchdogSmoke()
    try:
        node.wait_for_graph()

        # Warm every subscription and discard startup/discovery samples.
        node.pump(0.60, publish_command=True)
        node.outputs.clear()
        node.pump(0.20, publish_command=True)
        node.outputs.clear()

        # This is the field failure seam: a fresh normal Nav2 lateral stream and
        # a fresh terminal-lateral permit must not be interleaved with timer zeros.
        node.pump(0.80, publish_command=True)
        active_outputs = list(node.outputs)
        lateral_outputs = [
            message
            for message in active_outputs
            if math.isclose(message.linear.y, -0.05, abs_tol=1.0e-6)
            and math.isclose(message.linear.x, 0.0, abs_tol=1.0e-6)
            and math.isclose(message.angular.z, 0.0, abs_tol=1.0e-6)
        ]
        zero_outputs = [message for message in active_outputs if _is_zero(message)]
        if len(lateral_outputs) < 8:
            raise AssertionError(
                "fresh normal lateral stream was not passed often enough: "
                f"lateral={len(lateral_outputs)} total={len(active_outputs)}"
            )
        if zero_outputs:
            raise AssertionError(
                "fresh normal lateral stream was interrupted by safety-timer zeros: "
                f"zero={len(zero_outputs)} lateral={len(lateral_outputs)} "
                f"total={len(active_outputs)}"
            )

        # A crashed/stopped upstream must still be forced to zero by the normal
        # watchdog after its freshness window expires.
        node.outputs.clear()
        node.pump(0.75, publish_command=False)
        stale_outputs = list(node.outputs)
        stale_zero_outputs = [message for message in stale_outputs if _is_zero(message)]
        if not stale_zero_outputs:
            raise AssertionError("stale normal lateral stream did not produce watchdog zero")

        print(
            json.dumps(
                {
                    "ok": True,
                    "active_total": len(active_outputs),
                    "active_lateral": len(lateral_outputs),
                    "active_zero": len(zero_outputs),
                    "stale_total": len(stale_outputs),
                    "stale_zero": len(stale_zero_outputs),
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SafetyState(str, Enum):
    OK = "OK"
    ESTOP_ACTIVE = "ESTOP_ACTIVE"
    LOCALIZATION_INVALID = "LOCALIZATION_INVALID"
    COMMAND_STALE = "COMMAND_STALE"


@dataclass(frozen=True)
class SafetySnapshot:
    state: SafetyState
    motion_allowed: bool


class RobotSafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_safety")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("watchdog_timeout_sec", 1.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel_collision_checked")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel")
        self.declare_parameter("estop_topic", "/safety/estop")
        self.declare_parameter("localization_ok_topic", "/localization/health")
        self.declare_parameter("require_localization_health", False)
        self.declare_parameter("status_topic", "/safety/status")
        self.declare_parameter("motion_allowed_topic", "/safety/motion_allowed")
        self.declare_parameter("publish_zero_on_startup", True)

        self.require_localization_health = bool(self.get_parameter("require_localization_health").value)
        self.estop_active = False
        self.localization_ok = not self.require_localization_health
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_snapshot: SafetySnapshot | None = None

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_out_topic").value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.motion_allowed_pub = self.create_publisher(
            Bool, str(self.get_parameter("motion_allowed_topic").value), 10
        )

        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_in_topic").value), self.on_cmd, 10)
        self.create_subscription(Bool, str(self.get_parameter("estop_topic").value), self.on_estop, 10)
        localization_ok_topic = str(self.get_parameter("localization_ok_topic").value)
        if localization_ok_topic:
            self.create_subscription(Bool, localization_ok_topic, self.on_localization_ok, 10)

        timer_period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(timer_period, self.on_timer)

        if bool(self.get_parameter("publish_zero_on_startup").value):
            self.publish_command(self.zero_twist(), SafetySnapshot(SafetyState.COMMAND_STALE, False))

    def zero_twist(self) -> Twist:
        return Twist()

    def current_snapshot(self) -> SafetySnapshot:
        if self.estop_active:
            return SafetySnapshot(SafetyState.ESTOP_ACTIVE, False)
        if self.require_localization_health and not self.localization_ok:
            return SafetySnapshot(SafetyState.LOCALIZATION_INVALID, False)
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > float(self.get_parameter("watchdog_timeout_sec").value):
            return SafetySnapshot(SafetyState.COMMAND_STALE, False)
        return SafetySnapshot(SafetyState.OK, True)

    def publish_snapshot(self, snapshot: SafetySnapshot) -> None:
        if self.last_snapshot == snapshot:
            return
        self.status_pub.publish(String(data=snapshot.state.value))
        self.motion_allowed_pub.publish(Bool(data=snapshot.motion_allowed))
        self.last_snapshot = snapshot

    def publish_command(self, cmd: Twist, snapshot: SafetySnapshot) -> None:
        self.cmd_pub.publish(cmd)
        self.publish_snapshot(snapshot)

    def on_estop(self, msg: Bool) -> None:
        self.estop_active = bool(msg.data)
        snapshot = self.current_snapshot()
        if not snapshot.motion_allowed:
            self.publish_command(self.zero_twist(), snapshot)
        else:
            self.publish_snapshot(snapshot)

    def on_localization_ok(self, msg: Bool) -> None:
        self.localization_ok = bool(msg.data)
        snapshot = self.current_snapshot()
        if not snapshot.motion_allowed:
            self.publish_command(self.zero_twist(), snapshot)
        else:
            self.publish_snapshot(snapshot)

    def on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        snapshot = self.current_snapshot()
        if snapshot.motion_allowed:
            self.publish_command(msg, snapshot)
            return
        self.publish_command(self.zero_twist(), snapshot)

    def on_timer(self) -> None:
        snapshot = self.current_snapshot()
        if snapshot.motion_allowed:
            self.publish_snapshot(snapshot)
            return
        self.publish_command(self.zero_twist(), snapshot)


def main() -> None:
    rclpy.init()
    node = RobotSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Real robot_safety command routing, synthetic inputs, no network or hardware.

Requires a fresh Docker container with --network none --runtime runc, no device
mounts, private IPC, ROS_DOMAIN_ID=184 and ROS_LOCALHOST_ONLY=1. Pass an existing
candidate executable through --binary; the fixture never builds or deploys it.
All application interfaces are remapped under /elevator_bypass_test/.
"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_interfaces.msg import OperatingModeState
from robot_interfaces.srv import SetMotionHold
from std_msgs.msg import Bool, String


PREFIX = "/elevator_bypass_test/"
TRANSACTION = "isolated-postcall-20260910"


class Fixture(Node):
    def __init__(self):
        super().__init__("fixture", namespace=PREFIX[:-1])
        self.outputs = []
        self.pubs = {
            name: self.create_publisher(
                cls, PREFIX + name,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                if name == "mode" else 10)
            for name, cls in (
                ("cm", Twist), ("nav", Twist), ("permit", String),
                ("mode", OperatingModeState), ("estop", Bool), ("health", Bool))
        }
        self.create_subscription(Twist, PREFIX + "output", self.outputs.append, 100)
        self.status = None
        self.create_subscription(String, PREFIX + "status", self.set_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.hold_client = self.create_client(SetMotionHold, PREFIX + "hold")
        self.mode = "DOORWAY"
        self.generation = 1
        self.permit = TRANSACTION
        self.send_mode = True
        self.estop = False
        self.health = True
        self.cm_command = (0.0, 0.0, 0.0)
        self.hold_sequence = 0

    def set_status(self, message):
        self.status = message.data

    def wait_ready(self):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if (all(pub.get_subscription_count() for pub in self.pubs.values())
                    and self.count_publishers(PREFIX + "output")
                    and self.hold_client.service_is_ready()):
                return
        raise AssertionError("isolated safety interfaces not ready")

    def publish_command(self, name, values):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = values
        self.pubs[name].publish(msg)

    def pump(self, seconds, command):
        deadline = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                if self.send_mode:
                    msg = OperatingModeState()
                    msg.stamp = self.get_clock().now().to_msg()
                    msg.generation = self.generation
                    msg.mode = self.mode
                    msg.owner = "robot_elevator_manager"
                    msg.mission_id = "elevator_" + TRANSACTION
                    msg.lease_id = "isolated-operating-mode-lease"
                    msg.lease_active = self.mode != "NORMAL"
                    msg.lease_remaining.sec = 2
                    if self.mode == "NORMAL":
                        msg.transition_reason = "lease_released"
                    self.pubs["mode"].publish(msg)
                if self.permit is not None:
                    self.pubs["permit"].publish(String(data=self.permit))
                self.pubs["estop"].publish(Bool(data=self.estop))
                self.pubs["health"].publish(Bool(data=self.health))
                # Fresh CM zero keeps arriving even while nav is nonzero.
                self.publish_command("cm", self.cm_command)
                if command is not None:
                    self.publish_command("nav", command)
                next_publish = time.monotonic() + 0.03
            rclpy.spin_once(self, timeout_sec=0.003)

    def check(self, name, command, expected, *, warm=0.6, seconds=0.4):
        self.pump(warm, command)
        self.outputs.clear()
        self.pump(seconds, command)
        observed = [(msg.linear.x, msg.linear.y, msg.angular.z) for msg in self.outputs]
        bad = [values for values in observed if not all(
            math.isclose(got, want, abs_tol=1e-6) for got, want in zip(values, expected))]
        assert len(observed) >= 5 and not bad, {
            "case": name, "expected": expected, "samples": len(observed),
            "mismatches": bad[:5], "status": self.status}
        print(json.dumps({"case": name, "ok": True, "samples": len(observed),
                          "expected": expected, "status": self.status}), flush=True)

    def set_mode(self, mode):
        self.mode = mode
        self.generation += 1

    def set_hold(self, active):
        request = SetMotionHold.Request()
        request.owner = "isolated_fixture"
        request.transaction_id = TRANSACTION
        request.reason = "isolated routing regression"
        request.operation = request.OP_ACQUIRE if active else request.OP_RELEASE
        self.hold_sequence += 1
        request.command_sequence = self.hold_sequence
        future = self.hold_client.call_async(request)
        deadline = time.monotonic() + 4.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
        assert future.done() and future.result().success, "isolated hold request failed"

    def run(self):
        spin = (0.0, 0.0, 0.30)
        stop = (0.0, 0.0, 0.0)
        # Discovery and initial mode/permit activation intentionally publish a
        # transition stop. Settle those inputs before assessing steady routing.
        self.pump(1.0, spin)
        self.check("doorway_nav_wins_over_continuous_cm_zero", spin, spin)
        self.set_mode("ELEVATOR_WAIT")
        self.check("postcall_wait_nav_wins_over_continuous_cm_zero", spin, spin)

        self.permit = ""
        self.check("cleared_permit_returns_to_cm_zero", spin, stop)
        self.permit = TRANSACTION
        self.check("renewed_exact_permit_recovers_nav", spin, spin)
        self.permit = None
        self.check("expired_permit_stops_even_with_fresh_nav", spin, stop, warm=1.1)
        self.permit = TRANSACTION
        self.check("fresh_permit_recovers_after_expiry", spin, spin)

        self.check("stale_nav_stops_despite_fresh_cm_and_permit", None, stop)
        self.check("fresh_nav_recovers_after_command_expiry", spin, spin)
        self.estop = True
        self.check("estop_still_stops_authorized_bypass", spin, stop)
        self.estop = False
        self.check("cleared_estop_recovers_authorized_bypass", spin, spin)
        self.health = False
        self.check("localization_invalid_still_stops", spin, stop)
        self.health = True
        self.check("localization_recovery_allows_nav", spin, spin)

        self.set_hold(True)
        self.check("motion_hold_still_stops_authorized_bypass", spin, stop)
        self.set_hold(False)
        self.check("released_hold_requires_fresh_contract_and_recovers", spin, spin)

        self.permit = "different-transaction"
        self.check("wrong_transaction_does_not_select_nav", spin, stop)
        self.permit = TRANSACTION
        self.check("exact_transaction_recovers", spin, spin)
        self.set_mode("NORMAL")
        self.check("normal_mode_cannot_bypass_cm", spin, stop)
        self.permit = ""
        self.cm_command = (0.07, 0.0, 0.0)
        self.check("normal_mode_still_selects_checked_command", spin, self.cm_command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True)
    args = parser.parse_args()
    if (not Path("/.dockerenv").exists()
            or set(os.listdir("/sys/class/net")) != {"lo"}
            or os.environ.get("ROS_DOMAIN_ID") != "184"
            or os.environ.get("ROS_LOCALHOST_ONLY") != "1"):
        raise SystemExit("REFUSED: requires network-none Docker, domain 184, localhost only")
    topics = {
        "cmd_vel_in_topic": "cm", "elevator_entry_cmd_vel_in_topic": "nav",
        "api_cmd_vel_in_topic": "api", "docking_cmd_vel_in_topic": "docking",
        "cmd_vel_out_topic": "output", "cmd_vel_mirror_topic": "mirror",
        "elevator_entry_collision_bypass_permit_topic": "permit",
        "execution_mode_state_topic": "mode", "estop_topic": "estop",
        "localization_ok_topic": "health", "status_topic": "status",
        "motion_allowed_topic": "motion_allowed", "motion_hold_service": "hold",
        "recovery_hold_release_service": "release_hold",
        "execution_lease_service": "execution_lease",
        "motion_interlock_state_topic": "interlock",
        "dock_safety_interlock_state_topic": "dock_interlock",
        "dock_safety_interlock_reconcile_service": "reconcile_dock",
        "spin_to_drive_odom_topic": "wheel", "spin_to_drive_local_odom_topic": "local_odom",
        "spin_to_drive_imu_topic": "imu", "mode_controller_status_topic": "chassis_mode",
        "reverse_enable_topic": "reverse", "docking_reverse_enable_topic": "dock_reverse",
        "teleop_reverse_enable_topic": "teleop_reverse",
        "nav_terminal_reverse_enable_topic": "nav_reverse",
        "nav_terminal_lateral_enable_topic": "nav_lateral",
        "battery_state_topic": "battery", "docking_status_topic": "dock_status",
    }
    options = [args.binary, "--ros-args", "-r", "__node:=safety", "-r",
               "__ns:=" + PREFIX[:-1], "-r", "/rosout:=" + PREFIX + "rosout",
               "-r", "/parameter_events:=" + PREFIX + "parameter_events"]
    for key, value in topics.items():
        options += ["-p", f"{key}:={PREFIX}{value}"]
    # Keep command freshness, zero-priority, estop, localization and interlocks
    # enabled. Dock contact needs separate fixtures; no production latch is read.
    for key, value in {
        "publish_rate_hz": "20.0", "watchdog_timeout_sec": "1.0",
        "require_localization_health": "true", "block_normal_motion_when_docked": "false",
        "bms_docking_interlock_enabled": "false", "enable_bms_contact_guard": "false",
        "enable_docking_status_guard": "false", "enable_docked_latch_file_guard": "false",
        "docking_contact_latch_file": PREFIX + "no_latch.json",
    }.items():
        options += ["-p", f"{key}:={value}"]
    with subprocess.Popen(options) as process:
        rclpy.init(args=["--ros-args", "-r", "/rosout:=" + PREFIX + "rosout",
                          "-r", "/parameter_events:=" + PREFIX + "parameter_events"])
        node = Fixture()
        try:
            node.wait_ready()
            node.run()
            assert process.poll() is None, "safety process exited"
            print("PASS: real safety elevator bypass routing", flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    main()

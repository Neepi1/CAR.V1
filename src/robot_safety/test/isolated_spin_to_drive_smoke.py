#!/usr/bin/env python3
"""Exercise the real safety executable; ONLY inside a network-none test container.

No production topics, devices, service calls, or live sensor subscriptions.
See docs/spin_to_drive_actual_mode.md for the isolated invocation.
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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


PREFIX = "/spin_mode_test/"


def mode(code, *, fresh=True, available=True, desired=0):
    return json.dumps({
        "actual_motion_mode": {
            "available": available, "fresh": fresh, "code": code},
        "actual_motion_mode_source": "isolated_fixture",
        "desired_motion_mode": {"available": True, "fresh": True, "code": desired},
    }, separators=(",", ":"))


class Fixture(Node):
    def __init__(self):
        super().__init__("spin_mode_fixture")
        self.outputs = []
        self.pubs = {
            name: self.create_publisher(cls, PREFIX + name, 10)
            for name, cls in (
                ("input", Twist), ("api", Twist), ("docking", Twist),
                ("mode", String), ("wheel", Odometry), ("imu", Imu),
                ("lateral_permit", Bool), ("estop", Bool))
        }
        self.create_subscription(Twist, PREFIX + "output", self.outputs.append, 100)
        self.status = None
        self.wz = 0.06
        self.source = "input"
        self.estop = False

    def wait_ready(self):
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if (all(pub.get_subscription_count() for pub in self.pubs.values())
                    and self.count_publishers(PREFIX + "output")):
                return
        raise AssertionError("isolated safety endpoints not ready")

    def pump(self, seconds, command):
        deadline = time.monotonic() + seconds
        next_support = next_command = 0.0
        while time.monotonic() < deadline:
            stamp = time.monotonic()
            if stamp >= next_support:
                if self.status is not None:
                    self.pubs["mode"].publish(String(data=self.status))
                odom = Odometry()
                odom.twist.twist.angular.z = self.wz
                self.pubs["wheel"].publish(odom)
                imu = Imu()
                imu.angular_velocity.z = self.wz
                self.pubs["imu"].publish(imu)
                self.pubs["lateral_permit"].publish(Bool(data=True))
                self.pubs["estop"].publish(Bool(data=self.estop))
                next_support = stamp + 0.02
            if command is not None and stamp >= next_command:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = command
                self.pubs[self.source].publish(msg)
                next_command = stamp + 0.04
            rclpy.spin_once(self, timeout_sec=0.003)

    def check(self, name, command, *, zero=False, warm=0.12, seconds=0.24):
        # Drain previous-command samples before examining this phase.
        self.pump(warm, command)
        self.outputs.clear()
        self.pump(seconds, command)
        expected = (0.0, 0.0, 0.0) if zero else command
        assert len(self.outputs) >= 3, (name, "missing output", len(self.outputs))
        bad = [msg for msg in self.outputs if not all(
            math.isclose(got, want, abs_tol=1e-6)
            for got, want in zip(
                (msg.linear.x, msg.linear.y, msg.angular.z), expected))]
        zeros = sum(abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) < 1e-9
                    for msg in self.outputs)
        assert not bad, (
            f"{name}: expected={'zero' if zero else command}, "
            f"mismatches={len(bad)}/{len(self.outputs)}, zeros={zeros}")
        print(json.dumps({"case": name, "ok": True,
                          "samples": len(self.outputs), "zeros": zeros}), flush=True)

    def run(self):
        drive = (0.04235, 0.0, -0.02)
        slow_arc = (0.024011962, 0.0, -0.021120096)
        spin = (0.0, 0.0, 0.15)
        stop = (0.0, 0.0, 0.0)

        # Replay the 2026-09-08 10:18:14 UTC command pattern. Physical yaw
        # remains nonzero; actual mode is Ackermann, never SPINNING.
        self.status = mode(0, desired=2)
        self.pump(0.3, slow_arc)
        self.check("slow_ackermann_must_not_inject_zero", drive)
        for _ in range(3):
            self.check("threshold_crossing_below_003", slow_arc)
            self.check("threshold_crossing_above_003", drive)

        self.status = None
        self.pump(0.6, spin)
        self.check("missing_actual_feedback_does_not_guess_spin", drive)
        for status in (mode(2, fresh=False), mode(2, available=False), mode(255)):
            self.status = status
            self.pump(0.15, spin)
            self.check("invalid_actual_feedback_does_not_arm", drive)

        for code in (1, 3):
            self.status = mode(code)
            self.check("actual_lateral_passes", (0.0, 0.05, 0.0))
        self.status = mode(0)
        self.pump(0.15, drive)

        # A real mode-entry event, including delayed feedback, must be kept
        # through pure rotation/zero commands and repeated SPINNING heartbeats.
        self.status = mode(2)
        self.wz = 0.0
        self.check("stationary_spin_entry_does_not_truncate_spin", spin, seconds=0.4)
        self.wz = 0.06
        self.check("ongoing_spin_passes", spin)
        self.check("zero_command_passes_without_consuming_episode", stop)
        self.check("real_spin_tail_holds_translation", drive, zero=True)
        # Repeated same-mode feedback must not reset stability accumulation.
        self.wz = 0.0
        self.check("stable_tail_releases_without_ackermann_feedback", drive, warm=0.5)
        self.wz = 0.06
        self.check("same_spin_feedback_cannot_rearm_consumed_episode", drive)

        # A subsequent genuine entry rearms. Leaving mode=2 does not erase
        # the physical tail, nor does feedback loss reset the bounded timeout.
        self.status = mode(0)
        self.pump(0.15, drive)
        self.status = mode(2)
        self.pump(0.15, spin)
        self.status = mode(0)
        self.check("mode_exit_preserves_real_tail", drive, zero=True)
        self.status = None
        self.check("feedback_loss_preserves_pending_tail", drive, zero=True)
        self.pump(1.6, drive)
        self.check("feedback_loss_still_has_bounded_timeout", drive)

        self.status = mode(2)
        self.pump(0.15, spin)
        self.check("new_real_spin_rearms_after_timeout", drive, zero=True)
        self.pump(2.1, drive)
        self.check("same_mode_heartbeat_cannot_extend_timeout", drive)
        self.check("same_mode_after_timeout_cannot_rearm", drive)

        # Shared command sources retain their contracts after the episode.
        self.status = mode(0)
        for source in ("api", "docking"):
            self.source = source
            self.pump(0.3, slow_arc)
            self.check(source + "_slow_ackermann_passes", drive)
        self.estop = True
        self.check("estop_still_stops", drive, zero=True)
        self.estop = False
        self.source = "input"
        self.pump(0.5, drive)
        self.check("normal_command_recovers_after_estop", drive)
        self.pump(0.6, None)
        self.check("watchdog_still_stops", None, zero=True, warm=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--mode-exit-guard", choices=("true", "false"), default="true")
    args = parser.parse_args()
    if (not Path("/.dockerenv").exists()
            or set(os.listdir("/sys/class/net")) != {"lo"}
            or os.environ.get("ROS_DOMAIN_ID") != "183"):
        raise SystemExit("REFUSED: requires network-none Docker container, ROS_DOMAIN_ID=183")
    params = {
        "cmd_vel_in_topic": "input", "api_cmd_vel_in_topic": "api",
        "docking_cmd_vel_in_topic": "docking", "cmd_vel_out_topic": "output",
        "cmd_vel_mirror_topic": "mirror", "mode_controller_status_topic": "mode",
        "spin_to_drive_odom_topic": "wheel", "spin_to_drive_imu_topic": "imu",
        "nav_terminal_lateral_enable_topic": "lateral_permit", "estop_topic": "estop",
    }
    options = [args.binary, "--ros-args", "-r", "__node:=isolated_spin_safety"]
    for key, value in params.items():
        options += ["-p", f"{key}:={PREFIX}{value}"]
    # Other guards are tested separately. Crucially the affected settle feature
    # and its production thresholds, IMU check, and 2-second timeout stay ON.
    for key, value in {
        "require_localization_health": "false", "block_normal_motion_when_docked": "false",
        "bms_docking_interlock_enabled": "false", "enable_bms_contact_guard": "false",
        "enable_docking_status_guard": "false", "enable_docked_latch_file_guard": "false",
        "spin_to_drive_settle_enabled": "true", "mode_exit_guard_enabled": args.mode_exit_guard,
        "publish_zero_on_startup": "false", "zero_cmd_priority_enabled": "false",
        "watchdog_timeout_sec": "0.35",
    }.items():
        options += ["-p", f"{key}:={value}"]
    with subprocess.Popen(options) as process:
        rclpy.init()
        node = Fixture()
        try:
            node.wait_ready()
            node.run()
            print("PASS: actual-mode spin-to-drive integration", flush=True)
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

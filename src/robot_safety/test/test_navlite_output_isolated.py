"""Real safety output/log regression; private net/IPC/SHM namespace ONLY.

Never starts a chassis. All command endpoints use /navlite_safety_test.
The caller must supply a report-directory copy of the candidate executable.
"""
import os
from pathlib import Path
import re
import json
import signal
import socket
import subprocess
import time

import pytest

if os.environ.get("NJRH_ISOLATED_NAVLITE_TEST") != "1":
    pytest.skip("Private network/IPC/SHM test invocation required", allow_module_level=True)
assert os.environ.get("ROS_DOMAIN_ID") == "183"
assert {name for _, name in socket.if_nameindex()} == {"lo"}
rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

PREFIX = "/navlite_safety_test"


class Rig:
    def __init__(self, directory):
        binary = Path(os.environ["NJRH_TEST_SAFETY_BIN"]).resolve()
        assert str(binary).startswith("/tmp/njrh_reports/")
        rclpy.init()
        self.node = rclpy.create_node("navlite_safety_fixture")
        self.outputs = []
        self.log_path = directory / "robot_safety_common.log"
        self.log = self.log_path.open("w")
        self.recorder = None
        self.capture = directory / "capture"
        if os.environ.get("NJRH_NAVLITE_RECORD_SCRIPT"):
            self.recorder_log = (directory / "recorder.log").open("w")
            self.recorder = subprocess.Popen([
                "python3", "-B", os.environ["NJRH_NAVLITE_RECORD_SCRIPT"], "record",
                "--log", str(self.log_path), "--output", str(self.capture),
                "--duration", "60", "--interval", "0.1", "--no-stdin", "--min-free-mb", "0"],
                stdout=self.recorder_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                events = self.capture / "events.jsonl"
                if events.exists() and "opened" in events.read_text():
                    break
                assert self.recorder.poll() is None
                time.sleep(0.02)
            else:
                raise AssertionError("Test recorder did not open the fixture log")
        self.pubs = {name: self.node.create_publisher(typ, PREFIX + "/" + name, 10)
                     for name, typ in (("normal", Twist), ("api", Twist), ("docking", Twist),
                                       ("estop", Bool), ("wheel", Odometry),
                                       ("mode", String), ("battery", BatteryState))}
        self.node.create_subscription(Twist, PREFIX + "/output", self.outputs.append, 100)
        args = [str(binary), "--ros-args", "-r", "__node:=navlite_isolated_safety"]
        params = {
            "cmd_vel_in_topic": PREFIX + "/normal",
            "api_cmd_vel_in_topic": PREFIX + "/api",
            "docking_cmd_vel_in_topic": PREFIX + "/docking",
            "cmd_vel_out_topic": PREFIX + "/output",
            "cmd_vel_mirror_topic": PREFIX + "/mirror",
            "spin_to_drive_odom_topic": PREFIX + "/wheel",
            "spin_to_drive_settle_enabled": True,
            "spin_to_drive_require_imu_stable": False,
            "mode_exit_guard_enabled": False,
            "mode_controller_status_topic": PREFIX + "/mode",
            "require_localization_health": False,
            "docking_contact_latch_file": str(directory / "missing_latch.json"),
        }
        for key, value in params.items():
            args += ["-p", key + ":=" + (str(value).lower() if isinstance(value, bool) else str(value))]
        args += ["-r", "/safety/estop:=" + PREFIX + "/estop",
                 "-r", "/battery_state:=" + PREFIX + "/battery"]
        self.process = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.pump(0.04)
            if self.pubs["normal"].get_subscription_count() and self.outputs:
                return
        raise AssertionError("Isolated node discovery failed: " + self.text())

    def text(self):
        return self.log_path.read_text()

    def lines(self, role="safety"):
        return [line for line in self.text().splitlines() if "NAVLITE " + role + " " in line]

    def pump(self, seconds, *, source=None, vx=0.0, estop=None, wheel=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.text()
            if estop is not None:
                self.pubs["estop"].publish(Bool(data=estop))
            if wheel:
                odom = Odometry()
                odom.header.stamp = self.node.get_clock().now().to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.twist.twist.linear.x, odom.twist.twist.linear.y = 0.03, -0.01
                odom.twist.twist.angular.z = 0.006
                self.pubs["wheel"].publish(odom)
            if source:
                command = Twist()
                command.linear.x = vx
                self.pubs[source].publish(command)
            until = min(deadline, time.monotonic() + 0.035)
            while time.monotonic() < until:
                rclpy.spin_once(self.node, timeout_sec=0.005)

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.log.close()
        if self.recorder:
            self.recorder.send_signal(signal.SIGINT)
            self.recorder.wait(timeout=5)
            self.recorder_log.close()
            assert self.recorder.returncode == 0
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig(tmp_path):
    instance = Rig(tmp_path)
    try:
        yield instance
    finally:
        instance.close()


def test_schema_identifies_real_node_output(rig):
    assert "NAVLITE safety event=diagnostics_ready schema=2" in rig.text()
    assert "observation_only=1" in rig.text()


def test_upstream_zero_then_actual_nonzero_publication(rig):
    rig.pump(0.3, source="normal", vx=0.05)
    rig.pump(0.15, source="normal", vx=0.0)
    rig.pump(0.5, source="normal", vx=0.05)
    lines = rig.lines()
    zero = next(i for i, s in enumerate(lines) if "branch=upstream_zero_priority" in s)
    assert "in=(0.000000,0.000000,0.000000)" in lines[zero]
    assert any("publication=sent" in s and "zero=0" in s for s in lines[zero + 1:])
    assert any(t.linear.x == 0.0 for t in rig.outputs)
    assert rig.outputs[-1].linear.x == 0.05


def test_estop_release_is_state_only_until_next_command(rig):
    rig.pump(0.15, source="normal", vx=0.05)
    rig.pump(0.15, source="normal", vx=0.05, estop=True)
    assert any("final_reason=ESTOP_ACTIVE" in s and "zero=1" in s for s in rig.lines())
    rig.pump(0.035, estop=False)
    assert any("previous=ESTOP_ACTIVE state=OK allowed=1 publication=state_only" in s
               for s in rig.lines("safety_state"))
    rig.pump(0.2, source="normal", vx=0.05)
    assert "zero=0" in rig.lines()[-1]
    assert rig.outputs[-1].linear.x == 0.05


def test_watchdog_is_not_labelled_upstream_zero(rig):
    rig.pump(0.2, source="normal", vx=0.05)
    rig.pump(1.2)
    assert any("final_reason=COMMAND_STALE" in s and "input_known=0" in s
               for s in rig.lines())
    assert rig.outputs[-1].linear.x == 0.0
    rig.pump(0.2, source="normal", vx=0.05)
    assert "zero=0" in rig.lines()[-1]


def test_docking_priority_reports_ignored_input_as_no_message(rig):
    rig.pump(0.2, source="docking", vx=0.04)
    rig.pump(0.035, source="normal", vx=0.06)
    assert any("selected_source=docking" in s and "publication=no_message" in s
               for s in rig.lines("safety_arbitration"))
    assert any("selected_source=docking" in s and "out=(0.040000" in s for s in rig.lines())


def test_reverse_denial_reason_and_repeated_stop_are_bounded(rig):
    rig.pump(2.4, source="normal", vx=-0.05)
    rows = [s for s in rig.lines() if "final_reason=reverse_not_permitted" in s]
    assert 2 <= len(rows) <= 3, rows
    times = [float(re.search(r"steady_sec=([0-9.]+)", s).group(1)) for s in rows]
    assert all(b - a >= 0.99 for a, b in zip(times, times[1:]))
    assert all(t.linear.x == 0 for t in rig.outputs)


def test_existing_wheel_callback_is_observed_not_a_new_subscription(rig):
    rig.pump(0.1, wheel=True)
    rig.pump(0.2, source="normal", vx=0.05, wheel=True)
    rows = [s for s in rig.lines() if "zero=0" in s]
    assert rows
    assert "wheel_known=1" in rows[-1]
    assert "wheel=(0.030000,-0.010000,0.006000)" in rows[-1]
    assert "wheel_frame=odom wheel_child=base_link" in rows[-1]
    assert float(re.search(r"wheel_rx_age_sec=([0-9.]+)", rows[-1]).group(1)) < 0.2


def test_recorder_preserves_real_node_stop_and_resume(rig):
    if not rig.recorder:
        pytest.skip("Set NJRH_NAVLITE_RECORD_SCRIPT for actual file-tail integration")
    rig.pump(0.25, source="normal", vx=0.05, wheel=True)
    rig.pump(0.15, source="normal", vx=0.0, wheel=True)
    rig.pump(0.5, source="normal", vx=0.05, wheel=True)
    rig.pump(0.25)
    events = [json.loads(line) for line in (rig.capture / "events.jsonl").read_text().splitlines()]
    rows = [event for event in events if "NAVLITE safety " in event.get("text", "")]
    assert any("event=diagnostics_ready" in row["text"] for row in rows)
    stopped = next(i for i, row in enumerate(rows) if "branch=upstream_zero_priority" in row["text"])
    assert any("zero=0" in row["text"] and "publication=sent" in row["text"]
               for row in rows[stopped + 1:])
    assert all(row["source_stamp_ns"] is not None for row in rows)

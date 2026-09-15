"""Real docking/safety executables with fake inputs; run ONLY in an isolated netns.

Requires NJRH_ISOLATED_DOCK_TEST=1, ROS_DOMAIN_ID=183, and loopback-only networking.
All motion outputs are remapped to /dock_memory_test; no device driver is started.
NJRH_TEST_DOCKING_BIN / NJRH_TEST_SAFETY_BIN select candidate binaries.
"""
import os
import json
from pathlib import Path
import subprocess
import socket
import time

import pytest

if os.environ.get("NJRH_ISOLATED_DOCK_TEST") != "1":
    pytest.skip("Explicit isolated-network invocation required", allow_module_level=True)
assert os.environ.get("ROS_DOMAIN_ID") == "183"
assert {name for _, name in socket.if_nameindex()} == {"lo"}, "Refuse production networking"
rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robot_interfaces.msg import DockSafetyInterlockState
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

ROOT = Path(os.environ.get("NJRH_TEST_WORKSPACE", "/workspaces/njrh-v3/workspace1"))
PREFIX = "/dock_memory_test"
TOPICS = {
    "/battery_state": "battery", "/docking/status": "status",
    "/docking/undock": "undock", "/docking/start": "start", "/docking/stop": "stop",
    "/cmd_vel_docking": "command", "/cmd_vel": "output", "/cmd_vel_safe": "mirror",
    "/cmd_vel_collision_checked": "normal", "/cmd_vel_api": "api",
    "/ranger_mini3/docking_allow_reverse": "reverse",
    "/ranger_mini3/forced_mode": "mode", "/ranger_mini3/park": "park",
    "/local_state/odometry": "odom", "/wheel/odom": "wheel",
    "/motion_state": "motion", "/safety/dock_interlock_state": "memory",
    "/dock/target_observation": "observation", "/dock/gs2_scan": "scan",
}


class Rig:
    def __init__(self, tmp_path, kind, latch=False):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("dock_memory_fixture_" + kind)
        self.outputs, self.statuses, self.memories = [], [], []
        self.artifact = tmp_path / (kind + "_evidence.json")
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {}
        for name, typ, qos in [("battery", BatteryState, 10), ("odom", Odometry, 10),
                               ("command", Twist, 1), ("reverse", Bool, durable),
                               ("status", String, durable),
                               ("memory", DockSafetyInterlockState, durable)]:
            self.pubs[name] = self.node.create_publisher(typ, PREFIX + "/" + name, qos)
        self.node.create_subscription(Twist, PREFIX + ("/command" if kind == "docking" else "/output"),
                                      self.outputs.append, 10)
        self.node.create_subscription(String, PREFIX + "/status", self.statuses.append, durable)
        self.node.create_subscription(DockSafetyInterlockState, PREFIX + "/memory",
                                      self.memories.append, durable)
        self.client = self.node.create_client(Trigger, PREFIX + "/undock")
        self.stop_client = self.node.create_client(Trigger, PREFIX + "/stop")
        latch_path = tmp_path / "latch.json"
        if latch:
            latch_path.write_text('{"latched_docked":true,"source":"docking_manager"}')
        package, executable = (("robot_docking_manager", "docking_manager_node") if kind == "docking"
                               else ("robot_safety", "robot_safety_node"))
        key = "NJRH_TEST_DOCKING_BIN" if kind == "docking" else "NJRH_TEST_SAFETY_BIN"
        binary = os.environ.get(key, str(ROOT / "install" / package / "lib" / package / executable))
        args = [binary, "--ros-args", "-r", "__node:=isolated_" + kind]
        for original, target in TOPICS.items():
            args += ["-r", original + ":=" + PREFIX + "/" + target]
        params = {"docking_contact_latch_file": str(latch_path)}
        if kind == "docking":
            params.update({"undock.command_settle_s": 0.0, "undock.motion_start_timeout_s": 0.4,
                           "undock.speed_mps": 0.5, "undock.max_speed_mps": 0.5,
                           "undock.distance_m": 0.6})
        else:
            params.update({"spin_to_drive_settle_enabled": False, "mode_exit_guard_enabled": False,
                           "require_localization_health": False})
        for key, value in params.items():
            args += ["-p", key + ":=" + (str(value).lower() if isinstance(value, bool) else str(value))]
        self.log = (tmp_path / (kind + ".log")).open("w")
        self.process = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)

    def close(self):
        self.artifact.write_text(json.dumps(self.evidence(), indent=2))
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.log.close()
        self.node.destroy_node()

    def pump(self, seconds=0.3, *, contact=False, memory=None, memory_age=0.0,
             status=None, reverse=None, cmd_x=None, odom_x=0.0, full=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, "test node exited; inspect test log"
            stamp = self.node.get_clock().now()
            b = BatteryState()
            b.header.stamp = stamp.to_msg()
            b.current = 0.0
            b.voltage = 50.6
            b.percentage = 1.0
            b.present = False
            b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if contact else
                                     BatteryState.POWER_SUPPLY_STATUS_FULL if full else 0)
            self.pubs["battery"].publish(b)
            o = Odometry()
            o.header.stamp = stamp.to_msg()
            o.pose.pose.position.x = odom_x
            o.pose.pose.orientation.w = 1.0
            self.pubs["odom"].publish(o)
            if memory is not None:
                m = DockSafetyInterlockState()
                m.stamp = (stamp - Duration(seconds=memory_age)).to_msg()
                m.enabled = True
                m.active = m.memory_latched = memory
                self.pubs["memory"].publish(m)
            if status is not None:
                self.pubs["status"].publish(String(data=status))
            if reverse is not None:
                self.pubs["reverse"].publish(Bool(data=reverse))
            if cmd_x is not None:
                t = Twist()
                t.linear.x = cmd_x
                self.pubs["command"].publish(t)
            until = time.monotonic() + 0.04
            while time.monotonic() < until:
                rclpy.spin_once(self.node, timeout_sec=0.01)

    def undock(self):
        assert self.client.wait_for_service(timeout_sec=8)
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        assert future.done() and future.result() is not None
        return future.result()

    def prime_memory(self):
        self.pump(1.5, contact=True, status="docked")
        self.wait_for(lambda: self.memories and not self.memories[-1].live_bms_contact,
                      status="idle")
        assert self.memories and self.memories[-1].memory_latched

    def wait_for(self, predicate, **inputs):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.pump(0.1, **inputs)
            if predicate():
                return
        assert predicate(), self.evidence()

    def evidence(self):
        return {"status": [s.data for s in self.statuses[-12:]],
                "state": [(s.memory_latched, s.reverse_session_seen, s.reverse_permit_active,
                           s.battery_sample_fresh, s.live_bms_contact) for s in self.memories[-8:]]}

    def begin_reverse(self):
        self.wait_for(lambda: self.memories and self.memories[-1].reverse_session_seen and
                      self.memories[-1].reverse_permit_active,
                      status="undocking preparing phase=preparing", reverse=True)


@pytest.fixture
def rig_factory(tmp_path):
    rigs = []
    def create(kind, **kwargs):
        rig = Rig(tmp_path, kind, **kwargs)
        rigs.append(rig)
        return rig
    yield create
    for rig in reversed(rigs):
        rig.close()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize("evidence", ["memory", "contact", "latch", "full_latch"])
def test_real_service_accepts_each_supported_evidence(rig_factory, evidence):
    rig = rig_factory("docking", latch="latch" in evidence)
    rig.pump(1.5, memory=True if evidence == "memory" else None,
             contact=evidence == "contact", full=evidence == "full_latch")
    response = rig.undock()
    assert response.success, response.message


@pytest.mark.parametrize("evidence", ["none", "clear", "old_stamp", "expired", "full_only"])
def test_no_usable_evidence_cannot_create_undock_motion(rig_factory, evidence):
    rig = rig_factory("docking")
    rig.pump(1.5, memory=False if evidence == "clear" else
             True if evidence in ("old_stamp", "expired") else None,
             memory_age=3.0 if evidence == "old_stamp" else 0.0,
             full=evidence == "full_only")
    if evidence == "expired":
        rig.pump(1.2)
    assert not rig.undock().success
    assert all(t.linear.x == 0.0 for t in rig.outputs)


@pytest.mark.parametrize("terminal", ["undock_failed_motion_start_timeout",
                                      "stopped by service", "undock_failed_no_progress"])
def test_failed_or_cancelled_reverse_keeps_memory(rig_factory, terminal):
    rig = rig_factory("safety")
    rig.prime_memory()
    rig.begin_reverse()
    rig.pump(0.3, status="undocking active", reverse=True, cmd_x=-0.1)
    assert any(t.linear.x < 0.0 for t in rig.outputs), "Existing straight reverse must still pass"
    rig.wait_for(lambda: not rig.memories[-1].reverse_permit_active,
                 status=terminal, reverse=False, cmd_x=0.0)
    assert rig.memories[-1].memory_latched, "Failure was incorrectly treated as proven undock"


@pytest.mark.parametrize("order", ["status_first", "permit_first"])
def test_success_releases_memory_in_either_callback_order(rig_factory, order):
    rig = rig_factory("safety")
    rig.prime_memory()
    rig.begin_reverse()
    success = "undocked phase=succeeded failure_reason=none distance=0.600"
    if order == "status_first":
        rig.pump(0.2, status=success, reverse=True)
    else:
        rig.wait_for(lambda: not rig.memories[-1].reverse_permit_active, reverse=False)
        assert rig.memories[-1].memory_latched, "Reverse disabled is not a success result"
    rig.wait_for(lambda: rig.memories and not rig.memories[-1].memory_latched,
                 status=success, reverse=False)
    assert not rig.memories[-1].memory_latched, rig.evidence()


def test_old_success_does_not_release_a_new_failed_attempt(rig_factory):
    rig = rig_factory("safety")
    rig.prime_memory()
    rig.pump(0.2, status="undocked phase=succeeded failure_reason=none distance=0.600")
    rig.begin_reverse()
    rig.wait_for(lambda: not rig.memories[-1].reverse_permit_active,
                 status="undock_failed_motion_start_timeout", reverse=False)
    assert rig.memories[-1].memory_latched


@pytest.mark.parametrize("success", [True, False])
def test_actual_manager_and_safety_complete_recovery_together(rig_factory, success):
    safety = rig_factory("safety")
    safety.prime_memory()
    # The fixture primes contact history, then the actual manager is the sole
    # status owner. Do not let fixture transient history compete with its FSM.
    safety.node.destroy_publisher(safety.pubs.pop("status"))
    manager = rig_factory("docking")
    assert manager.client.wait_for_service(timeout_sec=8)
    safety.wait_for(lambda: any(info.node_name == "isolated_docking" for info in
                    safety.node.get_subscriptions_info_by_topic(PREFIX + "/memory")))
    safety.pump(1.5)
    assert all(t.linear.x == 0.0 for t in safety.outputs), "Memory alone must not initiate motion"
    response = manager.undock()
    assert response.success, response.message
    safety.pump(0.2)
    safety.pump(1.0, odom_x=-0.7 if success else 0.0)
    assert any(t.linear.x < 0.0 for t in safety.outputs), "No final controlled reverse command"
    statuses = [m.data for m in safety.statuses]
    if success:
        assert any(s.startswith("undocked phase=succeeded") for s in statuses), statuses
        safety.wait_for(lambda: not safety.memories[-1].memory_latched, odom_x=-0.7)
        assert not safety.memories[-1].memory_latched
    else:
        assert any(s.startswith("undock_failed_motion_start_timeout") for s in statuses), statuses
        assert safety.memories[-1].memory_latched


def test_success_cannot_clear_memory_while_contact_is_live(rig_factory):
    rig = rig_factory("safety")
    rig.prime_memory()
    rig.begin_reverse()
    rig.pump(0.3, contact=True,
             status="undocked phase=succeeded failure_reason=none distance=0.600", reverse=False)
    assert rig.memories[-1].memory_latched

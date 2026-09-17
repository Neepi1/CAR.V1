"""Contact/history regression against the actual safety executable in a private netns.

Uses the existing dock-memory ROS fixture; no docking manager or chassis is started.
The runner must set NJRH_TEST_SAFETY_BIN to a byte-identical test copy, not the
production executable path (the parent namespace's owner guard still sees PIDs).
"""
import os
import json
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(os.environ.get("NJRH_TEST_WORKSPACE", "/workspaces/njrh-v3/workspace1"))
sys.path.insert(0, str(ROOT / "src/robot_system_tests/test"))
from test_dock_memory_undock_isolated import Rig, PREFIX, rclpy, BatteryState, Twist  # noqa: E402


def raw_sample(rig, *, status=0, current=0.0, present=False, voltage=50.6):
    b = BatteryState()
    b.header.stamp = rig.node.get_clock().now().to_msg()
    b.power_supply_status, b.current, b.present, b.voltage = status, current, present, voltage
    b.percentage = 1.0
    rig.pubs["battery"].publish(b)
    deadline = time.monotonic() + 0.07
    while time.monotonic() < deadline:
        rclpy.spin_once(rig.node, timeout_sec=0.005)


@pytest.fixture
def safety(tmp_path, request):
    binary = Path(os.environ["NJRH_TEST_SAFETY_BIN"]).resolve()
    assert binary != (ROOT / "install/robot_safety/lib/robot_safety/robot_safety_node").resolve()
    rig = Rig(tmp_path, "safety", latch=getattr(request, "param", False))
    try:
        rig.wait_for(lambda: bool(rig.memories) and rig.memories[-1].battery_sample_fresh,
                     status="idle")
        # Battery discovery does not establish the durable status subscription.
        # Count includes this fixture's own status observer and the safety node.
        rig.wait_for(lambda: rig.pubs["status"].get_subscription_count() >= 2,
                     status="idle")
        assert not rig.memories[-1].memory_latched
        yield rig
    finally:
        rig.close()
        if rclpy.ok():
            rclpy.shutdown()


def test_no_contact_docking_command_remains_available(safety):
    safety.pump(0.4, status="idle", cmd_x=0.05)
    assert not safety.memories[-1].live_bms_contact
    assert not safety.memories[-1].memory_latched
    assert any(t.linear.x > 0.04 for t in safety.outputs)


def test_unconfirmed_transient_contact_without_docking_cannot_leave_memory(safety):
    # No external docking command has been sent, including zero commands.
    # Several short-lived charging-status samples exercise callback ordering.
    safety.pump(0.16, status="idle", contact=True)
    assert safety.memories[-1].live_bms_contact
    safety.pump(0.4, status="idle", contact=False)
    assert safety.memories[-1].battery_sample_fresh
    assert not safety.memories[-1].live_bms_contact
    assert not safety.memories[-1].persistent_dock_strong
    assert not safety.memories[-1].memory_latched
    # A later fine-docking attempt must not inherit an unconfirmed contact block.
    mark = len(safety.outputs)
    safety.pump(0.4, status="idle", cmd_x=0.05)
    assert any(t.linear.x > 0.04 for t in safety.outputs[mark:])


def test_confirmed_contact_stops_push_and_retains_docked_protection(safety):
    safety.wait_for(lambda: safety.memories[-1].memory_latched,
                    status="docked", contact=True)
    assert safety.memories[-1].memory_latched
    safety.outputs.clear()
    safety.pump(0.3, status="docked", contact=True, cmd_x=0.05)
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)
    safety.outputs.clear()
    safety.pump(0.4, status="idle", contact=False, cmd_x=0.05)
    assert safety.memories[-1].memory_latched
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)


def test_first_contact_edge_is_not_necessarily_the_latch_time(safety, tmp_path):
    stages = []
    for _ in range(2):
        raw_sample(safety, status=BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        m = safety.memories[-1]
        stages.append({"live": m.live_bms_contact, "memory": m.memory_latched,
                       "reason": m.reason, "generation": m.generation})
    (tmp_path / "two_samples.json").write_text(json.dumps(stages, indent=2))
    assert stages[0]["live"] and not stages[0]["memory"]
    assert stages[1]["live"] and not stages[1]["memory"]


@pytest.mark.parametrize("fields, expected", [
    ({}, False),
    ({"status": BatteryState.POWER_SUPPLY_STATUS_CHARGING}, True),
    ({"current": 0.11}, True),
    ({"present": True}, True),
    ({"present": True, "voltage": 39.0}, False),
    ({"current": -1.0}, False),
    ({"status": BatteryState.POWER_SUPPLY_STATUS_FULL}, False),
])
def test_raw_bms_conditions_without_docking_context(safety, fields, expected):
    raw_sample(safety, **fields)
    assert safety.memories[-1].live_bms_contact == expected
    assert not safety.memories[-1].memory_latched
    raw_sample(safety, **fields)
    assert safety.memories[-1].live_bms_contact == expected
    assert not safety.memories[-1].memory_latched


@pytest.mark.parametrize("full", [False, True])
def test_real_external_docking_context_still_confirms_contact(safety, full):
    safety.pump(0.12, status="idle", cmd_x=0.05)
    safety.pump(0.12, status="idle", cmd_x=0.05, contact=not full, full=full)
    assert safety.memories[-1].memory_latched
    safety.outputs.clear()
    safety.pump(0.3, status="idle", cmd_x=0.05)
    assert safety.memories[-1].memory_latched
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)


def test_external_command_after_contact_is_not_lost(safety):
    raw_sample(safety, status=BatteryState.POWER_SUPPLY_STATUS_CHARGING)
    assert not safety.memories[-1].memory_latched
    safety.pump(0.2, status="idle", contact=True, cmd_x=0.05)
    assert safety.memories[-1].memory_latched
    # A later no-contact sample is not proof that a confirmed dock was departed.
    safety.outputs.clear()
    safety.pump(0.4, status="idle", contact=False, cmd_x=0.05)
    assert safety.memories[-1].memory_latched
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)


def test_internal_stop_cannot_turn_full_status_into_contact(safety):
    raw_sample(safety, status=BatteryState.POWER_SUPPLY_STATUS_CHARGING)
    raw_sample(safety, status=BatteryState.POWER_SUPPLY_STATUS_FULL)
    assert not safety.memories[-1].live_bms_contact
    assert not safety.memories[-1].memory_latched


def test_internal_stop_cannot_refresh_expired_external_context(safety):
    safety.pump(0.1, status="idle", cmd_x=0.05)
    safety.pump(0.4, status="idle")
    raw_sample(safety, status=BatteryState.POWER_SUPPLY_STATUS_CHARGING)
    raw_sample(safety, status=BatteryState.POWER_SUPPLY_STATUS_CHARGING)
    assert safety.memories[-1].live_bms_contact
    assert not safety.memories[-1].memory_latched


def test_normal_navigation_is_not_a_docking_context(safety):
    pub = safety.node.create_publisher(Twist, PREFIX + "/normal", 1)
    command = Twist()
    command.linear.x = 0.05
    safety.pump(0.1, status="idle")
    for _ in range(2):
        pub.publish(command)
        raw_sample(safety, current=0.11)
    assert safety.memories[-1].live_bms_contact
    assert not safety.memories[-1].memory_latched
    safety.pump(0.4, status="idle")
    assert not safety.memories[-1].memory_latched
    mark = len(safety.outputs)
    safety.pump(0.4, status="idle", cmd_x=0.05)
    assert any(t.linear.x > 0.04 for t in safety.outputs[mark:])


@pytest.mark.parametrize("safety", [True], indirect=True)
def test_strong_persistent_evidence_still_confirms_and_protects(safety):
    assert safety.memories[-1].persistent_dock_strong
    raw_sample(safety, current=0.11)
    assert safety.memories[-1].memory_latched
    safety.outputs.clear()
    safety.pump(0.3, status="idle", cmd_x=0.05)
    assert safety.memories[-1].memory_latched
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)


def test_confirmed_memory_is_not_cleared_by_stale_bms(safety):
    safety.wait_for(lambda: safety.memories[-1].memory_latched,
                    status="docked", contact=True)
    safety.outputs.clear()
    deadline = time.monotonic() + 3.3
    command = Twist()
    command.linear.x = 0.05
    while time.monotonic() < deadline:
        safety.pubs["command"].publish(command)
        rclpy.spin_once(safety.node, timeout_sec=0.03)
    assert not safety.memories[-1].battery_sample_fresh
    assert safety.memories[-1].memory_latched
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)

"""Run with the existing private-netns dock-memory runner, never on the robot domain."""
import os
from pathlib import Path
import sys
import time

ROOT = Path(os.environ.get("NJRH_TEST_WORKSPACE", "/workspaces/njrh-v3/workspace1"))
sys.path.insert(0, str(ROOT / "src/robot_system_tests/test"))
from test_dock_memory_undock_isolated import rig_factory, PREFIX, rclpy, BatteryState, Trigger


def current(rig, value=0.7):
    msg = BatteryState()
    msg.header.stamp = rig.node.get_clock().now().to_msg()
    msg.current, msg.voltage, msg.percentage = value, 498.0, 85.0
    rig.pubs["battery"].publish(msg)
    drain(rig, 0.08)


def drain(rig, duration=0.1):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(rig.node, timeout_sec=0.01)


def call(rig, client):
    assert client.wait_for_service(timeout_sec=5)
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(rig.node, future, timeout_sec=3)
    assert future.done()
    return future.result()


def test_idle_current_does_not_admit_undock(rig_factory):
    rig = rig_factory("docking")
    rig.pump(1.0)
    current(rig)
    assert not rig.undock().success


def test_cached_predock_current_is_not_reused_but_new_fine_sample_stops(rig_factory):
    rig = rig_factory("docking")
    rig.pump(1.0)
    current(rig)
    start = rig.node.create_client(Trigger, PREFIX + "/start")
    rig.statuses.clear()
    assert call(rig, start).success
    drain(rig, 0.15)
    assert not any("contact_stopping" in s.data for s in rig.statuses)
    assert not (rig.artifact.parent / "latch.json").exists()
    rig.statuses.clear()
    rig.outputs.clear()
    current(rig, 1.1)
    assert any("contact_stopping" in s.data for s in rig.statuses)
    assert rig.outputs and all(t.linear.x == 0.0 and t.angular.z == 0.0 for t in rig.outputs)
    assert (rig.artifact.parent / "latch.json").exists()


def test_cancelled_fine_phase_does_not_retain_current_authority(rig_factory):
    rig = rig_factory("docking")
    rig.pump(1.0)
    start = rig.node.create_client(Trigger, PREFIX + "/start")
    assert call(rig, start).success
    assert call(rig, rig.stop_client).success
    rig.statuses.clear()
    current(rig)
    assert not any("contact_stopping" in s.data for s in rig.statuses)
    assert not rig.undock().success


def test_confirmed_latch_with_current_still_admits_controlled_undock(rig_factory):
    rig = rig_factory("docking", latch=True)
    rig.pump(1.0)
    current(rig)
    assert rig.undock().success


def test_fine_current_zero_reaches_final_safety_output(rig_factory):
    safety = rig_factory("safety")
    safety.node.destroy_publisher(safety.pubs.pop("status"))
    manager = rig_factory("docking")
    safety.pump(1.2)
    start = manager.node.create_client(Trigger, PREFIX + "/start")
    assert call(manager, start).success
    current(safety, 1.1)
    drain(manager)
    # Manager's existing contact-stop path wrote the shared strong latch.
    current(safety, 1.1)
    drain(manager)
    assert any("contact_stopping" in s.data for s in safety.statuses)
    # Immediate zero is a command-path assertion, not a claim that the safety
    # node's existing one-second file cache has already refreshed.
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)
    deadline = time.monotonic() + 2.0
    while not safety.memories[-1].memory_latched and time.monotonic() < deadline:
        current(safety, 1.1)
    assert safety.memories[-1].memory_latched
    safety.outputs.clear()
    from geometry_msgs.msg import Twist
    push = Twist()
    push.linear.x = 0.05
    for _ in range(4):
        safety.pubs["command"].publish(push)
        current(safety, 1.1)
    assert safety.outputs and all(t.linear.x == 0.0 for t in safety.outputs)

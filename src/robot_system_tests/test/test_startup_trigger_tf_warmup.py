"""Actual trigger main/shell with offline ROS/TF transport adapters only."""

import importlib.util
import sys
import types

import pytest

from test_amcl_startup_sequence import SCRIPTS, run_shell


def trigger_fixture(monkeypatch, mode="ready", observe=True):
    events = []
    clock = [0.0]
    buffers = []
    queued_tf = [True] if mode == "queued_pre_accept" else []
    response = types.SimpleNamespace(accepted=mode != "rejected", message="explicit_sequence=1")

    class TransformException(Exception):
        pass

    class Buffer:
        def __init__(self):
            events.append("buffer")
            self.fresh = False
            buffers.append(self)

        def lookup_transform(self, target, source, when):
            assert (target, source, when.nanoseconds) == ("map", "odom", 0)
            events.append("lookup")
            if mode == "lookup_error":
                raise RuntimeError("observer lookup failed")
            if not self.fresh:
                raise TransformException("map absent")
            return object()

    class Listener:
        def __init__(self, buffer, owner, *, spin_thread):
            assert buffer is buffers[0] and owner is node and spin_thread is False
            events.append("listener")
            # Default volatile reader does not inherit samples published before
            # it exists. Transport fixture, not a claim of simulated DDS success.
            if "outcome:resolved" in events:
                queued_tf.clear()
            if mode == "listener_error":
                raise RuntimeError("observer initialization failed")

        def unregister(self):
            events.append("listener-cleanup")
            if mode == "cleanup_error":
                raise RuntimeError("listener cleanup failed")

    class Future:
        def done(self):
            return mode != "rpc_timeout"

        def result(self):
            if mode == "rpc_error":
                raise RuntimeError("RPC failed")
            return response

    class Client:
        def wait_for_service(self, *, timeout_sec):
            assert timeout_sec == 20.0
            return True

        def call_async(self, request):
            assert request.reason == "fixture"
            events.append("call")
            return Future()

    class Node:
        def create_client(self, service_type, service):
            assert service == "/global_localization/trigger"
            return Client()

        def destroy_node(self):
            events.append("node-cleanup")

    node = Node()
    ros = types.ModuleType("rclpy")
    ros.init = lambda **kwargs: events.append("init")
    ros.create_node = lambda *args, **kwargs: node
    ros.ok = lambda: True
    ros.shutdown = lambda: events.append("shutdown")
    ros.spin_until_future_complete = lambda *args, **kwargs: events.append("rpc-spin")

    def spin_once(owner, *, timeout_sec):
        assert owner is node and 0 < timeout_sec <= 0.1
        events.append("tf-spin")
        clock[0] += timeout_sec
        if mode in ("ready", "cleanup_error"):
            buffers[0].fresh = True
        elif queued_tf:
            queued_tf.pop()
            buffers[0].fresh = True

    ros.spin_once = spin_once
    ros_time = types.ModuleType("rclpy.time")
    ros_time.Time = lambda: types.SimpleNamespace(nanoseconds=0)
    tf = types.ModuleType("tf2_ros")
    tf.Buffer, tf.TransformListener, tf.TransformException = Buffer, Listener, TransformException
    service = types.ModuleType("robot_interfaces.srv")
    service.TriggerLocalization = types.SimpleNamespace(Request=type("Request", (), {}))
    for name, module in {"rclpy": ros, "rclpy.time": ros_time, "tf2_ros": tf,
                         "robot_interfaces.srv": service}.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("trigger_tf_fixture", SCRIPTS / "call_global_localization_trigger.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "floor_handoff_requested", lambda: mode == "handoff" and clock[0] >= 0.1)
    monkeypatch.setattr(module, "record_trigger_outcome", lambda state: events.append("outcome:" + state))
    # The real deadline algorithm executes; only the OS monotonic clock and
    # transport callbacks are supplied deterministically.
    monkeypatch.setattr("time.monotonic", lambda: clock[0])
    arguments = [str(SCRIPTS / "call_global_localization_trigger.py"), "--reason", "fixture"]
    if observe:
        arguments += ["--map-odom-wait-sec", "8"]
    monkeypatch.setattr(sys, "argv", arguments)
    return module, events, clock


@pytest.mark.parametrize("mode,expected,marker", [
    ("ready", 0, "true"), ("missing", 0, "false"),
    ("lookup_error", 0, "false"), ("listener_error", 0, "false"),
    ("handoff", 20, None), ("rpc_error", 1, None), ("rejected", 0, None),
    ("rpc_timeout", 124, None), ("cleanup_error", 0, "true"),
    ("queued_pre_accept", 0, "false"),
])
def test_trigger_observes_tf_on_same_node_after_acceptance_without_changing_rpc_result(
        monkeypatch, capsys, mode, expected, marker):
    module, events, clock = trigger_fixture(monkeypatch, mode)
    assert module.main() == expected
    output = capsys.readouterr().out
    assert events.count("call") == 1
    assert events[-2:] == ["node-cleanup", "shutdown"]
    if mode in ("rpc_error", "rpc_timeout", "rejected"):
        assert "buffer" not in events and "listener" not in events and "tf-spin" not in events
    else:
        assert events.index("call") < events.index("outcome:resolved") < events.index("listener")
    if mode not in ("listener_error", "rpc_error", "rpc_timeout", "rejected"):
        assert events.count("listener-cleanup") == 1
    if marker is not None:
        assert "accepted: true" in output and "explicit_sequence=1" in output
        assert f"startup_tf_observed: {marker}" in output.splitlines()
        if mode != "listener_error":
            assert events.index("listener") < events.index("tf-spin")
    else:
        assert "startup_tf_observed:" not in output
    if mode == "missing":
        assert 8.0 <= clock[0] <= 8.1
    if mode == "handoff":
        assert clock[0] <= 0.2
    if mode == "ready":
        assert clock[0] <= 0.1


def test_default_trigger_interface_does_not_create_tf_listener(monkeypatch, capsys):
    module, events, _ = trigger_fixture(monkeypatch, observe=False)
    assert module.main() == 0
    assert "listener" not in events and "buffer" not in events
    assert "startup_tf_observed:" not in capsys.readouterr().out


@pytest.mark.parametrize("marker,expected,cold_probe", [
    ("true", 0, False), ("false", 1, False), ("legacy", 0, True),
    ("message_only", 0, True), ("duplicate", 0, True), ("conflicting", 0, True),
])
@pytest.mark.parametrize("tf_timeout,process_timeout", [("8", "88"), ("0.5", "80.5")])
def test_startup_shell_reuses_only_explicit_tf_proof(
        tmp_path, marker, expected, cold_probe, tf_timeout, process_timeout):
    output = "accepted: true\nmessage: explicit_sequence=1 map->odom ready owner=robot_localization_bridge"
    if marker in ("true", "false"):
        output += f"\nstartup_tf_observed: {marker}"
    elif marker == "message_only":
        output += " startup_tf_observed: true"
    elif marker in ("duplicate", "conflicting"):
        output += "\nstartup_tf_observed: true\nstartup_tf_observed: " + (
            "true" if marker == "duplicate" else "false")
    body = "fixture_output=" + repr(output).replace("\\n", "\n") + r'''
floor_handoff_guard() { return 0; }
initial_global_localization_baseline_sequence=0
NJRH_GLOBAL_LOCALIZATION_TRIGGER_ATTEMPT_TIMEOUT=75
NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT=90
NJRH_INITIAL_LOCALIZATION_MAP_ODOM_WAIT_SEC=8
timeout() {
  echo "outer:$*" >> events
  printf '%s\n' "$fixture_output"
}
wait_for_tf_transform() { echo "cold:$*" >> events; return 0; }
set_localization_ready_failure() { echo "failure:$1" >> events; }
rc=0
trigger_global_localization_for_navigation || rc=$?
echo "rc:$rc" >> events
'''
    body = body.replace("NJRH_INITIAL_LOCALIZATION_MAP_ODOM_WAIT_SEC=8",
                        "NJRH_INITIAL_LOCALIZATION_MAP_ODOM_WAIT_SEC=" + tf_timeout)
    run, events = run_shell(tmp_path, body, filename="run_navigation_runtime_services.sh", functions=(
        "trigger_global_localization_for_navigation", "trigger_output_explicit_relocalization_sequence",
        "trigger_output_reports_map_to_odom_ready"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events[-1] == f"rc:{expected}", events
    assert (f"cold:map odom {tf_timeout}" in events) is cold_probe, events
    assert f"--kill-after=2s {process_timeout}s python3" in events[0]
    assert f"--map-odom-wait-sec {tf_timeout}" in events[0]
    if expected:
        assert "failure:MAP_TO_ODOM_TIMEOUT" in events

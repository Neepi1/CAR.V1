"""Exercise the real trigger main with deterministic ROS transport/clock seams.

These tests check observation-only timing, not real DDS discovery or TF delivery.
"""

import re

import pytest

from test_startup_trigger_tf_warmup import trigger_fixture


TIMING = re.compile(
    r"^\[runtime-overlay\] TRIGGER_TIMING phase=(\w+) "
    r"elapsed_sec=([0-9.]+) result=(\w+)$", re.MULTILINE)


def timing_fixture(monkeypatch, mode="ready", observe=True):
    module, events, clock = trigger_fixture(monkeypatch, mode, observe)

    def timed_external(function, duration):
        def call(*args, **kwargs):
            clock[0] += duration
            return function(*args, **kwargs)
        return call

    ros = module.rclpy
    original_factory = ros.create_node

    def create_node(*args, **kwargs):
        clock[0] += 0.2
        node = original_factory(*args, **kwargs)
        original_client = node.create_client

        def create_client(*args, **kwargs):
            clock[0] += 0.3
            client = original_client(*args, **kwargs)
            monkeypatch.setattr(client, "wait_for_service", timed_external(client.wait_for_service, 0.7))
            original_call = client.call_async

            def call_async(*args, **kwargs):
                clock[0] += 0.4
                future = original_call(*args, **kwargs)
                monkeypatch.setattr(future, "result", timed_external(future.result, 0.6))
                return future

            monkeypatch.setattr(client, "call_async", call_async)
            return client

        monkeypatch.setattr(node, "create_client", create_client)
        monkeypatch.setattr(node, "destroy_node", timed_external(node.destroy_node, 0.8))
        return node

    monkeypatch.setattr(ros, "create_node", create_node)
    monkeypatch.setattr(ros, "init", timed_external(ros.init, 0.1))
    monkeypatch.setattr(ros, "spin_until_future_complete", timed_external(ros.spin_until_future_complete, 3.0))
    monkeypatch.setattr(ros, "shutdown", timed_external(ros.shutdown, 0.9))
    return module, events, clock


def test_timing_preserves_one_accepted_trigger_and_measures_existing_calls(monkeypatch, capsys):
    module, events, clock = timing_fixture(monkeypatch)
    assert module.main() == 0
    output = capsys.readouterr()
    assert output.out.splitlines() == [
        "accepted: true", "message: explicit_sequence=1", "startup_tf_observed: true"]
    assert events.count("call") == 1
    assert events[-3:] == ["listener-cleanup", "node-cleanup", "shutdown"]
    observed = TIMING.findall(output.err)
    assert [phase for phase, _, _ in observed] == [
        "rclpy_init", "node_init", "client_create", "service_discovery",
        "RPC_response", "accepted_TF", "node_destroy", "shutdown", "total"]
    assert [float(duration) for _, duration, _ in observed] == pytest.approx([
        0.1, 0.2, 0.3, 0.7, 4.0, 0.05, 0.8, 0.9, 7.05])
    assert clock[0] == pytest.approx(7.05)


@pytest.mark.parametrize("mode,expected,rpc_result,tf_result", [
    ("rpc_timeout", 124, "timeout", None),
    ("rpc_error", 1, "exception", None),
    ("rejected", 0, "completed", None),
    ("missing", 0, "completed", "unavailable"),
    ("listener_error", 0, "completed", "unavailable"),
    ("lookup_error", 0, "completed", "unavailable"),
    ("cleanup_error", 0, "completed", "observed"),
])
def test_timing_retains_existing_failure_codes_and_no_extra_dispatch(
        monkeypatch, capsys, mode, expected, rpc_result, tf_result):
    module, events, _ = timing_fixture(monkeypatch, mode)
    assert module.main() == expected
    output = capsys.readouterr()
    phases = {phase: result for phase, _, result in TIMING.findall(output.err)}
    assert phases["RPC_response"] == rpc_result
    assert phases.get("accepted_TF") == tf_result
    assert phases["total"] == f"rc_{expected}"
    assert events.count("call") == 1
    assert events[-2:] == ["node-cleanup", "shutdown"]
    if tf_result is None:
        assert "startup_tf_observed:" not in output.out


@pytest.mark.parametrize("when,dispatched,tf_result", [
    ("before_service", False, None),
    ("before_rpc", False, None),
    ("during_tf", True, "handoff"),
])
def test_handoff_records_only_entered_stages_and_keeps_return_20(
        monkeypatch, capsys, when, dispatched, tf_result):
    module, events, _ = timing_fixture(monkeypatch)
    # The existing transport fixture stubs the handoff file; drive its external
    # state at specific call boundaries without replacing the trigger algorithm.
    def handoff():
        if when == "before_service":
            return True
        if when == "before_rpc":
            return "service-ready" in events
        return "tf-spin" in events

    original_factory = module.rclpy.create_node

    def create_node(*args, **kwargs):
        node = original_factory(*args, **kwargs)
        original_client = node.create_client

        def create_client(*args, **kwargs):
            client = original_client(*args, **kwargs)
            original_wait = client.wait_for_service

            def wait_for_service(*args, **kwargs):
                result = original_wait(*args, **kwargs)
                events.append("service-ready")
                return result

            monkeypatch.setattr(client, "wait_for_service", wait_for_service)
            return client

        monkeypatch.setattr(node, "create_client", create_client)
        return node

    monkeypatch.setattr(module.rclpy, "create_node", create_node)
    monkeypatch.setattr(module, "floor_handoff_requested", handoff)
    assert module.main() == 20
    output = capsys.readouterr()
    phases = {phase: result for phase, _, result in TIMING.findall(output.err)}
    assert phases["total"] == "rc_20"
    assert phases.get("accepted_TF") == tf_result
    assert events.count("call") == int(dispatched)
    assert ("RPC_response" in phases) is dispatched
    assert ("service_discovery" in phases) is (when != "before_service")
    assert "startup_tf_observed:" not in output.out
    assert events[-2:] == ["node-cleanup", "shutdown"]


def test_unavailable_service_timing_does_not_dispatch(monkeypatch, capsys):
    module, events, _ = timing_fixture(monkeypatch)
    original_factory = module.rclpy.create_node

    def create_node(*args, **kwargs):
        node = original_factory(*args, **kwargs)
        original_client = node.create_client

        def create_client(*args, **kwargs):
            client = original_client(*args, **kwargs)

            def unavailable(*, timeout_sec):
                assert timeout_sec == 20.0
                return False

            monkeypatch.setattr(client, "wait_for_service", unavailable)
            return client

        monkeypatch.setattr(node, "create_client", create_client)
        return node

    monkeypatch.setattr(module.rclpy, "create_node", create_node)
    assert module.main() == 2
    output = capsys.readouterr()
    phases = {phase: result for phase, _, result in TIMING.findall(output.err)}
    assert phases["service_discovery"] == "unavailable"
    assert phases["total"] == "rc_2"
    assert "RPC_response" not in phases and "accepted_TF" not in phases
    assert "call" not in events
    assert "failure_code=SERVICE_UNAVAILABLE" in output.out


@pytest.mark.parametrize("function,phase", [("init", "rclpy_init"), ("create_node", "node_init")])
def test_initialization_exception_is_not_reclassified_by_timing(monkeypatch, capsys, function, phase):
    module, events, _ = timing_fixture(monkeypatch)

    def fail(*args, **kwargs):
        raise RuntimeError("constructor failed")

    monkeypatch.setattr(module.rclpy, function, fail)
    with pytest.raises(RuntimeError, match="constructor failed"):
        module.main()
    output = capsys.readouterr()
    phases = {name: result for name, _, result in TIMING.findall(output.err)}
    assert phases[phase] == phases["total"] == "exception"
    assert "client_create" not in phases and "call" not in events
    assert "node_destroy" not in phases and "shutdown" not in phases
    assert output.out == ""


@pytest.mark.parametrize("function,phase", [("destroy_node", "node_destroy"), ("shutdown", "shutdown")])
def test_cleanup_exception_is_reported_but_does_not_change_result(monkeypatch, capsys, function, phase):
    module, events, _ = timing_fixture(monkeypatch)

    def fail():
        raise RuntimeError("cleanup failed")

    if function == "shutdown":
        monkeypatch.setattr(module.rclpy, function, fail)
    else:
        original_factory = module.rclpy.create_node

        def create_node(*args, **kwargs):
            node = original_factory(*args, **kwargs)
            monkeypatch.setattr(node, function, fail)
            return node

        monkeypatch.setattr(module.rclpy, "create_node", create_node)
    assert module.main() == 0
    output = capsys.readouterr()
    phases = {name: result for name, _, result in TIMING.findall(output.err)}
    assert phases[phase] == "exception"
    assert phases["total"] == "rc_0"
    assert events.count("call") == 1
    assert "startup_tf_observed: true" in output.out


@pytest.mark.parametrize("stage", ["client_create", "service_discovery"])
def test_client_exception_keeps_existing_failure_response_and_cleanup(monkeypatch, capsys, stage):
    module, events, _ = timing_fixture(monkeypatch)
    original_factory = module.rclpy.create_node

    def fail(*args, **kwargs):
        raise RuntimeError("client failed")

    def create_node(*args, **kwargs):
        node = original_factory(*args, **kwargs)
        original_client = node.create_client

        def create_client(*args, **kwargs):
            if stage == "client_create":
                return fail()
            client = original_client(*args, **kwargs)
            monkeypatch.setattr(client, "wait_for_service", fail)
            return client

        monkeypatch.setattr(node, "create_client", create_client)
        return node

    monkeypatch.setattr(module.rclpy, "create_node", create_node)
    assert module.main() == 1
    output = capsys.readouterr()
    phases = {name: result for name, _, result in TIMING.findall(output.err)}
    assert phases[stage] == "exception"
    assert phases["total"] == "rc_1"
    assert "RPC_response" not in phases and "call" not in events
    assert events[-2:] == ["node-cleanup", "shutdown"]
    assert output.out.splitlines() == [
        "accepted: false", "message: failure_code=GLOBAL_LOCALIZATION_TRIGGER_CLIENT_ERROR client failed"]


def test_diagnostics_use_monotonic_time_and_do_not_create_default_tf_reader(monkeypatch, capsys):
    module, events, _ = timing_fixture(monkeypatch, observe=False)
    monkeypatch.setattr(module.time, "time", lambda: -1000000.0)
    assert module.main() == 0
    output = capsys.readouterr()
    phases = {name: float(duration) for name, duration, _ in TIMING.findall(output.err)}
    assert phases["total"] == pytest.approx(7.0)
    assert "accepted_TF" not in phases and "listener" not in events
    assert "startup_tf_observed:" not in output.out


def test_closed_timing_output_does_not_change_service_result(monkeypatch, capsys):
    module, events, _ = timing_fixture(monkeypatch, observe=False)

    class ClosedPipe:
        def write(self, _message):
            raise BrokenPipeError("diagnostic consumer exited")

        def flush(self):
            raise BrokenPipeError("diagnostic consumer exited")

    with monkeypatch.context() as patch:
        patch.setattr(module.sys, "stderr", ClosedPipe())
        assert module.main() == 0
    assert events.count("call") == 1
    assert events[-2:] == ["node-cleanup", "shutdown"]
    assert capsys.readouterr().out.splitlines() == [
        "accepted: true", "message: explicit_sequence=1"]

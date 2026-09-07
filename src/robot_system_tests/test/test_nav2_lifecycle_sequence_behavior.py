"""Behavior tests for the one-shot lifecycle startup client."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SEQUENCE_PATH = (
    ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "nav2_lifecycle_sequence.py"
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class DelayedFuture:
    def __init__(self, clock: FakeClock, ready_at: float, result: object) -> None:
        self._clock = clock
        self._ready_at = ready_at
        self._result = result

    def done(self) -> bool:
        return self._clock.monotonic() >= self._ready_at

    def result(self) -> object:
        return self._result


class DelayedClient:
    def __init__(self, clock: FakeClock, delay_sec: float, result: object) -> None:
        self._clock = clock
        self._delay_sec = delay_sec
        self._result = result
        self.call_count = 0
        self.removed_futures = []

    def call_async(self, _request: object) -> DelayedFuture:
        self.call_count += 1
        return DelayedFuture(
            self._clock,
            self._clock.monotonic() + self._delay_sec,
            self._result,
        )

    def remove_pending_request(self, future: DelayedFuture) -> None:
        self.removed_futures.append(future)


def load_sequence_module(clock: FakeClock):
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.ok = lambda: True
    fake_rclpy.spin_once = lambda _node, timeout_sec=0.0: clock.sleep(timeout_sec)
    fake_rclpy.init = lambda: None
    fake_rclpy.create_node = lambda _name: object()

    fake_lifecycle_msgs = types.ModuleType("lifecycle_msgs")
    fake_lifecycle_msg = types.ModuleType("lifecycle_msgs.msg")
    fake_lifecycle_srv = types.ModuleType("lifecycle_msgs.srv")
    fake_lifecycle_msg.State = type(
        "State",
        (),
        {
            "PRIMARY_STATE_UNKNOWN": 0,
            "PRIMARY_STATE_UNCONFIGURED": 1,
            "PRIMARY_STATE_INACTIVE": 2,
            "PRIMARY_STATE_ACTIVE": 3,
            "PRIMARY_STATE_FINALIZED": 4,
            "TRANSITION_STATE_CONFIGURING": 10,
            "TRANSITION_STATE_CLEANINGUP": 11,
            "TRANSITION_STATE_SHUTTINGDOWN": 12,
            "TRANSITION_STATE_ACTIVATING": 13,
            "TRANSITION_STATE_DEACTIVATING": 14,
            "TRANSITION_STATE_ERRORPROCESSING": 15,
        },
    )
    fake_lifecycle_msg.Transition = type(
        "Transition",
        (),
        {"TRANSITION_CONFIGURE": 1, "TRANSITION_ACTIVATE": 3},
    )
    fake_lifecycle_srv.ChangeState = type("ChangeState", (), {"Request": type("Request", (), {})})
    fake_lifecycle_srv.GetState = type("GetState", (), {"Request": type("Request", (), {})})

    previous = {
        name: sys.modules.get(name)
        for name in ("rclpy", "lifecycle_msgs", "lifecycle_msgs.msg", "lifecycle_msgs.srv")
    }
    sys.modules["rclpy"] = fake_rclpy
    sys.modules["lifecycle_msgs"] = fake_lifecycle_msgs
    sys.modules["lifecycle_msgs.msg"] = fake_lifecycle_msg
    sys.modules["lifecycle_msgs.srv"] = fake_lifecycle_srv
    try:
        spec = importlib.util.spec_from_file_location("nav2_lifecycle_sequence_under_test", SEQUENCE_PATH)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name, value in previous.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value
    module.time = clock
    return module


def test_late_get_state_response_is_consumed_instead_of_chasing_new_requests(
    monkeypatch, capsys
):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    response = object()
    client = DelayedClient(clock, delay_sec=2.5, result=response)
    monkeypatch.setenv("NJRH_NAV2_LIFECYCLE_GET_STATE_ATTEMPT_SEC", "2.0")

    actual = sequence.call_service(
        object(),
        client,
        object(),
        "/map_server",
        "get_state",
        deadline=6.0,
    )

    assert actual is response
    assert clock.monotonic() < 3.0
    assert client.call_count == 2
    assert len(client.removed_futures) == 1
    assert "lifecycle get_state accepted delayed response node=/map_server" in capsys.readouterr().out


def test_get_state_without_any_response_remains_fail_closed(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    client = DelayedClient(clock, delay_sec=100.0, result=object())
    monkeypatch.setenv("NJRH_NAV2_LIFECYCLE_GET_STATE_ATTEMPT_SEC", "0.2")

    with pytest.raises(TimeoutError, match="get_state timed out for /map_server"):
        sequence.call_service(
            object(),
            client,
            object(),
            "/map_server",
            "get_state",
            deadline=1.0,
        )

    assert clock.monotonic() >= 1.0
    assert len(client.removed_futures) == client.call_count

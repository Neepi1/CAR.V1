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
    fake_rclpy.create_node = lambda _name, **_kwargs: object()

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
    fake_lifecycle_srv.ChangeState = type(
        "ChangeState", (),
        {"Request": staticmethod(lambda: types.SimpleNamespace(
            transition=types.SimpleNamespace(id=0)))},
    )
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


def test_handoff_cancel_waits_for_dispatched_rpc_but_blocks_next_transition(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    response = object()
    client = DelayedClient(clock, delay_sec=0.2, result=response)
    permission = [True]

    def guard():
        if not permission[0]:
            raise ValueError("handoff failed")

    original_call = client.call_async

    def dispatch(request):
        future = original_call(request)
        permission[0] = False  # API cancellation becomes visible after send.
        return future

    monkeypatch.setattr(sequence, "require_transition_permission", guard)
    monkeypatch.setattr(client, "call_async", dispatch)
    assert sequence.call_service(object(), client, object(), "/controller_server",
                                 "configure", deadline=1.0) is response
    assert client.call_count == 1
    with pytest.raises(ValueError, match="handoff failed"):
        sequence.call_service(object(), client, object(), "/controller_server",
                              "activate", deadline=2.0)
    assert client.call_count == 1


class DiscoveringNode:
    """ROS boundary: each new client endpoint needs two seconds to discover."""

    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.created = []
        self.operations = []
        self.events = []

    def create_client(self, _service_type, service_name):
        node = self
        ready_at = self.clock.monotonic() + 2.0
        self.created.append(service_name)
        self.events.append(("create", service_name))

        class Client:
            def wait_for_service(self, timeout_sec):
                node.clock.sleep(min(timeout_sec, max(0.0, ready_at - node.clock.monotonic())))
                return node.clock.monotonic() >= ready_at

            def call_async(self, request):
                node.events.append(("call", service_name))
                operation = getattr(getattr(request, "transition", None), "id", "get_state")
                node.operations.append((service_name, operation))
                response = types.SimpleNamespace(
                    success=True, current_state=types.SimpleNamespace(id=3))
                return DelayedFuture(node.clock, node.clock.monotonic() + 0.05, response)

            def remove_pending_request(self, _future):
                pass

        return Client()


@pytest.mark.parametrize("operation", ["get_state", "change_state"])
def test_repeated_node_operations_reuse_discovered_client(operation, monkeypatch, capsys):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = DiscoveringNode(clock)
    monkeypatch.setattr(sequence, "require_transition_permission", lambda: None)
    if operation == "get_state":
        assert sequence.get_state(node, "/map_server", deadline=10.0) == 3
        assert sequence.get_state(node, "/map_server", deadline=10.0) == 3
    else:
        sequence.configure_node(node, "/map_server", 10.0, 5.0, True)
        sequence.activate_node(node, "/map_server", 10.0, 5.0, True)
    assert node.created == [f"/map_server/{operation}"]
    assert clock.monotonic() < 2.5  # Two real RPCs, only one cold endpoint wait.
    output = capsys.readouterr().out
    assert output.count("lifecycle timing phase=client_create ") == 1
    assert output.count("lifecycle timing phase=client_discovery ") == 2
    assert output.count("lifecycle timing phase=response ") == 2
    assert "elapsed_sec=2.000 result=ready" in output
    assert "elapsed_sec=0.000 result=ready" in output


@pytest.mark.parametrize("configure_all", [False, True])
def test_sequence_precreates_endpoints_without_parallel_transitions(configure_all, monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = DiscoveringNode(clock)
    monkeypatch.setattr(sequence.rclpy, "create_node", lambda _name, **_kwargs: node)
    monkeypatch.setattr(sequence, "require_transition_permission", lambda: None)
    arguments = ["--trust-change-state-response", "planner_server", "controller_server"]
    if configure_all:
        arguments.insert(0, "--configure-all-before-activate")
    assert sequence.main(arguments) == 0
    first_call = next(index for index, event in enumerate(node.events) if event[0] == "call")
    expected_endpoints = {
        f"/{name}/{operation}" for name in ("planner_server", "controller_server")
        for operation in ("get_state", "change_state")}
    assert {service for _, service in node.events[:first_call]} == expected_endpoints
    assert len(node.created) == 4
    planner, controller = "/planner_server/change_state", "/controller_server/change_state"
    expected_operations = ([(planner, 1), (controller, 1), (planner, 3), (controller, 3)]
                           if configure_all else
                           [(planner, 1), (planner, 3), (controller, 1), (controller, 3)])
    assert node.operations == expected_operations
    assert clock.monotonic() < 2.5


def test_main_disables_only_local_rosout_and_parameter_services(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = DiscoveringNode(clock)
    constructions = []

    def create_node(name, **kwargs):
        constructions.append((name, kwargs))
        return node

    monkeypatch.setattr(sequence.rclpy, "create_node", create_node)
    monkeypatch.setattr(sequence, "require_transition_permission", lambda: None)
    assert sequence.main(["--trust-change-state-response", "map_server"]) == 0
    assert constructions == [(
        f"nav2_lifecycle_sequence_{sequence.os.getpid()}",
        {"enable_rosout": False, "start_parameter_services": False},
    )]
    # The one-shot client must still configure and activate the real target
    # through its existing lifecycle service requests, in the same order.
    assert node.operations == [
        ("/map_server/change_state", 1), ("/map_server/change_state", 3),
    ]


def test_client_reuse_does_not_cross_ros_node_contexts():
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    first, second = DiscoveringNode(clock), DiscoveringNode(clock)
    assert sequence.get_state(first, "/map_server", deadline=10.0) == 3
    assert sequence.get_state(second, "/map_server", deadline=10.0) == 3
    assert first.created == second.created == ["/map_server/get_state"]
    assert len(first.operations) == len(second.operations) == 1
    assert clock.monotonic() >= 4.0  # New context must do its own real discovery.


class LifecycleTransportNode:
    """A service boundary where removing a pending RPC really loses its reply."""

    def __init__(self, clock, *, change_delay=6.0, change_success=True,
                 state_replies=((20.0, 2),), get_service_ready_at=0.0, change_error=None):
        self.clock = clock
        self.change_delay = change_delay
        self.change_success = change_success
        self.change_error = change_error
        self.state_replies = state_replies
        self.get_service_ready_at = get_service_ready_at
        self.clients = {}
        self.operations = []
        self.state_call_count = 0
        self.on_dispatch = lambda _operation: None

    def create_client(self, _service_type, service_name):
        node = self
        is_state = service_name.endswith('/get_state')

        class Future(DelayedFuture):
            removed = False

            def done(self):
                return not self.removed and super().done()

            def result(self):
                if isinstance(self._result, Exception):
                    raise self._result
                return super().result()

        class Client:
            def __init__(self):
                self.futures = []

            def service_is_ready(self):
                return not is_state or node.clock.monotonic() >= node.get_service_ready_at

            def wait_for_service(self, timeout_sec):
                if not self.service_is_ready():
                    node.clock.sleep(timeout_sec)
                return self.service_is_ready()

            def call_async(self, request):
                if is_state:
                    operation = 'get_state'
                    index = min(node.state_call_count, len(node.state_replies) - 1)
                    delay, state = node.state_replies[index]
                    node.state_call_count += 1
                    response = types.SimpleNamespace(current_state=types.SimpleNamespace(id=state))
                else:
                    operation = request.transition.id
                    delay = node.change_delay
                    response = node.change_error or types.SimpleNamespace(success=node.change_success)
                node.operations.append(operation)
                node.on_dispatch(operation)
                future = Future(node.clock, node.clock.monotonic() + delay, response)
                self.futures.append(future)
                return future

            def remove_pending_request(self, future):
                future.removed = True

        client = Client()
        self.clients[service_name] = client
        return client

    def unresolved_futures(self):
        return [future for client in self.clients.values() for future in client.futures
                if not future.removed and not future.done()]


def test_configure_retains_late_success_while_get_state_response_is_slow(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock)
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    sequence.configure_node(node, '/planner_server', 30.0, 5.0, True)

    assert 6.0 <= clock.monotonic() < 6.2
    assert node.operations.count(1) == 1
    assert node.state_call_count >= 1
    assert node.unresolved_futures() == []


def test_late_transition_can_complete_while_get_state_service_is_undiscovered(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, get_service_ready_at=100.0)
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    sequence.configure_node(node, '/map_server', 30.0, 5.0, True)

    assert 6.0 <= clock.monotonic() < 6.2
    assert node.operations == [1]
    assert node.unresolved_futures() == []


@pytest.mark.parametrize('operation,target,transition', [('configure_node', 2, 1), ('activate_node', 3, 3)])
def test_state_confirmation_wins_when_transition_response_is_missing(
    monkeypatch, operation, target, transition, capsys
):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=100.0, state_replies=((0.2, target),))
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    getattr(sequence, operation)(node, '/controller_server', 10.0, 0.2, True)

    assert clock.monotonic() < 0.8
    assert node.operations.count(transition) == 1
    assert node.unresolved_futures() == []
    assert 'source=get_state' in capsys.readouterr().out


@pytest.mark.parametrize('operation,initial,target,transition', [
    ('configure_node', 1, 2, 1), ('activate_node', 2, 3, 3),
])
def test_untrusted_late_transition_still_requires_target_state(
    monkeypatch, operation, initial, target, transition, capsys
):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=0.6,
                                  state_replies=((0.0, initial), (2.0, target)))
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    getattr(sequence, operation)(node, '/amcl', 5.0, 0.2, False)

    assert 2.3 <= clock.monotonic() < 2.7
    assert node.operations.count(transition) == 1
    assert node.unresolved_futures() == []
    output = capsys.readouterr().out
    assert 'source=delayed_change_state' not in output
    assert 'source=get_state' in output


@pytest.mark.parametrize('operation,initial,transition', [
    ('configure_node', 1, 1), ('activate_node', 2, 3),
])
def test_old_state_during_pending_transition_does_not_dispatch_it_again(
    monkeypatch, operation, initial, transition
):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=1.0, state_replies=((0.0, initial),))
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    getattr(sequence, operation)(node, '/planner_server', 5.0, 0.2, True)

    assert 1.0 <= clock.monotonic() < 1.2
    assert node.operations.count(transition) == 1
    assert node.state_call_count > 1
    assert node.unresolved_futures() == []


@pytest.mark.parametrize('change_delay,change_success,state_replies,trust', [
    (100.0, True, ((100.0, 2),), True),  # Both RPC paths missing.
    (0.6, False, ((0.0, 1),), True),    # A rejected late response is not readiness.
    (0.6, True, ((0.0, 1),), False),    # Untrusted success cannot replace GetState.
])
def test_unproven_transition_is_bounded_and_cleans_pending_futures(
    monkeypatch, change_delay, change_success, state_replies, trust, capsys
):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=change_delay,
        change_success=change_success, state_replies=state_replies)
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    with pytest.raises(TimeoutError, match='configure confirmation timed out'):
        sequence.configure_node(node, '/map_server', 1.0, 0.2, trust)

    assert 1.2 <= clock.monotonic() < 1.5
    assert node.operations.count(1) == 1
    assert node.unresolved_futures() == []
    assert 'result=ready source=' not in capsys.readouterr().out


def test_handoff_during_pending_transition_still_blocks_next_transition(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock)
    permission = [True]

    def guard():
        if not permission[0]:
            raise ValueError('handoff owns the next transition')

    node.on_dispatch = lambda operation: permission.__setitem__(0, False) if operation == 1 else None
    monkeypatch.setattr(sequence, 'require_transition_permission', guard)

    sequence.configure_node(node, '/planner_server', 30.0, 5.0, True)
    assert 6.0 <= clock.monotonic() < 6.2
    node.state_replies = ((0.0, 2),)
    with pytest.raises(ValueError, match='handoff owns the next transition'):
        sequence.activate_node(node, '/planner_server', 1.0, 0.2, True)

    assert [operation for operation in node.operations if operation != 'get_state'] == [1]
    assert node.unresolved_futures() == []


def test_confirmation_exception_cleans_both_rpc_paths(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=100.0, state_replies=((100.0, 2),))
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    def spin(_node, timeout_sec):
        clock.sleep(timeout_sec)
        if clock.monotonic() >= 0.5:
            raise RuntimeError('fixture executor failure')

    monkeypatch.setattr(sequence.rclpy, 'spin_once', spin)
    with pytest.raises(RuntimeError, match='fixture executor failure'):
        sequence.configure_node(node, '/map_server', 2.0, 0.2, True)

    assert node.operations.count(1) == 1 and node.state_call_count == 1
    assert node.unresolved_futures() == []


def test_confirmation_client_creation_failure_cleans_original_transition(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=100.0)
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)
    create_client = node.create_client

    def fail_state_client(service_type, service_name):
        if service_name.endswith('/get_state'):
            raise RuntimeError('fixture state client creation failed')
        return create_client(service_type, service_name)

    monkeypatch.setattr(node, 'create_client', fail_state_client)
    with pytest.raises(RuntimeError, match='fixture state client creation failed'):
        sequence.configure_node(node, '/map_server', 2.0, 0.2, True)

    assert node.operations == [1]
    assert node.unresolved_futures() == []


def test_transition_reply_exception_before_timeout_does_not_cause_redispatch(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=0.1, state_replies=((0.0, 1),),
                                  change_error=RuntimeError('fixture response failure'))
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    with pytest.raises(TimeoutError):
        sequence.configure_node(node, '/planner_server', 1.0, 0.2, True)

    assert node.operations.count(1) == 1
    assert node.unresolved_futures() == []


def test_confirmation_keeps_older_get_state_reply_eligible(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, change_delay=100.0,
                                  state_replies=((2.5, 2), (100.0, 1)))
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    sequence.configure_node(node, '/planner_server', 4.0, 0.2, True)

    assert 2.8 <= clock.monotonic() < 3.1
    assert node.state_call_count == 2 and node.operations.count(1) == 1
    assert node.unresolved_futures() == []


def test_activate_retains_late_success_and_cleans_state_wait(monkeypatch):
    clock = FakeClock()
    sequence = load_sequence_module(clock)
    node = LifecycleTransportNode(clock, state_replies=((20.0, 3),))
    monkeypatch.setattr(sequence, 'require_transition_permission', lambda: None)

    sequence.activate_node(node, '/planner_server', 30.0, 5.0, True)

    assert 6.0 <= clock.monotonic() < 6.2
    assert node.operations.count(3) == 1
    assert node.state_call_count >= 1
    assert node.unresolved_futures() == []

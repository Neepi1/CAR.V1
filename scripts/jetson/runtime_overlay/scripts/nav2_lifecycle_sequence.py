#!/usr/bin/env python3
"""Robust sequential lifecycle activation for resident Nav2 startup."""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Iterable

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState


def log(message: str) -> None:
    print(f"[runtime-overlay] {message}", flush=True)


def warn(message: str) -> None:
    print(f"[runtime-overlay] {message}", file=sys.stderr, flush=True)


def require_transition_permission() -> None:
    if os.environ.get("NJRH_STARTUP_OWNER_PID") or os.environ.get("NJRH_FLOOR_STARTUP_HANDOFF_NONCE"):
        from floor_startup_handoff import require_startup_side_effect_permission
        require_startup_side_effect_permission()


def normalize_node_name(name: str) -> str:
    name = name.strip()
    if not name:
        raise ValueError("empty lifecycle node name")
    return name if name.startswith("/") else f"/{name}"


def state_label(state_id: int, fallback: str = "") -> str:
    labels = {
        State.PRIMARY_STATE_UNKNOWN: "unknown",
        State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
        State.PRIMARY_STATE_INACTIVE: "inactive",
        State.PRIMARY_STATE_ACTIVE: "active",
        State.PRIMARY_STATE_FINALIZED: "finalized",
        State.TRANSITION_STATE_CONFIGURING: "configuring",
        State.TRANSITION_STATE_CLEANINGUP: "cleaningup",
        State.TRANSITION_STATE_SHUTTINGDOWN: "shuttingdown",
        State.TRANSITION_STATE_ACTIVATING: "activating",
        State.TRANSITION_STATE_DEACTIVATING: "deactivating",
        State.TRANSITION_STATE_ERRORPROCESSING: "errorprocessing",
    }
    return labels.get(state_id, fallback or str(state_id))


def lifecycle_client(node, service_type, service_name: str):
    # The cache belongs to this ROS node/context, never to a later startup or
    # to another helper process. Reusing an endpoint does not cache its state.
    clients = getattr(node, "_njrh_lifecycle_clients", None)
    if clients is None:
        clients = {}
        node._njrh_lifecycle_clients = clients
    key = (service_type, service_name)
    if key not in clients:
        started = time.monotonic()
        clients[key] = node.create_client(service_type, service_name)
        log(f"lifecycle timing phase=client_create service={service_name} "
            f"elapsed_sec={time.monotonic() - started:.3f}")
    return clients[key]


def wait_for_service(client, node_name: str, service_name: str, deadline: float) -> bool:
    started = time.monotonic()
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            log(f"lifecycle timing phase=client_discovery node={node_name} service={service_name} "
                f"elapsed_sec={time.monotonic() - started:.3f} result=ready")
            return True
    warn(f"lifecycle timing phase=client_discovery node={node_name} service={service_name} "
         f"elapsed_sec={time.monotonic() - started:.3f} result=unavailable")
    warn(f"lifecycle service unavailable node={node_name} service={service_name}")
    return False


def remove_pending(client, future) -> None:
    try:
        client.remove_pending_request(future)
    except (AttributeError, KeyError):
        pass


class PendingTransitionTimeout(TimeoutError):
    """Transfer an already-sent transition to its bounded state confirmation."""

    def __init__(self, node_name, operation, client, future):
        super().__init__(f"{operation} timed out for {node_name}")
        self.operation = operation
        self.client = client
        self.future = future


def call_service(node, client, request, node_name: str, operation: str, deadline: float,
                 *, keep_transition_pending: bool = False):
    started = time.monotonic()
    retry_get_state = operation == "get_state"
    get_state_attempt_sec = max(
        0.2,
        float(os.environ.get("NJRH_NAV2_LIFECYCLE_GET_STATE_ATTEMPT_SEC", "2.0")),
    )
    # Fast DDS may return a read-only state response after the retry interval.
    # Keep every outstanding Future eligible so a newer probe cannot hide it.
    pending_futures = []
    transferred = None
    transition_future = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not retry_get_state:
                # Check each undispatched transition. Never abandon an
                # already-sent RPC merely because its owner changed while waiting.
                require_transition_permission()
            future = client.call_async(request)
            pending_futures.append(future)
            if not retry_get_state:
                transition_future = future
            attempt_deadline = deadline
            if retry_get_state:
                attempt_deadline = min(deadline, time.monotonic() + get_state_attempt_sec)
            while rclpy.ok() and time.monotonic() < attempt_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                completed = [candidate for candidate in pending_futures if candidate.done()]
                for candidate in completed:
                    pending_futures.remove(candidate)
                    try:
                        response = candidate.result()
                    except Exception as exc:  # noqa: BLE001 - report and retry within deadline.
                        warn(f"lifecycle {operation} exception node={node_name}: {exc}")
                        continue
                    if retry_get_state and candidate is not future:
                        log(f"lifecycle get_state accepted delayed response node={node_name}")
                    log(f"lifecycle timing phase=response node={node_name} operation={operation} "
                        f"elapsed_sec={time.monotonic() - started:.3f} result=received")
                    return response
            if retry_get_state:
                warn(f"lifecycle get_state attempt timed out node={node_name}; retrying within startup deadline")
            time.sleep(0.1)
        warn(f"lifecycle timing phase=response node={node_name} operation={operation} "
             f"elapsed_sec={time.monotonic() - started:.3f} result=timeout")
        if keep_transition_pending and transition_future is not None and rclpy.ok():
            # A response exception also leaves the sent transition's outcome
            # unproven; it must not reopen the old-state redispatch path.
            transferred = transition_future
            raise PendingTransitionTimeout(node_name, operation, client, transferred)
        raise TimeoutError(f"{operation} timed out for {node_name}")
    finally:
        for unresolved in pending_futures:
            if unresolved is not transferred:
                remove_pending(client, unresolved)


def get_state(node, node_name: str, deadline: float) -> int:
    client = lifecycle_client(node, GetState, f"{node_name}/get_state")
    if not wait_for_service(client, node_name, f"{node_name}/get_state", deadline):
        raise TimeoutError(f"get_state service unavailable for {node_name}")
    response = call_service(node, client, GetState.Request(), node_name, "get_state", deadline)
    return int(response.current_state.id)


def change_state(
    node,
    node_name: str,
    transition_id: int,
    transition_name: str,
    deadline: float,
    response_timeout_sec: float,
) -> None:
    client = lifecycle_client(node, ChangeState, f"{node_name}/change_state")
    if not wait_for_service(client, node_name, f"{node_name}/change_state", deadline):
        raise TimeoutError(f"change_state service unavailable for {node_name}")
    request = ChangeState.Request()
    request.transition.id = transition_id
    response_deadline = min(
        deadline,
        time.monotonic() + max(0.2, response_timeout_sec),
    )
    response = call_service(node, client, request, node_name, transition_name, response_deadline,
                            keep_transition_pending=True)
    if not bool(response.success):
        raise RuntimeError(f"{transition_name} rejected for {node_name}")


def confirm_pending_transition(node, node_name, pending, expected_states, deadline,
                               trust_change_state_response):
    """Observe one sent transition and read-only states; never resend the transition."""
    started = time.monotonic()
    state_client = None
    state_futures = []
    transition_observed = False
    next_state_request = started
    last_state = State.PRIMARY_STATE_UNKNOWN
    try:
        state_client = lifecycle_client(node, GetState, f"{node_name}/get_state")
        attempt_sec = max(0.2, float(os.environ.get(
            "NJRH_NAV2_LIFECYCLE_GET_STATE_ATTEMPT_SEC", "2.0")))
        while rclpy.ok() and time.monotonic() < deadline:
            # Keep spinning even while GetState is undiscovered. A blocking
            # service wait would hide the retained ChangeState response again.
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            if not transition_observed and pending.future.done():
                transition_observed = True
                try:
                    success = bool(pending.future.result().success)
                except Exception as exc:  # noqa: BLE001 - actual state can still prove completion.
                    success = False
                    warn(f"lifecycle delayed {pending.operation} exception node={node_name}: {exc}")
                log(f"lifecycle delayed transition node={node_name} operation={pending.operation} "
                    f"success={str(success).lower()}")
                if success and trust_change_state_response:
                    log(f"lifecycle timing phase=transition_confirmation node={node_name} "
                        f"operation={pending.operation} elapsed_sec={time.monotonic() - started:.3f} "
                        "result=ready source=delayed_change_state")
                    return
            for future in list(state_futures):
                if not future.done():
                    continue
                state_futures.remove(future)
                try:
                    last_state = int(future.result().current_state.id)
                except Exception as exc:  # noqa: BLE001 - read-only retries retain the same deadline.
                    warn(f"lifecycle get_state exception node={node_name}: {exc}")
                    continue
                log(f"lifecycle node={node_name} state={state_label(last_state)} [{last_state}]")
                if last_state in expected_states:
                    log(f"lifecycle timing phase=transition_confirmation node={node_name} "
                        f"operation={pending.operation} elapsed_sec={time.monotonic() - started:.3f} "
                        "result=ready source=get_state")
                    return
                if not state_futures:
                    next_state_request = time.monotonic() + 0.2
            if time.monotonic() >= next_state_request and state_client.service_is_ready():
                state_futures.append(state_client.call_async(GetState.Request()))
                # Match existing read-only retry pacing, retaining older replies.
                next_state_request = time.monotonic() + attempt_sec + 0.1
        warn(f"lifecycle timing phase=transition_confirmation node={node_name} "
             f"operation={pending.operation} elapsed_sec={time.monotonic() - started:.3f} result=timeout")
        raise TimeoutError(f"{pending.operation} confirmation timed out for {node_name}; "
                           f"last={state_label(last_state)} [{last_state}]")
    finally:
        remove_pending(pending.client, pending.future)
        for future in state_futures:
            remove_pending(state_client, future)


def wait_for_state(node, node_name: str, expected: int, deadline: float) -> None:
    last_state = State.PRIMARY_STATE_UNKNOWN
    while time.monotonic() < deadline:
        last_state = get_state(node, node_name, deadline)
        if last_state == expected:
            return
        time.sleep(0.2)
    raise TimeoutError(
        f"{node_name} did not reach {state_label(expected)}; last={state_label(last_state)} [{last_state}]"
    )


def configure_node(
    node,
    node_name: str,
    per_node_timeout_sec: float,
    change_state_response_timeout_sec: float,
    trust_change_state_response: bool,
) -> None:
    def deadline() -> float:
        return time.monotonic() + per_node_timeout_sec

    if trust_change_state_response:
        try:
            log(f"lifecycle configure node={node_name}")
            change_state(
                node,
                node_name,
                Transition.TRANSITION_CONFIGURE,
                "configure",
                deadline(),
                change_state_response_timeout_sec,
            )
            return
        except PendingTransitionTimeout as exc:
            warn(f"lifecycle configure response pending node={node_name}; checking response and state")
            confirm_pending_transition(node, node_name, exc,
                (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE), deadline(), True)
            return
        except Exception as exc:  # noqa: BLE001 - fall back to state inspection below.
            warn(f"lifecycle configure direct transition did not complete node={node_name}: {exc}; checking state")

    current = get_state(node, node_name, deadline())
    log(f"lifecycle node={node_name} state={state_label(current)} [{current}]")
    if current == State.PRIMARY_STATE_ACTIVE:
        return
    if current == State.PRIMARY_STATE_UNCONFIGURED:
        log(f"lifecycle configure node={node_name}")
        try:
            change_state(
                node,
                node_name,
                Transition.TRANSITION_CONFIGURE,
                "configure",
                deadline(),
                change_state_response_timeout_sec,
            )
            if trust_change_state_response:
                return
        except PendingTransitionTimeout as exc:
            warn(f"lifecycle configure response pending node={node_name}; confirming state")
            confirm_pending_transition(node, node_name, exc, (State.PRIMARY_STATE_INACTIVE,),
                                       deadline(), trust_change_state_response)
            return
        except TimeoutError as exc:
            warn(f"lifecycle configure response lost node={node_name}: {exc}; confirming state")
        wait_for_state(node, node_name, State.PRIMARY_STATE_INACTIVE, deadline())
        return
    if current == State.PRIMARY_STATE_INACTIVE:
        return
    if current == State.TRANSITION_STATE_CONFIGURING:
        wait_for_state(node, node_name, State.PRIMARY_STATE_INACTIVE, deadline())
        return
    current = get_state(node, node_name, deadline())
    if current not in (State.PRIMARY_STATE_ACTIVE, State.PRIMARY_STATE_INACTIVE):
        raise RuntimeError(f"{node_name} cannot be configured from {state_label(current)} [{current}]")


def activate_node(
    node,
    node_name: str,
    per_node_timeout_sec: float,
    change_state_response_timeout_sec: float,
    trust_change_state_response: bool,
) -> None:
    def deadline() -> float:
        return time.monotonic() + per_node_timeout_sec

    if trust_change_state_response:
        try:
            log(f"lifecycle activate node={node_name}")
            change_state(
                node,
                node_name,
                Transition.TRANSITION_ACTIVATE,
                "activate",
                deadline(),
                change_state_response_timeout_sec,
            )
            return
        except PendingTransitionTimeout as exc:
            warn(f"lifecycle activate response pending node={node_name}; checking response and state")
            confirm_pending_transition(node, node_name, exc, (State.PRIMARY_STATE_ACTIVE,),
                                       deadline(), True)
            return
        except Exception as exc:  # noqa: BLE001 - fall back to state inspection below.
            warn(f"lifecycle activate direct transition did not complete node={node_name}: {exc}; checking state")

    current = get_state(node, node_name, deadline())
    if current == State.PRIMARY_STATE_ACTIVE:
        return
    if current == State.PRIMARY_STATE_INACTIVE:
        log(f"lifecycle activate node={node_name}")
        try:
            change_state(
                node,
                node_name,
                Transition.TRANSITION_ACTIVATE,
                "activate",
                deadline(),
                change_state_response_timeout_sec,
            )
            if trust_change_state_response:
                return
        except PendingTransitionTimeout as exc:
            warn(f"lifecycle activate response pending node={node_name}; confirming state")
            confirm_pending_transition(node, node_name, exc, (State.PRIMARY_STATE_ACTIVE,),
                                       deadline(), trust_change_state_response)
            return
        except TimeoutError as exc:
            warn(f"lifecycle activate response lost node={node_name}: {exc}; confirming state")
        wait_for_state(node, node_name, State.PRIMARY_STATE_ACTIVE, deadline())
        return
    if current == State.TRANSITION_STATE_ACTIVATING:
        wait_for_state(node, node_name, State.PRIMARY_STATE_ACTIVE, deadline())
        return
    current = get_state(node, node_name, deadline())
    if current != State.PRIMARY_STATE_ACTIVE:
        raise RuntimeError(f"{node_name} cannot be activated from {state_label(current)} [{current}]")


def bringup_node(
    node,
    node_name: str,
    per_node_timeout_sec: float,
    change_state_response_timeout_sec: float,
    trust_change_state_response: bool,
) -> None:
    start = time.monotonic()
    configure_node(
        node,
        node_name,
        per_node_timeout_sec,
        change_state_response_timeout_sec,
        trust_change_state_response,
    )
    log(f"lifecycle configure complete node={node_name} elapsed_sec={time.monotonic() - start:.3f}")
    start = time.monotonic()
    activate_node(
        node,
        node_name,
        per_node_timeout_sec,
        change_state_response_timeout_sec,
        trust_change_state_response,
    )
    log(f"lifecycle activate complete node={node_name} elapsed_sec={time.monotonic() - start:.3f}")


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--per-node-timeout-sec", type=float, default=float(os.environ.get("NJRH_NAV2_LIFECYCLE_NODE_TIMEOUT_SEC", "60")))
    parser.add_argument(
        "--change-state-response-timeout-sec",
        type=float,
        default=float(os.environ.get("NJRH_NAV2_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC", "5.0")),
        help="Short wait for a ChangeState response before confirming the actual lifecycle state.",
    )
    parser.add_argument(
        "--configure-all-before-activate",
        action="store_true",
        help="Configure every node first, then activate in order, matching Nav2 lifecycle manager bringup shape.",
    )
    parser.add_argument(
        "--trust-change-state-response",
        action="store_true",
        help=(
            "Treat successful ChangeState responses as authoritative and use GetState only as a fallback when "
            "a direct transition is rejected or times out."
        ),
    )
    parser.add_argument("nodes", nargs="+", help="Lifecycle nodes to configure/activate in order")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    nodes = [normalize_node_name(name) for name in args.nodes]
    rclpy.init()
    node = rclpy.create_node(
        f"nav2_lifecycle_sequence_{os.getpid()}",
        enable_rosout=False,
        start_parameter_services=False,
    )
    # Advertise all requested endpoints before waiting on the first node. DDS
    # discovery can overlap that node's load; lifecycle transitions stay serial.
    for node_name in nodes:
        lifecycle_client(node, GetState, f"{node_name}/get_state")
        lifecycle_client(node, ChangeState, f"{node_name}/change_state")
    if args.configure_all_before_activate:
        log("lifecycle sequence: configuring all managed nodes before activation")
        for node_name in nodes:
            configure_node(
                node,
                node_name,
                args.per_node_timeout_sec,
                args.change_state_response_timeout_sec,
                args.trust_change_state_response,
            )
        for node_name in nodes:
            activate_node(
                node,
                node_name,
                args.per_node_timeout_sec,
                args.change_state_response_timeout_sec,
                args.trust_change_state_response,
            )
    else:
        for node_name in nodes:
            bringup_node(
                node,
                node_name,
                args.per_node_timeout_sec,
                args.change_state_response_timeout_sec,
                args.trust_change_state_response,
            )
    log("lifecycle sequence: managed nodes are active")
    return 0


if __name__ == "__main__":
    exit_code = 0
    try:
        exit_code = main(sys.argv[1:])
    except Exception as exc:  # noqa: BLE001 - shell caller needs one clear failure line.
        warn(f"lifecycle sequence failed: {exc}")
        exit_code = 1
    # Fast DDS can block indefinitely while a short-lived lifecycle client
    # destroys its node after a lost service response. All requested lifecycle
    # states have already been confirmed here, so let process teardown reclaim
    # the one-shot participant instead of turning success into an outer timeout.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)

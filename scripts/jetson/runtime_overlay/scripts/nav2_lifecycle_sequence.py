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


def wait_for_service(client, node_name: str, service_name: str, deadline: float) -> bool:
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            return True
    warn(f"lifecycle service unavailable node={node_name} service={service_name}")
    return False


def call_service(node, client, request, node_name: str, operation: str, deadline: float):
    retry_get_state = operation == "get_state"
    get_state_attempt_sec = max(
        0.2,
        float(os.environ.get("NJRH_NAV2_LIFECYCLE_GET_STATE_ATTEMPT_SEC", "2.0")),
    )
    while time.monotonic() < deadline:
        future = client.call_async(request)
        attempt_deadline = deadline
        if retry_get_state:
            attempt_deadline = min(deadline, time.monotonic() + get_state_attempt_sec)
        while rclpy.ok() and not future.done() and time.monotonic() < attempt_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if future.done():
            try:
                return future.result()
            except Exception as exc:  # noqa: BLE001 - report and retry within deadline.
                warn(f"lifecycle {operation} exception node={node_name}: {exc}")
        elif retry_get_state:
            warn(f"lifecycle get_state attempt timed out node={node_name}; retrying within startup deadline")
        time.sleep(0.1)
    raise TimeoutError(f"{operation} timed out for {node_name}")


def get_state(node, node_name: str, deadline: float) -> int:
    client = node.create_client(GetState, f"{node_name}/get_state")
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
    client = node.create_client(ChangeState, f"{node_name}/change_state")
    if not wait_for_service(client, node_name, f"{node_name}/change_state", deadline):
        raise TimeoutError(f"change_state service unavailable for {node_name}")
    request = ChangeState.Request()
    request.transition.id = transition_id
    response_deadline = min(
        deadline,
        time.monotonic() + max(0.2, response_timeout_sec),
    )
    response = call_service(node, client, request, node_name, transition_name, response_deadline)
    if not bool(response.success):
        raise RuntimeError(f"{transition_name} rejected for {node_name}")


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
    node = rclpy.create_node(f"nav2_lifecycle_sequence_{os.getpid()}")
    try:
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - shell caller needs one clear failure line.
        warn(f"lifecycle sequence failed: {exc}")
        raise SystemExit(1)

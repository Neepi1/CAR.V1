#!/usr/bin/env python3
"""Exact, single-transaction handoff from cold startup to floor-manager.

This module never changes ROS state. The floor-manager owns the request and
runtime context; startup owns only the acknowledgement. A retained request is
also a write fence: timeout/failure must not resurrect the old startup identity.
"""

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import sys
import time


IDENTITY_FIELDS = ("transaction_id", "building_id", "floor_id", "map_id",
                   "asset_epoch", "asset_digest")
PATH_FIELDS = ("nav_map_yaml", "localizer_map_png", "localizer_params_yaml",
               "keepout_mask_yaml", "speed_mask_yaml")
DEFAULT_REQUEST = "/tmp/njrh_floor_startup_handoff.json"
DEFAULT_ACK = "/tmp/njrh_floor_startup_handoff_ack.json"


def read_json(path):
    with open(path, encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError("handoff/context must be an object")
    return value


def request_pending(path):
    """A new startup ignores history; malformed/nonterminal records fail closed."""
    if not Path(path).exists():
        return False
    try:
        request = read_json(path)
        return not (request.get("schema") == "njrh.floor_startup_handoff.v1"
                    and request.get("version") == 1
                    and request.get("state") in ("committed", "failed"))
    except (OSError, ValueError):
        return True


def same_identity(left, right):
    return all(left.get(key) == right.get(key) for key in IDENTITY_FIELDS)


def load_request(path):
    request = read_json(path)
    if (request.get("schema") != "njrh.floor_startup_handoff.v1"
            or request.get("version") != 1 or request.get("state") not in (
            "requested", "committed", "failed")):
        raise ValueError("unsupported handoff version/state")
    if not isinstance(request.get("request_nonce"), str) or not re.fullmatch(
            r"[A-Za-z0-9_.-]+", request["request_nonce"]):
        raise ValueError("invalid request_nonce")
    for key in IDENTITY_FIELDS[:4]:
        if not isinstance(request.get(key), str) or not re.fullmatch(
                r"[A-Za-z0-9_.-]+", request[key]):
            raise ValueError(f"invalid {key}")
    if type(request.get("asset_epoch")) is not int or request["asset_epoch"] <= 0:
        raise ValueError("invalid asset_epoch")
    if not isinstance(request.get("asset_digest"), str) or not re.fullmatch(
            r"sha256:[0-9a-f]{64}", request["asset_digest"]):
        raise ValueError("invalid canonical digest")
    if (type(request.get("explicit_sequence_baseline")) is not int
            or request["explicit_sequence_baseline"] < 0
            or type(request.get("speed_filter_enabled")) is not bool):
        raise ValueError("invalid baseline/speed-filter contract")
    root = Path(request["asset_root"])
    if (not root.is_absolute() or root.resolve(strict=True) != root
            or root.name != request["map_id"] or root.parent.name != "maps"
            or root.parent.parent.name != request["floor_id"]
            or root.parent.parent.parent.name != request["building_id"]):
        raise ValueError("asset_root must name the immutable exact source bundle")
    manifest = read_json(root / "manifest.json")
    if any(manifest.get(key) != request[key] for key in IDENTITY_FIELDS[1:]):
        raise ValueError("manifest identity does not match handoff")
    for key in PATH_FIELDS:
        candidate = Path(request[key])
        if (not candidate.is_absolute() or not candidate.is_file()
                or candidate.resolve(strict=True) != candidate
                or not candidate.is_relative_to(root)):
            raise ValueError(f"{key} must be an immutable in-bundle file")
    for relative in ("poses.yaml", "filters/binary_mask.yaml"):
        candidate = root / relative
        if not candidate.is_file() or candidate.resolve(strict=True) != candidate:
            raise ValueError(f"missing/noncanonical bundle path: {relative}")
    return request


def target_health_ready(request, health, context):
    """Pending target TF is not ordinary goal-start authorization."""
    return (same_identity(request, context)
            and context.get("state") == "floor_switch_pending"
            and context.get("confirmed") is False
            and all(health.get(key) == request[key] for key in IDENTITY_FIELDS[1:])
            and health.get("transition_active") is True
            and health.get("runtime_context_valid") is False
            and health.get("localizer_ready") is True
            and health.get("bridge_ready") is True
            and health.get("tf_unique") is True
            and type(health.get("localizer_generation")) is int
            and health["localizer_generation"] > 0
            and type(health.get("explicit_relocalization_sequence")) is int
            and health["explicit_relocalization_sequence"] > request["explicit_sequence_baseline"])


def same_request(left, right):
    return (same_identity(left, right)
            and left.get("request_nonce") == right.get("request_nonce")
            and all(left.get(key) == right.get(key) for key in (
                "asset_root", "explicit_sequence_baseline", "speed_filter_enabled", *PATH_FIELDS)))


def atomic_json(path, value):
    destination = Path(path)
    temporary = destination.with_name(destination.name + f".{os.getpid()}.tmp")
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def acknowledgement(request, state, failure="", detail="", evidence=None):
    result = {key: request[key] for key in IDENTITY_FIELDS}
    result.update(schema="njrh.floor_startup_handoff_ack.v1", version=1,
                  request_nonce=request["request_nonce"], state=state,
                  failure=failure, detail=detail, updated_at=time.time(),
                  startup_owner_pid=int(os.environ.get("NJRH_STARTUP_OWNER_PID", os.getpid())),
                  startup_instance=os.environ.get("NJRH_STARTUP_INSTANCE", ""))
    if state == "runtime_ready":
        if (not evidence or not same_identity(request, evidence)
                or evidence.get("request_nonce") != request["request_nonce"]
                or type(evidence.get("explicit_relocalization_sequence")) is not int
                or evidence["explicit_relocalization_sequence"] <= request["explicit_sequence_baseline"]
                or type(evidence.get("localizer_generation")) is not int
                or evidence["localizer_generation"] <= 0):
            raise ValueError("runtime_ready requires this transaction's target TF evidence")
        result.update({key: evidence[key] for key in (
            "explicit_relocalization_sequence", "localizer_generation")})
    return result


def environment(request):
    root = Path(request["asset_root"])
    values = dict(
        NJRH_FLOOR_STARTUP_HANDOFF_NONCE=request["request_nonce"],
        NJRH_RUNTIME_TRANSACTION_ID=request["transaction_id"],
        NJRH_BUILDING_ID=request["building_id"], NJRH_FLOOR_ID=request["floor_id"],
        NJRH_MAP_CONTEXT_BUILDING_ID=request["building_id"],
        NJRH_MAP_CONTEXT_FLOOR_ID=request["floor_id"],
        NJRH_MAP_ID=request["map_id"], NJRH_NAV_MAP_ID=request["map_id"],
        NJRH_MAP_ASSET_EPOCH=str(request["asset_epoch"]),
        NJRH_MAP_ASSET_DIGEST=request["asset_digest"],
        NJRH_CURRENT_FLOOR_ROOT=str(root), NJRH_FLOOR_ASSET_CONTEXT_READY="1",
        NJRH_FLOOR_POSES_YAML=str(root / "poses.yaml"),
        NAV2_BINARY_MASK_YAML=str(root / "filters/binary_mask.yaml"),
        NAV2_MAP_YAML=request["nav_map_yaml"],
        NAV2_LOCALIZER_MAP_PNG=request["localizer_map_png"],
        NAV2_LOCALIZER_MAP_YAML=request["localizer_params_yaml"],
        NAV2_KEEP_OUT_MASK_YAML=request["keepout_mask_yaml"],
        NAV2_SPEED_MASK_YAML=request["speed_mask_yaml"],
        NJRH_ENABLE_SPEED_FILTER=str(request["speed_filter_enabled"]).lower())
    return "\n".join(f"export {key}={shlex.quote(value)}" for key, value in values.items())


def current_request(path):
    request = load_request(path)
    expected_nonce = os.environ.get("NJRH_FLOOR_STARTUP_HANDOFF_NONCE", "")
    if expected_nonce and request["request_nonce"] != expected_nonce:
        raise ValueError("handoff was replaced; old startup must not continue")
    if expected_nonce:
        for field, variable in (("transaction_id", "NJRH_RUNTIME_TRANSACTION_ID"),
                                ("building_id", "NJRH_BUILDING_ID"),
                                ("floor_id", "NJRH_FLOOR_ID"), ("map_id", "NJRH_MAP_ID"),
                                ("asset_epoch", "NJRH_MAP_ASSET_EPOCH"),
                                ("asset_digest", "NJRH_MAP_ASSET_DIGEST")):
            if str(request[field]) != os.environ.get(variable, ""):
                raise ValueError(f"handoff changed adopted {field}")
    if request["state"] == "failed":
        raise ValueError("floor-manager marked this handoff failed")
    return request


def require_current_side_effect(request_path, context_path):
    request = current_request(request_path)
    if request["state"] == "committed":
        context = read_json(context_path)
        if (not same_identity(request, context) or context.get("state") != "ready"
                or context.get("confirmed") is not True):
            raise ValueError("startup no longer owns the current runtime identity")
    return request


def require_startup_side_effect_permission():
    """Inherited only by startup workers; unrelated operator tools are unchanged."""
    nonce = os.environ.get("NJRH_FLOOR_STARTUP_HANDOFF_NONCE", "")
    if not nonce and not os.environ.get("NJRH_STARTUP_OWNER_PID", ""):
        return
    path = os.environ.get("NJRH_FLOOR_STARTUP_HANDOFF_FILE", DEFAULT_REQUEST)
    if nonce:
        require_current_side_effect(path, os.environ.get(
            "NJRH_RUNTIME_MAP_CONTEXT_FILE", "/tmp/njrh_runtime_map_context.json"))
    elif request_pending(path):
        raise ValueError("old startup may not dispatch after handoff was requested")


def wait_target(request_path, context_path, evidence_path):
    request = current_request(request_path)
    # Lazy imports leave the pure protocol testable without ROS installed.
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from robot_interfaces.msg import LocalizationHealth

    rclpy.init(args=None)
    node = rclpy.create_node("floor_startup_target_observer")
    accepted = None

    def observe(message):
        nonlocal accepted
        health = {key: getattr(message, key) for key in (
            *IDENTITY_FIELDS[1:], "transition_active", "runtime_context_valid",
            "localizer_ready", "bridge_ready", "tf_unique", "localizer_generation",
            "explicit_relocalization_sequence")}
        stamp = message.stamp.sec + message.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        try:
            current = current_request(request_path)
            if (same_request(request, current) and 0.0 <= age <= 0.75
                    and target_health_ready(request, health, read_json(context_path))):
                accepted = acknowledgement(request, "adopted")
                accepted.update({key: health[key] for key in (
                    "localizer_generation", "explicit_relocalization_sequence")})
        except (OSError, ValueError, KeyError):
            return

    node.create_subscription(LocalizationHealth, "/localization/floor_health", observe,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.VOLATILE))
    while rclpy.ok() and accepted is None:
        current = current_request(request_path)
        if not same_request(request, current):
            raise ValueError("handoff changed while waiting for pending target TF")
        rclpy.spin_once(node, timeout_sec=0.2)
    if accepted is None:
        return 130
    atomic_json(evidence_path, accepted)
    return 0


def committed(request, context, evidence):
    return (same_identity(request, context) and context.get("state") == "ready"
            and context.get("confirmed") is True
            and context.get("explicit_relocalization_sequence") == evidence.get(
                "explicit_relocalization_sequence")
            and context.get("localizer_generation") == evidence.get("localizer_generation"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("is-requested", "export-env", "export-evidence", "assert-current", "ack",
                                             "wait-target", "wait-commit", "check-trigger-outcome"))
    parser.add_argument("--request", default=os.environ.get(
        "NJRH_FLOOR_STARTUP_HANDOFF_FILE", DEFAULT_REQUEST))
    parser.add_argument("--ack", default=os.environ.get(
        "NJRH_FLOOR_STARTUP_HANDOFF_ACK_FILE", DEFAULT_ACK))
    parser.add_argument("--context", default=os.environ.get(
        "NJRH_RUNTIME_MAP_CONTEXT_FILE", "/tmp/njrh_runtime_map_context.json"))
    parser.add_argument("--evidence", default=os.environ.get(
        "NJRH_FLOOR_STARTUP_HANDOFF_EVIDENCE_FILE", "/tmp/njrh_floor_startup_target_evidence.json"))
    parser.add_argument("--state", choices=("adopted", "runtime_ready", "failed"))
    parser.add_argument("--failure", default="")
    parser.add_argument("--detail", default="")
    args = parser.parse_args()
    if args.command == "is-requested":
        return 0 if request_pending(args.request) else 1
    request = current_request(args.request)
    if args.command == "assert-current":
        require_current_side_effect(args.request, args.context)
    elif args.command == "export-env":
        if request["state"] != "requested":
            raise ValueError("only a requested handoff can be adopted")
        print(environment(request))
    elif args.command == "check-trigger-outcome":
        path = os.environ.get("NJRH_STARTUP_TRIGGER_OUTCOME_FILE", "")
        if path and Path(path).exists() and read_json(path).get("state") != "resolved":
            raise ValueError("previous startup trigger side effect is unknown")
    elif args.command == "export-evidence":
        evidence = read_json(args.evidence)
        acknowledgement(request, "runtime_ready", evidence=evidence)
        for key, field in (("NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE", "explicit_relocalization_sequence"),
                           ("NJRH_RUNTIME_LOCALIZER_GENERATION", "localizer_generation")):
            print(f"export {key}={evidence[field]}")
    elif args.command == "ack":
        evidence = read_json(args.evidence) if args.state == "runtime_ready" else None
        atomic_json(args.ack, acknowledgement(request, args.state, args.failure, args.detail, evidence))
    elif args.command == "wait-target":
        return wait_target(args.request, args.context, args.evidence)
    elif args.command == "wait-commit":
        evidence = read_json(args.evidence)
        while True:
            if not same_request(request, current_request(args.request)):
                raise ValueError("handoff replaced before COMMIT")
            try:
                if committed(request, read_json(args.context), evidence):
                    return 0
            except (OSError, ValueError):
                pass
            time.sleep(0.2)
    return 0


if __name__ == "__main__":
    try:
        result = main()
    except (OSError, ValueError, KeyError) as error:
        print(f"floor-startup handoff refused: {error}", file=sys.stderr, flush=True)
        result = 1
    sys.stdout.flush()
    sys.stderr.flush()
    # Like the existing read-only startup waiter, do not hang in DDS shutdown.
    os._exit(result)

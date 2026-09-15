#!/usr/bin/env python3
"""Small ROS client for the startup global localization trigger service."""

import argparse
from contextlib import contextmanager
import json
import os
import sys
import time

import rclpy
from robot_interfaces.srv import TriggerLocalization


def _timing_log(message):
    # Diagnostics must not replace a service result or cleanup exception when
    # the caller has already closed its stderr pipe.
    try:
        print(message, file=sys.stderr, flush=True)
    except (OSError, ValueError):
        pass


@contextmanager
def _startup_phase(phase):
    started = time.monotonic()
    outcome = {"result": "completed"}
    _timing_log(f"[runtime-overlay] TRIGGER_TIMING_BEGIN phase={phase}")
    try:
        yield outcome
    except BaseException:
        outcome["result"] = "exception"
        raise
    finally:
        elapsed = time.monotonic() - started
        _timing_log(f"[runtime-overlay] TRIGGER_TIMING phase={phase} "
                    f"elapsed_sec={elapsed:.3f} result={outcome['result']}")


def floor_handoff_requested():
    from floor_startup_handoff import request_pending
    return request_pending(os.environ.get(
        "NJRH_FLOOR_STARTUP_HANDOFF_FILE", "/tmp/njrh_floor_startup_handoff.json"))


def record_trigger_outcome(state):
    path = os.environ.get("NJRH_STARTUP_TRIGGER_OUTCOME_FILE", "")
    if path:
        from floor_startup_handoff import atomic_json
        atomic_json(path, {"state": state})


def wait_for_accepted_bridge(node, minimum_sequence: int) -> int:
    """Wait for a later applied correction, not navigation admission; never trigger."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    accepted_sequence = None

    def on_status(message):
        nonlocal accepted_sequence, minimum_sequence
        try:
            status = json.loads(message.data)
            sequence = status.get("last_explicit_relocalization_sequence")
            if type(sequence) is not int or sequence < 0:
                return
            if minimum_sequence < 0:
                # No pre-trigger baseline was observed: wait for the next explicit
                # correction rather than treating an old latched pose as new.
                minimum_sequence = sequence
            if (sequence > minimum_sequence
                    and status.get("has_map_to_odom") is True
                    and status.get("map_to_odom_publisher_owner") == "robot_localization_bridge"
                    and status.get("correction_active") is False
                    and status.get("current_sequence") is not None
                    and status.get("current_sequence") == status.get("target_sequence")):
                accepted_sequence = sequence
        except (ValueError, TypeError, AttributeError):
            return

    subscription = node.create_subscription(
        String, "/localization/bridge_status", on_status,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE))
    while rclpy.ok() and accepted_sequence is None:
        if floor_handoff_requested():
            print("handoff: requested", flush=True)
            return 20
        rclpy.spin_once(node, timeout_sec=0.5)
    if floor_handoff_requested():
        return 20
    if accepted_sequence is None:
        return 130
    print(f"accepted: true\nmessage: later bridge localization accepted explicit_sequence={accepted_sequence}",
          flush=True)
    return 0


def observe_accepted_map_odom(node, buffer, timeout_sec):
    """Observe actual TF from a listener created after service acceptance."""
    try:
        from rclpy.time import Time
        from tf2_ros import TransformException

        if floor_handoff_requested():
            return None
        if buffer is None:
            return False
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok():
            if floor_handoff_requested():
                return None
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            # This node also owns the listener: blocking lookup would prevent
            # its callbacks from filling the buffer on this executor.
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
            if floor_handoff_requested():
                return None
            try:
                buffer.lookup_transform("map", "odom", Time())
                return True
            except TransformException:
                pass
        return False
    except Exception as exc:
        # Observation failure must not rewrite the wrapper's accepted result.
        print(f"startup TF observation failed: {exc}", file=sys.stderr, flush=True)
        return False


def _run_trigger() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", default="/global_localization/trigger")
    parser.add_argument("--reason", required=True)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--map-odom-wait-sec", type=float, default=0.0)
    parser.add_argument("--wait-for-bridge-after", type=int, default=None)
    args = parser.parse_args()

    timeout_sec = max(float(args.timeout_sec), 1.0)
    with _startup_phase("rclpy_init"):
        rclpy.init(args=None)
    with _startup_phase("node_init"):
        node = rclpy.create_node(
            "startup_global_localization_trigger_client", enable_rosout=False, start_parameter_services=False)
    if args.wait_for_bridge_after is not None:
        # The CLI exits directly below, like the lifecycle startup client. Do
        # not let a DDS shutdown hang turn an accepted correction into a wait.
        return wait_for_accepted_bridge(node, args.wait_for_bridge_after)
    tf_buffer = None
    tf_listener = None
    try:
        if floor_handoff_requested():
            return 20
        with _startup_phase("client_create"):
            client = node.create_client(TriggerLocalization, args.service)
        with _startup_phase("service_discovery") as timing:
            service_ready = client.wait_for_service(timeout_sec=timeout_sec)
            timing["result"] = "ready" if service_ready else "unavailable"
        if not service_ready:
            print(f"accepted: false")
            print(f"message: failure_code=SERVICE_UNAVAILABLE service is not available: {args.service}")
            return 2

        request = TriggerLocalization.Request()
        request.reason = args.reason
        if floor_handoff_requested():
            return 20
        record_trigger_outcome("in_flight")
        with _startup_phase("RPC_response") as timing:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
            if not future.done():
                timing["result"] = "timeout"
                record_trigger_outcome("unknown")
                print("accepted: false")
                print(
                    "message: failure_code=GLOBAL_LOCALIZATION_TRIGGER_TIMEOUT "
                    f"request did not complete within {timeout_sec:.1f}s"
                )
                return 124

            response = future.result()
        record_trigger_outcome("resolved")
        print(f"accepted: {'true' if response.accepted else 'false'}")
        print(f"message: {response.message}", flush=True)
        if response.accepted and args.map_odom_wait_sec > 0:
            if floor_handoff_requested():
                return 20
            with _startup_phase("accepted_TF") as timing:
                try:
                    from tf2_ros import Buffer, TransformListener

                    # Reuse the discovered node, but create the volatile TF reader
                    # only now: pre-acceptance samples must not remain in its queue.
                    tf_buffer = Buffer()
                    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
                except Exception as exc:
                    tf_buffer = None
                    print(f"startup TF observer initialization failed: {exc}", file=sys.stderr, flush=True)
                tf_observed = observe_accepted_map_odom(node, tf_buffer, args.map_odom_wait_sec)
                if tf_observed is None or floor_handoff_requested():
                    timing["result"] = "handoff"
                    return 20
                timing["result"] = "observed" if tf_observed else "unavailable"
            print(f"startup_tf_observed: {'true' if tf_observed else 'false'}", flush=True)
        return 0
    except Exception as exc:
        print("accepted: false")
        print(f"message: failure_code=GLOBAL_LOCALIZATION_TRIGGER_CLIENT_ERROR {exc}")
        return 1
    finally:
        if tf_listener is not None:
            try:
                tf_listener.unregister()
            except Exception:
                pass
        try:
            with _startup_phase("node_destroy"):
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                with _startup_phase("shutdown"):
                    rclpy.shutdown()
        except Exception:
            pass


def main() -> int:
    # main-to-return elapsed includes every existing cleanup path, but not
    # Python/module import time before main is entered.
    with _startup_phase("total") as timing:
        result = _run_trigger()
        timing["result"] = f"rc_{result}"
        return result


if __name__ == "__main__":
    result = main()
    if "--wait-for-bridge-after" in sys.argv:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(result)
    sys.exit(result)

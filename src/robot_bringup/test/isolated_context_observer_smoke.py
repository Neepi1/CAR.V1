#!/usr/bin/env python3
"""Real DDS metadata smoke tests for the short-lived startup context observer.

Run ONLY in a private Linux network namespace with its private loopback up:
  ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    python3 isolated_context_observer_smoke.py --worker /candidate/observer \
    --runtime-script /candidate/run_navigation_runtime_services.sh

A disposable --network=none container may explicitly add
--network-none-container. No network namespace is created by this script.
Every topic/service is remapped to a unique fixture name. The only publisher
is a synthetic String bridge status; the Trigger service proves graph presence
and must never receive a request. No ROS CLI, sensor, TF, goal, or velocity is
used. The worker supplies the actual Fast DDS source timestamps.

Each final observation has a 1-second service and 1-second bridge budget. The
whole process has a 42-second work deadline and a 44-second cleanup watchdog.
The sequence-8 case feeds the real worker output through the real shell commit
function with its observation boundary faked; it is not a replacement gate.
"""

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time


def emit(value):
    print(json.dumps(value, sort_keys=True), flush=True)


class Child:
    def __init__(self, command, env, cwd):
        self.command = [str(value) for value in command]
        self.lines = []
        self.process = subprocess.Popen(
            self.command, env=env, cwd=cwd, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, errors="replace")
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            if len(self.lines) < 1500:
                self.lines.append(line.rstrip("\n"))

    def stop(self):
        if self.process.poll() is None:
            # A failed backlog case may still own a SIGSTOP'ed worker.
            self.process.send_signal(signal.SIGCONT)
            self.process.terminate()
            try:
                self.process.wait(timeout=0.3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=0.3)
        else:
            self.process.wait(timeout=0.1)
        self.reader.join(timeout=0.15)
        if not self.reader.is_alive():
            self.process.stdout.close()

    def result(self):
        return {"pid": self.process.pid, "rc": self.process.poll(),
                "command": self.command, "output": list(self.lines)}


class Suite:
    def __init__(self):
        self.started = time.monotonic()
        self.deadline = self.started + 42.0
        self.children = []
        self.finished = threading.Event()
        self.results = []
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _watchdog(self):
        if self.finished.wait(44.0):
            return
        for child in self.children:
            if child.process.poll() is None:
                child.process.kill()
            try:
                child.process.wait(timeout=0.05)
            except subprocess.TimeoutExpired:
                pass
        emit({"result": "failed", "reason": "suite_watchdog", "budget_sec": 44})
        os._exit(124)

    def remaining(self, maximum=0.025):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("42-second work budget exhausted")
        return min(maximum, remaining)

    def launch(self, command, env, cwd):
        self.remaining()
        child = Child(command, env, cwd)
        self.children.append(child)
        return child


def require_isolation(parser, args):
    if sys.platform != "linux":
        parser.error("This smoke must run in an isolated Linux environment")
    for key, value in (("ROS_DOMAIN_ID", "219"), ("ROS_LOCALHOST_ONLY", "1"),
                       ("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")):
        if os.environ.get(key) != value:
            parser.error(f"Require explicit {key}={value}")
    # sysfs may still be mounted against the original network namespace after
    # unshare --net. Query sockets in OUR namespace, never that stale mount.
    if {name for _index, name in socket.if_nameindex()} != {"lo"}:
        parser.error("Private network must contain only lo")
    import fcntl
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        flags = fcntl.ioctl(probe.fileno(), 0x8913, struct.pack("256s", b"lo"))
    if not struct.unpack_from("H", flags, 16)[0] & 1:
        parser.error("Bring up lo only inside the private network namespace first")
    same_as_pid1 = os.stat("/proc/self/ns/net").st_ino == os.stat("/proc/1/ns/net").st_ino
    if same_as_pid1 and not args.network_none_container:
        parser.error("Require a separate network namespace, or explicit --network-none-container")
    for key in ("ROS_DISCOVERY_SERVER", "FASTDDS_DEFAULT_PROFILES_FILE",
                "FASTRTPS_DEFAULT_PROFILES_FILE", "ROS_SECURITY_KEYSTORE"):
        if os.environ.get(key):
            parser.error(f"Unset {key}; production discovery/security profiles are forbidden")
    if not Path(args.worker).is_file() or not os.access(args.worker, os.X_OK):
        parser.error("--worker must name the isolated candidate executable")
    if not Path(args.runtime_script).is_file():
        parser.error("--runtime-script must name the candidate shell source")


def exact_shell_sequence_gate(suite, runtime_script, output, env, cwd):
    """Use the actual commit function; fake only its observer/system boundaries."""
    source = Path(runtime_script).read_text(encoding="utf-8")
    match = re.search(r"(?ms)^commit_runtime_ready_context\(\) \{\n.*?^\}\n", source)
    if match is None:
        raise AssertionError("Candidate no longer exposes commit_runtime_ready_context")
    fixture = r'''set -euo pipefail
SCRIPT_DIR=unused
runtime_ready=0
floor_startup_handoff_active=0
NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE=7
NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK=true
startup_context_observer_pid=''
startup_context_observer_dir=''
context_observer_pid=''
context_observer_dir=''
floor_handoff_guard() { return 0; }
runtime_context_explicit_relocalization_sequence() { printf '7\n'; }
ensure_localization_layer_alive() { echo UNEXPECTED_OWNER_FALLBACK; return 0; }
wait_for_fresh_tf_transform() { echo UNEXPECTED_TF_FALLBACK; return 0; }
write_runtime_map_context() { echo UNEXPECTED_PERSIST; return 0; }
runtime_map_context_matches_current_floor() { echo UNEXPECTED_VERIFY; return 0; }
set_localization_ready_failure() { echo "FAILURE:$1"; }
timeout() { printf '%s\n' "$FIXTURE_OBSERVER_OUTPUT"; return 0; }
'''
    fixture += match.group(0)
    fixture += r'''
rc=0
commit_runtime_ready_context "sequence-eight-fixture" observe_wrapper_service || rc=$?
echo "SHELL_RESULT rc=$rc ready=$runtime_ready"
[[ "$rc" -ne 0 && "$runtime_ready" == 0 ]]
'''
    child_env = dict(env, FIXTURE_OBSERVER_OUTPUT=output)
    child = suite.launch(["bash", "-c", fixture], child_env, cwd)
    child.process.wait(timeout=suite.remaining(2.0))
    child.stop()
    combined = "\n".join(child.lines)
    if (child.process.returncode != 0 or "SHELL_RESULT rc=1 ready=0" not in combined
            or "does not exactly match" not in combined or "UNEXPECTED_" in combined
            or "command not found" in combined or "unbound variable" in combined):
        raise AssertionError({"shell_rc": child.process.returncode, "shell_output": combined})
    return combined


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", required=True)
    parser.add_argument("--runtime-script", required=True)
    parser.add_argument("--network-none-container", action="store_true",
                        help="Explicitly confirm this is a disposable --network=none container")
    args = parser.parse_args()
    require_isolation(parser, args)
    args.worker = str(Path(args.worker).resolve())
    args.runtime_script = str(Path(args.runtime_script).resolve())
    suite = Suite()
    old_handlers = {}

    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.signal(signum, interrupted)
    # Clear inherited startup ownership and marker paths before any ROS import.
    for key in list(os.environ):
        if key.startswith("NJRH_"):
            del os.environ[key]
    os.environ["PYTHONUNBUFFERED"] = "1"
    context = executor = node = None
    summary = None
    original_cwd = Path.cwd()
    try:
        with tempfile.TemporaryDirectory(prefix="isolated_context_observer_") as private:
            root = Path(private)
            os.chdir(root)  # Never load a default Fast DDS XML from the production cwd.
            env = dict(os.environ)
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
            from std_srvs.srv import Trigger

            context = Context()
            rclpy.init(args=[], context=context)
            name = f"private_context_observer_fixture_{os.getpid()}"
            node = Node(name, context=context, enable_rosout=False,
                        start_parameter_services=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            rpc_calls = []

            def spin():
                executor.spin_once(timeout_sec=suite.remaining())

            until = time.monotonic() + 0.2
            while time.monotonic() < until:
                spin()
                others = [candidate for candidate in node.get_node_names() if candidate != name]
                if others:
                    raise RuntimeError(f"Private domain is not empty: {others}")

            def respond(_request, response):
                rpc_calls.append(time.monotonic())
                response.success = True
                return response

            @contextmanager
            def checked_case(case_name):
                started = time.monotonic()
                topic_root = f"/private_context_observer_{os.getpid()}/{case_name}"
                topic, service_name = topic_root + "/bridge", topic_root + "/trigger"
                case_dir = root / case_name
                case_dir.mkdir(mode=0o700)
                marker = case_dir / "commit"
                publisher = node.create_publisher(
                    String, topic, QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                                              durability=DurabilityPolicy.VOLATILE))
                service = node.create_service(Trigger, service_name, respond)
                state = {"name": case_name, "marker": marker, "publisher": publisher,
                         "service": service, "children": [], "details": {}}
                error = None
                try:
                    owner = suite.launch([sys.executable, "-c", "import time; time.sleep(60)"],
                                         env, case_dir)
                    state["owner"] = owner
                    state["children"].append(owner)
                    child_env = dict(env, NJRH_STARTUP_OWNER_PID=str(owner.process.pid),
                                     NJRH_FLOOR_STARTUP_HANDOFF_FILE=str(case_dir / "absent_handoff.json"))
                    worker = suite.launch([
                        args.worker, "--commit-file", marker, "--warmup-wait-sec", "10",
                        "--service-wait-sec", "1", "--bridge-wait-sec", "1",
                        "--minimum-sequence", "6", "--ros-args",
                        "-r", f"/localization/bridge_status:={topic}",
                        "-r", f"/global_localization/trigger:={service_name}"], child_env, case_dir)
                    state["worker"] = worker
                    state["children"].append(worker)
                    ready_deadline = time.monotonic() + 2.5
                    while True:
                        spin()
                        if worker.process.poll() is not None:
                            raise AssertionError({"reason": "worker exited before warmup", **worker.result()})
                        ready = any("CONTEXT_STEP phase=observer_start" in line and
                                    "result=ready" in line for line in worker.lines)
                        if ready and node.count_subscribers(topic) == 1:
                            break
                        if time.monotonic() >= ready_deadline:
                            raise TimeoutError({"reason": "worker/subscription not ready", **worker.result()})
                    if any("service_ready:" in line or "explicit_sequence:" in line
                           for line in worker.lines):
                        raise AssertionError({"reason": "worker emitted proof before commit",
                                              **worker.result()})
                    yield state
                except BaseException as exc:
                    error = exc
                    raise
                finally:
                    for child in reversed(state["children"]):
                        child.stop()
                    if state["service"] is not None:
                        node.destroy_service(state["service"])
                    node.destroy_publisher(publisher)
                    result = {"case": case_name, "status": "passed" if error is None else "failed",
                              "elapsed_sec": round(time.monotonic() - started, 3),
                              "details": state["details"],
                              "error": None if error is None else repr(error),
                              "children": [child.result() for child in state["children"]]}
                    suite.results.append(result)
                    emit(result)

            def publish(state, sequence=7, owner="robot_localization_bridge"):
                state["publisher"].publish(String(data=json.dumps({
                    "last_explicit_relocalization_sequence": sequence,
                    "has_map_to_odom": True, "map_to_odom_publisher_owner": owner})))

            def commit(state):
                cutoff = time.time_ns()
                temporary = state["marker"].with_suffix(".pending")
                temporary.write_text(str(cutoff) + "\n", encoding="ascii")
                temporary.replace(state["marker"])
                state["details"]["commit_cutoff_ns"] = cutoff

            def wait_result(state, sequence=None, owner="robot_localization_bridge", budget=2.6):
                worker = state["worker"]
                deadline = time.monotonic() + budget
                while worker.process.poll() is None:
                    if sequence is not None:
                        publish(state, sequence, owner)
                    spin()
                    if time.monotonic() >= deadline:
                        raise TimeoutError({"reason": "worker final wait exceeded", **worker.result()})
                worker.stop()
                return worker.process.returncode, "\n".join(worker.lines)

            with checked_case("old_backlog_without_fresh_sample") as state:
                worker = state["worker"]
                worker.process.send_signal(signal.SIGSTOP)
                stop_deadline = time.monotonic() + 0.3
                while True:
                    process_state = Path(f"/proc/{worker.process.pid}/status").read_text()
                    if re.search(r"(?m)^State:\s+[Tt]", process_state):
                        break
                    spin()
                    if time.monotonic() > stop_deadline:
                        raise TimeoutError("fixture worker did not enter SIGSTOP")
                for _ in range(20):
                    publish(state)
                    spin()
                state["details"]["old_messages_sent_before_commit"] = 20
                commit(state)
                worker.process.send_signal(signal.SIGCONT)
                rc, output = wait_result(state)
                assert rc == 3 and "service_ready: true" in output and "explicit_sequence:" not in output, output

            for case_name, sequence, owner, expected_rc in (
                ("fresh_sequence_seven", 7, "robot_localization_bridge", 0),
                ("wrong_owner", 7, "fixture_wrong_owner", 3),
                ("old_sequence_six", 6, "robot_localization_bridge", 3),
                ("sequence_eight_parent_rejects", 8, "robot_localization_bridge", 0),
            ):
                with checked_case(case_name) as state:
                    commit(state)
                    rc, output = wait_result(state, sequence, owner)
                    assert rc == expected_rc and "service_ready: true" in output, output
                    if expected_rc == 0:
                        assert f"explicit_sequence: {sequence}" in output, output
                    else:
                        assert "explicit_sequence:" not in output, output
                    if sequence == 8:
                        state["details"]["shell_gate_output"] = exact_shell_sequence_gate(
                            suite, args.runtime_script, output, env, root)

            with checked_case("service_disappeared_before_commit") as state:
                node.destroy_service(state["service"])
                state["service"] = None
                until = time.monotonic() + 0.4
                while time.monotonic() < until:
                    spin()  # Allow endpoint-disposal discovery; no service request is sent.
                commit(state)
                rc, output = wait_result(state, 7)
                assert rc == 2 and "service_ready: false" in output and "explicit_sequence:" not in output, output

            with checked_case("term_during_warmup") as state:
                state["worker"].process.send_signal(signal.SIGTERM)
                rc, output = wait_result(state, budget=1.5)
                assert rc != 0 and "explicit_sequence:" not in output, output

            with checked_case("startup_owner_exited") as state:
                state["owner"].stop()
                rc, output = wait_result(state, budget=1.5)
                assert rc != 0 and "explicit_sequence:" not in output, output

            assert not rpc_calls, "Observer must only inspect the service graph, never call the fixture"
            summary = {"result": "passed", "cases": len(suite.results), "service_rpc_calls": 0,
                       "metadata": "real worker DDS source_timestamp; publisher never fabricates metadata"}
    finally:
        for child in reversed(suite.children):
            child.stop()
        if executor is not None:
            executor.shutdown(timeout_sec=0.5)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        os.chdir(original_cwd)
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        suite.finished.set()
    summary["elapsed_sec"] = round(time.monotonic() - suite.started, 3)
    summary["all_owned_children_reaped"] = all(child.process.poll() is not None for child in suite.children)
    assert summary["all_owned_children_reaped"]
    emit(summary)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BaseException as exc:
        if isinstance(exc, SystemExit):
            raise
        emit({"result": "failed", "error": f"{type(exc).__name__}: {exc}"})
        sys.exit(1)

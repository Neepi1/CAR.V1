#!/usr/bin/env python3
"""Real startup-client checks in an otherwise empty, private ROS domain.

Run inside a disposable --network=none --ipc=private container, with:
  ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  python3 isolated_startup_clients_smoke.py --probe-bin /candidate/probe \
    --baseline-bin /baseline/probe --lifecycle-script /candidate/lifecycle.py

Only synthetic Odometry, /clock and fixture lifecycle/services are used. No
sensor, motion, localization trigger or production service is accessed. JSON
lines include each case's result and captured child output. Each case has a
12-second work budget and a 14-second watchdog, reserving cleanup below 15 s.
The baseline endpoint case deliberately runs last: an exited baseline's old
same-name graph entries must not contaminate the candidate's absence checks.
"""

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback


PARAMETER_SERVICES = {
    "describe_parameters", "get_parameter_types", "get_parameters",
    "list_parameters", "set_parameters", "set_parameters_atomically",
}


def emit(value):
    print(json.dumps(value, sort_keys=True), flush=True)


class Child:
    def __init__(self, command, env):
        self.command = [str(value) for value in command]
        self.lines = []
        self.process = subprocess.Popen(
            self.command, env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, errors="replace")
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            if len(self.lines) < 2000:
                self.lines.append(line.rstrip("\n"))

    def stop(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=0.4)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=0.4)
        else:
            self.process.wait(timeout=0.1)
        self.reader.join(timeout=0.2)
        if not self.reader.is_alive():
            self.process.stdout.close()

    def result(self):
        return {"command": self.command, "pid": self.process.pid,
                "rc": self.process.poll(), "output": list(self.lines)}


class Case:
    def __init__(self, name, child_env):
        self.name = name
        self.env = child_env
        self.started = time.monotonic()
        self.deadline = self.started + 12.0
        self.children = []
        self.details = {}
        self.finished = threading.Event()
        self.watchdog = threading.Thread(target=self._watch, daemon=True)
        self.watchdog.start()

    def _watch(self):
        if self.finished.wait(14.0):
            return
        # Only Popen children owned by this case are touched. No name/pid scan,
        # process-group operation or external ROS action is involved.
        for child in self.children:
            if child.process.poll() is None:
                child.process.kill()
            try:
                child.process.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                pass
        emit({"case": self.name, "status": "failed", "error": "case_watchdog",
              "elapsed_sec": round(time.monotonic() - self.started, 3),
              "children": [child.result() for child in self.children]})
        os._exit(124)

    def remaining(self):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"{self.name}: 12-second work budget exhausted")
        return remaining

    def launch(self, command):
        self.remaining()
        child = Child(command, self.env)
        self.children.append(child)
        return child


@contextmanager
def checked_case(name, env):
    case = Case(name, env)
    error = None
    try:
        yield case
    except BaseException as exc:
        error = exc
        raise
    finally:
        try:
            for child in case.children:
                child.stop()
            elapsed = time.monotonic() - case.started
            emit({"case": name, "status": "passed" if error is None else "failed",
                  "elapsed_sec": round(elapsed, 3), "details": case.details,
                  "error": None if error is None else f"{type(error).__name__}: {error}",
                  "children": [child.result() for child in case.children]})
        finally:
            case.finished.set()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-bin", required=True)
    parser.add_argument("--baseline-bin", required=True)
    parser.add_argument("--lifecycle-script", required=True)
    args = parser.parse_args()
    for key, expected in (("ROS_DOMAIN_ID", "224"), ("ROS_LOCALHOST_ONLY", "1"),
                          ("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")):
        if os.environ.get(key) != expected:
            parser.error(f"Requires explicit {key}={expected}")
    if os.environ.get("ROS_DISCOVERY_SERVER"):
        parser.error("Unset ROS_DISCOVERY_SERVER in the private fixture")
    network_dir = Path("/sys/class/net")
    if not network_dir.is_dir() or {p.name for p in network_dir.iterdir()} != {"lo"}:
        parser.error("Requires a Linux --network=none container (only lo may exist)")
    for value in (args.probe_bin, args.baseline_bin):
        if not Path(value).is_file() or not os.access(value, os.X_OK):
            parser.error(f"Not an executable file: {value}")
    if not Path(args.lifecycle_script).is_file():
        parser.error(f"Missing lifecycle script: {args.lifecycle_script}")

    # These fixtures must not inherit production startup ownership/receipt paths.
    env = {key: value for key, value in os.environ.items() if not key.startswith("NJRH_")}
    env["PYTHONUNBUFFERED"] = "1"

    import rclpy
    from lifecycle_msgs.msg import State, Transition
    from lifecycle_msgs.srv import ChangeState, GetState
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Trigger

    context = Context()
    executor = None
    node = None
    suffix = str(os.getpid())
    fixture_name = "startup_clients_fixture_" + suffix
    root = "/startup_clients_smoke_" + suffix
    old_handlers = {}

    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.signal(signum, interrupted)

    try:
        with checked_case("isolated_fixture_start", env) as case:
            rclpy.init(args=[], context=context)
            node = Node(fixture_name, context=context,
                        enable_rosout=False, start_parameter_services=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=min(0.05, case.remaining()))
                others = [n for n in node.get_node_names() if n != fixture_name]
                if others:
                    raise RuntimeError(f"Domain 224 is occupied: {others}")
            case.details = {"domain": 224, "interfaces": ["lo"]}

        def spin(case, child=None):
            executor.spin_once(timeout_sec=min(0.03, case.remaining()))

        def endpoints(name):
            prefix = "/" + name + "/"
            services = {service[len(prefix):] for service, _ in node.get_service_names_and_types()
                        if service.startswith(prefix)}
            return {
                "node_seen": name in node.get_node_names(),
                "parameter_services": sorted(services & PARAMETER_SERVICES),
                "rosout_publishers": sum(info.node_name == name
                                         for info in node.get_publishers_info_by_topic("/rosout")),
                "parameter_event_publishers": sum(info.node_name == name for info in
                                                  node.get_publishers_info_by_topic("/parameter_events")),
            }

        def no_auxiliary(snapshot, *, cpp):
            return (snapshot["node_seen"] and not snapshot["parameter_services"]
                    and snapshot["rosout_publishers"] == 0
                    and (not cpp or snapshot["parameter_event_publishers"] == 0))

        def wait_exit(case, child, publish=None):
            while child.process.poll() is None:
                if publish is not None:
                    publish()
                spin(case)
            child.stop()
            return child.process.returncode

        def delayed_service_case(binary, baseline):
            label = "baseline_auxiliary_endpoints" if baseline else "candidate_auxiliary_endpoints"
            with checked_case(label, env) as case:
                service = root + ("/baseline_ready" if baseline else "/candidate_ready")
                child = case.launch([binary, "service", service, "9"])
                ready_since = None
                snapshot = None
                while True:
                    if child.process.poll() is not None:
                        raise AssertionError("Probe exited before the delayed fixture service existed")
                    spin(case)
                    snapshot = endpoints("runtime_readiness_probe")
                    case.details["last_observed"] = snapshot
                    expected = (snapshot["node_seen"] and
                                set(snapshot["parameter_services"]) == PARAMETER_SERVICES and
                                snapshot["rosout_publishers"] >= 1 and
                                snapshot["parameter_event_publishers"] >= 1) if baseline else no_auxiliary(snapshot, cpp=True)
                    if expected:
                        if ready_since is None:
                            ready_since = time.monotonic()
                        if time.monotonic() - ready_since >= 0.8:
                            break
                    else:
                        ready_since = None
                case.details["observed_before_service"] = snapshot
                case.details["service_created_at_sec"] = round(time.monotonic() - case.started, 3)

                def respond(_request, response):
                    response.success = True
                    return response

                server = node.create_service(Trigger, service, respond)
                try:
                    assert wait_exit(case, child) == 0, child.result()
                finally:
                    node.destroy_service(server)
                # Allow DDS disposal, but never call a same-name residual entry
                # a duplicate live process. The baseline is deliberately last.
                until = min(time.monotonic() + 0.4, case.deadline)
                while time.monotonic() < until:
                    spin(case)

        delayed_service_case(args.probe_bin, False)

        with checked_case("candidate_lifecycle_reaches_active", env) as case:
            target = root + "/lifecycle_target"
            state = {"id": State.PRIMARY_STATE_UNCONFIGURED, "first": True,
                     "transitions": [], "observed": None}
            case.details = state
            child = None

            def get_state(_request, response):
                if state["first"]:
                    state["first"] = False
                    # Keep the first real RPC in flight while the graph settles.
                    # DDS discovery progresses independently of this executor.
                    until = time.monotonic() + 1.0
                    client_name = f"nav2_lifecycle_sequence_{child.process.pid}"
                    while time.monotonic() < until:
                        case.remaining()
                        state["observed"] = endpoints(client_name)
                        time.sleep(0.05)
                    assert no_auxiliary(state["observed"], cpp=False), state["observed"]
                response.current_state.id = state["id"]
                return response

            def change_state(request, response):
                transition = int(request.transition.id)
                state["transitions"].append(transition)
                if transition == Transition.TRANSITION_CONFIGURE and state["id"] == State.PRIMARY_STATE_UNCONFIGURED:
                    state["id"] = State.PRIMARY_STATE_INACTIVE
                    response.success = True
                elif transition == Transition.TRANSITION_ACTIVATE and state["id"] == State.PRIMARY_STATE_INACTIVE:
                    state["id"] = State.PRIMARY_STATE_ACTIVE
                    response.success = True
                else:
                    response.success = False
                return response

            state_service = node.create_service(GetState, target + "/get_state", get_state)
            change_service = node.create_service(ChangeState, target + "/change_state", change_state)
            try:
                child = case.launch([sys.executable, args.lifecycle_script,
                                     "--per-node-timeout-sec", "8",
                                     "--change-state-response-timeout-sec", "5", target])
                assert wait_exit(case, child) == 0, child.result()
                assert state["id"] == State.PRIMARY_STATE_ACTIVE, state
                assert state["transitions"] == [Transition.TRANSITION_CONFIGURE,
                                                Transition.TRANSITION_ACTIVATE], state
                assert state["observed"] is not None, "GetState fixture was not called"
                case.details = state
            finally:
                node.destroy_service(state_service)
                node.destroy_service(change_service)

        for stale in (False, True):
            with checked_case("stale_odometry_rejected" if stale else "fresh_odometry_accepted", env) as case:
                topic = root + ("/old_odom" if stale else "/fresh_odom")
                publisher = node.create_publisher(Odometry, topic, 10)
                child = case.launch([args.probe_bin, "fresh-header-topic", topic, "7", "0.75", "0.25"])
                published = 0

                def publish():
                    nonlocal published
                    message = Odometry()
                    message.header.frame_id = "isolated_odom"
                    message.child_frame_id = "isolated_base"
                    message.header.stamp = node.get_clock().now().to_msg()
                    if stale:
                        message.header.stamp.sec -= 10
                    message.pose.pose.orientation.w = 1.0
                    publisher.publish(message)
                    published += 1

                try:
                    rc = wait_exit(case, child, publish)
                    assert rc == (1 if stale else 0), child.result()
                    assert published > 0
                    output = "\n".join(child.lines)
                    if stale:
                        assert "last_age=" in output and "fresh stamped topic ready:" not in output, child.result()
                    else:
                        assert "fresh stamped topic ready:" in output, child.result()
                    case.details = {"published": published, "stamp_offset_sec": -10 if stale else 0}
                finally:
                    node.destroy_publisher(publisher)

        with checked_case("python_initial_use_sim_time_override", env) as case:
            simulated = Node("startup_clients_sim_clock_" + suffix, context=context,
                             enable_rosout=False, start_parameter_services=False,
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
            clock_publisher = node.create_publisher(Clock, "/clock", 10)
            executor.add_node(simulated)
            observed = []
            try:
                assert simulated.get_parameter("use_sim_time").value is True
                for seconds in (1234, 5678):
                    while simulated.get_clock().now().nanoseconds != seconds * 1_000_000_000:
                        message = Clock()
                        message.clock.sec = seconds
                        clock_publisher.publish(message)
                        spin(case)
                    observed.append(simulated.get_clock().now().nanoseconds)
                snapshot = endpoints(simulated.get_name())
                assert no_auxiliary(snapshot, cpp=False), snapshot
                case.details = {"observed_ros_clock_ns": observed, "endpoints": snapshot,
                                "scope": "rclpy initial override; not a C++ CLI override test"}
            finally:
                executor.remove_node(simulated)
                simulated.destroy_node()
                node.destroy_publisher(clock_publisher)

        delayed_service_case(args.baseline_bin, True)
    finally:
        # A separate bounded case also covers DDS resource destruction; no
        # successful test is allowed to leave this harness hanging on shutdown.
        with checked_case("fixture_teardown", env):
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    emit({"result": "passed", "cases": 8,
          "note": "Python parameter_events publisher is not claimed disabled"})
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        emit({"result": "failed", "error": f"{type(exc).__name__}: {exc}",
              "traceback": traceback.format_exc()})
        sys.exit(1)

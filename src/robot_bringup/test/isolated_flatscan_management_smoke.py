#!/usr/bin/env python3
"""Opt-in metadata-only ROS smoke. Requires empty domain 222, localhost only.

ROS_DOMAIN_ID=222 ROS_LOCALHOST_ONLY=1 python3 isolated_flatscan_management_smoke.py \
  --guard-bin /candidate/runtime_health_guard --check-bin /candidate/runtime_flatscan_check

No real pipeline/localizer, owner, service calls, TF or motion topics. String
publishers for /scan and /flatscan supply graph identities only. All children,
ROS resources and temporary AMCL status/socket paths are cleaned in finally.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def executable(value):
    path = Path(value).expanduser().resolve()
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError(f"Not executable: {path}")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--guard-bin", required=True)
    parser.add_argument("--check-bin", "--checker-bin", dest="check_bin", required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "222" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("Requires explicit ROS_DOMAIN_ID=222 ROS_LOCALHOST_ONLY=1")
    guard_bin, check_bin = executable(args.guard_bin), executable(args.check_bin)
    # Disallow inherited discovery-server/profile settings that could escape localhost.
    for key in ("ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        if os.environ.get(key):
            parser.error(f"Unset {key} for isolated localhost discovery")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    context = Context()
    executor = node = guard = None
    metrics = {}
    started = time.monotonic()
    old_signals = {}

    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    for signum in (signal.SIGINT, signal.SIGTERM):
        old_signals[signum] = signal.signal(signum, interrupted)
    with tempfile.TemporaryDirectory(prefix="njrh_flat_smoke_") as temp:
        snapshot = Path(temp) / "health.json"
        log_path = Path(temp) / "guard.log"
        env = os.environ.copy()
        env.update({
            "ROS_DOMAIN_ID": "222", "ROS_LOCALHOST_ONLY": "1",
            "NJRH_RUNTIME_MANAGEMENT_ENABLED": "true",
            "NJRH_RUNTIME_HEALTH_FILE": str(snapshot),
            "NJRH_AMCL_RUNTIME_STATUS_FILE": str(Path(temp) / "amcl.env"),
            "NJRH_RUNTIME_LOG_DIR": str(Path(temp) / "logs"),
            "NJRH_RUNTIME_HEALTH_SAMPLE_PERIOD_SEC": "1.0",
            "NJRH_RUNTIME_HEALTH_WRITE_PERIOD_SEC": "1.0",
            "NJRH_RUNTIME_HEALTH_GRAPH_PERIOD_SEC": "5.0",
            "NJRH_RUNTIME_HEALTH_ODOM_NO_UPDATE_TIMEOUT_SEC": "3.0",
            "NJRH_RUNTIME_HEALTH_OBSERVE_TOPIC_MESSAGES": "false",
            "NJRH_RUNTIME_HEALTH_OBSERVE_HEAVY_TOPICS": "false",
            "NJRH_RUNTIME_HEALTH_OBSERVE_TF": "false",
            "NJRH_RUNTIME_HEALTH_OBSERVE_ALL_TF": "false",
            "NJRH_AMCL_RUNTIME_STATUS_HEARTBEAT_SEC": "2.0",
            "NJRH_AMCL_RUNTIME_STATUS_TTL_SEC": "5.0",
            "NJRH_AMCL_STARTUP_HEARTBEAT_GRACE_SEC": "45.0",
        })
        try:
            rclpy.init(args=[], context=context)
            name = f"flatscan_smoke_{os.getpid()}"
            node = Node(name, context=context, enable_rosout=False, start_parameter_services=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            # Only this probe exists before the domain vacancy check completes.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                others = [n for n in node.get_node_names() if n != name]
                if others:
                    raise RuntimeError(f"Domain 222 already occupied; no guard started: {others}")
                for topic in ("/scan", "/flatscan", "/global_localization/flatscan_input_status"):
                    if node.count_publishers(topic) or node.count_subscribers(topic):
                        raise RuntimeError(f"Domain 222 has endpoints on {topic}; refusing to start")

            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            scan = node.create_publisher(String, "/scan", qos)
            flat = node.create_publisher(String, "/flatscan", qos)
            metadata = node.create_publisher(String, "/global_localization/flatscan_input_status", qos)
            state = {"mode": "unavailable", "sequence": 0, "input_sequence": 0,
                     "received": 0.0, "header_stamp": 0.0, "payload": None, "replay": None}
            boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()

            def publish():
                scan.publish(String(data="graph-fixture"))
                flat.publish(String(data="graph-fixture"))
                if state["mode"] == "metadata_stopped":
                    return
                if state["mode"] == "replay":
                    metadata.publish(String(data=state["replay"]))
                    return
                now = time.monotonic()
                if state["mode"] in ("fresh", "header_replay"):
                    state["input_sequence"] += 15
                    state["received"] = now
                if state["mode"] == "fresh":
                    state["header_stamp"] = time.time()
                state["sequence"] += 1
                payload = dict(schema="njrh.flatscan_input.v1", boot_id=boot,
                               generation=f"fixture-{os.getpid()}", sequence=state["sequence"],
                               input_supported=True, available=state["input_sequence"] > 0,
                               input_sequence=state["input_sequence"],
                               received_monotonic_sec=state["received"], emitted_monotonic_sec=now,
                               header_stamp_sec=state["header_stamp"])
                state["payload"] = json.dumps(payload, separators=(",", ":"))
                metadata.publish(String(data=state["payload"]))

            node.create_timer(1.0, publish)
            with log_path.open("w") as log:
                guard = subprocess.Popen([str(guard_bin), "--output", str(snapshot)], env=env,
                                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

            def observe(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    if time.monotonic() - started > 100:
                        raise TimeoutError("Smoke exceeded 100 seconds")
                    if guard.poll() is not None:
                        raise RuntimeError(f"Guard exited: {guard.returncode}")
                    executor.spin_once(timeout_sec=0.05)

            def check(path=snapshot):
                result = subprocess.run([str(check_bin), str(path), "2.0", "10.0"],
                                        env=env, capture_output=True, text=True, timeout=3)
                return result.returncode, result.stdout.strip()

            def wait_for(code, seconds=12):
                end = time.monotonic() + seconds
                result = None
                while time.monotonic() < end:
                    observe(0.3)
                    result = check()
                    if result[0] == code:
                        return result
                raise AssertionError(f"Expected {code}, last check={result}")

            observe(6)
            data = json.loads(snapshot.read_text())
            assert "flatscan_monitor" in data, "Candidate has no management integration"
            assert data["flatscan_monitor"]["graph"]["metadata_publishers"] == 1
            assert check()[0] == 40, "Absent input must remain UNKNOWN"
            metrics["startup_unknown"] = True
            state["mode"] = "fresh"
            metrics["healthy"] = wait_for(0)
            # Graph fixture only: guard must NOT subscribe to either full-stream topic.
            assert node.count_subscribers("/scan") == 0
            assert node.count_subscribers("/flatscan") == 0

            state["mode"] = "input_dropout"
            metrics["dropout_candidate"] = wait_for(50, 15)
            frozen_snapshot = Path(temp) / "frozen_snapshot.json"
            frozen_snapshot.write_text(snapshot.read_text())
            before = check(frozen_snapshot)[1]
            assert check(frozen_snapshot)[1] == before, "Same snapshot must keep the same evidence ID"
            state["mode"] = "fresh"
            wait_for(0)

            state["mode"] = "header_replay"
            replayed_header = state["header_stamp"]
            seq_before_replay = state["input_sequence"]
            metrics["old_frame_new_receives_unknown"] = wait_for(40, 8)
            # Keep receiving the old frame beyond the stream timeout. Neither rx
            # timestamps nor metadata timer sequences can make it healthy again.
            for _ in range(11):
                observe(1)
                assert check()[0] == 40, "Repeated old FlatScan must not be healthy or authorize restart"
            observed = json.loads(snapshot.read_text())["flatscan_monitor"]
            assert observed["input"]["input_sequence"] > seq_before_replay
            assert observed["input"]["header_stamp_sec"] == replayed_header
            assert observed["stamp_progress"]["replay_detected"]
            state["mode"] = "fresh"
            wait_for(0)

            state["replay"] = state["payload"]
            frozen = json.loads(state["replay"])
            state["mode"] = "replay"
            metrics["replay_unknown"] = wait_for(40, 8)
            for _ in range(3):
                observe(1)
                assert check()[0] == 40, "Replayed metadata cannot become producer fault"
            received = json.loads(snapshot.read_text())["flatscan_monitor"]["input"]
            assert received["sequence"] <= frozen["sequence"]
            assert received["emitted_monotonic_sec"] <= frozen["emitted_monotonic_sec"]

            state["mode"] = "fresh"
            wait_for(0)
            state["mode"] = "metadata_stopped"
            metrics["metadata_stopped_unknown"] = wait_for(40, 8)
            for _ in range(3):
                observe(1)
                assert check()[0] == 40, "Missing metadata cannot authorize recovery"
            assert guard.poll() is None
            metrics["guard_survived_all_modes"] = True
            metrics["elapsed_sec"] = round(time.monotonic() - started, 3)
            metrics["note"] = "No owner/pipeline is launched; UNKNOWN is verified never to return candidate code 50"
            print(json.dumps(metrics, indent=2), flush=True)
        except BaseException:
            if log_path.exists():
                print(log_path.read_text(errors="replace")[-8000:], flush=True)
            raise
        finally:
            # Cleanup is test-process-specific, never pkill/systemctl or production paths.
            try:
                if guard is not None and guard.poll() is None:
                    guard.terminate()
                    try:
                        guard.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        guard.kill()
                        guard.wait(timeout=5)
            finally:
                try:
                    if executor is not None:
                        executor.shutdown(timeout_sec=2)
                finally:
                    try:
                        if node is not None:
                            node.destroy_node()
                    finally:
                        try:
                            if context.ok():
                                context.shutdown()
                        finally:
                            for signum, handler in old_signals.items():
                                signal.signal(signum, handler)


if __name__ == "__main__":
    main()

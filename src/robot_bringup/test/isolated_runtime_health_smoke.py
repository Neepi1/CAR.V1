#!/usr/bin/env python3
"""Opt-in ROS smoke test: synthetic odometry only, domain 223, localhost only.

Example (after sourcing the candidate ROS install):
  ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1 python3 isolated_runtime_health_smoke.py \
    --guard-bin /candidate/runtime_health_guard --check-bin /candidate/runtime_health_check

Only the guard binary is launched, never a shell wrapper or runtime owner. No
cmd_vel, TF, service calls, systemctl, or robot connection is used. The publisher
belongs to this process; both it and the child guard are destroyed in finally.
"""

import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time


def executable(value, label):
    if not value:
        raise RuntimeError(f"Specify --{label}-bin or NJRH_RUNTIME_HEALTH_{label.upper()}_BIN")
    path = Path(value).expanduser().resolve()
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(f"Not an executable: {path}")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--guard-bin", default=os.environ.get("NJRH_RUNTIME_HEALTH_GUARD_BIN"))
    parser.add_argument("--check-bin", default=os.environ.get("NJRH_RUNTIME_HEALTH_CHECK_BIN"))
    args = parser.parse_args()
    # Refuse to initialize ROS on the robot's normal domain, even accidentally.
    if os.environ.get("ROS_DOMAIN_ID") != "223" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("Requires explicit ROS_DOMAIN_ID=223 and ROS_LOCALHOST_ONLY=1")
    guard_bin = executable(args.guard_bin, "guard")
    check_bin = executable(args.check_bin or str(guard_bin.with_name("runtime_health_check")), "check")

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    context = Context()
    executor = None
    node = None
    guard = None
    state = {"mode": "fresh", "sent": 0, "last_stamp": None, "frozen_stamp": None}
    metrics = {}
    with tempfile.TemporaryDirectory(prefix="isolated_runtime_health_") as temp:
        snapshot_path = Path(temp) / "health.json"
        log_path = Path(temp) / "guard.log"
        env = os.environ.copy()
        # Fixed test-only settings prevent inherited production observer toggles.
        env.update({
            "NJRH_RUNTIME_HEALTH_FILE": str(snapshot_path),
            "NJRH_RUNTIME_HEALTH_SAMPLE_PERIOD_SEC": "1.0",
            "NJRH_RUNTIME_HEALTH_WRITE_PERIOD_SEC": "1.0",
            "NJRH_RUNTIME_HEALTH_GRAPH_PERIOD_SEC": "5.0",
            "NJRH_RUNTIME_HEALTH_ODOM_NO_UPDATE_TIMEOUT_SEC": "3.0",
            "NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC": "0.75",
            "NJRH_RUNTIME_HEALTH_OBSERVE_TOPIC_MESSAGES": "false",
            "NJRH_RUNTIME_HEALTH_OBSERVE_HEAVY_TOPICS": "false",
            "NJRH_RUNTIME_HEALTH_OBSERVE_TF": "false",
            "NJRH_RUNTIME_HEALTH_OBSERVE_ALL_TF": "false",
            "NJRH_COMMON_LOCAL_STATE_HEALTH_MONITOR": "false",
            "NJRH_COMMON_RANGER_CHASSIS_HEALTH_EXIT_ON_LOSS": "false",
        })
        try:
            rclpy.init(args=[], context=context)
            executor = SingleThreadedExecutor(context=context)
            preflight_name = f"health_smoke_preflight_{os.getpid()}"
            node = Node(preflight_name, context=context, enable_rosout=False)
            executor.add_node(node)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                others = [name for name in node.get_node_names() if name != preflight_name]
                if others:
                    raise RuntimeError(f"Isolation domain 223 is already occupied: {others}")
            executor.remove_node(node)
            node.destroy_node()
            node = None
            node = Node("robot_local_state", context=context, enable_rosout=False)
            executor.add_node(node)
            publisher = node.create_publisher(
                Odometry, "/local_state/odometry",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

            def publish():
                if state["mode"] == "paused":
                    return
                msg = Odometry()
                msg.header.frame_id = "odom"
                msg.child_frame_id = "base_link"
                msg.pose.pose.orientation.w = 1.0
                msg.header.stamp = (
                    state["frozen_stamp"] if state["mode"] == "replay"
                    else node.get_clock().now().to_msg()
                )
                state["last_stamp"] = copy.deepcopy(msg.header.stamp)
                publisher.publish(msg)
                state["sent"] += 1

            node.create_timer(0.02, publish)
            with log_path.open("w", encoding="utf-8") as log:
                guard = subprocess.Popen(
                    [str(guard_bin), "--output", str(snapshot_path)],
                    stdout=log, stderr=subprocess.STDOUT, env=env,
                )

            def latest():
                if guard.poll() is not None:
                    raise RuntimeError(f"Guard exited with code {guard.returncode}")
                try:
                    return json.loads(snapshot_path.read_text(encoding="utf-8"))
                except FileNotFoundError:
                    return None

            def diagnostic():
                result = subprocess.run(
                    [str(check_bin), str(snapshot_path), "2.0", "diagnostic"],
                    capture_output=True, text=True, timeout=5, env=env,
                )
                return result.returncode, result.stdout + result.stderr

            def observe(seconds):
                samples = []
                previous_sequence = None
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                    snapshot = latest()
                    if snapshot and snapshot["sequence"] != previous_sequence:
                        samples.append(snapshot)
                        previous_sequence = snapshot["sequence"]
                return samples

            def wait_ready():
                deadline = time.monotonic() + 12.0
                last = "no snapshot"
                while time.monotonic() < deadline:
                    samples = observe(0.25)
                    if samples:
                        rc, last = diagnostic()
                        if rc == 0:
                            # The CLI may read a newer atomic snapshot than
                            # observe() did just before recovery was published.
                            recovered = latest()
                            if recovered and recovered["odom_watch"]["no_update_age_sec"] < 3.0:
                                return recovered
                raise AssertionError(f"Did not become ready: {last}")

            first = wait_ready()
            before_sent = state["sent"]
            start = time.monotonic()
            samples = observe(6.0)
            last = samples[-1]
            elapsed = time.monotonic() - start
            topic = "/local_state/odometry"
            count = last["topics"][topic]["message_count"] - first["topics"][topic]["message_count"]
            sequence_delta = last["sequence"] - first["sequence"]
            published = state["sent"] - before_sent
            assert 4 <= count <= 8, f"Expected about 1Hz takes, got {count} in {elapsed:.2f}s"
            assert 4 <= sequence_delta <= 8, f"Unexpected snapshot rate: {sequence_delta}"
            assert published >= 200, f"Synthetic 50Hz publisher starved: {published} messages"
            for snapshot in samples:
                assert snapshot["implementation"] == "cpp"
                assert snapshot["sample_period_sec"] == 1.0
                assert snapshot["message_count_semantics"] == "sampled_messages_not_publisher_rate"
                assert snapshot["odom_watch"]["no_update_age_sec"] < 3.0
                assert snapshot["odom_watch"]["seen_valid"]
            assert diagnostic()[0] == 0
            metrics["steady"] = {"elapsed_sec": elapsed, "published": published,
                                 "sampled": count, "snapshot_delta": sequence_delta}

            for mode in ("paused", "replay"):
                state["frozen_stamp"] = copy.deepcopy(state["last_stamp"])
                state["mode"] = mode
                samples = observe(5.2)
                rc, text = diagnostic()
                assert rc == 56, f"{mode}: expected fault 56 after >4s, got {rc}: {text}"
                assert samples[-1]["odom_watch"]["no_update_age_sec"] >= 3.0
                metrics[mode] = {"rc": rc, "no_update_age_sec":
                                 samples[-1]["odom_watch"]["no_update_age_sec"]}
                state["mode"] = "fresh"
                recovered = wait_ready()
                assert recovered["odom_watch"]["no_update_age_sec"] < 3.0
                metrics[mode]["recovery_rc"] = 0
            print(json.dumps({"ok": True, "domain_id": 223, "metrics": metrics}, indent=2))
            return 0
        except BaseException:
            if log_path.exists():
                print(log_path.read_text(encoding="utf-8", errors="replace"))
            raise
        finally:
            try:
                if guard is not None:
                    if guard.poll() is None:
                        guard.terminate()
                    try:
                        guard.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        guard.kill()
                        guard.wait(timeout=5)
            finally:
                try:
                    if executor is not None:
                        try:
                            if node is not None:
                                executor.remove_node(node)
                        finally:
                            executor.shutdown(timeout_sec=5)
                            executor = None
                finally:
                    try:
                        if node is not None:
                            node.destroy_node()
                    finally:
                        if context.ok():
                            rclpy.shutdown(context=context)


if __name__ == "__main__":
    raise SystemExit(main())

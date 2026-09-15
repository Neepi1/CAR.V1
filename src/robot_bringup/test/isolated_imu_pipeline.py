#!/usr/bin/env python3
"""Differential/oracle test for the two-node IMU pipeline host.

Run in a disposable Docker container with --network=none --ipc=private and no
devices or production processes. Source the existing Humble overlay first:

  python3 src/robot_bringup/test/isolated_imu_pipeline.py --isolated \
    --pipeline-bin /candidate/imu_pipeline_node \
    --remap-bin /baseline/imu_axis_remap_node \
    --filter-bin /baseline/imu_gyro_bias_filter_node --report-dir /tmp/imu-test

Domain 197, loopback-only DDS, unique test topics and private parameter files
are mandatory. Only test fixtures preserve source stamps and widen freshness
to 30 s; production configuration is not loaded or changed. One input remains
in flight at a time. This is correctness/lifecycle evidence, NOT a CPU or
400 Hz throughput benchmark. Keep the sibling numeric-oracle source available.
"""

import argparse
from collections import deque
from copy import deepcopy
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import traceback


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isolated", action="store_true")
    for name in ("pipeline-bin", "remap-bin", "filter-bin", "report-dir"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if not __debug__:
        parser.error("assertions are required; do not run python -O")
    if not args.isolated or not Path("/.dockerenv").is_file():
        parser.error("--isolated and a disposable Docker container are required")
    net = Path("/sys/class/net")
    if not net.is_dir() or {item.name for item in net.iterdir()} != {"lo"}:
        parser.error("refusing ROS startup: only lo is allowed (--network=none)")
    for name in ("pipeline_bin", "remap_bin", "filter_bin"):
        path = getattr(args, name).resolve()
        if not path.is_file() or not os.access(path, os.X_OK):
            parser.error(f"--{name.replace('_', '-')} must be executable")
        setattr(args, name, path)
    args.report_dir = args.report_dir.resolve()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    if (args.report_dir / "report.json").exists():
        parser.error("report.json already exists; use a fresh --report-dir")
    profile = args.report_dir / "isolated_dds.xml"
    profile.write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor>
    <transport_id>imu_test_loopback</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="imu_test" is_default_profile="true"><rtps>
    <userTransports><transport_id>imu_test_loopback</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles>
''', encoding="utf-8")
    for key in ("ROS_DISCOVERY_SERVER", "RMW_FASTRTPS_USE_QOS_FROM_XML"):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID="197", ROS_LOCALHOST_ONLY="1",
                      RMW_IMPLEMENTATION="rmw_fastrtps_cpp", SKIP_DEFAULT_XML="1",
                      FASTDDS_BUILTIN_TRANSPORTS="UDPv4",
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                      FASTDDS_DEFAULT_PROFILES_FILE=str(profile),
                      ROS_LOG_DIR=str(args.report_dir / "ros_logs"))
    return args


def load_oracle():
    path = Path(__file__).resolve().parents[2] / "robot_local_state/test/isolated_imu_equivalence.py"
    spec = importlib.util.spec_from_file_location("imu_numeric_oracle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def params_file(path, node_name, parameters):
    # JSON scalars/arrays are valid YAML; no extra PyYAML dependency is needed.
    text = node_name + ":\n  ros__parameters:\n"
    text += "".join(f"    {key}: {json.dumps(value)}\n" for key, value in parameters.items())
    path.write_text(text, encoding="utf-8")
    return path


class PipelinePair:
    """Standalone baseline and candidate host, with one common raw publisher."""

    def __init__(self, node, args, mode, report):
        self.node, self.mode, self.report = node, mode, report
        self.with_filter = mode != "remap_only"
        self.prefix = f"/imu_pipeline_test_{os.getpid()}_{mode}"
        self.frame = f"imu_pipeline_frame_{os.getpid()}_{mode}"
        self.base_frame = self.frame + "_base"
        self.canonical_topics = [self.prefix + f"/canonical_{i}" for i in range(2)]
        self.corrected_topics = [self.prefix + f"/corrected_{i}" for i in range(2)]
        self.canonical = [deque(), deque()]
        self.corrected = [deque(), deque()]
        self.bias = [{}, {}]
        self.processes, self.logs, self.publishers, self.subscriptions = [], [], [], []
        self.host = None
        self.observer = None
        self.last_stamp = 0
        self.input_count = 0
        self.remap_q = oracle.normalized((0.13, -0.21, 0.32, 0.91))
        self.remap_rotation = oracle.matrix(self.remap_q)
        self.tf_q = oracle.normalized((-0.22, 0.31, -0.17, 0.86))
        self.qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.directory = args.report_dir / mode
        self.directory.mkdir(exist_ok=False)
        try:
            self.input = self.publisher(Imu, "raw", self.qos)
            self.odom = self.publisher(Odometry, "odom", self.qos)
            self.tf = self.publisher(TFMessage, "tf_static", QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.subscribe(Imu, self.canonical_topics[0], self.canonical[0].append, self.qos)
            if self.with_filter:
                for index in range(2):
                    self.subscribe(Imu, self.corrected_topics[index],
                                   self.corrected[index].append, qos_profile_sensor_data)
                    self.subscribe(Vector3Stamped, self.prefix + f"/bias_{index}",
                                   lambda msg, i=index: self.bias[i].update(
                                       {oracle.stamp_key(msg): msg}), self.qos)
            remaps, filters = [], []
            for index in range(2):
                remaps.append(params_file(self.directory / f"remap_{index}.yaml", "imu_axis_remap", {
                    "input_topic": self.prefix + "/raw",
                    "output_topic": self.canonical_topics[index],
                    "output_frame_id": self.frame,
                    "rotation_matrix": [v for row in self.remap_rotation for v in row],
                    "override_angular_velocity_covariance": not self.with_filter,
                    "angular_velocity_covariance_diagonal": [0.11, 0.22, 0.33],
                    "mark_orientation_unavailable": not self.with_filter,
                }))
                filters.append(params_file(self.directory / f"filter_{index}.yaml", "imu_gyro_bias_filter", {
                    "imu_topic": self.canonical_topics[index],
                    "output_imu_topic": self.corrected_topics[index],
                    "bias_topic": self.prefix + f"/bias_{index}",
                    "odom_topic": self.prefix + "/odom",
                    "cmd_vel_topic": self.prefix + "/unused_cmd",
                    "use_cmd_vel_stationary": False,
                    "stationary_required_sec": 0.0,
                    "odom_timeout_sec": 30.0,
                    "corrected_output_preserve_source_stamp": True,
                    "bias_publish_preserve_source_stamp": True,
                    "corrected_output_max_source_age_sec": 30.0,
                    "transform_output_to_target_frame": True,
                    "output_target_frame": self.base_frame,
                    "override_output_angular_velocity_z_covariance": True,
                    "output_angular_velocity_z_covariance": 0.007,
                }))
            ros_args = ["--ros-args", "-r", f"/tf:={self.prefix}/tf",
                        "-r", f"/tf_static:={self.prefix}/tf_static"]
            self.start("baseline_remap", [str(args.remap_bin), *ros_args,
                                          "--params-file", str(remaps[0])])
            if self.with_filter:
                self.start("baseline_filter", [str(args.filter_bin), *ros_args,
                                               "--params-file", str(filters[0])])
            host = [str(args.pipeline_bin), "--remap-params", str(remaps[1]),
                    "--filter-params", str(filters[1])]
            if mode == "dds_only":
                host.append("--dds-only")
            if not self.with_filter:
                host.append("--without-bias-filter")
            self.host = self.start("host", host + ros_args)
            self.wait(lambda: self.input.get_subscription_count() == 2 and all(
                self.node.count_publishers(topic) == 1 for topic in self.canonical_topics),
                "raw/canonical endpoint discovery", 15.0)
            if self.with_filter:
                self.wait(lambda: all(self.node.count_publishers(topic) == 1
                                      for topic in self.corrected_topics)
                          and self.tf.get_subscription_count() == 2
                          and self.odom.get_subscription_count() == 2,
                          "filter/TF/odom endpoint discovery", 15.0)
                self.wait(lambda: self.node.count_subscribers(self.canonical_topics[1]) == 1,
                          "canonical has only the internal filter")
                self.transform(self.tf_q)
                self.stationary(False)
            else:
                self.wait(lambda: self.node.count_subscribers(self.canonical_topics[1]) == 0,
                          "remap-only has no filter subscription")
                self.observe(True)
                assert all(self.node.count_publishers(topic) == 0 for topic in self.corrected_topics)
                assert self.tf.get_subscription_count() == 0
                self.passed("remap-only has no filter outputs or TF listener")
        except BaseException:
            self.close()
            raise

    def passed(self, label):
        self.report["checks"].append({"mode": self.mode, "check": label, "status": "passed"})
        print(f"PASS {self.mode}: {label}", flush=True)

    def publisher(self, msg_type, suffix, qos):
        publisher = self.node.create_publisher(msg_type, self.prefix + "/" + suffix, qos)
        self.publishers.append(publisher)
        return publisher

    def subscribe(self, msg_type, topic, callback, qos):
        subscription = self.node.create_subscription(msg_type, topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription

    def start(self, label, command):
        log = (self.directory / (label + ".log")).open("w", encoding="utf-8")
        self.logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        self.report["commands"].append({"mode": self.mode, "role": label, "argv": command})
        return process

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            for process in self.processes:
                assert process.poll() is None, f"child {process.pid} exited early: {process.returncode}"
            rclpy.spin_once(self.node, timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))

    def wait(self, predicate, label, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, f"{self.mode}: timeout waiting for {label}"
            self.spin(0.01)

    def observe(self, enable):
        assert enable == (self.observer is None), "observer transition must change state"
        if enable:
            self.observer = self.subscribe(Imu, self.canonical_topics[1], self.canonical[1].append, self.qos)
        else:
            self.node.destroy_subscription(self.observer)
            self.subscriptions.remove(self.observer)
            self.observer = None
        wanted = int(self.with_filter) + int(enable)
        self.wait(lambda: self.node.count_subscribers(self.canonical_topics[1]) == wanted,
                  f"external canonical observer {'join' if enable else 'leave'}")
        self.spin(0.15)
        assert not any(self.canonical) and not any(self.corrected), "observer change replayed an old sample"
        self.passed(f"external canonical observer {'joined' if enable else 'left'}")

    def sample(self, variant=0, gyro=None):
        msg = Imu()
        self.last_stamp = max(self.last_stamp + 1_000_000, time.time_ns() - 50_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(self.last_stamp, 1_000_000_000)
        msg.header.frame_id = "fixture_raw"
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (0.2, -0.1, 0.3, 1.7)
        oracle.set_xyz(msg.angular_velocity, gyro if gyro is not None else (0.21 + variant * 0.01, -0.34, 0.57))
        oracle.set_xyz(msg.linear_acceleration, (1.2, -2.3 + variant * 0.01, 9.4))
        for index, field in enumerate(("orientation_covariance", "angular_velocity_covariance",
                                       "linear_acceleration_covariance")):
            scale = (index + 1) * (variant + 1) * 0.13
            setattr(msg, field, [scale * v for v in (4.0, 0.3, -0.2, 0.3, 5.0, 0.4, -0.2, 0.4, 6.0)])
        return msg

    def expected_remap(self, msg):
        result = oracle.rotate_imu(msg, self.remap_rotation, self.frame)
        if not self.with_filter:
            result.angular_velocity_covariance = [0.11, 0.0, 0.0, 0.0, 0.22, 0.0, 0.0, 0.0, 0.33]
            result.orientation.x = result.orientation.y = result.orientation.z = 0.0
            result.orientation.w = 1.0
            result.orientation_covariance = [-1.0] + [0.0] * 8
        elif msg.orientation_covariance[0] >= 0.0:
            q = oracle.normalized((*oracle.xyz(msg.orientation), msg.orientation.w))
            q = oracle.normalized(oracle.multiply(q, (-self.remap_q[0], -self.remap_q[1],
                                                      -self.remap_q[2], self.remap_q[3])))
            result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = q
            result.orientation_covariance = oracle.rotated_cov(msg.orientation_covariance, self.remap_rotation)
        return result

    def check_canonical(self, expected, label):
        self.wait(lambda: bool(self.canonical[0]) and (self.observer is None or bool(self.canonical[1])), label)
        baseline = self.canonical[0].popleft()
        oracle.close_numeric(oracle.imu_values(baseline), oracle.imu_values(expected), label + " baseline oracle")
        if self.observer is not None:
            actual = self.canonical[1].popleft()
            oracle.close_numeric(oracle.imu_values(actual), oracle.imu_values(baseline), label + " host/baseline")

    def send(self, msg, label, bias=(0.0, 0.0, 0.0), stationary=False):
        assert not any(self.canonical) and not any(self.corrected), label + ": unexpected pending output"
        expected = self.expected_remap(msg)
        self.publish_input(msg)
        self.check_canonical(expected, label + " canonical")
        if self.with_filter:
            corrected = deepcopy(expected)
            oracle.set_xyz(corrected.angular_velocity, (0.0, 0.0, 0.0) if stationary else tuple(
                value - offset for value, offset in zip(oracle.xyz(expected.angular_velocity), bias)))
            corrected = oracle.rotate_imu(corrected, oracle.matrix(self.tf_q), self.base_frame)
            for index in (2, 5, 6, 7):
                corrected.angular_velocity_covariance[index] = 0.0
            corrected.angular_velocity_covariance[8] = 0.007
            self.wait(lambda: all(self.corrected), label + " corrected")
            values = [queue.popleft() for queue in self.corrected]
            oracle.close_numeric(oracle.imu_values(values[0]), oracle.imu_values(values[1]), label + " corrected host/baseline")
            oracle.close_numeric(oracle.imu_values(values[0]), oracle.imu_values(corrected), label + " corrected oracle")
            key = oracle.stamp_key(msg)
            self.wait(lambda: all(key in values for values in self.bias), label + " bias")
            expected_bias = (self.frame, msg.header.stamp.sec, msg.header.stamp.nanosec, bias)
            for index, values in enumerate(self.bias):
                oracle.close_numeric(oracle.bias_values(values[key]), expected_bias, label + f" bias {index}")
        self.spin(0.06)
        assert not any(self.canonical) and not any(self.corrected), label + ": duplicate output for one input"
        self.passed(label + " (all fields, source stamp, no duplicate corrected output)")

    def reject(self, msg, label):
        for _ in range(3):
            assert not any(self.canonical) and not any(self.corrected)
            self.publish_input(msg)
            self.check_canonical(self.expected_remap(msg), label + " remap still forwards")
            self.spin(0.12)
            assert not any(self.canonical) and not any(self.corrected), label + ": rejected input produced output"
        self.passed(label + " (remap forwards; both filters reject)")

    def publish_input(self, msg):
        self.input.publish(msg)
        self.input_count += 1

    def verify_input_count(self):
        # Checking corrected outputs alone would miss double IPC/DDS delivery:
        # the existing monotonic-stamp guard could hide the second callback.
        # The existing 10 s rates log counts entry to on_imu BEFORE that guard.
        pattern = r"IMU bias filter rates[^\n]*totals input=(\d+)"

        def counts():
            result = []
            for name in ("baseline_filter", "host"):
                matches = re.findall(pattern, (self.directory / (name + ".log")).read_text(
                    encoding="utf-8", errors="replace"))
                result.append(int(matches[-1]) if matches else -1)
            return result

        self.wait(lambda: all(count >= self.input_count for count in counts()),
                  "post-fixture filter input totals", timeout=12.0)
        received = counts()
        assert received == [self.input_count, self.input_count], (
            f"duplicate/missing filter callback: sent={self.input_count}, received={received}")
        self.report.setdefault("filter_input_counts", []).append({
            "mode": self.mode, "sent": self.input_count,
            "standalone_received": received[0], "host_received": received[1]})
        self.passed("filter input count exactly equals raw sends, including observer transitions")

    def transform(self, q):
        self.tf_q = oracle.normalized(q)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = self.base_frame, self.frame
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = self.tf_q
        for _ in range(3):
            self.tf.publish(TFMessage(transforms=[tf]))
            self.spin(0.05)

    def stationary(self, value):
        odom = Odometry()
        odom.twist.twist.linear.x = 0.0 if value else 0.5
        for _ in range(3):
            self.odom.publish(odom)
            self.spin(0.03)

    def stop_host(self, signum):
        assert self.host.poll() is None, "host exited before shutdown test"
        started = time.monotonic()
        os.killpg(self.host.pid, signum)
        code = self.host.wait(timeout=5.0)
        assert code == 0, f"host failed graceful {signal.Signals(signum).name}: {code}"
        assert not Path(f"/proc/{self.host.pid}/task").exists(), "host threads remain after exit"
        try:
            os.killpg(self.host.pid, 0)
        except ProcessLookupError:
            pass
        else:
            raise AssertionError("host process group still contains a descendant")
        self.report["shutdowns"].append({"mode": self.mode, "signal": signal.Signals(signum).name,
                                        "exit_code": code, "elapsed_sec": time.monotonic() - started})
        self.passed(f"host {signal.Signals(signum).name} exited cleanly with no threads/descendants")

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for process in self.processes:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
                self.report.setdefault("forced_cleanup_pids", []).append(process.pid)
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        for publisher in self.publishers:
            self.node.destroy_publisher(publisher)
        for log in self.logs:
            log.close()


def run_case(node, args, mode, report):
    pair = PipelinePair(node, args, mode, report)
    try:
        if not pair.with_filter:
            for variant in range(3):
                pair.send(pair.sample(variant), "remap-only oracle")
            pair.stop_host(signal.SIGTERM)
            return
        pair.send(pair.sample(), "canonical has no external subscriber")
        pair.observe(True)
        for variant in range(4):
            msg = pair.sample(variant)
            if variant == 1:
                msg.orientation_covariance[0] = -1.0
            elif variant == 2:
                msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0.0
            elif variant == 3:
                msg.angular_velocity_covariance[4] = math.nan
            pair.send(msg, f"observer attached variant {variant}")
        pair.observe(False)
        pair.send(pair.sample(), "canonical external subscriber departed")
        pair.transform((0.31, 0.11, -0.25, 0.89))
        pair.send(pair.sample(), "TF listener remains live after retained transform update")
        pair.observe(True)
        msg = pair.sample()
        pair.send(msg, "canonical external subscriber rejoined")
        pair.reject(deepcopy(msg), "duplicate source stamp")
        old = deepcopy(msg)
        old.header.stamp.sec, old.header.stamp.nanosec = divmod(oracle.stamp_key(msg) - 100_000_000, 1_000_000_000)
        pair.reject(old, "out-of-order source stamp")
        stale = pair.sample()
        stale.header.stamp.sec -= 60
        pair.reject(stale, "stale source stamp")
        pair.send(pair.sample(), "fresh sample recovers after stale source")
        pair.stationary(True)
        msg = pair.sample(gyro=(0.01, -0.02, 0.03))
        learned = oracle.xyz(pair.expected_remap(msg).angular_velocity)
        pair.send(msg, "stationary bias initialization", learned, stationary=True)
        msg = pair.sample(gyro=(0.02, -0.01, 0.025))
        learned = tuple(0.98 * previous + 0.02 * value for previous, value in zip(
            learned, oracle.xyz(pair.expected_remap(msg).angular_velocity)))
        pair.send(msg, "stationary bias EWMA", learned, stationary=True)
        duplicate = deepcopy(msg)
        oracle.set_xyz(duplicate.angular_velocity, (-0.01, 0.015, -0.02))
        pair.reject(duplicate, "duplicate cannot update learned bias")
        pair.stationary(False)
        pair.send(pair.sample(gyro=(0.04, -0.03, 0.02)), "moving keeps identical bias and subtracts it", learned)
        pair.verify_input_count()
        pair.stop_host(signal.SIGINT if mode == "ipc" else signal.SIGTERM)
    finally:
        pair.close()


def main():
    args = arguments()  # Fail closed before importing ROS or starting participants.
    global oracle, rclpy, Imu, Odometry, TFMessage, TransformStamped, Vector3Stamped
    global QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    oracle = load_oracle()
    import rclpy
    from geometry_msgs.msg import TransformStamped, Vector3Stamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    report = {"schema": "isolated_imu_pipeline_v1", "status": "running", "domain_id": 197,
              "network_interfaces": ["lo"], "checks": [], "commands": [], "shutdowns": [],
              "note": "single-sample correctness test; source-stamp/freshness fixtures are test-only"}
    rclpy.init(args=[])
    node = rclpy.create_node(f"imu_pipeline_harness_{os.getpid()}")
    try:
        for mode in ("ipc", "dds_only", "remap_only"):
            run_case(node, args, mode, report)
        assert not report.get("forced_cleanup_pids"), "a child required forced cleanup"
        report["status"] = "passed"
    except BaseException:
        report["status"] = "failed"
        report["error"] = traceback.format_exc()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
        (args.report_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"PASS isolated IMU pipeline: {len(report['checks'])} checks; {args.report_dir / 'report.json'}", flush=True)


if __name__ == "__main__":
    main()

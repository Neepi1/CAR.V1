#!/usr/bin/env python3
"""Black-box IMU old/new equivalence and independent numeric-oracle tests.

Run ONLY in a disposable ROS Humble container with --network=none --ipc=private,
no device mounts, and no production processes. Source the existing ROS overlay:

  python3 isolated_imu_equivalence.py --isolated --domain-id 197 \
    --remap-old /baseline/imu_axis_remap_node \
    --remap-new /candidate/imu_axis_remap_node \
    --filter-old /baseline/imu_gyro_bias_filter_node \
    --filter-new /candidate/imu_gyro_bias_filter_node

--type remap or --type filter selects one family. Supplying the same executable
as old/new is useful for an oracle smoke test, but is NOT baseline equivalence.
The only non-stdlib dependencies are the ROS installation's rclpy/messages.
No hardware, launch files, node implementation internals, or production topics
are used. Node frequencies/QoS stay unchanged; source stamps are preserved and
the filter freshness window is widened to 30 seconds ONLY for these fixtures.
This is not a throughput, real-time scheduling, or hardware acceptance test.
"""

import argparse
from collections import deque
from copy import deepcopy
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isolated", action="store_true",
                        help="confirm disposable network-none, device-free container")
    parser.add_argument("--domain-id", type=int, default=197)
    parser.add_argument("--type", choices=("all", "remap", "filter"), default="all")
    for family in ("remap", "filter"):
        for version in ("old", "new"):
            parser.add_argument(f"--{family}-{version}", type=Path)
    args = parser.parse_args()
    if not __debug__:
        parser.error("do not use python -O: this test requires assertions")
    if not args.isolated:
        parser.error("--isolated is required; never run this on a production ROS host")
    if not (Path("/.dockerenv").exists() or Path("/run/.containerenv").exists()):
        parser.error("a disposable Linux container is required")
    interfaces = Path("/sys/class/net")
    if not interfaces.is_dir() or {p.name for p in interfaces.iterdir()} != {"lo"}:
        parser.error("refusing ROS startup: container must have only lo (--network=none)")
    if not 101 <= args.domain_id <= 229:
        parser.error("choose a dedicated test domain in 101..229")
    for family in ("remap", "filter"):
        if args.type not in ("all", family):
            continue
        for version in ("old", "new"):
            path = getattr(args, f"{family}_{version}")
            if path is None or not path.is_file() or not os.access(path, os.X_OK):
                parser.error(f"--{family}-{version} must name an executable file")
            setattr(args, f"{family}_{version}", path.resolve())
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    # Do not inherit a production discovery server or an external DDS profile.
    for name in ("ROS_DISCOVERY_SERVER", "FASTDDS_DEFAULT_PROFILES_FILE",
                 "FASTRTPS_DEFAULT_PROFILES_FILE", "RMW_FASTRTPS_USE_QOS_FROM_XML"):
        os.environ.pop(name, None)
    os.environ["SKIP_DEFAULT_XML"] = "1"
    return args


def close_numeric(actual, expected, label):
    if isinstance(expected, (tuple, list)):
        assert len(actual) == len(expected), f"{label}: length differs"
        for index, (a, b) in enumerate(zip(actual, expected)):
            close_numeric(a, b, f"{label}[{index}]")
    elif isinstance(expected, float):
        assert (math.isnan(actual) and math.isnan(expected)) or math.isclose(
            actual, expected, rel_tol=2e-12, abs_tol=2e-12), (
                f"{label}: actual={actual!r}, expected={expected!r}")
    else:
        assert actual == expected, f"{label}: actual={actual!r}, expected={expected!r}"


def xyz(vector):
    return (vector.x, vector.y, vector.z)


def imu_values(message):
    return (message.header.frame_id, message.header.stamp.sec,
            message.header.stamp.nanosec,
            (*xyz(message.orientation), message.orientation.w),
            tuple(message.orientation_covariance), xyz(message.angular_velocity),
            tuple(message.angular_velocity_covariance), xyz(message.linear_acceleration),
            tuple(message.linear_acceleration_covariance))


def bias_values(message):
    return (message.header.frame_id, message.header.stamp.sec,
            message.header.stamp.nanosec, xyz(message.vector))


def stamp_key(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def normalized(q):
    norm = math.sqrt(sum(v * v for v in q))
    return tuple(v / norm for v in q) if norm else (0.0, 0.0, 0.0, 1.0)


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
            w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s)


def matrix(q):
    x, y, z, w = normalized(q)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def rotated(values, rotation):
    return tuple(sum(a*b for a, b in zip(row, values)) for row in rotation)


def rotated_cov(values, rotation):
    return [sum(rotation[i][a] * values[a*3+b] * rotation[j][b]
                for a in range(3) for b in range(3))
            for i in range(3) for j in range(3)]


def set_xyz(target, values):
    target.x, target.y, target.z = values


def rotate_imu(message, rotation, frame):
    output = deepcopy(message)
    output.header.frame_id = frame
    for field in ("angular_velocity", "linear_acceleration"):
        set_xyz(getattr(output, field), rotated(xyz(getattr(message, field)), rotation))
        setattr(output, field + "_covariance",
                rotated_cov(getattr(message, field + "_covariance"), rotation))
    return output


class Pair:
    """Two real executables, isolated topics, bounded waits, one sample in flight."""

    serial = 0

    def __init__(self, node, family, old, new, params):
        Pair.serial += 1
        self.node = node
        self.family = family
        self.prefix = f"/imu_equivalence_{os.getpid()}_{Pair.serial}"
        self.outputs = [deque(), deque()]
        self.biases = [{}, {}]
        self.processes = []
        self.logs = []
        self.publishers = []
        self.subscriptions = []
        self.last_stamp = 0
        self.qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.input = self.publisher(Imu, "input", self.qos)
        self.odom = self.publisher(Odometry, "odom", self.qos)
        tf_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf = self.publisher(TFMessage, "tf_static", tf_qos)
        try:
            for index, executable in enumerate((old, new)):
                output_topic = f"{self.prefix}/output_{index}"
                bias_topic = f"{self.prefix}/bias_{index}"
                self.subscriptions.append(node.create_subscription(
                    Imu, output_topic,
                    lambda msg, i=index: self.outputs[i].append(msg), qos_profile_sensor_data))
                self.subscriptions.append(node.create_subscription(
                    Vector3Stamped, bias_topic,
                    lambda msg, i=index: self.biases[i].update({stamp_key(msg): msg}), self.qos))
                configured = dict(params)
                if family == "remap":
                    configured.update(input_topic=f"{self.prefix}/input", output_topic=output_topic)
                else:
                    configured.update(imu_topic=f"{self.prefix}/input", output_imu_topic=output_topic,
                                      bias_topic=bias_topic, odom_topic=f"{self.prefix}/odom",
                                      cmd_vel_topic=f"{self.prefix}/unused_command")
                command = [str(executable), "--ros-args", "-r",
                           f"__node:=imu_equivalence_{Pair.serial}_{index}",
                           "-r", f"/tf:={self.prefix}/tf", "-r",
                           f"/tf_static:={self.prefix}/tf_static"]
                for name, value in configured.items():
                    encoded = str(value).lower() if isinstance(value, bool) else str(value)
                    command += ["-p", f"{name}:={encoded}"]
                log = tempfile.TemporaryFile(mode="w+")
                self.logs.append(log)
                self.processes.append(subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            self.wait(lambda: self.input.get_subscription_count() == 2 and all(
                node.count_publishers(f"{self.prefix}/output_{i}") == 1 for i in range(2)),
                "DDS endpoint discovery", timeout=15.0)
            self.spin(0.15)
        except BaseException:
            self.close(failed=True)
            raise

    def publisher(self, message_type, suffix, qos):
        publisher = self.node.create_publisher(message_type, f"{self.prefix}/{suffix}", qos)
        self.publishers.append(publisher)
        return publisher

    def check_processes(self):
        for index, process in enumerate(self.processes):
            assert process.poll() is None, f"{self.family} process {index} exited: {process.returncode}"

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.check_processes()
            rclpy.spin_once(self.node, timeout_sec=min(0.01, max(0.0, deadline-time.monotonic())))

    def wait(self, predicate, label, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, f"timeout waiting for {self.family}: {label}"
            self.spin(0.01)

    def sample(self, variant=0, frame="test_imu", gyro=None):
        msg = Imu()
        self.last_stamp = max(self.last_stamp + 1_000_000, time.time_ns() - 50_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(self.last_stamp, 1_000_000_000)
        msg.header.frame_id = frame
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (0.2, -0.1, 0.3, 1.7)
        set_xyz(msg.angular_velocity, gyro or (0.21+variant*0.01, -0.34, 0.57))
        set_xyz(msg.linear_acceleration, (1.2, -2.3+variant*0.01, 9.4))
        for index, field in enumerate(("orientation_covariance", "angular_velocity_covariance",
                                       "linear_acceleration_covariance")):
            scale = (index+1) * (variant+1) * 0.13
            setattr(msg, field, [scale*v for v in (4.0, 0.3, -0.2, 0.3, 5.0, 0.4, -0.2, 0.4, 6.0)])
        return msg

    def send(self, message, expected, label, expected_bias=(0.0, 0.0, 0.0)):
        assert not any(self.outputs), f"{label}: unexpected output before input"
        self.input.publish(message)
        self.wait(lambda: all(self.outputs), label)
        received = [queue.popleft() for queue in self.outputs]
        close_numeric(imu_values(received[0]), imu_values(received[1]), label + " old/new")
        close_numeric(imu_values(received[0]), imu_values(expected), label + " oracle")
        if self.family == "filter":
            key = stamp_key(message)
            self.wait(lambda: all(key in values for values in self.biases), label + " bias")
            bias = [values[key] for values in self.biases]
            close_numeric(bias_values(bias[0]), bias_values(bias[1]), label + " bias old/new")
            close_numeric(bias_values(bias[0]), (message.header.frame_id,
                          message.header.stamp.sec, message.header.stamp.nanosec, expected_bias),
                          label + " bias oracle")
        self.spin(0.025)
        assert not any(self.outputs), f"{label}: more than one output for one input"

    def reject(self, message, label):
        # Repeat rejected inputs so a single best-effort DDS loss cannot mask a regression.
        for _ in range(3):
            self.input.publish(message)
            self.spin(0.10)
            assert not any(self.outputs), f"{label}: rejected sample produced corrected output"

    def transform(self, quaternion):
        self.wait(lambda: self.tf.get_subscription_count() == 2, "TF listeners")
        tf = TransformStamped()
        tf.header.frame_id = "test_base"
        tf.child_frame_id = "test_imu"
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quaternion
        # Retained static TF updates exercise rotation changes without TF interpolation.
        for _ in range(3):
            self.tf.publish(TFMessage(transforms=[tf]))
            self.spin(0.05)

    def stationary(self, value):
        self.wait(lambda: self.odom.get_subscription_count() == 2, "stationary-gate odom")
        msg = Odometry()
        msg.twist.twist.linear.x = 0.0 if value else 0.5
        for _ in range(3):
            self.odom.publish(msg)
            self.spin(0.03)

    def close(self, failed=False):
        for process in self.processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for process in self.processes:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3.0)
        for index, log in enumerate(self.logs):
            if failed:
                log.seek(0)
                print(f"--- {self.family} process {index} ---\n{log.read()[-12000:]}", file=sys.stderr)
            log.close()
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        for publisher in self.publishers:
            self.node.destroy_publisher(publisher)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, _exc, _traceback):
        self.close(failed=exc_type is not None)


def remap_cases(node, args):
    q = normalized((0.13, -0.21, 0.32, 0.91))
    rotation = matrix(q)
    for gyro_override, accel_override, unavailable in (
            (False, False, False), (True, False, False), (False, True, False),
            (True, True, False), (True, True, True)):
        name = f"remap gyro={gyro_override} accel={accel_override} unavailable={unavailable}"
        params = dict(output_frame_id="test_remapped", rotation_matrix=[v for row in rotation for v in row],
                      override_angular_velocity_covariance=gyro_override,
                      override_linear_acceleration_covariance=accel_override,
                      mark_orientation_unavailable=unavailable,
                      angular_velocity_covariance_diagonal=[0.11, 0.22, 0.33],
                      linear_acceleration_covariance_diagonal=[0.44, 0.55, 0.66])
        with Pair(node, "remap", args.remap_old, args.remap_new, params) as pair:
            for variant in range(6):
                msg = pair.sample(variant)
                if variant % 3 == 1:
                    msg.orientation_covariance[0] = -1.0
                elif variant % 3 == 2:
                    msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0.0
                if variant == 3:
                    msg.angular_velocity_covariance = [0.0] * 9
                    msg.linear_acceleration_covariance = [0.0] * 9
                elif variant == 4:
                    msg.angular_velocity_covariance[4] = math.nan
                    msg.linear_acceleration_covariance[0] = -1.0
                expected = rotate_imu(msg, rotation, "test_remapped")
                if gyro_override:
                    expected.angular_velocity_covariance = [0.11, 0.0, 0.0, 0.0, 0.22, 0.0, 0.0, 0.0, 0.33]
                if accel_override:
                    expected.linear_acceleration_covariance = [0.44, 0.0, 0.0, 0.0, 0.55, 0.0, 0.0, 0.0, 0.66]
                if unavailable:
                    expected.orientation.x = expected.orientation.y = expected.orientation.z = 0.0
                    expected.orientation.w = 1.0
                    expected.orientation_covariance = [-1.0] + [0.0] * 8
                elif msg.orientation_covariance[0] >= 0.0:
                    orientation = normalized(multiply(normalized((*xyz(msg.orientation), msg.orientation.w)),
                                                      (-q[0], -q[1], -q[2], q[3])))
                    expected.orientation.x, expected.orientation.y, expected.orientation.z, expected.orientation.w = orientation
                    expected.orientation_covariance = rotated_cov(msg.orientation_covariance, rotation)
                pair.send(msg, expected, name + f" sample={variant}")
        print("PASS " + name, flush=True)


def filter_cases(node, args):
    common = dict(transform_output_to_target_frame=True, output_target_frame="test_base",
                  use_cmd_vel_stationary=False, stationary_required_sec=0.0,
                  odom_timeout_sec=30.0, corrected_output_preserve_source_stamp=True,
                  corrected_output_max_source_age_sec=30.0)
    for override in (False, True):
        params = dict(common, override_output_angular_velocity_z_covariance=override,
                      output_angular_velocity_z_covariance=0.007)
        with Pair(node, "filter", args.filter_old, args.filter_new, params) as pair:
            rotation = matrix((0.13, -0.21, 0.32, 0.91))

            def expected(msg, gyro=None):
                result = deepcopy(msg)
                if gyro is not None:
                    set_xyz(result.angular_velocity, gyro)
                if result.header.frame_id != "test_base":
                    result = rotate_imu(result, rotation, "test_base")
                if override:
                    for index in (2, 5, 6, 7):
                        result.angular_velocity_covariance[index] = 0.0
                    result.angular_velocity_covariance[8] = 0.007
                return result

            pair.transform(normalized((0.13, -0.21, 0.32, 0.91)))
            for variant in (0, 0, 1, 2, 0):
                msg = pair.sample(variant)
                pair.send(msg, expected(msg), f"filter override={override} covariance={variant}")
            for _ in range(2):
                msg = pair.sample()
                msg.angular_velocity_covariance[4] = math.nan
                msg.linear_acceleration_covariance[0] = -1.0
                pair.send(msg, expected(msg), "nonfinite/unavailable covariance after warm cache")
            msg = pair.sample()
            pair.send(msg, expected(msg), "finite covariance after nonfinite input")
            rotation = matrix((-0.22, 0.31, -0.17, 0.86))
            pair.transform(normalized((-0.22, 0.31, -0.17, 0.86)))
            msg = pair.sample(0)
            pair.send(msg, expected(msg), "updated TF, unchanged covariance")
            pair.reject(deepcopy(msg), "duplicate stamp after warm cache")
            old = deepcopy(msg)
            old_ns = stamp_key(old) - 100_000_000
            old.header.stamp.sec, old.header.stamp.nanosec = divmod(old_ns, 1_000_000_000)
            pair.reject(old, "out-of-order stamp after warm cache")
            pair.reject(pair.sample(frame="missing_test_frame"), "missing TF after warm cache")
            pair.reject(pair.sample(frame=""), "empty frame after warm cache")
            msg = pair.sample(frame="test_base")
            pair.send(msg, expected(msg), "already in target frame")
            msg = pair.sample()
            pair.send(msg, expected(msg), "return to transformed frame")
            stale = pair.sample()
            stale.header.stamp.sec -= 60
            pair.reject(stale, "stale source after warm cache")
            msg = pair.sample()
            pair.send(msg, expected(msg), "recover after stale source")
            pair.stationary(True)
            learned = (0.01, -0.02, 0.03)
            msg = pair.sample(gyro=learned)
            pair.send(msg, expected(msg, (0.0, 0.0, 0.0)), "bias initialization", learned)
            sample = (0.02, -0.01, 0.04)
            learned = tuple(0.02*b + 0.98*a for a, b in zip(learned, sample))
            msg = pair.sample(gyro=sample)
            pair.send(msg, expected(msg, (0.0, 0.0, 0.0)), "bias EWMA", learned)
            duplicate = deepcopy(msg)
            set_xyz(duplicate.angular_velocity, (-0.04, 0.03, -0.02))
            pair.reject(duplicate, "duplicate cannot update bias or publish")
            pair.stationary(False)
            msg = pair.sample(gyro=(0.04, -0.03, 0.02))
            corrected = tuple(a-b for a, b in zip(xyz(msg.angular_velocity), learned))
            pair.send(msg, expected(msg, corrected), "moving bias subtraction", learned)
            pair.spin(0.15)
            assert not any(pair.outputs), "timer repeated an already published generation"
        print(f"PASS filter transform/covariance/stamp/bias override={override}", flush=True)

    with Pair(node, "filter", args.filter_old, args.filter_new,
              dict(common, transform_output_to_target_frame=False)) as pair:
        msg = pair.sample(frame="no_tf_required")
        pair.send(msg, msg, "transform bypass")
    print("PASS filter transform bypass", flush=True)

    with Pair(node, "filter", args.filter_old, args.filter_new,
              dict(common, drop_output_on_transform_failure=False)) as pair:
        pair.transform((0.0, 0.0, 0.0, 1.0))
        msg = pair.sample()
        pair.send(msg, rotate_imu(msg, matrix((0.0, 0.0, 0.0, 1.0)), "test_base"), "warm fallback case")
        for frame in ("missing_test_frame", ""):
            msg = pair.sample(frame=frame)
            pair.send(msg, msg, "configured transform-failure passthrough")
    print("PASS filter configured transform-failure passthrough", flush=True)


if __name__ == "__main__":
    args = arguments()  # Safety checks happen before rclpy imports or ROS startup.
    import rclpy
    from geometry_msgs.msg import TransformStamped, Vector3Stamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=[])
    node = rclpy.create_node(f"imu_equivalence_harness_{os.getpid()}")
    try:
        for family, run in (("remap", remap_cases), ("filter", filter_cases)):
            if args.type in ("all", family):
                old, new = getattr(args, family + "_old"), getattr(args, family + "_new")
                print(f"{family}: {'ORACLE SMOKE (same executable)' if old == new else 'OLD/NEW DIFFERENTIAL'}",
                      flush=True)
                run(node, args)
        print("PASS all requested isolated IMU cases", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

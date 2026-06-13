#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=180
LABEL="navigation_tf_jitter"

usage() {
  cat <<'USAGE'
Usage: observe_navigation_tf_jitter_180s.sh [--duration-sec N] [--label LABEL]

Records lightweight odom, TF, AMCL, bridge-status, rosout, and CPU samples
while an existing runtime or user-started navigation is active. This script
does not send navigation goals and does not subscribe to pointcloud topics.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[observe-tf-jitter] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[observe-tf-jitter] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${NJRH_PROJECT_ROOT}/reports/navigation_tf_jitter_180s/${timestamp}_${safe_label}_${DURATION_SEC}s"
mkdir -p "${report_dir}"

echo "[observe-tf-jitter] duration_sec=${DURATION_SEC} report_dir=${report_dir}" >&2

python3 - "${DURATION_SEC}" "${report_dir}" <<'PY'
import csv
import json
import math
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

duration_sec = float(sys.argv[1])
report_dir = Path(sys.argv[2])
admission_impl = os.environ.get("NJRH_AMCL_SCAN_ADMISSION_IMPL", "cpp")


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pct(values, percentile):
    if not values:
        return None
    data = sorted(values)
    if len(data) == 1:
        return data[0]
    k = (len(data) - 1) * percentile / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return data[int(k)]
    return data[f] * (c - k) + data[c] * (k - f)


class Series:
    def __init__(self):
        self.count = 0
        self.first_recv = None
        self.last_recv = None
        self.last_stamp = None
        self.recv_gaps = []
        self.stamp_gaps = []

    def add(self, recv, stamp):
        if self.first_recv is None:
            self.first_recv = recv
        if self.last_recv is not None:
            self.recv_gaps.append(recv - self.last_recv)
        if self.last_stamp is not None:
            self.stamp_gaps.append(stamp - self.last_stamp)
        self.last_recv = recv
        self.last_stamp = stamp
        self.count += 1

    def hz(self):
        if self.first_recv is None or self.last_recv is None or self.last_recv <= self.first_recv:
            return 0.0
        return max(0, self.count - 1) / (self.last_recv - self.first_recv)

    @staticmethod
    def stats(values):
        if not values:
            return {"avg_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}
        return {
            "avg_ms": statistics.mean(values) * 1000.0,
            "p95_ms": pct(values, 95) * 1000.0,
            "p99_ms": pct(values, 99) * 1000.0,
            "max_ms": max(values) * 1000.0,
        }

    def summary(self):
        return {
            "count": self.count,
            "hz": self.hz(),
            "recv_gap": self.stats(self.recv_gaps),
            "stamp_gap": self.stats(self.stamp_gaps),
        }


class Probe(Node):
    def __init__(self):
        super().__init__("observe_navigation_tf_jitter")
        self.series = {
            "/wheel/odom": Series(),
            "/wheel/odom_ekf": Series(),
            "/local_state/odometry": Series(),
            "/scan_amcl": Series(),
            "/amcl_pose": Series(),
            "tf:odom->base_link": Series(),
            "tf:map->odom": Series(),
            "tf:base_link->lidar_level_link": Series(),
        }
        self.bridge_status_samples = []
        self.cpu_samples = []
        self.rosout_events = []
        self.future_extrapolation_count = 0
        self.message_filter_drop_count = 0
        self.failed_to_make_progress_count = 0
        self.start_wall = time.time()
        self.last_cpu_sample = 0.0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(depth=50)
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Odometry, "/wheel/odom", lambda msg: self.on_odom("/wheel/odom", msg), sensor_qos)
        self.create_subscription(
            Odometry, "/wheel/odom_ekf", lambda msg: self.on_odom("/wheel/odom_ekf", msg), sensor_qos
        )
        self.create_subscription(
            Odometry, "/local_state/odometry", lambda msg: self.on_odom("/local_state/odometry", msg), sensor_qos
        )
        self.create_subscription(LaserScan, "/scan_amcl", self.on_scan_amcl, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, status_qos)
        self.create_subscription(String, "/localization/bridge_status", self.on_bridge_status, status_qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, sensor_qos)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, tf_static_qos)
        self.create_subscription(Log, "/rosout", self.on_rosout, status_qos)
        self.create_timer(1.0, self.on_timer)

        self.sample_file = (report_dir / "samples.csv").open("w", newline="")
        self.sample_writer = csv.writer(self.sample_file)
        self.sample_writer.writerow(["source", "recv_wall", "stamp_sec", "x", "y", "yaw"])

    def rel_time(self):
        return time.time() - self.start_wall

    def on_odom(self, source, msg):
        recv = self.rel_time()
        stamp = stamp_sec(msg.header.stamp)
        self.series[source].add(recv, stamp)
        pose = msg.pose.pose
        self.sample_writer.writerow(
            [source, f"{recv:.6f}", f"{stamp:.9f}", f"{pose.position.x:.6f}", f"{pose.position.y:.6f}", f"{yaw_from_quat(pose.orientation):.6f}"]
        )

    def on_scan_amcl(self, msg):
        recv = self.rel_time()
        stamp = stamp_sec(msg.header.stamp)
        self.series["/scan_amcl"].add(recv, stamp)
        self.sample_writer.writerow(["/scan_amcl", f"{recv:.6f}", f"{stamp:.9f}", "", "", ""])

    def on_amcl_pose(self, msg):
        recv = self.rel_time()
        stamp = stamp_sec(msg.header.stamp)
        self.series["/amcl_pose"].add(recv, stamp)
        pose = msg.pose.pose
        self.sample_writer.writerow(
            ["/amcl_pose", f"{recv:.6f}", f"{stamp:.9f}", f"{pose.position.x:.6f}", f"{pose.position.y:.6f}", f"{yaw_from_quat(pose.orientation):.6f}"]
        )

    def on_tf(self, msg):
        recv = self.rel_time()
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            key = f"tf:{parent}->{child}"
            if key not in self.series:
                continue
            stamp = stamp_sec(transform.header.stamp)
            self.series[key].add(recv, stamp)
            tf = transform.transform
            self.sample_writer.writerow(
                [key, f"{recv:.6f}", f"{stamp:.9f}", f"{tf.translation.x:.6f}", f"{tf.translation.y:.6f}", f"{yaw_from_quat(tf.rotation):.6f}"]
            )

    def on_bridge_status(self, msg):
        recv = self.rel_time()
        data = {}
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"raw": msg.data}
        self.bridge_status_samples.append((recv, data))

    def on_rosout(self, msg):
        text = msg.msg or ""
        if "future" in text.lower() and "extrapolation" in text.lower():
            self.future_extrapolation_count += 1
            self.rosout_events.append((self.rel_time(), msg.name, "TF_FUTURE_EXTRAPOLATION", text))
        elif "Message Filter dropping message" in text:
            self.message_filter_drop_count += 1
            self.rosout_events.append((self.rel_time(), msg.name, "MESSAGE_FILTER_DROP", text))
        elif "Failed to make progress" in text:
            self.failed_to_make_progress_count += 1
            self.rosout_events.append((self.rel_time(), msg.name, "FAILED_TO_MAKE_PROGRESS", text))

    def on_timer(self):
        now = self.rel_time()
        if now - self.last_cpu_sample < 0.9:
            return
        self.last_cpu_sample = now
        try:
            output = subprocess.check_output(
                ["ps", "-eo", "pid=,psr=,pcpu=,comm=,args="],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )
        except Exception:
            return
        patterns = (
            "amcl_scan_admission_node",
            "amcl_scan_admission_relay.py",
            "nav2_amcl",
            "localization_bridge_node",
            "ekf_node",
            "wheel_odom_ekf_input",
            "controller_server",
            "hesai_accel_driver_node",
            "robot_safety_node",
            "ranger_base_node",
        )
        for line in output.splitlines():
            if any(pattern in line for pattern in patterns):
                self.cpu_samples.append((now, line.strip()))


def relay_process_info():
    preferred = (
        ("cpp", "amcl_scan_admission_node"),
        ("python", "amcl_scan_admission_relay.py"),
    )
    if admission_impl == "python":
        preferred = tuple(reversed(preferred))
    try:
        output = subprocess.check_output(
            ["ps", "-eo", "pid=,psr=,pcpu=,args="],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=1.0,
        )
    except Exception:
        return {"admission_impl": admission_impl, "pid": None}
    for impl, pattern in preferred:
        for line in output.splitlines():
            if pattern not in line or "observe_navigation_tf_jitter_180s" in line:
                continue
            parts = line.strip().split(None, 3)
            if len(parts) < 4:
                continue
            pid = parts[0]
            allowed = None
            try:
                with open(f"/proc/{pid}/status", "r", encoding="utf-8") as f:
                    for status_line in f:
                        if status_line.startswith("Cpus_allowed_list:"):
                            allowed = status_line.split(":", 1)[1].strip()
                            break
            except Exception:
                pass
            return {
                "admission_impl": admission_impl,
                "actual_impl": impl,
                "pid": int(pid),
                "psr": int(parts[1]),
                "pcpu": float(parts[2]),
                "Cpus_allowed_list": allowed,
                "command": parts[3],
            }
    return {"admission_impl": admission_impl, "pid": None}


rclpy.init()
probe = Probe()
deadline = time.time() + duration_sec
try:
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
finally:
    probe.sample_file.close()
    probe.destroy_node()
    rclpy.shutdown()

summary = {name: series.summary() for name, series in probe.series.items()}
last_bridge = probe.bridge_status_samples[-1][1] if probe.bridge_status_samples else {}
summary["bridge_status_last"] = {
    key: last_bridge.get(key)
    for key in (
        "latest_odom_tf_age_ms",
        "map_to_odom_age_ms",
        "amcl_scan_admission_hz",
        "amcl_ready",
        "last_reject_reason",
        "last_accept_reason",
        "active_correction_source",
    )
}
summary["rosout_counts"] = {
    "tf_future_extrapolation": probe.future_extrapolation_count,
    "message_filter_drop": probe.message_filter_drop_count,
    "failed_to_make_progress": probe.failed_to_make_progress_count,
}
summary["admission_relay"] = relay_process_info()

with (report_dir / "summary.json").open("w") as f:
    json.dump(summary, f, indent=2, sort_keys=True)

with (report_dir / "bridge_status_samples.csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["recv_wall", "data"])
    for recv, data in probe.bridge_status_samples:
        writer.writerow([f"{recv:.6f}", json.dumps(data, sort_keys=True)])

with (report_dir / "cpu_samples.csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["recv_wall", "ps_line"])
    for recv, line in probe.cpu_samples:
        writer.writerow([f"{recv:.6f}", line])

with (report_dir / "rosout_events.csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["recv_wall", "node", "event", "message"])
    for row in probe.rosout_events:
        writer.writerow(row)

with (report_dir / "summary.md").open("w") as f:
    f.write("# Navigation TF Jitter Observation\n\n")
    f.write(f"- duration_sec: {duration_sec:.1f}\n")
    f.write(f"- tf_future_extrapolation_count: {probe.future_extrapolation_count}\n")
    f.write(f"- message_filter_drop_count: {probe.message_filter_drop_count}\n")
    f.write(f"- failed_to_make_progress_count: {probe.failed_to_make_progress_count}\n\n")
    f.write("## AMCL Scan Admission Relay\n\n")
    f.write("```json\n")
    f.write(json.dumps(summary["admission_relay"], indent=2, sort_keys=True))
    f.write("\n```\n\n")
    f.write("## Series\n\n")
    f.write("| source | count | hz | recv_avg_ms | recv_p95_ms | recv_max_ms | stamp_max_ms |\n")
    f.write("| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n")
    for name, data in summary.items():
        if not isinstance(data, dict) or "recv_gap" not in data:
            continue
        recv = data["recv_gap"]
        stamp = data["stamp_gap"]
        f.write(
            f"| {name} | {data['count']} | {data['hz']:.3f} | "
            f"{(recv['avg_ms'] or 0.0):.3f} | {(recv['p95_ms'] or 0.0):.3f} | "
            f"{(recv['max_ms'] or 0.0):.3f} | {(stamp['max_ms'] or 0.0):.3f} |\n"
        )
    f.write("\n## Last Bridge Status\n\n")
    f.write("```json\n")
    f.write(json.dumps(summary["bridge_status_last"], indent=2, sort_keys=True))
    f.write("\n```\n")

print(report_dir)
PY

echo "[observe-tf-jitter] wrote ${report_dir}"

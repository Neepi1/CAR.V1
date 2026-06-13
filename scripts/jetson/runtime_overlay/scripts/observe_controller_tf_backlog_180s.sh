#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

DURATION_SEC=180
LABEL="controller_tf_backlog"

usage() {
  cat <<'USAGE'
Usage: observe_controller_tf_backlog_180s.sh [--duration-sec N] [--label LABEL]

Records controller_server TF backlog symptoms while an existing runtime or
user-started navigation is active. This script does not send navigation goals,
does not change Nav2 parameters, and does not subscribe to pointcloud topics.
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
      echo "[controller-tf-backlog] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[controller-tf-backlog] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${NJRH_PROJECT_ROOT}/reports/controller_tf_backlog_180s/${timestamp}_${safe_label}_${DURATION_SEC}s"
mkdir -p "${report_dir}"

echo "[controller-tf-backlog] duration_sec=${DURATION_SEC} report_dir=${report_dir}" >&2

python3 - "${DURATION_SEC}" "${report_dir}" <<'PY'
import csv
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

duration_sec = float(sys.argv[1])
report_dir = Path(sys.argv[2])

REQUESTED_LATEST_RE = re.compile(
    r"Requested time\s+([0-9]+(?:\.[0-9]+)?)\s+but the latest data is at time\s+([0-9]+(?:\.[0-9]+)?)",
    re.IGNORECASE,
)


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


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


def ms_stats(values):
    if not values:
        return {"count": 0, "avg_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}
    return {
        "count": len(values),
        "avg_ms": statistics.mean(values) * 1000.0,
        "p95_ms": pct(values, 95) * 1000.0,
        "p99_ms": pct(values, 99) * 1000.0,
        "max_ms": max(values) * 1000.0,
    }


class Series:
    def __init__(self):
        self.count = 0
        self.first_recv = None
        self.last_recv = None
        self.last_stamp = None
        self.recv_gaps = []
        self.stamp_gaps = []
        self.recv_minus_stamp = []

    def add(self, recv_wall, recv_rel, stamp):
        if self.first_recv is None:
            self.first_recv = recv_rel
        if self.last_recv is not None:
            self.recv_gaps.append(recv_rel - self.last_recv)
        if self.last_stamp is not None:
            self.stamp_gaps.append(stamp - self.last_stamp)
        self.recv_minus_stamp.append(recv_wall - stamp)
        self.last_recv = recv_rel
        self.last_stamp = stamp
        self.count += 1

    def hz(self):
        if self.first_recv is None or self.last_recv is None or self.last_recv <= self.first_recv:
            return 0.0
        return max(0, self.count - 1) / (self.last_recv - self.first_recv)

    def summary(self):
        return {
            "count": self.count,
            "hz": self.hz(),
            "recv_gap": ms_stats(self.recv_gaps),
            "stamp_gap": ms_stats(self.stamp_gaps),
            "recv_minus_stamp": ms_stats(self.recv_minus_stamp),
        }


class CmdSeries:
    def __init__(self):
        self.count = 0
        self.nonzero_count = 0
        self.first_recv = None
        self.last_recv = None
        self.max_abs_linear_x = 0.0
        self.max_abs_angular_z = 0.0

    def add(self, recv_rel, msg):
        if self.first_recv is None:
            self.first_recv = recv_rel
        self.last_recv = recv_rel
        self.count += 1
        lx = float(msg.linear.x)
        az = float(msg.angular.z)
        self.max_abs_linear_x = max(self.max_abs_linear_x, abs(lx))
        self.max_abs_angular_z = max(self.max_abs_angular_z, abs(az))
        if abs(lx) > 1.0e-4 or abs(az) > 1.0e-4:
            self.nonzero_count += 1

    def hz(self):
        if self.first_recv is None or self.last_recv is None or self.last_recv <= self.first_recv:
            return 0.0
        return max(0, self.count - 1) / (self.last_recv - self.first_recv)

    def summary(self):
        return {
            "count": self.count,
            "hz": self.hz(),
            "nonzero_count": self.nonzero_count,
            "max_abs_linear_x": self.max_abs_linear_x,
            "max_abs_angular_z": self.max_abs_angular_z,
        }


def controller_pid():
    try:
        output = subprocess.check_output(
            ["ps", "-eo", "pid=,args="],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=1.0,
        )
    except Exception:
        return None
    for line in output.splitlines():
        if "controller_server" not in line:
            continue
        if not ("nav2_controller" in line or "__node:=controller_server" in line or "/controller_server" in line):
            continue
        if "observe_controller_tf_backlog_180s" in line:
            continue
        parts = line.strip().split(None, 1)
        if parts:
            try:
                return int(parts[0])
            except ValueError:
                pass
    return None


def proc_cpuset(pid):
    if not pid:
        return None
    try:
        with open(f"/proc/{pid}/status", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("Cpus_allowed_list:"):
                    return line.split(":", 1)[1].strip()
    except Exception:
        return None
    return None


class Probe(Node):
    def __init__(self):
        super().__init__("observe_controller_tf_backlog")
        self.start_wall = time.time()
        self.tf_series = {
            "tf:map->odom": Series(),
            "tf:odom->base_link": Series(),
        }
        self.cmd_series = {
            "/cmd_vel_nav_raw": CmdSeries(),
            "/cmd_vel_nav": CmdSeries(),
            "/cmd_vel_collision_checked": CmdSeries(),
            "/cmd_vel_safe": CmdSeries(),
            "/cmd_vel": CmdSeries(),
        }
        self.rosout_events = []
        self.tf_future_lags = []
        self.tf_future_count = 0
        self.local_costmap_drop_count = 0
        self.controller_drop_count = 0
        self.failed_to_make_progress_count = 0
        self.last_cpu_sample = 0.0
        self.controller_cpu_samples = []

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(depth=200)
        self.create_subscription(TFMessage, "/tf", self.on_tf, sensor_qos)
        self.create_subscription(Log, "/rosout", self.on_rosout, status_qos)
        for topic in self.cmd_series:
            self.create_subscription(Twist, topic, lambda msg, name=topic: self.on_cmd(name, msg), status_qos)
        self.create_timer(1.0, self.on_timer)

        self.tf_file = (report_dir / "tf_samples.csv").open("w", newline="")
        self.tf_writer = csv.writer(self.tf_file)
        self.tf_writer.writerow(["source", "recv_rel", "recv_wall", "stamp_sec"])
        self.thread_file = (report_dir / "controller_threads.csv").open("w", newline="")
        self.thread_writer = csv.writer(self.thread_file)
        self.thread_writer.writerow(["recv_rel", "pid", "tid", "psr", "pcpu", "comm", "wchan", "cpus_allowed_list"])
        self.cmd_file = (report_dir / "cmd_vel_samples.csv").open("w", newline="")
        self.cmd_writer = csv.writer(self.cmd_file)
        self.cmd_writer.writerow(["topic", "recv_rel", "linear_x", "angular_z"])

    def rel(self):
        return time.time() - self.start_wall

    def on_tf(self, msg):
        recv_wall = time.time()
        recv_rel = self.rel()
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            key = f"tf:{parent}->{child}"
            if key not in self.tf_series:
                continue
            stamp = stamp_sec(transform.header.stamp)
            self.tf_series[key].add(recv_wall, recv_rel, stamp)
            self.tf_writer.writerow([key, f"{recv_rel:.6f}", f"{recv_wall:.9f}", f"{stamp:.9f}"])

    def on_cmd(self, topic, msg):
        recv_rel = self.rel()
        self.cmd_series[topic].add(recv_rel, msg)
        self.cmd_writer.writerow([topic, f"{recv_rel:.6f}", f"{msg.linear.x:.6f}", f"{msg.angular.z:.6f}"])

    def on_rosout(self, msg):
        text = msg.msg or ""
        lowered = text.lower()
        event = None
        details = {}
        if "future" in lowered and "extrapolation" in lowered:
            self.tf_future_count += 1
            event = "TF_FUTURE_EXTRAPOLATION"
            match = REQUESTED_LATEST_RE.search(text)
            if match:
                requested = float(match.group(1))
                latest = float(match.group(2))
                lag = requested - latest
                self.tf_future_lags.append(lag)
                details = {"requested": requested, "latest": latest, "lag_ms": lag * 1000.0}
        if "Message Filter dropping message" in text:
            if "local_costmap" in msg.name or "local_costmap" in text:
                self.local_costmap_drop_count += 1
                event = event or "LOCAL_COSTMAP_MESSAGE_FILTER_DROP"
            elif "controller" in msg.name:
                self.controller_drop_count += 1
                event = event or "CONTROLLER_MESSAGE_FILTER_DROP"
        if "Failed to make progress" in text:
            self.failed_to_make_progress_count += 1
            event = event or "FAILED_TO_MAKE_PROGRESS"
        if event:
            self.rosout_events.append((self.rel(), msg.name, event, json.dumps(details, sort_keys=True), text))

    def on_timer(self):
        now = self.rel()
        if now - self.last_cpu_sample < 0.9:
            return
        self.last_cpu_sample = now
        pid = controller_pid()
        if not pid:
            return
        allowed = proc_cpuset(pid)
        try:
            output = subprocess.check_output(
                ["ps", "-L", "-p", str(pid), "-o", "pid=,tid=,psr=,pcpu=,comm=,wchan="],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )
        except Exception:
            return
        for line in output.splitlines():
            parts = line.strip().split(None, 6)
            if len(parts) < 6:
                continue
            row = [f"{now:.6f}", *parts[:6], allowed or ""]
            self.thread_writer.writerow(row[:8])
            self.controller_cpu_samples.append(row[:8])

    def close(self):
        self.tf_file.close()
        self.thread_file.close()
        self.cmd_file.close()


rclpy.init()
probe = Probe()
deadline = time.time() + duration_sec
try:
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
finally:
    probe.close()
    probe.destroy_node()
    rclpy.shutdown()

pid = controller_pid()
summary = {
    "duration_sec": duration_sec,
    "profile": os.environ.get("NJRH_NAV2_CONTROLLER_CPU_PROFILE", "current"),
    "expected_controller_cpuset": os.environ.get("NJRH_CPUSET_CONTROLLER_SERVER"),
    "controller_pid": pid,
    "controller_cpus_allowed_list": proc_cpuset(pid),
    "tf_series": {name: series.summary() for name, series in probe.tf_series.items()},
    "cmd_vel": {name: series.summary() for name, series in probe.cmd_series.items()},
    "rosout_counts": {
        "tf_future_extrapolation": probe.tf_future_count,
        "local_costmap_message_filter_drop": probe.local_costmap_drop_count,
        "controller_message_filter_drop": probe.controller_drop_count,
        "failed_to_make_progress": probe.failed_to_make_progress_count,
    },
    "controller_requested_latest_lag": ms_stats(probe.tf_future_lags),
}

with (report_dir / "summary.json").open("w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2, sort_keys=True)

with (report_dir / "rosout_events.csv").open("w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["recv_rel", "node", "event", "details", "message"])
    for row in probe.rosout_events:
        writer.writerow(row)

with (report_dir / "summary.md").open("w", encoding="utf-8") as f:
    f.write("# Controller TF Backlog Observation\n\n")
    f.write(f"- duration_sec: {duration_sec:.1f}\n")
    f.write(f"- profile: {summary['profile']}\n")
    f.write(f"- expected_controller_cpuset: {summary['expected_controller_cpuset']}\n")
    f.write(f"- controller_pid: {summary['controller_pid']}\n")
    f.write(f"- controller_cpus_allowed_list: {summary['controller_cpus_allowed_list']}\n")
    f.write(f"- tf_future_extrapolation_count: {probe.tf_future_count}\n")
    f.write(f"- local_costmap_message_filter_drop_count: {probe.local_costmap_drop_count}\n")
    f.write(f"- failed_to_make_progress_count: {probe.failed_to_make_progress_count}\n\n")
    f.write("## Controller Requested/Latest Lag\n\n")
    f.write("```json\n")
    f.write(json.dumps(summary["controller_requested_latest_lag"], indent=2, sort_keys=True))
    f.write("\n```\n\n")
    f.write("## TF Series\n\n")
    f.write("| source | count | hz | recv_gap_p99_ms | recv_gap_max_ms | stamp_gap_max_ms | recv_minus_stamp_p99_ms |\n")
    f.write("| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n")
    for name, data in summary["tf_series"].items():
        recv = data["recv_gap"]
        stamp = data["stamp_gap"]
        age = data["recv_minus_stamp"]
        f.write(
            f"| {name} | {data['count']} | {data['hz']:.3f} | "
            f"{(recv['p99_ms'] or 0.0):.3f} | {(recv['max_ms'] or 0.0):.3f} | "
            f"{(stamp['max_ms'] or 0.0):.3f} | {(age['p99_ms'] or 0.0):.3f} |\n"
        )
    f.write("\n## Cmd Vel Chain\n\n")
    f.write("| topic | count | hz | nonzero_count | max_abs_linear_x | max_abs_angular_z |\n")
    f.write("| --- | ---: | ---: | ---: | ---: | ---: |\n")
    for name, data in summary["cmd_vel"].items():
        f.write(
            f"| {name} | {data['count']} | {data['hz']:.3f} | {data['nonzero_count']} | "
            f"{data['max_abs_linear_x']:.4f} | {data['max_abs_angular_z']:.4f} |\n"
        )

print(str(report_dir))
PY

echo "[controller-tf-backlog] wrote ${report_dir}" >&2

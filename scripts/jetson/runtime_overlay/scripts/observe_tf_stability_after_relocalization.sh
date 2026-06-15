#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=180
LABEL="post_relocalization_tf_stability"

usage() {
  cat <<'USAGE'
Usage: observe_tf_stability_after_relocalization.sh [--duration-sec N] [--label LABEL]

Records map->odom publish heartbeat, TF samples, rosout TF drops, cmd_vel chain,
and API state after a manual or docking relocalization. It is read-only and does not subscribe to pointcloud topics.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-180}"
      shift 2
      ;;
    --label)
      LABEL="${2:-post_relocalization_tf_stability}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[tf-settle-observe] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[tf-settle-observe] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${NJRH_PROJECT_ROOT}/reports/tf_stability_after_relocalization/${timestamp}_${safe_label}_${DURATION_SEC}s"
mkdir -p "${report_dir}"

python3 - "${DURATION_SEC}" "${report_dir}" <<'PY'
import csv
import json
import math
import re
import statistics
import sys
import time
import urllib.request
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

duration_sec = float(sys.argv[1])
report_dir = Path(sys.argv[2])

DROP_RE = re.compile(
    r"Message Filter dropping|earlier than all (the )?data|future extrapolation|transformPose",
    re.IGNORECASE,
)


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def percentile(values, pct):
    if not values:
        return None
    data = sorted(values)
    if len(data) == 1:
        return data[0]
    k = (len(data) - 1) * pct / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return data[int(k)]
    return data[f] * (c - k) + data[c] * (k - f)


def stats_ms(values):
    if not values:
        return {"count": 0, "avg_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}
    return {
        "count": len(values),
        "avg_ms": statistics.mean(values) * 1000.0,
        "p95_ms": percentile(values, 95) * 1000.0,
        "p99_ms": percentile(values, 99) * 1000.0,
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
            "recv_gap": stats_ms(self.recv_gaps),
            "stamp_gap": stats_ms(self.stamp_gaps),
            "recv_minus_stamp": stats_ms(self.recv_minus_stamp),
        }


class CmdStats:
    def __init__(self):
        self.count = 0
        self.nonzero_count = 0
        self.max_abs_linear_x = 0.0
        self.max_abs_angular_z = 0.0

    def add(self, msg):
        self.count += 1
        linear = abs(msg.linear.x)
        angular = abs(msg.angular.z)
        self.max_abs_linear_x = max(self.max_abs_linear_x, linear)
        self.max_abs_angular_z = max(self.max_abs_angular_z, angular)
        if linear > 1.0e-3 or angular > 1.0e-3:
            self.nonzero_count += 1

    def summary(self):
        return {
            "count": self.count,
            "nonzero_count": self.nonzero_count,
            "max_abs_linear_x": self.max_abs_linear_x,
            "max_abs_angular_z": self.max_abs_angular_z,
        }


class Observer(Node):
    def __init__(self):
        super().__init__("observe_tf_stability_after_relocalization")
        self.start_wall = time.time()
        self.start_mono = time.monotonic()
        self.bridge_status_rows = []
        self.rosout_rows = []
        self.api_rows = []
        self.map_odom = Series()
        self.odom_base = Series()
        self.cmd = {topic: CmdStats() for topic in [
            "/cmd_vel_nav",
            "/cmd_vel_collision_checked",
            "/cmd_vel_safe",
            "/cmd_vel",
        ]}
        self.create_subscription(String, "/localization/bridge_status", self.on_bridge_status, 10)
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.create_subscription(Log, "/rosout", self.on_rosout, 100)
        for topic in self.cmd:
            self.create_subscription(Twist, topic, lambda msg, t=topic: self.on_cmd(t, msg), 10)
        self.create_timer(1.0, self.poll_api)

    def rel(self):
        return time.monotonic() - self.start_mono

    def on_bridge_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        data["_t_rel"] = self.rel()
        self.bridge_status_rows.append(data)

    def on_tf(self, msg):
        recv_wall = time.time()
        recv_rel = self.rel()
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            if parent == "map" and child == "odom":
                self.map_odom.add(recv_wall, recv_rel, stamp_sec(transform.header.stamp))
            elif parent == "odom" and child == "base_link":
                self.odom_base.add(recv_wall, recv_rel, stamp_sec(transform.header.stamp))

    def on_rosout(self, msg):
        text = msg.msg or ""
        if DROP_RE.search(text):
            self.rosout_rows.append({
                "t_rel": self.rel(),
                "name": msg.name,
                "level": int(msg.level),
                "message": text,
            })

    def on_cmd(self, topic, msg):
        self.cmd[topic].add(msg)

    def poll_api(self):
        row = {"t_rel": self.rel()}
        for name, url in {
            "status": "http://127.0.0.1:8080/api/v1/status",
            "navigation": "http://127.0.0.1:8080/api/v1/navigation/state",
            "docking": "http://127.0.0.1:8080/api/v1/docking/state",
        }.items():
            try:
                with urllib.request.urlopen(url, timeout=0.25) as response:
                    row[name] = json.loads(response.read().decode("utf-8", errors="replace"))
            except Exception as exc:
                row[name] = {"error": str(exc)}
        self.api_rows.append(row)


rclpy.init()
node = Observer()
deadline = time.monotonic() + duration_sec
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

with (report_dir / "bridge_status_samples.jsonl").open("w") as f:
    for row in node.bridge_status_rows:
        f.write(json.dumps(row, sort_keys=True) + "\n")
with (report_dir / "rosout_tf_drops.jsonl").open("w") as f:
    for row in node.rosout_rows:
        f.write(json.dumps(row, sort_keys=True) + "\n")
with (report_dir / "api_poll.jsonl").open("w") as f:
    for row in node.api_rows:
        f.write(json.dumps(row, sort_keys=True) + "\n")

with (report_dir / "cmd_vel_summary.csv").open("w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=["topic", "count", "nonzero_count", "max_abs_linear_x", "max_abs_angular_z"],
    )
    writer.writeheader()
    for topic, series in node.cmd.items():
        row = {"topic": topic}
        row.update(series.summary())
        writer.writerow(row)

latest_bridge = node.bridge_status_rows[-1] if node.bridge_status_rows else {}
bridge_gaps = [
    float(row.get("map_odom_publish_gap_ms"))
    for row in node.bridge_status_rows
    if isinstance(row.get("map_odom_publish_gap_ms"), (int, float))
]
bridge_hz = [
    float(row.get("map_odom_publish_loop_hz"))
    for row in node.bridge_status_rows
    if isinstance(row.get("map_odom_publish_loop_hz"), (int, float))
]

summary = {
    "duration_sec": duration_sec,
    "bridge_status_samples": len(node.bridge_status_rows),
    "latest_bridge_status": latest_bridge,
    "bridge_map_odom_publish_gap_ms_max_observed": max(bridge_gaps) if bridge_gaps else None,
    "bridge_map_odom_publish_hz_min_observed": min(bridge_hz) if bridge_hz else None,
    "tf_map_odom": node.map_odom.summary(),
    "tf_odom_base": node.odom_base.summary(),
    "rosout_tf_drop_count": len(node.rosout_rows),
    "cmd_vel": {topic: series.summary() for topic, series in node.cmd.items()},
    "api_samples": len(node.api_rows),
}
(report_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True))
(report_dir / "summary.md").write_text(
    "\n".join([
        "# TF Stability After Relocalization",
        "",
        f"- duration_sec: {duration_sec:.0f}",
        f"- bridge_status_samples: {summary['bridge_status_samples']}",
        f"- bridge_map_odom_publish_gap_ms_max_observed: {summary['bridge_map_odom_publish_gap_ms_max_observed']}",
        f"- bridge_map_odom_publish_hz_min_observed: {summary['bridge_map_odom_publish_hz_min_observed']}",
        f"- tf_map_odom_count: {summary['tf_map_odom']['count']}",
        f"- tf_map_odom_hz: {summary['tf_map_odom']['hz']:.3f}",
        f"- tf_map_odom_recv_gap_p99_ms: {summary['tf_map_odom']['recv_gap']['p99_ms']}",
        f"- tf_odom_base_count: {summary['tf_odom_base']['count']}",
        f"- tf_odom_base_hz: {summary['tf_odom_base']['hz']:.3f}",
        f"- rosout_tf_drop_count: {summary['rosout_tf_drop_count']}",
        f"- latest_owner: {latest_bridge.get('map_to_odom_publisher_owner')}",
        f"- publisher_decoupled_from_correction: {latest_bridge.get('publisher_decoupled_from_correction')}",
        f"- latest_failure_hint: {node.rosout_rows[-1]['message'] if node.rosout_rows else 'none'}",
    ]) + "\n"
)

node.destroy_node()
rclpy.shutdown()
print(report_dir / "summary.md")
PY

echo "[tf-settle-observe] wrote ${report_dir}"
echo "[tf-settle-observe] summary ${report_dir}/summary.md"

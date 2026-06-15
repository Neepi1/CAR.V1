#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=10
EXPECT_PASS=false

usage() {
  cat <<'USAGE'
Usage: verify_bridge_map_odom_publisher.sh [--duration-sec N] [--expect-pass]

Verifies that robot_localization_bridge exposes a decoupled map->odom publisher
heartbeat and that /tf contains map->odom samples. This script is read-only.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-10}"
      shift 2
      ;;
    --expect-pass)
      EXPECT_PASS=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[bridge-map-odom] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[bridge-map-odom] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

python3 - "${DURATION_SEC}" <<'PY'
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

duration_sec = float(sys.argv[1])


class Probe(Node):
    def __init__(self):
        super().__init__("verify_bridge_map_odom_publisher_probe")
        self.bridge_status = []
        self.map_odom_tf_count = 0
        self.tf_first = None
        self.tf_last = None
        self.tf_recv_gaps = []
        self.create_subscription(String, "/localization/bridge_status", self.on_status, 10)
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)

    def on_status(self, msg):
        try:
            self.bridge_status.append(json.loads(msg.data))
        except Exception:
            pass

    def on_tf(self, msg):
        now = time.monotonic()
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                if self.tf_first is None:
                    self.tf_first = now
                if self.tf_last is not None:
                    self.tf_recv_gaps.append(now - self.tf_last)
                self.tf_last = now
                self.map_odom_tf_count += 1


def num(value, fallback=-1.0):
    try:
        result = float(value)
    except Exception:
        return fallback
    return result if math.isfinite(result) else fallback


rclpy.init()
node = Probe()
deadline = time.monotonic() + duration_sec
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

latest = node.bridge_status[-1] if node.bridge_status else {}
failures = []
warnings = []

required_fields = [
    "publisher_decoupled_from_correction",
    "map_odom_publish_loop_hz",
    "map_odom_publish_gap_ms",
    "map_odom_publish_gap_max_ms",
    "map_odom_publish_callback_duration_us",
    "map_odom_latest_accepted_sequence",
    "map_odom_last_published_sequence",
    "map_odom_latest_source",
    "map_odom_state_valid",
    "map_odom_correction_paused",
    "map_odom_frozen_due_to_pause",
    "map_odom_publish_missed_count",
    "map_to_odom_publisher_owner",
    "has_map_to_odom",
]
for field in required_fields:
    if field not in latest:
        failures.append(f"missing bridge_status.{field}")

if latest.get("publisher_decoupled_from_correction") is not True:
    failures.append("publisher_decoupled_from_correction is not true")
if latest.get("map_to_odom_publisher_owner") != "robot_localization_bridge":
    failures.append(f"unexpected map->odom owner {latest.get('map_to_odom_publisher_owner')!r}")
if latest.get("has_map_to_odom") is not True:
    failures.append("has_map_to_odom is not true")
if latest.get("map_odom_state_valid") is not True:
    failures.append("map_odom_state_valid is not true")

gap_fail_ms = num(latest.get("map_odom_publish_gap_fail_ms"), 250.0)
gap_ms = num(latest.get("map_odom_publish_gap_ms"))
if gap_ms < 0.0 or gap_ms > gap_fail_ms:
    failures.append(f"map_odom_publish_gap_ms={gap_ms:.1f} exceeds fail threshold {gap_fail_ms:.1f}")

loop_hz = num(latest.get("map_odom_publish_loop_hz"))
if loop_hz < 30.0:
    warnings.append(f"map_odom_publish_loop_hz={loop_hz:.1f} below expected 50Hz class")

if node.map_odom_tf_count <= 0:
    failures.append("/tf map->odom sample was not observed")

tf_hz = 0.0
if node.tf_first is not None and node.tf_last is not None and node.tf_last > node.tf_first:
    tf_hz = max(0, node.map_odom_tf_count - 1) / (node.tf_last - node.tf_first)

summary = {
    "duration_sec": duration_sec,
    "bridge_status_samples": len(node.bridge_status),
    "tf_map_odom_count": node.map_odom_tf_count,
    "tf_map_odom_hz": tf_hz,
    "latest_bridge_status": latest,
    "warnings": warnings,
    "failures": failures,
}
print(json.dumps(summary, indent=2, sort_keys=True))

node.destroy_node()
rclpy.shutdown()
sys.exit(1 if failures else 0)
PY

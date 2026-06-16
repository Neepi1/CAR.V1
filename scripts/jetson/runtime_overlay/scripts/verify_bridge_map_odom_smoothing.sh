#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=120
EXPECT_PASS=false

usage() {
  cat <<'USAGE'
Usage: verify_bridge_map_odom_smoothing.sh [--duration-sec N] [--expect-pass]

Read-only verifier for robot_localization_bridge current/target map->odom
smoothing. It does not trigger localization or send motion commands.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-120}"
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
      echo "[bridge-smoothing] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[bridge-smoothing] --duration-sec must be an integer" >&2
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
        super().__init__("verify_bridge_map_odom_smoothing_probe")
        self.status = []
        self.tf_count = 0
        self.tf_first = None
        self.tf_last = None
        self.create_subscription(String, "/localization/bridge_status", self.on_status, 10)
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)

    def on_status(self, msg):
        try:
            self.status.append(json.loads(msg.data))
        except Exception:
            pass

    def on_tf(self, msg):
        now = time.monotonic()
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                if self.tf_first is None:
                    self.tf_first = now
                self.tf_last = now
                self.tf_count += 1


def finite(value, fallback=-1.0):
    try:
        out = float(value)
    except Exception:
        return fallback
    return out if math.isfinite(out) else fallback


rclpy.init()
node = Probe()
deadline = time.monotonic() + duration_sec
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

latest = node.status[-1] if node.status else {}
failures = []
warnings = []

required = [
    "smoothing_enabled",
    "correction_active",
    "safe_for_goal_start",
    "current_sequence",
    "target_sequence",
    "last_accepted_sequence",
    "last_published_sequence",
    "current_source",
    "target_source",
    "remaining_translation_error_m",
    "remaining_yaw_error_rad",
    "last_step_translation_m",
    "last_step_yaw_rad",
    "smoothing_translation_rate_mps",
    "smoothing_yaw_rate_radps",
    "last_correction_delta_translation_m",
    "last_correction_delta_yaw_rad",
    "last_correction_source",
    "large_correction_rejected_count",
    "online_correction_smoothed_count",
    "online_correction_snap_count",
    "publisher_decoupled_from_correction",
    "map_odom_publish_loop_hz",
    "map_odom_publish_gap_ms",
    "map_to_odom_publisher_owner",
    "has_map_to_odom",
]
for field in required:
    if field not in latest:
        failures.append(f"missing bridge_status.{field}")

if latest.get("smoothing_enabled") is not True:
    failures.append("smoothing_enabled is not true")
if latest.get("publisher_decoupled_from_correction") is not True:
    failures.append("publisher_decoupled_from_correction is not true")
if latest.get("map_to_odom_publisher_owner") != "robot_localization_bridge":
    failures.append(f"map_to_odom_publisher_owner={latest.get('map_to_odom_publisher_owner')!r}")
if latest.get("has_map_to_odom") is not True:
    failures.append("has_map_to_odom is not true")

loop_hz = finite(latest.get("map_odom_publish_loop_hz"))
if loop_hz < 30.0:
    warnings.append(f"map_odom_publish_loop_hz={loop_hz:.1f}, expected 50Hz class")
gap_ms = finite(latest.get("map_odom_publish_gap_ms"))
gap_fail_ms = finite(latest.get("map_odom_publish_gap_fail_ms"), 250.0)
if gap_ms < 0.0 or gap_ms > gap_fail_ms:
    failures.append(f"map_odom_publish_gap_ms={gap_ms:.1f} exceeds fail threshold {gap_fail_ms:.1f}")

if node.tf_count <= 0:
    failures.append("/tf map->odom sample was not observed")

active_samples = [s for s in node.status if s.get("correction_active") is True]
if len(active_samples) >= 2:
    first = finite(active_samples[0].get("remaining_translation_error_m"), 0.0) + finite(active_samples[0].get("remaining_yaw_error_rad"), 0.0)
    last = finite(active_samples[-1].get("remaining_translation_error_m"), 0.0) + finite(active_samples[-1].get("remaining_yaw_error_rad"), 0.0)
    if last > first + 0.01:
        failures.append(f"remaining smoothing error increased during active correction: first={first:.3f} last={last:.3f}")

tf_hz = 0.0
if node.tf_first is not None and node.tf_last is not None and node.tf_last > node.tf_first:
    tf_hz = max(0, node.tf_count - 1) / (node.tf_last - node.tf_first)

summary = {
    "duration_sec": duration_sec,
    "bridge_status_samples": len(node.status),
    "tf_map_odom_count": node.tf_count,
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

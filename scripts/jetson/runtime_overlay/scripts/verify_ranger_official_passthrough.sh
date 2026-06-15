#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=10
COMPARE_CMD=0
PROFILE=""
SET_PROFILE=0
DO_RESTART=0
WATCH_ONCE=0

usage() {
  cat <<'EOF'
Usage:
  verify_ranger_official_passthrough.sh [--duration-sec N] [--compare-cmd]
      [--profile official_passthrough|custom] [--set-profile] [--restart] [--watch-once]

Default is read-only and does not publish motion commands.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-10}"
      shift 2
      ;;
    --compare-cmd)
      COMPARE_CMD=1
      shift
      ;;
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --set-profile)
      SET_PROFILE=1
      shift
      ;;
    --restart)
      DO_RESTART=1
      shift
      ;;
    --watch-once)
      WATCH_ONCE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[verify-ranger-pass] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -n "${PROFILE}" ]]; then
  case "${PROFILE}" in
    official_passthrough|custom) ;;
    *) echo "[verify-ranger-pass] invalid profile: ${PROFILE}" >&2; exit 2 ;;
  esac
fi

if [[ "${SET_PROFILE}" -eq 1 ]]; then
  [[ -n "${PROFILE}" ]] || { echo "[verify-ranger-pass] --set-profile requires --profile" >&2; exit 2; }
  args=(--profile "${PROFILE}")
  [[ "${DO_RESTART}" -eq 1 ]] && args+=(--restart)
  bash "${SCRIPT_DIR}/set_ranger_mode_controller_profile.sh" "${args[@]}" --print
elif [[ "${DO_RESTART}" -eq 1 ]]; then
  args=(--restart --print)
  [[ -n "${PROFILE}" ]] && args=(--profile "${PROFILE}" "${args[@]}")
  bash "${SCRIPT_DIR}/set_ranger_mode_controller_profile.sh" "${args[@]}"
fi

failures=0
warn() { echo "[verify-ranger-pass][WARN] $*"; }
fail() { echo "[verify-ranger-pass][FAIL] $*"; failures=$((failures + 1)); }
pass() { echo "[verify-ranger-pass][PASS] $*"; }

topic_info_contains() {
  local topic="$1"
  local pattern="$2"
  local info
  info="$(timeout 10 ros2 topic info -v "${topic}" 2>/dev/null || true)"
  grep -Eq "${pattern}" <<<"${info}"
}

topic_exists() {
  timeout 8 ros2 topic info "$1" >/dev/null 2>&1
}

status_json="$(timeout 5 ros2 topic echo --once --field data /ranger_mini3_mode_controller/status 2>/dev/null | head -n 1 || true)"
if [[ -z "${status_json}" ]]; then
  fail "no /ranger_mini3_mode_controller/status sample"
else
  python3 - "$status_json" "$PROFILE" <<'PY' || failures=$((failures + 1))
import json
import sys

status = json.loads(sys.argv[1])
expected = sys.argv[2] or "official_passthrough"
checks = {
    "mode_controller_profile": status.get("mode_controller_profile"),
    "custom_ackermann_enabled": status.get("custom_ackermann_enabled"),
    "cmd_vel_passthrough": status.get("cmd_vel_passthrough"),
    "desired_motion_mode_source": status.get("desired_motion_mode_source"),
}
print("[verify-ranger-pass] status=" + json.dumps(checks, sort_keys=True))
if expected == "official_passthrough":
    assert status.get("mode_controller_profile") == "official_passthrough"
    assert status.get("custom_ackermann_enabled") is False
    assert status.get("cmd_vel_passthrough") is True
    assert status.get("desired_motion_mode_source") == "predicted_from_cmd_vel_safe"
PY
fi

topic_exists /cmd_vel_safe && pass "/cmd_vel_safe exists" || fail "/cmd_vel_safe missing"
topic_exists /cmd_vel && pass "/cmd_vel exists" || fail "/cmd_vel missing"
topic_info_contains /cmd_vel "Node name: ranger_mini3_mode_controller" &&
  pass "/cmd_vel publisher is ranger_mini3_mode_controller" ||
  fail "/cmd_vel publisher is not ranger_mini3_mode_controller"
topic_info_contains /cmd_vel_safe "Node name: robot_safety" &&
  pass "robot_safety publishes /cmd_vel_safe" ||
  fail "robot_safety does not publish /cmd_vel_safe"
topic_info_contains /cmd_vel "Node name: ranger_base_node|Node name: ranger_base" &&
  pass "ranger_base_node subscribes /cmd_vel" ||
  warn "ranger_base_node /cmd_vel subscription not visible"

for topic in /motion_state /system_state /ranger_mini3/desired_motion_mode \
  /ranger_mini3/docking_allow_reverse /ranger_mini3/teleop_allow_reverse /ranger_mini3/allow_reverse
do
  topic_exists "${topic}" && pass "${topic} visible" || warn "${topic} not currently visible"
done

if [[ "${COMPARE_CMD}" -eq 1 ]]; then
  python3 - "$DURATION_SEC" <<'PY' || failures=$((failures + 1))
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

duration = float(sys.argv[1])
allowed = {"reverse_not_allowed", "timeout_zero", "park_requested", "startup_zero", "lateral_not_allowed", "custom_profile"}

class Probe(Node):
    def __init__(self):
        super().__init__("ranger_passthrough_verify_probe")
        self.safe = []
        self.out = []
        self.status = {}
        self.create_subscription(Twist, "/cmd_vel_safe", lambda m: self.safe.append((time.time(), m)), 20)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self.out.append((time.time(), m)), 20)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", self.on_status, 20)

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            self.status = {}

def triple(msg):
    return (msg.linear.x, msg.linear.y, msg.angular.z)

rclpy.init()
node = Probe()
deadline = time.time() + duration
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)

if not node.safe or not node.out:
    print("[verify-ranger-pass][WARN] compare-cmd saw no paired command samples")
    rclpy.shutdown()
    raise SystemExit(0)

bad = 0
for out_stamp, out in node.out:
    nearest = min(node.safe, key=lambda item: abs(item[0] - out_stamp))
    sx, sy, sz = triple(nearest[1])
    ox, oy, oz = triple(out)
    if max(abs(sx - ox), abs(sy - oy), abs(sz - oz)) <= 1.0e-6:
        continue
    reasons = set(str(node.status.get("diff_reason", "")).split(","))
    if not reasons.intersection(allowed):
        bad += 1

print(f"[verify-ranger-pass] compare safe_samples={len(node.safe)} out_samples={len(node.out)} bad={bad}")
rclpy.shutdown()
if bad:
    raise SystemExit(1)
PY
fi

if [[ "${WATCH_ONCE}" -eq 1 ]]; then
  timeout 5 ros2 topic echo --once /ranger_mini3_mode_controller/status || true
fi

if [[ "${failures}" -gt 0 ]]; then
  echo "[verify-ranger-pass] FAIL failures=${failures}"
  exit 1
fi
echo "[verify-ranger-pass] PASS"

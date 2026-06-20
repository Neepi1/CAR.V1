#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=common_env.sh
  source "${SCRIPT_DIR}/common_env.sh"
fi

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=30
TARGET_DELTA_DEG=10
MAX_ANGULAR_Z=0.25
APPLY_SMALL_YAW_TEST=false
DRY_RUN=false
REQUIRE_SAFETY_READY=false
OUTPUT_DIR=""
PREFIX="[predock-yaw-probe]"

usage() {
  cat <<'EOF'
Usage:
  run_predock_yaw_alignment_probe.sh --dry-run
  run_predock_yaw_alignment_probe.sh --apply-small-yaw-test --target-delta-deg 10

Default mode is observe-only and delegates to observe_predock_yaw_alignment_trace.sh.

DANGER: --apply-small-yaw-test publishes a bounded angular command only to
/cmd_vel_docking. It never publishes /cmd_vel directly, never calls
/api/v1/docking/start, never sends Nav2 goals, and never triggers relocalization.
Run it only in an open area with a human ready to stop the robot.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply-small-yaw-test)
      APPLY_SMALL_YAW_TEST=true
      shift
      ;;
    --target-delta-deg)
      TARGET_DELTA_DEG="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --max-angular-z)
      MAX_ANGULAR_Z="${2:-}"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --require-safety-ready)
      REQUIRE_SAFETY_READY=true
      shift
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "${PREFIX} FAIL unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 5 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 5" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/predock_yaw_probe_${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

if [[ "${APPLY_SMALL_YAW_TEST}" != "true" || "${DRY_RUN}" == "true" ]]; then
  {
    echo "# Predock Yaw Probe"
    echo
    echo "- mode: \`observe_only\`"
    echo "- apply_small_yaw_test: \`${APPLY_SMALL_YAW_TEST}\`"
    echo "- dry_run: \`${DRY_RUN}\`"
    echo "- command_published: \`false\`"
    echo "- note: rerun with \`--apply-small-yaw-test\` only in an open area to publish /cmd_vel_docking."
  } >"${OUTPUT_DIR}/summary.md"
  bash "${SCRIPT_DIR}/observe_predock_yaw_alignment_trace.sh" \
    --duration-sec "${DURATION_SEC}" \
    --api-url "${API_URL}" \
    --output-dir "${OUTPUT_DIR}/observe"
  echo "${PREFIX} wrote ${OUTPUT_DIR}"
  exit 0
fi

echo "${PREFIX} DANGER: publishing bounded angular.z to /cmd_vel_docking only."
echo "${PREFIX} DANGER: ensure open area, no active navigation, no active docking, and human supervision."

python3 - "${API_URL}" "${OUTPUT_DIR}" "${DURATION_SEC}" "${TARGET_DELTA_DEG}" "${MAX_ANGULAR_Z}" "${REQUIRE_SAFETY_READY}" <<'PY'
import csv
import json
import math
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

api_url = sys.argv[1].rstrip("/")
out_dir = Path(sys.argv[2])
duration_sec = int(sys.argv[3])
target_delta_deg = float(sys.argv[4])
max_angular_z = min(abs(float(sys.argv[5])), 0.25)
require_safety_ready = sys.argv[6].lower() == "true"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def now_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def api_get(path, timeout=1.0):
    req = urllib.request.Request(f"{api_url}{path}", method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def norm_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


status = api_get("/api/v1/status")
nav = api_get("/api/v1/navigation/state")
docking = api_get("/api/v1/docking/state")
preflight_failures = []
if nav.get("navigation_goal", {}).get("state") in ("accepted", "executing", "running"):
    preflight_failures.append("active navigation goal is present")
if docking.get("docking_active") is True or docking.get("state") in ("running", "undocking"):
    preflight_failures.append("active docking or undocking is present")
if require_safety_ready:
    safety_status = str(status.get("safety", {}).get("status", ""))
    if "ESTOP" in safety_status or "EMERGENCY" in safety_status:
        preflight_failures.append(f"robot_safety not ready: {safety_status}")
if preflight_failures:
    raw = {
        "preflight_failures": preflight_failures,
        "status": status,
        "navigation_state": nav,
        "docking_state": docking,
    }
    (out_dir / "raw.json").write_text(json.dumps(raw, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (out_dir / "summary.md").write_text(
        "# Predock Yaw Probe\n\n"
        "- result: `FAIL`\n"
        f"- preflight_failures: `{preflight_failures}`\n"
        "- command_published: `false`\n",
        encoding="utf-8",
    )
    raise SystemExit(1)


class Probe(Node):
    def __init__(self):
        super().__init__("run_predock_yaw_alignment_probe")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_docking", 10)
        self.safe = Twist()
        self.base = Twist()
        self.local_yaw = None
        self.wheel_yaw = None
        self.safety_status = ""
        self.mode_status = ""
        self.create_subscription(Twist, "/cmd_vel_safe", self._twist_cb("safe"), 10)
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb("base"), 10)
        self.create_subscription(Odometry, "/local_state/odometry", self._odom_cb("local"), 10)
        self.create_subscription(Odometry, "/wheel/odom", self._odom_cb("wheel"), 10)
        self.create_subscription(String, "/safety/status", self._string_cb("safety"), 10)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", self._string_cb("mode"), 10)

    def _twist_cb(self, name):
        def cb(msg):
            if name == "safe":
                self.safe = msg
            else:
                self.base = msg
        return cb

    def _odom_cb(self, name):
        def cb(msg):
            yaw = yaw_from_quat(msg.pose.pose.orientation)
            if name == "local":
                self.local_yaw = yaw
            else:
                self.wheel_yaw = yaw
        return cb

    def _string_cb(self, name):
        def cb(msg):
            if name == "safety":
                self.safety_status = msg.data
            else:
                self.mode_status = msg.data
        return cb

    def publish_cmd(self, z):
        msg = Twist()
        msg.angular.z = z
        self.cmd_pub.publish(msg)


rclpy.init()
node = Probe()
warmup_end = time.monotonic() + 1.0
while time.monotonic() < warmup_end:
    rclpy.spin_once(node, timeout_sec=0.05)

target_delta = math.radians(target_delta_deg)
direction = 1.0 if target_delta >= 0.0 else -1.0
cmd_z = direction * max_angular_z
start_local_yaw = node.local_yaw
start_wheel_yaw = node.wheel_yaw
samples = []
zero_sent = False

try:
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        current_local_delta = ""
        if start_local_yaw is not None and node.local_yaw is not None:
            current_local_delta = norm_angle(node.local_yaw - start_local_yaw)
        error = target_delta - current_local_delta if isinstance(current_local_delta, float) else target_delta
        if isinstance(current_local_delta, float) and abs(current_local_delta) >= abs(target_delta):
            break
        if "BLOCK" in node.safety_status or "ESTOP" in node.safety_status:
            break
        node.publish_cmd(cmd_z)
        sample = {
            "timestamp": now_iso(),
            "target_delta_rad": target_delta,
            "cmd_vel_docking.angular.z": cmd_z,
            "cmd_vel_safe.angular.z": float(node.safe.angular.z),
            "cmd_vel.angular.z": float(node.base.angular.z),
            "local_state_yaw_delta": current_local_delta,
            "wheel_odom_yaw_delta": norm_angle(node.wheel_yaw - start_wheel_yaw) if start_wheel_yaw is not None and node.wheel_yaw is not None else "",
            "yaw_error": error,
            "robot_safety_state": node.safety_status,
            "mode_controller_status": node.mode_status,
        }
        samples.append(sample)
        time.sleep(0.05)
finally:
    zero = Twist()
    zero_end = time.monotonic() + 0.5
    while time.monotonic() < zero_end:
        node.cmd_pub.publish(zero)
        zero_sent = True
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()

fields = [
    "timestamp",
    "target_delta_rad",
    "cmd_vel_docking.angular.z",
    "cmd_vel_safe.angular.z",
    "cmd_vel.angular.z",
    "local_state_yaw_delta",
    "wheel_odom_yaw_delta",
    "yaw_error",
    "robot_safety_state",
    "mode_controller_status",
]
with (out_dir / "timeline.csv").open("w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(samples)

def any_nonzero(field):
    return any(abs(float(s.get(field) or 0.0)) > 1e-4 for s in samples)

local_deltas = [abs(float(s["local_state_yaw_delta"])) for s in samples if isinstance(s.get("local_state_yaw_delta"), float)]
errors = [abs(float(s["yaw_error"])) for s in samples if isinstance(s.get("yaw_error"), float)]
mode_text = " ".join(str(s.get("mode_controller_status", "")) for s in samples)
determinations = {
    "cmd_vel_docking_has_angular_z": any_nonzero("cmd_vel_docking.angular.z"),
    "cmd_vel_safe_has_angular_z": any_nonzero("cmd_vel_safe.angular.z"),
    "cmd_vel_has_angular_z": any_nonzero("cmd_vel.angular.z"),
    "actual_motion_mode_spinning_or_explained": "SPINNING" in mode_text or "code: 2" in mode_text or '"code":2' in mode_text or bool(samples and samples[-1].get("robot_safety_state")),
    "mode_switching_not_stuck": "mode_switching" not in mode_text.lower(),
    "yaw_delta_matches_command_direction": bool(local_deltas and samples and math.copysign(1.0, float(samples[-1].get("local_state_yaw_delta") or 0.0)) == math.copysign(1.0, target_delta)),
    "yaw_error_decreased": bool(len(errors) >= 2 and errors[-1] < errors[0]),
    "zero_cmd_sent_after_probe": zero_sent,
}
failures = [key for key, value in determinations.items() if not value]
raw = {
    "preflight": {"status": status, "navigation_state": nav, "docking_state": docking},
    "target_delta_deg": target_delta_deg,
    "max_angular_z": max_angular_z,
    "determinations": determinations,
    "samples": samples,
}
(out_dir / "raw.json").write_text(json.dumps(raw, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
summary = [
    "# Predock Yaw Probe",
    "",
    f"- report_dir: `{out_dir}`",
    "- mode: `apply_small_yaw_test`",
    "- published_topic: `/cmd_vel_docking`",
    "- direct_cmd_vel_publish: `false`",
    f"- result: `{'FAIL' if failures else 'PASS'}`",
    f"- failures: `{failures}`",
    "",
    "## Determinations",
]
for key, value in determinations.items():
    summary.append(f"- {key}: `{str(value).lower()}`")
(out_dir / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
print(f"summary={out_dir / 'summary.md'}")
raise SystemExit(1 if failures else 0)
PY

echo "${PREFIX} wrote ${OUTPUT_DIR}"

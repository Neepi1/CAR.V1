#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=common_env.sh
  source "${SCRIPT_DIR}/common_env.sh"
fi

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=180
DOCK_ID=""
WATCH=false
OUTPUT_DIR=""
PREFIX="[predock-yaw-trace]"

usage() {
  cat <<'EOF'
Usage:
  observe_predock_yaw_alignment_trace.sh [--duration-sec N] [--dock-id ID] [--watch]

Default mode is read-only. It does not send docking requests, Nav2 goals,
relocalization triggers, or velocity commands. It records predock yaw alignment
state, command-chain twist topics, odometry yaw deltas, safety state, and mode
controller evidence without subscribing to heavy pointcloud topics.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --dock-id)
      DOCK_ID="${2:-}"
      shift 2
      ;;
    --watch)
      WATCH=true
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
if [[ "${WATCH}" == "true" && "${DURATION_SEC}" -lt 3600 ]]; then
  DURATION_SEC=3600
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/predock_yaw_alignment_${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
python3 - "${DURATION_SEC}" "${API_URL}" "${OUTPUT_DIR}" "${DOCK_ID}" <<'PY'
import csv
import json
import math
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

duration_sec = int(sys.argv[1])
api_url = sys.argv[2].rstrip("/")
out_dir = Path(sys.argv[3])
dock_id_filter = sys.argv[4]

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except Exception as exc:  # pragma: no cover
    rclpy = None
    ROS_IMPORT_ERROR = repr(exc)
else:
    ROS_IMPORT_ERROR = ""


def now_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def api_get(path, timeout=0.5):
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


def nested(obj, path, default=""):
    cur = obj
    for part in path:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(part)
    return default if cur is None else cur


class TraceNode(Node):
    def __init__(self):
        super().__init__("observe_predock_yaw_alignment_trace")
        self.twists = {}
        self.strings = {}
        self.yaws = {}
        for topic in ("/cmd_vel_docking", "/cmd_vel_safe", "/cmd_vel"):
            self.create_subscription(Twist, topic, self._twist_cb(topic), 10)
        self.create_subscription(Odometry, "/local_state/odometry", self._odom_cb("/local_state/odometry"), 10)
        self.create_subscription(Odometry, "/wheel/odom", self._odom_cb("/wheel/odom"), 10)
        for topic in ("/safety/status", "/ranger_mini3_mode_controller/status", "/motion_state"):
            self.create_subscription(String, topic, self._string_cb(topic), 10)

    def _twist_cb(self, topic):
        def cb(msg):
            self.twists[topic] = {
                "linear_x": float(msg.linear.x),
                "angular_z": float(msg.angular.z),
                "stamp": now_iso(),
            }
        return cb

    def _odom_cb(self, topic):
        def cb(msg):
            self.yaws[topic] = yaw_from_quat(msg.pose.pose.orientation)
        return cb

    def _string_cb(self, topic):
        def cb(msg):
            self.strings[topic] = msg.data
        return cb


node = None
if rclpy is not None:
    rclpy.init()
    node = TraceNode()

samples = []
first_local_yaw = None
first_wheel_yaw = None
deadline = time.monotonic() + duration_sec
sample_period = 0.2
next_sample = 0.0
while time.monotonic() < deadline:
    if node is not None:
        rclpy.spin_once(node, timeout_sec=0.02)
    if time.monotonic() >= next_sample:
        state = api_get("/api/v1/docking/state")
        status = api_get("/api/v1/status")
        job = state.get("docking") if isinstance(state, dict) else {}
        if not isinstance(job, dict):
            job = {}
        if dock_id_filter and str(job.get("dock_id", "")) != dock_id_filter:
            next_sample = time.monotonic() + sample_period
            continue
        local_yaw = nested(getattr(node, "yaws", {}), ["/local_state/odometry"], "")
        wheel_yaw = nested(getattr(node, "yaws", {}), ["/wheel/odom"], "")
        if first_local_yaw is None and isinstance(local_yaw, float):
            first_local_yaw = local_yaw
        if first_wheel_yaw is None and isinstance(wheel_yaw, float):
            first_wheel_yaw = wheel_yaw
        base_error = job.get("predock_base_yaw_error_rad", "")
        contact_error = job.get("predock_contact_yaw_error_rad", "")
        try:
            normalized = norm_angle(float(contact_error if contact_error != "" else base_error))
        except Exception:
            normalized = ""
        sample = {
            "timestamp": now_iso(),
            "docking_phase": job.get("phase", ""),
            "docking_state": job.get("state", state.get("state", "")),
            "goal_completion_policy": job.get("goal_completion_policy", ""),
            "dock_id": job.get("dock_id", ""),
            "dock_profile_approach_direction": job.get("approach_direction", ""),
            "predock_xy_ok": job.get("predock_xy_ok", ""),
            "predock_yaw_aligned": job.get("predock_yaw_aligned", ""),
            "predock_yaw_align_active": job.get("predock_yaw_align_active", ""),
            "fine_docking_entry_ready": job.get("fine_entry_ok", ""),
            "fine_docking_entry_block_reason": job.get("fine_entry_failure_code", ""),
            "dock_insertion_yaw_map": job.get("dock_pose", {}).get("yaw", "") if isinstance(job.get("dock_pose"), dict) else "",
            "expected_base_yaw_map": job.get("predock_expected_base_yaw", ""),
            "expected_contact_yaw_map": job.get("predock_expected_contact_yaw", ""),
            "target_odom_yaw_snapshot": job.get("target_odom_yaw_snapshot", ""),
            "current_map_base_yaw": job.get("predock_current_base_yaw", ""),
            "current_odom_base_yaw": "",
            "current_map_contact_yaw": job.get("predock_current_contact_yaw", ""),
            "current_odom_contact_yaw": "",
            "base_yaw_error": base_error,
            "contact_yaw_error": contact_error,
            "normalized_yaw_error": normalized,
            "xy_drift": job.get("predock_distance_m", ""),
            "cmd_vel_docking.linear.x": nested(getattr(node, "twists", {}), ["/cmd_vel_docking", "linear_x"], ""),
            "cmd_vel_docking.angular.z": nested(getattr(node, "twists", {}), ["/cmd_vel_docking", "angular_z"], ""),
            "cmd_vel_safe.linear.x": nested(getattr(node, "twists", {}), ["/cmd_vel_safe", "linear_x"], ""),
            "cmd_vel_safe.angular.z": nested(getattr(node, "twists", {}), ["/cmd_vel_safe", "angular_z"], ""),
            "cmd_vel.linear.x": nested(getattr(node, "twists", {}), ["/cmd_vel", "linear_x"], ""),
            "cmd_vel.angular.z": nested(getattr(node, "twists", {}), ["/cmd_vel", "angular_z"], ""),
            "robot_safety_state": nested(status, ["safety", "status"], nested(getattr(node, "strings", {}), ["/safety/status"], "")),
            "robot_safety_block_reason": nested(status, ["navigation", "normal_motion_blocked_reason"], ""),
            "mode_controller_desired_motion_mode": "",
            "mode_controller_actual_motion_mode": nested(getattr(node, "strings", {}), ["/ranger_mini3_mode_controller/status"], ""),
            "mode_switching": "",
            "can_0x221_angular_velocity": "",
            "local_state_odometry_yaw_delta": norm_angle(local_yaw - first_local_yaw) if isinstance(local_yaw, float) and first_local_yaw is not None else "",
            "wheel_odom_yaw_delta": norm_angle(wheel_yaw - first_wheel_yaw) if isinstance(wheel_yaw, float) and first_wheel_yaw is not None else "",
            "timeout_remaining": max(0.0, deadline - time.monotonic()),
            "predock_yaw_failure_code": job.get("predock_yaw_align_failure_code", ""),
        }
        samples.append(sample)
        next_sample = time.monotonic() + sample_period

if node is not None:
    node.destroy_node()
    rclpy.shutdown()

fields = [
    "timestamp",
    "docking_phase",
    "docking_state",
    "goal_completion_policy",
    "dock_id",
    "dock_profile_approach_direction",
    "predock_xy_ok",
    "predock_yaw_aligned",
    "predock_yaw_align_active",
    "fine_docking_entry_ready",
    "fine_docking_entry_block_reason",
    "dock_insertion_yaw_map",
    "expected_base_yaw_map",
    "expected_contact_yaw_map",
    "target_odom_yaw_snapshot",
    "current_map_base_yaw",
    "current_odom_base_yaw",
    "current_map_contact_yaw",
    "current_odom_contact_yaw",
    "base_yaw_error",
    "contact_yaw_error",
    "normalized_yaw_error",
    "xy_drift",
    "cmd_vel_docking.linear.x",
    "cmd_vel_docking.angular.z",
    "cmd_vel_safe.linear.x",
    "cmd_vel_safe.angular.z",
    "cmd_vel.linear.x",
    "cmd_vel.angular.z",
    "robot_safety_state",
    "robot_safety_block_reason",
    "mode_controller_desired_motion_mode",
    "mode_controller_actual_motion_mode",
    "mode_switching",
    "can_0x221_angular_velocity",
    "local_state_odometry_yaw_delta",
    "wheel_odom_yaw_delta",
    "timeout_remaining",
    "predock_yaw_failure_code",
]
with (out_dir / "timeline.csv").open("w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(samples)

def nonzero(sample, field):
    try:
        return abs(float(sample.get(field, 0.0))) > 1e-4
    except Exception:
        return False

def abs_float(value):
    try:
        return abs(float(value))
    except Exception:
        return None

cmd_generated = any(nonzero(s, "cmd_vel_docking.angular.z") or nonzero(s, "cmd_vel_docking.linear.x") for s in samples)
cmd_safety = any(nonzero(s, "cmd_vel_safe.angular.z") or nonzero(s, "cmd_vel_safe.linear.x") for s in samples)
cmd_base = any(nonzero(s, "cmd_vel.angular.z") or nonzero(s, "cmd_vel.linear.x") for s in samples)
mode_text = " ".join(str(s.get("mode_controller_actual_motion_mode", "")) for s in samples)
spinning = "SPINNING" in mode_text or "code: 2" in mode_text or '"code":2' in mode_text
local_deltas = [abs_float(s.get("local_state_odometry_yaw_delta")) for s in samples]
local_deltas = [x for x in local_deltas if x is not None]
yaw_moved = bool(local_deltas and max(local_deltas) > 0.01)
errors = [abs_float(s.get("normalized_yaw_error")) for s in samples]
errors = [x for x in errors if x is not None]
yaw_error_decreasing = bool(len(errors) >= 2 and errors[-1] < errors[0])
target_wrong = bool(cmd_generated and yaw_moved and len(errors) >= 2 and errors[-1] > errors[0] + 0.03)
path_blocked = bool(cmd_generated and not cmd_safety)
closure_pass = bool(cmd_generated and cmd_safety and cmd_base and yaw_moved and yaw_error_decreasing)

determinations = {
    "command_generated": cmd_generated,
    "command_reached_safety": cmd_safety,
    "command_reached_base": cmd_base,
    "chassis_entered_spinning": spinning,
    "mode_switching_blocking": "mode_switching" in mode_text.lower() and not spinning,
    "yaw_moved": yaw_moved,
    "yaw_error_decreasing": yaw_error_decreasing,
    "target_yaw_likely_wrong": target_wrong,
    "command_path_likely_blocked": path_blocked,
    "predock_yaw_alignment_closure_pass": closure_pass,
}

raw = {
    "metadata": {
        "duration_sec": duration_sec,
        "api_url": api_url,
        "dock_id_filter": dock_id_filter,
        "observe_only": True,
        "ros_import_error": ROS_IMPORT_ERROR,
    },
    "determinations": determinations,
    "samples": samples,
}
(out_dir / "raw.json").write_text(json.dumps(raw, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")

latest = samples[-1] if samples else {}
summary = [
    "# Predock Yaw Alignment Trace",
    "",
    f"- report_dir: `{out_dir}`",
    f"- samples: `{len(samples)}`",
    f"- latest_docking_state: `{latest.get('docking_state', '')}`",
    f"- latest_docking_phase: `{latest.get('docking_phase', '')}`",
    f"- latest_predock_yaw_aligned: `{latest.get('predock_yaw_aligned', '')}`",
    f"- latest_fine_entry_ready: `{latest.get('fine_docking_entry_ready', '')}`",
    f"- latest_fine_entry_block_reason: `{latest.get('fine_docking_entry_block_reason', '')}`",
    "",
    "## Determinations",
]
for key, value in determinations.items():
    summary.append(f"- {key}: `{str(value).lower()}`")
(out_dir / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
print(f"summary={out_dir / 'summary.md'}")
PY

echo "${PREFIX} wrote ${OUTPUT_DIR}"

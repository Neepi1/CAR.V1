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
GOAL_ID=""
POSE_ID=""
OUTPUT_DIR=""
PREFIX="[pose-required-observe]"

usage() {
  cat <<'EOF'
Usage:
  observe_pose_required_navigation.sh [--duration-sec N] [--goal-id ID] [--pose-id ID]

Default mode is observe-only. This script never sends navigation goals,
velocity commands, relocalization triggers, or ROS topic publications.
It samples lightweight API state, command-chain twist topics, action status
topics, bridge status, and rosout counters. It does not subscribe to heavy
pointcloud topics.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --goal-id)
      GOAL_ID="${2:-}"
      shift 2
      ;;
    --pose-id)
      POSE_ID="${2:-}"
      shift 2
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 10 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 10" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/pose_required_navigation_${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
python3 - "${DURATION_SEC}" "${API_URL}" "${OUTPUT_DIR}" "${GOAL_ID}" "${POSE_ID}" <<'PY'
import csv
import json
import math
import re
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

duration_sec = int(sys.argv[1])
api_url = sys.argv[2].rstrip("/")
out_dir = Path(sys.argv[3])
goal_filter = sys.argv[4]
pose_filter = sys.argv[5]

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from action_msgs.msg import GoalStatusArray
    from rcl_interfaces.msg import Log
except Exception as exc:  # pragma: no cover - runtime fallback on non-ROS hosts
    rclpy = None
    ROS_IMPORT_ERROR = repr(exc)
else:
    ROS_IMPORT_ERROR = ""


def now_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def api_get(path, timeout=1.0):
    req = urllib.request.Request(f"{api_url}{path}", method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except Exception as exc:
        return 0, {"ok": False, "error": str(exc)}


def nested(obj, path, default=None):
    cur = obj
    for part in path:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(part)
    return default if cur is None else cur


def as_float(value, default=""):
    try:
        number = float(value)
    except Exception:
        return default
    if not math.isfinite(number):
        return default
    return number


def parse_tolerance(reason, key):
    if not isinstance(reason, str):
        return ""
    patterns = {
        "distance": r"(?:position_tolerance|tolerance)=([0-9.]+)",
        "yaw": r"(?:yaw_tolerance)=([0-9.]+)",
    }
    match = re.search(patterns[key], reason)
    return match.group(1) if match else ""


class Observer(Node):
    def __init__(self):
        super().__init__("observe_pose_required_navigation")
        self.twists = {
            "/cmd_vel_collision_checked": None,
            "/cmd_vel_safe": None,
            "/cmd_vel": None,
        }
        self.bridge = {}
        self.safety_status = ""
        self.action_status = {
            "compute_path_to_pose": "",
            "follow_path": "",
            "navigate_to_pose": "",
        }
        self.local_costmap_filter_drops = 0
        self.controller_tf_extrapolations = 0
        self.create_subscription(Twist, "/cmd_vel_collision_checked", self._twist_cb("/cmd_vel_collision_checked"), 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._twist_cb("/cmd_vel_safe"), 10)
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb("/cmd_vel"), 10)
        self.create_subscription(String, "/localization/bridge_status", self._bridge_cb, 10)
        self.create_subscription(String, "/safety/status", self._safety_cb, 10)
        status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(GoalStatusArray, "/compute_path_to_pose/_action/status", self._status_cb("compute_path_to_pose"), status_qos)
        self.create_subscription(GoalStatusArray, "/follow_path/_action/status", self._status_cb("follow_path"), status_qos)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._status_cb("navigate_to_pose"), status_qos)
        self.create_subscription(Log, "/rosout", self._rosout_cb, status_qos)

    def _twist_cb(self, topic):
        def cb(msg):
            self.twists[topic] = {
                "linear_x": float(msg.linear.x),
                "angular_z": float(msg.angular.z),
                "stamp": now_iso(),
            }
        return cb

    def _bridge_cb(self, msg):
        try:
            self.bridge = json.loads(msg.data)
        except Exception:
            self.bridge = {}

    def _safety_cb(self, msg):
        self.safety_status = msg.data

    def _status_cb(self, name):
        def cb(msg):
            if msg.status_list:
                self.action_status[name] = ",".join(str(item.status) for item in msg.status_list)
            else:
                self.action_status[name] = "empty"
        return cb

    def _rosout_cb(self, msg):
        text = f"{msg.name} {msg.msg}"
        if "Message Filter dropping message" in text and "local_costmap" in text:
            self.local_costmap_filter_drops += 1
        if "Extrapolation" in text or "extrapolation" in text:
            if "controller" in text or "getTransform" in text or "transformPoseInTargetFrame" in text:
                self.controller_tf_extrapolations += 1


node = None
if rclpy is not None:
    rclpy.init()
    node = Observer()

samples = []
deadline = time.monotonic() + duration_sec
next_sample = 0.0
while time.monotonic() < deadline:
    if node is not None:
        rclpy.spin_once(node, timeout_sec=0.05)
    if time.monotonic() >= next_sample:
        status_code, nav_state = api_get("/api/v1/navigation/state")
        _, status = api_get("/api/v1/status")
        goal = nav_state.get("navigation_goal") or nav_state.get("goal") or nested(status, ["navigation", "goal"], {})
        if not isinstance(goal, dict):
            goal = {}
        if goal_filter and str(goal.get("id", "")) != goal_filter:
            match_filter = False
        elif pose_filter and str(goal.get("pose_id", "")) != pose_filter:
            match_filter = False
        else:
            match_filter = True
        reason = goal.get("final_pose_verify_reason", "")
        sample = {
            "timestamp": now_iso(),
            "api_status_code": status_code,
            "match_filter": match_filter,
            "goal_id": goal.get("id", ""),
            "pose_id": goal.get("pose_id", ""),
            "goal_completion_policy": goal.get("goal_completion_policy", ""),
            "position_reached": goal.get("position_reached", ""),
            "yaw_align_required": goal.get("yaw_align_required", ""),
            "yaw_align_active": goal.get("yaw_align_active", ""),
            "yaw_align_succeeded": goal.get("yaw_align_succeeded", ""),
            "yaw_align_failed": goal.get("yaw_align_failed", ""),
            "final_pose_verified": goal.get("final_pose_verified", ""),
            "task_complete": goal.get("task_complete", ""),
            "navigation_goal_state": goal.get("state", ""),
            "phase": goal.get("phase", ""),
            "final_distance": goal.get("final_distance_m", ""),
            "distance_tolerance": parse_tolerance(reason, "distance"),
            "yaw_error": goal.get("final_yaw_error_rad", ""),
            "yaw_tolerance": parse_tolerance(reason, "yaw"),
            "final_yaw_align_requested": goal.get("final_yaw_align_requested", ""),
            "final_yaw_align_cmd_active": goal.get("ordinary_final_yaw_align_active", ""),
            "cmd_vel_collision_checked.angular.z": nested(node.twists if node else {}, ["/cmd_vel_collision_checked", "angular_z"], ""),
            "cmd_vel_safe.angular.z": nested(node.twists if node else {}, ["/cmd_vel_safe", "angular_z"], ""),
            "cmd_vel.angular.z": nested(node.twists if node else {}, ["/cmd_vel", "angular_z"], ""),
            "safety_status": nested(status, ["safety", "status"], getattr(node, "safety_status", "")),
            "safety_block_reason": nested(status, ["navigation", "normal_motion_blocked_reason"], ""),
            "compute_path_to_pose_status": nested(getattr(node, "action_status", {}), ["compute_path_to_pose"], ""),
            "follow_path_status": nested(getattr(node, "action_status", {}), ["follow_path"], ""),
            "navigate_to_pose_status": nested(getattr(node, "action_status", {}), ["navigate_to_pose"], ""),
            "bridge_map_odom_publish_gap_ms": nested(getattr(node, "bridge", {}), ["map_odom_publish_gap_ms"], ""),
            "amcl_accepted_or_applied_correction_count": nested(getattr(node, "bridge", {}), ["amcl_accepted_count"], nested(getattr(node, "bridge", {}), ["last_accepted_sequence"], "")),
            "local_costmap_message_filter_drop_count": getattr(node, "local_costmap_filter_drops", 0),
            "controller_tf_extrapolation_count": getattr(node, "controller_tf_extrapolations", 0),
        }
        samples.append(sample)
        next_sample = time.monotonic() + 1.0

if node is not None:
    node.destroy_node()
    rclpy.shutdown()

raw = {
    "metadata": {
        "duration_sec": duration_sec,
        "api_url": api_url,
        "goal_filter": goal_filter,
        "pose_filter": pose_filter,
        "ros_import_error": ROS_IMPORT_ERROR,
        "observe_only": True,
    },
    "samples": samples,
}
(out_dir / "raw.json").write_text(json.dumps(raw, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")

fields = [
    "timestamp",
    "goal_id",
    "pose_id",
    "goal_completion_policy",
    "position_reached",
    "yaw_align_required",
    "yaw_align_active",
    "yaw_align_succeeded",
    "yaw_align_failed",
    "final_pose_verified",
    "task_complete",
    "navigation_goal_state",
    "phase",
    "final_distance",
    "distance_tolerance",
    "yaw_error",
    "yaw_tolerance",
    "final_yaw_align_requested",
    "final_yaw_align_cmd_active",
    "cmd_vel_collision_checked.angular.z",
    "cmd_vel_safe.angular.z",
    "cmd_vel.angular.z",
    "safety_status",
    "safety_block_reason",
    "compute_path_to_pose_status",
    "follow_path_status",
    "navigate_to_pose_status",
    "bridge_map_odom_publish_gap_ms",
    "amcl_accepted_or_applied_correction_count",
    "local_costmap_message_filter_drop_count",
    "controller_tf_extrapolation_count",
]
with (out_dir / "timeline.csv").open("w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(samples)

matched = [s for s in samples if s.get("match_filter")]
active = [s for s in matched if s.get("navigation_goal_state") not in ("", "idle")]
latest = active[-1] if active else (matched[-1] if matched else {})
failures = []
passes = []
unknowns = []

if any(s.get("goal_completion_policy") == "pose_required" for s in active):
    passes.append("ordinary navigation policy observed as pose_required")
elif active:
    failures.append("active ordinary navigation did not report goal_completion_policy=pose_required")
else:
    unknowns.append("no matching active navigation goal observed")

for s in active:
    if s.get("phase") == "position_reached_yaw_aligning" and str(s.get("task_complete")).lower() == "true":
        failures.append("position_reached_yaw_aligning sample had task_complete=true")
    if str(s.get("final_pose_verified")).lower() == "false" and str(s.get("task_complete")).lower() == "true":
        failures.append("final_pose_verified=false sample had task_complete=true")

if any(str(s.get("final_yaw_align_requested")).lower() == "true" for s in active):
    failures.append("API final_yaw_align was requested; Phase N3 expects Nav2-native yaw completion")
else:
    passes.append("API final_yaw_align was not requested; Nav2-native yaw completion path preserved")

failure_class = "none"
if latest.get("navigation_goal_state") == "failed":
    phase = latest.get("phase", "")
    if "follow" in phase.lower() or latest.get("controller_tf_extrapolation_count", 0):
        failure_class = "Nav2 FollowPath / controller TF"
    elif "final_yaw_align" in phase:
        failure_class = "final_yaw_align"
    elif "final_pose" in phase or "verify" in phase:
        failure_class = "final_pose_verify"
    elif latest.get("safety_block_reason"):
        failure_class = "safety block"
    else:
        failure_class = phase or "unknown failed phase"

summary = [
    "# Pose Required Navigation Observation",
    "",
    f"- report_dir: `{out_dir}`",
    f"- samples: `{len(samples)}`",
    f"- active_matching_samples: `{len(active)}`",
    f"- latest_goal_id: `{latest.get('goal_id', '')}`",
    f"- latest_pose_id: `{latest.get('pose_id', '')}`",
    f"- latest_policy: `{latest.get('goal_completion_policy', '')}`",
    f"- latest_state: `{latest.get('navigation_goal_state', '')}`",
    f"- latest_phase: `{latest.get('phase', '')}`",
    f"- latest_final_distance: `{latest.get('final_distance', '')}`",
    f"- latest_yaw_error: `{latest.get('yaw_error', '')}`",
    f"- local_costmap_message_filter_drop_count: `{latest.get('local_costmap_message_filter_drop_count', 0)}`",
    f"- controller_tf_extrapolation_count: `{latest.get('controller_tf_extrapolation_count', 0)}`",
    f"- failure_classification: `{failure_class}`",
    "",
    "## Verdict",
    f"- result: `{'FAIL' if failures else ('UNKNOWN' if unknowns and not passes else 'PASS')}`",
    f"- passes: `{passes}`",
    f"- failures: `{failures}`",
    f"- unknowns: `{unknowns}`",
]
(out_dir / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
print(f"summary={out_dir / 'summary.md'}")
PY

echo "${PREFIX} wrote ${OUTPUT_DIR}"

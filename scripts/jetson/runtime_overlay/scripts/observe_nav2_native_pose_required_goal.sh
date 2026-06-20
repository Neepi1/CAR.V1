#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

DURATION_SEC=180
LABEL="nav2_native_pose_required"

usage() {
  cat <<USAGE
observe_nav2_native_pose_required_goal.sh [--duration-sec N] [--label LABEL]

Read-only observer. It does not send goals, docking requests, relocalization
requests, velocity commands, or pointcloud subscriptions.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="$2"
      shift 2
      ;;
    --label)
      LABEL="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${WORKSPACE_ROOT}/reports/nav2_native_pose_required_goal_${TIMESTAMP}_${LABEL}"
mkdir -p "${OUTPUT_DIR}"

python3 - "$OUTPUT_DIR" "$DURATION_SEC" <<'PY'
import csv
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

out = Path(sys.argv[1])
duration = float(sys.argv[2])
api_url = "http://127.0.0.1:8080/api/v1/navigation/state"
raw_path = out / "raw.json"
timeline_path = out / "timeline.csv"
summary_path = out / "summary.md"

samples = []
deadline = time.monotonic() + duration
while time.monotonic() < deadline:
    stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    try:
        with urllib.request.urlopen(api_url, timeout=1.5) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except Exception as exc:
        payload = {"ok": False, "error": str(exc)}
    goal = payload.get("navigation_goal", {}) if isinstance(payload, dict) else {}
    safety = payload.get("safety", {}) if isinstance(payload, dict) else {}
    samples.append({
        "stamp": stamp,
        "ok": payload.get("ok"),
        "state": goal.get("state", ""),
        "phase": goal.get("phase", ""),
        "goal_id": goal.get("id", ""),
        "pose_id": goal.get("pose_id", ""),
        "goal_completion_policy": goal.get("goal_completion_policy", ""),
        "native_nav2_goal_completion": goal.get("native_nav2_goal_completion", payload.get("native_nav2_goal_completion", "")),
        "api_final_yaw_align_enabled": goal.get("api_final_yaw_align_enabled", payload.get("api_final_yaw_align_enabled", "")),
        "nav2_rotation_shim_enabled": goal.get("nav2_rotation_shim_enabled", payload.get("nav2_rotation_shim_enabled", "")),
        "nav2_result_code": goal.get("nav2_result_code", ""),
        "nav2_succeeded": goal.get("nav2_succeeded", ""),
        "final_distance_m": goal.get("final_distance_m", ""),
        "final_yaw_error_rad": goal.get("final_yaw_error_rad", ""),
        "position_reached": goal.get("position_reached", ""),
        "final_pose_verified": goal.get("final_pose_verified", ""),
        "task_complete": goal.get("task_complete", ""),
        "api_final_yaw_align_requested": goal.get("final_yaw_align_requested", ""),
        "api_final_yaw_align_attempted": goal.get("final_yaw_align_attempted", ""),
        "ordinary_final_yaw_align_active": goal.get("ordinary_final_yaw_align_active", ""),
        "predock_yaw_align_active": goal.get("predock_yaw_align_active", ""),
        "cmd_owner_conflict_detected": goal.get("cmd_owner_conflict_detected", ""),
        "safety_status": safety.get("status", ""),
        "motion_allowed": safety.get("motion_allowed", ""),
        "detail": goal.get("detail", ""),
    })
    time.sleep(1.0)

raw_path.write_text(json.dumps(samples, ensure_ascii=False, indent=2), encoding="utf-8")
fields = [
    "stamp", "goal_id", "pose_id", "goal_completion_policy", "native_nav2_goal_completion",
    "api_final_yaw_align_enabled", "nav2_rotation_shim_enabled", "state", "phase",
    "nav2_result_code", "nav2_succeeded", "final_distance_m", "final_yaw_error_rad",
    "position_reached", "final_pose_verified", "task_complete", "api_final_yaw_align_requested",
    "api_final_yaw_align_attempted", "ordinary_final_yaw_align_active", "predock_yaw_align_active",
    "cmd_owner_conflict_detected", "safety_status", "motion_allowed", "detail",
]
with timeline_path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()
    for sample in samples:
        writer.writerow({k: sample.get(k, "") for k in fields})

active = [s for s in samples if s.get("goal_completion_policy") == "pose_required"]
latest = samples[-1] if samples else {}
api_final_yaw_used = any(
    str(s.get("api_final_yaw_align_attempted")).lower() == "true" or
    str(s.get("ordinary_final_yaw_align_active")).lower() == "true"
    for s in samples
)
native_used = any(str(s.get("native_nav2_goal_completion")).lower() == "true" for s in active)
goal_success_xy_yaw = any(
    str(s.get("task_complete")).lower() == "true" and
    str(s.get("final_pose_verified")).lower() == "true"
    for s in active
)
final_yaw_handled_by_nav2 = native_used and not api_final_yaw_used
cmd_owner_conflict = any(str(s.get("cmd_owner_conflict_detected")).lower() == "true" for s in samples)

def fnum(v):
    try:
        return float(v)
    except Exception:
        return math.nan

distances = [fnum(s.get("final_distance_m")) for s in samples if not math.isnan(fnum(s.get("final_distance_m")))]
xy_drift_after_yaw_observed = False
if len(distances) >= 2:
    xy_drift_after_yaw_observed = (max(distances) - min(distances)) > 0.05

summary = [
    "# Nav2 Native Pose Required Goal Observation",
    "",
    f"- samples: `{len(samples)}`",
    f"- latest_goal_id: `{latest.get('goal_id', '')}`",
    f"- latest_pose_id: `{latest.get('pose_id', '')}`",
    f"- latest_policy: `{latest.get('goal_completion_policy', '')}`",
    f"- latest_state: `{latest.get('state', '')}`",
    f"- latest_phase: `{latest.get('phase', '')}`",
    f"- latest_final_distance_m: `{latest.get('final_distance_m', '')}`",
    f"- latest_final_yaw_error_rad: `{latest.get('final_yaw_error_rad', '')}`",
    f"- nav2_native_goal_completion_used: `{str(native_used).lower()}`",
    f"- api_final_yaw_align_used: `{str(api_final_yaw_used).lower()}`",
    f"- final_yaw_handled_by_nav2: `{str(final_yaw_handled_by_nav2).lower()}`",
    f"- goal_success_xy_yaw: `{str(goal_success_xy_yaw).lower()}`",
    f"- xy_drift_after_yaw_observed: `{str(xy_drift_after_yaw_observed).lower()}`",
    f"- cmd_owner_conflict: `{str(cmd_owner_conflict).lower()}`",
    "",
    "Artifacts:",
    f"- timeline: `{timeline_path}`",
    f"- raw: `{raw_path}`",
]
summary_path.write_text("\n".join(summary) + "\n", encoding="utf-8")
print(f"[nav2-native-observe] summary {summary_path}")
PY

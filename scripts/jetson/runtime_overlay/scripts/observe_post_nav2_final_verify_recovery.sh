#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
DURATION_SEC=180
LABEL="post_nav2_final_verify"
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:?}"
      shift 2
      ;;
    --label)
      LABEL="${2:?}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:?}"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage:
  observe_post_nav2_final_verify_recovery.sh [--duration-sec N] [--label LABEL]

Read-only observer. It does not send navigation goals, docking requests,
relocalization requests, or velocity commands. Start it before or during a
normal pose_required navigation goal.
EOF
      exit 0
      ;;
    *)
      echo "[post-nav2-final-observe] unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]]; then
  echo "[post-nav2-final-observe] --duration-sec must be an integer" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/post_nav2_final_verify_recovery/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

SAMPLES="${OUTPUT_DIR}/samples.jsonl"
SUMMARY="${OUTPUT_DIR}/summary.md"

python3 - "$DURATION_SEC" "$SAMPLES" "$SUMMARY" <<'PY'
import json
import subprocess
import sys
import time
import urllib.request

duration = int(sys.argv[1])
samples_path = sys.argv[2]
summary_path = sys.argv[3]

def get_json(path):
    try:
        with urllib.request.urlopen("http://127.0.0.1:8080" + path, timeout=1.0) as response:
            return json.loads(response.read().decode("utf-8"))
    except Exception as exc:
        return {"_error": repr(exc)}

def bridge_status():
    try:
        out = subprocess.check_output(
            [
                "bash",
                "-lc",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                "source install/setup.bash >/dev/null 2>&1; "
                "timeout 1 ros2 topic echo --once /localization/bridge_status 2>/dev/null",
            ],
            cwd="/workspaces/njrh-v3/workspace1",
            timeout=2.0,
            text=True,
        )
        for line in out.splitlines():
            if line.startswith("data: "):
                raw = line[len("data: "):].strip()
                if raw.startswith("'") and raw.endswith("'"):
                    raw = raw[1:-1]
                return json.loads(raw)
        return {"_error": "bridge_status_no_data"}
    except Exception as exc:
        return {"_error": repr(exc)}

started = time.time()
last_goal = {}
events = []
with open(samples_path, "w", encoding="utf-8") as f:
    while time.time() - started < duration:
        nav = get_json("/api/v1/navigation/state")
        goal = nav.get("navigation_goal", {}) if isinstance(nav, dict) else {}
        bridge = bridge_status()
        sample = {
            "t": time.time(),
            "goal": {
                "id": goal.get("id"),
                "state": goal.get("state"),
                "phase": goal.get("phase"),
                "detail": goal.get("detail"),
                "nav2_succeeded": goal.get("nav2_succeeded"),
                "final_distance_m": goal.get("final_distance_m"),
                "final_yaw_error_rad": goal.get("final_yaw_error_rad"),
                "task_complete": goal.get("task_complete"),
                "final_pose_verified": goal.get("final_pose_verified"),
                "final_verify_retry_count": goal.get("final_verify_retry_count"),
                "final_verify_retry_reason": goal.get("final_verify_retry_reason"),
                "final_verify_retry_goal_sent": goal.get("final_verify_retry_goal_sent"),
                "post_nav2_final_verify_bridge_wait_elapsed_ms": goal.get("post_nav2_final_verify_bridge_wait_elapsed_ms"),
                "post_nav2_final_verify_bridge_wait_timeout": goal.get("post_nav2_final_verify_bridge_wait_timeout"),
                "final_yaw_align_attempted": goal.get("final_yaw_align_attempted"),
                "api_final_yaw_align_enabled": goal.get("api_final_yaw_align_enabled"),
            },
            "bridge": {
                "correction_active": bridge.get("correction_active"),
                "safe_for_goal_start": bridge.get("safe_for_goal_start"),
                "remaining_translation_error_m": bridge.get("remaining_translation_error_m"),
                "remaining_yaw_error_rad": bridge.get("remaining_yaw_error_rad"),
                "last_accepted_correction_translation_m": bridge.get("last_accepted_correction_translation_m"),
                "last_accept_reason": bridge.get("last_accept_reason"),
            },
        }
        f.write(json.dumps(sample, ensure_ascii=False) + "\n")
        if goal != last_goal:
            events.append(sample)
            last_goal = dict(goal)
        time.sleep(1.0)

latest = events[-1] if events else {}
goal = latest.get("goal", {})
bridge = latest.get("bridge", {})
lines = [
    "# Post-Nav2 Final Verify Recovery Observation",
    "",
    f"- duration_sec: `{duration}`",
    f"- samples: `{samples_path}`",
    f"- latest_goal_id: `{goal.get('id', '')}`",
    f"- latest_goal_state: `{goal.get('state', '')}`",
    f"- latest_goal_phase: `{goal.get('phase', '')}`",
    f"- latest_task_complete: `{goal.get('task_complete', '')}`",
    f"- latest_final_distance_m: `{goal.get('final_distance_m', '')}`",
    f"- latest_final_yaw_error_rad: `{goal.get('final_yaw_error_rad', '')}`",
    f"- latest_final_verify_retry_count: `{goal.get('final_verify_retry_count', '')}`",
    f"- latest_final_verify_retry_reason: `{goal.get('final_verify_retry_reason', '')}`",
    f"- latest_final_verify_retry_goal_sent: `{goal.get('final_verify_retry_goal_sent', '')}`",
    f"- latest_bridge_wait_elapsed_ms: `{goal.get('post_nav2_final_verify_bridge_wait_elapsed_ms', '')}`",
    f"- latest_bridge_wait_timeout: `{goal.get('post_nav2_final_verify_bridge_wait_timeout', '')}`",
    f"- api_final_yaw_align_attempted: `{goal.get('final_yaw_align_attempted', '')}`",
    f"- bridge_correction_active: `{bridge.get('correction_active', '')}`",
    f"- bridge_safe_for_goal_start: `{bridge.get('safe_for_goal_start', '')}`",
    f"- bridge_remaining_translation_error_m: `{bridge.get('remaining_translation_error_m', '')}`",
    f"- bridge_last_accepted_correction_translation_m: `{bridge.get('last_accepted_correction_translation_m', '')}`",
]
with open(summary_path, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
print(f"[post-nav2-final-observe] summary {summary_path}")
PY

#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=common_env.sh
  source "${SCRIPT_DIR}/common_env.sh"
fi

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=180
LABEL="navigation_final_yaw_align"
OUTPUT_DIR=""
PREFIX="[nav-final-yaw-observe]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/observe_navigation_final_yaw_align.sh \
    --duration-sec 180 \
    --label nav_goal_1

Read-only observer. Start it, then trigger a normal navigation goal from the App.
It polls /api/v1/navigation/state once per second and never sends goals,
velocity commands, or ROS topic publications.
EOF
}

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
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
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/navigation_final_yaw_align/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

STATE_JSONL="${OUTPUT_DIR}/navigation_state.jsonl"
SUMMARY_MD="${OUTPUT_DIR}/summary.md"

{
  echo "timestamp_utc=${TIMESTAMP}"
  echo "duration_sec=${DURATION_SEC}"
  echo "label=${LABEL}"
  echo "api_url=${API_URL}"
  echo "workspace_root=${WORKSPACE_ROOT}"
} >"${OUTPUT_DIR}/metadata.env"

END_TIME=$((SECONDS + DURATION_SEC))
while [[ "${SECONDS}" -lt "${END_TIME}" ]]; do
  NOW_ISO="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  BODY="$(curl -fsS --max-time 1 "${API_URL}/api/v1/navigation/state" 2>/dev/null || true)"
  if [[ -z "${BODY}" ]]; then
    printf '{"observed_at":"%s","ok":false,"error":"api_unavailable"}\n' "${NOW_ISO}" >>"${STATE_JSONL}"
  else
    python3 - "${NOW_ISO}" "${BODY}" >>"${STATE_JSONL}" <<'PY'
import json
import sys
observed_at, raw = sys.argv[1], sys.argv[2]
try:
    payload = json.loads(raw)
except json.JSONDecodeError as exc:
    print(json.dumps({"observed_at": observed_at, "ok": False, "error": f"json_decode:{exc}"}))
    raise SystemExit(0)
payload["observed_at"] = observed_at
print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
PY
  fi
  sleep 1
done

python3 - "${STATE_JSONL}" "${SUMMARY_MD}" <<'PY'
import json
import sys
from collections import Counter

state_path, summary_path = sys.argv[1], sys.argv[2]
samples = []
for line in open(state_path, encoding="utf-8"):
    line = line.strip()
    if not line:
        continue
    try:
        samples.append(json.loads(line))
    except json.JSONDecodeError:
        pass

goals = [s.get("navigation_goal") or {} for s in samples if isinstance(s.get("navigation_goal"), dict)]
phases = Counter(g.get("phase", "") for g in goals if g.get("phase"))
states = Counter(g.get("state", "") for g in goals if g.get("state"))
policies = Counter(g.get("goal_completion_policy", "") for g in goals if g.get("goal_completion_policy"))

def first_time(predicate):
    for sample, goal in zip((s for s in samples if isinstance(s.get("navigation_goal"), dict)), goals):
        if predicate(goal):
            return sample.get("observed_at", "")
    return ""

latest_goal = goals[-1] if goals else {}
position_time = first_time(lambda g: g.get("position_reached") is True)
yaw_start_time = first_time(lambda g: g.get("yaw_align_active") is True or g.get("phase") == "position_reached_yaw_aligning")
verified_time = first_time(lambda g: g.get("final_pose_verified") is True)
task_complete_time = first_time(lambda g: g.get("task_complete") is True)

lines = [
    "# Navigation final yaw observation",
    "",
    f"- samples: `{len(samples)}`",
    f"- states: `{dict(states)}`",
    f"- phases: `{dict(phases)}`",
    f"- goal_completion_policy: `{dict(policies)}`",
    f"- position_reached_time: `{position_time}`",
    f"- yaw_align_start_time: `{yaw_start_time}`",
    f"- final_pose_verified_time: `{verified_time}`",
    f"- task_complete_time: `{task_complete_time}`",
    f"- latest_state: `{latest_goal.get('state', '')}`",
    f"- latest_phase: `{latest_goal.get('phase', '')}`",
    f"- latest_detail: `{latest_goal.get('detail', '')}`",
    f"- final_distance_m: `{latest_goal.get('final_distance_m', '')}`",
    f"- final_yaw_error_rad: `{latest_goal.get('final_yaw_error_rad', '')}`",
    f"- yaw_align_required: `{latest_goal.get('yaw_align_required', '')}`",
    f"- yaw_align_active: `{latest_goal.get('yaw_align_active', '')}`",
    f"- yaw_align_succeeded: `{latest_goal.get('yaw_align_succeeded', '')}`",
    f"- yaw_align_failed: `{latest_goal.get('yaw_align_failed', '')}`",
    f"- final_pose_verified: `{latest_goal.get('final_pose_verified', '')}`",
    f"- task_complete: `{latest_goal.get('task_complete', '')}`",
    f"- final_yaw_align_retry_count: `{latest_goal.get('final_yaw_align_retry_count', '')}`",
    f"- reposition_after_yaw_drift_retry_count: `{latest_goal.get('reposition_after_yaw_drift_retry_count', '')}`",
    f"- cmd_owner_conflict_detected: `{latest_goal.get('cmd_owner_conflict_detected', '')}`",
]
open(summary_path, "w", encoding="utf-8").write("\n".join(lines) + "\n")
PY

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${SUMMARY_MD}"

#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=180
LABEL="normal_nav_minimal"
OUTPUT_DIR=""
PREFIX="[normal-nav-observe]"
CLI_KILL_AFTER_SEC="${NJRH_DIAG_CLI_KILL_AFTER_SEC:-5}"

usage() {
  cat <<'USAGE'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/observe_normal_navigation_minimal_path.sh --duration-sec 180 --label nav_test_1

Read-only observer. Start it, then send a normal navigation goal from the App.
It records whether force_accept or /global_localization/trigger occurred near
the goal, plus Nav2 action status, bridge smoothing status, and filtered rosout.
USAGE
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 20 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 20" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/normal_navigation_minimal_path/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

PIDS=()

terminate_process_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "-${pid}" >/dev/null 2>&1 || kill -TERM "${pid}" >/dev/null 2>&1 || true
  fi
}

wait_process_group_exit() {
  local pid="$1"
  local deadline=$((SECONDS + CLI_KILL_AFTER_SEC))
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    if ! kill -0 "${pid}" 2>/dev/null && ! pgrep -g "${pid}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    terminate_process_group "${pid}"
  done
  for pid in "${PIDS[@]:-}"; do
    wait_process_group_exit "${pid}" || {
      echo "${PREFIX} WARN diagnostic process group ${pid} did not exit after TERM; sending targeted KILL" >>"${OUTPUT_DIR}/cleanup.log"
      kill -KILL "-${pid}" >/dev/null 2>&1 || kill -KILL "${pid}" >/dev/null 2>&1 || true
    }
  done
}
trap cleanup EXIT INT TERM

start_logged() {
  local name="$1"
  shift
  local log_file="${OUTPUT_DIR}/${name}.log"
  setsid bash -c '
    log_file="$1"
    shift
    {
      echo "$ $*"
      "$@"
    } >"${log_file}" 2>&1
  ' _ "${log_file}" "$@" &
  PIDS+=("$!")
}

start_timeout_logged() {
  local name="$1"
  shift
  start_logged "${name}" timeout --kill-after="${CLI_KILL_AFTER_SEC}s" --signal=TERM "${DURATION_SEC}" "$@"
}

api_snapshot() {
  local name="$1"
  {
    echo "GET ${API_URL}/api/v1/status"
    curl -fsS "${API_URL}/api/v1/status" || true
    echo
    echo "GET ${API_URL}/api/v1/navigation/state"
    curl -fsS "${API_URL}/api/v1/navigation/state" || true
    echo
  } >"${OUTPUT_DIR}/${name}.json" 2>"${OUTPUT_DIR}/${name}.err"
}

start_api_poll() {
  start_logged "api_poll" bash -lc '
    api_url="$1"
    duration_sec="$2"
    deadline=$((SECONDS + duration_sec))
    while [[ "${SECONDS}" -lt "${deadline}" ]]; do
      printf "{\"captured_at\":\"%s\",\"navigation_state\":" "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
      curl -fsS "${api_url}/api/v1/navigation/state" 2>/dev/null || printf "null"
      printf ",\"status\":"
      curl -fsS "${api_url}/api/v1/status" 2>/dev/null || printf "null"
      printf "}\n"
      sleep 1
    done
  ' _ "${API_URL}" "${DURATION_SEC}"
}

write_summary() {
  if ! python3 - "${OUTPUT_DIR}" <<'PY' >"${OUTPUT_DIR}/summary.md" 2>"${OUTPUT_DIR}/summary.err"; then
import json
import pathlib
import re
import sys

out = pathlib.Path(sys.argv[1])

def text(name):
    path = out / name
    return path.read_text(errors="ignore") if path.exists() else ""

def json_objects(src):
    dec = json.JSONDecoder()
    pos = 0
    result = []
    while True:
        start = src.find("{", pos)
        if start < 0:
            return result
        try:
            obj, end = dec.raw_decode(src[start:])
        except Exception:
            pos = start + 1
            continue
        result.append(obj)
        pos = start + end

poll = json_objects(text("api_poll.log"))
nav_states = [x.get("navigation_state") for x in poll if isinstance(x, dict)]
statuses = [x.get("status") for x in poll if isinstance(x, dict)]
last_nav = next((x for x in reversed(nav_states) if isinstance(x, dict)), {})
last_status = next((x for x in reversed(statuses) if isinstance(x, dict)), {})
bridge_raw = text("echo_bridge_status.log")
bridge_matches = re.findall(r"data: '([^']+)'", bridge_raw)
bridge = {}
if bridge_matches:
    try:
        bridge = json.loads(bridge_matches[-1])
    except Exception:
        bridge = {}
rosout = text("rosout_filtered.log")
force_mentions = len(re.findall(r"force accepting next localization_result|force_accept", rosout, re.I))
trigger_mentions = len(re.findall(r"/global_localization/trigger|triggering localization", rosout, re.I))

print("# Normal Navigation Minimal Path Observation")
print()
print(f"- report_dir: `{out}`")
print(f"- final_navigation_state: `{last_nav.get('state')}`")
print(f"- final_goal_state: `{(last_nav.get('navigation_goal') or {}).get('state')}`")
print(f"- final_goal_phase: `{(last_nav.get('navigation_goal') or {}).get('phase')}`")
print(f"- compute_path_status_messages: `{text('compute_path_to_pose_status.log').count('---')}`")
print(f"- follow_path_status_messages: `{text('follow_path_status.log').count('---')}`")
print(f"- force_accept_mentions: `{force_mentions}`")
print(f"- global_localization_trigger_mentions: `{trigger_mentions}`")
print(f"- bridge_safe_for_goal_start_last: `{bridge.get('safe_for_goal_start')}`")
print(f"- bridge_correction_active_last: `{bridge.get('correction_active')}`")
print(f"- bridge_remaining_translation_error_m_last: `{bridge.get('remaining_translation_error_m')}`")
print(f"- bridge_map_odom_gap_ms_last: `{bridge.get('map_odom_publish_gap_ms')}`")
print(f"- status_localization_recovery_required_last: `{((last_status.get('localization') or {}) if isinstance(last_status, dict) else {}).get('localization_recovery_required')}`")
print()
print("## Inspect First")
print("- `api_poll.log`")
print("- `echo_bridge_status.log`")
print("- `compute_path_to_pose_status.log`")
print("- `follow_path_status.log`")
print("- `rosout_filtered.log`")
PY
    {
      echo "# Normal Navigation Minimal Path Observation"
      echo
      echo "- report_dir: \`${OUTPUT_DIR}\`"
      echo "- summary_error: see \`summary.err\`"
    } >"${OUTPUT_DIR}/summary.md"
  fi
}

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} start App navigation now if you want to capture a live run"

api_snapshot "api_before"
start_api_poll
start_timeout_logged "echo_bridge_status" ros2 topic echo /localization/bridge_status --full-length
start_timeout_logged "compute_path_to_pose_status" ros2 topic echo /compute_path_to_pose/_action/status --full-length
start_timeout_logged "follow_path_status" ros2 topic echo /follow_path/_action/status --full-length
start_timeout_logged "cmd_vel_collision_checked" ros2 topic echo /cmd_vel_collision_checked --full-length
start_timeout_logged "cmd_vel_safe" ros2 topic echo /cmd_vel_safe --full-length
start_timeout_logged "cmd_vel" ros2 topic echo /cmd_vel --full-length
start_timeout_logged "rosout_filtered" bash -lc "timeout '${DURATION_SEC}' ros2 topic echo /rosout --qos-reliability best_effort 2>/dev/null | grep -Ei 'navigate|NavigateToPose|compute_path|FollowPath|force_accept|force accepting|global_localization|triggering localization|localization_result|bridge|safe_for_goal_start|controller_server|planner_server|bt_navigator|tf|extrapolat|Message Filter|abort|failed|succeed|success' || true"

sleep "${DURATION_SEC}"
cleanup
api_snapshot "api_after"
write_summary

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"

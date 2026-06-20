#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=180
LABEL="predock_yaw_align"
OUTPUT_DIR=""
PREFIX="[dock-yaw-observe]"
CLI_KILL_AFTER_SEC="${NJRH_DIAG_CLI_KILL_AFTER_SEC:-5}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/observe_docking_predock_yaw_align.sh --duration-sec 180 --label dock_test_1

Read-only observer. Start it, then trigger return-to-dock from the App.
It does not send goals or velocity. It records API docking state, native Nav2
predock XY+yaw verification, optional explicit /cmd_vel_docking fallback, Ranger
mode alignment, GS2 scan freshness, safety, and filtered rosout lines for the
capture window.
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 20 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 20" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/docking_predock_yaw_align/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

PIDS=()
write_dds_env_log() {
  {
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
    echo "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-}"
    echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
    echo "FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-}"
    echo "NJRH_FASTDDS_PROFILE_ENABLED=${NJRH_FASTDDS_PROFILE_ENABLED:-}"
    echo "NJRH_FASTDDS_ALLOWED_ADDRESSES=${NJRH_FASTDDS_ALLOWED_ADDRESSES:-}"
    echo "NJRH_FASTDDS_ALLOWED_INTERFACES=${NJRH_FASTDDS_ALLOWED_INTERFACES:-}"
    if [[ -n "${FASTDDS_DEFAULT_PROFILES_FILE:-}" && -f "${FASTDDS_DEFAULT_PROFILES_FILE}" ]]; then
      echo "FASTDDS_DEFAULT_PROFILES_FILE_EXISTS=true"
    else
      echo "FASTDDS_DEFAULT_PROFILES_FILE_EXISTS=false"
    fi
    ip -br addr 2>/dev/null || true
  } >"${OUTPUT_DIR}/dds_env.log" 2>&1
}

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
    echo "GET ${API_URL}/api/v1/docking/state"
    curl -fsS "${API_URL}/api/v1/docking/state" || true
    echo
    echo "GET ${API_URL}/api/v1/navigation/state"
    curl -fsS "${API_URL}/api/v1/navigation/state" || true
    echo
  } >"${OUTPUT_DIR}/${name}.json" 2>"${OUTPUT_DIR}/${name}.err"
}

api_poll_loop() {
  local deadline=$((SECONDS + DURATION_SEC))
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    printf '{"captured_at":"%s","docking_state":' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    curl -fsS "${API_URL}/api/v1/docking/state" 2>/dev/null || printf 'null'
    printf ',"bridge_status":null}\n'
    sleep 1
  done
}

start_api_poll() {
  start_logged "api_poll" bash -lc '
    api_url="$1"
    duration_sec="$2"
    deadline=$((SECONDS + duration_sec))
    while [[ "${SECONDS}" -lt "${deadline}" ]]; do
      printf "{\"captured_at\":\"%s\",\"docking_state\":" "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
      curl -fsS "${api_url}/api/v1/docking/state" 2>/dev/null || printf "null"
      printf ",\"bridge_status\":null}\n"
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
    p = out / name
    return p.read_text(errors="ignore") if p.exists() else ""

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

def select_docking_state(objs):
    for obj in reversed(objs):
        if not isinstance(obj, dict):
            continue
        if "docking_active" in obj and isinstance(obj.get("docking"), dict):
            return obj
    return {}

poll = json_objects(text("api_poll.log"))
last = poll[-1] if poll else {}
dock = last.get("docking_state") or {}
if not dock:
    dock = select_docking_state(json_objects(text("api_after.json")))
job = dock.get("docking") or {}
bridge = last.get("bridge_status") or {}
if not bridge:
    raw_bridge = text("echo_bridge_status.log")
    matches = re.findall(r"data: '([^']+)'", raw_bridge)
    if matches:
        bridge = matches[-1]
if isinstance(bridge, str):
    try:
        bridge = json.loads(bridge)
    except Exception:
        bridge = {}
if not isinstance(bridge, dict):
    bridge = {}
cmd = text("echo_cmd_vel_docking.log")
mode = text("echo_mode_controller_status.log")
actual_spinning_mentions = len(re.findall(r'SPINNING|code: 2|"code":2', mode))

print("# Docking Predock Native Nav2 Observation")
print()
print(f"- report_dir: `{out}`")
print(f"- final_docking_state: `{dock.get('state')}`")
print(f"- final_phase: `{job.get('phase')}`")
print(f"- nav_goal_succeeded: `{job.get('nav_goal_succeeded')}`")
print(f"- predock_pose_verified: `{job.get('predock_pose_verified')}`")
print(f"- predock_yaw_verified_by_nav2: `{job.get('predock_yaw_verified_by_nav2')}`")
print(f"- predock_xy_ok: `{job.get('predock_xy_ok')}`")
print(f"- predock_base_yaw_ok: `{job.get('predock_base_yaw_ok')}`")
print(f"- predock_contact_yaw_ok: `{job.get('predock_contact_yaw_ok')}`")
print(f"- expected_base_yaw_at_predock: `{job.get('expected_base_yaw_at_predock')}`")
print(f"- current_base_yaw_map: `{job.get('current_base_yaw_map')}`")
print(f"- base_yaw_error: `{job.get('base_yaw_error')}`")
print(f"- reverse_yaw_offset_applied: `{job.get('reverse_yaw_offset_applied')}`")
print(f"- contact_frame_available: `{job.get('contact_frame_available')}`")
print(f"- predock_yaw_align_attempted: `{job.get('predock_yaw_align_attempted')}`")
print(f"- predock_yaw_align_succeeded: `{job.get('predock_yaw_align_succeeded')}`")
print(f"- predock_yaw_align_failure_code: `{job.get('predock_yaw_align_failure_code')}`")
print(f"- fine_entry_ok: `{job.get('fine_entry_ok')}`")
print(f"- fine_entry_failure_code: `{job.get('fine_entry_failure_code')}`")
print(f"- fine_bridge_settle_started: `{job.get('fine_bridge_settle_started')}`")
print(f"- fine_bridge_settle_complete: `{job.get('fine_bridge_settle_complete')}`")
print(f"- fine_bridge_settle_failure_code: `{job.get('fine_bridge_settle_failure_code')}`")
print(f"- fine_bridge_settle_duration_sec: `{job.get('fine_bridge_settle_duration_sec')}`")
print(f"- fine_bridge_settle_remaining_translation_m: `{job.get('fine_bridge_settle_remaining_translation_m')}`")
print(f"- fine_bridge_settle_remaining_yaw_rad: `{job.get('fine_bridge_settle_remaining_yaw_rad')}`")
print(f"- global_correction_paused_last: `{job.get('global_correction_paused')}`")
print(f"- bridge_global_correction_paused_last: `{bridge.get('global_correction_paused')}`")
print(f"- cmd_vel_docking_messages: `{cmd.count('---')}`")
print(f"- mode_status_messages: `{mode.count('---')}`")
print(f"- actual_spinning_mentions: `{actual_spinning_mentions}`")
print()
print("## Inspect First")
print("- `api_poll.log`")
print("- `echo_cmd_vel_docking.log`")
print("- `echo_mode_controller_status.log`")
print("- `echo_bridge_status.log`")
print("- `rosout_filtered.log`")
PY
    {
      echo "# Docking Predock Native Nav2 Observation"
      echo
      echo "- report_dir: \`${OUTPUT_DIR}\`"
      echo "- summary_error: see \`summary.err\`"
      echo
      echo "## Inspect First"
      echo "- \`api_poll.log\`"
      echo "- \`echo_cmd_vel_docking.log\`"
      echo "- \`echo_mode_controller_status.log\`"
      echo "- \`echo_bridge_status.log\`"
      echo "- \`rosout_filtered.log\`"
    } >"${OUTPUT_DIR}/summary.md"
  fi
}

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} start App docking now if you want to capture a live run"

write_dds_env_log
api_snapshot "api_before"
start_api_poll
start_timeout_logged "echo_docking_status" ros2 topic echo /docking/status --full-length
start_timeout_logged "echo_cmd_vel_docking" ros2 topic echo /cmd_vel_docking --full-length
start_timeout_logged "echo_cmd_vel_safe" ros2 topic echo /cmd_vel_safe --full-length
start_timeout_logged "echo_cmd_vel" ros2 topic echo /cmd_vel --full-length
start_timeout_logged "echo_mode_controller_status" ros2 topic echo /ranger_mini3_mode_controller/status --full-length
start_timeout_logged "echo_desired_motion_mode" ros2 topic echo /ranger_mini3/desired_motion_mode --full-length
start_timeout_logged "echo_motion_state" ros2 topic echo /motion_state --full-length
start_timeout_logged "echo_system_state" ros2 topic echo /system_state --full-length
start_timeout_logged "echo_bridge_status" ros2 topic echo /localization/bridge_status --full-length
start_timeout_logged "echo_safety_status" ros2 topic echo /safety/status --full-length
start_timeout_logged "echo_safety_motion_allowed" ros2 topic echo /safety/motion_allowed --full-length
start_timeout_logged "hz_gs2_scan" ros2 topic hz /dock/gs2_scan
start_timeout_logged "hz_cmd_vel_docking" ros2 topic hz /cmd_vel_docking
start_timeout_logged "rosout_filtered" bash -lc "timeout '${DURATION_SEC}' ros2 topic echo /rosout --qos-reliability best_effort 2>/dev/null | grep -Ei 'dock|predock|yaw|fine|gs2|correction|pause|safety|motion_mode|cmd_vel|tf|relocal|failed|failure|timeout|reject|transformPose|Message Filter|ActionServer|Aborting|FollowPath|controller_server|planner_server|bt_navigator|extrapolat|global plan|costmap' || true"

sleep "${DURATION_SEC}"
cleanup
api_snapshot "api_after"
write_summary

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"

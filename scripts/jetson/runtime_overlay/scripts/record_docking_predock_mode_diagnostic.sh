#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=120
LABEL="dock_predock_mode"
OUTPUT_DIR=""
RECORD_BAG=true
PREFIX="[dock-predock-mode]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/record_docking_predock_mode_diagnostic.sh
  bash scripts/jetson/runtime_overlay/scripts/record_docking_predock_mode_diagnostic.sh --duration-sec 150 --label dock_retry_1

Read-only recorder for docking / pre-dock navigation failures. Start this script,
then trigger docking from the App while the capture is active.

It records:
  - API navigation/docking/status snapshots
  - Nav2 action status and rosout errors
  - command chain: cmd_vel_nav_raw, cmd_vel_collision_checked, cmd_vel_safe, cmd_vel, cmd_vel_docking
  - Ranger mode control topics: desired/forced/reverse/park plus best-effort motion_state/system_state capture
  - wheel/local odom and localization bridge status
  - runtime log lines written during the capture window
  - optional rosbag with only low-bandwidth diagnostic topics, no pointcloud

Options:
  --duration-sec N   Capture duration in seconds. Default: 120.
  --label LABEL      Report label. Default: dock_predock_mode.
  --api-url URL      robot_api_server base URL. Default: http://127.0.0.1:8080.
  --output-dir DIR   Report directory. Default: reports/docking_predock_mode/<timestamp>_<label>_<duration>s.
  --bag              Enable rosbag recording. Default.
  --no-bag           Disable rosbag recording.
  -h, --help         Show this help.
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
    --bag)
      RECORD_BAG=true
      shift
      ;;
    --no-bag)
      RECORD_BAG=false
      shift
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
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/docking_predock_mode/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}/logs_since_start"

PIDS=()
LOG_FILES=(
  "${NJRH_RUNTIME_LOG_DIR}/robot_api_server.log"
  "${NJRH_RUNTIME_LOG_DIR}/resident_navigation_runtime.log"
  "${NJRH_RUNTIME_LOG_DIR}/ranger_mini3_mode_controller_common.log"
  "${NJRH_RUNTIME_LOG_DIR}/ranger_mini3_mode_controller.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_safety_common.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_safety.log"
  "${NJRH_RUNTIME_LOG_DIR}/docking_manager.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_localization_bridge.log"
  "${NJRH_RUNTIME_LOG_DIR}/global_localization_localization.log"
  "${NJRH_RUNTIME_LOG_DIR}/amcl_scan_admission.log"
)

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

run_logged() {
  local name="$1"
  shift
  {
    echo "\$ $*"
    "$@"
  } >"${OUTPUT_DIR}/${name}.log" 2>&1
}

start_logged() {
  local name="$1"
  shift
  {
    echo "\$ $*"
    "$@"
  } >"${OUTPUT_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

start_timeout_logged() {
  local name="$1"
  shift
  start_logged "${name}" timeout --signal=INT "${DURATION_SEC}" "$@"
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
    echo "GET ${API_URL}/api/v1/docking/state"
    curl -fsS "${API_URL}/api/v1/docking/state" || true
    echo
  } >"${OUTPUT_DIR}/${name}.json" 2>"${OUTPUT_DIR}/${name}.err"
}

api_poll_loop() {
  local deadline=$((SECONDS + DURATION_SEC))
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    printf '{"captured_at":"%s","status":' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    curl -fsS "${API_URL}/api/v1/status" 2>/dev/null || printf 'null'
    printf ',"navigation_state":'
    curl -fsS "${API_URL}/api/v1/navigation/state" 2>/dev/null || printf 'null'
    printf ',"docking_state":'
    curl -fsS "${API_URL}/api/v1/docking/state" 2>/dev/null || printf 'null'
    printf '}\n'
    sleep 1
  done
}

sample_proc_loop() {
  local deadline=$((SECONDS + DURATION_SEC))
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    {
      echo "=== $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
      ps -eo pid,psr,pcpu,pmem,etime,comm,args --sort=-pcpu |
        grep -E 'controller_server|bt_navigator|planner_server|velocity_smoother|collision_monitor|robot_api_server|docking_manager|ranger_mini3_mode_controller|robot_safety|localization_bridge|amcl|hesai|ranger_base' |
        grep -v grep || true
      echo
    } >>"${OUTPUT_DIR}/process_cpu.log" 2>&1
    sleep 1
  done
}

record_bag() {
  local bag_dir="${OUTPUT_DIR}/bag"
  mkdir -p "${bag_dir}"
  timeout --signal=INT "${DURATION_SEC}" ros2 bag record \
    --output "${bag_dir}" \
    /cmd_vel_nav_raw \
    /cmd_vel_smoothed \
    /cmd_vel_collision_checked \
    /cmd_vel_safe \
    /cmd_vel \
    /cmd_vel_docking \
    /ranger_mini3/desired_motion_mode \
    /ranger_mini3/forced_mode \
    /ranger_mini3/docking_allow_reverse \
    /ranger_mini3/park \
    /motion_state \
    /system_state \
    /docking/status \
    /safety/status \
    /safety/motion_allowed \
    /wheel/odom \
    /local_state/odometry \
    /localization/bridge_status \
    /amcl_pose \
    /amcl_scan_admission/status \
    /navigate_to_pose/_action/status \
    /follow_path/_action/status \
    /compute_path_to_pose/_action/status \
    /tf \
    /tf_static
}

record_rosout_filtered() {
  timeout "${DURATION_SEC}" ros2 topic echo /rosout --qos-reliability best_effort 2>/dev/null |
    grep -Ei 'dock|predock|pre-dock|failed|abort|cancel|progress|collision|safety|tf|transform|extrapolat|message filter|controller|bt_navigator|goal|amcl|costmap|exception|follow_path|motion mode|cmd_vel' \
    >"${OUTPUT_DIR}/rosout_filtered.log" || true
}

record_candump_221() {
  if command -v candump >/dev/null 2>&1; then
    timeout "${DURATION_SEC}" candump -L 'can0,221:7FF' >"${OUTPUT_DIR}/can_221_motion_state.log" 2>&1 || true
  else
    echo "candump not found" >"${OUTPUT_DIR}/can_221_motion_state.log"
  fi
}

capture_log_positions() {
  local out="${OUTPUT_DIR}/log_start_lines.tsv"
  : >"${out}"
  local file
  for file in "${LOG_FILES[@]}"; do
    if [[ -f "${file}" ]]; then
      printf '%s\t%s\n' "${file}" "$(wc -l <"${file}")" >>"${out}"
    fi
  done
}

copy_logs_since_start() {
  local tsv="${OUTPUT_DIR}/log_start_lines.tsv"
  local file line base out
  [[ -f "${tsv}" ]] || return 0
  while IFS=$'\t' read -r file line; do
    [[ -f "${file}" ]] || continue
    base="$(basename "${file}")"
    out="${OUTPUT_DIR}/logs_since_start/${base}"
    tail -n +"$((line + 1))" "${file}" >"${out}" 2>&1 || true
    grep -Ei 'dock|predock|pre-dock|relocal|localization|initialpose|settle|NavigateToPose|FollowPath|Goal|goal|abort|aborted|failed|succeed|success|cancel|controller|planner|bt_navigator|TF|extrapolat|Message Filter|progress|patience|collision|costmap|cmd_vel|motion mode|desired|actual|safety' \
      "${out}" >"${out%.log}.filtered.log" 2>/dev/null || true
  done <"${tsv}"
}

write_summary() {
  python3 - "${OUTPUT_DIR}" <<'PY' >"${OUTPUT_DIR}/summary.md" 2>/dev/null || true
import json
import pathlib
import re
import sys

out = pathlib.Path(sys.argv[1])

def text(name):
    p = out / name
    return p.read_text(errors="ignore") if p.exists() else ""

def json_objects(src):
    decoder = json.JSONDecoder()
    pos = 0
    objs = []
    while True:
        start = src.find("{", pos)
        if start < 0:
            break
        try:
            obj, end = decoder.raw_decode(src[start:])
        except Exception:
            pos = start + 1
            continue
        objs.append(obj)
        pos = start + end
    return objs

def nested(obj, path):
    cur = obj
    for part in path.split("."):
        if not isinstance(cur, dict):
            return None
        cur = cur.get(part)
    return cur

def twist_stats(log_name):
    body = text(log_name)
    vals = [float(v) for v in re.findall(r"^\s*(?:x|y|z):\s*(-?\d+(?:\.\d+)?)\s*$", body, re.M)]
    return {
        "messages": body.count("---"),
        "max_abs": max((abs(v) for v in vals), default=0.0),
    }

api = json_objects(text("api_after.json"))
status = api[0] if len(api) > 0 else {}
nav = api[1] if len(api) > 1 else {}
dock = api[2] if len(api) > 2 else {}

filtered = "\n".join(p.read_text(errors="ignore") for p in sorted((out / "logs_since_start").glob("*.filtered.log")))
mode_mismatch = re.findall(r"desired motion mode .*?actual .*?(?:\n|$)", filtered)

print("# Docking Predock Mode Diagnostic Summary")
print()
print(f"- report_dir: `{out}`")
print(f"- api_status_mode: `{nested(status, 'mode')}`")
print(f"- navigation_state: `{nested(nav, 'state')}`")
print(f"- docking_state: `{nested(dock, 'state')}`")
print(f"- docking_last_status: `{nested(dock, 'last_status')}`")
print(f"- docking_job_detail: `{nested(dock, 'docking.detail')}`")
print(f"- docking_nav_goal_sent: `{nested(dock, 'docking.nav_goal_sent')}`")
print(f"- docking_nav_goal_succeeded: `{nested(dock, 'docking.nav_goal_succeeded')}`")
print(f"- docking_service_called: `{nested(dock, 'docking.docking_service_called')}`")
print(f"- docking_relocalization_requested: `{nested(dock, 'docking.relocalization_requested')}`")
print(f"- docking_post_predock_relocalization_requested: `{nested(dock, 'docking.post_predock_relocalization_requested')}`")
print(f"- nav_goal_detail: `{nested(nav, 'navigation_goal.detail')}`")
print(f"- nav_goal_result_code: `{nested(nav, 'navigation_goal.nav2_result_code')}`")
print()
print("## Command Chain")
for name in [
    "echo_cmd_vel_nav_raw.log",
    "echo_cmd_vel_smoothed.log",
    "echo_cmd_vel_collision_checked.log",
    "echo_cmd_vel_safe.log",
    "echo_cmd_vel.log",
    "echo_cmd_vel_docking.log",
]:
    stats = twist_stats(name)
    print(f"- {name}: messages={stats['messages']} max_abs_component={stats['max_abs']:.6f}")
print()
print("## Failure Signals In Logs")
print(f"- failed_to_make_progress_count: `{filtered.count('Failed to make progress')}`")
print(f"- future_extrapolation_count: `{len(re.findall('extrapolation into the future', filtered, re.I))}`")
print(f"- message_filter_drop_count: `{len(re.findall('Message Filter dropping message', filtered))}`")
print(f"- desired_actual_mode_mismatch_count: `{len(mode_mismatch)}`")
for item in mode_mismatch[-10:]:
    print(f"- mode_mismatch: `{item.strip()}`")
print()
print("## Raw Files To Inspect First")
print("- `logs_since_start/resident_navigation_runtime.filtered.log`")
print("- `logs_since_start/ranger_mini3_mode_controller_common.filtered.log`")
print("- `api_poll.jsonl`")
print("- `rosout_filtered.log`")
print("- `bag/` if rosbag recording succeeded")
PY
}

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC}"
echo "${PREFIX} start the App docking command now, keep this terminal open"

capture_log_positions
api_snapshot "api_before"

run_logged "topic_list" ros2 topic list
run_logged "node_list" ros2 node list
run_logged "topic_info_cmd_chain" bash -lc \
  "for t in /cmd_vel_nav_raw /cmd_vel_smoothed /cmd_vel_collision_checked /cmd_vel_safe /cmd_vel /cmd_vel_docking /ranger_mini3/desired_motion_mode /motion_state /system_state /docking/status /safety/status /wheel/odom /local_state/odometry; do echo === \$t ===; ros2 topic info -v \$t 2>&1 || true; done"

start_logged "api_poll" api_poll_loop
start_logged "process_cpu" sample_proc_loop
start_logged "rosout_filtered" record_rosout_filtered
start_logged "can_221_motion_state" record_candump_221

start_timeout_logged "echo_cmd_vel_nav_raw" ros2 topic echo /cmd_vel_nav_raw --full-length
start_timeout_logged "echo_cmd_vel_smoothed" ros2 topic echo /cmd_vel_smoothed --full-length
start_timeout_logged "echo_cmd_vel_collision_checked" ros2 topic echo /cmd_vel_collision_checked --full-length
start_timeout_logged "echo_cmd_vel_safe" ros2 topic echo /cmd_vel_safe --full-length
start_timeout_logged "echo_cmd_vel" ros2 topic echo /cmd_vel --full-length
start_timeout_logged "echo_cmd_vel_docking" ros2 topic echo /cmd_vel_docking --full-length
start_timeout_logged "echo_desired_motion_mode" ros2 topic echo /ranger_mini3/desired_motion_mode --full-length
start_timeout_logged "echo_forced_mode" ros2 topic echo /ranger_mini3/forced_mode --full-length
start_timeout_logged "echo_docking_allow_reverse" ros2 topic echo /ranger_mini3/docking_allow_reverse --full-length
start_timeout_logged "echo_park" ros2 topic echo /ranger_mini3/park --full-length
start_timeout_logged "echo_motion_state_best_effort" ros2 topic echo /motion_state --full-length
start_timeout_logged "echo_system_state_best_effort" ros2 topic echo /system_state --full-length
start_timeout_logged "echo_docking_status" ros2 topic echo /docking/status --full-length
start_timeout_logged "echo_safety_status" ros2 topic echo /safety/status --full-length
start_timeout_logged "echo_safety_motion_allowed" ros2 topic echo /safety/motion_allowed --full-length
start_timeout_logged "echo_wheel_odom" ros2 topic echo /wheel/odom --full-length
start_timeout_logged "echo_local_state_odometry" ros2 topic echo /local_state/odometry --full-length
start_timeout_logged "echo_bridge_status" ros2 topic echo /localization/bridge_status --full-length
start_timeout_logged "echo_nav_action_status" ros2 topic echo /navigate_to_pose/_action/status --full-length
start_timeout_logged "echo_follow_path_status" ros2 topic echo /follow_path/_action/status --full-length

start_timeout_logged "hz_cmd_vel_nav_raw" ros2 topic hz /cmd_vel_nav_raw
start_timeout_logged "hz_cmd_vel_collision_checked" ros2 topic hz /cmd_vel_collision_checked
start_timeout_logged "hz_cmd_vel_safe" ros2 topic hz /cmd_vel_safe
start_timeout_logged "hz_cmd_vel" ros2 topic hz /cmd_vel
start_timeout_logged "hz_wheel_odom" ros2 topic hz /wheel/odom
start_timeout_logged "hz_local_state_odometry" ros2 topic hz /local_state/odometry

if [[ "${RECORD_BAG}" == "true" ]]; then
  start_logged "rosbag_record" record_bag
fi

sleep "${DURATION_SEC}"
cleanup
api_snapshot "api_after"
copy_logs_since_start
write_summary

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"

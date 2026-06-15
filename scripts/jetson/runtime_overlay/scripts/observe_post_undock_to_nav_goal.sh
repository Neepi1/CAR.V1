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
LABEL="post_undock_to_nav_goal"
REPORT=""
PREFIX="[post-undock-nav-observe]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/observe_post_undock_to_nav_goal.sh --duration-sec 180

Read-only observer for Phase U1. Start it before sending a normal navigation
goal from the App while the robot is docked. The script does not send goals,
does not publish velocity, and does not subscribe to heavy point clouds.
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
    --report)
      REPORT="${2:-}"
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
OUT_DIR="${NJRH_PROJECT_ROOT}/reports/post_undock_to_nav_goal_${TIMESTAMP}_${LABEL}"
mkdir -p "${OUT_DIR}"
if [[ -z "${REPORT}" ]]; then
  REPORT="${OUT_DIR}.md"
fi

PIDS=()
cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

start_logged() {
  local name="$1"
  shift
  {
    echo "\$ $*"
    "$@"
  } >"${OUT_DIR}/${name}.log" 2>&1 &
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
    echo "GET ${API_URL}/api/v1/navigation/state"
    curl -fsS "${API_URL}/api/v1/navigation/state" || true
    echo
    echo "GET ${API_URL}/api/v1/docking/state"
    curl -fsS "${API_URL}/api/v1/docking/state" || true
    echo
    echo "GET ${API_URL}/api/v1/status"
    curl -fsS "${API_URL}/api/v1/status" || true
    echo
  } >"${OUT_DIR}/${name}.json" 2>"${OUT_DIR}/${name}.err"
}

api_poll_loop() {
  local deadline=$((SECONDS + DURATION_SEC))
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    printf '{"captured_at":"%s","navigation_state":' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    curl -fsS "${API_URL}/api/v1/navigation/state" 2>/dev/null || printf 'null'
    printf ',"docking_state":'
    curl -fsS "${API_URL}/api/v1/docking/state" 2>/dev/null || printf 'null'
    printf ',"bridge_status":'
    bridge_status="$(timeout 1 ros2 topic echo /localization/bridge_status --once --full-length 2>/dev/null | sed -n 's/^data: //p' | tail -n 1)"
    if [[ -n "${bridge_status}" ]]; then
      printf '%s' "${bridge_status}"
    else
      printf 'null'
    fi
    printf '}\n'
    sleep 1
  done
}

write_summary() {
  python3 - "${OUT_DIR}" "${REPORT}" <<'PY' >"${OUT_DIR}/summary.tmp" 2>"${OUT_DIR}/summary.err"
import json
import pathlib
import re
import sys

out = pathlib.Path(sys.argv[1])
report = pathlib.Path(sys.argv[2])

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

poll = json_objects(text("api_poll.log"))
first = poll[0] if poll else {}
last = poll[-1] if poll else {}

def nav(obj):
    return obj.get("navigation_state") or {}

def dock(obj):
    return obj.get("docking_state") or {}

def job_from_dock(obj):
    d = dock(obj)
    return d.get("docking") or {}

def settle_from_nav(obj):
    n = nav(obj)
    return n.get("post_undock_settle") or {}

def first_time(predicate):
    for item in poll:
        if predicate(item):
            return item.get("captured_at", "")
    return ""

undock_required_time = first_time(
    lambda item: bool((nav(item).get("pre_navigation_dock_check") or {}).get("final_auto_undock_required")))
undock_start_time = first_time(lambda item: job_from_dock(item).get("phase") == "undocking")
undock_success_time = first_time(
    lambda item: job_from_dock(item).get("phase") == "relocalize_after_undock" or
    job_from_dock(item).get("state") == "undocked")
relocalize_start_time = first_time(
    lambda item: bool(job_from_dock(item).get("post_undock_relocalization_started")))
bridge_accept_time = first_time(
    lambda item: bool(job_from_dock(item).get("post_undock_relocalization_accepted")))
settle_start_time = first_time(
    lambda item: bool(job_from_dock(item).get("post_undock_settle_started")) or
    bool(settle_from_nav(item).get("post_undock_settle_started")))
settle_pass_time = first_time(
    lambda item: bool(job_from_dock(item).get("post_undock_settle_complete")) or
    bool(settle_from_nav(item).get("post_undock_settle_complete")))
nav_goal_send_time = first_time(
    lambda item: (nav(item).get("navigation_goal") or {}).get("state") == "running" or
    (nav(item).get("navigation_goal") or {}).get("nav2_result_code", 0) != 0)

rosout = text("rosout_filtered.log")
cmd_safe = text("echo_cmd_vel_safe.log")
cmd = text("echo_cmd_vel.log")
last_nav = nav(last)
last_dock_job = job_from_dock(last)
last_settle = settle_from_nav(last)

lines = [
    "# Post-Undock To Nav Goal Observation",
    "",
    f"- report_dir: `{out}`",
    f"- duration_sec: `{len(poll)}` samples",
    f"- navigation_request_time: `{poll[0].get('captured_at', '') if poll else ''}`",
    f"- auto_undock_required_time: `{undock_required_time}`",
    f"- auto_undock_reason: `{((last_nav.get('pre_navigation_dock_check') or {}).get('auto_undock_reason', ''))}`",
    f"- undock_start_time: `{undock_start_time}`",
    f"- undock_success_time: `{undock_success_time}`",
    f"- post_undock_relocalization_trigger_time: `{relocalize_start_time}`",
    f"- bridge_accept_time: `{bridge_accept_time}`",
    f"- settle_barrier_start_time: `{settle_start_time}`",
    f"- settle_barrier_pass_time: `{settle_pass_time}`",
    f"- settle_failure_reason: `{last_settle.get('post_undock_settle_failure_reason') or last_dock_job.get('post_undock_settle_failure_reason', '')}`",
    f"- pending_goal_held: `{last_settle.get('pending_goal_held_for_post_undock_settle', last_dock_job.get('pending_goal_held_for_post_undock_settle'))}`",
    f"- pending_goal_released: `{last_settle.get('pending_goal_released_after_post_undock_settle', last_dock_job.get('pending_goal_released_after_post_undock_settle'))}`",
    f"- nav2_goal_send_time: `{nav_goal_send_time}`",
    f"- nav2_result_code: `{(last_nav.get('navigation_goal') or {}).get('nav2_result_code')}`",
    f"- nav2_state: `{(last_nav.get('navigation_goal') or {}).get('state')}`",
    f"- nav2_phase: `{(last_nav.get('navigation_goal') or {}).get('phase')}`",
    f"- controller_tf_extrapolation_count: `{len(re.findall(r'extrapolat|TF_OLD_DATA|TF.*future|future.*TF', rosout, re.I))}`",
    f"- local_costmap_message_filter_drop_count: `{len(re.findall(r'local_costmap.*MessageFilter|Message Filter.*local_costmap|dropping.*local_costmap', rosout, re.I))}`",
    f"- cmd_vel_safe_messages: `{cmd_safe.count('---')}`",
    f"- cmd_vel_messages: `{cmd.count('---')}`",
    "",
    "## Inspect First",
    "- `api_poll.log`",
    "- `rosout_filtered.log`",
    "- `echo_docking_status.log`",
    "- `echo_bridge_status.log`",
    "- `echo_cmd_vel_safe.log`",
    "- `echo_cmd_vel.log`",
]
report.write_text("\n".join(lines) + "\n")
print(report)
PY
  if [[ "$?" -ne 0 ]]; then
    {
      echo "# Post-Undock To Nav Goal Observation"
      echo
      echo "- report_dir: \`${OUT_DIR}\`"
      echo "- summary_error: see \`${OUT_DIR}/summary.err\`"
    } >"${REPORT}"
  fi
}

echo "${PREFIX} report=${REPORT}"
echo "${PREFIX} start App navigation from docked state now if you want to capture a live run"

api_snapshot "api_before"
start_logged "api_poll" api_poll_loop
start_timeout_logged "echo_docking_status" ros2 topic echo /docking/status --full-length
start_timeout_logged "echo_bridge_status" ros2 topic echo /localization/bridge_status --full-length
start_timeout_logged "echo_cmd_vel_safe" ros2 topic echo /cmd_vel_safe --full-length
start_timeout_logged "echo_cmd_vel" ros2 topic echo /cmd_vel --full-length
start_timeout_logged "rosout_filtered" bash -lc "timeout '${DURATION_SEC}' ros2 topic echo /rosout --qos-reliability best_effort 2>/dev/null | grep -Ei 'undock|post.?undock|relocal|settle|navigate|NavigateToPose|controller|local_costmap|MessageFilter|tf|extrapolat|abort|result code|cmd_vel' || true"

sleep "${DURATION_SEC}"
cleanup
api_snapshot "api_after"
write_summary

echo "${PREFIX} wrote ${OUT_DIR}"
echo "${PREFIX} summary ${REPORT}"

#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=30
POST_GOAL_JSON=""
POST_GOAL_DELAY_SEC=3
OUTPUT_DIR=""
RECORD_BAG=false
PREFIX="[undock-diag]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/record_undock_diagnostic.sh
  bash scripts/jetson/runtime_overlay/scripts/record_undock_diagnostic.sh --duration-sec 40
  bash scripts/jetson/runtime_overlay/scripts/record_undock_diagnostic.sh --post-goal-json '{"pose_id":"delivery_512355","building_id":"B10","floor_id":"F1"}'

Records one pre-navigation undock attempt from a single terminal. Start this
script, then send a navigation goal from the App while the capture is active.
It writes API snapshots, docking/safety/mode topics, command-chain echoes,
odometry, and a small summary into a timestamped report directory.

This script is read-only unless --post-goal-json is provided. It never publishes
velocity commands by itself.

Options:
  --duration-sec N       Capture duration in seconds. Default: 30.
  --api-url URL          robot_api_server base URL. Default: http://127.0.0.1:8080.
  --output-dir DIR       Report directory. Default: reports/undock_diagnostics/<utc timestamp>.
  --post-goal-json JSON  POST this JSON to /api/v1/navigation/goal after recorders start.
  --post-delay-sec N     Delay before POST when --post-goal-json is used. Default: 3.
  --bag                  Also record a small rosbag with undock topics.
  -h, --help             Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
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
    --post-goal-json)
      POST_GOAL_JSON="${2:-}"
      shift 2
      ;;
    --post-delay-sec)
      POST_GOAL_DELAY_SEC="${2:-}"
      shift 2
      ;;
    --bag)
      RECORD_BAG=true
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 8 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 8" >&2
  exit 2
fi

if ! [[ "${POST_GOAL_DELAY_SEC}" =~ ^[0-9]+$ ]]; then
  echo "${PREFIX} FAIL --post-delay-sec must be an integer" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/undock_diagnostics/${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

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
    printf '{"captured_at":"%s","navigation_state":' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    curl -fsS "${API_URL}/api/v1/navigation/state" 2>/dev/null || printf 'null'
    printf ',"docking_state":'
    curl -fsS "${API_URL}/api/v1/docking/state" 2>/dev/null || printf 'null'
    printf '}\n'
    sleep 1
  done
}

write_summary() {
  python3 - "${OUTPUT_DIR}" <<'PY' >"${OUTPUT_DIR}/summary.md" 2>/dev/null || true
import json
import math
import pathlib
import re
import sys

out = pathlib.Path(sys.argv[1])

def read(name):
    path = out / name
    return path.read_text(errors="ignore") if path.exists() else ""

def json_objects_from_text(text):
    decoder = json.JSONDecoder()
    pos = 0
    objects = []
    while True:
        start = text.find("{", pos)
        if start < 0:
            break
        try:
            payload, end = decoder.raw_decode(text[start:])
        except Exception:
            pos = start + 1
            continue
        objects.append(payload)
        pos = start + end
    return objects

def get_path(payload, dotted):
    cur = payload
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur

def scalar_values_from_log(text, field):
    return [m.group(1).strip().strip("'\"") for m in re.finditer(rf"^\s*{re.escape(field)}:\s*(.+?)\s*$", text, re.M)]

def twist_stats(text):
    values = [float(v) for v in re.findall(r"^\s*(?:x|y|z):\s*(-?\d+(?:\.\d+)?)\s*$", text, re.M)]
    max_abs = max((abs(v) for v in values), default=0.0)
    return {
        "messages": text.count("---"),
        "max_abs_component": max_abs,
        "nonzero": max_abs > 1.0e-6,
    }

def bool_seen(text, value):
    return re.search(rf"^\s*data:\s*{str(value).lower()}\s*$", text, re.M) is not None

def odom_motion(text):
    xs = [float(v) for v in re.findall(r"^\s*x:\s*(-?\d+(?:\.\d+)?)\s*$", text, re.M)]
    ys = [float(v) for v in re.findall(r"^\s*y:\s*(-?\d+(?:\.\d+)?)\s*$", text, re.M)]
    # Odometry logs also contain orientation x/y. Keep the conservative fallback:
    # pair every first x/y occurrence stream and report span only as a hint.
    count = min(len(xs), len(ys))
    if count < 2:
        return {"samples": 0, "max_delta_m": 0.0}
    base_x, base_y = xs[0], ys[0]
    max_delta = 0.0
    for x, y in zip(xs[:count], ys[:count]):
        max_delta = max(max_delta, math.hypot(x - base_x, y - base_y))
    return {"samples": count, "max_delta_m": max_delta}

api_after = json_objects_from_text(read("api_after.json"))
status = api_after[0] if len(api_after) >= 1 else {}
nav_state = api_after[1] if len(api_after) >= 2 else {}
docking_state = api_after[2] if len(api_after) >= 3 else {}

cmd_docking = twist_stats(read("echo_cmd_vel_docking.log"))
cmd_safe = twist_stats(read("echo_cmd_vel_safe.log"))
cmd_base = twist_stats(read("echo_cmd_vel.log"))
odom = odom_motion(read("echo_local_state_odometry.log"))
reverse_text = read("echo_docking_allow_reverse.log")
forced_modes = scalar_values_from_log(read("echo_forced_mode.log"), "data")
park_text = read("echo_park.log")
docking_statuses = scalar_values_from_log(read("echo_docking_status.log"), "data")
safety_statuses = scalar_values_from_log(read("echo_safety_status.log"), "data")

print("# Undock Diagnostic Summary")
print()
print(f"- report_dir: `{out}`")
print(f"- api_navigation_state: `{get_path(nav_state, 'state')}`")
print(f"- api_docking_state: `{get_path(docking_state, 'state')}`")
print(f"- api_docking_last_status: `{get_path(docking_state, 'last_status')}`")
print(f"- api_docking_job_detail: `{get_path(docking_state, 'docking.detail')}`")
print(f"- safety_status_after: `{get_path(status, 'safety.status')}`")
print(f"- blocked_by_docked_contact_after: `{get_path(nav_state, 'blocked_by_docked_contact')}`")
print()
print("## Command Chain")
print(f"- /cmd_vel_docking: messages={cmd_docking['messages']} max_abs_component={cmd_docking['max_abs_component']:.6f} nonzero={cmd_docking['nonzero']}")
print(f"- /cmd_vel_safe: messages={cmd_safe['messages']} max_abs_component={cmd_safe['max_abs_component']:.6f} nonzero={cmd_safe['nonzero']}")
print(f"- /cmd_vel: messages={cmd_base['messages']} max_abs_component={cmd_base['max_abs_component']:.6f} nonzero={cmd_base['nonzero']}")
print(f"- /local_state/odometry: samples_hint={odom['samples']} max_delta_hint_m={odom['max_delta_m']:.6f}")
print()
print("## Mode Signals")
print(f"- docking_allow_reverse_true_observed: `{bool_seen(reverse_text, True)}`")
print(f"- docking_allow_reverse_false_observed: `{bool_seen(reverse_text, False)}`")
print(f"- forced_mode_values: `{', '.join(forced_modes[-10:]) if forced_modes else 'none'}`")
print(f"- park_true_observed: `{bool_seen(park_text, True)}`")
print(f"- park_false_observed: `{bool_seen(park_text, False)}`")
print()
print("## Status Streams")
print(f"- docking_status_tail: `{', '.join(docking_statuses[-8:]) if docking_statuses else 'none'}`")
print(f"- safety_status_tail: `{', '.join(safety_statuses[-8:]) if safety_statuses else 'none'}`")
print()
print("## First Read")
if not cmd_docking["nonzero"]:
    print("- No nonzero /cmd_vel_docking was captured. The undock service may not have started, or the capture window missed the 2s undock attempt.")
elif not cmd_safe["nonzero"]:
    print("- /cmd_vel_docking was nonzero but /cmd_vel_safe was not. Focus on robot_safety blocking, watchdog timing, or arbitration.")
elif odom["max_delta_m"] < 0.005:
    print("- /cmd_vel_safe was nonzero but odometry barely changed. Focus on Ranger mode controller, chassis acceptance, reverse permit timing, or physical dock constraint.")
else:
    print("- Command chain and odometry both moved during capture. If the API still failed, inspect the exact distance threshold and timing in raw logs.")
PY
}

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC}"
echo "${PREFIX} api_url=${API_URL}"
echo "${PREFIX} read_only=$([[ -z "${POST_GOAL_JSON}" ]] && echo true || echo false)"

api_snapshot "api_before"
start_logged "api_state_poll.jsonl" api_poll_loop

start_timeout_logged "echo_cmd_vel_docking" ros2 topic echo /cmd_vel_docking
start_timeout_logged "echo_cmd_vel_safe" ros2 topic echo /cmd_vel_safe
start_timeout_logged "echo_cmd_vel" ros2 topic echo /cmd_vel
start_timeout_logged "echo_docking_status" ros2 topic echo /docking/status
start_timeout_logged "echo_safety_status" ros2 topic echo /safety/status
start_timeout_logged "echo_local_state_odometry" ros2 topic echo /local_state/odometry
start_timeout_logged "echo_battery_state" ros2 topic echo /battery_state
start_timeout_logged "echo_docking_allow_reverse" ros2 topic echo /ranger_mini3/docking_allow_reverse
start_timeout_logged "echo_forced_mode" ros2 topic echo /ranger_mini3/forced_mode
start_timeout_logged "echo_park" ros2 topic echo /ranger_mini3/park
start_timeout_logged "echo_mode_controller_status" ros2 topic echo /ranger_mini3_mode_controller/status

start_timeout_logged "hz_local_state_odometry" ros2 topic hz /local_state/odometry --window 20
start_timeout_logged "hz_cmd_vel_docking" ros2 topic hz /cmd_vel_docking --window 20
start_timeout_logged "hz_cmd_vel_safe" ros2 topic hz /cmd_vel_safe --window 20

run_logged "topic_info_before" bash -lc \
  'for topic in /cmd_vel_docking /cmd_vel_safe /cmd_vel /docking/status /safety/status /local_state/odometry /ranger_mini3/docking_allow_reverse /ranger_mini3/forced_mode /ranger_mini3/park /ranger_mini3_mode_controller/status /battery_state; do echo "## ${topic}"; timeout 6 ros2 topic info -v "${topic}"; done'
run_logged "nodes_before" timeout 8 ros2 node list
run_logged "processes_before" bash -lc \
  'pgrep -af "docking_manager_node|robot_safety_node|ranger_mini3_mode_controller|ranger_base_node|robot_api_server" || true'

echo "${PREFIX} CAPTURE ACTIVE: send the App navigation goal now."
echo "${PREFIX} Do not open more terminals; wait until this script prints done."

if [[ "${RECORD_BAG}" == "true" ]]; then
  BAG_DIR="${OUTPUT_DIR}/rosbag_undock"
  start_timeout_logged "rosbag_record" ros2 bag record -o "${BAG_DIR}" \
    /cmd_vel_docking \
    /cmd_vel_safe \
    /cmd_vel \
    /docking/status \
    /safety/status \
    /local_state/odometry \
    /battery_state \
    /ranger_mini3/docking_allow_reverse \
    /ranger_mini3/forced_mode \
    /ranger_mini3/park \
    /ranger_mini3_mode_controller/status
fi

if [[ -n "${POST_GOAL_JSON}" ]]; then
  (
    sleep "${POST_GOAL_DELAY_SEC}"
    {
      echo "POST ${API_URL}/api/v1/navigation/goal"
      curl -fsS -X POST "${API_URL}/api/v1/navigation/goal" \
        -H "Content-Type: application/json" \
        --data "${POST_GOAL_JSON}" || true
      echo
    } >"${OUTPUT_DIR}/post_goal_response.json" 2>"${OUTPUT_DIR}/post_goal_response.err"
  ) &
  PIDS+=("$!")
fi

sleep "${DURATION_SEC}"
cleanup
trap - EXIT INT TERM

api_snapshot "api_after"
run_logged "topic_info_after" bash -lc \
  'for topic in /cmd_vel_docking /cmd_vel_safe /cmd_vel /docking/status /safety/status /local_state/odometry /ranger_mini3/docking_allow_reverse /ranger_mini3/forced_mode /ranger_mini3/park /ranger_mini3_mode_controller/status /battery_state; do echo "## ${topic}"; timeout 6 ros2 topic info -v "${topic}"; done'
run_logged "processes_after" bash -lc \
  'pgrep -af "docking_manager_node|robot_safety_node|ranger_mini3_mode_controller|ranger_base_node|robot_api_server" || true'

write_summary

echo "${PREFIX} done"
echo "${PREFIX} summary=${OUTPUT_DIR}/summary.md"

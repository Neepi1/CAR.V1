#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=60
POST_GOAL_JSON=""
POST_GOAL_FILE=""
POST_GOAL_DELAY_SEC=2
OUTPUT_DIR=""
RECORD_BAG=false
PREFIX="[nav-goal-diag]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/record_navigation_goal_diagnostic.sh
  bash scripts/jetson/runtime_overlay/scripts/record_navigation_goal_diagnostic.sh --duration-sec 90
  bash scripts/jetson/runtime_overlay/scripts/record_navigation_goal_diagnostic.sh --post-goal-json '{"pose_id":"delivery_512355","building_id":"B10","floor_id":"F1"}'
  bash scripts/jetson/runtime_overlay/scripts/record_navigation_goal_diagnostic.sh --post-goal-file /tmp/nav_goal.json

Records one navigation attempt from a single terminal. Start the script, then
send a goal from the App while the capture is active. The script writes API
state, Nav2 lifecycle/params, TF snapshots, topic info, command-chain echoes,
and hz measurements into a timestamped report directory.

Options:
  --duration-sec N       Capture duration in seconds. Default: 60.
  --api-url URL          robot_api_server base URL. Default: http://127.0.0.1:8080.
  --output-dir DIR       Report directory. Default: reports/navigation_goal_diagnostics/<utc timestamp>.
  --post-goal-json JSON  POST this JSON to /api/v1/navigation/goal after recorders start.
  --post-goal-file FILE  POST JSON from FILE after recorders start; preferred for SSH/Windows shells.
  --post-delay-sec N     Delay before POST when --post-goal-json is used. Default: 2.
  --bag                  Also record a small rosbag with low-bandwidth topics.
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
    --post-goal-file)
      POST_GOAL_FILE="${2:-}"
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 5 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 5" >&2
  exit 2
fi

if ! [[ "${POST_GOAL_DELAY_SEC}" =~ ^[0-9]+$ ]]; then
  echo "${PREFIX} FAIL --post-delay-sec must be an integer" >&2
  exit 2
fi

if [[ -n "${POST_GOAL_FILE}" && ! -r "${POST_GOAL_FILE}" ]]; then
  echo "${PREFIX} FAIL --post-goal-file is not readable: ${POST_GOAL_FILE}" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/navigation_goal_diagnostics/${TIMESTAMP}"
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
    echo "GET ${API_URL}/api/v1/robot/pose"
    curl -fsS "${API_URL}/api/v1/robot/pose" || true
    echo
  } >"${OUTPUT_DIR}/${name}.json" 2>"${OUTPUT_DIR}/${name}.err"
}

api_poll_loop() {
  local deadline=$((SECONDS + DURATION_SEC))
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    printf '{"captured_at":"%s","navigation_state":' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    curl -fsS "${API_URL}/api/v1/navigation/state" 2>/dev/null || printf 'null'
    printf '}\n'
    sleep 1
  done
}

write_summary() {
  python3 - "${OUTPUT_DIR}" <<'PY' >"${OUTPUT_DIR}/summary.md" 2>/dev/null || true
import json
import pathlib
import ast
import sys

out = pathlib.Path(sys.argv[1])
print("# Navigation Goal Diagnostic Summary")
print()
print(f"- report_dir: `{out}`")

def load_embedded_json(path, marker):
    text = path.read_text(errors="ignore")
    idx = text.find(marker)
    if idx < 0:
        return None
    rest = text[idx + len(marker):].strip()
    start = rest.find("{")
    if start < 0:
        return None
    decoder = json.JSONDecoder()
    try:
        payload, _ = decoder.raw_decode(rest[start:])
        return payload
    except Exception:
        return None

after = load_embedded_json(out / "api_after.json", "GET ")
nav_state = None
if after:
    # api_after contains three JSON objects; the second is navigation state.
    text = (out / "api_after.json").read_text(errors="ignore")
    chunks = []
    decoder = json.JSONDecoder()
    pos = 0
    while True:
        start = text.find("{", pos)
        if start < 0:
            break
        try:
            payload, end = decoder.raw_decode(text[start:])
        except Exception:
            pos = start + 1
            continue
        chunks.append(payload)
        pos = start + end
    if len(chunks) >= 2:
        nav_state = chunks[1]

goal = (nav_state or {}).get("navigation_goal", {})
if goal:
    print()
    print("## Latest API Goal")
    for key in (
        "id",
        "state",
        "phase",
        "pose_id",
        "detail",
        "nav2_result_code",
        "nav2_succeeded",
        "position_reached",
        "final_pose_verified",
        "task_complete",
        "final_distance_m",
        "final_yaw_error_rad",
        "final_verify_xy_error_m",
        "final_verify_yaw_error_rad",
        "post_nav2_final_verify_enabled",
        "final_verify_retry_count",
        "final_verify_retry_reason",
        "final_verify_retry_goal_sent",
        "final_yaw_align_requested",
        "final_yaw_align_attempted",
        "final_yaw_align_succeeded",
        "final_yaw_align_blocked",
    ):
        if key in goal:
            print(f"- {key}: `{goal[key]}`")

def load_bridge_status_samples():
    path = out / "echo_bridge_status.log"
    if not path.exists():
        return []
    samples = []
    for line in path.read_text(errors="ignore").splitlines():
        stripped = line.strip()
        if not stripped or stripped in ("---",):
            continue
        if stripped.startswith("$ "):
            continue
        if stripped.startswith("data:"):
            raw = stripped.split(":", 1)[1].strip()
        else:
            raw = stripped
        if not raw:
            continue
        try:
            if raw[0] in ("'", '"'):
                raw = ast.literal_eval(raw)
            payload = json.loads(raw)
        except Exception:
            continue
        if isinstance(payload, dict):
            samples.append(payload)
    return samples

bridge_samples = load_bridge_status_samples()
print()
print("## AMCL Bridge Corrections")
if bridge_samples:
    first = bridge_samples[0]
    last = bridge_samples[-1]

    def number(data, key, default=0.0):
        value = data.get(key, default)
        try:
            return float(value)
        except Exception:
            return default

    def delta(key):
        return number(last, key) - number(first, key)

    print(f"- bridge_status_samples: `{len(bridge_samples)}`")
    print(f"- amcl_accepted_delta: `{delta('amcl_accepted_count'):.0f}`")
    print(f"- amcl_rejected_delta: `{delta('amcl_rejected_count'):.0f}`")
    print(f"- accepted_result_delta: `{delta('accepted_result_count'):.0f}`")
    print(f"- rejected_result_delta: `{delta('rejected_result_count'):.0f}`")
    for key in (
        "last_accepted_source",
        "last_rejected_source",
        "last_accept_reason",
        "last_reject_reason",
        "last_accepted_correction_translation_m",
        "last_accepted_correction_yaw_rad",
        "remaining_translation_error_m",
        "remaining_yaw_error_rad",
        "safe_for_goal_start",
        "correction_active",
    ):
        if key in last:
            print(f"- {key}: `{last[key]}`")
else:
    print("- bridge_status_samples: `0`")

print()
print("## HZ Logs")
for path in sorted(out.glob("hz_*.log")):
    lines = [line.strip() for line in path.read_text(errors="ignore").splitlines() if "average rate:" in line]
    if lines:
        print(f"- {path.name}: `{lines[-1]}`")
    else:
        print(f"- {path.name}: no average rate observed")

print()
print("## Command Chain Logs")
for path in sorted(out.glob("echo_cmd_vel*.log")):
    text = path.read_text(errors="ignore")
    non_zero_hint = any(token in text for token in ("x: 0.", "z: 0.", "x: -0.", "z: -0."))
    status = "messages captured" if "---" in text else "no messages captured"
    print(f"- {path.name}: {status}")

reverse_path = out / "echo_ranger_allow_reverse.log"
if reverse_path.exists():
    reverse_text = reverse_path.read_text(errors="ignore")
    print(f"- echo_ranger_allow_reverse.log: {'messages captured' if '---' in reverse_text else 'no messages captured'}")
    print(f"- ranger_allow_reverse_true_observed: `{'data: true' in reverse_text}`")
    print(f"- ranger_allow_reverse_false_observed: `{'data: false' in reverse_text}`")

print()
print("Use the raw logs in this directory to inspect exact command values, TF snapshots, and topic subscribers.")
PY
}

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC}"
echo "${PREFIX} api_url=${API_URL}"

api_snapshot "api_before"
start_logged "api_navigation_state_poll.jsonl" api_poll_loop

start_timeout_logged "hz_local_state_odometry" ros2 topic hz /local_state/odometry --window 20
start_timeout_logged "hz_wheel_odom" ros2 topic hz /wheel/odom --window 20
start_timeout_logged "hz_scan" ros2 topic hz /scan --window 20
start_timeout_logged "hz_local_costmap" ros2 topic hz /local_costmap/costmap --window 20
start_timeout_logged "hz_bridge_status" ros2 topic hz /localization/bridge_status --window 20
start_timeout_logged "hz_cmd_vel_nav_raw" ros2 topic hz /cmd_vel_nav_raw --window 20
start_timeout_logged "hz_cmd_vel_nav" ros2 topic hz /cmd_vel_nav --window 20
start_timeout_logged "hz_cmd_vel_collision_checked" ros2 topic hz /cmd_vel_collision_checked --window 20
start_timeout_logged "hz_cmd_vel_api" ros2 topic hz /cmd_vel_api --window 20
start_timeout_logged "hz_cmd_vel" ros2 topic hz /cmd_vel --window 20

start_timeout_logged "echo_cmd_vel_nav_raw" ros2 topic echo /cmd_vel_nav_raw
start_timeout_logged "echo_cmd_vel_nav" ros2 topic echo /cmd_vel_nav
start_timeout_logged "echo_cmd_vel_collision_checked" ros2 topic echo /cmd_vel_collision_checked
start_timeout_logged "echo_cmd_vel_api" ros2 topic echo /cmd_vel_api
start_timeout_logged "echo_cmd_vel" ros2 topic echo /cmd_vel
start_timeout_logged "echo_local_state_odometry" ros2 topic echo /local_state/odometry
start_timeout_logged "echo_wheel_odom" ros2 topic echo /wheel/odom
start_timeout_logged "echo_safety_status" ros2 topic echo /safety/status
start_timeout_logged "echo_nav_action_status" ros2 topic echo /navigate_to_pose/_action/status
start_timeout_logged "echo_nav_action_feedback" ros2 topic echo /navigate_to_pose/_action/feedback
start_timeout_logged "echo_compute_path_action_status" ros2 topic echo /compute_path_to_pose/_action/status
start_timeout_logged "echo_compute_path_action_feedback" ros2 topic echo /compute_path_to_pose/_action/feedback
start_timeout_logged "echo_follow_path_action_status" ros2 topic echo /follow_path/_action/status
start_timeout_logged "echo_follow_path_action_feedback" ros2 topic echo /follow_path/_action/feedback
start_timeout_logged "echo_bridge_status" ros2 topic echo --field data /localization/bridge_status
start_timeout_logged "echo_ranger_allow_reverse" ros2 topic echo /ranger_mini3/allow_reverse
start_timeout_logged "tf_odom_base_link" ros2 run tf2_ros tf2_echo odom base_link
start_timeout_logged "tf_map_base_link" ros2 run tf2_ros tf2_echo map base_link

start_logged "ros_nodes_before" timeout 8 ros2 node list
start_logged "ros_topics_before" timeout 8 ros2 topic list -t
start_logged "nav2_lifecycle_before" bash -lc \
  'for node in /controller_server /bt_navigator /local_costmap/local_costmap /planner_server; do echo "## ${node}"; timeout 6 ros2 lifecycle get "${node}"; done'
start_logged "nav2_params_before" bash -lc \
  'timeout 6 ros2 param get /controller_server progress_checker.required_movement_radius; timeout 6 ros2 param get /controller_server progress_checker.movement_time_allowance; timeout 6 ros2 param get /local_costmap/local_costmap global_frame; timeout 6 ros2 param get /local_costmap/local_costmap robot_base_frame; timeout 6 ros2 param get /local_costmap/local_costmap publish_frequency; timeout 6 ros2 param get /local_costmap/local_costmap update_frequency'
start_logged "scan_topic_info_before" timeout 8 ros2 topic info -v /scan
start_logged "cmd_topic_info_before" bash -lc \
  'for topic in /cmd_vel_nav_raw /cmd_vel_nav /cmd_vel_collision_checked /cmd_vel_api /cmd_vel; do echo "## ${topic}"; timeout 6 ros2 topic info -v "${topic}"; done'

echo "${PREFIX} CAPTURE ACTIVE: send the App goal now if --post-goal-json is not used"

if [[ "${RECORD_BAG}" == "true" ]]; then
  BAG_DIR="${OUTPUT_DIR}/rosbag_cmd_chain"
  start_timeout_logged "rosbag_record" ros2 bag record -o "${BAG_DIR}" \
    /cmd_vel_nav_raw \
    /cmd_vel_nav \
    /cmd_vel_collision_checked \
    /cmd_vel_api \
    /cmd_vel \
    /wheel/odom \
    /local_state/odometry \
    /safety/status \
    /localization/bridge_status \
    /ranger_mini3/allow_reverse \
    /navigate_to_pose/_action/status \
    /navigate_to_pose/_action/feedback \
    /compute_path_to_pose/_action/status \
    /compute_path_to_pose/_action/feedback \
    /follow_path/_action/status \
    /follow_path/_action/feedback
fi

if [[ -n "${POST_GOAL_JSON}" || -n "${POST_GOAL_FILE}" ]]; then
  (
    sleep "${POST_GOAL_DELAY_SEC}"
    {
      echo "POST ${API_URL}/api/v1/navigation/goal"
      if [[ -n "${POST_GOAL_FILE}" ]]; then
        echo "POST_GOAL_FILE ${POST_GOAL_FILE}"
        curl -sS -w '\nHTTP_STATUS:%{http_code}\n' -X POST "${API_URL}/api/v1/navigation/goal" \
          -H "Content-Type: application/json" \
          --data-binary @"${POST_GOAL_FILE}" || true
      else
        curl -sS -w '\nHTTP_STATUS:%{http_code}\n' -X POST "${API_URL}/api/v1/navigation/goal" \
          -H "Content-Type: application/json" \
          --data "${POST_GOAL_JSON}" || true
      fi
      echo
    } >"${OUTPUT_DIR}/post_goal_response.json" 2>"${OUTPUT_DIR}/post_goal_response.err"
  ) &
  PIDS+=("$!")
fi

sleep "${DURATION_SEC}"
cleanup
trap - EXIT INT TERM

api_snapshot "api_after"
run_logged "ros_topics_after" timeout 8 ros2 topic list -t
run_logged "nav2_lifecycle_after" bash -lc \
  'for node in /controller_server /bt_navigator /local_costmap/local_costmap /planner_server; do echo "## ${node}"; timeout 6 ros2 lifecycle get "${node}"; done'
run_logged "scan_topic_info_after" timeout 8 ros2 topic info -v /scan
run_logged "cmd_topic_info_after" bash -lc \
  'for topic in /cmd_vel_nav_raw /cmd_vel_nav /cmd_vel_collision_checked /cmd_vel_api /cmd_vel; do echo "## ${topic}"; timeout 6 ros2 topic info -v "${topic}"; done'

write_summary

echo "${PREFIX} done"
echo "${PREFIX} summary=${OUTPUT_DIR}/summary.md"

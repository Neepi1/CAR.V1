#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=20
EXECUTE_GOAL=false
BAG=false
CAPTURE_TF=false
CLI_KILL_AFTER_SEC="${NJRH_DIAG_CLI_KILL_AFTER_SEC:-5}"
GOAL_JSON=""
BUILDING_ID=""
FLOOR_ID=""
MAP_ID=""
POSE_ID=""
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
REPORT_DIR="${NJRH_PROJECT_ROOT}/reports/nav2_zero_linear_progress_${TIMESTAMP}_logs"
REPORT_FILE="${NJRH_PROJECT_ROOT}/reports/nav2_zero_linear_progress_${TIMESTAMP}.md"
PREFIX="[nav2-zero-linear]"
PIDS=()

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/diagnose_nav2_zero_linear_progress_failure.sh --duration-sec 20
  bash scripts/jetson/runtime_overlay/scripts/diagnose_nav2_zero_linear_progress_failure.sh --duration-sec 25 --execute-goal --building-id B10 --floor-id F1 --pose-id delivery_512355
  bash scripts/jetson/runtime_overlay/scripts/diagnose_nav2_zero_linear_progress_failure.sh --duration-sec 25 --execute-goal --goal-json '{"pose_id":"delivery_512355","building_id":"B10","floor_id":"F1"}'

Default mode only records existing topics while the operator sends a short goal
from the App. It does not publish a goal unless --execute-goal is provided.

Options:
  --duration-sec N       Capture duration. Default: 20.
  --bag true|false       Record a small rosbag. Default: false.
  --capture-tf true|false
                         Capture /tf, /tf_static, and tf2_echo streams.
                         Default: false because these create high-churn DDS
                         inspection participants on production systems.
  --execute-goal         POST /api/v1/navigation/goal after recorders start.
  --goal-json JSON       Explicit navigation goal JSON for --execute-goal.
  --building-id ID       Goal building_id when building JSON from fields.
  --floor-id ID          Goal floor_id when building JSON from fields.
  --map-id ID            Optional map_id when building JSON from fields.
  --pose-id ID           Goal pose_id when building JSON from fields.
  --api-url URL          Default: http://127.0.0.1:8080.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --bag)
      BAG="${2:-false}"
      shift 2
      ;;
    --capture-tf)
      CAPTURE_TF="${2:-false}"
      shift 2
      ;;
    --execute-goal)
      EXECUTE_GOAL=true
      shift
      ;;
    --goal-json)
      GOAL_JSON="${2:-}"
      shift 2
      ;;
    --building-id)
      BUILDING_ID="${2:-}"
      shift 2
      ;;
    --floor-id)
      FLOOR_ID="${2:-}"
      shift 2
      ;;
    --map-id)
      MAP_ID="${2:-}"
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

mkdir -p "${REPORT_DIR}" "$(dirname "${REPORT_FILE}")"

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
  } >"${REPORT_DIR}/dds_env.log" 2>&1
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
      echo "${PREFIX} WARN diagnostic process group ${pid} did not exit after TERM; sending targeted KILL" >>"${REPORT_DIR}/cleanup.log"
      kill -KILL "-${pid}" >/dev/null 2>&1 || kill -KILL "${pid}" >/dev/null 2>&1 || true
    }
  done
}
trap cleanup EXIT INT TERM

start_timeout_logged() {
  local name="$1"
  shift
  local log_file="${REPORT_DIR}/${name}.log"
  setsid bash -c '
    log_file="$1"
    duration="$2"
    kill_after="$3"
    shift 3
    {
      echo "$ timeout --kill-after=${kill_after}s --signal=TERM ${duration} $*"
      timeout --kill-after="${kill_after}s" --signal=TERM "${duration}" "$@"
    } >"${log_file}" 2>&1
  ' _ "${log_file}" "${DURATION_SEC}" "${CLI_KILL_AFTER_SEC}" "$@" &
  PIDS+=("$!")
}

run_logged() {
  local name="$1"
  shift
  {
    echo "\$ $*"
    "$@"
  } >"${REPORT_DIR}/${name}.log" 2>&1
}

build_goal_json() {
  if [[ -n "${GOAL_JSON}" ]]; then
    printf '%s\n' "${GOAL_JSON}"
    return 0
  fi
  python3 - "${BUILDING_ID}" "${FLOOR_ID}" "${MAP_ID}" "${POSE_ID}" <<'PY'
import json
import sys

building_id, floor_id, map_id, pose_id = sys.argv[1:5]
body = {}
if pose_id:
    body["pose_id"] = pose_id
if building_id:
    body["building_id"] = building_id
if floor_id:
    body["floor_id"] = floor_id
if map_id:
    body["map_id"] = map_id
print(json.dumps(body, separators=(",", ":")))
PY
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

echo "${PREFIX} report=${REPORT_FILE}"
echo "${PREFIX} logs=${REPORT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC}"
echo "${PREFIX} capture_tf=${CAPTURE_TF}"
echo "${PREFIX} CAPTURE ACTIVE: send a short App goal now if --execute-goal is not used"

write_dds_env_log
run_logged "api_before" bash -lc "curl -fsS '${API_URL}/api/v1/navigation/state'; echo; curl -fsS '${API_URL}/api/v1/robot/pose'; echo"
start_timeout_logged "api_navigation_state_poll" bash -lc "$(declare -f api_poll_loop); API_URL='${API_URL}'; DURATION_SEC='${DURATION_SEC}'; api_poll_loop"

start_timeout_logged "echo_cmd_vel_nav_raw" ros2 topic echo /cmd_vel_nav_raw
start_timeout_logged "echo_cmd_vel_nav" ros2 topic echo /cmd_vel_nav
start_timeout_logged "echo_cmd_vel_collision_checked" ros2 topic echo /cmd_vel_collision_checked
start_timeout_logged "echo_cmd_vel_safe" ros2 topic echo /cmd_vel_safe
start_timeout_logged "echo_cmd_vel" ros2 topic echo /cmd_vel
start_timeout_logged "echo_wheel_odom" ros2 topic echo /wheel/odom
start_timeout_logged "echo_local_state_odometry" ros2 topic echo /local_state/odometry
start_timeout_logged "echo_safety_status" ros2 topic echo /safety/status
if [[ "${CAPTURE_TF}" == "true" ]]; then
  start_timeout_logged "echo_tf" ros2 topic echo /tf
  start_timeout_logged "echo_tf_static" ros2 topic echo /tf_static
  start_timeout_logged "tf_odom_base_link" ros2 run tf2_ros tf2_echo odom base_link
  start_timeout_logged "tf_map_base_link" ros2 run tf2_ros tf2_echo map base_link
else
  printf 'skipped; rerun with --capture-tf true when this high-churn capture is explicitly needed\n' >"${REPORT_DIR}/echo_tf.log"
  printf 'skipped; rerun with --capture-tf true when this high-churn capture is explicitly needed\n' >"${REPORT_DIR}/echo_tf_static.log"
  printf 'skipped; rerun with --capture-tf true when this high-churn capture is explicitly needed\n' >"${REPORT_DIR}/tf_odom_base_link.log"
  printf 'skipped; rerun with --capture-tf true when this high-churn capture is explicitly needed\n' >"${REPORT_DIR}/tf_map_base_link.log"
fi

start_timeout_logged "hz_scan" ros2 topic hz /scan --window 20
start_timeout_logged "hz_local_costmap" ros2 topic hz /local_costmap/costmap --window 20

if [[ "${BAG}" == "true" ]]; then
  start_timeout_logged "rosbag_record" ros2 bag record -o "${REPORT_DIR}/rosbag" \
    /cmd_vel_nav_raw /cmd_vel_nav /cmd_vel_collision_checked /cmd_vel_safe /cmd_vel \
    /wheel/odom /local_state/odometry /safety/status /navigate_to_pose/_action/status
fi

if [[ "${EXECUTE_GOAL}" == "true" ]]; then
  BODY="$(build_goal_json)"
  if [[ "${BODY}" == "{}" ]]; then
    echo "${PREFIX} FAIL --execute-goal requires --goal-json or building/floor/pose fields" >&2
  else
    {
      echo "POST ${API_URL}/api/v1/navigation/goal"
      echo "${BODY}"
      curl -fsS -X POST "${API_URL}/api/v1/navigation/goal" -H "Content-Type: application/json" --data "${BODY}" || true
      echo
    } >"${REPORT_DIR}/post_goal_response.log" 2>&1 &
    PIDS+=("$!")
  fi
fi

sleep "${DURATION_SEC}"
cleanup
trap - EXIT INT TERM

run_logged "api_after" bash -lc "curl -fsS '${API_URL}/api/v1/navigation/state'; echo; curl -fsS '${API_URL}/api/v1/robot/pose'; echo"
run_logged "topic_info_cmd_chain" bash -lc \
  'for topic in /cmd_vel_nav_raw /cmd_vel_nav /cmd_vel_collision_checked /cmd_vel_safe /cmd_vel; do echo "## ${topic}"; timeout 6 ros2 topic info -v "${topic}"; done'
run_logged "topic_info_scan" timeout 8 ros2 topic info -v /scan

python3 - "${REPORT_DIR}" "${REPORT_FILE}" "${DURATION_SEC}" <<'PY'
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

log_dir = Path(sys.argv[1])
report = Path(sys.argv[2])
duration = float(sys.argv[3])
EPS = 1e-3
MOVE_EPS = 0.03
YAW_EPS = 0.05

def read(name):
    path = log_dir / name
    return path.read_text(errors="ignore") if path.exists() else ""

def parse_twist(name):
    text = read(name)
    values = []
    for block in text.split("---"):
        lin = re.search(r"linear:\s*\n\s*x:\s*([-+eE0-9.]+)", block)
        ang = re.search(
            r"angular:\s*\n\s*x:\s*[-+eE0-9.]+\s*\n\s*y:\s*[-+eE0-9.]+\s*\n\s*z:\s*([-+eE0-9.]+)",
            block,
        )
        if lin or ang:
            values.append((float(lin.group(1)) if lin else 0.0, float(ang.group(1)) if ang else 0.0))
    return values

def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

def unwrap(values):
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        prev = out[-1]
        while value - prev > math.pi:
            value -= 2 * math.pi
        while value - prev < -math.pi:
            value += 2 * math.pi
        out.append(value)
    return out

def parse_odom(name):
    text = read(name)
    values = []
    for block in text.split("---"):
        pos = re.search(r"position:\s*\n\s*x:\s*([-+eE0-9.]+)\s*\n\s*y:\s*([-+eE0-9.]+)", block)
        ori = re.search(
            r"orientation:\s*\n\s*x:\s*[-+eE0-9.]+\s*\n\s*y:\s*[-+eE0-9.]+\s*\n\s*z:\s*([-+eE0-9.]+)\s*\n\s*w:\s*([-+eE0-9.]+)",
            block,
        )
        if pos and ori:
            values.append((float(pos.group(1)), float(pos.group(2)), yaw_from_quat(float(ori.group(1)), float(ori.group(2)))))
    return values

def parse_tf(name):
    text = read(name)
    values = []
    for block in text.split("At time"):
        trans = re.search(r"Translation:\s*\[\s*([-+eE0-9.]+),\s*([-+eE0-9.]+),\s*([-+eE0-9.]+)\]", block)
        rpy = re.search(r"Rotation: in RPY \(radian\) \[\s*[-+eE0-9.]+,\s*[-+eE0-9.]+,\s*([-+eE0-9.]+)\]", block)
        if trans and rpy:
            values.append((float(trans.group(1)), float(trans.group(2)), float(rpy.group(1))))
    return values

def twist_metrics(values):
    if not values:
        return {
            "samples": 0,
            "max_linear_x": 0.0,
            "mean_abs_linear_x": 0.0,
            "nonzero_linear_ratio": 0.0,
            "max_angular_z": 0.0,
            "mean_abs_angular_z": 0.0,
            "nonzero_angular_ratio": 0.0,
            "integrated_yaw_cmd": 0.0,
        }
    dt = duration / max(len(values), 1)
    return {
        "samples": len(values),
        "max_linear_x": max(abs(v[0]) for v in values),
        "mean_abs_linear_x": sum(abs(v[0]) for v in values) / len(values),
        "nonzero_linear_ratio": sum(1 for v in values if abs(v[0]) > EPS) / len(values),
        "max_angular_z": max(abs(v[1]) for v in values),
        "mean_abs_angular_z": sum(abs(v[1]) for v in values) / len(values),
        "nonzero_angular_ratio": sum(1 for v in values if abs(v[1]) > EPS) / len(values),
        "integrated_yaw_cmd": sum(v[1] * dt for v in values),
    }

def odom_metrics(values):
    if not values:
        return {"samples": 0, "net_xy": 0.0, "dx": 0.0, "dy": 0.0, "dyaw": 0.0}
    yaws = unwrap([v[2] for v in values])
    return {
        "samples": len(values),
        "net_xy": math.hypot(values[-1][0] - values[0][0], values[-1][1] - values[0][1]),
        "dx": values[-1][0] - values[0][0],
        "dy": values[-1][1] - values[0][1],
        "dyaw": yaws[-1] - yaws[0],
    }

cmd_names = {
    "cmd_vel_nav_raw": "echo_cmd_vel_nav_raw.log",
    "cmd_vel_nav": "echo_cmd_vel_nav.log",
    "cmd_vel_collision_checked": "echo_cmd_vel_collision_checked.log",
    "cmd_vel_safe": "echo_cmd_vel_safe.log",
    "cmd_vel": "echo_cmd_vel.log",
}
cmd = {name: twist_metrics(parse_twist(path)) for name, path in cmd_names.items()}
odom = {
    "wheel_odom": odom_metrics(parse_odom("echo_wheel_odom.log")),
    "local_state_odometry": odom_metrics(parse_odom("echo_local_state_odometry.log")),
    "tf_odom_base_link": odom_metrics(parse_tf("tf_odom_base_link.log")),
    "tf_map_base_link": odom_metrics(parse_tf("tf_map_base_link.log")),
}

cases = []
raw = cmd["cmd_vel_nav_raw"]
collision = cmd["cmd_vel_collision_checked"]
safe = cmd["cmd_vel_safe"]
final = cmd["cmd_vel"]
wheel = odom["wheel_odom"]
local = odom["local_state_odometry"]
max_yaw_delta = max(abs(wheel["dyaw"]), abs(local["dyaw"]), abs(odom["tf_odom_base_link"]["dyaw"]), abs(odom["tf_map_base_link"]["dyaw"]))

if raw["max_linear_x"] <= EPS and raw["max_angular_z"] > EPS:
    cases.append("CASE_A_CONTROLLER_ZERO_LINEAR")
if raw["max_linear_x"] > EPS and collision["max_linear_x"] <= EPS:
    cases.append("CASE_B_COLLISION_ZERO_LINEAR")
if collision["max_linear_x"] > EPS and safe["max_linear_x"] <= EPS:
    cases.append("CASE_C_SAFETY_ZERO_LINEAR")
if safe["max_linear_x"] > EPS and final["max_linear_x"] > EPS and max(wheel["net_xy"], local["net_xy"]) <= MOVE_EPS:
    cases.append("CASE_D_MODE_CONTROLLER_OR_CHASSIS_NOT_EXECUTING")
if final["max_linear_x"] > EPS and max(wheel["net_xy"], local["net_xy"]) <= MOVE_EPS:
    cases.append("CASE_E_ODOM_NOT_REFLECTING_MOTION_REQUIRES_PHYSICAL_CONFIRMATION")
if raw["max_linear_x"] <= EPS and raw["max_angular_z"] > EPS and max_yaw_delta > 0.10:
    cases.append("CASE_F_ROTATION_PROGRESS_ONLY")
if raw["max_angular_z"] > EPS and max_yaw_delta <= YAW_EPS:
    cases.append("CASE_G_ROTATION_STALL")
if not cases:
    cases.append("CASE_UNCLASSIFIED")

def last_average(name):
    rates = re.findall(r"average rate:\s*([0-9.]+)", read(name))
    return rates[-1] if rates else "not observed"

status_values = Counter(re.findall(r"data:\s*([A-Za-z0-9_-]+)", read("echo_safety_status.log")))

with report.open("w", encoding="utf-8") as f:
    f.write("# Nav2 Zero Linear Progress Diagnostic\n\n")
    f.write(f"- log_dir: `{log_dir}`\n")
    f.write(f"- duration_sec: `{duration:g}`\n")
    f.write(f"- cases: `{', '.join(cases)}`\n\n")
    f.write("## Command Metrics\n")
    for name, metrics in cmd.items():
        f.write(
            f"- {name}: samples={metrics['samples']} max_linear_x={metrics['max_linear_x']:.6f} "
            f"mean_abs_linear_x={metrics['mean_abs_linear_x']:.6f} "
            f"nonzero_linear_ratio={metrics['nonzero_linear_ratio']:.3f} "
            f"max_angular_z={metrics['max_angular_z']:.6f} "
            f"mean_abs_angular_z={metrics['mean_abs_angular_z']:.6f} "
            f"nonzero_angular_ratio={metrics['nonzero_angular_ratio']:.3f} "
            f"integrated_yaw_cmd={metrics['integrated_yaw_cmd']:.6f}\n"
        )
    f.write("\n## Odom And TF Movement\n")
    for name, metrics in odom.items():
        f.write(
            f"- {name}: samples={metrics['samples']} dx={metrics['dx']:.6f} dy={metrics['dy']:.6f} "
            f"net_xy={metrics['net_xy']:.6f} dyaw={metrics['dyaw']:.6f}\n"
        )
    f.write("\n## Topic Rates\n")
    f.write(f"- /scan hz: `{last_average('hz_scan.log')}`\n")
    f.write(f"- /local_costmap/costmap hz: `{last_average('hz_local_costmap.log')}`\n")
    f.write("\n## Safety Status Samples\n")
    f.write(f"- {dict(status_values)}\n")
    f.write("\n## Interpretation\n")
    if "CASE_A_CONTROLLER_ZERO_LINEAR" in cases:
        f.write("- Controller output has angular.z but near-zero linear.x. Focus on RotationShim/controller startup behavior before changing downstream safety or pointcloud links.\n")
    if "CASE_F_ROTATION_PROGRESS_ONLY" in cases:
        f.write("- Rotation produced measurable yaw. PoseProgressChecker should count this as progress and avoid a false XY-only progress failure.\n")
    if "CASE_G_ROTATION_STALL" in cases:
        f.write("- Angular command exists but yaw barely changes. Inspect chassis/mode-controller execution rather than progress checker alone.\n")
    if "CASE_B_COLLISION_ZERO_LINEAR" in cases:
        f.write("- Collision monitor appears to zero linear velocity.\n")
    if "CASE_C_SAFETY_ZERO_LINEAR" in cases:
        f.write("- robot_safety appears to zero linear velocity.\n")
    if "CASE_D_MODE_CONTROLLER_OR_CHASSIS_NOT_EXECUTING" in cases:
        f.write("- Commands reach the final chain but odom does not move. Inspect mode controller and chassis execution.\n")
PY

echo "${PREFIX} report=${REPORT_FILE}"

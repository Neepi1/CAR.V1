#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/map_server_helpers.sh"

ACTION="start"
MODE="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
PID_FILE="${NJRH_AMCL_PID_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_shadow_localization.pid}"
LOG_FILE="${NJRH_AMCL_LOG_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_shadow_localization.log}"
PARAMS_FILE="${NJRH_AMCL_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/amcl_shadow.yaml}"
SEED_SERVICE="${NJRH_AMCL_SEED_SERVICE:-/robot_localization_bridge/seed_amcl_initial_pose}"
AMCL_NODE_NAME="${NJRH_AMCL_NODE_NAME:-amcl}"
SCAN_RELAY_IMPL="${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}"
SCAN_RELAY_CPP_BIN="${NJRH_AMCL_SCAN_ADMISSION_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_localization_bridge/lib/robot_localization_bridge/amcl_scan_admission_node}"
SCAN_RELAY_SCRIPT="${NJRH_AMCL_SCAN_ADMISSION_SCRIPT:-${SCRIPT_DIR}/amcl_scan_admission_relay.py}"
SCAN_RELAY_PID_FILE="${NJRH_AMCL_SCAN_ADMISSION_PID_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_scan_admission.pid}"
SCAN_RELAY_LOG_FILE="${NJRH_AMCL_SCAN_ADMISSION_LOG_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_scan_admission.log}"
STATUS_FILE="${NJRH_AMCL_RUNTIME_STATUS_FILE:-/tmp/njrh_amcl_runtime_status.env}"
NOMOTION_PROBE="${NJRH_AMCL_NOMOTION_PROBE:-${SCRIPT_DIR}/amcl_nomotion_update_probe.py}"

AMCL_EXIT_READY=0
AMCL_EXIT_DEGRADED=10
AMCL_EXIT_FAILED=20
AMCL_EXIT_GATED_NOT_READY=21
AMCL_EXIT_SCAN_ADMISSION_FAILED=22
AMCL_EXIT_LIFECYCLE_FAILED=23
AMCL_EXIT_SEED_FAILED=24
AMCL_EXIT_POSE_MISSING=25

AMCL_PID_STALE_CLEARED=false
SCAN_ADMISSION_PID_STALE_CLEARED=false
AMCL_SEED_SUCCEEDED=false
AMCL_SEED_RESPONSE_OK=false
AMCL_NOMOTION_PROBE_USED=false
AMCL_NOMOTION_POSE_RECEIVED=false
AMCL_NOMOTION_POSE_COUNT=0
AMCL_NOMOTION_POSE_HEADER_AGE_MS=""

usage() {
  cat <<'USAGE'
Usage: run_amcl_shadow_localization.sh [--restart|--stop|--print|--start-resident|--complete-readiness] [--mode disabled|shadow|gated]

Starts the AMCL candidate localization node. AMCL never publishes TF in this
profile; robot_localization_bridge remains the only map->odom owner. When AMCL
mode is enabled, /scan_amcl is an AMCL production admission input, not a debug
topic: it preserves /scan stamps and ranges while dropping stale or non-TF-ready
scans before they reach AMCL's MessageFilter.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --restart)
      ACTION="restart"
      shift
      ;;
    --start-resident)
      ACTION="resident"
      shift
      ;;
    --complete-readiness)
      ACTION="complete"
      shift
      ;;
    --stop)
      ACTION="stop"
      shift
      ;;
    --print)
      ACTION="print"
      shift
      ;;
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[runtime-overlay] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  disabled|shadow|gated)
    ;;
  *)
    echo "[runtime-overlay] invalid NJRH_AMCL_LOCALIZATION_MODE=${MODE}; expected disabled, shadow, or gated" >&2
    exit 2
    ;;
esac

scan_admission_enabled() {
  [[ "${MODE}" != "disabled" && "${NJRH_AMCL_SCAN_ADMISSION_ENABLED:-true}" == "true" ]]
}

effective_scan_topic() {
  if scan_admission_enabled; then
    printf '%s\n' "${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}"
  else
    printf '%s\n' "${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}"
  fi
}

print_config() {
  echo "NJRH_AMCL_LOCALIZATION_MODE=${MODE}"
  echo "NJRH_AMCL_PARAMS_FILE=${PARAMS_FILE}"
  echo "NJRH_AMCL_POSE_TOPIC=${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}"
  echo "NJRH_AMCL_INITIAL_POSE_TOPIC=${NJRH_AMCL_INITIAL_POSE_TOPIC:-/initialpose}"
  echo "NJRH_AMCL_PID_FILE=${PID_FILE}"
  echo "NJRH_AMCL_RUNTIME_STATUS_FILE=${STATUS_FILE}"
  echo "NJRH_AMCL_RUNTIME_STATUS_TTL_SEC=${NJRH_AMCL_RUNTIME_STATUS_TTL_SEC:-5.0}"
  echo "NJRH_AMCL_LOG_FILE=${LOG_FILE}"
  echo "NJRH_AMCL_SEED_SERVICE=${SEED_SERVICE}"
  echo "NJRH_AMCL_NOMOTION_PROBE=${NOMOTION_PROBE}"
  echo "AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC=${AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-${NJRH_AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-5.0}}"
  echo "AMCL_NOMOTION_UPDATE_ACCEPT_RECEIVED_AFTER_CALL=${AMCL_NOMOTION_UPDATE_ACCEPT_RECEIVED_AFTER_CALL:-${NJRH_AMCL_NOMOTION_UPDATE_ACCEPT_RECEIVED_AFTER_CALL:-true}}"
  echo "AMCL_SEED_READINESS_DO_NOT_REQUIRE_FRESH_HEADER_WHEN_STATIC=${AMCL_SEED_READINESS_DO_NOT_REQUIRE_FRESH_HEADER_WHEN_STATIC:-${NJRH_AMCL_SEED_READINESS_DO_NOT_REQUIRE_FRESH_HEADER_WHEN_STATIC:-true}}"
  echo "AMCL_CORRECTION_MAX_POSE_AGE_SEC=${AMCL_CORRECTION_MAX_POSE_AGE_SEC:-${NJRH_AMCL_CORRECTION_MAX_POSE_AGE_SEC:-1.0}}"
  echo "NJRH_AMCL_TF_WARMUP_SEC=${NJRH_AMCL_TF_WARMUP_SEC:-3.0}"
  echo "NJRH_AMCL_SEED_RETRY_COUNT=${NJRH_AMCL_SEED_RETRY_COUNT:-5}"
  echo "NJRH_AMCL_SCAN_ADMISSION_ENABLED=${NJRH_AMCL_SCAN_ADMISSION_ENABLED:-true}"
  echo "NJRH_AMCL_SCAN_INPUT_TOPIC=${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}"
  echo "NJRH_AMCL_SCAN_OUTPUT_TOPIC=${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}"
  echo "NJRH_AMCL_EFFECTIVE_SCAN_TOPIC=$(effective_scan_topic)"
  echo "NJRH_AMCL_SCAN_RATE_HZ=${NJRH_AMCL_SCAN_RATE_HZ:-5.0}"
  echo "NJRH_AMCL_SCAN_PRESERVE_STAMP=${NJRH_AMCL_SCAN_PRESERVE_STAMP:-true}"
  echo "NJRH_AMCL_SCAN_MAX_AGE_MS=${NJRH_AMCL_SCAN_MAX_AGE_MS:-250}"
  echo "NJRH_AMCL_SCAN_TARGET_FRAME=${NJRH_AMCL_SCAN_TARGET_FRAME:-odom}"
  echo "NJRH_AMCL_SCAN_ADMISSION_IMPL=${SCAN_RELAY_IMPL}"
  echo "NJRH_AMCL_SCAN_ADMISSION_CPP_BIN=${SCAN_RELAY_CPP_BIN}"
  echo "NJRH_CPUSET_AMCL=${NJRH_CPUSET_AMCL:-${NJRH_CPUSET_LOCALIZATION:-6}}"
  echo "NJRH_CPUSET_AMCL_SCAN_ADMISSION=${NJRH_CPUSET_AMCL_SCAN_ADMISSION:-${NJRH_CPUSET_LOCALIZATION:-6}}"
}

pid_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

env_quote() {
  local value="${1:-}"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  value="${value//\$/\\\$}"
  value="${value//\`/\\\`}"
  printf '"%s"' "${value}"
}

write_status_line() {
  local key="$1"
  local value="${2:-}"
  printf '%s=%s\n' "${key}" "$(env_quote "${value}")"
}

topic_publisher_count() {
  local topic="$1"
  timeout 4 ros2 topic info "${topic}" 2>/dev/null \
    | awk -F: '/Publisher count/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}'
}

topic_subscriber_count() {
  local topic="$1"
  timeout 4 ros2 topic info "${topic}" 2>/dev/null \
    | awk -F: '/Subscription count/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}'
}

amcl_node_exists() {
  timeout 4 ros2 node list 2>/dev/null | grep -Fxq "/${AMCL_NODE_NAME}"
}

pid_cmdline_matches() {
  local pid="$1"
  local kind="$2"
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  local args
  args="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  case "${kind}" in
    amcl)
      [[ "${args}" == *"nav2_amcl/amcl"* && "${args}" == *"__node:=${AMCL_NODE_NAME}"* ]] ||
        [[ "${args}" == *"ros2 run nav2_amcl amcl"* && "${args}" == *"__node:=${AMCL_NODE_NAME}"* ]]
      ;;
    scan_admission)
      [[ "${args}" == *"amcl_scan_admission_node"* ]] ||
        [[ "${args}" == *"amcl_scan_admission_relay.py"* ]]
      ;;
    *)
      return 1
      ;;
  esac
}

validated_pid_from_file() {
  local file="$1"
  local kind="$2"
  local pid
  pid="$(pid_from_file "${file}" 2>/dev/null || true)"
  [[ -n "${pid}" ]] || return 1
  if pid_alive "${pid}" && pid_cmdline_matches "${pid}" "${kind}"; then
    printf '%s\n' "${pid}"
    return 0
  fi
  echo "[runtime-overlay] stale ${kind} pid file cleared: file=${file} pid=${pid}" >&2
  rm -f "${file}"
  case "${kind}" in
    amcl) AMCL_PID_STALE_CLEARED=true ;;
    scan_admission) SCAN_ADMISSION_PID_STALE_CLEARED=true ;;
  esac
  return 1
}

first_process_pid() {
  awk 'NF {print; exit}'
}

status_lifecycle_active() {
  local state
  state="$(amcl_lifecycle_state 2>/dev/null || true)"
  [[ "${state}" == active* ]]
}

amcl_pose_age_ms_once() {
  local pose_topic="${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}"
  timeout 5 python3 - "${pose_topic}" <<'PY' 2>/dev/null || true
import sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

topic = sys.argv[1]
rclpy.init()
node = rclpy.create_node("amcl_pose_age_once")
state = {"age": ""}

def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

def on_msg(msg):
    now_sec = node.get_clock().now().nanoseconds * 1.0e-9
    state["age"] = str(max(0.0, (now_sec - stamp_to_sec(msg.header.stamp)) * 1000.0))

node.create_subscription(PoseWithCovarianceStamped, topic, on_msg, 10)
deadline = node.get_clock().now().nanoseconds + int(2.0e9)
while rclpy.ok() and not state["age"] and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
print(state["age"])
PY
}

write_amcl_runtime_status() {
  local start_result="$1"
  local ready="$2"
  local degraded="$3"
  local reason="${4:-}"
  local state="AMCL_FAILED"
  case "${start_result}" in
    disabled) state="AMCL_DISABLED" ;;
    ready) state="AMCL_READY" ;;
    degraded) state="AMCL_DEGRADED" ;;
    waiting_seed) state="AMCL_WAITING_SEED" ;;
    failed) state="AMCL_FAILED" ;;
  esac

  mkdir -p "$(dirname "${STATUS_FILE}")"
  local amcl_pid=""
  local amcl_pid_alive=false
  local scan_pid=""
  local scan_alive=false
  local amcl_node=false
  local lifecycle_active=false
  local amcl_pose_publishers="0"
  local scan_status_publishers="0"
  local scan_amcl_publishers="0"
  local amcl_pose_age_ms=""
  local map_owner=""
  local status_stamp_sec=""
  local amcl_process_ready=false
  local amcl_seeded=false
  local amcl_tracking_ready=false
  local amcl_correction_ready=false
  local amcl_static_standby=false
  local amcl_not_moving_no_update_ok=false

  amcl_pid="$(validated_pid_from_file "${PID_FILE}" amcl 2>/dev/null || true)"
  if [[ -z "${amcl_pid}" ]]; then
    amcl_pid="$(amcl_process_pids | first_process_pid || true)"
  fi
  if [[ -n "${amcl_pid}" ]] && pid_alive "${amcl_pid}" && pid_cmdline_matches "${amcl_pid}" amcl; then
    amcl_pid_alive=true
  fi

  scan_pid="$(validated_pid_from_file "${SCAN_RELAY_PID_FILE}" scan_admission 2>/dev/null || true)"
  if [[ -z "${scan_pid}" ]]; then
    scan_pid="$(scan_relay_process_pids | first_process_pid || true)"
  fi
  if [[ -n "${scan_pid}" ]] && pid_alive "${scan_pid}" && pid_cmdline_matches "${scan_pid}" scan_admission; then
    scan_alive=true
  fi

  amcl_node_exists && amcl_node=true
  status_lifecycle_active && lifecycle_active=true
  amcl_pose_publishers="$(topic_publisher_count "${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}")"
  scan_status_publishers="$(topic_publisher_count "${NJRH_AMCL_SCAN_ADMISSION_STATUS_TOPIC:-/amcl_scan_admission/status}")"
  scan_amcl_publishers="$(topic_publisher_count "${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}")"
  amcl_pose_age_ms="$(amcl_pose_age_ms_once)"
  map_owner="$(timeout 4 ros2 topic echo --once --field data /localization/bridge_status 2>/dev/null \
    | python3 -c 'import json,sys; s=sys.stdin.read().strip(); print(json.loads(s).get("map_to_odom_publisher_owner","") if s else "")' 2>/dev/null || true)"
  status_stamp_sec="$(date +%s)"

  if [[ "${amcl_pid_alive}" == "true" && "${amcl_node}" == "true" && "${lifecycle_active}" == "true" ]]; then
    amcl_process_ready=true
  fi
  if [[ "${AMCL_SEED_SUCCEEDED}" == "true" || "${AMCL_SEED_RESPONSE_OK}" == "true" ]]; then
    amcl_seeded=true
  fi
  if [[ "${AMCL_NOMOTION_POSE_RECEIVED}" == "true" && "${amcl_process_ready}" == "true" && "${amcl_seeded}" == "true" ]]; then
    amcl_tracking_ready=true
    amcl_correction_ready=false
    amcl_static_standby=true
    amcl_not_moving_no_update_ok=true
  elif [[ "${ready}" == "true" && "${amcl_process_ready}" == "true" && "${scan_alive}" == "true" ]]; then
    amcl_tracking_ready=true
    amcl_correction_ready=true
  fi
  if [[ "${amcl_process_ready}" == "true" && "${amcl_seeded}" == "true" && "${ready}" != "true" ]]; then
    amcl_static_standby=true
    amcl_not_moving_no_update_ok=true
  fi

  {
    write_status_line AMCL_STATUS_STAMP_SEC "${status_stamp_sec}"
    write_status_line AMCL_STATUS_AGE_MS "0"
    write_status_line AMCL_STATUS_STALE "false"
    write_status_line AMCL_STATUS_TTL_SEC "${NJRH_AMCL_RUNTIME_STATUS_TTL_SEC:-5.0}"
    write_status_line AMCL_MODE "${MODE}"
    write_status_line AMCL_STATE "${state}"
    write_status_line AMCL_START_RESULT "${start_result}"
    write_status_line AMCL_READY "${ready}"
    write_status_line AMCL_DEGRADED "${degraded}"
    write_status_line AMCL_FAILURE_REASON "${reason}"
    write_status_line AMCL_NODE_EXISTS "${amcl_node}"
    write_status_line AMCL_LIFECYCLE_ACTIVE "${lifecycle_active}"
    write_status_line AMCL_PID "${amcl_pid}"
    write_status_line AMCL_PID_ALIVE "${amcl_pid_alive}"
    write_status_line AMCL_PROCESS_ALIVE "${amcl_pid_alive}"
    write_status_line AMCL_PROCESS_READY "${amcl_process_ready}"
    write_status_line AMCL_PID_STALE_CLEARED "${AMCL_PID_STALE_CLEARED}"
    write_status_line SCAN_ADMISSION_PID "${scan_pid}"
    write_status_line SCAN_ADMISSION_ALIVE "${scan_alive}"
    write_status_line SCAN_ADMISSION_PID_STALE_CLEARED "${SCAN_ADMISSION_PID_STALE_CLEARED}"
    write_status_line SCAN_ADMISSION_IMPL "${SCAN_RELAY_IMPL}"
    write_status_line SCAN_ADMISSION_STATUS_PUBLISHER_COUNT "${scan_status_publishers:-0}"
    write_status_line SCAN_AMCL_PUBLISHER_COUNT "${scan_amcl_publishers:-0}"
    write_status_line AMCL_POSE_PUBLISHER_COUNT "${amcl_pose_publishers:-0}"
    write_status_line AMCL_LAST_POSE_AGE_MS "${amcl_pose_age_ms:-}"
    write_status_line AMCL_POSE_LAST_RECEIVE_AGE_MS "${amcl_pose_age_ms:-}"
    write_status_line AMCL_SEED_SUCCEEDED "${AMCL_SEED_SUCCEEDED}"
    write_status_line AMCL_SEEDED "${amcl_seeded}"
    write_status_line AMCL_SEED_RESPONSE_OK "${AMCL_SEED_RESPONSE_OK}"
    write_status_line AMCL_NOMOTION_PROBE_USED "${AMCL_NOMOTION_PROBE_USED}"
    write_status_line AMCL_NOMOTION_POSE_RECEIVED "${AMCL_NOMOTION_POSE_RECEIVED}"
    write_status_line AMCL_NOMOTION_POSE_COUNT "${AMCL_NOMOTION_POSE_COUNT}"
    write_status_line AMCL_NOMOTION_POSE_HEADER_AGE_MS "${AMCL_NOMOTION_POSE_HEADER_AGE_MS}"
    write_status_line AMCL_TRACKING_READY "${amcl_tracking_ready}"
    write_status_line AMCL_CORRECTION_READY "${amcl_correction_ready}"
    write_status_line AMCL_STATIC_STANDBY "${amcl_static_standby}"
    write_status_line AMCL_NOT_MOVING_NO_UPDATE_OK "${amcl_not_moving_no_update_ok}"
    write_status_line AMCL_DEGRADED_REASON "${reason}"
    write_status_line MAP_TO_ODOM_OWNER "${map_owner}"
    write_status_line TIMESTAMP "$(date -Is)"
  } >"${STATUS_FILE}"
}

finish_amcl_status() {
  local start_result="$1"
  local ready="$2"
  local degraded="$3"
  local reason="${4:-}"
  local code="${5:-0}"
  write_amcl_runtime_status "${start_result}" "${ready}" "${degraded}" "${reason}"
  case "${start_result}" in
    ready|disabled)
      echo "[runtime-overlay] AMCL_READY mode=${MODE} result=${start_result} status_file=${STATUS_FILE}" >&2
      ;;
    waiting_seed)
      echo "[runtime-overlay] AMCL_WAITING_SEED mode=${MODE} reason=${reason} status_file=${STATUS_FILE}" >&2
      ;;
    degraded)
      echo "[runtime-overlay] AMCL_DEGRADED mode=${MODE} reason=${reason} status_file=${STATUS_FILE}" >&2
      ;;
    failed)
      echo "[runtime-overlay] AMCL_FAILED mode=${MODE} reason=${reason} status_file=${STATUS_FILE}" >&2
      ;;
  esac
  return "${code}"
}

wait_for_pid_exit() {
  local pid="$1"
  local attempts="${2:-30}"
  local i
  for ((i = 0; i < attempts; i += 1)); do
    if ! pid_alive "${pid}"; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

pid_from_file() {
  local file="$1"
  [[ -f "${file}" ]] || return 1
  local pid
  pid="$(tr -dc '0-9' <"${file}" || true)"
  [[ -n "${pid}" ]] || return 1
  printf '%s\n' "${pid}"
}

amcl_pid_from_file() {
  validated_pid_from_file "${PID_FILE}" amcl
}

scan_relay_pid_from_file() {
  validated_pid_from_file "${SCAN_RELAY_PID_FILE}" scan_admission
}

amcl_process_pids() {
  ps -eo pid=,args= |
    awk -v node_name="__node:=${AMCL_NODE_NAME}" '
      /nav2_amcl\/amcl/ && index($0, node_name) > 0 {print $1}
      /ros2 run nav2_amcl amcl/ && index($0, node_name) > 0 {print $1}
    ' || true
}

scan_relay_process_pids() {
  ps -eo pid=,args= |
    awk '
      /amcl_scan_admission_node/ && !/awk/ {print $1}
      /amcl_scan_admission_relay.py/ && !/awk/ {print $1}
    ' || true
}

scan_relay_cpp_process_pids() {
  ps -eo pid=,args= |
    awk '/amcl_scan_admission_node/ && !/awk/ {print $1}' || true
}

scan_relay_python_process_pids() {
  ps -eo pid=,args= |
    awk '/amcl_scan_admission_relay.py/ && !/awk/ {print $1}' || true
}

scan_relay_pid_matches_impl() {
  local pid="$1"
  local impl="$2"
  local args
  args="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  case "${impl}" in
    cpp) [[ "${args}" == *"amcl_scan_admission_node"* ]] ;;
    python) [[ "${args}" == *"amcl_scan_admission_relay.py"* ]] ;;
    *) return 1 ;;
  esac
}

scan_relay_allowed_cpus() {
  local pid="$1"
  awk '/^Cpus_allowed_list:/ {print $2; exit}' "/proc/${pid}/status" 2>/dev/null || true
}

stop_pid_softly() {
  local label="$1"
  local pid="$2"
  [[ -n "${pid}" ]] || return 0
  if pid_alive "${pid}"; then
    echo "[runtime-overlay] stopping ${label} pid=${pid}" >&2
    kill -INT "${pid}" 2>/dev/null || true
    wait_for_pid_exit "${pid}" 30 || {
      kill -TERM "${pid}" 2>/dev/null || true
      wait_for_pid_exit "${pid}" 30 || true
    }
  fi
}

stop_scan_admission_relay() {
  local pid=""
  pid="$(scan_relay_pid_from_file 2>/dev/null || true)"
  stop_pid_softly "AMCL scan admission relay" "${pid}"
  local extra_pids
  extra_pids="$(scan_relay_process_pids)"
  if [[ -n "${extra_pids}" ]]; then
    echo "[runtime-overlay] stopping remaining AMCL scan admission relay processes: ${extra_pids}" >&2
    kill -INT ${extra_pids} 2>/dev/null || true
    sleep 1
    extra_pids="$(scan_relay_process_pids)"
    [[ -z "${extra_pids}" ]] || kill -TERM ${extra_pids} 2>/dev/null || true
  fi
  rm -f "${SCAN_RELAY_PID_FILE}"
}

stop_amcl() {
  stop_scan_admission_relay
  timeout 5 ros2 lifecycle set "/${AMCL_NODE_NAME}" shutdown >/dev/null 2>&1 || true

  local pid=""
  pid="$(amcl_pid_from_file 2>/dev/null || true)"
  stop_pid_softly "AMCL" "${pid}"
  local extra_pids
  extra_pids="$(amcl_process_pids)"
  if [[ -n "${extra_pids}" ]]; then
    echo "[runtime-overlay] stopping remaining AMCL processes: ${extra_pids}" >&2
    kill -INT ${extra_pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_STOP_INT_WAIT_SEC:-1}"
    extra_pids="$(amcl_process_pids)"
    [[ -z "${extra_pids}" ]] || kill -TERM ${extra_pids} 2>/dev/null || true
  fi
  rm -f "${PID_FILE}"
}

wait_for_amcl_node() {
  local timeout_sec="${1:-15}"
  runtime_readiness_probe node "/${AMCL_NODE_NAME}" "${timeout_sec}" >/dev/null 2>&1
}

amcl_param_value() {
  local param="$1"
  timeout 5 ros2 param get "/${AMCL_NODE_NAME}" "${param}" 2>/dev/null || true
}

request_amcl_lifecycle_transition() {
  local transition_id="$1"
  local transition_label="$2"
  local timeout_sec="${3:-20}"
  local output
  output="$(timeout "${timeout_sec}" ros2 service call \
    "/${AMCL_NODE_NAME}/change_state" \
    lifecycle_msgs/srv/ChangeState \
    "{transition: {id: ${transition_id}, label: ${transition_label}}}" 2>&1 || true)"
  if grep -Eq 'success[=:][[:space:]]*(True|true)|success:[[:space:]]*true' <<<"${output}"; then
    return 0
  fi
  echo "[runtime-overlay] /${AMCL_NODE_NAME} lifecycle transition ${transition_label} failed: ${output}" >&2
  return 1
}

amcl_lifecycle_state() {
  local state
  state="$(timeout 5 ros2 lifecycle get "/${AMCL_NODE_NAME}" 2>/dev/null || true)"
  if [[ "${state}" == active* || "${state}" == inactive* || "${state}" == unconfigured* ]]; then
    printf '%s\n' "${state}"
    return 0
  fi

  local output
  output="$(timeout 10 ros2 service call \
    "/${AMCL_NODE_NAME}/get_state" \
    lifecycle_msgs/srv/GetState \
    "{}" 2>&1 || true)"
  if grep -Eq "label='active'|label:[[:space:]]*active" <<<"${output}"; then
    printf 'active [3]\n'
    return 0
  fi
  if grep -Eq "label='inactive'|label:[[:space:]]*inactive" <<<"${output}"; then
    printf 'inactive [2]\n'
    return 0
  fi
  if grep -Eq "label='unconfigured'|label:[[:space:]]*unconfigured" <<<"${output}"; then
    printf 'unconfigured [1]\n'
    return 0
  fi
  printf 'unknown\n'
}

activate_amcl_lifecycle() {
  wait_for_amcl_node "${NJRH_AMCL_NODE_WAIT_SEC:-15}" || {
    echo "[runtime-overlay] /${AMCL_NODE_NAME} did not appear after start" >&2
    return 1
  }

  local state
  state="$(amcl_lifecycle_state)"
  if [[ "${state}" != active* && "${state}" != inactive* ]]; then
    request_amcl_lifecycle_transition 1 configure "${NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC:-20}" || return 1
  fi
  state="$(amcl_lifecycle_state)"
  if [[ "${state}" != active* ]]; then
    request_amcl_lifecycle_transition 3 activate "${NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC:-20}" || return 1
  fi
  state="$(amcl_lifecycle_state)"
  if [[ "${state}" != active* ]]; then
    echo "[runtime-overlay] /${AMCL_NODE_NAME} lifecycle is not active yet: ${state:-unknown}" >&2
    return 1
  fi
  local tf_broadcast
  tf_broadcast="$(amcl_param_value tf_broadcast)"
  if [[ "${tf_broadcast}" != *"False"* && "${tf_broadcast}" != *"false"* ]]; then
    echo "[runtime-overlay] AMCL tf_broadcast is not false: ${tf_broadcast:-missing}" >&2
    return 1
  fi
}

amcl_warn() {
  local reason="$1"
  echo "[runtime-overlay] AMCL_WARN ${reason}" >&2
  return 0
}

scan_frame_from_topic() {
  local topic="${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}"
  local frame
  frame="$(timeout 6 ros2 topic echo "${topic}" --once --field header.frame_id 2>/dev/null | awk 'NF {print; exit}' || true)"
  if [[ -n "${frame}" ]]; then
    printf '%s\n' "${frame}"
  else
    printf '%s\n' "${NJRH_AMCL_SCAN_FRAME_REQUIRED:-lidar_level_link}"
  fi
}

wait_for_amcl_tf_warmup() {
  local require_map_odom="${1:-true}"
  local map_timeout="${NJRH_AMCL_MAP_WAIT_SEC:-15}"
  local scan_timeout="${NJRH_AMCL_SCAN_WAIT_SEC:-15}"
  local tf_timeout="${NJRH_AMCL_TF_WAIT_SEC:-15}"
  local scan_frame

  wait_for_occupancy_grid "/map" "${map_timeout}" || return 1
  wait_for_topic_message "${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}" "${scan_timeout}" || return 1
  scan_frame="$(scan_frame_from_topic)"
  if [[ "${require_map_odom}" == "true" ]]; then
    wait_for_tf_transform "map" "odom" "${tf_timeout}" || return 1
  fi
  wait_for_tf_transform "odom" "base_link" "${tf_timeout}" || return 1
  wait_for_tf_transform "base_link" "${scan_frame}" "${tf_timeout}" || return 1

  local warmup_sec="${NJRH_AMCL_TF_WARMUP_SEC:-3.0}"
  echo "[runtime-overlay] AMCL TF cache warmup ${warmup_sec}s after map/scan/TF gates; scan_frame=${scan_frame} require_map_odom=${require_map_odom}" >&2
  sleep "${warmup_sec}"
}

seed_amcl_initial_pose() {
  local retry_count="${NJRH_AMCL_SEED_RETRY_COUNT:-5}"
  local retry_period_ms="${NJRH_AMCL_SEED_RETRY_PERIOD_MS:-300}"
  local wait_sec="${NJRH_AMCL_SEED_SERVICE_WAIT_SEC:-8}"
  local attempt
  for ((attempt = 1; attempt <= retry_count; attempt += 1)); do
    if ! wait_for_ros_service "${SEED_SERVICE}" "${wait_sec}" >/dev/null 2>&1; then
      echo "[runtime-overlay] AMCL seed service unavailable attempt=${attempt}/${retry_count}: ${SEED_SERVICE}" >&2
    else
      local output
      output="$(timeout "${NJRH_AMCL_SEED_CALL_TIMEOUT_SEC:-8}" ros2 service call \
        "${SEED_SERVICE}" std_srvs/srv/Trigger "{}" 2>&1 || true)"
      echo "[runtime-overlay] AMCL initial pose seed attempt=${attempt}/${retry_count}: ${output}" >&2
      if grep -Eiq "success[:=][[:space:]]*true|success=True" <<<"${output}"; then
        AMCL_SEED_SUCCEEDED=true
        return 0
      fi
    fi
    sleep "$(awk -v ms="${retry_period_ms}" 'BEGIN {printf "%.3f", ms / 1000.0}')"
  done
  AMCL_SEED_SUCCEEDED=false
  return 1
}

start_scan_admission_relay() {
  scan_admission_enabled || return 0
  case "${SCAN_RELAY_IMPL}" in
    cpp|python)
      ;;
    *)
      echo "[runtime-overlay] invalid NJRH_AMCL_SCAN_ADMISSION_IMPL=${SCAN_RELAY_IMPL}; expected cpp or python" >&2
      return 1
      ;;
  esac
  if [[ "${SCAN_RELAY_IMPL}" == "cpp" && ! -x "${SCAN_RELAY_CPP_BIN}" ]]; then
    echo "[runtime-overlay] AMCL C++ scan admission binary missing: ${SCAN_RELAY_CPP_BIN}" >&2
    echo "[runtime-overlay] build robot_localization_bridge or explicitly set NJRH_AMCL_SCAN_ADMISSION_IMPL=python for temporary fallback" >&2
    return 1
  fi
  if [[ "${SCAN_RELAY_IMPL}" == "python" && ! -f "${SCAN_RELAY_SCRIPT}" ]]; then
    echo "[runtime-overlay] AMCL Python scan admission relay script missing: ${SCAN_RELAY_SCRIPT}" >&2
    return 1
  fi
  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  local relay_cpuset
  relay_cpuset="$(njrh_cpuset_for amcl_scan_admission 2>/dev/null || true)"
  relay_cpuset="${relay_cpuset:-${NJRH_CPUSET_AMCL_SCAN_ADMISSION:-${NJRH_CPUSET_LOCALIZATION:-6}}}"
  export NJRH_CPUSET_AMCL_SCAN_ADMISSION="${relay_cpuset}"
  if [[ -z "${relay_cpuset}" ]]; then
    echo "[runtime-overlay] AMCL scan admission relay has no CPU affinity cpuset" >&2
    return 1
  fi
  if ! command -v taskset >/dev/null 2>&1; then
    echo "[runtime-overlay] taskset is required for AMCL scan admission relay affinity" >&2
    return 1
  fi
  if ! taskset -c "${relay_cpuset}" true >/dev/null 2>&1; then
    echo "[runtime-overlay] invalid AMCL scan admission relay cpuset: ${relay_cpuset}" >&2
    return 1
  fi
  if pid="$(scan_relay_pid_from_file 2>/dev/null || true)" && [[ -n "${pid}" ]] && pid_alive "${pid}"; then
    if ! scan_relay_pid_matches_impl "${pid}" "${SCAN_RELAY_IMPL}"; then
      echo "[runtime-overlay] AMCL scan admission relay already running pid=${pid} but implementation does not match expected=${SCAN_RELAY_IMPL}; restart required" >&2
      return 1
    fi
    njrh_apply_affinity_to_pids amcl_scan_admission "${pid}" >/dev/null 2>&1 || true
    local allowed
    allowed="$(scan_relay_allowed_cpus "${pid}")"
    if [[ "${allowed}" != "${relay_cpuset}" ]]; then
      echo "[runtime-overlay] AMCL scan admission relay already running pid=${pid} but Cpus_allowed_list=${allowed:-missing}, expected=${relay_cpuset}" >&2
      return 1
    fi
    echo "[runtime-overlay] AMCL scan admission relay already running implementation=${SCAN_RELAY_IMPL} pid=${pid} cpuset=${allowed}" >&2
    return 0
  fi

  : >"${SCAN_RELAY_LOG_FILE}"
  local scan_rate_hz
  local scan_max_age_ms
  local scan_wait_for_tf_timeout_ms
  local scan_input_topic
  local scan_output_topic
  scan_rate_hz="$(awk -v v="${NJRH_AMCL_SCAN_RATE_HZ:-5.0}" 'BEGIN {printf "%.3f", v + 0.0}')"
  scan_max_age_ms="$(awk -v v="${NJRH_AMCL_SCAN_MAX_AGE_MS:-250.0}" 'BEGIN {printf "%.3f", v + 0.0}')"
  scan_wait_for_tf_timeout_ms="$(awk -v v="${NJRH_AMCL_SCAN_WAIT_FOR_TF_TIMEOUT_MS:-20.0}" 'BEGIN {printf "%.3f", v + 0.0}')"
  scan_input_topic="${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}"
  scan_output_topic="${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}"
  local relay_cmd=()
  if [[ "${SCAN_RELAY_IMPL}" == "cpp" ]]; then
    relay_cmd=(ros2 run robot_localization_bridge amcl_scan_admission_node --ros-args
      -p "input_topic:=${scan_input_topic}" \
      -p "output_topic:=${scan_output_topic}" \
      -p "status_topic:=${NJRH_AMCL_SCAN_ADMISSION_STATUS_TOPIC:-/amcl_scan_admission/status}" \
      -p "target_frame:=${NJRH_AMCL_SCAN_TARGET_FRAME:-odom}" \
      -p "frame_required:=${NJRH_AMCL_SCAN_FRAME_REQUIRED:-lidar_level_link}" \
      -p "max_rate_hz:=${scan_rate_hz}" \
      -p "max_scan_age_ms:=${scan_max_age_ms}" \
      -p "tf_wait_timeout_ms:=${scan_wait_for_tf_timeout_ms}" \
      -p "require_tf_available:=${NJRH_AMCL_SCAN_DROP_IF_TF_UNAVAILABLE:-true}" \
      -p "preserve_stamp:=${NJRH_AMCL_SCAN_PRESERVE_STAMP:-true}" \
      -p "require_seeded:=${NJRH_AMCL_SCAN_ADMISSION_REQUIRE_SEEDED:-false}" \
      -p "require_tf_warmup:=${NJRH_AMCL_SCAN_ADMISSION_REQUIRE_TF_WARMUP:-false}" \
      -p "startup_warmup_sec:=${NJRH_AMCL_SCAN_ADMISSION_STARTUP_WARMUP_SEC:-0.0}" \
      -p "status_log_period_sec:=${NJRH_AMCL_SCAN_ADMISSION_STATUS_LOG_PERIOD_SEC:-1.0}" \
      -p "drop_if_future_stamp:=${NJRH_AMCL_SCAN_DROP_IF_FUTURE_STAMP:-true}" \
      -p "max_future_stamp_ms:=${NJRH_AMCL_SCAN_MAX_FUTURE_STAMP_MS:-50.0}")
  else
    relay_cmd=(python3 "${SCAN_RELAY_SCRIPT}" --ros-args
      -p "input_topic:=${scan_input_topic}" \
      -p "output_topic:=${scan_output_topic}" \
      -p "status_topic:=${NJRH_AMCL_SCAN_ADMISSION_STATUS_TOPIC:-/amcl_scan_admission/status}" \
      -p "target_frame:=${NJRH_AMCL_SCAN_TARGET_FRAME:-odom}" \
      -p "frame_required:=${NJRH_AMCL_SCAN_FRAME_REQUIRED:-lidar_level_link}" \
      -p "rate_hz:=${scan_rate_hz}" \
      -p "max_age_ms:=${scan_max_age_ms}" \
      -p "wait_for_tf_timeout_ms:=${scan_wait_for_tf_timeout_ms}" \
      -p "drop_if_tf_unavailable:=${NJRH_AMCL_SCAN_DROP_IF_TF_UNAVAILABLE:-true}")
  fi
  if [[ -n "${NJRH_NICE_AMCL_SCAN_ADMISSION:-}" ]]; then
    relay_cmd=(nice -n "${NJRH_NICE_AMCL_SCAN_ADMISSION}" "${relay_cmd[@]}")
  fi
  echo "[runtime-overlay] starting AMCL scan admission relay implementation=${SCAN_RELAY_IMPL} input_topic=${scan_input_topic} output_topic=${scan_output_topic} rate_hz=${scan_rate_hz} max_scan_age_ms=${scan_max_age_ms} tf_wait_timeout_ms=${scan_wait_for_tf_timeout_ms} cpuset=${relay_cpuset}" >&2
  nohup taskset -c "${relay_cpuset}" "${relay_cmd[@]}" >>"${SCAN_RELAY_LOG_FILE}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" >"${SCAN_RELAY_PID_FILE}"
  sleep "${NJRH_AMCL_SCAN_ADMISSION_START_SETTLE_SEC:-0.5}"
  if ! pid_alive "${pid}"; then
    echo "[runtime-overlay] AMCL scan admission relay implementation=${SCAN_RELAY_IMPL} failed to stay alive; check ${SCAN_RELAY_LOG_FILE}" >&2
    rm -f "${SCAN_RELAY_PID_FILE}"
    return 1
  fi
  local allowed
  allowed="$(scan_relay_allowed_cpus "${pid}")"
  if [[ "${allowed}" != "${relay_cpuset}" ]]; then
    echo "[runtime-overlay] AMCL scan admission relay pid=${pid} Cpus_allowed_list=${allowed:-missing}, expected=${relay_cpuset}" >&2
    stop_pid_softly "AMCL scan admission relay" "${pid}"
    rm -f "${SCAN_RELAY_PID_FILE}"
    return 1
  fi
  echo "[runtime-overlay] AMCL scan admission relay started implementation=${SCAN_RELAY_IMPL} pid=${pid} cpuset=${allowed} input_topic=${scan_input_topic} output_topic=${scan_output_topic} rate_hz=${scan_rate_hz} max_scan_age_ms=${scan_max_age_ms} tf_wait_timeout_ms=${scan_wait_for_tf_timeout_ms}" >&2
}

wait_for_scan_admission_status_ready() {
  scan_admission_enabled || return 0
  local timeout_sec="${NJRH_AMCL_SCAN_ADMISSION_READY_TIMEOUT_SEC:-8}"
  local status_topic="${NJRH_AMCL_SCAN_ADMISSION_STATUS_TOPIC:-/amcl_scan_admission/status}"
  timeout "$(( ${timeout_sec%.*} + 3 ))" python3 - "${status_topic}" "${timeout_sec}" <<'PY' 2>/dev/null
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

topic = sys.argv[1]
timeout_sec = float(sys.argv[2])
rclpy.init()
node = rclpy.create_node("amcl_scan_admission_ready_waiter")
state = {"ready": False, "last": None}

def on_msg(msg):
    state["last"] = msg.data
    try:
        data = json.loads(msg.data)
    except Exception:
        return
    if data.get("enabled") is True and float(data.get("hz", 0.0) or 0.0) > 0.0 and str(data.get("last_error", "none")) in ("", "none"):
        state["ready"] = True

node.create_subscription(String, topic, on_msg, 10)
deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1.0e9)
while rclpy.ok() and not state["ready"] and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if not state["ready"]:
    print(f"not_ready last={state['last']}")
    raise SystemExit(1)
print("ready")
PY
}

request_amcl_nomotion_update_and_wait_for_pose() {
  local pose_topic="${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}"
  local service="${NJRH_AMCL_NOMOTION_UPDATE_SERVICE:-/request_nomotion_update}"
  local timeout_sec="${AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-${NJRH_AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-5.0}}"
  local max_age_sec="${AMCL_CORRECTION_MAX_POSE_AGE_SEC:-${NJRH_AMCL_CORRECTION_MAX_POSE_AGE_SEC:-1.0}}"
  local warmup_sec="${NJRH_AMCL_NOMOTION_PRE_SUBSCRIBE_WARMUP_SEC:-0.2}"
  local require_header_fresh="${NJRH_AMCL_SEED_READINESS_REQUIRE_HEADER_FRESH:-false}"
  local timeout_int="${timeout_sec%.*}"
  [[ -n "${timeout_int}" ]] || timeout_int=5
  [[ -x "${NOMOTION_PROBE}" ]] || {
    echo "[runtime-overlay] AMCL no-motion helper missing or not executable: ${NOMOTION_PROBE}" >&2
    return 30
  }
  local output
  AMCL_NOMOTION_PROBE_USED=true
  set +e
  output="$(timeout "$(( timeout_int + 6 ))" python3 "${NOMOTION_PROBE}" \
    --pose-topic "${pose_topic}" \
    --service "${service}" \
    --timeout-sec "${timeout_sec}" \
    --pre-subscribe-warmup-sec "${warmup_sec}" \
    --require-header-fresh "${require_header_fresh}" \
    --max-header-age-sec "${max_age_sec}" 2>&1)"
  local rc=$?
  set -e
  AMCL_NOMOTION_POSE_RECEIVED="$(python3 -c 'import json,sys; d=json.loads(sys.stdin.read() or "{}"); print("true" if d.get("pose_received") else "false")' <<<"${output}" 2>/dev/null || echo false)"
  AMCL_NOMOTION_POSE_COUNT="$(python3 -c 'import json,sys; d=json.loads(sys.stdin.read() or "{}"); print(int(d.get("pose_count") or 0))' <<<"${output}" 2>/dev/null || echo 0)"
  AMCL_NOMOTION_POSE_HEADER_AGE_MS="$(python3 -c 'import json,sys; d=json.loads(sys.stdin.read() or "{}"); v=d.get("pose_header_age_at_receive_sec"); print("" if v is None else f"{float(v)*1000.0:.3f}")' <<<"${output}" 2>/dev/null || true)"
  if [[ "${rc}" -ne 0 ]]; then
    echo "[runtime-overlay] AMCL no-motion probe failed rc=${rc}: ${output}" >&2
    return "${rc}"
  fi
  echo "[runtime-overlay] AMCL no-motion update response: ${output}" >&2
  AMCL_SEED_RESPONSE_OK=true
  AMCL_NOMOTION_POSE_RECEIVED=true
}

wait_for_amcl_pose_fresh_or_nomotion_update() {
  wait_for_amcl_pose_fresh && return 0
  request_amcl_nomotion_update_and_wait_for_pose
}

wait_for_amcl_pose_fresh() {
  local pose_topic="${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}"
  local timeout_sec="${NJRH_AMCL_POSE_FRESH_TIMEOUT_SEC:-5.0}"
  local max_age_sec="${NJRH_AMCL_POSE_MAX_AGE_SEC:-1.0}"
  timeout "$(( ${timeout_sec%.*} + 3 ))" python3 - "${pose_topic}" "${timeout_sec}" "${max_age_sec}" <<'PY' 2>/dev/null
import sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

topic = sys.argv[1]
timeout_sec = float(sys.argv[2])
max_age_sec = float(sys.argv[3])
rclpy.init()
node = rclpy.create_node("amcl_pose_fresh_waiter")
result = {"fresh": False, "age": None}

def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

def on_msg(msg):
    now_sec = node.get_clock().now().nanoseconds * 1.0e-9
    age = now_sec - stamp_to_sec(msg.header.stamp)
    result["age"] = age
    if 0.0 <= age <= max_age_sec:
        result["fresh"] = True

sub = node.create_subscription(PoseWithCovarianceStamped, topic, on_msg, 10)
deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1.0e9)
while rclpy.ok() and not result["fresh"] and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if not result["fresh"]:
    print(f"stale_or_missing age={result['age']}")
    raise SystemExit(1)
print(f"fresh age={result['age']}")
PY
}

complete_amcl_readiness_sequence() {
  wait_for_amcl_tf_warmup true || return 1
  start_scan_admission_relay || return 2
  wait_for_scan_admission_status_ready || return 3
  seed_amcl_initial_pose || return 4
  if [[ "${NJRH_AMCL_READY_REQUIRE_FRESH_POSE:-true}" == "true" ]]; then
    wait_for_amcl_pose_fresh_or_nomotion_update || return 5
  fi
}

start_amcl_node() {
  if [[ "${MODE}" == "disabled" ]]; then
    echo "[runtime-overlay] AMCL localization mode disabled; not starting AMCL" >&2
    return 0
  fi
  [[ -f "${PARAMS_FILE}" ]] || {
    echo "[runtime-overlay] AMCL params file missing: ${PARAMS_FILE}" >&2
    return 1
  }

  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  local amcl_cpuset
  amcl_cpuset="$(njrh_cpuset_for amcl 2>/dev/null || true)"
  amcl_cpuset="${amcl_cpuset:-${NJRH_CPUSET_AMCL:-${NJRH_CPUSET_LOCALIZATION:-6}}}"
  export NJRH_CPUSET_AMCL="${amcl_cpuset}"
  if ! command -v taskset >/dev/null 2>&1; then
    echo "[runtime-overlay] taskset is required for AMCL affinity" >&2
    return 1
  fi
  if ! taskset -c "${amcl_cpuset}" true >/dev/null 2>&1; then
    echo "[runtime-overlay] invalid AMCL cpuset=${amcl_cpuset}" >&2
    return 1
  fi

  if pid="$(amcl_pid_from_file 2>/dev/null || true)" && [[ -n "${pid}" ]] && pid_alive "${pid}"; then
    echo "[runtime-overlay] AMCL already running pid=${pid}" >&2
    mapfile -t existing_amcl_pids < <(amcl_process_pids)
    if [[ "${#existing_amcl_pids[@]}" -gt 0 ]]; then
      njrh_apply_affinity_to_pids amcl "${existing_amcl_pids[@]}" >/dev/null 2>&1 || true
    fi
    activate_amcl_lifecycle || return 1
    return 0
  fi

  echo "[runtime-overlay] starting AMCL mode=${MODE}; params=${PARAMS_FILE}; scan_topic=$(effective_scan_topic); cpuset=${amcl_cpuset}" >&2
  : >"${LOG_FILE}"
  nohup taskset -c "${amcl_cpuset}" ros2 run nav2_amcl amcl --ros-args \
    --params-file "${PARAMS_FILE}" \
    -p "scan_topic:=$(effective_scan_topic)" \
    -p "tf_broadcast:=false" \
    -r "__node:=${AMCL_NODE_NAME}" \
    >>"${LOG_FILE}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" >"${PID_FILE}"
  sleep "${NJRH_AMCL_START_SETTLE_SEC:-1}"
  if ! pid_alive "${pid}"; then
    echo "[runtime-overlay] AMCL failed to stay alive; check ${LOG_FILE}" >&2
    rm -f "${PID_FILE}"
    return 1
  fi

  activate_amcl_lifecycle || return 1
}

start_amcl_resident() {
  start_amcl_node || {
    finish_amcl_status failed false false "AMCL node failed to start or activate" "${AMCL_EXIT_LIFECYCLE_FAILED}"
    return $?
  }
  if [[ "${MODE}" == "disabled" ]]; then
    finish_amcl_status disabled true false "" "${AMCL_EXIT_READY}"
    return 0
  fi
  wait_for_amcl_tf_warmup false || {
    amcl_warn "resident AMCL active but map/scan/odom/base_link/scan_frame warmup is not complete yet"
  }
  start_scan_admission_relay || {
    if [[ "${MODE}" == "shadow" ]]; then
      finish_amcl_status degraded false true "scan admission failed during resident start" "${AMCL_EXIT_DEGRADED}"
      return $?
    fi
    finish_amcl_status failed false false "scan admission failed during resident start" "${AMCL_EXIT_SCAN_ADMISSION_FAILED}"
    return $?
  }
  finish_amcl_status waiting_seed false false "resident AMCL started; waiting for initial pose seed" "${AMCL_EXIT_READY}"
  echo "[runtime-overlay] AMCL_RESIDENT mode=${MODE} scan_topic=$(effective_scan_topic)" >&2
}

start_amcl() {
  start_amcl_node || {
    finish_amcl_status failed false false "AMCL node failed to start or activate" "${AMCL_EXIT_LIFECYCLE_FAILED}"
    return $?
  }
  if [[ "${MODE}" == "disabled" ]]; then
    finish_amcl_status disabled true false "" "${AMCL_EXIT_READY}"
    return 0
  fi
  local readiness_rc=0
  complete_amcl_readiness_sequence || readiness_rc=$?
  if [[ "${readiness_rc}" -ne 0 ]]; then
    local reason="AMCL lifecycle is active but not ready"
    local exit_code="${AMCL_EXIT_GATED_NOT_READY}"
    case "${readiness_rc}" in
      1)
        reason="map/scan/TF warmup failed"
        exit_code="${AMCL_EXIT_GATED_NOT_READY}"
        ;;
      2)
        reason="scan admission process failed"
        exit_code="${AMCL_EXIT_SCAN_ADMISSION_FAILED}"
        ;;
      3)
        reason="scan admission status not ready"
        exit_code="${AMCL_EXIT_SCAN_ADMISSION_FAILED}"
        ;;
      4)
        reason="AMCL initial pose seed failed"
        exit_code="${AMCL_EXIT_SEED_FAILED}"
        ;;
      5)
        if [[ "${AMCL_NOMOTION_PROBE_USED}" == "true" && "${AMCL_NOMOTION_POSE_RECEIVED}" != "true" ]]; then
          reason="NOMOTION_NO_POSE"
        else
          reason="/amcl_pose missing or stale after seed"
        fi
        exit_code="${AMCL_EXIT_POSE_MISSING}"
        ;;
    esac
    if [[ "${MODE}" == "shadow" ]]; then
      amcl_warn "${reason}; continuing triggered localization baseline as visible degraded shadow mode"
      finish_amcl_status degraded false true "${reason}" "${AMCL_EXIT_DEGRADED}"
      return $?
    fi
    finish_amcl_status failed false false "${reason}" "${exit_code}"
    return $?
  fi
  finish_amcl_status ready true false "" "${AMCL_EXIT_READY}"
  echo "[runtime-overlay] AMCL_READY mode=${MODE} scan_topic=$(effective_scan_topic)" >&2
}

case "${ACTION}" in
  print)
    print_config
    ;;
  stop)
    stop_amcl
    write_amcl_runtime_status stopped false false "AMCL stopped"
    ;;
  restart)
    stop_amcl
    start_amcl
    ;;
  resident)
    start_amcl_resident
    ;;
  complete)
    start_amcl
    ;;
  start)
    start_amcl
    ;;
esac

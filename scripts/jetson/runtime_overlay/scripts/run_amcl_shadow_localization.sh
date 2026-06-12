#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

ACTION="start"
MODE="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
PID_FILE="${NJRH_AMCL_PID_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_shadow_localization.pid}"
LOG_FILE="${NJRH_AMCL_LOG_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_shadow_localization.log}"
PARAMS_FILE="${NJRH_AMCL_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/amcl_shadow.yaml}"
SEED_SERVICE="${NJRH_AMCL_SEED_SERVICE:-/robot_localization_bridge/seed_amcl_initial_pose}"
AMCL_NODE_NAME="${NJRH_AMCL_NODE_NAME:-amcl}"

usage() {
  cat <<'USAGE'
Usage: run_amcl_shadow_localization.sh [--restart|--stop|--print] [--mode disabled|shadow|gated]

Starts the opt-in AMCL candidate localization node. AMCL never publishes TF in
this profile; robot_localization_bridge remains the only map->odom owner.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --restart)
      ACTION="restart"
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

print_config() {
  echo "NJRH_AMCL_LOCALIZATION_MODE=${MODE}"
  echo "NJRH_AMCL_PARAMS_FILE=${PARAMS_FILE}"
  echo "NJRH_AMCL_POSE_TOPIC=${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}"
  echo "NJRH_AMCL_INITIAL_POSE_TOPIC=${NJRH_AMCL_INITIAL_POSE_TOPIC:-/initialpose}"
  echo "NJRH_AMCL_PID_FILE=${PID_FILE}"
  echo "NJRH_AMCL_LOG_FILE=${LOG_FILE}"
  echo "NJRH_AMCL_SEED_SERVICE=${SEED_SERVICE}"
}

pid_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
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

amcl_pid_from_file() {
  [[ -f "${PID_FILE}" ]] || return 1
  local pid
  pid="$(tr -dc '0-9' <"${PID_FILE}" || true)"
  [[ -n "${pid}" ]] || return 1
  printf '%s\n' "${pid}"
}

amcl_process_pids() {
  ps -eo pid=,args= |
    awk -v node_name="__node:=${AMCL_NODE_NAME}" '
      /nav2_amcl\/amcl/ && index($0, node_name) > 0 {print $1}
      /ros2 run nav2_amcl amcl/ && index($0, node_name) > 0 {print $1}
    ' || true
}

stop_amcl() {
  timeout 5 ros2 lifecycle set "/${AMCL_NODE_NAME}" shutdown >/dev/null 2>&1 || true

  local pid=""
  pid="$(amcl_pid_from_file 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && pid_alive "${pid}"; then
    echo "[runtime-overlay] stopping AMCL pid=${pid}" >&2
    kill -INT "${pid}" 2>/dev/null || true
    wait_for_pid_exit "${pid}" "${NJRH_AMCL_STOP_INT_ATTEMPTS:-30}" || {
      kill -TERM "${pid}" 2>/dev/null || true
      wait_for_pid_exit "${pid}" "${NJRH_AMCL_STOP_TERM_ATTEMPTS:-30}" || true
    }
  fi
  local extra_pids
  extra_pids="$(amcl_process_pids)"
  if [[ -n "${extra_pids}" ]]; then
    echo "[runtime-overlay] stopping remaining AMCL processes: ${extra_pids}" >&2
    kill -INT ${extra_pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_STOP_INT_WAIT_SEC:-1}"
    extra_pids="$(amcl_process_pids)"
    if [[ -n "${extra_pids}" ]]; then
      kill -TERM ${extra_pids} 2>/dev/null || true
    fi
  fi
  rm -f "${PID_FILE}"
}

wait_for_amcl_node() {
  local timeout_sec="${1:-15}"
  runtime_readiness_probe node "/${AMCL_NODE_NAME}" "${timeout_sec}" >/dev/null 2>&1
}

activate_amcl_lifecycle() {
  wait_for_amcl_node "${NJRH_AMCL_NODE_WAIT_SEC:-15}" || {
    echo "[runtime-overlay] /${AMCL_NODE_NAME} did not appear after start" >&2
    return 1
  }
  timeout 8 ros2 lifecycle set "/${AMCL_NODE_NAME}" configure >/dev/null 2>&1 || true
  timeout 8 ros2 lifecycle set "/${AMCL_NODE_NAME}" activate >/dev/null 2>&1 || true
  local state
  state="$(timeout 5 ros2 lifecycle get "/${AMCL_NODE_NAME}" 2>/dev/null || true)"
  if [[ "${state}" != active* ]]; then
    echo "[runtime-overlay] /${AMCL_NODE_NAME} lifecycle is not active yet: ${state:-unknown}" >&2
    return 1
  fi
}

seed_amcl_initial_pose() {
  if ! wait_for_ros_service "${SEED_SERVICE}" "${NJRH_AMCL_SEED_SERVICE_WAIT_SEC:-8}" >/dev/null 2>&1; then
    echo "[runtime-overlay] AMCL seed service unavailable: ${SEED_SERVICE}" >&2
    return 0
  fi
  local output
  output="$(timeout "${NJRH_AMCL_SEED_CALL_TIMEOUT_SEC:-8}" ros2 service call \
    "${SEED_SERVICE}" std_srvs/srv/Trigger "{}" 2>&1 || true)"
  echo "[runtime-overlay] AMCL initial pose seed result: ${output}" >&2
}

start_amcl() {
  if [[ "${MODE}" == "disabled" ]]; then
    echo "[runtime-overlay] AMCL localization mode disabled; not starting AMCL" >&2
    return 0
  fi
  [[ -f "${PARAMS_FILE}" ]] || {
    echo "[runtime-overlay] AMCL params file missing: ${PARAMS_FILE}" >&2
    exit 1
  }

  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  if ! timeout 5 ros2 topic echo /map --once >/dev/null 2>&1; then
    echo "[runtime-overlay] /map is not publishing yet; AMCL will start and wait for the map topic" >&2
  fi

  if pid="$(amcl_pid_from_file 2>/dev/null || true)" && [[ -n "${pid}" ]] && pid_alive "${pid}"; then
    echo "[runtime-overlay] AMCL already running pid=${pid}" >&2
    seed_amcl_initial_pose
    return 0
  fi

  echo "[runtime-overlay] starting AMCL mode=${MODE}; params=${PARAMS_FILE}" >&2
  : >"${LOG_FILE}"
  nohup ros2 run nav2_amcl amcl --ros-args \
    --params-file "${PARAMS_FILE}" \
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
  seed_amcl_initial_pose
}

case "${ACTION}" in
  print)
    print_config
    ;;
  stop)
    stop_amcl
    ;;
  restart)
    stop_amcl
    start_amcl
    ;;
  start)
    start_amcl
    ;;
esac

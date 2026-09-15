#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/map_server_helpers.sh"
source "${SCRIPT_DIR}/amcl_startup_progress.sh"

amcl_startup_side_effect_guard() {
  # Only startup-owned invocations inherit these variables. Standalone AMCL
  # maintenance keeps its existing interface and authorization semantics.
  if [[ -z "${NJRH_STARTUP_OWNER_PID:-}" && -z "${NJRH_FLOOR_STARTUP_HANDOFF_NONCE:-}" ]]; then
    return 0
  fi
  PYTHONPATH="${SCRIPT_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
    python3 -c 'from floor_startup_handoff import require_startup_side_effect_permission; require_startup_side_effect_permission()'
}

ACTION="start"
MODE="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
PID_FILE="${NJRH_AMCL_PID_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_shadow_localization.pid}"
LOG_FILE="${NJRH_AMCL_LOG_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_shadow_localization.log}"
PARAMS_FILE="${NJRH_AMCL_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/amcl_shadow.yaml}"
SEED_SERVICE="${NJRH_AMCL_SEED_SERVICE:-/robot_localization_bridge/seed_amcl_initial_pose}"
AMCL_NODE_NAME="${NJRH_AMCL_NODE_NAME:-amcl}"
AMCL_BIN="${NJRH_AMCL_BIN:-/opt/ros/humble/lib/nav2_amcl/amcl}"
AMCL_LIFECYCLE_HELPER="${NJRH_AMCL_LIFECYCLE_HELPER:-${SCRIPT_DIR}/nav2_lifecycle_sequence.py}"
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
AMCL_EXIT_PENDING=26

AMCL_PID_STALE_CLEARED=false
SCAN_ADMISSION_PID_STALE_CLEARED=false
AMCL_SEED_SUCCEEDED=false
AMCL_SEED_RESPONSE_OK=false
AMCL_NOMOTION_PROBE_USED=false
AMCL_NOMOTION_POSE_RECEIVED=false
AMCL_NOMOTION_POSE_COUNT=0
AMCL_NOMOTION_POSE_HEADER_AGE_MS=""
AMCL_STATIC_STANDBY_ACCEPTED=false
AMCL_STARTUP_EPOCH_SEC=""

usage() {
  cat <<'USAGE'
Usage: run_amcl_shadow_localization.sh [--restart|--stop|--print|--start-resident|--complete-readiness|--heartbeat] [--mode disabled|shadow|gated]

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
    --heartbeat)
      ACTION="heartbeat"
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
  echo "NJRH_AMCL_BIN=${AMCL_BIN}"
  echo "NJRH_AMCL_LIFECYCLE_HELPER=${AMCL_LIFECYCLE_HELPER}"
  echo "NJRH_AMCL_LIFECYCLE_NODE_TIMEOUT_SEC=${NJRH_AMCL_LIFECYCLE_NODE_TIMEOUT_SEC:-${NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC:-12}}"
  echo "NJRH_AMCL_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC=${NJRH_AMCL_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC:-5}"
  echo "NJRH_AMCL_LIFECYCLE_HELPER_TIMEOUT_SEC=${NJRH_AMCL_LIFECYCLE_HELPER_TIMEOUT_SEC:-30}"
  echo "NJRH_AMCL_NOMOTION_PROBE=${NOMOTION_PROBE}"
  echo "AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC=${AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-${NJRH_AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-5.0}}"
  echo "AMCL_NOMOTION_UPDATE_ACCEPT_RECEIVED_AFTER_CALL=${AMCL_NOMOTION_UPDATE_ACCEPT_RECEIVED_AFTER_CALL:-${NJRH_AMCL_NOMOTION_UPDATE_ACCEPT_RECEIVED_AFTER_CALL:-true}}"
  echo "AMCL_SEED_READINESS_DO_NOT_REQUIRE_FRESH_HEADER_WHEN_STATIC=${AMCL_SEED_READINESS_DO_NOT_REQUIRE_FRESH_HEADER_WHEN_STATIC:-${NJRH_AMCL_SEED_READINESS_DO_NOT_REQUIRE_FRESH_HEADER_WHEN_STATIC:-true}}"
  echo "AMCL_CORRECTION_MAX_POSE_AGE_SEC=${AMCL_CORRECTION_MAX_POSE_AGE_SEC:-${NJRH_AMCL_CORRECTION_MAX_POSE_AGE_SEC:-1.0}}"
  echo "NJRH_AMCL_TF_WARMUP_SEC=${NJRH_AMCL_TF_WARMUP_SEC:-3.0}"
  echo "NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC=${NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC:-8}"
  echo "NJRH_AMCL_LIFECYCLE_POST_TRANSITION_STATE_WAIT_SEC=${NJRH_AMCL_LIFECYCLE_POST_TRANSITION_STATE_WAIT_SEC:-2}"
  echo "NJRH_AMCL_PARAM_READY_TIMEOUT_SEC=${NJRH_AMCL_PARAM_READY_TIMEOUT_SEC:-5}"
  echo "NJRH_AMCL_SEED_RETRY_COUNT=${NJRH_AMCL_SEED_RETRY_COUNT:-5}"
  echo "NJRH_AMCL_SCAN_ADMISSION_ENABLED=${NJRH_AMCL_SCAN_ADMISSION_ENABLED:-true}"
  echo "NJRH_AMCL_SCAN_INPUT_TOPIC=${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}"
  echo "NJRH_AMCL_SCAN_OUTPUT_TOPIC=${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}"
  echo "NJRH_AMCL_EFFECTIVE_SCAN_TOPIC=$(effective_scan_topic)"
  echo "NJRH_AMCL_SCAN_RATE_HZ=${NJRH_AMCL_SCAN_RATE_HZ:-5.0}"
  echo "NJRH_AMCL_SCAN_PRESERVE_STAMP=${NJRH_AMCL_SCAN_PRESERVE_STAMP:-true}"
  echo "NJRH_AMCL_SCAN_MAX_AGE_MS=${NJRH_AMCL_SCAN_MAX_AGE_MS:-1000}"
  echo "NJRH_AMCL_SCAN_TARGET_FRAME=${NJRH_AMCL_SCAN_TARGET_FRAME:-odom}"
  echo "NJRH_AMCL_SCAN_ADMISSION_IMPL=${SCAN_RELAY_IMPL}"
  echo "NJRH_AMCL_SCAN_ADMISSION_CPP_BIN=${SCAN_RELAY_CPP_BIN}"
  echo "NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_FRESH_WAIT=${NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_FRESH_WAIT:-true}"
  echo "NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_ADMISSION_READY_WAIT=${NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_ADMISSION_READY_WAIT:-true}"
  echo "NJRH_CPUSET_AMCL=${NJRH_CPUSET_AMCL:-${NJRH_CPUSET_LOCALIZATION:-6}}"
  echo "NJRH_CPUSET_AMCL_SCAN_ADMISSION=${NJRH_CPUSET_AMCL_SCAN_ADMISSION:-${NJRH_CPUSET_LOCALIZATION:-6}}"
}

pid_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
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

amcl_status_cli() {
  local binary="${NJRH_AMCL_STATUS_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_bringup/lib/robot_bringup/runtime_amcl_status}"
  [[ -x "${binary}" ]] || {
    echo "[runtime-overlay] native AMCL status client missing: ${binary}" >&2
    return 1
  }
  "${binary}" --status-file "${STATUS_FILE}" \
    --timeout-ms "${NJRH_AMCL_STATUS_REQUEST_TIMEOUT_MS:-1000}" "$@"
}

write_amcl_runtime_status() {
  local start_result="$1" ready="$2" degraded="$3" reason="${4:-}"
  local relay_exe="${SCAN_RELAY_CPP_BIN}" relay_arg=""
  if [[ "${SCAN_RELAY_IMPL}" == "python" ]]; then
    relay_exe="${NJRH_AMCL_SCAN_ADMISSION_PYTHON_BIN:-$(command -v python3)}"
    relay_arg="${SCAN_RELAY_SCRIPT}"
  fi
  local scan_enabled=false
  scan_admission_enabled && scan_enabled=true
  local owner_pid="${NJRH_AMCL_STATUS_OWNER_PID:-${NJRH_STARTUP_OWNER_PID:-$PPID}}"
  local owner_generation="${NJRH_AMCL_STATUS_OWNER_GENERATION:-${NJRH_FLOOR_STARTUP_HANDOFF_NONCE:-standalone}}"
  local map_generation="${NJRH_BUILDING_ID:-}|${NJRH_FLOOR_ID:-}|${NJRH_NAV_MAP_ID:-${NJRH_MAP_ID:-}}|${NJRH_MAP_ASSET_EPOCH:-}|${NJRH_MAP_ASSET_DIGEST:-}|${NAV2_MAP_YAML:-}|${PARAMS_FILE}"
  # Only lifecycle/seed receipts enter the guard. Never source the public file
  # as proof; the guard owns incarnation-safe evidence merging and publication.
  amcl_status_cli submit \
    --owner-pid "${owner_pid}" --owner-generation "${owner_generation}" \
    --map-generation "${map_generation}" \
    --amcl-pid-file "${PID_FILE}" --amcl-exe "${AMCL_BIN}" --amcl-argument "__node:=${AMCL_NODE_NAME}" \
    --relay-pid-file "${SCAN_RELAY_PID_FILE}" --relay-exe "${relay_exe}" --relay-argument "${relay_arg}" \
    --set "AMCL_MODE=${MODE}" --set "AMCL_START_RESULT=${start_result}" \
    --set "AMCL_READY=${ready}" --set "AMCL_DEGRADED=${degraded}" --set "AMCL_FAILURE_REASON=${reason}" \
    --set "AMCL_STARTUP_EPOCH_SEC=${AMCL_STARTUP_EPOCH_SEC}" \
    --set "AMCL_PID_STALE_CLEARED=${AMCL_PID_STALE_CLEARED}" \
    --set "SCAN_ADMISSION_PID_STALE_CLEARED=${SCAN_ADMISSION_PID_STALE_CLEARED}" \
    --set "SCAN_ADMISSION_IMPL=${SCAN_RELAY_IMPL}" --set "SCAN_ADMISSION_ENABLED=${scan_enabled}" \
    --set "LIFECYCLE_VERIFIED=${AMCL_PROGRESS_LIFECYCLE:-false}" --set "PROGRESS_KEY=${AMCL_PROGRESS_KEY:-}" \
    --set "AMCL_SEED_SUCCEEDED=${AMCL_SEED_SUCCEEDED}" --set "AMCL_SEED_RESPONSE_OK=${AMCL_SEED_RESPONSE_OK}" \
    --set "AMCL_NOMOTION_PROBE_USED=${AMCL_NOMOTION_PROBE_USED}" \
    --set "AMCL_NOMOTION_POSE_RECEIVED=${AMCL_NOMOTION_POSE_RECEIVED}" \
    --set "AMCL_NOMOTION_POSE_COUNT=${AMCL_NOMOTION_POSE_COUNT}" \
    --set "AMCL_NOMOTION_POSE_HEADER_AGE_MS=${AMCL_NOMOTION_POSE_HEADER_AGE_MS}" \
    --set "AMCL_STATIC_STANDBY_ACCEPTED=${AMCL_STATIC_STANDBY_ACCEPTED}" \
    --set "NODE_NAME=${AMCL_NODE_NAME}" --set "POSE_TOPIC=${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}" \
    --set "SCAN_TOPIC=$(effective_scan_topic)" \
    --set "ADMISSION_STATUS_TOPIC=${NJRH_AMCL_SCAN_ADMISSION_STATUS_TOPIC:-/amcl_scan_admission/status}" \
    >/dev/null
}

finish_amcl_status() {
  amcl_startup_side_effect_guard || return "${AMCL_EXIT_FAILED}"
  local start_result="$1"
  local ready="$2"
  local degraded="$3"
  local reason="${4:-}"
  local code="${5:-0}"
  if ! write_amcl_runtime_status "${start_result}" "${ready}" "${degraded}" "${reason}"; then
    # A cold status observer must not turn resident preparation into failure.
    # READY (including disabled mode) still requires an accepted evidence commit.
    if [[ "${ready}" == "false" && "${degraded}" == "false" &&
          ( "${start_result}" == "starting" || "${start_result}" == "waiting_seed" ) ]]; then
      echo "[runtime-overlay] AMCL progress status not submitted: result=${start_result}; preserving initialization result=${code}" >&2
    else
      return "${AMCL_EXIT_FAILED}"
    fi
  fi
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

heartbeat_amcl_runtime_status() {
  # Compatibility command: the existing guard owns all recurring refreshes.
  amcl_status_cli ping >/dev/null
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

amcl_heartbeat_process_pids() {
  local self_pid="$$"
  ps -eo pid=,args= |
    awk -v self_pid="${self_pid}" '
      $1 != self_pid && /run_amcl_shadow_localization.sh/ && /--heartbeat/ && !/awk/ {print $1}
    ' || true
}

amcl_nonstop_runner_process_pids() {
  local self_pid="$$"
  ps -eo pid=,args= |
    awk -v self_pid="${self_pid}" '
      $1 != self_pid && /run_amcl_shadow_localization.sh/ && !/--stop/ && !/awk/ {print $1}
    ' || true
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
    sleep "${NJRH_AMCL_SCAN_ADMISSION_STOP_INT_WAIT_SEC:-0.3}"
    extra_pids="$(scan_relay_process_pids)"
    [[ -z "${extra_pids}" ]] || kill -TERM ${extra_pids} 2>/dev/null || true
  fi
  rm -f "${SCAN_RELAY_PID_FILE}"
}

stop_amcl_heartbeat_processes() {
  local extra_pids
  extra_pids="$(amcl_heartbeat_process_pids)"
  if [[ -n "${extra_pids}" ]]; then
    echo "[runtime-overlay] stopping remaining AMCL runtime status heartbeat processes: ${extra_pids}" >&2
    kill -INT ${extra_pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC:-0.3}"
    extra_pids="$(amcl_heartbeat_process_pids)"
    [[ -z "${extra_pids}" ]] || kill -TERM ${extra_pids} 2>/dev/null || true
  fi
}

stop_amcl_runner_processes() {
  local extra_pids
  extra_pids="$(amcl_nonstop_runner_process_pids)"
  if [[ -n "${extra_pids}" ]]; then
    echo "[runtime-overlay] stopping remaining AMCL runner processes: ${extra_pids}" >&2
    kill -INT ${extra_pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_RUNNER_STOP_INT_WAIT_SEC:-0.3}"
    extra_pids="$(amcl_nonstop_runner_process_pids)"
    [[ -z "${extra_pids}" ]] || kill -TERM ${extra_pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_RUNNER_STOP_TERM_WAIT_SEC:-0.3}"
    extra_pids="$(amcl_nonstop_runner_process_pids)"
    if [[ -n "${extra_pids}" ]]; then
      echo "[runtime-overlay] AMCL runner processes ignored SIGTERM; killing exact pids: ${extra_pids}" >&2
      kill -KILL ${extra_pids} 2>/dev/null || true
      sleep "${NJRH_AMCL_RUNNER_STOP_KILL_WAIT_SEC:-0.2}"
    fi
  fi
}

stop_amcl() {
  stop_amcl_runner_processes
  stop_amcl_heartbeat_processes
  stop_scan_admission_relay
  timeout "${NJRH_AMCL_LIFECYCLE_SHUTDOWN_TIMEOUT_SEC:-2}" ros2 lifecycle set "/${AMCL_NODE_NAME}" shutdown >/dev/null 2>&1 || true

  local pid=""
  pid="$(amcl_pid_from_file 2>/dev/null || true)"
  stop_pid_softly "AMCL" "${pid}"
  local extra_pids
  extra_pids="$(amcl_process_pids)"
  if [[ -n "${extra_pids}" ]]; then
    echo "[runtime-overlay] stopping remaining AMCL processes: ${extra_pids}" >&2
    kill -INT ${extra_pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_STOP_INT_WAIT_SEC:-0.3}"
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
  amcl_client_timeout "${NJRH_AMCL_PARAM_GET_TIMEOUT_SEC:-3}" ros2 param get "/${AMCL_NODE_NAME}" "${param}" 2>/dev/null || true
}

amcl_cmdline_tf_broadcast_false() {
  local pid=""
  pid="$(amcl_pid_from_file 2>/dev/null || true)"
  if [[ -z "${pid}" ]]; then
    pid="$(amcl_process_pids | first_process_pid || true)"
  fi
  [[ -n "${pid}" && -r "/proc/${pid}/cmdline" ]] || return 1
  local args
  args="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  [[ "${args}" == *"tf_broadcast:=false"* || "${args}" == *"tf_broadcast:=False"* ]]
}

wait_for_amcl_tf_broadcast_false() {
  local timeout_sec="${NJRH_AMCL_PARAM_READY_TIMEOUT_SEC:-5}"
  local timeout_int="${timeout_sec%.*}"
  [[ -n "${timeout_int}" ]] || timeout_int=5
  local deadline=$(( $(date +%s) + timeout_int ))
  local tf_broadcast=""
  local last_tf_broadcast=""

  while true; do
    amcl_budget_timeout 1 >/dev/null || return 124
    tf_broadcast="$(amcl_param_value tf_broadcast)"
    if [[ "${tf_broadcast}" == *"False"* || "${tf_broadcast}" == *"false"* ]]; then
      return 0
    fi
    last_tf_broadcast="${tf_broadcast}"
    if [[ "${tf_broadcast}" == *"True"* || "${tf_broadcast}" == *"true"* ]]; then
      echo "[runtime-overlay] AMCL tf_broadcast is true, expected false: ${tf_broadcast}" >&2
      return 1
    fi
    if [[ "$(date +%s)" -ge "${deadline}" ]]; then
      break
    fi
    amcl_budget_sleep "${NJRH_AMCL_PARAM_READY_POLL_SEC:-0.25}" || return 124
  done

  echo "[runtime-overlay] AMCL tf_broadcast did not become readable as false within ${timeout_sec}s: ${last_tf_broadcast:-missing}" >&2
  if amcl_cmdline_tf_broadcast_false; then
    echo "[runtime-overlay] AMCL tf_broadcast parameter service was not readable, but process launch argument is tf_broadcast:=false; continuing" >&2
    return 0
  fi
  return 1
}

activate_amcl_lifecycle() {
  amcl_startup_side_effect_guard || return 1
  [[ -f "${AMCL_LIFECYCLE_HELPER}" ]] || {
    echo "[runtime-overlay] AMCL lifecycle helper missing: ${AMCL_LIFECYCLE_HELPER}" >&2
    return 1
  }

  local node_timeout_sec="${NJRH_AMCL_LIFECYCLE_NODE_TIMEOUT_SEC:-${NJRH_AMCL_LIFECYCLE_TRANSITION_TIMEOUT_SEC:-12}}"
  local response_timeout_sec="${NJRH_AMCL_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC:-5}"
  local helper_timeout_sec="${NJRH_AMCL_LIFECYCLE_HELPER_TIMEOUT_SEC:-30}"
  local output=""
  if [[ "${AMCL_PROGRESS_LIFECYCLE:-false}" != "true" ]]; then
    if ! output="$(amcl_client_timeout "${helper_timeout_sec}" \
      python3 "${AMCL_LIFECYCLE_HELPER}" \
      --per-node-timeout-sec "${node_timeout_sec}" \
      --change-state-response-timeout-sec "${response_timeout_sec}" \
      "/${AMCL_NODE_NAME}" 2>&1)"; then
      echo "[runtime-overlay] /${AMCL_NODE_NAME} lifecycle query/transition incomplete via bounded client: ${output}" >&2
      return 26
    fi
    [[ -z "${output}" ]] || printf '%s\n' "${output}" >&2
    AMCL_PROGRESS_LIFECYCLE=true
    amcl_progress_save
  else
    echo "[runtime-overlay] AMCL reusing confirmed lifecycle activation for this process and startup context" >&2
  fi
  if [[ "${AMCL_PROGRESS_PARAM:-false}" != "true" ]]; then
    wait_for_amcl_tf_broadcast_false || return $?
    AMCL_PROGRESS_PARAM=true
    amcl_progress_save
  fi
}

amcl_warn() {
  local reason="$1"
  echo "[runtime-overlay] AMCL_WARN ${reason}" >&2
  return 0
}

amcl_preparation_step() {
  local phase="$1" limit="$2"
  shift 2
  local flag="AMCL_PROGRESS_${phase}" budget started rc=0
  [[ "${!flag:-false}" == "true" ]] && return 0
  budget="$(amcl_budget_timeout "${limit}")" || return 124
  started=${SECONDS}
  echo "[runtime-overlay] AMCL_STEP_BEGIN step=${phase} at=$(date -u +%FT%TZ) budget_sec=${budget}" >&2
  NJRH_RUNTIME_READINESS_PROBE_PROCESS_TIMEOUT_SEC="${budget}" \
    NJRH_RUNTIME_READINESS_PROBE_KILL_AFTER_SEC=0.2 "$@" || rc=$?
  echo "[runtime-overlay] AMCL_STEP_END step=${phase} at=$(date -u +%FT%TZ) elapsed_sec=$((SECONDS-started)) rc=${rc}" >&2
  [[ "${rc}" -eq 0 ]] || return "${rc}"
  printf -v "${flag}" '%s' true
  amcl_progress_save
}

wait_for_amcl_tf_warmup() {
  local require_map_odom="${1:-true}"
  local map_timeout="${NJRH_AMCL_MAP_WAIT_SEC:-15}"
  local scan_timeout="${NJRH_AMCL_SCAN_WAIT_SEC:-15}"
  local tf_timeout="${NJRH_AMCL_TF_WAIT_SEC:-15}"
  local scan_frame="${AMCL_PROGRESS_SCAN_FRAME:-${NJRH_AMCL_SCAN_FRAME_REQUIRED:-lidar_level_link}}"
  local phase flag pending="" output rc=0 limit budget started key value
  for phase in MAP SCAN MAP_TF ODOM_TF SENSOR_TF; do
    [[ "${phase}" == MAP_TF && "${require_map_odom}" != true ]] && continue
    flag="AMCL_PROGRESS_${phase}"
    [[ "${!flag:-false}" == true ]] || pending="${pending:+${pending},}${phase}"
  done
  if [[ -n "${pending}" ]]; then
    limit="$(awk -v m="${map_timeout}" -v s="${scan_timeout}" -v t="${tf_timeout}" \
      'BEGIN {v=m; if(s>v)v=s; if(t>v)v=t; printf "%.3f",v}')"
    budget="$(amcl_budget_timeout "${limit}")" || return 124
    started=${SECONDS}
    echo "[runtime-overlay] AMCL_STEP_BEGIN step=INPUTS at=$(date -u +%FT%TZ) budget_sec=${budget} pending=${pending}" >&2
    output="$(NJRH_RUNTIME_READINESS_PROBE_PROCESS_TIMEOUT_SEC="${budget}" \
      NJRH_RUNTIME_READINESS_PROBE_KILL_AFTER_SEC=0.2 runtime_readiness_probe \
      amcl-inputs "${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}" \
      "${NJRH_AMCL_SCAN_FRAME_REQUIRED:-lidar_level_link}" \
      "${AMCL_PROGRESS_SCAN_FRAME:--}" "${pending}" \
      "${map_timeout}" "${scan_timeout}" "${tf_timeout}" 2>&1)" || rc=$?
    [[ -z "${output}" ]] || printf '%s\n' "${output}" >&2
    # The bounded observer flushes successes as they arrive. Preserve partial
    # progress even when its deadline expires, under the existing identity.
    while IFS='=' read -r key value; do
      case "${key}" in
        AMCL_INPUT_READY)
          case "${value}" in
            MAP|SCAN|MAP_TF|ODOM_TF|SENSOR_TF)
              if [[ ",${pending}," == *",${value},"* ]]; then
                printf -v "AMCL_PROGRESS_${value}" '%s' true
              fi
              ;;
          esac
          ;;
        AMCL_INPUT_FRAME)
          [[ -n "${value}" && "${value}" != *$'\r'* ]] && AMCL_PROGRESS_SCAN_FRAME="${value}"
          ;;
      esac
    done <<<"${output}"
    amcl_progress_save
    echo "[runtime-overlay] AMCL_STEP_END step=INPUTS at=$(date -u +%FT%TZ) elapsed_sec=$((SECONDS-started)) rc=${rc}" >&2
    [[ "${rc}" -eq 0 ]] || return "${rc}"
    for phase in MAP SCAN MAP_TF ODOM_TF SENSOR_TF; do
      [[ ",${pending}," == *",${phase},"* ]] || continue
      flag="AMCL_PROGRESS_${phase}"
      [[ "${!flag:-false}" == true ]] || return 1
    done
    scan_frame="${AMCL_PROGRESS_SCAN_FRAME:-${scan_frame}}"
  fi

  local warmup_sec="${NJRH_AMCL_TF_WARMUP_SEC:-3.0}"
  if [[ "${AMCL_PROGRESS_WARMUP:-false}" != "true" ]]; then
    echo "[runtime-overlay] AMCL TF cache warmup ${warmup_sec}s after map/scan/TF gates; scan_frame=${scan_frame} require_map_odom=${require_map_odom}" >&2
  fi
  amcl_preparation_step WARMUP "${warmup_sec}" amcl_budget_sleep "${warmup_sec}"
}

seed_amcl_initial_pose() {
  local retry_count="${NJRH_AMCL_SEED_RETRY_COUNT:-5}"
  local retry_period_ms="${NJRH_AMCL_SEED_RETRY_PERIOD_MS:-300}"
  local wait_sec="${NJRH_AMCL_SEED_SERVICE_WAIT_SEC:-8}"
  local call_timeout_sec="${NJRH_AMCL_SEED_CALL_TIMEOUT_SEC:-8}"
  local attempt
  for ((attempt = 1; attempt <= retry_count; attempt += 1)); do
    local output
    local wait_int="${wait_sec%.*}"
    local call_int="${call_timeout_sec%.*}"
    [[ -n "${wait_int}" ]] || wait_int=8
    [[ -n "${call_int}" ]] || call_int=8
    amcl_require_client_budget "$((wait_int + call_int + 4))" || return 124
    amcl_startup_side_effect_guard || return 1
    # Authorization verification itself can take time; recheck before creating
    # a client. The Python request retains its own immediate owner check.
    amcl_require_client_budget "$((wait_int + call_int + 4))" || return 124
    output="$(amcl_client_timeout "$((wait_int + call_int + 4))" python3 - "${SEED_SERVICE}" "${wait_sec}" "${call_timeout_sec}" "${SCRIPT_DIR}" <<'PY' 2>&1 || true
import sys
import time

import rclpy
from std_srvs.srv import Trigger

service = sys.argv[1]
wait_sec = float(sys.argv[2])
call_timeout_sec = float(sys.argv[3])

rclpy.init()
node = rclpy.create_node(
    "amcl_seed_initial_pose_client", enable_rosout=False, start_parameter_services=False)
client = node.create_client(Trigger, service)
try:
    if not client.wait_for_service(timeout_sec=wait_sec):
        print(f"success=False message='service unavailable: {service}'")
        raise SystemExit(2)
    import os
    if os.environ.get("NJRH_STARTUP_OWNER_PID") or os.environ.get("NJRH_FLOOR_STARTUP_HANDOFF_NONCE"):
        sys.path.insert(0, sys.argv[4])
        from floor_startup_handoff import require_startup_side_effect_permission
        require_startup_side_effect_permission()
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + call_timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        print(f"success=False message='service call timeout: {service}'")
        raise SystemExit(3)
    response = future.result()
    success = bool(response.success)
    message = str(response.message)
    print(f"success={success} message={message!r}", flush=True)
    raise SystemExit(0 if success else 4)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
)"
    echo "[runtime-overlay] AMCL initial pose seed attempt=${attempt}/${retry_count}: ${output}" >&2
    if grep -Eiq "success[:=][[:space:]]*true|success=True" <<<"${output}"; then
      AMCL_SEED_SUCCEEDED=true
      return 0
    fi
    amcl_budget_sleep "$(awk -v ms="${retry_period_ms}" 'BEGIN {printf "%.3f", ms / 1000.0}')" || return 124
  done
  AMCL_SEED_SUCCEEDED=false
  return 1
}

start_scan_admission_relay() {
  amcl_startup_side_effect_guard || return 1
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
    if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" ]]; then
      relay_cpuset="$(njrh_effective_cpuset_for amcl_scan_admission)"
    fi
    allowed="$(scan_relay_allowed_cpus "${pid}")"
    if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" && "${allowed}" != "${relay_cpuset}" ]]; then
      # The single startup->steady transition may occur between these reads.
      relay_cpuset="$(njrh_effective_cpuset_for amcl_scan_admission)"
      allowed="$(scan_relay_allowed_cpus "${pid}")"
    fi
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
  scan_max_age_ms="$(awk -v v="${NJRH_AMCL_SCAN_MAX_AGE_MS:-1000.0}" 'BEGIN {printf "%.3f", v + 0.0}')"
  scan_wait_for_tf_timeout_ms="$(awk -v v="${NJRH_AMCL_SCAN_WAIT_FOR_TF_TIMEOUT_MS:-20.0}" 'BEGIN {printf "%.3f", v + 0.0}')"
  scan_input_topic="${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}"
  scan_output_topic="${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}"
  local relay_cmd=()
  if [[ "${SCAN_RELAY_IMPL}" == "cpp" ]]; then
    relay_cmd=("${SCAN_RELAY_CPP_BIN}" --ros-args
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
  amcl_startup_side_effect_guard || return 1
  local relay_affinity=(taskset -c "${relay_cpuset}")
  if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" ]]; then
    njrh_affinity_prefix relay_affinity amcl_scan_admission
  fi
  nohup "${relay_affinity[@]}" "${relay_cmd[@]}" >>"${SCAN_RELAY_LOG_FILE}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" >"${SCAN_RELAY_PID_FILE}"
  amcl_budget_sleep "${NJRH_AMCL_SCAN_ADMISSION_START_SETTLE_SEC:-0.5}" || return 124
  if ! pid_alive "${pid}"; then
    echo "[runtime-overlay] AMCL scan admission relay implementation=${SCAN_RELAY_IMPL} failed to stay alive; check ${SCAN_RELAY_LOG_FILE}" >&2
    rm -f "${SCAN_RELAY_PID_FILE}"
    return 1
  fi
  local allowed
  if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" ]]; then
    relay_cpuset="$(njrh_effective_cpuset_for amcl_scan_admission)"
  fi
  allowed="$(scan_relay_allowed_cpus "${pid}")"
  if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" && "${allowed}" != "${relay_cpuset}" ]]; then
    relay_cpuset="$(njrh_effective_cpuset_for amcl_scan_admission)"
    allowed="$(scan_relay_allowed_cpus "${pid}")"
  fi
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
  local min_ready_hz="${NJRH_AMCL_SCAN_ADMISSION_READY_MIN_HZ:-0.5}"
  amcl_client_timeout "$(( ${timeout_sec%.*} + 3 ))" python3 - "${status_topic}" "${timeout_sec}" "${min_ready_hz}" <<'PY' 2>/dev/null
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

topic = sys.argv[1]
timeout_sec = float(sys.argv[2])
min_ready_hz = float(sys.argv[3])
rclpy.init()
node = rclpy.create_node(
    "amcl_scan_admission_ready_waiter", enable_rosout=False, start_parameter_services=False)
state = {"ready": False, "last": None, "ready_status": None}

blocking_errors = (
    "AMCL_SCAN_TF_UNAVAILABLE",
    "AMCL_SCAN_FRAME_MISMATCH",
    "AMCL_SCAN_FUTURE_STAMP",
    "AMCL_SCAN_WARMUP",
)

def on_msg(msg):
    state["last"] = msg.data
    try:
        data = json.loads(msg.data)
    except Exception:
        return
    last_error = str(data.get("last_error", "none") or "none")
    if data.get("enabled") is not True:
        return
    if any(last_error.startswith(error) for error in blocking_errors):
        return
    hz = float(data.get("hz", 0.0) or 0.0)
    published_count = int(data.get("published_count", 0) or 0)
    if hz >= min_ready_hz and published_count > 0:
        state["ready_status"] = data
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
print(f"ready status={json.dumps(state['ready_status'], sort_keys=True)}")
PY
}

wait_for_fresh_amcl_scan_input() {
  local topic="${NJRH_AMCL_SCAN_INPUT_TOPIC:-/scan}"
  local timeout_sec="${NJRH_AMCL_SCAN_FRESH_WAIT_SEC:-20}"
  local max_age_sec="${NJRH_AMCL_SCAN_FRESH_MAX_AGE_SEC:-1.0}"
  local max_future_sec="${NJRH_AMCL_SCAN_FRESH_MAX_FUTURE_SEC:-0.05}"
  local budget
  budget="$(amcl_budget_timeout "${timeout_sec}")" || return 124
  NJRH_RUNTIME_READINESS_PROBE_PROCESS_TIMEOUT_SEC="${budget}" \
    NJRH_RUNTIME_READINESS_PROBE_KILL_AFTER_SEC=0.2 runtime_readiness_probe \
    fresh-header-topic \
    "${topic}" \
    "${timeout_sec}" \
    "${max_age_sec}" \
    "${max_future_sec}"
}

request_amcl_nomotion_update_and_wait_for_pose() {
  amcl_startup_side_effect_guard || return 1
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
  output="$(amcl_client_timeout "$(( timeout_int + 6 ))" python3 "${NOMOTION_PROBE}" \
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
  amcl_client_timeout "$(( ${timeout_sec%.*} + 3 ))" python3 - "${pose_topic}" "${timeout_sec}" "${max_age_sec}" <<'PY' 2>/dev/null
import sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

topic = sys.argv[1]
timeout_sec = float(sys.argv[2])
max_age_sec = float(sys.argv[3])
rclpy.init()
node = rclpy.create_node(
    "amcl_pose_fresh_waiter", enable_rosout=False, start_parameter_services=False)
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

amcl_static_standby_fast_seed_enabled() {
  [[ "${NJRH_AMCL_STATIC_STANDBY_WITHOUT_POSE_OK:-true}" == "true" &&
    "${NJRH_AMCL_STATIC_STANDBY_SKIP_POSE_WAIT:-true}" == "true" &&
    "${NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_FRESH_WAIT:-true}" == "true" ]]
}

complete_amcl_readiness_sequence() {
  wait_for_amcl_tf_warmup true || return 1
  if amcl_static_standby_fast_seed_enabled; then
    echo "[runtime-overlay] AMCL static standby fast seed: skipping /scan fresh wait; scan relay status still gates TF/frame/rate readiness" >&2
  else
    wait_for_fresh_amcl_scan_input || return 6
  fi
  start_scan_admission_relay || return 2
  if amcl_static_standby_fast_seed_enabled &&
    [[ "${NJRH_AMCL_STATIC_STANDBY_SKIP_SCAN_ADMISSION_READY_WAIT:-true}" == "true" ]]; then
    echo "[runtime-overlay] AMCL static standby fast seed: scan admission relay is running; status readiness remains asynchronous until scans are admitted" >&2
  else
    wait_for_scan_admission_status_ready || return 3
  fi
  seed_amcl_initial_pose || return 4
  if [[ "${NJRH_AMCL_READY_REQUIRE_FRESH_POSE:-true}" == "true" ]]; then
    if [[ "${NJRH_AMCL_STATIC_STANDBY_WITHOUT_POSE_OK:-true}" == "true" &&
          "${NJRH_AMCL_STATIC_STANDBY_SKIP_POSE_WAIT:-true}" == "true" ]]; then
      AMCL_STATIC_STANDBY_ACCEPTED=true
      echo "[runtime-overlay] AMCL static standby accepted immediately after seed; correction remains pending until AMCL publishes a candidate" >&2
    else
      if ! wait_for_amcl_pose_fresh_or_nomotion_update; then
        if [[ "${NJRH_AMCL_STATIC_STANDBY_WITHOUT_POSE_OK:-true}" == "true" ]]; then
          AMCL_STATIC_STANDBY_ACCEPTED=true
          echo "[runtime-overlay] AMCL static standby accepted after seed without a fresh/no-motion pose; correction remains pending until AMCL publishes a candidate" >&2
        else
          return 5
        fi
      fi
    fi
  fi
}

start_amcl_node() {
  amcl_budget_begin || return $?
  amcl_startup_side_effect_guard || return 1
  if [[ "${MODE}" == "disabled" ]]; then
    echo "[runtime-overlay] AMCL localization mode disabled; not starting AMCL" >&2
    return 0
  fi
  [[ -f "${PARAMS_FILE}" ]] || {
    echo "[runtime-overlay] AMCL params file missing: ${PARAMS_FILE}" >&2
    return 1
  }
  [[ -x "${AMCL_BIN}" ]] || {
    echo "[runtime-overlay] AMCL binary missing or not executable: ${AMCL_BIN}" >&2
    return 1
  }

  # Resume preparation only for the exact process / owner / map incarnation.
  # The public status heartbeat is not proof of lifecycle activation.
  amcl_progress_load
  AMCL_STARTUP_EPOCH_SEC="${AMCL_STARTUP_EPOCH_SEC:-$(date +%s)}"
  write_amcl_runtime_status starting false false "resident AMCL startup is in progress" ||
    echo "[runtime-overlay] AMCL starting status not submitted; continuing resident preparation without claiming READY" >&2

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
    activate_amcl_lifecycle || return $?
    return 0
  fi

  echo "[runtime-overlay] starting AMCL mode=${MODE}; params=${PARAMS_FILE}; scan_topic=$(effective_scan_topic); cpuset=${amcl_cpuset}" >&2
  amcl_startup_side_effect_guard || return 1
  : >"${LOG_FILE}"
  local amcl_affinity=(taskset -c "${amcl_cpuset}")
  if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" ]]; then
    njrh_affinity_prefix amcl_affinity amcl
  fi
  nohup "${amcl_affinity[@]}" "${AMCL_BIN}" --ros-args \
    --params-file "${PARAMS_FILE}" \
    -p "scan_topic:=$(effective_scan_topic)" \
    -p "tf_broadcast:=false" \
    -r "__node:=${AMCL_NODE_NAME}" \
    >>"${LOG_FILE}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" >"${PID_FILE}"
  amcl_budget_sleep "${NJRH_AMCL_START_SETTLE_SEC:-1}" || return 124
  if ! pid_alive "${pid}"; then
    echo "[runtime-overlay] AMCL failed to stay alive; check ${LOG_FILE}" >&2
    rm -f "${PID_FILE}"
    return 1
  fi

  amcl_progress_load
  activate_amcl_lifecycle || return $?
}

start_amcl_resident() {
  local node_rc=0
  start_amcl_node || node_rc=$?
  if [[ "${node_rc}" -ne 0 ]]; then
    if [[ "${node_rc}" -eq 26 || "${node_rc}" -eq 124 ]]; then
      finish_amcl_status starting false false "AMCL initialization pending: lifecycle query/transition incomplete" "${AMCL_EXIT_PENDING}"
      return $?
    fi
    finish_amcl_status failed false false "AMCL node failed to start or activate" "${AMCL_EXIT_LIFECYCLE_FAILED}"
    return $?
  fi
  if [[ "${MODE}" == "disabled" ]]; then
    finish_amcl_status disabled true false "" "${AMCL_EXIT_READY}" || return $?
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
  finish_amcl_status waiting_seed false false "resident AMCL started; waiting for initial pose seed" "${AMCL_EXIT_READY}" || return $?
  echo "[runtime-overlay] AMCL_RESIDENT mode=${MODE} scan_topic=$(effective_scan_topic)" >&2
}

start_amcl() {
  local node_rc=0
  start_amcl_node || node_rc=$?
  if [[ "${node_rc}" -ne 0 ]]; then
    if [[ "${ACTION}" == "complete" && ( "${node_rc}" -eq 26 || "${node_rc}" -eq 124 ) ]]; then
      finish_amcl_status starting false false "AMCL initialization pending: lifecycle query/transition incomplete" "${AMCL_EXIT_PENDING}"
      return $?
    fi
    finish_amcl_status failed false false "AMCL node failed to start or activate" "${AMCL_EXIT_LIFECYCLE_FAILED}"
    return $?
  fi
  if [[ "${MODE}" == "disabled" ]]; then
    finish_amcl_status disabled true false "" "${AMCL_EXIT_READY}" || return $?
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
      6)
        reason="/scan did not become fresh before AMCL scan admission"
        exit_code="${AMCL_EXIT_GATED_NOT_READY}"
        ;;
    esac
    if [[ "${MODE}" == "shadow" ]]; then
      amcl_warn "${reason}; continuing triggered localization baseline as visible degraded shadow mode"
      finish_amcl_status degraded false true "${reason}" "${AMCL_EXIT_DEGRADED}"
      return $?
    fi
    if [[ "${ACTION}" == "complete" ]]; then
      finish_amcl_status starting false false "AMCL initialization pending: ${reason}" "${AMCL_EXIT_PENDING}"
      return $?
    fi
    finish_amcl_status failed false false "${reason}" "${exit_code}"
    return $?
  fi
  finish_amcl_status ready true false "" "${AMCL_EXIT_READY}" || return $?
  echo "[runtime-overlay] AMCL_READY mode=${MODE} scan_topic=$(effective_scan_topic) at=$(date -u +%FT%TZ)" >&2
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
  heartbeat)
    # Resident status refresh is not part of its launcher's startup attempt.
    unset NJRH_AMCL_STARTUP_DEADLINE_MS
    heartbeat_amcl_runtime_status
    ;;
  start)
    start_amcl
    ;;
esac

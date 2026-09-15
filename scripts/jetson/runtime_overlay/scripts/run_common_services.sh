#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMON_STARTUP_EPOCH="${SECONDS}"

log_common_startup_stage() {
  local stage="$1"
  local elapsed_sec=$((SECONDS - COMMON_STARTUP_EPOCH))
  echo "[runtime-overlay] COMMON_STARTUP_STAGE stage=${stage} elapsed_sec=${elapsed_sec}" >&2
}

log_common_startup_stage "script_start"
# A new full-chain owner never joins a caller's previous startup scope.
unset NJRH_STARTUP_CPU_SESSION
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/common_startup_helpers.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/imu_pipeline_helpers.sh"
# Give unprefixed helpers the same initial mask as the five-core navigation
# children. The existing site-default startup placement is unchanged.
if [[ "${NJRH_NAVIGATION_CPU_PROFILE:-site_default}" == navigation_5cpu ]]; then
  njrh_apply_affinity_to_current_process navigation_runtime_owner
fi
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
log_common_startup_stage "helpers_loaded"
njrh_load_pointcloud_accel_profile
njrh_load_pointcloud_ingress_profile
log_common_startup_stage "profiles_loaded"

common_pids=()
export NJRH_COMMON_SERVICES_MANAGED=true
api_http_ready=0
docking_startup_done=0
common_last_started_pid=""
resident_navigation_autostart_pid=""
# A unique path avoids accepting an earlier boot's scheduling receipt.
mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
NJRH_NAVIGATION_STARTUP_RECEIPT="$(mktemp "${NJRH_RUNTIME_LOG_DIR}/navigation_startup.XXXXXX")"
runtime_health_guard_started=0
ranger_chassis_common_health_failures=0
docking_sensor_common_health_failures=0
docking_health_observer_failures=0
robot_local_state_common_health_failures=0
runtime_health_observer_failures=0
NAV_LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}"
njrh_resolve_imu_pipeline_mode "${NAV_LOCAL_STATE_MODE}"
DOCKING_SENSOR_BACKEND="${NJRH_DOCKING_SENSOR_BACKEND:-orbbec_336l}"
# FAST-LIO2 is mapping-owned by default. Daily navigation uses wheel+IMU EKF
# local odom, so common services must not keep the lidar-inertial frontend
# resident unless an explicit diagnostic FAST-LIO local-state mode is selected.
FASTLIO_AUTOSTART="${NJRH_FASTLIO_AUTOSTART:-false}"
FASTLIO_CONFIG_FILE="${NJRH_FASTLIO_CONFIG_FILE:-${NJRH_OVERLAY_ROOT}/config/fastlio.yaml}"
FASTLIO_POINTS_TOPIC="${NJRH_FASTLIO_POINTS_TOPIC:-/cloud_registered_body}"
FASTLIO_ODOM_TOPIC="${NJRH_FASTLIO_ODOM_TOPIC:-/Odometry}"
FASTLIO_TOPIC_FRESH_TIMEOUT="${NJRH_FASTLIO_TOPIC_FRESH_TIMEOUT:-8}"
FASTLIO_TOPIC_MAX_AGE_SEC="${NJRH_FASTLIO_TOPIC_MAX_AGE_SEC:-1.0}"
FASTLIO_TOPIC_MAX_FUTURE_SEC="${NJRH_FASTLIO_TOPIC_MAX_FUTURE_SEC:-0.25}"
FASTLIO_ODOM_FRESH_TIMEOUT="${NJRH_FASTLIO_ODOM_FRESH_TIMEOUT:-8}"
FASTLIO_ODOM_MAX_AGE_SEC="${NJRH_FASTLIO_ODOM_MAX_AGE_SEC:-1.0}"
FASTLIO_ODOM_MAX_FUTURE_SEC="${NJRH_FASTLIO_ODOM_MAX_FUTURE_SEC:-0.25}"
LAST_NAVIGATION_MAP_FILE="${NJRH_LAST_NAVIGATION_MAP_FILE:-${NJRH_RELEASE_ASSETS_DIR}/last_navigation_map.json}"
RESIDENT_NAVIGATION_AUTOSTART="${NJRH_RESIDENT_NAVIGATION_AUTOSTART:-auto}"
resident_navigation_autostart_selection_resolved=0
resident_navigation_autostart_started=0
autostart_building_id=""
autostart_floor_id=""
autostart_map_id=""
autostart_display_name=""

stale_amcl_heartbeat_pids() {
  ps -eo pid=,args= |
    awk '/run_amcl_shadow_localization.sh/ && /--heartbeat/ && !/awk/ {print $1}' || true
}

stale_amcl_seed_helper_pids() {
  ps -eo pid=,args= |
    grep -F "/robot_localization_bridge/seed_amcl_initial_pose" |
    grep -v "grep -F" |
    awk '{print $1}' || true
}

cleanup_stale_amcl_runtime_status_owner() {
  local pids
  pids="$(stale_amcl_heartbeat_pids)"
  if [[ -n "${pids}" ]]; then
    echo "[runtime-overlay] stopping stale AMCL runtime status heartbeat before common startup: ${pids}" >&2
    kill -INT ${pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC:-0.2}"
    pids="$(stale_amcl_heartbeat_pids)"
    [[ -z "${pids}" ]] || kill -TERM ${pids} 2>/dev/null || true
  fi
  pids="$(stale_amcl_seed_helper_pids)"
  if [[ -n "${pids}" ]]; then
    echo "[runtime-overlay] stopping stale AMCL seed helper before common startup: ${pids}" >&2
    kill -INT ${pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_SEED_HELPER_STOP_INT_WAIT_SEC:-0.2}"
    pids="$(stale_amcl_seed_helper_pids)"
    [[ -z "${pids}" ]] || kill -TERM ${pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_SEED_HELPER_STOP_TERM_WAIT_SEC:-0.2}"
    pids="$(stale_amcl_seed_helper_pids)"
    if [[ -n "${pids}" ]]; then
      echo "[runtime-overlay] killing stale AMCL seed helper before common startup: ${pids}" >&2
      kill -KILL ${pids} 2>/dev/null || true
    fi
  fi
  rm -f /tmp/njrh_amcl_runtime_status.env 2>/dev/null || true
}

cleanup_stale_amcl_runtime_status_owner

start_common_process() {
  local name="$1"
  local pattern="$2"
  shift 2
  local log_file="${NJRH_RUNTIME_LOG_DIR}/${name}.log"
  common_last_started_pid=""

  if reuse_common_services_enabled && pgrep -f "${pattern}" >/dev/null 2>&1; then
    echo "[runtime-overlay] reusing existing ${name}; pattern=${pattern}" >&2
    return 0
  fi

  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  rotate_runtime_log "${log_file}"
  echo "[runtime-overlay] starting ${name}" >&2
  local startup_affinity=()
  # Place sensor initialization as well as its eventual ROS nodes. Otherwise
  # repeated environment loading competes with Nav2 on the owner's compute pool.
  # No startup ordering, waits, process ownership or site-default changes.
  if [[ "${NJRH_NAVIGATION_CPU_PROFILE:-site_default}" == "navigation_5cpu" &&
        ( "${name}" == "jt128_driver" || "${name}" == "pointcloud_accel_pipeline" ) ]] &&
      njrh_affinity_enabled; then
    # Non-legacy pointcloud startup also initializes run_driver.sh. Only its
    # bootstrap inherits this mask; run_driver explicitly places each worker.
    njrh_affinity_prefix startup_affinity lidar_startup
  fi
  "${startup_affinity[@]}" "$@" >>"${log_file}" 2>&1 &
  local pid=$!
  common_last_started_pid="${pid}"
  common_pids+=("${pid}")
  sleep "${NJRH_COMMON_PROCESS_START_SETTLE_SEC:-0.2}"
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[runtime-overlay] common service failed to stay alive: ${name}. Check ${log_file}" >&2
    return 1
  fi
  echo "[runtime-overlay] common service ready: ${name} (pid=${pid})" >&2
}

start_orbbec_336l_depth_common() {
  local log_file="${NJRH_RUNTIME_LOG_DIR}/orbbec_336l_depth.log"
  common_last_started_pid=""
  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  rotate_runtime_log "${log_file}"
  echo "[runtime-overlay] starting orbbec_336l_depth" >&2
  # Always enter the wrapper. Only its flock, never an argv substring, owns 336L.
  bash "${SCRIPT_DIR}/run_orbbec_336l_depth.sh" >>"${log_file}" 2>&1 &
  local pid=$!
  sleep "${NJRH_COMMON_PROCESS_START_SETTLE_SEC:-0.2}"
  if kill -0 "${pid}" 2>/dev/null; then
    common_last_started_pid="${pid}"
    common_pids+=("${pid}")
    echo "[runtime-overlay] common service ready: orbbec_336l_depth (pid=${pid})" >&2
    return 0
  fi
  local status=0
  wait "${pid}" || status=$?
  if [[ "${status}" == "73" ]]; then
    echo "[runtime-overlay] orbbec_336l_depth owner lock already held; no duplicate driver started" >&2
    return 0
  fi
  echo "[runtime-overlay] common service failed to stay alive: orbbec_336l_depth (exit=${status}). Check ${log_file}" >&2
  return 1
}

canonical_jt128_ingress_running() {
  local pointcloud_pipeline_pattern="pointcloud_perception_pipeline.launch.py|component_container_mt.*pointcloud_perception_pipeline|pointcloud_perception_pipeline"
  local pointcloud_standalone_pattern="pointcloud_axis_remap|pointcloud_accel_axis"
  if [[ "${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}" == "driver_integrated" ]]; then
    pgrep -f "hesai_accel_driver_node" >/dev/null 2>&1 &&
      njrh_expected_imu_ingress_running
  else
    pgrep -f "hesai_ros_driver_node" >/dev/null 2>&1 &&
      { pgrep -f "${pointcloud_pipeline_pattern}" >/dev/null 2>&1 || pgrep -f "${pointcloud_standalone_pattern}" >/dev/null 2>&1; } &&
      njrh_expected_imu_ingress_running
  fi
}

pointcloud_accel_pipeline_aux_running() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  pgrep -f "run_pointcloud_accel_pipeline.sh|laser_scan_to_flatscan" >/dev/null 2>&1
}

process_count_for_pattern() {
  { pgrep -f "$1" 2>/dev/null || true; } | wc -l | tr -d '[:space:]'
}

query_robot_api_process_ownership() {
  local binary="${NJRH_RUNTIME_PROCESS_CHECK_BIN:-${PROJECT_ROOT}/install/robot_bringup/lib/robot_bringup/runtime_process_check}"
  local bash_executable output rc extra
  api_process_check_status=observer_unavailable
  api_supervisor_count=unknown
  api_node_count=unknown
  if [[ ! -x "${binary}" ]]; then
    echo "[runtime-overlay] API ownership observer unavailable: missing native checker ${binary}" >&2
    return 40
  fi
  bash_executable="$(command -v bash)" || return 40
  # One native scan returns both counts. Never fall back to per-PID shell scans.
  if output="$("${binary}" \
    --supervisor-exe "${bash_executable}" \
    --supervisor-arg "${SCRIPT_DIR}/run_robot_api_server_supervised.sh" \
    --api-exe "${PROJECT_ROOT}/install/robot_api_server/lib/robot_api_server/robot_api_server_node")"; then
    rc=0
  else
    rc=$?
  fi
  local status supervisor_count node_count
  # Accept a single bounded numeric record, not shell code or partial output.
  if [[ "${output}" != *$'\n'* ]] &&
      read -r status supervisor_count node_count extra <<<"${output}" &&
      [[ -z "${extra}" && "${supervisor_count}" =~ ^(0|[1-9][0-9]{0,9})$ &&
        "${node_count}" =~ ^(0|[1-9][0-9]{0,9})$ ]]; then
    if { [[ "${rc}" == 0 && "${status}" == unique &&
             "${supervisor_count}" == 1 && "${node_count}" == 1 ]]; } ||
       { [[ "${rc}" == 50 && "${status}" == ownership_fault &&
             ( "${supervisor_count}" != 1 || "${node_count}" != 1 ) ]]; }; then
      api_process_check_status="${status}"
      api_supervisor_count="${supervisor_count}"
      api_node_count="${node_count}"
      return "${rc}"
    fi
  fi
  case "${rc}:${output}" in
    '40:observer_unavailable - -') return 40 ;;
    '42:observer_transient - -') api_process_check_status=observer_transient; return 42 ;;
  esac
  api_process_check_status=observer_error
  return 41
}

pointcloud_accel_pipeline_aux_unique() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  local pipeline_count
  local flatscan_count
  pipeline_count="$(process_count_for_pattern "[r]un_pointcloud_accel_pipeline.sh")"
  flatscan_count="$(process_count_for_pattern "[l]aser_scan_to_flatscan")"
  [[ "${pipeline_count:-0}" -eq 1 && "${flatscan_count:-0}" -eq 1 ]]
}

pointcloud_accel_pipeline_aux_complete() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  pointcloud_accel_pipeline_aux_running && pointcloud_accel_pipeline_aux_unique
}

canonical_jt128_runtime_complete() {
  canonical_jt128_ingress_running && pointcloud_accel_pipeline_aux_complete
}

start_runtime_health_guard_common() {
  if [[ "${runtime_health_guard_started}" -eq 1 ]]; then
    return 0
  fi
  local health_file
  health_file="$(runtime_health_file)"
  rm -f "${health_file}" 2>/dev/null || true
  start_common_process "runtime_health_guard" "runtime_health_guard|run_runtime_health_guard.sh" \
    bash "${SCRIPT_DIR}/run_runtime_health_guard.sh"
  runtime_health_guard_started=1
}

wait_for_runtime_health_local_state_ready() {
  local timeout_sec="${NJRH_RUNTIME_HEALTH_LOCAL_STATE_READY_TIMEOUT_SEC:-12}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if runtime_health_check "local_state_ready" >/dev/null 2>&1; then
      echo "[runtime-overlay] runtime health confirms local_state_ready before resident navigation autostart" >&2
      return 0
    fi
    if runtime_health_check "local_state_topic_ready" >/dev/null 2>&1; then
      echo "[runtime-overlay] runtime health confirms local_state_topic_ready before resident navigation autostart; TF freshness was already checked by direct readiness" >&2
      return 0
    fi
    if runtime_health_check "local_state_endpoint" >/dev/null 2>&1; then
      echo "[runtime-overlay] runtime health confirms local_state_endpoint before resident navigation autostart; freshness was already checked by direct readiness" >&2
      return 0
    fi
    sleep 0.2
  done
  echo "[runtime-overlay] runtime health did not confirm local_state_ready within ${timeout_sec}s; continuing because robot_local_state direct readiness already passed" >&2
  return 0
}

wait_for_runtime_health_local_state_endpoint_ready() {
  local timeout_sec="${NJRH_RUNTIME_HEALTH_LOCAL_STATE_ENDPOINT_TIMEOUT_SEC:-0}"
  local deadline=$((SECONDS + timeout_sec))
  local key="local_state_endpoint"
  if [[ "${NAV_LOCAL_STATE_MODE}" == "fastlio" ]]; then
    key="local_state_fastlio_endpoint"
  fi
  while (( SECONDS < deadline )); do
    if runtime_health_check "${key}" >/dev/null 2>&1; then
      echo "[runtime-overlay] runtime health confirms ${key} before resident navigation autostart" >&2
      return 0
    fi
    sleep 0.2
  done
  echo "[runtime-overlay] runtime health did not confirm ${key} within ${timeout_sec}s; continuing because robot_local_state endpoint direct readiness already passed" >&2
  return 0
}

direct_local_state_odom_ready_for_health_confirmation() {
  local timeout_sec="${NJRH_COMMON_LOCAL_STATE_DIRECT_CONFIRM_TIMEOUT_SEC:-3}"
  local max_age_sec="${NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC:-0.75}"
  local max_future_sec="${NJRH_COMMON_LOCAL_STATE_DIRECT_CONFIRM_MAX_FUTURE_SEC:-0.25}"
  runtime_readiness_probe fresh-header-topic \
    "/local_state/odometry" \
    "${timeout_sec}" \
    "${max_age_sec}" \
    "${max_future_sec}"
}

verify_ranger_chassis_common_health_or_exit() {
  [[ "${NJRH_COMMON_RANGER_CHASSIS_HEALTH_MONITOR:-false}" == "true" ]] || return 0
  local timeout_sec="${NJRH_COMMON_RANGER_CHASSIS_HEALTH_TIMEOUT_SEC:-3}"
  local max_failures="${NJRH_COMMON_RANGER_CHASSIS_HEALTH_MAX_FAILURES:-5}"
  if ranger_chassis_liveness_ready "${timeout_sec}" >/dev/null 2>&1; then
    ranger_chassis_common_health_failures=0
    return 0
  fi
  ranger_chassis_common_health_failures=$((ranger_chassis_common_health_failures + 1))
  if (( ranger_chassis_common_health_failures < max_failures )); then
    echo "[runtime-overlay] ranger_chassis_common liveness degraded (${ranger_chassis_common_health_failures}/${max_failures}); waiting before next health check" >&2
    return 0
  fi
  if [[ "${NJRH_COMMON_RANGER_CHASSIS_HEALTH_EXIT_ON_LOSS:-false}" != "true" ]]; then
    echo "[runtime-overlay] ranger_chassis_common health lost after ${ranger_chassis_common_health_failures} consecutive checks; continuing because NJRH_COMMON_RANGER_CHASSIS_HEALTH_EXIT_ON_LOSS=false" >&2
    ranger_chassis_common_health_failures=0
    return 0
  fi
  echo "[runtime-overlay] ranger_chassis_common health lost after ${ranger_chassis_common_health_failures} consecutive checks; exiting common runtime so systemd restarts the complete navigation chain" >&2
  return 1
}

verify_robot_local_state_common_health_or_exit() {
  [[ "${NJRH_COMMON_LOCAL_STATE_HEALTH_MONITOR:-true}" == "true" ]] || return 0
  local max_failures="${NJRH_COMMON_LOCAL_STATE_HEALTH_MAX_FAILURES:-3}"
  local diagnostic=""
  local diagnostic_rc=0
  local required_processes="alive"

  if diagnostic="$(runtime_health_local_state_diagnostic 2>&1)"; then
    runtime_health_observer_failures=0
    robot_local_state_common_health_failures=0
    return 0
  else
    diagnostic_rc=$?
  fi

  case "${diagnostic_rc}" in
    40|41|42|43|44|45|46)
      runtime_health_observer_failures=$((runtime_health_observer_failures + 1))
      robot_local_state_common_health_failures=0
      echo "[runtime-overlay] runtime health observer degraded (${runtime_health_observer_failures}); does not authorize complete-chain recovery; diagnostic=${diagnostic}" >&2
      return 0
      ;;
    50|51|52|53|54|55|56)
      runtime_health_observer_failures=0
      ;;
    *)
      runtime_health_observer_failures=$((runtime_health_observer_failures + 1))
      robot_local_state_common_health_failures=0
      echo "[runtime-overlay] runtime health observer classifier failed rc=${diagnostic_rc} (${runtime_health_observer_failures}); does not authorize complete-chain recovery; diagnostic=${diagnostic}" >&2
      return 0
      ;;
  esac

  # A retry may read the same atomic snapshot. It is not independent evidence.
  local evidence_generation=""
  local evidence_sequence=""
  if [[ ! "${diagnostic}" =~ evidence_id=([a-zA-Z0-9._-]+):([0-9]{1,16})([[:space:]]|$) ]]; then
    robot_local_state_common_health_failures=0
    echo "[runtime-overlay] runtime health observer lacks sequenced fault evidence; does not authorize complete-chain recovery" >&2
    return 0
  fi
  evidence_generation="${BASH_REMATCH[1]}"
  evidence_sequence="$((10#${BASH_REMATCH[2]}))"
  if [[ "${evidence_generation}" != "${robot_local_state_fault_generation:-}" ]]; then
    robot_local_state_common_health_failures=0
    robot_local_state_fault_generation="${evidence_generation}"
    robot_local_state_fault_sequence=0
  fi
  if (( evidence_sequence <= ${robot_local_state_fault_sequence:-0} )); then
    return 0
  fi
  robot_local_state_fault_sequence="${evidence_sequence}"

  if ! local_state_required_processes_running; then
    required_processes="missing"
  fi
  if direct_local_state_odom_ready_for_health_confirmation; then
    runtime_health_observer_failures=$((runtime_health_observer_failures + 1))
    robot_local_state_common_health_failures=0
    echo "[runtime-overlay] runtime health observer contradicted by independent fresh local odom (${runtime_health_observer_failures}); does not authorize complete-chain recovery; diagnostic=${diagnostic}" >&2
    return 0
  fi
  robot_local_state_common_health_failures=$((robot_local_state_common_health_failures + 1))
  if (( robot_local_state_common_health_failures < max_failures )); then
    echo "[runtime-overlay] robot_local_state fresh-snapshot fault candidate (${robot_local_state_common_health_failures}/${max_failures}); required_processes=${required_processes}; diagnostic=${diagnostic}" >&2
    return 0
  fi
  echo "[runtime-overlay] robot_local_state fault confirmed by ${robot_local_state_common_health_failures} consecutive fresh snapshots; required_processes=${required_processes}; diagnostic=${diagnostic}; exiting common owner so systemd restarts the complete navigation chain" >&2
  return 1
}

verify_robot_api_server_common_health_or_exit() {
  local rc
  if query_robot_api_process_ownership; then
    return 0
  else
    rc=$?
  fi
  if [[ "${rc}" != 50 ]]; then
    echo "[runtime-overlay] API ownership ${api_process_check_status}; does not authorize complete-chain recovery" >&2
    return 0
  fi
  echo "[runtime-overlay] robot_api_server ownership lost or non-unique (supervisor=${api_supervisor_count}, node=${api_node_count}); exiting common owner so systemd restarts the complete runtime chain" >&2
  return 1
}

wait_for_robot_api_server_common_ready() {
  local timeout_sec="${NJRH_ROBOT_API_PROCESS_READY_TIMEOUT_SEC:-120}"
  local poll_sec="${NJRH_ROBOT_API_PROCESS_READY_POLL_SEC:-1}"
  local deadline
  api_process_check_status=observer_unavailable
  api_supervisor_count=unknown
  api_node_count=unknown
  if [[ ! "${timeout_sec}" =~ ^[0-9]+$ || "${timeout_sec}" -le 0 ]]; then
    timeout_sec=120
  fi
  deadline=$((SECONDS + timeout_sec))
  echo "[runtime-overlay] waiting up to ${timeout_sec}s for exact robot_api_server supervisor and node ownership" >&2
  while [[ "${SECONDS}" -lt "${deadline}" ]]; do
    if query_robot_api_process_ownership; then
      echo "[runtime-overlay] exact robot_api_server ownership ready (supervisor=1, node=1)" >&2
      return 0
    fi
    sleep "${poll_sec}"
  done
  echo "[runtime-overlay] robot_api_server did not establish exact ownership within ${timeout_sec}s (status=${api_process_check_status}, supervisor=${api_supervisor_count}, node=${api_node_count})" >&2
  return 1
}

verify_docking_sensor_common_health_or_exit() {
  [[ "${DOCKING_SENSOR_BACKEND}" == "orbbec_336l" ]] || return 0
  [[ "${NJRH_COMMON_DOCKING_SENSOR_HEALTH_MONITOR:-true}" == "true" ]] || return 0
  local max_failures="${NJRH_COMMON_DOCKING_SENSOR_HEALTH_MAX_FAILURES:-3}"
  if ! runtime_health_available; then
    docking_health_observer_failures=$((docking_health_observer_failures + 1))
    docking_sensor_common_health_failures=0
    echo "[runtime-overlay] docking health observer unavailable/stale (${docking_health_observer_failures}); does not authorize complete-chain recovery" \
      >&2
    return 0
  fi
  docking_health_observer_failures=0
  if runtime_health_check "docking_sensor_healthy" >/dev/null 2>&1; then
    docking_sensor_common_health_failures=0
    return 0
  fi
  if (( docking_sensor_common_health_failures < max_failures )); then
    docking_sensor_common_health_failures=$((docking_sensor_common_health_failures + 1))
    if (( docking_sensor_common_health_failures < max_failures )); then
      echo "[runtime-overlay] Orbbec docking stream health degraded (${docking_sensor_common_health_failures}/${max_failures}); docking manager remains fail-closed; common runtime stays alive" >&2
    else
      echo "[runtime-overlay] Orbbec docking stream unhealthy after ${docking_sensor_common_health_failures} consecutive checks; docking manager remains fail-closed; common runtime stays alive" >&2
    fi
  fi
  return 0
}

start_robot_local_state_common() {
  NJRH_LOCAL_STATE_START_READY_MODE="${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}" \
  start_canonical_helper \
    "robot_local_state_common" \
    env NJRH_LOCAL_STATE_START_READY_MODE="${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}" \
      LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" \
      bash "${SCRIPT_DIR}/run_local_state.sh"
}

can_start_robot_local_state_common_in_background() {
  [[ "${NJRH_COMMON_LOCAL_STATE_BACKGROUND_START:-true}" == "true" ]] || return 1
  [[ "${NAV_LOCAL_STATE_MODE}" != "fastlio" ]] || return 1
}

robot_local_state_common_background_pid=""
start_robot_local_state_common_background_if_enabled() {
  if ! can_start_robot_local_state_common_in_background; then
    return 0
  fi
  echo "[runtime-overlay] starting robot_local_state_common in background while common sensors initialize" >&2
  NJRH_LOCAL_STATE_START_READY_MODE="${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}" \
    start_common_canonical_helper_background "robot_local_state_common" \
      env NJRH_LOCAL_STATE_START_READY_MODE="${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}" \
        LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" \
        bash "${SCRIPT_DIR}/run_local_state.sh"
  robot_local_state_common_background_pid="${common_startup_waiters[robot_local_state_common]:-reused}"
}

wait_for_robot_local_state_common_background_if_started() {
  if [[ -z "${robot_local_state_common_background_pid}" ]]; then
    start_robot_local_state_common
    return $?
  fi
  if wait_for_common_startup_job "robot_local_state_common"; then
    robot_local_state_common_background_pid=""
    return 0
  fi
  robot_local_state_common_background_pid=""
  echo "[runtime-overlay] robot_local_state_common background startup failed" >&2
  return 1
}

resident_navigation_context_status() {
  python3 - <<'PY'
import json
import pathlib
import sys

path = pathlib.Path("/tmp/njrh_runtime_map_context.json")
if not path.exists():
    print("missing runtime map context")
    raise SystemExit(2)
try:
    data = json.loads(path.read_text())
except Exception as exc:
    print(f"invalid runtime map context: {exc}")
    raise SystemExit(2)

state = data.get("state", "")
confirmed = bool(data.get("confirmed", False))
stage = data.get("startup_stage", "")
elapsed = data.get("startup_elapsed_sec", "")
message = data.get("message", "")
map_id = data.get("map_id", "")
if state == "ready" and confirmed:
    try:
        import urllib.request

        with urllib.request.urlopen("http://127.0.0.1:8080/api/v1/status", timeout=1.0) as response:
            api = json.loads(response.read().decode("utf-8"))
        localization = api.get("localization", {}) if isinstance(api.get("localization"), dict) else {}
        safe = bool(localization.get("safe_for_goal_start", api.get("safe_for_goal_start", False)))
        amcl_stale = bool(localization.get("amcl_status_file_stale", False))
        amcl_tracking = localization.get("amcl_tracking_ready", api.get("amcl_tracking_ready", None))
        goal_detail = localization.get("goal_start_detail", "")
        if not safe:
            print(
                "context ready but API goal-start not safe: "
                f"safe_for_goal_start={safe} amcl_tracking_ready={amcl_tracking} "
                f"amcl_status_file_stale={amcl_stale} detail={goal_detail}"
            )
            raise SystemExit(2)
    except SystemExit:
        raise
    except Exception as exc:
        print(f"context ready but API status unavailable: {exc}")
        raise SystemExit(2)
    print(f"ready startup_elapsed_sec={elapsed} stage={stage} map_id={map_id}")
    raise SystemExit(0)
if state == "failed":
    print(f"failed startup_elapsed_sec={elapsed} stage={stage} message={message}")
    raise SystemExit(3)
print(f"starting state={state} confirmed={confirmed} startup_elapsed_sec={elapsed} stage={stage} message={message}")
raise SystemExit(2)
PY
}

wait_for_resident_navigation_context_ready() {
  local timeout_sec="${NJRH_RESIDENT_NAVIGATION_READY_TIMEOUT_SEC:-120}"
  local hard_timeout_sec="${NJRH_RESIDENT_NAVIGATION_READY_HARD_TIMEOUT_SEC:-240}"
  if (( hard_timeout_sec < timeout_sec )); then
    hard_timeout_sec="${timeout_sec}"
  fi
  local deadline=$((SECONDS + timeout_sec))
  local hard_deadline=$((SECONDS + hard_timeout_sec))
  local soft_timeout_reported=0
  local status
  local rc
  while (( SECONDS < hard_deadline )); do
    status="$(resident_navigation_context_status 2>&1)" && {
      echo "[runtime-overlay] resident navigation context ready: ${status}" >&2
      return 0
    }
    rc=$?
    if [[ "${rc}" -eq 3 ]]; then
      echo "[runtime-overlay] resident navigation context failed: ${status}" >&2
      return 1
    fi
    if (( soft_timeout_reported == 0 && SECONDS >= deadline )); then
      echo "[runtime-overlay] resident navigation context exceeded soft SLA ${timeout_sec}s but is still starting: ${status}" >&2
      soft_timeout_reported=1
    fi
    sleep "${NJRH_RESIDENT_NAVIGATION_READY_POLL_SEC:-1}"
  done
  status="$(resident_navigation_context_status 2>&1)" && {
    echo "[runtime-overlay] resident navigation context ready: ${status}" >&2
    return 0
  }
  echo "[runtime-overlay] resident navigation context did not become ready within hard timeout ${hard_timeout_sec}s: ${status}" >&2
  return 1
}

resident_navigation_runtime_process_running() {
  pgrep -f "[r]un_navigation_runtime_services.sh" >/dev/null 2>&1
}

resident_navigation_runtime_pids() {
  ps -eo pid=,args= |
    awk '/run_navigation_runtime_services.sh/ && !/awk/ {print $1}' || true
}

resident_navigation_layer_pids() {
  local pattern
  pattern="run_navigation_runtime_services.sh|nav2_lifecycle_sequence.py|call_global_localization_trigger.py|run_nav2_navigation.sh|run_occupancy_grid_localization.sh|standard_navigation.launch.py|occupancy_localization_stack.launch.py|occupancy_grid_localizer_container|occupancy_grid_localizer|robot_localization_bridge/localization_bridge_node|localization_bridge_node --ros-args|amcl --ros-args|nav2_amcl|amcl_scan_admission|__node:=map_server|__node:=controller_server|__node:=planner_server|__node:=bt_navigator|__node:=behavior_server|__node:=velocity_smoother|__node:=collision_monitor|__node:=lifecycle_manager_navigation|__node:=lifecycle_manager_costmap_filters"
  ps -eo pid=,args= |
    awk -v pattern="${pattern}" '$0 ~ pattern && $0 !~ /awk/ {print $1}' || true
}

resident_navigation_layers_running() {
  [[ -n "$(resident_navigation_layer_pids)" ]]
}

stop_stale_resident_navigation_runtime_processes() {
  local pids
  pids="$(resident_navigation_runtime_pids)"
  if [[ -n "${pids}" ]]; then
    echo "[runtime-overlay] stopping stale resident navigation runtime processes: ${pids}" >&2
    kill -INT ${pids} 2>/dev/null || true
    sleep "${NJRH_RESIDENT_NAVIGATION_STOP_INT_WAIT_SEC:-0.5}"
    pids="$(resident_navigation_runtime_pids)"
    [[ -z "${pids}" ]] || kill -TERM ${pids} 2>/dev/null || true
    sleep "${NJRH_RESIDENT_NAVIGATION_STOP_TERM_WAIT_SEC:-0.5}"
    pids="$(resident_navigation_runtime_pids)"
    if [[ -n "${pids}" ]]; then
      echo "[runtime-overlay] stale resident navigation runtime ignored SIGTERM; killing exact pids: ${pids}" >&2
      kill -KILL ${pids} 2>/dev/null || true
      sleep "${NJRH_RESIDENT_NAVIGATION_STOP_KILL_WAIT_SEC:-0.2}"
    fi
  fi
}

cleanup_resident_navigation_runtime_layers() {
  stop_stale_resident_navigation_runtime_processes
  if ! resident_navigation_layers_running; then
    echo "[runtime-overlay] no stale resident navigation layers found; skipping Nav2/localization/AMCL cleanup sweep" >&2
    rm -f /tmp/njrh_runtime_map_context.json /tmp/njrh_amcl_runtime_status.env 2>/dev/null || true
    return 0
  fi
  timeout --kill-after="${NJRH_COMMON_AMCL_STOP_KILL_AFTER_SEC:-1}" \
    "${NJRH_COMMON_AMCL_STOP_TIMEOUT_SEC:-1.5}" \
    env \
      NJRH_AMCL_LIFECYCLE_SHUTDOWN_TIMEOUT_SEC="${NJRH_AMCL_LIFECYCLE_SHUTDOWN_TIMEOUT_SEC:-1}" \
      NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC="${NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC:-0.1}" \
      NJRH_AMCL_RUNNER_STOP_INT_WAIT_SEC="${NJRH_AMCL_RUNNER_STOP_INT_WAIT_SEC:-0.1}" \
      NJRH_AMCL_RUNNER_STOP_TERM_WAIT_SEC="${NJRH_AMCL_RUNNER_STOP_TERM_WAIT_SEC:-0.1}" \
      NJRH_AMCL_RUNNER_STOP_KILL_WAIT_SEC="${NJRH_AMCL_RUNNER_STOP_KILL_WAIT_SEC:-0.1}" \
      NJRH_AMCL_SCAN_ADMISSION_STOP_INT_WAIT_SEC="${NJRH_AMCL_SCAN_ADMISSION_STOP_INT_WAIT_SEC:-0.1}" \
      NJRH_AMCL_STOP_INT_WAIT_SEC="${NJRH_AMCL_STOP_INT_WAIT_SEC:-0.1}" \
      bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --stop >/dev/null 2>&1 || true
  NJRH_STANDARD_NAV_STACK_STOP_INT_WAIT_SEC="${NJRH_STANDARD_NAV_STACK_STOP_INT_WAIT_SEC:-0.2}" \
    stop_existing_standard_nav_stack || true
  NJRH_LOCALIZATION_STACK_STOP_WAIT_SEC="${NJRH_LOCALIZATION_STACK_STOP_WAIT_SEC:-0.2}" \
    stop_existing_localization_stack || true
  rm -f /tmp/njrh_runtime_map_context.json /tmp/njrh_amcl_runtime_status.env 2>/dev/null || true
}

prepare_resident_navigation_autostart() {
  if resident_navigation_runtime_process_running && resident_navigation_context_status >/dev/null 2>&1; then
    echo "[runtime-overlay] reusing healthy resident navigation runtime" >&2
    return 0
  fi

  echo "[runtime-overlay] clearing stale resident navigation runtime before autostart" >&2
  cleanup_resident_navigation_runtime_layers
}

resolve_resident_navigation_autostart_selection() {
  [[ "${resident_navigation_autostart_selection_resolved}" -eq 0 ]] || return 0
  resident_navigation_autostart_selection_resolved=1
  autostart_building_id=""
  autostart_floor_id=""
  autostart_map_id=""
  autostart_display_name=""

  if [[ "${RESIDENT_NAVIGATION_AUTOSTART}" == "true" && -n "${NJRH_FLOOR_ID:-}" ]]; then
    autostart_building_id="${NJRH_BUILDING_ID:-building_1}"
    autostart_floor_id="${NJRH_FLOOR_ID}"
    echo "[runtime-overlay] resident navigation autostart uses explicit floor ${autostart_building_id}/${autostart_floor_id}" >&2
    return 0
  fi

  if selection="$(load_last_navigation_map_selection)"; then
    IFS=$'\t' read -r autostart_building_id autostart_floor_id autostart_map_id autostart_display_name <<<"${selection}"
    echo "[runtime-overlay] resident navigation autostart selected last map ${autostart_building_id}/${autostart_floor_id}/${autostart_map_id}" >&2
  else
    echo "[runtime-overlay] no valid last navigation map; common services stay alive in NO_MAP mode" >&2
  fi
}

start_resident_navigation_autostart_if_selected() {
  [[ "${RESIDENT_NAVIGATION_AUTOSTART}" != "false" ]] || return 0
  [[ "${resident_navigation_autostart_started}" -eq 0 ]] || return 0
  resolve_resident_navigation_autostart_selection
  [[ -n "${autostart_floor_id}" ]] || return 0
  resolve_floor_assets_if_needed "${autostart_building_id}" "${autostart_floor_id}"
  autostart_map_id="${NJRH_NAV_MAP_ID:-${autostart_map_id}}"
  autostart_display_name="${NJRH_NAV_MAP_NAME:-${autostart_display_name}}"

  prepare_resident_navigation_autostart
  if [[ "${NJRH_COMMON_SERVICES_MANAGED:-false}" != "true" ]] && common_require_flatscan_before_resident_autostart; then
    ensure_flatscan_ready_before_navigation_autostart || return 1
  else
    echo "[runtime-overlay] skipping common /flatscan precheck before resident navigation autostart; resident localization owns /flatscan readiness and repair gates" >&2
  fi
  start_common_process "resident_navigation_runtime" "run_navigation_runtime_services.sh" \
    env \
      NJRH_NAVIGATION_STARTUP_RECEIPT="${NJRH_NAVIGATION_STARTUP_RECEIPT}" \
      NJRH_NAVIGATION_START_SOURCE="systemd_autostart" \
      NJRH_NAVIGATION_RESUME_LOG_FILE="${NJRH_RUNTIME_LOG_DIR}/resident_navigation_runtime.log" \
      NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION="${NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-true}" \
      NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE="${NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE:-true}" \
      NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START="${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}" \
      NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK="${NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false}" \
      NJRH_NAV2_LIFECYCLE_PARALLEL_CORE="${NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false}" \
      NJRH_MAP_ID="${autostart_map_id}" \
      NJRH_MAP_DISPLAY_NAME="${autostart_display_name}" \
      NJRH_MAP_CONTEXT_BUILDING_ID="${autostart_building_id}" \
      NJRH_MAP_CONTEXT_FLOOR_ID="${autostart_floor_id}" \
      bash "${SCRIPT_DIR}/run_navigation_runtime_services.sh" "${autostart_building_id}" "${autostart_floor_id}"
  resident_navigation_autostart_started=1
  resident_navigation_autostart_pid="${common_last_started_pid}"
}

wait_for_resident_navigation_autostart_if_started() {
  [[ "${resident_navigation_autostart_started}" -eq 1 ]] || return 0
  echo "[runtime-overlay] resident navigation autostart launched; common services will not block on navigation readiness" >&2
  return 0
}

stop_stale_pointcloud_accel_pipeline_processes() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  local pipeline_count
  local flatscan_count
  pipeline_count="$(process_count_for_pattern "[r]un_pointcloud_accel_pipeline.sh")"
  flatscan_count="$(process_count_for_pattern "[l]aser_scan_to_flatscan")"
  if [[ "${pipeline_count:-0}" -eq 0 && "${flatscan_count:-0}" -eq 0 ]]; then
    return 0
  fi
  echo "[runtime-overlay] stopping stale pointcloud accel pipeline before restart: run_pointcloud_accel_pipeline=${pipeline_count:-0} laser_scan_to_flatscan=${flatscan_count:-0}" >&2
  local patterns=(
    "[r]un_pointcloud_accel_pipeline.sh"
    "[h]esai_ros_driver_node"
    "[p]ointcloud_accel_axis_node"
    "[h]esai_accel_driver_node"
    "[j]t128_accel_driver_node"
    "[p]ointcloud_axis_remap_node"
    "[p]ointcloud_axis_remap"
    "[n]av_cloud_preprocessor"
    "[p]ointcloud_to_laserscan_node"
    "[p]ointcloud_to_laserscan"
    "[s]can_republisher_node"
    "[l]aser_scan_to_flatscan"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -INT -f "${pattern}" 2>/dev/null || true
  done
  sleep "${NJRH_POINTCLOUD_ACCEL_STOP_INT_WAIT_SEC:-1}"
  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep "${NJRH_POINTCLOUD_ACCEL_STOP_TERM_WAIT_SEC:-1}"
  for pattern in "${patterns[@]}"; do
    pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
    if [[ -n "${pids}" ]]; then
      echo "[runtime-overlay] stale pointcloud accel process ignored SIGTERM; killing exact pids for pattern=${pattern}: ${pids}" >&2
      kill -KILL ${pids} 2>/dev/null || true
    fi
  done
  sleep "${NJRH_POINTCLOUD_ACCEL_STOP_KILL_WAIT_SEC:-0.2}"
}

current_pointcloud_accel_profile_for_common() {
  local profile="${NJRH_POINTCLOUD_ACCEL_PROFILE:-ipc_worker}"
  if [[ -f "${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env" ]]; then
    # shellcheck source=../config/pointcloud_accel_profile.env
    source "${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env"
    profile="${NJRH_POINTCLOUD_ACCEL_PROFILE:-${profile}}"
  fi
  printf '%s\n' "${profile}"
}

ensure_flatscan_ready_before_navigation_autostart() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  local timeout_sec="${NJRH_COMMON_FLATSCAN_READY_TIMEOUT_SEC:-45}"
  local repair_timeout_sec="${NJRH_COMMON_FLATSCAN_REPAIR_TIMEOUT_SEC:-60}"
  local profile

  if wait_for_topic_publisher_from_node "/flatscan" "laser_scan_to_flatscan" "${timeout_sec}"; then
    echo "[runtime-overlay] /flatscan publisher ready before resident navigation autostart" >&2
    return 0
  fi

  profile="$(current_pointcloud_accel_profile_for_common)"
  case "${profile}" in
    ipc_worker|driver_integrated|split_local_nav|local_priority)
      ;;
    *)
      echo "[runtime-overlay] /flatscan missing before resident navigation and pointcloud profile is invalid: ${profile}" >&2
      return 1
      ;;
  esac

  echo "[runtime-overlay] /flatscan missing before resident navigation; restarting pointcloud accel profile=${profile}" >&2
  bash "${SCRIPT_DIR}/set_pointcloud_accel_profile.sh" --profile "${profile}" --restart >&2 || return 1
  wait_for_topic_publisher_from_node "/flatscan" "laser_scan_to_flatscan" "${repair_timeout_sec}" || {
    echo "[runtime-overlay] /flatscan publisher did not recover within ${repair_timeout_sec}s after pointcloud accel restart" >&2
    return 1
  }
  echo "[runtime-overlay] /flatscan publisher recovered before resident navigation autostart" >&2
}

common_require_flatscan_before_resident_autostart() {
  case "${NJRH_COMMON_REQUIRE_FLATSCAN_BEFORE_RESIDENT_AUTOSTART:-false}" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

fastlio_runtime_running() {
  pgrep -f "ros2 run fast_lio fastlio_mapping|fast_lio fastlio_mapping|laser_mapping" >/dev/null 2>&1
}

fastlio_runtime_output_fresh() {
  fastlio_runtime_running || return 1
  runtime_readiness_probe \
    fresh-header-topic \
    "${FASTLIO_ODOM_TOPIC}" \
    "${FASTLIO_ODOM_FRESH_TIMEOUT}" \
    "${FASTLIO_ODOM_MAX_AGE_SEC}" \
    "${FASTLIO_ODOM_MAX_FUTURE_SEC}" >/dev/null 2>&1
}

wait_for_fastlio_runtime_output() {
  runtime_readiness_probe \
    fresh-header-topic \
    "${FASTLIO_ODOM_TOPIC}" \
    "${FASTLIO_ODOM_FRESH_TIMEOUT}" \
    "${FASTLIO_ODOM_MAX_AGE_SEC}" \
    "${FASTLIO_ODOM_MAX_FUTURE_SEC}"
}

stop_fastlio_runtime_processes() {
  local patterns=(
    "ros2 run fast_lio fastlio_mapping"
    "fast_lio/lib/fast_lio/fastlio_mapping"
    "fast_lio/fastlio_mapping"
    "fastlio_mapping --ros-args"
    "laser_mapping"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -INT -f "${pattern}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_INT_WAIT_SEC:-1}"
  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_TERM_WAIT_SEC:-1}"
  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
}

fastlio_pid_is_mapping_owned() {
  local pid="$1"
  [[ -r "/proc/${pid}/environ" ]] || return 1
  tr '\0' '\n' <"/proc/${pid}/environ" | grep -qx "NJRH_SLAM2D_PRIVATE_FASTLIO=1"
}

stop_non_mapping_fastlio_runtime_processes() {
  local pattern="ros2 run fast_lio fastlio_mapping|fast_lio/lib/fast_lio/fastlio_mapping|fast_lio/fastlio_mapping|fastlio_mapping --ros-args|laser_mapping"
  local candidates=()
  local pids=()
  local cmdline pid
  mapfile -t candidates < <(pgrep -f "${pattern}" 2>/dev/null || true)
  for pid in "${candidates[@]}"; do
    [[ -r "/proc/${pid}/cmdline" ]] || continue
    cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
    grep -Eq "${pattern}" <<<"${cmdline}" || continue
    fastlio_pid_is_mapping_owned "${pid}" && continue
    pids+=("${pid}")
  done
  [[ ${#pids[@]} -gt 0 ]] || return 0
  echo "[runtime-overlay] FAST-LIO2 common autostart disabled; stopping non-mapping FAST-LIO leftovers: ${pids[*]}" >&2
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_INT_WAIT_SEC:-1}"
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_TERM_WAIT_SEC:-1}"
  for pid in "${pids[@]}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
}

start_fastlio_common() {
  [[ -f "${FASTLIO_CONFIG_FILE}" ]] || {
    echo "[runtime-overlay] missing FAST-LIO runtime file: ${FASTLIO_CONFIG_FILE}" >&2
    return 1
  }

  if reuse_common_services_enabled && fastlio_runtime_running; then
    if fastlio_runtime_output_fresh; then
      echo "[runtime-overlay] reusing existing fastlio_mapping common runtime; ${FASTLIO_ODOM_TOPIC} is fresh" >&2
      return 0
    fi
    echo "[runtime-overlay] existing fastlio_mapping process has stale/missing ${FASTLIO_ODOM_TOPIC}; restarting FAST-LIO" >&2
    stop_fastlio_runtime_processes
  fi

  if reuse_common_services_enabled && fastlio_runtime_running; then
    echo "[runtime-overlay] reusing existing fastlio_mapping common runtime after stale-output cleanup" >&2
  else
    start_common_process "fastlio_mapping" "ros2 run fast_lio fastlio_mapping|fast_lio fastlio_mapping|laser_mapping" \
      njrh_run_affined fastlio_mapping ros2 run fast_lio fastlio_mapping \
        --ros-args \
        --params-file "${FASTLIO_CONFIG_FILE}" \
        -p use_sim_time:=false \
          -r /tf:=/tf_fastlio_internal \
          -r /tf_static:=/tf_static_fastlio_internal
  fi

  if ! wait_for_fastlio_runtime_output; then
    echo "[runtime-overlay] FAST-LIO failed to publish fresh ${FASTLIO_ODOM_TOPIC}; stopping stale runtime" >&2
    stop_fastlio_runtime_processes
    return 1
  fi
}

load_last_navigation_map_selection() {
  [[ -f "${LAST_NAVIGATION_MAP_FILE}" ]] || return 1
  python3 - "${LAST_NAVIGATION_MAP_FILE}" "${NJRH_RELEASE_ASSETS_DIR}" <<'PY'
import json
import re
import sys
from pathlib import Path

state_path = Path(sys.argv[1])
maps_root = Path(sys.argv[2])
safe = re.compile(r"^[A-Za-z0-9_.-]+$")

try:
    data = json.loads(state_path.read_text(encoding="utf-8"))
except Exception as exc:
    print(f"[runtime-overlay] cannot read last navigation map file {state_path}: {exc}", file=sys.stderr)
    raise SystemExit(1)

map_id = str(data.get("map_id") or "")
building_id = str(data.get("building_id") or "")
floor_id = str(data.get("floor_id") or "")
display_name = str(data.get("display_name") or map_id).replace("\t", " ").replace("\n", " ")
if not (safe.fullmatch(map_id) and safe.fullmatch(building_id) and safe.fullmatch(floor_id)):
    print("[runtime-overlay] last navigation map file has invalid ids", file=sys.stderr)
    raise SystemExit(1)

current_root = maps_root / building_id / floor_id / "current"
current_manifest = current_root / "manifest.json"
required = [
    current_manifest,
    current_root / "nav" / "nav_map.yaml",
    current_root / "localizer" / "localizer_params.yaml",
    current_root / "localizer" / "localizer_map.png",
]
missing = [str(path) for path in required if not path.is_file()]
if missing:
    print("[runtime-overlay] last navigation map is not selected in current/: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)

try:
    current_data = json.loads(current_manifest.read_text(encoding="utf-8"))
except Exception as exc:
    print(f"[runtime-overlay] cannot read current manifest {current_manifest}: {exc}", file=sys.stderr)
    raise SystemExit(1)

if str(current_data.get("map_id") or "") != map_id:
    print(
        f"[runtime-overlay] last navigation map {map_id} does not match current manifest "
        f"{current_data.get('map_id')}",
        file=sys.stderr,
    )
    raise SystemExit(1)

print("\t".join([building_id, floor_id, map_id, display_name]))
PY
}

latch_safety_stop_for_runtime_shutdown() {
  echo "[runtime-overlay] latching safety estop before complete runtime shutdown" >&2
  timeout --kill-after=0.2 1.0 \
    ros2 topic pub --once \
    --qos-reliability reliable \
    /safety/estop std_msgs/msg/Bool "{data: true}" \
    >/dev/null 2>&1 &
  local estop_publish_pid=$!
  # Stop resident direct motion producers immediately. robot_safety remains
  # alive long enough to consume the estop and publish the final zero command.
  pkill -INT -f \
    "robot_docking_manager/docking_manager_node|docking_manager_node --ros-args|__node:=controller_server|__node:=behavior_server" \
    2>/dev/null || true
  wait "${estop_publish_pid}" 2>/dev/null || true
}

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" ]]; then
    njrh_finish_startup_cpu_boost common_exit
  fi
  local pid
  latch_safety_stop_for_runtime_shutdown
  cleanup_common_startup_helpers
  cleanup_resident_navigation_runtime_layers
  for pid in "${common_pids[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  cleanup_overlay_helpers
  cleanup_canonical_helpers
  rm -f "${NJRH_NAVIGATION_STARTUP_RECEIPT}"
  sleep 1
  cleanup_resident_navigation_runtime_layers
  for pid in "${common_pids[@]:-}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

# Create only in the full-chain owner, after exit cleanup is installed.
if [[ "${NAV_LOCAL_STATE_MODE}" == ekf && "${FASTLIO_AUTOSTART}" != true ]]; then
  njrh_begin_startup_cpu_boost
fi

start_docking_common_last() {
  case "${DOCKING_SENSOR_BACKEND}" in
    gs2)
      if pgrep -f "[o]rbbec_camera|[o]rbbec_depth_dock_node|[r]un_orbbec_336l_depth.sh|[r]un_orbbec_docking_perception.sh" >/dev/null 2>&1; then
        echo "[runtime-overlay] GS2 backend refused while Orbbec docking processes are still running" >&2
        return 1
      fi
      if [[ "${NJRH_GS2_AUTOSTART:-true}" == "true" ]]; then
        start_common_process "gs2_driver" "robot_eai_gs2/gs2_driver_node|gs2_driver_node --ros-args|ros2 launch robot_eai_gs2 gs2.launch.py" \
          bash "${SCRIPT_DIR}/run_gs2_driver.sh" || return 1
      fi
      ;;
    orbbec_336l)
      if pgrep -f "robot_eai_gs2/[g]s2_driver_node|[g]s2_driver_node --ros-args|ros2 launch robot_eai_gs2 [g]s2.launch.py" >/dev/null 2>&1; then
        echo "[runtime-overlay] Orbbec backend refused while GS2 docking processes are still running" >&2
        return 1
      fi
      sensors_config="${ROBOT_DESCRIPTION_CONFIG_FILE:-${NJRH_OVERLAY_ROOT}/config/sensors.yaml}"
      for required_key in \
        docking_camera_x docking_camera_y docking_camera_z \
        docking_camera_roll docking_camera_pitch docking_camera_yaw
      do
        if ! grep -Eq "^[[:space:]]*${required_key}:" "${sensors_config}"; then
          echo "[runtime-overlay] Orbbec docking backend refused: missing ${required_key} in ${sensors_config}" >&2
          return 1
        fi
      done
      start_orbbec_336l_depth_common || return 1
      start_common_process "orbbec_docking_perception" "robot_docking_perception/orbbec_depth_dock_node|orbbec_depth_dock_node" \
        bash "${SCRIPT_DIR}/run_orbbec_docking_perception.sh" || return 1
      ;;
    *)
      echo "[runtime-overlay] unsupported NJRH_DOCKING_SENSOR_BACKEND=${DOCKING_SENSOR_BACKEND}" >&2
      return 1
      ;;
  esac
  log_common_startup_stage "docking_sensor_started"
  if [[ "${DOCKING_SENSOR_BACKEND}" == "orbbec_336l" ]]; then
    if wait_for_fresh_header_topic_message \
      "/dock/target_observation" \
      "${NJRH_DOCKING_SENSOR_READY_TIMEOUT_SEC:-15}" \
      "${NJRH_DOCKING_SENSOR_MAX_AGE_SEC:-1.0}" \
      "${NJRH_DOCKING_SENSOR_MAX_FUTURE_SEC:-0.25}"; then
      log_common_startup_stage "docking_sensor_ready"
    else
      log_common_startup_stage "docking_sensor_degraded"
      echo "[runtime-overlay] docking observation unavailable; keeping API/navigation/common alive; manager retains its observation checks" >&2
    fi
  fi
  if [[ "${NJRH_DOCKING_MANAGER_AUTOSTART:-true}" == "true" ]]; then
    start_common_process "docking_manager" "robot_docking_manager/docking_manager_node|docking_manager_node --ros-args|run_docking_manager.sh" \
      bash "${SCRIPT_DIR}/run_docking_manager.sh" || return 1
    log_common_startup_stage "docking_manager_ready"
  else
    echo "[runtime-overlay] docking_manager autostart disabled" >&2
  fi
}

update_common_deferred_startup() {
  if [[ "${api_http_ready}" == 0 ]] && common_api_http_ready; then
    api_http_ready=1
    log_common_startup_stage "robot_api_server_ready"
  fi
  [[ "${docking_startup_done}" == 0 && "${api_http_ready}" == 1 ]] || return 0
  common_navigation_initialization_finished || return 0
  # Exactly one startup attempt in this owner; no extra producer or respawn policy.
  docking_startup_done=1
  if [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" ]]; then
    njrh_finish_startup_cpu_boost common_initialization_finished
  fi
  log_common_startup_stage "main_initialization_finished"
  if ! start_docking_common_last; then
    log_common_startup_stage "docking_startup_failed"
    echo "[runtime-overlay] docking startup failed; common core remains running" >&2
  fi
  log_common_startup_stage "common_startup_finished"
}

require_can_interface_up
log_common_startup_stage "can_ready"

# Resolve/clean the selected-map branch before launching common producers;
# its map/Isaac initialization then overlaps all sensor readiness below.
if [[ "${NAV_LOCAL_STATE_MODE}" == "ekf" && "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]]; then
  start_resident_navigation_autostart_if_selected
fi

# Launch first, join readiness later. Keep ownership in this shell, rather than
# placing start_canonical_helper itself in a subshell and losing its PID list.
start_common_canonical_helper_background "robot_description_static_tf_common" \
  bash "${SCRIPT_DIR}/run_robot_description.sh"
log_common_startup_stage "static_tf_started"

if reuse_common_services_enabled && canonical_jt128_runtime_complete; then
  echo "[runtime-overlay] reusing existing jt128_driver; canonical driver/remap chain is complete" >&2
else
  if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
    start_common_process "jt128_driver" "__njrh_force_start_jt128_driver_chain__" \
      env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_driver.sh"
  else
    stop_stale_pointcloud_accel_pipeline_processes
    start_common_process "pointcloud_accel_pipeline" "__njrh_force_start_pointcloud_accel_pipeline__" \
      env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh"
  fi
fi
log_common_startup_stage "pointcloud_started"
start_common_canonical_helper_background "ranger_chassis_common" \
  bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
log_common_startup_stage "ranger_chassis_started"
start_robot_local_state_common_background_if_enabled
# Keep the explicit FAST-LIO diagnostic dependency order unchanged. Normal
# wheel+IMU navigation does not enter this branch or start FAST-LIO.
if [[ "${FASTLIO_AUTOSTART}" == "true" || "${NAV_LOCAL_STATE_MODE}" == "fastlio" ]]; then
  wait_for_common_startup_job "robot_description_static_tf_common"
  wait_for_common_startup_job "ranger_chassis_common"
fi
if [[ "${FASTLIO_AUTOSTART}" == "true" ]] || { [[ "${NAV_LOCAL_STATE_MODE}" == "fastlio" ]] && fastlio_runtime_running; }; then
  start_fastlio_common
elif [[ "${NAV_LOCAL_STATE_MODE}" == "fastlio" ]]; then
  echo "[runtime-overlay] NJRH_NAV_LOCAL_STATE_MODE=fastlio requires NJRH_FASTLIO_AUTOSTART=true or an already managed FAST-LIO runtime" >&2
  exit 1
else
  stop_non_mapping_fastlio_runtime_processes
  echo "[runtime-overlay] FAST-LIO2 common autostart disabled; mapping starts FAST-LIO2 only while mapping is active" >&2
fi
log_common_startup_stage "fastlio_policy_done"
if [[ "${NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART:-true}" == "true" ]]; then
  start_runtime_health_guard_common
else
  echo "[runtime-overlay] runtime_health_guard autostart disabled; startup readiness probes are disabled" >&2
fi
log_common_startup_stage "runtime_health_guard_ready"
wait_for_common_startup_job "robot_description_static_tf_common"
log_common_startup_stage "static_tf_ready"
wait_for_common_startup_job "ranger_chassis_common"
log_common_startup_stage "ranger_chassis_ready"
echo "[runtime-overlay] local_perception_common disabled; local costmap/collision_monitor consume /scan for standard marking+clearing" >&2
start_overlay_helper "floor_manager_common" bash "${SCRIPT_DIR}/run_floor_manager.sh"
log_common_startup_stage "floor_manager_ready"
start_overlay_helper "robot_safety_common" bash "${SCRIPT_DIR}/run_robot_safety.sh"
log_common_startup_stage "robot_safety_ready"
start_overlay_helper "mode_manager_common" bash "${SCRIPT_DIR}/run_mode_manager.sh"
log_common_startup_stage "mode_manager_ready"
start_common_process "robot_api_server" "run_robot_api_server.sh|run_robot_api_server_supervised.sh|robot_api_server/robot_api_server_node|robot_api_server_node --ros-args" \
  bash "${SCRIPT_DIR}/run_robot_api_server_supervised.sh"
if [[ "${NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE:-false}" == "true" ]]; then
  start_resident_navigation_autostart_if_selected
fi
if [[ "${NAV_LOCAL_STATE_MODE}" == "passthrough" || "${NAV_LOCAL_STATE_MODE}" == "legacy" ]]; then
  # Explicit diagnostic fallback: keep the canonical /local_state/odometry
  # and odom->base_link owner, but back it directly with /wheel/odom.
  kill_canonical_pattern "robot_localization/ekf_node"
  kill_canonical_pattern "ekf_node --ros-args.*__node:=robot_local_state"
fi
wait_for_robot_local_state_common_background_if_started
log_common_startup_stage "local_state_ready"
if [[ "${NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART:-true}" == "true" ]]; then
  if [[ "${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}" == "endpoint" ]]; then
    wait_for_runtime_health_local_state_endpoint_ready
  else
    wait_for_runtime_health_local_state_ready
  fi
fi
log_common_startup_stage "local_state_health_checked"
if [[ "${NJRH_RESIDENT_NAVIGATION_EARLY_AUTOSTART:-true}" == "true" ]]; then
  start_resident_navigation_autostart_if_selected
fi
log_common_startup_stage "resident_navigation_started"

log_common_startup_stage "ranger_chassis_core_ready"
wait_for_robot_api_server_common_ready
log_common_startup_stage "robot_api_server_process_ready"

if [[ "${RESIDENT_NAVIGATION_AUTOSTART}" != "false" ]]; then
  start_resident_navigation_autostart_if_selected
  wait_for_resident_navigation_autostart_if_started
fi
log_common_startup_stage "common_core_services_started"
echo "[runtime-overlay] docking deferred until API HTTP and navigation initialization finish (localization success is not required)" >&2
update_common_deferred_startup

echo "[runtime-overlay] common services are running; start mapping or resident navigation scripts in reuse mode" >&2
while true; do
  sleep "${NJRH_COMMON_MAIN_HEALTH_PERIOD_SEC:-5}"
  verify_ranger_chassis_common_health_or_exit
  verify_robot_local_state_common_health_or_exit
  verify_robot_api_server_common_health_or_exit
  update_common_deferred_startup
  if [[ "${docking_startup_done}" == 1 ]]; then
    verify_docking_sensor_common_health_or_exit
  fi
done

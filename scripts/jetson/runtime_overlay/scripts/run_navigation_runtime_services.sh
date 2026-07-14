#!/usr/bin/env bash
set -euo pipefail

startup_epoch_sec="$(date +%s)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/commercial_runtime_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
floor_id="${2:-${NJRH_FLOOR_ID:-}}"
export NJRH_RUNTIME_MAP_CONTEXT_FILE="${NJRH_RUNTIME_MAP_CONTEXT_FILE:-/tmp/njrh_runtime_map_context.json}"
export NJRH_RUNTIME_FAILURE_CODE=""
export NJRH_RUNTIME_LOCALIZATION_MODE="${NJRH_ISAAC_LOCALIZATION_MODE:-triggered}"
export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK=""
export NJRH_RUNTIME_MAP_TO_ODOM_AGE_MS=""
export NJRH_AMCL_RUNTIME_STATUS_FILE="${NJRH_AMCL_RUNTIME_STATUS_FILE:-/tmp/njrh_amcl_runtime_status.env}"
export NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP="${NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP:-true}"

[[ -n "${floor_id}" ]] || {
  echo "[runtime-overlay] floor_id is required for resident navigation runtime" >&2
  echo "[runtime-overlay] keep common services running in NO_MAP mode until a released floor asset is selected" >&2
  exit 2
}

resolve_floor_assets_if_needed "${building_id}" "${floor_id}"

localization_pid=""
navigation_pid=""
amcl_resident_pid=""
amcl_readiness_pid=""
amcl_status_heartbeat_pid=""
nav2_lifecycle_bringup_pid=""
initial_global_localization_pid=""
nav2_prestarted=0
nav2_lifecycle_background_started=0
amcl_runtime_started=0
exit_code=0
runtime_ready=0
cleanup_started=0
localization_ready_failure_reason=""

stale_amcl_heartbeat_pids() {
  ps -eo pid=,args= |
    awk '/run_amcl_shadow_localization.sh/ && /--heartbeat/ && !/awk/ {print $1}' || true
}

cleanup_stale_amcl_runtime_status_owner() {
  local pids
  pids="$(stale_amcl_heartbeat_pids)"
  if [[ -n "${pids}" ]]; then
    echo "[runtime-overlay] stopping stale AMCL runtime status heartbeat before resident navigation startup: ${pids}" >&2
    kill -INT ${pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC:-0.2}"
    pids="$(stale_amcl_heartbeat_pids)"
    [[ -z "${pids}" ]] || kill -TERM ${pids} 2>/dev/null || true
  fi
  rm -f "${NJRH_AMCL_RUNTIME_STATUS_FILE}" 2>/dev/null || true
}

cleanup_stale_amcl_runtime_status_owner

env_flag_true() {
  case "${1,,}" in
    1|true|yes|on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

log_startup_stage() {
  local stage="$1"
  local now_sec
  now_sec="$(date +%s)"
  local elapsed_sec=$((now_sec - startup_epoch_sec))
  export NJRH_RUNTIME_STARTUP_STAGE="${stage}"
  export NJRH_RUNTIME_STARTUP_ELAPSED_SEC="${elapsed_sec}"
  echo "[runtime-overlay] STARTUP_STAGE stage=${stage} elapsed_sec=${elapsed_sec}" >&2
  if [[ "${runtime_ready}" -eq 0 && "${cleanup_started}" -eq 0 ]]; then
    write_runtime_map_context "starting" "false" "resident navigation runtime starting: ${stage}"
  fi
}

log_startup_stage "script_start"

wait_for_child_exit() {
  local pid="$1"
  local attempts="${2:-20}"
  local i
  for ((i = 0; i < attempts; i += 1)); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

terminate_child() {
  local pid="$1"
  local label="$2"
  [[ -n "${pid}" ]] || return 0
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi
  echo "[runtime-overlay] stopping ${label} pid=${pid}" >&2
  kill -INT "${pid}" 2>/dev/null || true
  wait_for_child_exit "${pid}" "${NJRH_NAV_RUNTIME_STOP_INT_ATTEMPTS:-20}" || {
    kill -TERM "${pid}" 2>/dev/null || true
    wait_for_child_exit "${pid}" "${NJRH_NAV_RUNTIME_STOP_TERM_ATTEMPTS:-20}" || {
      kill -KILL "${pid}" 2>/dev/null || true
    }
  }
  wait "${pid}" 2>/dev/null || true
}

stop_navigation_layer_after_failure() {
  local reason="$1"
  if [[ -n "${navigation_pid}" ]]; then
    terminate_child "${navigation_pid}" "resident Nav2 layer"
    navigation_pid=""
  fi
  echo "[runtime-overlay] sweeping standard Nav2 stack after resident Nav2 ${reason}; localization layer remains running" >&2
  stop_existing_standard_nav_stack
}

cleanup() {
  if [[ "${cleanup_started}" -eq 1 ]]; then
    return 0
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  if [[ "${runtime_ready}" -eq 1 ]]; then
    return 0
  fi
  if [[ "${amcl_runtime_started}" -eq 1 || "${NJRH_AMCL_LOCALIZATION_MODE:-disabled}" != "disabled" ]]; then
    bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --stop >/dev/null 2>&1 || true
  fi
  terminate_child "${initial_global_localization_pid}" "initial global localization trigger"
  terminate_child "${nav2_lifecycle_bringup_pid}" "Nav2 lifecycle activation"
  terminate_child "${navigation_pid}" "resident Nav2 layer"
  terminate_child "${amcl_resident_pid}" "AMCL resident warmup"
  terminate_child "${amcl_readiness_pid}" "AMCL readiness completion"
  terminate_child "${amcl_status_heartbeat_pid}" "AMCL runtime status heartbeat"
  terminate_child "${localization_pid}" "resident localization layer"
  stop_existing_standard_nav_stack
  stop_existing_localization_stack
}

on_signal() {
  runtime_ready=0
  cleanup
  exit 130
}

on_exit() {
  local status=$?
  if [[ "${status}" -ne 0 && "${runtime_ready}" -ne 1 ]]; then
    write_runtime_map_context "failed" "false" "resident navigation runtime failed; check ${NJRH_NAVIGATION_RESUME_LOG_FILE:-/tmp/njrh_navigation_resume.log}"
  fi
  cleanup
  exit "${status}"
}

trap cleanup EXIT
trap on_exit EXIT
trap on_signal INT TERM

nav_lifecycle_active() {
  local node_name="$1"
  local state
  state="$(timeout 3 ros2 lifecycle get "${node_name}" 2>/dev/null || true)"
  [[ "${state}" == active* ]]
}

nav_lifecycle_active_quick() {
  local node_name="$1"
  local timeout_sec="${NJRH_NAV2_LIFECYCLE_ACTIVE_POLL_TIMEOUT_SEC:-0.5}"
  [[ "${node_name}" == /* ]] || node_name="/${node_name}"
  runtime_readiness_probe lifecycle-active "${node_name}" "${timeout_sec}" >/dev/null 2>&1
}

nav_lifecycle_nodes_active_quick() {
  local node_name
  for node_name in "$@"; do
    nav_lifecycle_active_quick "${node_name}" || return 1
  done
}

resident_navigation_ready() {
  [[ "${NJRH_NAV_REUSE_READY_CONTEXT:-true}" == "true" ]] || return 1
  runtime_map_context_matches_current_floor
}

bridge_status_once() {
  timeout 5 ros2 topic echo /localization/bridge_status --once --field data 2>/dev/null || true
}

bridge_status_field() {
  local status="$1"
  local field="$2"
  BRIDGE_STATUS_JSON="${status}" python3 - "${field}" <<'PY'
import json
import os
import sys

field = sys.argv[1]
try:
    data = json.loads(os.environ.get("BRIDGE_STATUS_JSON", ""))
except Exception:
    print("")
    raise SystemExit(0)
if isinstance(data, str):
    try:
        data = json.loads(data)
    except Exception:
        print("")
        raise SystemExit(0)
if not isinstance(data, dict):
    print("")
    raise SystemExit(0)
value = data.get(field, "")
if isinstance(value, bool):
    print("true" if value else "false")
else:
    print(value)
PY
}

extract_failure_code() {
  local text="$1"
  grep -Eo 'failure_code=[A-Z_]+' <<<"${text}" | head -n 1 | cut -d= -f2
}

runtime_failure_code_for_wrapper_code() {
  local wrapper_code="$1"
  case "${wrapper_code}" in
    ISAAC_SERVICE_TIMEOUT)
      printf '%s\n' "GLOBAL_LOCALIZATION_TRIGGER_TIMEOUT"
      ;;
    LOCALIZATION_RESULT_TIMEOUT)
      printf '%s\n' "LOCALIZATION_RESULT_TIMEOUT"
      ;;
    MAP_TO_ODOM_TIMEOUT)
      printf '%s\n' "MAP_TO_ODOM_TIMEOUT"
      ;;
    MAP_TO_ODOM_WRONG_OWNER)
      printf '%s\n' "MAP_TO_ODOM_WRONG_OWNER"
      ;;
    TF_HISTORY_MISSING)
      printf '%s\n' "TF_HISTORY_MISSING"
      ;;
    BRIDGE_REJECTED_RESULT)
      printf '%s\n' "BRIDGE_REJECTED_RESULT"
      ;;
    FRESH_LOCALIZATION_RETRY_REQUIRED)
      printf '%s\n' "FRESH_LOCALIZATION_RETRY_REQUIRED"
      ;;
    BRIDGE_ACCEPT_TIMEOUT)
      printf '%s\n' "BRIDGE_ACCEPT_TIMEOUT"
      ;;
    *)
      printf '%s\n' "GLOBAL_LOCALIZATION_TRIGGER_TIMEOUT"
      ;;
  esac
}

wait_for_bridge_has_map_to_odom() {
  local timeout_sec="${1:-20}"
  local deadline=$((SECONDS + timeout_sec))
  local status
  local has_map
  local owner
  local mode
  local age_ms
  while (( SECONDS < deadline )); do
    status="$(bridge_status_once)"
    has_map="$(bridge_status_field "${status}" has_map_to_odom)"
    owner="$(bridge_status_field "${status}" map_to_odom_publisher_owner)"
    mode="$(bridge_status_field "${status}" gate_mode)"
    age_ms="$(bridge_status_field "${status}" map_to_odom_age_ms)"
    if [[ "${has_map}" == "true" && "${owner}" != "robot_localization_bridge" ]]; then
      echo "[runtime-overlay] map->odom wrong owner in bridge_status: owner=${owner}" >&2
      export NJRH_RUNTIME_LOCALIZATION_MODE="${mode:-${NJRH_ISAAC_LOCALIZATION_MODE:-triggered}}"
      export NJRH_RUNTIME_MAP_TO_ODOM_AGE_MS="${age_ms:-}"
      return 2
    fi
    if [[ "${has_map}" == "true" && "${owner}" == "robot_localization_bridge" ]]; then
      export NJRH_RUNTIME_LOCALIZATION_MODE="${mode:-${NJRH_ISAAC_LOCALIZATION_MODE:-triggered}}"
      export NJRH_RUNTIME_MAP_TO_ODOM_AGE_MS="${age_ms:-}"
      return 0
    fi
    sleep 0.2
  done
  return 1
}

trigger_output_reports_map_to_odom_ready() {
  local output="$1"
  grep -Eq 'map->odom ready owner=robot_localization_bridge' <<<"${output}"
}

trigger_output_reports_startup_service_race() {
  local output="$1"
  grep -Eiq 'waiting for service to become available|service[^[:cntrl:]]*not[^[:cntrl:]]*available|service is not available' <<<"${output}"
}

trigger_output_reports_transient_stale_bridge_timeout() {
  local output="$1"
  if grep -Eiq 'failure_code=FRESH_LOCALIZATION_RETRY_REQUIRED' <<<"${output}"; then
    return 0
  fi
  grep -Eiq 'failure_code=BRIDGE_ACCEPT_TIMEOUT' <<<"${output}" &&
    grep -Eiq 'isaac_triggered_pose_stale_ms' <<<"${output}"
}

trigger_output_reports_fresh_localization_retry_required() {
  local output="$1"
  grep -Eiq 'failure_code=FRESH_LOCALIZATION_RETRY_REQUIRED' <<<"${output}"
}

trigger_output_reports_transient_map_to_odom_timeout() {
  local output="$1"
  grep -Eiq 'failure_code=MAP_TO_ODOM_TIMEOUT' <<<"${output}" &&
    grep -Eiq 'has_map_to_odom=true' <<<"${output}" &&
    grep -Eiq 'owner=robot_localization_bridge' <<<"${output}"
}

trigger_output_reports_transient_localization_result_timeout() {
  local output="$1"
  grep -Eiq 'failure_code=LOCALIZATION_RESULT_TIMEOUT' <<<"${output}"
}

trigger_output_reports_transient_amcl_pose_stale_reject() {
  local output="$1"
  grep -Eiq 'failure_code=BRIDGE_REJECTED_RESULT' <<<"${output}" &&
    grep -Eiq 'AMCL_POSE_STALE' <<<"${output}"
}

initial_localization_ready_from_bridge_after_wrapper_failure() {
  local bridge_timeout="${1:-8}"
  local tf_timeout="${2:-8}"
  local wrapper_code="${3:-none}"
  local max_tf_age_sec="${NJRH_INITIAL_LOCALIZATION_MAP_ODOM_MAX_AGE_SEC:-0.5}"
  local bridge_status_ready=false
  local bridge_status_rc=0
  if wait_for_bridge_has_map_to_odom "${bridge_timeout}"; then
    bridge_status_ready=true
  else
    bridge_status_rc=$?
    if [[ "${bridge_status_rc}" -eq 2 ]]; then
      return 1
    fi
    echo "[runtime-overlay] bridge_status did not report has_map_to_odom before wrapper timeout fallback; checking live bridge process and fresh TF" >&2
    local bridge_pattern=""
    bridge_pattern="$(helper_process_pattern "robot_localization_bridge" 2>/dev/null || true)"
    if [[ -z "${bridge_pattern}" ]] || ! helper_process_running "${bridge_pattern}"; then
      return 1
    fi
  fi
  if ! wait_for_fresh_tf_transform "map" "odom" "${tf_timeout}" "${max_tf_age_sec}"; then
    echo "[runtime-overlay] fresh map->odom TF probe failed during wrapper-timeout fallback; checking plain TF edge before rejecting accepted bridge state" >&2
    if ! wait_for_tf_transform "map" "odom" "${tf_timeout}"; then
      return 1
    fi
    echo "[runtime-overlay] fresh map->odom TF probe failed but plain TF edge exists; continuing because active bridge/map->odom ownership was already verified" >&2
  fi
  export NJRH_RUNTIME_FAILURE_CODE=""
  export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK="true"
  echo "[runtime-overlay] global localization wrapper did not return accepted before timeout, but bridge map->odom and fresh TF are ready; continuing startup: wrapper_code=${wrapper_code} bridge_status_ready=${bridge_status_ready}" >&2
  echo "[runtime-overlay] initial localization accepted: bridge_status.has_map_to_odom=true and map->odom are ready" >&2
}

trigger_global_localization_for_navigation() {
  local reason="resident_navigation_start:${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}"
  local call_timeout="${NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-90}"
  local attempt_timeout="${NJRH_GLOBAL_LOCALIZATION_TRIGGER_ATTEMPT_TIMEOUT:-75}"
  local bridge_timeout="${NJRH_INITIAL_LOCALIZATION_BRIDGE_ACCEPT_WAIT_SEC:-8}"
  local tf_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_ODOM_WAIT_SEC:-8}"
  local payload="{reason: '${reason}'}"
  local trigger_output
  local wrapper_code
  local runtime_code
  local deadline=$((SECONDS + call_timeout))
  local attempt=1
  local remaining
  local this_timeout
  local trigger_rc=1
  local accepted=false

  echo "[runtime-overlay] requesting global localization through wrapper and waiting for bridge/map->odom" >&2
  while (( SECONDS < deadline )); do
    remaining=$((deadline - SECONDS))
    this_timeout="${attempt_timeout}"
    if (( this_timeout > remaining )); then
      this_timeout="${remaining}"
    fi
    if (( this_timeout < 5 )); then
      this_timeout=5
    fi
    echo "[runtime-overlay] global localization trigger attempt=${attempt} timeout=${this_timeout}s remaining=${remaining}s" >&2
    if trigger_output="$(python3 "${SCRIPT_DIR}/call_global_localization_trigger.py" \
      --reason "${reason}" \
      --timeout-sec "${this_timeout}" 2>&1)"; then
      trigger_rc=0
      if grep -Eq 'accepted[=:][[:space:]]*(True|true)|accepted:[[:space:]]*true' <<<"${trigger_output}"; then
        accepted=true
        break
      fi
      wrapper_code="$(extract_failure_code "${trigger_output}")"
      if trigger_output_reports_startup_service_race "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} hit startup service race; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_stale_bridge_timeout "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} saw transient stale Isaac result; retrying for fresh result: wrapper_code=${wrapper_code:-none}" >&2
        if ! trigger_output_reports_fresh_localization_retry_required "${trigger_output}"; then
          sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        fi
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_map_to_odom_timeout "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} saw transient stale map->odom during startup; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_localization_result_timeout "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} saw transient localization result timeout during startup; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_amcl_pose_stale_reject "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} saw transient stale AMCL pose reject during startup; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      break
    else
      trigger_rc=$?
      wrapper_code="$(extract_failure_code "${trigger_output}")"
      if trigger_output_reports_startup_service_race "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} could not reach service; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_stale_bridge_timeout "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} timed out on transient stale Isaac result; retrying for fresh result: wrapper_code=${wrapper_code:-none}" >&2
        if ! trigger_output_reports_fresh_localization_retry_required "${trigger_output}"; then
          sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        fi
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_map_to_odom_timeout "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} timed out on transient stale map->odom during startup; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_localization_result_timeout "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} timed out on transient localization result timeout during startup; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      if trigger_output_reports_transient_amcl_pose_stale_reject "${trigger_output}" && (( SECONDS < deadline )); then
        echo "[runtime-overlay] global localization trigger attempt=${attempt} timed out on transient stale AMCL pose reject during startup; retrying: wrapper_code=${wrapper_code:-none}" >&2
        sleep "${NJRH_GLOBAL_LOCALIZATION_TRIGGER_RETRY_SLEEP_SEC:-1}"
        attempt=$((attempt + 1))
        continue
      fi
      break
    fi
  done

  if [[ "${accepted}" != "true" ]]; then
    wrapper_code="$(extract_failure_code "${trigger_output}")"
    if initial_localization_ready_from_bridge_after_wrapper_failure "${bridge_timeout}" "${tf_timeout}" "${wrapper_code:-none}"; then
      return 0
    fi
    if [[ "${trigger_rc}" -ne 0 ]]; then
      runtime_code="$(runtime_failure_code_for_wrapper_code "${wrapper_code:-ISAAC_SERVICE_TIMEOUT}")"
      export NJRH_RUNTIME_FAILURE_CODE="${runtime_code}"
      export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK="false"
      set_localization_ready_failure \
        "${runtime_code}" \
        "/global_localization/trigger did not complete within ${call_timeout}s; wrapper_code=${wrapper_code:-none}; output=${trigger_output}"
      return 1
    fi
    runtime_code="$(runtime_failure_code_for_wrapper_code "${wrapper_code:-GLOBAL_LOCALIZATION_TRIGGER_FAILED}")"
    export NJRH_RUNTIME_FAILURE_CODE="${runtime_code}"
    export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK="false"
    set_localization_ready_failure \
      "${runtime_code}" \
      "/global_localization/trigger rejected or timed out internally; wrapper_code=${wrapper_code:-none}; output=${trigger_output}"
    return 1
  fi

  export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK="true"
  echo "[runtime-overlay] global localization wrapper accepted: ${trigger_output}" >&2
  if trigger_output_reports_map_to_odom_ready "${trigger_output}"; then
    echo "[runtime-overlay] wrapper already verified bridge map->odom readiness" >&2
  else
    wait_for_bridge_has_map_to_odom "${bridge_timeout}" || {
      local status=$?
      if [[ "${status}" -eq 2 ]]; then
        export NJRH_RUNTIME_FAILURE_CODE="MAP_TO_ODOM_WRONG_OWNER"
        set_localization_ready_failure "MAP_TO_ODOM_WRONG_OWNER" "bridge_status reports map->odom owner is not robot_localization_bridge"
      else
        export NJRH_RUNTIME_FAILURE_CODE="MAP_TO_ODOM_TIMEOUT"
        set_localization_ready_failure "MAP_TO_ODOM_TIMEOUT" "bridge_status.has_map_to_odom=true was not observed within ${bridge_timeout}s"
      fi
      return 1
    }
  fi
  wait_for_tf_transform "map" "odom" "${tf_timeout}" || {
    export NJRH_RUNTIME_FAILURE_CODE="MAP_TO_ODOM_TIMEOUT"
    export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK="false"
    set_localization_ready_failure "MAP_TO_ODOM_TIMEOUT" "map->odom TF was not published after bridge acceptance"
    return 1
  }
  export NJRH_RUNTIME_FAILURE_CODE=""
  echo "[runtime-overlay] initial localization accepted: bridge_status.has_map_to_odom=true and map->odom are ready" >&2
}

start_initial_global_localization_background() {
  if [[ -n "${initial_global_localization_pid}" ]] && kill -0 "${initial_global_localization_pid}" 2>/dev/null; then
    return 0
  fi
  echo "[runtime-overlay] starting initial global localization trigger in background; readiness still waits for bridge/map->odom acceptance" >&2
  log_startup_stage "initial_global_localization_trigger_started"
  (
    set +e
    trigger_global_localization_for_navigation
    rc=$?
    if [[ "${rc}" -eq 0 ]]; then
      echo "[runtime-overlay] initial global localization trigger completed in background" >&2
    else
      echo "[runtime-overlay] initial global localization trigger background failed rc=${rc}" >&2
    fi
    exit "${rc}"
  ) &
  initial_global_localization_pid=$!
  echo "[runtime-overlay] initial global localization trigger running in background pid=${initial_global_localization_pid}" >&2
}

wait_for_initial_global_localization() {
  if [[ -z "${initial_global_localization_pid}" ]]; then
    trigger_global_localization_for_navigation
    return $?
  fi
  local pid="${initial_global_localization_pid}"
  local rc=0
  set +e
  wait "${pid}"
  rc=$?
  set -e
  initial_global_localization_pid=""
  if [[ "${rc}" -ne 0 ]]; then
    echo "[runtime-overlay] initial global localization background returned rc=${rc}" >&2
    return "${rc}"
  fi
  export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK="true"
  wait_for_bridge_has_map_to_odom 1 >/dev/null 2>&1 || true
  echo "[runtime-overlay] initial global localization background joined successfully" >&2
}

amcl_mode_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  case "${mode}" in
    disabled|shadow|gated)
      printf '%s\n' "${mode}"
      ;;
    *)
      set_localization_ready_failure "AMCL_MODE_INVALID" "invalid NJRH_AMCL_LOCALIZATION_MODE=${mode}"
      return 1
      ;;
  esac
}

load_amcl_runtime_status() {
  AMCL_MODE=""
  AMCL_START_RESULT=""
  AMCL_READY="false"
  AMCL_DEGRADED="false"
  AMCL_FAILURE_REASON=""
  AMCL_NODE_EXISTS="false"
  AMCL_LIFECYCLE_ACTIVE="false"
  AMCL_PROCESS_READY="false"
  AMCL_STATUS_STALE="false"
  AMCL_PID_ALIVE="false"
  SCAN_ADMISSION_ALIVE="false"
  SCAN_ADMISSION_STATUS_PUBLISHER_COUNT="0"
  AMCL_POSE_PUBLISHER_COUNT="0"
  AMCL_SEED_SUCCEEDED="false"
  if [[ -f "${NJRH_AMCL_RUNTIME_STATUS_FILE}" ]]; then
    if ! bash -n "${NJRH_AMCL_RUNTIME_STATUS_FILE}" >/dev/null 2>&1; then
      echo "[runtime-overlay] ignoring invalid AMCL runtime status file: ${NJRH_AMCL_RUNTIME_STATUS_FILE}" >&2
      return 0
    fi
    # shellcheck disable=SC1090
    if ! source "${NJRH_AMCL_RUNTIME_STATUS_FILE}"; then
      echo "[runtime-overlay] failed to source AMCL runtime status file: ${NJRH_AMCL_RUNTIME_STATUS_FILE}" >&2
      return 0
    fi
  fi
}

amcl_resident_runtime_status_ready_for_seed() {
  load_amcl_runtime_status
  [[ "${AMCL_STATUS_STALE:-false}" != "true" &&
    "${AMCL_START_RESULT:-}" == "waiting_seed" &&
    "${AMCL_PROCESS_READY:-false}" == "true" &&
    "${SCAN_ADMISSION_ALIVE:-false}" == "true" ]]
}

log_amcl_runtime_status() {
  load_amcl_runtime_status
  echo "[runtime-overlay] AMCL_STATUS mode=${AMCL_MODE:-unknown} result=${AMCL_START_RESULT:-unknown} ready=${AMCL_READY:-false} degraded=${AMCL_DEGRADED:-false} node=${AMCL_NODE_EXISTS:-false} lifecycle=${AMCL_LIFECYCLE_ACTIVE:-false} pid_alive=${AMCL_PID_ALIVE:-false} scan_alive=${SCAN_ADMISSION_ALIVE:-false} scan_status_publishers=${SCAN_ADMISSION_STATUS_PUBLISHER_COUNT:-0} pose_publishers=${AMCL_POSE_PUBLISHER_COUNT:-0} seed=${AMCL_SEED_SUCCEEDED:-false} reason=${AMCL_FAILURE_REASON:-}" >&2
}

run_amcl_localization_step() {
  local mode="$1"
  local phase="$2"
  shift 2
  local rc=0
  set +e
  NJRH_AMCL_LOCALIZATION_MODE="${mode}" \
    NJRH_AMCL_RUNTIME_STATUS_FILE="${NJRH_AMCL_RUNTIME_STATUS_FILE}" \
    bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --mode "${mode}" "$@"
  rc=$?
  set -e
  log_amcl_runtime_status
  case "${rc}" in
    0)
      return 0
      ;;
    10)
      if [[ "${mode}" == "shadow" ]]; then
        echo "[runtime-overlay] AMCL_DEGRADED phase=${phase}; continuing triggered baseline because mode=shadow" >&2
        write_runtime_map_context "degraded" "true" "AMCL shadow degraded: ${AMCL_FAILURE_REASON:-unknown}"
        return 0
      fi
      set_localization_ready_failure "AMCL_GATED_DEGRADED" "AMCL gated returned degraded in phase=${phase}: ${AMCL_FAILURE_REASON:-unknown}"
      return 1
      ;;
    21)
      set_localization_ready_failure "AMCL_GATED_NOT_READY" "AMCL gated not ready in phase=${phase}: ${AMCL_FAILURE_REASON:-unknown}"
      return 1
      ;;
    22)
      set_localization_ready_failure "AMCL_SCAN_ADMISSION_FAILED" "AMCL scan admission failed in phase=${phase}: ${AMCL_FAILURE_REASON:-unknown}"
      return 1
      ;;
    23)
      set_localization_ready_failure "AMCL_LIFECYCLE_FAILED" "AMCL lifecycle failed in phase=${phase}: ${AMCL_FAILURE_REASON:-unknown}"
      return 1
      ;;
    24)
      set_localization_ready_failure "AMCL_SEED_FAILED" "AMCL seed failed in phase=${phase}: ${AMCL_FAILURE_REASON:-unknown}"
      return 1
      ;;
    25)
      set_localization_ready_failure "AMCL_POSE_MISSING" "AMCL pose missing/stale in phase=${phase}: ${AMCL_FAILURE_REASON:-unknown}"
      return 1
      ;;
    *)
      set_localization_ready_failure "AMCL_FAILED" "AMCL runner failed in phase=${phase} exit=${rc}: ${AMCL_FAILURE_REASON:-unknown}"
      return 1
      ;;
  esac
}

start_amcl_resident_if_enabled_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  mode="$(amcl_mode_for_navigation)" || return 1
  case "${mode}" in
    disabled)
      echo "[runtime-overlay] AMCL continuous localization disabled" >&2
      return 0
      ;;
  esac
  echo "[runtime-overlay] starting resident AMCL localization candidate mode=${mode}" >&2
  if ! run_amcl_localization_step "${mode}" "start-resident" --start-resident; then
    return 1
  fi
  amcl_runtime_started=1
}

start_amcl_resident_background_if_enabled_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  mode="$(amcl_mode_for_navigation)" || return 1
  case "${mode}" in
    disabled)
      echo "[runtime-overlay] AMCL resident warmup skipped because AMCL mode is disabled" >&2
      return 0
      ;;
  esac

  (
    set +e
    NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE=true start_amcl_resident_if_enabled_for_navigation
    rc=$?
    if [[ "${rc}" -eq 0 ]]; then
      echo "[runtime-overlay] AMCL resident warmup completed in background" >&2
    else
      echo "[runtime-overlay] AMCL resident warmup background failed rc=${rc}; foreground readiness will retry startup path after Nav2 ready" >&2
    fi
    exit "${rc}"
  ) &
  amcl_resident_pid=$!
  echo "[runtime-overlay] AMCL resident warmup running in background pid=${amcl_resident_pid}" >&2
  return 0
}

wait_for_amcl_resident_background_if_running() {
  [[ -n "${amcl_resident_pid}" ]] || return 0
  local pid="${amcl_resident_pid}"
  local rc=0
  set +e
  wait "${pid}"
  rc=$?
  set -e
  amcl_resident_pid=""
  if [[ "${rc}" -eq 0 ]]; then
    echo "[runtime-overlay] AMCL resident warmup background joined" >&2
    return 0
  fi
  echo "[runtime-overlay] AMCL resident warmup background returned rc=${rc}; continuing to readiness retry loop" >&2
  return "${rc}"
}

start_amcl_readiness_background_if_enabled_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  mode="$(amcl_mode_for_navigation)" || return 1
  case "${mode}" in
    disabled)
      echo "[runtime-overlay] AMCL readiness background skipped because AMCL mode is disabled" >&2
      return 0
      ;;
  esac
  if [[ -n "${amcl_readiness_pid}" ]] && kill -0 "${amcl_readiness_pid}" 2>/dev/null; then
    return 0
  fi
  if [[ -n "${amcl_resident_pid}" ]]; then
    wait_for_amcl_resident_background_if_running || {
      echo "[runtime-overlay] AMCL resident warmup did not complete cleanly before readiness; readiness background will retry start-resident" >&2
    }
  fi
  (
    set +e
    if amcl_resident_runtime_status_ready_for_seed; then
      echo "[runtime-overlay] AMCL resident already warm from status file; skipping repeated resident warmup before readiness seed" >&2
    elif ! NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE=true start_amcl_resident_if_enabled_for_navigation; then
      echo "[runtime-overlay] AMCL readiness background could not finish start-resident; complete-readiness will retry in foreground" >&2
      exit 1
    fi
    if complete_amcl_readiness_with_retries_for_navigation; then
      echo "[runtime-overlay] AMCL readiness completed in background" >&2
      exit 0
    fi
    echo "[runtime-overlay] AMCL readiness background failed; foreground readiness will retry after Nav2 ready" >&2
    exit 1
  ) &
  amcl_readiness_pid=$!
  echo "[runtime-overlay] AMCL readiness running in background pid=${amcl_readiness_pid}" >&2
  return 0
}

wait_for_amcl_readiness_background_if_running() {
  [[ -n "${amcl_readiness_pid}" ]] || return 0
  local pid="${amcl_readiness_pid}"
  local rc=0
  set +e
  wait "${pid}"
  rc=$?
  set -e
  amcl_readiness_pid=""
  if [[ "${rc}" -eq 0 ]]; then
    echo "[runtime-overlay] AMCL readiness background joined" >&2
    return 0
  fi
  echo "[runtime-overlay] AMCL readiness background returned rc=${rc}; foreground readiness will retry" >&2
  return "${rc}"
}

start_amcl_status_heartbeat_if_enabled_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  mode="$(amcl_mode_for_navigation)" || return 1
  case "${mode}" in
    disabled)
      return 0
      ;;
  esac
  if [[ -n "${amcl_status_heartbeat_pid}" ]] && kill -0 "${amcl_status_heartbeat_pid}" 2>/dev/null; then
    return 0
  fi
  (
    set +e
    NJRH_AMCL_LOCALIZATION_MODE="${mode}" \
      NJRH_AMCL_RUNTIME_STATUS_FILE="${NJRH_AMCL_RUNTIME_STATUS_FILE}" \
      bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --mode "${mode}" --heartbeat
  ) &
  amcl_status_heartbeat_pid=$!
  echo "[runtime-overlay] AMCL runtime status heartbeat running in background pid=${amcl_status_heartbeat_pid}" >&2
}

complete_amcl_readiness_if_enabled_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  mode="$(amcl_mode_for_navigation)" || return 1
  case "${mode}" in
    disabled)
      return 0
      ;;
  esac
  echo "[runtime-overlay] completing AMCL readiness from accepted triggered localization mode=${mode}" >&2
  if ! run_amcl_localization_step "${mode}" "complete-readiness" --complete-readiness; then
    return 1
  fi
  amcl_runtime_started=1
}

complete_amcl_readiness_with_retries_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  mode="$(amcl_mode_for_navigation)" || return 1
  case "${mode}" in
    disabled)
      echo "[runtime-overlay] AMCL readiness completion skipped because AMCL mode is disabled" >&2
      return 0
      ;;
  esac

  local timeout_sec="${NJRH_AMCL_READINESS_COMPLETION_TIMEOUT_SEC:-45}"
  local retry_sec="${NJRH_AMCL_READINESS_COMPLETION_RETRY_SEC:-3}"
  local deadline=$((SECONDS + timeout_sec))
  local attempt=1
  local rc=1
  local previous_nonfatal="${NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE:-false}"
  export NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE=true
  while true; do
    echo "[runtime-overlay] AMCL readiness completion attempt=${attempt} timeout_sec=${timeout_sec}" >&2
    set +e
    complete_amcl_readiness_if_enabled_for_navigation
    rc=$?
    if [[ "${rc}" -eq 0 ]]; then
      set -e
      export NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE="${previous_nonfatal}"
      echo "[runtime-overlay] AMCL readiness completed" >&2
      return 0
    fi
    set -e
    if (( SECONDS >= deadline )); then
      break
    fi
    echo "[runtime-overlay] AMCL readiness attempt=${attempt} failed rc=${rc}; retrying in ${retry_sec}s" >&2
    sleep "${retry_sec}"
    attempt=$((attempt + 1))
  done
  export NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE="${previous_nonfatal}"
  set_localization_ready_failure \
    "AMCL_READINESS_TIMEOUT" \
    "AMCL did not become ready within ${timeout_sec}s after Nav2 activation; last rc=${rc}: ${AMCL_FAILURE_REASON:-unknown}"
  return "${rc}"
}

scan_flatscan_admission_diagnostics() {
  local scan_info
  local scan_publishers
  local flatscan_process
  local profile
  profile="${NJRH_POINTCLOUD_ACCEL_PROFILE:-unknown}"
  if [[ -f "${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env" ]]; then
    # shellcheck source=../config/pointcloud_accel_profile.env
    source "${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env"
    profile="${NJRH_POINTCLOUD_ACCEL_PROFILE:-${profile}}"
  fi
  scan_info="$(timeout 4 ros2 topic info -v /scan 2>&1 || true)"
  scan_publishers="$(awk -F: '/Publisher count/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}' <<<"${scan_info}")"
  if pgrep -af "laser_scan_to_flatscan" >/dev/null 2>&1; then
    flatscan_process="present"
  else
    flatscan_process="missing"
  fi

  echo "[runtime-overlay] FLATSCAN_MISSING diagnostics:" >&2
  echo "[runtime-overlay]   /scan publisher_count=${scan_publishers:-0}" >&2
  echo "[runtime-overlay]   laser_scan_to_flatscan_process=${flatscan_process}" >&2
  echo "[runtime-overlay]   pointcloud_accel_profile=${profile}" >&2
  echo "[runtime-overlay]   suggested_fix=bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh; bash scripts/jetson/runtime_overlay/scripts/set_pointcloud_accel_profile.sh --profile ${profile} --restart" >&2
}

current_pointcloud_accel_profile() {
  local profile="${NJRH_POINTCLOUD_ACCEL_PROFILE:-}"
  if [[ -f "${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env" ]]; then
    # shellcheck source=../config/pointcloud_accel_profile.env
    source "${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env"
  fi
  printf '%s\n' "${NJRH_POINTCLOUD_ACCEL_PROFILE:-${profile:-ipc_worker}}"
}

topic_publisher_count() {
  local topic="$1"
  timeout 4 ros2 topic info -v "${topic}" 2>/dev/null \
    | awk -F: '/Publisher count/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}'
}

wait_for_flatscan_publisher_ready() {
  local timeout_sec="${1:-10}"
  wait_for_topic_publisher_from_node "/flatscan" "laser_scan_to_flatscan" "${timeout_sec}" || {
    echo "[runtime-overlay] /flatscan publisher from laser_scan_to_flatscan was not ready within ${timeout_sec}s" >&2
    return 1
  }
  echo "[runtime-overlay] /flatscan publisher ready from laser_scan_to_flatscan" >&2
}

start_flatscan_helper_for_navigation_repair() {
  local helper_prefix
  local helper_bin
  local params
  local cpuset
  local log_file
  local existing_pids

  existing_pids="$(pgrep -f "[l]aser_scan_to_flatscan" 2>/dev/null || true)"
  if [[ -n "${existing_pids}" ]]; then
    echo "[runtime-overlay] /flatscan repair stopping stale laser_scan_to_flatscan process without touching pointcloud driver: ${existing_pids}" >&2
    kill -INT ${existing_pids} 2>/dev/null || true
    sleep "${NJRH_FLATSCAN_HELPER_REPAIR_INT_WAIT_SEC:-0.5}"
    existing_pids="$(pgrep -f "[l]aser_scan_to_flatscan" 2>/dev/null || true)"
    [[ -z "${existing_pids}" ]] || kill -TERM ${existing_pids} 2>/dev/null || true
    sleep "${NJRH_FLATSCAN_HELPER_REPAIR_TERM_WAIT_SEC:-0.5}"
  fi
  if ! helper_prefix="$(ros2 pkg prefix jt128_nav_tools 2>/dev/null)"; then
    echo "[runtime-overlay] /flatscan repair failed: jt128_nav_tools package is unavailable" >&2
    return 1
  fi
  helper_bin="${helper_prefix}/lib/jt128_nav_tools/laser_scan_to_flatscan"
  if [[ ! -x "${helper_bin}" ]]; then
    echo "[runtime-overlay] /flatscan repair failed: helper binary missing or not executable: ${helper_bin}" >&2
    return 1
  fi
  params="${NJRH_FLATSCAN_PARAMS:-${NJRH_POINTCLOUD_ACCEL_FLATSCAN_PARAMS:-${NJRH_OVERLAY_ROOT}/config/jt128_flatscan.yaml}}"
  log_file="${NJRH_RUNTIME_LOG_DIR}/flatscan_helper_navigation_repair.log"
  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  cpuset="$(njrh_cpuset_for laser_scan_to_flatscan)"
  echo "[runtime-overlay] starting standalone /flatscan repair helper without restarting pointcloud profile params=${params}" >&2
  if njrh_affinity_enabled && [[ -n "${cpuset}" ]]; then
    echo "[runtime-overlay] cpu affinity: laser_scan_to_flatscan -> CPU ${cpuset}" >&2
    nohup taskset -c "${cpuset}" "${helper_bin}" \
      --ros-args --params-file "${params}" \
      -r scan:=/scan -r flatscan:=/flatscan \
      >"${log_file}" 2>&1 &
  else
    nohup "${helper_bin}" \
      --ros-args --params-file "${params}" \
      -r scan:=/scan -r flatscan:=/flatscan \
      >"${log_file}" 2>&1 &
  fi
  echo "[runtime-overlay] standalone /flatscan repair helper pid=$! log=${log_file}" >&2
}

recover_flatscan_helper_for_navigation() {
  local wait_sec="${1:-30}"
  local scan_publishers
  local profile

  scan_publishers="$(topic_publisher_count /scan)"
  if [[ "${scan_publishers:-0}" -le 0 ]]; then
    echo "[runtime-overlay] /flatscan repair skipped: /scan has no publisher" >&2
    return 1
  fi

  profile="$(current_pointcloud_accel_profile)"
  case "${profile}" in
    ipc_worker|nitros)
      ;;
    legacy)
      echo "[runtime-overlay] /flatscan repair skipped: legacy pointcloud accel profile was removed" >&2
      return 1
      ;;
    *)
      echo "[runtime-overlay] /flatscan repair skipped: invalid pointcloud accel profile=${profile}" >&2
      return 1
      ;;
  esac

  echo "[runtime-overlay] attempting /flatscan repair without restarting pointcloud accel profile=${profile}" >&2
  if ! start_flatscan_helper_for_navigation_repair; then
    echo "[runtime-overlay] /flatscan helper-only repair failed to launch" >&2
    if ! env_flag_true "${NJRH_FLATSCAN_REPAIR_RESTART_POINTCLOUD:-false}"; then
      return 1
    fi
    echo "[runtime-overlay] /flatscan repair fallback: restarting pointcloud accel profile=${profile}" >&2
    if ! timeout "${NJRH_FLATSCAN_REPAIR_RESTART_TIMEOUT_SEC:-20}" \
      bash "${SCRIPT_DIR}/set_pointcloud_accel_profile.sh" --profile "${profile}" --restart >&2; then
      echo "[runtime-overlay] /flatscan repair failed to launch pointcloud accel restart" >&2
      return 1
    fi
  fi

  if wait_for_flatscan_publisher_ready "${wait_sec}"; then
    echo "[runtime-overlay] /flatscan repair succeeded" >&2
    return 0
  fi

  echo "[runtime-overlay] /flatscan repair did not restore /flatscan within ${wait_sec}s" >&2
  return 1
}

set_localization_ready_failure() {
  local reason="$1"
  local detail="$2"
  local failure="${reason}: ${detail}"
  if [[ "${NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE:-false}" == "true" ]]; then
    echo "[runtime-overlay] nonfatal localization startup admission warning: ${failure}" >&2
    return 0
  fi
  localization_ready_failure_reason="${failure}"
  echo "[runtime-overlay] localization startup admission failed: ${localization_ready_failure_reason}" >&2
  write_runtime_map_context "failed" "false" "${localization_ready_failure_reason}"
}

ensure_common_local_state_ready_for_navigation_start() {
  local timeout_sec="${NJRH_INITIAL_LOCAL_STATE_READY_TIMEOUT_SEC:-12}"
  local max_odom_age_sec="${NJRH_NAV_LOCAL_ODOM_MAX_AGE_SEC:-0.75}"
  local max_odom_future_sec="${NJRH_NAV_LOCAL_ODOM_MAX_FUTURE_SEC:-0.25}"
  local max_tf_age_sec="${NJRH_NAV_TF_MAX_AGE_SEC:-0.25}"

  if runtime_health_check "local_state_ready" >/dev/null 2>&1; then
    echo "[runtime-overlay] common local_state ready from runtime health snapshot before navigation startup" >&2
    return 0
  fi

  LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}" \
    local_state_endpoint_ready "${timeout_sec}" || {
      export NJRH_RUNTIME_FAILURE_CODE="LOCAL_STATE_ENDPOINT_NOT_READY"
      set_localization_ready_failure \
        "LOCAL_STATE_ENDPOINT_NOT_READY" \
        "resident robot_local_state endpoint was not ready before AMCL/global localization startup"
      return 1
    }

  wait_for_fresh_header_topic_message \
    "/local_state/odometry" \
    "${timeout_sec}" \
    "${max_odom_age_sec}" \
    "${max_odom_future_sec}" || {
      export NJRH_RUNTIME_FAILURE_CODE="LOCAL_STATE_ODOM_NOT_FRESH"
      set_localization_ready_failure \
        "LOCAL_STATE_ODOM_NOT_FRESH" \
        "/local_state/odometry was missing or stale before AMCL/global localization startup"
      return 1
    }

  wait_for_fresh_tf_transform "odom" "base_link" "${timeout_sec}" "${max_tf_age_sec}" || {
    export NJRH_RUNTIME_FAILURE_CODE="ODOM_BASE_TF_NOT_FRESH"
    set_localization_ready_failure \
      "ODOM_BASE_TF_NOT_FRESH" \
      "odom->base_link was missing or stale before AMCL/global localization startup"
    return 1
  }

  echo "[runtime-overlay] common local_state ready for navigation startup: /local_state/odometry and odom->base_link are fresh" >&2
}

ensure_localization_stack_ready_for_navigation() {
  local service_timeout="${NJRH_INITIAL_LOCALIZATION_SERVICE_WAIT_SEC:-45}"
  local map_server_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_SERVER_WAIT_SEC:-45}"
  local map_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_WAIT_SEC:-45}"
  local flatscan_timeout="${NJRH_INITIAL_LOCALIZATION_FLATSCAN_WAIT_SEC:-5}"
  local flatscan_repair_timeout="${NJRH_INITIAL_LOCALIZATION_FLATSCAN_REPAIR_WAIT_SEC:-20}"
  local publisher_timeout="${NJRH_INITIAL_LOCALIZATION_RESULT_PUBLISHER_WAIT_SEC:-45}"
  local require_result_publisher="${NJRH_INITIAL_LOCALIZATION_REQUIRE_RESULT_PUBLISHER:-false}"
  local stack_timeout="${NJRH_INITIAL_LOCALIZATION_STACK_WAIT_SEC:-45}"

  if runtime_readiness_probe localization-stack \
    "${NAV2_MAP_YAML}" "/flatscan" "${stack_timeout}"; then
    echo "[runtime-overlay] localization stack passed the single-participant startup gate" >&2
    if env_flag_true "${require_result_publisher}"; then
      if ! wait_for_topic_publisher "/localization_result" "${publisher_timeout}"; then
        set_localization_ready_failure "LOCALIZATION_RESULT_PUBLISHER_MISSING" "/localization_result publisher not ready within ${publisher_timeout}s"
        return 1
      fi
    else
      echo "[runtime-overlay] skipping /localization_result publisher pre-gate; trigger wrapper verifies result, bridge acceptance, and map->odom" >&2
    fi
    echo "[runtime-overlay] localization stack ready for initial relocalization" >&2
    return 0
  fi
  echo "[runtime-overlay] falling back to detailed localization readiness diagnostics and FlatScan repair" >&2

  if ! wait_for_ros_service "/global_localization/trigger" "${service_timeout}"; then
    set_localization_ready_failure "GLOBAL_LOCALIZATION_TRIGGER_SERVICE_MISSING" "/global_localization/trigger not ready within ${service_timeout}s"
    return 1
  fi
  if ! wait_for_ros_service "/trigger_grid_search_localization" "${service_timeout}"; then
    set_localization_ready_failure "GRID_SEARCH_LOCALIZATION_SERVICE_MISSING" "/trigger_grid_search_localization not ready within ${service_timeout}s"
    return 1
  fi
  if ! ensure_map_server_active "${NAV2_MAP_YAML:-}" "${map_server_timeout}"; then
    set_localization_ready_failure "MAP_SERVER_NOT_ACTIVE" "/map_server did not publish selected map within ${map_server_timeout}s"
    return 1
  fi
  if ! wait_for_occupancy_grid "/map" "${map_timeout}" >/dev/null; then
    set_localization_ready_failure "MAP_TOPIC_MISSING" "/map OccupancyGrid not ready within ${map_timeout}s"
    return 1
  fi
  if ! wait_for_flatscan_publisher_ready "${flatscan_timeout}"; then
    scan_flatscan_admission_diagnostics
    if ! recover_flatscan_helper_for_navigation "${flatscan_repair_timeout}"; then
      set_localization_ready_failure "FLATSCAN_MISSING" "/flatscan publisher was not ready within ${flatscan_timeout}s and repair did not recover it within ${flatscan_repair_timeout}s"
      return 1
    fi
  fi
  if env_flag_true "${require_result_publisher}"; then
    if ! wait_for_topic_publisher "/localization_result" "${publisher_timeout}"; then
      set_localization_ready_failure "LOCALIZATION_RESULT_PUBLISHER_MISSING" "/localization_result publisher not ready within ${publisher_timeout}s"
      return 1
    fi
  else
    echo "[runtime-overlay] skipping /localization_result publisher pre-gate; trigger wrapper verifies result, bridge acceptance, and map->odom" >&2
  fi
  echo "[runtime-overlay] localization stack ready for initial relocalization" >&2
}

wait_for_nav2_layer_ready() {
  local lifecycle_timeout="${NJRH_NAV_LIFECYCLE_READY_TIMEOUT:-}"
  local costmap_timeout="${NJRH_NAV_GLOBAL_COSTMAP_READY_TIMEOUT:-90}"

  if [[ -z "${lifecycle_timeout}" ]]; then
    if [[ "${NJRH_NAV2_EXTERNAL_LIFECYCLE_BRINGUP:-true}" == "true" ]]; then
      lifecycle_timeout="${NJRH_NAV2_EXTERNAL_LIFECYCLE_READY_TIMEOUT:-210}"
    else
      lifecycle_timeout="90"
    fi
  fi

  ensure_navigation_layer_alive || return 1
  standard_nav_stack_lifecycle_active "${lifecycle_timeout}" || {
    echo "[runtime-overlay] Nav2 lifecycle nodes did not become active" >&2
    return 1
  }
  wait_for_global_costmap_static "${costmap_timeout}" || return 1
  echo "[runtime-overlay] Nav2 lifecycle and global costmap are ready" >&2
}

start_resident_navigation_layer() {
  local lifecycle_hold="$1"
  local stage="$2"
  local verb="$3"
  if [[ -n "${navigation_pid}" ]]; then
    return 0
  fi
  echo "[runtime-overlay] ${verb} resident Nav2 layer for ${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" >&2
  clear_nav2_lifecycle_ready_status
  rm -f "${NJRH_NAV2_HOLD_READY_FILE:-/tmp/njrh_nav2_launch_hold_ready.env}" 2>/dev/null || true
  NJRH_BUILDING_ID="${NJRH_BUILDING_ID}" \
  NJRH_FLOOR_ID="${NJRH_FLOOR_ID}" \
  NJRH_NAV2_LIFECYCLE_HOLD="${lifecycle_hold}" \
  NJRH_SKIP_PRESTART_NAV2_STOP="${NJRH_SKIP_PRESTART_NAV2_STOP:-true}" \
  bash "${SCRIPT_DIR}/run_nav2_navigation.sh" &
  navigation_pid=$!
  log_startup_stage "${stage}"
}

run_nav2_lifecycle_sequence() {
  local timeout_sec="${1:-180}"
  shift || true
  local nodes=("$@")
  local node_timeout="${NJRH_NAV2_LIFECYCLE_NODE_TIMEOUT_SEC:-60}"
  local change_state_response_timeout="${NJRH_NAV2_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC:-5}"
  local kill_after="${NJRH_NAV2_LIFECYCLE_BRINGUP_KILL_AFTER_SEC:-5}"
  local sequence_args=(
    --per-node-timeout-sec "${node_timeout}"
    --change-state-response-timeout-sec "${change_state_response_timeout}"
  )
  echo "[runtime-overlay] Nav2 lifecycle startup method=repo_sequence nodes=${nodes[*]}" >&2
  if [[ "${NJRH_NAV2_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE:-true}" == "true" ]]; then
    sequence_args+=(--trust-change-state-response)
  fi
  if [[ "${NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false}" == "true" ]]; then
    sequence_args+=(--configure-all-before-activate)
  fi

  if [[ "${NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false}" == "true" ]]; then
    local core_nodes=(
      planner_server
      controller_server
      velocity_smoother
      collision_monitor
      behavior_server
      smoother_server
    )
    local pids=()
    local node
    local pid
    local rc=0
    echo "[runtime-overlay] Nav2 lifecycle parallel core activation enabled: ${core_nodes[*]}" >&2
    for node in "${core_nodes[@]}"; do
      run_nav2_lifecycle_sequence_until_active "${timeout_sec}" "${node}" &
      pids+=("$!")
    done
    for pid in "${pids[@]}"; do
      if ! wait "${pid}"; then
        rc=1
      fi
    done
    if [[ "${rc}" -ne 0 ]]; then
      return "${rc}"
    fi
    if [[ "${NJRH_NAV2_LIFECYCLE_PARALLEL_BT:-true}" == "true" ]]; then
      run_nav2_lifecycle_sequence_until_active "${timeout_sec}" bt_navigator
      return $?
    fi
    return 0
  fi

  timeout --kill-after="${kill_after}" "${timeout_sec}" \
    python3 "${SCRIPT_DIR}/nav2_lifecycle_sequence.py" \
      "${sequence_args[@]}" \
      "${nodes[@]}"
}

run_nav2_lifecycle_sequence_until_active() {
  local timeout_sec="${1:-180}"
  shift || true
  local nodes=("$@")
  local node_timeout="${NJRH_NAV2_LIFECYCLE_NODE_TIMEOUT_SEC:-60}"
  local change_state_response_timeout="${NJRH_NAV2_LIFECYCLE_CHANGE_STATE_RESPONSE_TIMEOUT_SEC:-5}"
  local kill_after="${NJRH_NAV2_LIFECYCLE_BRINGUP_KILL_AFTER_SEC:-5}"
  local sequence_args=(
    --per-node-timeout-sec "${node_timeout}"
    --change-state-response-timeout-sec "${change_state_response_timeout}"
  )
  local rc=1

  if [[ "${NJRH_NAV2_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE:-true}" == "true" ]]; then
    sequence_args+=(--trust-change-state-response)
  fi
  if [[ "${NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false}" == "true" ]]; then
    sequence_args+=(--configure-all-before-activate)
  fi

  timeout --kill-after="${kill_after}" "${timeout_sec}" \
    python3 "${SCRIPT_DIR}/nav2_lifecycle_sequence.py" \
      "${sequence_args[@]}" \
      "${nodes[@]}" &
  local helper_pid=$!
  local poll_sec="${NJRH_NAV2_LIFECYCLE_ACTIVE_POLL_INTERVAL_SEC:-0.5}"
  local quick_deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < quick_deadline )); do
    if nav_lifecycle_nodes_active_quick "${nodes[@]}"; then
      echo "[runtime-overlay] lifecycle nodes active; stopping lifecycle helper pid=${helper_pid}: ${nodes[*]}" >&2
      kill -TERM "${helper_pid}" 2>/dev/null || true
      wait "${helper_pid}" 2>/dev/null || true
      return 0
    fi
    if ! ps -p "${helper_pid}" >/dev/null 2>&1; then
      break
    fi
    sleep "${poll_sec}"
  done
  if wait "${helper_pid}"; then
    rc=0
  else
    rc=$?
  fi
  if [[ "${rc}" -eq 0 ]]; then
    return 0
  fi

  if nav_lifecycle_nodes_active_quick "${nodes[@]}"; then
    echo "[runtime-overlay] lifecycle helper exited rc=${rc}, but nodes are active: ${nodes[*]}" >&2
    return 0
  fi
  return "${rc}"
}

activate_prestarted_nav2_lifecycle() {
  [[ "${nav2_prestarted}" -eq 1 ]] || return 0
  if [[ "${nav2_lifecycle_background_started}" -eq 1 ]]; then
    if wait_for_prestarted_nav2_lifecycle_background; then
      write_nav2_lifecycle_ready_status "${navigation_pid}" "resident_background_sequence"
      return 0
    fi
    return 1
  fi
  local timeout_sec="${NJRH_NAV2_LIFECYCLE_BRINGUP_TIMEOUT_SEC:-180}"
  local nodes=(
    planner_server
    controller_server
    velocity_smoother
    collision_monitor
    behavior_server
    smoother_server
    bt_navigator
  )
  ensure_navigation_layer_alive || return 1
  echo "[runtime-overlay] activating prestarted Nav2 lifecycle with retrying lifecycle sequence timeout=${timeout_sec}s" >&2
  log_startup_stage "nav2_lifecycle_activation_started"
  run_nav2_lifecycle_sequence "${timeout_sec}" "${nodes[@]}" || {
      echo "[runtime-overlay] prestarted Nav2 lifecycle sequence failed or timed out" >&2
      return 1
    }
  write_nav2_lifecycle_ready_status "${navigation_pid}" "resident_foreground_sequence"
  echo "[runtime-overlay] lifecycle_manager_navigation external lifecycle sequence: Managed nodes are active" >&2
}

wait_for_prestarted_nav2_launch_hold_ready() {
  local timeout_sec="${NJRH_NAV2_PRESTART_HOLD_READY_TIMEOUT_SEC:-25}"
  local max_age_sec="${NJRH_NAV2_PRESTART_HOLD_READY_MAX_AGE_SEC:-60}"
  local status_file="${NJRH_NAV2_HOLD_READY_FILE:-/tmp/njrh_nav2_launch_hold_ready.env}"
  local deadline=$((SECONDS + timeout_sec))
  local now_sec
  local age_sec
  while (( SECONDS < deadline )); do
    ensure_navigation_layer_alive || return 1
    if [[ -f "${status_file}" ]]; then
      NAV2_HOLD_READY=""
      NAV2_HOLD_READY_STAMP_SEC=""
      NAV2_HOLD_READY_WRAPPER_PID=""
      NAV2_HOLD_READY_BASHPID=""
      NAV2_HOLD_READY_CONTROLLER_PID=""
      # shellcheck disable=SC1090
      source "${status_file}" 2>/dev/null || true
      now_sec="$(date +%s)"
      age_sec=$((now_sec - ${NAV2_HOLD_READY_STAMP_SEC:-0}))
      if [[ "${NAV2_HOLD_READY:-false}" == "true" ]] \
        && [[ "${NAV2_HOLD_READY_WRAPPER_PID:-}" == "${navigation_pid}" || "${NAV2_HOLD_READY_BASHPID:-}" == "${navigation_pid}" ]] \
        && (( age_sec <= max_age_sec )) \
        && [[ -n "${NAV2_HOLD_READY_CONTROLLER_PID:-}" ]] \
        && kill -0 "${NAV2_HOLD_READY_CONTROLLER_PID}" 2>/dev/null; then
        echo "[runtime-overlay] prestarted Nav2 held launch ready from ${status_file}: wrapper_pid=${navigation_pid} controller_pid=${NAV2_HOLD_READY_CONTROLLER_PID}; lifecycle activation may start" >&2
        return 0
      fi
    fi
    sleep 0.5
  done
  echo "[runtime-overlay] prestarted Nav2 held launch did not write a matching ready status within ${timeout_sec}s file=${status_file} wrapper_pid=${navigation_pid}" >&2
  return 1
}

start_prestarted_nav2_lifecycle_background() {
  [[ "${nav2_prestarted}" -eq 1 ]] || return 0
  [[ "${nav2_lifecycle_background_started}" -eq 0 ]] || return 0
  local timeout_sec="${NJRH_NAV2_LIFECYCLE_BRINGUP_TIMEOUT_SEC:-180}"
  local nodes=(
    planner_server
    controller_server
    velocity_smoother
    collision_monitor
    behavior_server
    smoother_server
    bt_navigator
  )
  ensure_navigation_layer_alive || return 1
  wait_for_prestarted_nav2_launch_hold_ready || return 1
  echo "[runtime-overlay] starting prestarted Nav2 lifecycle in background with retrying lifecycle sequence timeout=${timeout_sec}s" >&2
  log_startup_stage "nav2_lifecycle_activation_started"
  (
    run_nav2_lifecycle_sequence "${timeout_sec}" "${nodes[@]}"
  ) &
  nav2_lifecycle_bringup_pid=$!
  nav2_lifecycle_background_started=1
  echo "[runtime-overlay] Nav2 lifecycle activation running in background pid=${nav2_lifecycle_bringup_pid}" >&2
}

wait_for_prestarted_nav2_lifecycle_background() {
  if [[ "${nav2_lifecycle_background_started}" -ne 1 ]]; then
    return 0
  fi
  if [[ -z "${nav2_lifecycle_bringup_pid}" ]]; then
    return 0
  fi
  local nodes=(
    planner_server
    controller_server
    velocity_smoother
    collision_monitor
    behavior_server
    smoother_server
    bt_navigator
  )
  local rc=0
  local poll_sec="${NJRH_NAV2_LIFECYCLE_BACKGROUND_ACTIVE_POLL_SEC:-0.5}"
  local active_wait_sec="${NJRH_NAV2_LIFECYCLE_BACKGROUND_ACTIVE_WAIT_SEC:-45}"
  local deadline=$((SECONDS + active_wait_sec))

  while (( SECONDS < deadline )); do
    if nav_lifecycle_nodes_active_quick "${nodes[@]}"; then
      echo "[runtime-overlay] prestarted Nav2 lifecycle nodes active before background helper exit; stopping helper pid=${nav2_lifecycle_bringup_pid}" >&2
      kill -TERM "${nav2_lifecycle_bringup_pid}" 2>/dev/null || true
      wait "${nav2_lifecycle_bringup_pid}" 2>/dev/null || true
      nav2_lifecycle_bringup_pid=""
      nav2_lifecycle_background_started=0
      echo "[runtime-overlay] lifecycle_manager_navigation external lifecycle sequence: Managed nodes are active" >&2
      return 0
    fi
    if ! ps -p "${nav2_lifecycle_bringup_pid}" >/dev/null 2>&1; then
      break
    fi
    sleep "${poll_sec}"
  done

  if wait "${nav2_lifecycle_bringup_pid}"; then
    rc=0
  else
    rc=$?
  fi
  nav2_lifecycle_bringup_pid=""
  nav2_lifecycle_background_started=0
  if [[ "${rc}" -ne 0 ]]; then
    if nav_lifecycle_nodes_active_quick "${nodes[@]}"; then
      echo "[runtime-overlay] prestarted Nav2 lifecycle helper exited rc=${rc}, but managed nodes are active" >&2
      echo "[runtime-overlay] lifecycle_manager_navigation external lifecycle sequence: Managed nodes are active" >&2
      return 0
    fi
    echo "[runtime-overlay] prestarted Nav2 lifecycle sequence background failed with ${rc}" >&2
    return "${rc}"
  fi
  echo "[runtime-overlay] lifecycle_manager_navigation external lifecycle sequence: Managed nodes are active" >&2
}

ensure_helper_process_no_probe() {
  local helper_name="$1"
  shift
  local helper_pattern=""
  helper_pattern="$(helper_process_pattern "${helper_name}" 2>/dev/null || true)"
  if [[ -n "${helper_pattern}" ]] && helper_process_running "${helper_pattern}"; then
    echo "[runtime-overlay] ${helper_name} process already running; skipping readiness probe" >&2
    return 0
  fi
  echo "[runtime-overlay] starting ${helper_name} without readiness probe" >&2
  "$@" >>"${NJRH_RUNTIME_LOG_DIR}/${helper_name}.log" 2>&1 &
  helper_pids+=("$!")
  sleep "${NJRH_NAV_HELPER_START_SETTLE_SEC:-0.5}"
}

ensure_global_localization_wrapper_resident() {
  local helper_name="global_localization_localization"
  local service_timeout="${NJRH_GLOBAL_LOCALIZATION_RESIDENT_SERVICE_WAIT_SEC:-10}"
  local helper_pattern=""
  helper_pattern="$(helper_process_pattern "${helper_name}" 2>/dev/null || true)"
  if [[ -n "${helper_pattern}" ]] && helper_process_running "${helper_pattern}"; then
    echo "[runtime-overlay] resident global localization wrapper already running" >&2
  else
    echo "[runtime-overlay] resident global localization wrapper missing after startup trigger; starting persistent wrapper" >&2
    ensure_helper_process_no_probe "${helper_name}" bash "${SCRIPT_DIR}/run_global_localization.sh"
  fi
  if ! wait_for_ros_service "/global_localization/trigger" "${service_timeout}"; then
    set_localization_ready_failure \
      "GLOBAL_LOCALIZATION_RESIDENT_SERVICE_MISSING" \
      "/global_localization/trigger not ready after resident wrapper check within ${service_timeout}s"
    return 1
  fi
}

ensure_localization_layer_alive() {
  if [[ -z "${localization_pid}" ]]; then
    echo "[runtime-overlay] localization layer pid is not set" >&2
    return 1
  fi
  if kill -0 "${localization_pid}" 2>/dev/null; then
    return 0
  fi
  wait "${localization_pid}" || exit_code=$?
  echo "[runtime-overlay] resident localization layer exited before Nav2 activation with ${exit_code}" >&2
  write_runtime_map_context "failed" "false" "resident localization layer exited before Nav2 activation with ${exit_code}"
  return 1
}

ensure_navigation_layer_alive() {
  if [[ -z "${navigation_pid}" ]]; then
    echo "[runtime-overlay] navigation layer pid is not set" >&2
    return 1
  fi
  if kill -0 "${navigation_pid}" 2>/dev/null; then
    return 0
  fi
  wait "${navigation_pid}" || exit_code=$?
  echo "[runtime-overlay] resident Nav2 layer exited before runtime ready with ${exit_code}" >&2
  write_runtime_map_context "failed" "false" "resident Nav2 layer exited before runtime ready with ${exit_code}"
  return 1
}

if resident_navigation_ready; then
  echo "[runtime-overlay] resident navigation runtime already ready for ${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" >&2
  log_startup_stage "resident_context_reused"
  write_runtime_map_context "ready" "true" "existing resident navigation context matches selected floor"
  runtime_ready=1
  while true; do
    sleep 3600
  done
fi

write_runtime_map_context "starting" "false" "resident navigation runtime starting"
echo "[runtime-overlay] starting resident navigation localization layer for ${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" >&2
NJRH_BUILDING_ID="${NJRH_BUILDING_ID}" \
NJRH_FLOOR_ID="${NJRH_FLOOR_ID}" \
bash "${SCRIPT_DIR}/run_occupancy_grid_localization.sh" &
localization_pid=$!
log_startup_stage "localization_layer_started"
if [[ "${NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-false}" == "true" ]]; then
  echo "[runtime-overlay] prestarting resident AMCL warmup before initial global localization" >&2
  start_amcl_resident_background_if_enabled_for_navigation || {
    echo "[runtime-overlay] AMCL resident warmup background failed to launch; readiness phase will retry after initial localization" >&2
  }
else
  echo "[runtime-overlay] deferring resident AMCL warmup until after initial global localization" >&2
fi

if [[ "${NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION:-false}" == "true" ]]; then
  start_resident_navigation_layer "true" "nav2_layer_prestarted" "prestarting"
  nav2_prestarted=1
  sleep "${NJRH_NAV2_PRESTART_SETTLE_SEC:-0.1}"
  ensure_navigation_layer_alive || exit 1
  log_startup_stage "nav2_layer_started"
  if [[ "${NJRH_NAV2_LIFECYCLE_BACKGROUND_START:-true}" == "true" ]]; then
    start_prestarted_nav2_lifecycle_background || {
      write_runtime_map_context "failed" "false" "prestarted resident Nav2 lifecycle background activation failed to launch"
      echo "[runtime-overlay] prestarted resident Nav2 lifecycle background activation failed to launch; localization layer remains running for diagnostics and retry" >&2
      stop_navigation_layer_after_failure "lifecycle background launch failure"
    }
  fi
fi

ensure_common_local_state_ready_for_navigation_start || {
  write_runtime_map_context "failed" "false" "${localization_ready_failure_reason:-resident robot_local_state was not ready before initial localization}"
  exit 1
}
log_startup_stage "common_local_state_ready"

if [[ -z "${navigation_pid}" ]] && env_flag_true "${NJRH_NAV2_HELD_PRESTART_AFTER_LOCAL_STATE:-true}"; then
  if env_flag_true "${NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE:-true}"; then
    localizer_prestart_service_timeout="${NJRH_NAV2_HELD_PRESTART_LOCALIZER_SERVICE_WAIT_SEC:-45}"
    echo "[runtime-overlay] prioritizing localization startup until Isaac service and bridge odom are ready before held Nav2 prestart" >&2
    ensure_localization_layer_alive || exit 1
    if ! runtime_readiness_probe localization-prestart "${localizer_prestart_service_timeout}"; then
      set_localization_ready_failure \
        "LOCALIZATION_PRESTART_NOT_READY" \
        "Isaac service or localization bridge odom not ready before held Nav2 prestart within ${localizer_prestart_service_timeout}s"
      write_runtime_map_context "failed" "false" "${localization_ready_failure_reason}"
      exit 1
    fi
    log_startup_stage "localizer_service_ready_for_nav2_prestart"
  fi
  start_resident_navigation_layer "true" "nav2_layer_prestarted_held" "prestarting held"
  nav2_prestarted=1
  sleep "${NJRH_NAV2_PRESTART_SETTLE_SEC:-0.1}"
  ensure_navigation_layer_alive || exit 1
fi

if env_flag_true "${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}"; then
  start_initial_global_localization_background
else
  echo "[runtime-overlay] initial global localization trigger will run after localization stack and floor context are ready" >&2
fi

sleep "${NJRH_NAV_LOCALIZATION_START_SETTLE_SEC:-0.1}"
ensure_localization_layer_alive || exit 1
ensure_localization_stack_ready_for_navigation || {
  write_runtime_map_context "failed" "false" "${localization_ready_failure_reason:-resident localization stack did not become ready before initial relocalization}"
  exit 1
}
log_startup_stage "localization_stack_ready"

if [[ "${nav2_prestarted}" -eq 1 ]] \
  && [[ "${nav2_lifecycle_background_started}" -eq 0 ]] \
  && env_flag_true "${NJRH_NAV2_LIFECYCLE_BACKGROUND_START:-true}" \
  && env_flag_true "${NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false}"; then
  echo "[runtime-overlay] starting prestarted Nav2 lifecycle background after localization stack readiness; final ready still waits for bridge map->odom and active Nav2" >&2
  start_prestarted_nav2_lifecycle_background || {
    echo "[runtime-overlay] prestarted Nav2 lifecycle background after localization stack readiness did not launch; foreground activation will retry after initial localization" >&2
  }
fi

ensure_helper_process_no_probe "floor_manager" bash "${SCRIPT_DIR}/run_floor_manager.sh"

echo "[runtime-overlay] selecting floor context in floor_manager; resident localization layer already owns map/localizer loading" >&2
payload="{building_id: '${NJRH_BUILDING_ID}', floor_id: '${NJRH_FLOOR_ID}', resume_navigation: false}"
timeout "${NJRH_FLOOR_MANAGER_SWITCH_CALL_TIMEOUT:-0.2}" ros2 service call /floor_manager/switch_floor robot_interfaces/srv/SwitchFloor "${payload}" >/dev/null 2>&1 || {
  echo "[runtime-overlay] floor switch request did not complete; continuing because selected floor assets were already resolved by runtime" >&2
}
log_startup_stage "floor_context_selected"

wait_for_initial_global_localization || {
  write_runtime_map_context "failed" "false" "initial global localization did not pass trigger wrapper, bridge, and map->odom gates"
  exit 1
}
log_startup_stage "initial_global_localization_ready"
if env_flag_true "${NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY:-false}" || \
  env_flag_true "${NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE:-false}"; then
  echo "[runtime-overlay] starting AMCL readiness in parallel with Nav2 lifecycle activation" >&2
  start_amcl_readiness_background_if_enabled_for_navigation || {
    echo "[runtime-overlay] AMCL readiness background failed to launch; readiness completion will retry after Nav2 ready" >&2
  }
  log_startup_stage "amcl_readiness_started"
else
  echo "[runtime-overlay] deferring AMCL readiness until after Nav2 lifecycle activation; runtime ready is gated by bridge map->odom and Nav2 active state" >&2
  log_startup_stage "amcl_readiness_deferred"
fi
ensure_localization_layer_alive || {
  write_runtime_map_context "failed" "false" "resident localization layer exited during initial relocalization"
  exit 1
}

if [[ -z "${navigation_pid}" ]]; then
  start_resident_navigation_layer "false" "nav2_layer_started" "starting"
  log_startup_stage "nav2_layer_started_after_initial_localization"
elif [[ "${nav2_lifecycle_background_started}" -eq 0 ]]; then
  log_startup_stage "nav2_layer_started"
fi

sleep "${NJRH_NAV_RUNTIME_READY_MARK_DELAY_SEC:-0.0}"
ensure_localization_layer_alive || exit 1
ensure_navigation_layer_alive || exit 1
if ! activate_prestarted_nav2_lifecycle; then
  write_runtime_map_context "failed" "false" "prestarted resident Nav2 lifecycle activation failed after initial relocalization"
  echo "[runtime-overlay] prestarted resident Nav2 lifecycle activation failed; localization layer remains running for diagnostics and retry" >&2
  stop_navigation_layer_after_failure "lifecycle activation failure"
fi
if ! wait_for_nav2_layer_ready; then
  write_runtime_map_context "failed" "false" "resident Nav2 layer did not become ready after initial relocalization"
  echo "[runtime-overlay] resident Nav2 layer failed startup; localization layer remains running for diagnostics and retry" >&2
  stop_navigation_layer_after_failure "startup failure"
else
  log_startup_stage "nav2_layer_ready"
  if env_flag_true "${NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY:-false}"; then
    if ! wait_for_amcl_readiness_background_if_running; then
      echo "[runtime-overlay] AMCL readiness background did not complete cleanly; foreground readiness will restart or repair it" >&2
      if ! complete_amcl_readiness_with_retries_for_navigation; then
        write_runtime_map_context "failed" "false" "resident navigation runtime did not reach AMCL tracking readiness after Nav2 activation"
        echo "[runtime-overlay] resident navigation runtime failed AMCL readiness; localization and Nav2 remain available for diagnostics" >&2
        exit 1
      fi
    fi
    load_amcl_runtime_status
    if [[ "${NJRH_AMCL_LOCALIZATION_MODE:-disabled}" != "disabled" && "${AMCL_READY:-false}" != "true" ]]; then
      write_runtime_map_context "failed" "false" "resident navigation runtime did not reach AMCL tracking readiness after Nav2 activation"
      echo "[runtime-overlay] resident navigation runtime failed AMCL readiness; localization and Nav2 remain available for diagnostics" >&2
      exit 1
    fi
    log_startup_stage "amcl_tracking_ready"
  else
    if [[ -z "${amcl_readiness_pid}" ]]; then
      start_amcl_readiness_background_if_enabled_for_navigation || {
        echo "[runtime-overlay] AMCL readiness background failed to launch after Nav2 ready; continuing with bridge map->odom and Nav2 active runtime ready" >&2
      }
    fi
    load_amcl_runtime_status
    local_ready_message="resident navigation runtime ready after trigger wrapper, bridge map->odom, and Nav2 activation; AMCL tracking continues in background"
  fi
  start_amcl_status_heartbeat_if_enabled_for_navigation || {
    write_runtime_map_context "failed" "false" "resident navigation runtime could not start AMCL runtime status heartbeat"
    echo "[runtime-overlay] AMCL runtime status heartbeat failed to launch" >&2
    exit 1
  }
  ensure_global_localization_wrapper_resident || {
    write_runtime_map_context "failed" "false" "${localization_ready_failure_reason:-resident global localization wrapper was not available after startup}"
    echo "[runtime-overlay] resident global localization wrapper failed to stay available after startup" >&2
    exit 1
  }
  runtime_ready=1
  if env_flag_true "${NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY:-false}"; then
    write_runtime_map_context "ready" "true" "resident navigation runtime ready after trigger wrapper, bridge map->odom, Nav2 activation, and AMCL tracking readiness"
  else
    write_runtime_map_context "ready" "true" "${local_ready_message}"
    if [[ "${NJRH_AMCL_LOCALIZATION_MODE:-disabled}" != "disabled" && "${AMCL_READY:-false}" == "true" ]]; then
      log_startup_stage "amcl_tracking_ready"
    else
      echo "[runtime-overlay] AMCL tracking readiness continues in background; runtime ready is gated by bridge map->odom and Nav2 active state" >&2
      log_startup_stage "amcl_background_warming"
    fi
  fi
  echo "[runtime-overlay] resident navigation runtime launched for ${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" >&2
fi

while true; do
  if [[ -n "${amcl_status_heartbeat_pid}" ]] && ! kill -0 "${amcl_status_heartbeat_pid}" 2>/dev/null; then
    wait "${amcl_status_heartbeat_pid}" || exit_code=$?
    echo "[runtime-overlay] AMCL runtime status heartbeat exited with ${exit_code}" >&2
    write_runtime_map_context "failed" "false" "AMCL runtime status heartbeat exited with ${exit_code}"
    runtime_ready=0
    exit "${exit_code}"
  fi
  if [[ -n "${localization_pid}" ]] && ! kill -0 "${localization_pid}" 2>/dev/null; then
    wait "${localization_pid}" || exit_code=$?
    echo "[runtime-overlay] resident localization layer exited with ${exit_code}" >&2
    write_runtime_map_context "failed" "false" "resident localization layer exited with ${exit_code}"
    runtime_ready=0
    exit "${exit_code}"
  fi
  if [[ -n "${navigation_pid}" ]] && ! kill -0 "${navigation_pid}" 2>/dev/null; then
    wait "${navigation_pid}" || exit_code=$?
    echo "[runtime-overlay] resident Nav2 layer exited with ${exit_code}" >&2
    write_runtime_map_context "failed" "false" "resident Nav2 layer exited with ${exit_code}"
    runtime_ready=0
    stop_navigation_layer_after_failure "exit"
  fi
  sleep 2
done

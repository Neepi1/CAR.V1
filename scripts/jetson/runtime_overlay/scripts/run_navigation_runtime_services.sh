#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/commercial_runtime_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"

building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
floor_id="${2:-${NJRH_FLOOR_ID:-}}"
export NJRH_RUNTIME_MAP_CONTEXT_FILE="${NJRH_RUNTIME_MAP_CONTEXT_FILE:-/tmp/njrh_runtime_map_context.json}"
export NJRH_RUNTIME_FAILURE_CODE=""
export NJRH_RUNTIME_LOCALIZATION_MODE="${NJRH_ISAAC_LOCALIZATION_MODE:-triggered}"
export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK=""
export NJRH_RUNTIME_MAP_TO_ODOM_AGE_MS=""

[[ -n "${floor_id}" ]] || {
  echo "[runtime-overlay] floor_id is required for resident navigation runtime" >&2
  echo "[runtime-overlay] keep common services running in NO_MAP mode until a released floor asset is selected" >&2
  exit 2
}

resolve_floor_assets "${building_id}" "${floor_id}"

localization_pid=""
navigation_pid=""
amcl_runtime_started=0
exit_code=0
runtime_ready=0
cleanup_started=0
localization_ready_failure_reason=""

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
  terminate_child "${navigation_pid}" "resident Nav2 layer"
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

resident_navigation_ready() {
  [[ "${NJRH_NAV_REUSE_READY_CONTEXT:-false}" == "true" ]] || return 1
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

trigger_global_localization_for_navigation() {
  local reason="resident_navigation_start:${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}"
  local call_timeout="${NJRH_GLOBAL_LOCALIZATION_TRIGGER_CALL_TIMEOUT:-60}"
  local bridge_timeout="${NJRH_INITIAL_LOCALIZATION_BRIDGE_ACCEPT_WAIT_SEC:-25}"
  local tf_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_ODOM_WAIT_SEC:-20}"
  local payload="{reason: '${reason}'}"
  local trigger_output
  local wrapper_code
  local runtime_code

  echo "[runtime-overlay] requesting global localization through wrapper and waiting for bridge/map->odom" >&2
  if ! trigger_output="$(timeout "${call_timeout}" ros2 service call \
    /global_localization/trigger \
    robot_interfaces/srv/TriggerLocalization \
    "${payload}" 2>&1)"; then
    wrapper_code="$(extract_failure_code "${trigger_output}")"
    runtime_code="$(runtime_failure_code_for_wrapper_code "${wrapper_code:-ISAAC_SERVICE_TIMEOUT}")"
    export NJRH_RUNTIME_FAILURE_CODE="${runtime_code}"
    export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK="false"
    set_localization_ready_failure \
      "${runtime_code}" \
      "/global_localization/trigger did not complete within ${call_timeout}s; wrapper_code=${wrapper_code:-none}; output=${trigger_output}"
    return 1
  fi

  if ! grep -Eq 'accepted[=:][[:space:]]*(True|true)|accepted:[[:space:]]*true' <<<"${trigger_output}"; then
    wrapper_code="$(extract_failure_code "${trigger_output}")"
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

start_amcl_if_enabled_for_navigation() {
  local mode="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
  case "${mode}" in
    disabled)
      echo "[runtime-overlay] AMCL continuous localization disabled" >&2
      return 0
      ;;
    shadow|gated)
      ;;
    *)
      set_localization_ready_failure "AMCL_MODE_INVALID" "invalid NJRH_AMCL_LOCALIZATION_MODE=${mode}"
      return 1
      ;;
  esac
  echo "[runtime-overlay] starting AMCL continuous localization candidate mode=${mode}" >&2
  NJRH_AMCL_LOCALIZATION_MODE="${mode}" \
    bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --mode "${mode}" --restart
  amcl_runtime_started=1
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

set_localization_ready_failure() {
  local reason="$1"
  local detail="$2"
  localization_ready_failure_reason="${reason}: ${detail}"
  echo "[runtime-overlay] localization startup admission failed: ${localization_ready_failure_reason}" >&2
  write_runtime_map_context "failed" "false" "${localization_ready_failure_reason}"
}

ensure_localization_stack_ready_for_navigation() {
  local service_timeout="${NJRH_INITIAL_LOCALIZATION_SERVICE_WAIT_SEC:-45}"
  local map_server_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_SERVER_WAIT_SEC:-45}"
  local map_timeout="${NJRH_INITIAL_LOCALIZATION_MAP_WAIT_SEC:-45}"
  local flatscan_timeout="${NJRH_INITIAL_LOCALIZATION_FLATSCAN_WAIT_SEC:-45}"
  local publisher_timeout="${NJRH_INITIAL_LOCALIZATION_RESULT_PUBLISHER_WAIT_SEC:-45}"

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
  if ! wait_for_topic_message "/flatscan" "${flatscan_timeout}"; then
    scan_flatscan_admission_diagnostics
    set_localization_ready_failure "FLATSCAN_MISSING" "/flatscan FlatScan message not ready within ${flatscan_timeout}s"
    return 1
  fi
  if ! wait_for_topic_publisher "/localization_result" "${publisher_timeout}"; then
    set_localization_ready_failure "LOCALIZATION_RESULT_PUBLISHER_MISSING" "/localization_result publisher not ready within ${publisher_timeout}s"
    return 1
  fi
  echo "[runtime-overlay] localization stack ready for initial relocalization" >&2
}

wait_for_nav2_layer_ready() {
  local lifecycle_timeout="${NJRH_NAV_LIFECYCLE_READY_TIMEOUT:-90}"
  local costmap_timeout="${NJRH_NAV_GLOBAL_COSTMAP_READY_TIMEOUT:-90}"

  ensure_navigation_layer_alive || return 1
  standard_nav_stack_lifecycle_active "${lifecycle_timeout}" || {
    echo "[runtime-overlay] Nav2 lifecycle nodes did not become active" >&2
    return 1
  }
  wait_for_global_costmap_static "${costmap_timeout}" || return 1
  echo "[runtime-overlay] Nav2 lifecycle and global costmap are ready" >&2
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

sleep "${NJRH_NAV_LOCALIZATION_START_SETTLE_SEC:-3}"
ensure_localization_layer_alive || exit 1
ensure_localization_stack_ready_for_navigation || {
  write_runtime_map_context "failed" "false" "${localization_ready_failure_reason:-resident localization stack did not become ready before initial relocalization}"
  exit 1
}

ensure_helper_process_no_probe "floor_manager" bash "${SCRIPT_DIR}/run_floor_manager.sh"

echo "[runtime-overlay] selecting floor context in floor_manager; resident localization layer already owns map/localizer loading" >&2
payload="{building_id: '${NJRH_BUILDING_ID}', floor_id: '${NJRH_FLOOR_ID}', resume_navigation: false}"
timeout "${NJRH_FLOOR_MANAGER_SWITCH_CALL_TIMEOUT:-8}" ros2 service call /floor_manager/switch_floor robot_interfaces/srv/SwitchFloor "${payload}" >/dev/null 2>&1 || {
  echo "[runtime-overlay] floor switch request did not complete; continuing because selected floor assets were already resolved by runtime" >&2
}

trigger_global_localization_for_navigation || {
  write_runtime_map_context "failed" "false" "initial global localization did not pass trigger wrapper, bridge, and map->odom gates"
  exit 1
}
start_amcl_if_enabled_for_navigation || {
  write_runtime_map_context "failed" "false" "AMCL candidate localization failed to start after initial global localization"
  exit 1
}
ensure_localization_layer_alive || {
  write_runtime_map_context "failed" "false" "resident localization layer exited during initial relocalization"
  exit 1
}

echo "[runtime-overlay] starting resident Nav2 layer for ${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" >&2
NJRH_BUILDING_ID="${NJRH_BUILDING_ID}" \
NJRH_FLOOR_ID="${NJRH_FLOOR_ID}" \
bash "${SCRIPT_DIR}/run_nav2_navigation.sh" &
navigation_pid=$!

sleep "${NJRH_NAV_RUNTIME_READY_MARK_DELAY_SEC:-1}"
ensure_localization_layer_alive || exit 1
ensure_navigation_layer_alive || exit 1
wait_for_nav2_layer_ready || {
  write_runtime_map_context "failed" "false" "resident Nav2 layer did not become ready after initial relocalization"
  exit 1
}
runtime_ready=1
write_runtime_map_context "ready" "true" "resident navigation runtime ready after trigger wrapper, bridge map->odom, and Nav2 activation"
echo "[runtime-overlay] resident navigation runtime launched for ${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" >&2

while true; do
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
    exit "${exit_code}"
  fi
  sleep 2
done

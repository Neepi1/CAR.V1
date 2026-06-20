#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile
njrh_load_pointcloud_ingress_profile

common_pids=()
runtime_health_guard_started=0
NAV_LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}"
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
    sleep "${NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC:-1}"
    pids="$(stale_amcl_heartbeat_pids)"
    [[ -z "${pids}" ]] || kill -TERM ${pids} 2>/dev/null || true
  fi
  pids="$(stale_amcl_seed_helper_pids)"
  if [[ -n "${pids}" ]]; then
    echo "[runtime-overlay] stopping stale AMCL seed helper before common startup: ${pids}" >&2
    kill -INT ${pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_SEED_HELPER_STOP_INT_WAIT_SEC:-0.5}"
    pids="$(stale_amcl_seed_helper_pids)"
    [[ -z "${pids}" ]] || kill -TERM ${pids} 2>/dev/null || true
    sleep "${NJRH_AMCL_SEED_HELPER_STOP_TERM_WAIT_SEC:-0.5}"
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

  if reuse_common_services_enabled && pgrep -f "${pattern}" >/dev/null 2>&1; then
    echo "[runtime-overlay] reusing existing ${name}; pattern=${pattern}" >&2
    return 0
  fi

  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  echo "[runtime-overlay] starting ${name}" >&2
  "$@" >>"${log_file}" 2>&1 &
  local pid=$!
  common_pids+=("${pid}")
  sleep "${NJRH_COMMON_PROCESS_START_SETTLE_SEC:-0.2}"
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[runtime-overlay] common service failed to stay alive: ${name}. Check ${log_file}" >&2
    return 1
  fi
  echo "[runtime-overlay] common service ready: ${name} (pid=${pid})" >&2
}

canonical_jt128_ingress_running() {
  local pointcloud_pipeline_pattern="pointcloud_perception_pipeline.launch.py|component_container_mt.*pointcloud_perception_pipeline|pointcloud_perception_pipeline"
  local pointcloud_standalone_pattern="pointcloud_axis_remap|pointcloud_accel_axis"
  if [[ "${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}" == "driver_integrated" ]]; then
    pgrep -f "hesai_accel_driver_node" >/dev/null 2>&1 &&
      pgrep -f "imu_axis_remap" >/dev/null 2>&1
  else
    pgrep -f "hesai_ros_driver_node" >/dev/null 2>&1 &&
      { pgrep -f "${pointcloud_pipeline_pattern}" >/dev/null 2>&1 || pgrep -f "${pointcloud_standalone_pattern}" >/dev/null 2>&1; } &&
      pgrep -f "imu_axis_remap" >/dev/null 2>&1
  fi
}

pointcloud_accel_pipeline_aux_running() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  pgrep -f "run_pointcloud_accel_pipeline.sh|laser_scan_to_flatscan" >/dev/null 2>&1
}

process_count_for_pattern() {
  { pgrep -f "$1" 2>/dev/null || true; } | wc -l | tr -d '[:space:]'
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
  start_common_process "runtime_health_guard" "runtime_health_guard.py|run_runtime_health_guard.sh" \
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
    sleep 0.2
  done
  echo "[runtime-overlay] runtime health did not confirm local_state_ready within ${timeout_sec}s; continuing because robot_local_state direct readiness already passed" >&2
  return 0
}

wait_for_runtime_health_local_state_endpoint_ready() {
  local timeout_sec="${NJRH_RUNTIME_HEALTH_LOCAL_STATE_ENDPOINT_TIMEOUT_SEC:-3}"
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
    "${NJRH_COMMON_AMCL_STOP_TIMEOUT_SEC:-3}" \
    env \
      NJRH_AMCL_LIFECYCLE_SHUTDOWN_TIMEOUT_SEC="${NJRH_AMCL_LIFECYCLE_SHUTDOWN_TIMEOUT_SEC:-1}" \
      NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC="${NJRH_AMCL_HEARTBEAT_STOP_INT_WAIT_SEC:-0.1}" \
      NJRH_AMCL_RUNNER_STOP_INT_WAIT_SEC="${NJRH_AMCL_RUNNER_STOP_INT_WAIT_SEC:-0.1}" \
      NJRH_AMCL_RUNNER_STOP_TERM_WAIT_SEC="${NJRH_AMCL_RUNNER_STOP_TERM_WAIT_SEC:-0.1}" \
      NJRH_AMCL_RUNNER_STOP_KILL_WAIT_SEC="${NJRH_AMCL_RUNNER_STOP_KILL_WAIT_SEC:-0.1}" \
      NJRH_AMCL_SCAN_ADMISSION_STOP_INT_WAIT_SEC="${NJRH_AMCL_SCAN_ADMISSION_STOP_INT_WAIT_SEC:-0.1}" \
      NJRH_AMCL_STOP_INT_WAIT_SEC="${NJRH_AMCL_STOP_INT_WAIT_SEC:-0.1}" \
      bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --stop >/dev/null 2>&1 || true
  stop_existing_standard_nav_stack || true
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

  prepare_resident_navigation_autostart
  if common_require_flatscan_before_resident_autostart; then
    ensure_flatscan_ready_before_navigation_autostart || return 1
  else
    echo "[runtime-overlay] skipping common /flatscan precheck before resident navigation autostart; resident localization owns /flatscan readiness and repair gates" >&2
  fi
  start_common_process "resident_navigation_runtime" "run_navigation_runtime_services.sh" \
    env \
      NJRH_NAVIGATION_RESUME_LOG_FILE="${NJRH_RUNTIME_LOG_DIR}/resident_navigation_runtime.log" \
      NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION="${NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-false}" \
      NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START="${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}" \
      NJRH_MAP_ID="${autostart_map_id}" \
      NJRH_MAP_DISPLAY_NAME="${autostart_display_name}" \
      NJRH_MAP_CONTEXT_BUILDING_ID="${autostart_building_id}" \
      NJRH_MAP_CONTEXT_FLOOR_ID="${autostart_floor_id}" \
      bash "${SCRIPT_DIR}/run_navigation_runtime_services.sh" "${autostart_building_id}" "${autostart_floor_id}"
  resident_navigation_autostart_started=1
}

wait_for_resident_navigation_autostart_if_started() {
  [[ "${resident_navigation_autostart_started}" -eq 1 ]] || return 0
  wait_for_resident_navigation_context_ready
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
  local pids=()
  local proc pid
  for proc in /proc/[0-9]*; do
    [[ -r "${proc}/cmdline" ]] || continue
    pid="${proc##*/}"
    tr '\0' ' ' <"${proc}/cmdline" | grep -Eq "ros2 run fast_lio fastlio_mapping|fast_lio/lib/fast_lio/fastlio_mapping|fast_lio/fastlio_mapping|fastlio_mapping --ros-args|laser_mapping" || continue
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

cleanup() {
  trap - EXIT INT TERM
  local pid
  cleanup_resident_navigation_runtime_layers
  for pid in "${common_pids[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  cleanup_overlay_helpers
  cleanup_canonical_helpers
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

require_can_interface_up

start_canonical_helper "ranger_chassis_common" bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
start_canonical_helper "robot_description_static_tf_common" bash "${SCRIPT_DIR}/run_robot_description.sh"

if reuse_common_services_enabled && canonical_jt128_runtime_complete; then
  echo "[runtime-overlay] reusing existing jt128_driver; canonical driver/remap chain is complete" >&2
else
  if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
    start_common_process "jt128_driver" "__njrh_force_start_jt128_driver_chain__" \
      bash "${SCRIPT_DIR}/run_driver.sh"
  else
    stop_stale_pointcloud_accel_pipeline_processes
    start_common_process "pointcloud_accel_pipeline" "__njrh_force_start_pointcloud_accel_pipeline__" \
      bash "${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh"
  fi
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
if [[ "${NJRH_GS2_AUTOSTART:-true}" == "true" ]]; then
  start_common_process "gs2_driver" "robot_eai_gs2/gs2_driver_node|gs2_driver_node --ros-args|ros2 launch robot_eai_gs2 gs2.launch.py" \
    bash "${SCRIPT_DIR}/run_gs2_driver.sh"
fi
if [[ "${NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART:-true}" == "true" ]]; then
  start_runtime_health_guard_common
else
  echo "[runtime-overlay] runtime_health_guard autostart disabled; startup readiness probes are disabled" >&2
fi
if [[ "${NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE:-true}" == "true" ]]; then
  start_resident_navigation_autostart_if_selected
fi
if [[ "${NAV_LOCAL_STATE_MODE}" == "passthrough" || "${NAV_LOCAL_STATE_MODE}" == "legacy" ]]; then
  # Explicit diagnostic fallback: keep the canonical /local_state/odometry
  # and odom->base_link owner, but back it directly with /wheel/odom.
  kill_canonical_pattern "robot_localization/ekf_node"
  kill_canonical_pattern "ekf_node --ros-args.*__node:=robot_local_state"
fi
start_canonical_helper \
  "robot_local_state_common" \
  env NJRH_LOCAL_STATE_START_READY_MODE="${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}" \
    LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" \
    bash "${SCRIPT_DIR}/run_local_state.sh"
if [[ "${NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART:-true}" == "true" ]]; then
  if [[ "${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}" == "endpoint" ]]; then
    wait_for_runtime_health_local_state_endpoint_ready
  else
    wait_for_runtime_health_local_state_ready
  fi
fi
if [[ "${NJRH_RESIDENT_NAVIGATION_EARLY_AUTOSTART:-true}" == "true" ]]; then
  start_resident_navigation_autostart_if_selected
fi
echo "[runtime-overlay] local_perception_common disabled; local costmap/collision_monitor consume /scan for standard marking+clearing" >&2
start_overlay_helper "floor_manager_common" bash "${SCRIPT_DIR}/run_floor_manager.sh"
start_overlay_helper "robot_safety_common" bash "${SCRIPT_DIR}/run_robot_safety.sh"
start_overlay_helper "ranger_mini3_mode_controller_common" bash "${SCRIPT_DIR}/run_ranger_mini3_mode_controller.sh"
if [[ "${NJRH_DOCKING_MANAGER_AUTOSTART:-true}" == "true" ]]; then
  start_common_process "docking_manager" "robot_docking_manager/docking_manager_node|docking_manager_node --ros-args|run_docking_manager.sh" \
    bash "${SCRIPT_DIR}/run_docking_manager.sh"
else
  echo "[runtime-overlay] docking_manager autostart disabled; set NJRH_DOCKING_MANAGER_AUTOSTART=true for resident /docking services" >&2
fi
start_common_process "robot_api_server" "run_robot_api_server.sh|run_robot_api_server_supervised.sh|robot_api_server/robot_api_server_node|robot_api_server_node --ros-args" \
  bash "${SCRIPT_DIR}/run_robot_api_server_supervised.sh"

if [[ "${RESIDENT_NAVIGATION_AUTOSTART}" != "false" ]]; then
  start_resident_navigation_autostart_if_selected
  wait_for_resident_navigation_autostart_if_started
fi

echo "[runtime-overlay] common services are running; start mapping or resident navigation scripts in reuse mode" >&2
while true; do
  sleep 3600
done

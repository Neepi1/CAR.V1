#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset NJRH_COMMON_ENV_SETUP_DONE NJRH_COMMON_ENV_PARENT_READY
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile
njrh_load_pointcloud_ingress_profile

PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE}"
INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE}"
RESTART="${NJRH_POINTCLOUD_ACCEL_RESTART:-false}"
FLATSCAN_PARAMS="${NJRH_FLATSCAN_PARAMS:-${NJRH_POINTCLOUD_ACCEL_FLATSCAN_PARAMS:-${NJRH_OVERLAY_ROOT}/config/jt128_flatscan.yaml}}"
FLATSCAN_HELPER_REQUIRED="${NJRH_FLATSCAN_HELPER_REQUIRED:-true}"
FLATSCAN_HELPER_RESTART="${NJRH_FLATSCAN_HELPER_RESTART:-true}"
FLATSCAN_HELPER_MAX_RESTARTS="${NJRH_FLATSCAN_HELPER_MAX_RESTARTS:-5}"
FLATSCAN_HELPER_RESTART_BACKOFF_SEC="${NJRH_FLATSCAN_HELPER_RESTART_BACKOFF_SEC:-1.0}"
FLATSCAN_HELPER_MISSING_CONFIRMATIONS="${NJRH_FLATSCAN_HELPER_MISSING_CONFIRMATIONS:-3}"
FLATSCAN_HELPER_RESTART_COOLDOWN_SEC="${NJRH_FLATSCAN_HELPER_RESTART_COOLDOWN_SEC:-60}"
FLATSCAN_HELPER_HEALTHY_RESET_SEC="${NJRH_FLATSCAN_HELPER_HEALTHY_RESET_SEC:-60}"
FLATSCAN_GRAPH_PROBE_TIMEOUT_SEC="${NJRH_FLATSCAN_GRAPH_PROBE_TIMEOUT_SEC:-4}"
FLATSCAN_MESSAGE_CONFIRM_TIMEOUT_SEC="${NJRH_FLATSCAN_MESSAGE_CONFIRM_TIMEOUT_SEC:-10}"
FLATSCAN_WAIT_SEC="${NJRH_FLATSCAN_WAIT_SEC:-30}"
FLATSCAN_MIN_HZ="${NJRH_FLATSCAN_MIN_HZ:-5.0}"
FLATSCAN_SUPERVISE_PERIOD_SEC="${NJRH_FLATSCAN_SUPERVISE_PERIOD_SEC:-10.0}"
FLATSCAN_STATUS_FILE="${NJRH_FLATSCAN_HELPER_STATUS_FILE:-${NJRH_RUNTIME_LOG_DIR}/flatscan_helper_status.env}"

driver_pid=""
local_perception_pid=""
flatscan_pid=""
flatscan_helper_mode="none"
flatscan_helper_restart_count=0
flatscan_helper_graph_miss_count=0
flatscan_helper_health_state="starting"
flatscan_helper_healthy_since_epoch=0
flatscan_helper_restart_cooldown_until_epoch=0

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

stop_pointcloud_profile_processes() {
  local patterns=(
    "[h]esai_ros_driver_node"
    "[p]ointcloud_accel_axis_node"
    "[h]esai_accel_driver_node"
    "[j]t128_accel_driver_node"
    "[p]ointcloud_axis_remap_node"
    "[p]ointcloud_axis_remap"
    "[p]ointcloud_perception_pipeline.launch.py"
    "[c]omponent_container_mt.*pointcloud_perception_pipeline"
    "[p]ointcloud_downsample"
    "[r]obot_local_perception/local_perception_node"
    "[i]nstall/robot_local_perception/.*/local_perception_node"
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
  sleep 1
  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
}

profile_process_running() {
  pgrep -f "$1" >/dev/null 2>&1
}

write_flatscan_helper_status() {
  mkdir -p "$(dirname "${FLATSCAN_STATUS_FILE}")"
  {
    echo "# Runtime status for the /scan -> /flatscan compatibility helper."
    printf 'FLATSCAN_HELPER_MODE=%q\n' "${flatscan_helper_mode}"
    printf 'FLATSCAN_HELPER_PID=%q\n' "${flatscan_pid}"
    printf 'FLATSCAN_HELPER_RESTART_COUNT=%q\n' "${flatscan_helper_restart_count}"
    printf 'FLATSCAN_HELPER_GRAPH_MISS_COUNT=%q\n' "${flatscan_helper_graph_miss_count}"
    printf 'FLATSCAN_HELPER_HEALTH_STATE=%q\n' "${flatscan_helper_health_state}"
    printf 'FLATSCAN_HELPER_HEALTHY_SINCE_EPOCH=%q\n' "${flatscan_helper_healthy_since_epoch}"
    printf 'FLATSCAN_HELPER_RESTART_COOLDOWN_UNTIL_EPOCH=%q\n' "${flatscan_helper_restart_cooldown_until_epoch}"
    printf 'FLATSCAN_HELPER_MISSING_CONFIRMATIONS=%q\n' "${FLATSCAN_HELPER_MISSING_CONFIRMATIONS}"
    printf 'FLATSCAN_HELPER_REQUIRED=%q\n' "${FLATSCAN_HELPER_REQUIRED}"
    printf 'FLATSCAN_HELPER_RESTART=%q\n' "${FLATSCAN_HELPER_RESTART}"
    printf 'FLATSCAN_HELPER_UPDATED_AT=%q\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } >"${FLATSCAN_STATUS_FILE}"
}

topic_publisher_count() {
  local topic="$1"
  local output
  output="$(timeout --kill-after=1 "${FLATSCAN_GRAPH_PROBE_TIMEOUT_SEC}" \
    ros2 topic info -v "${topic}" 2>/dev/null || true)"
  awk -F: '/Publisher count/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}' <<<"${output}"
}

scan_publisher_exists() {
  local publishers
  publishers="$(topic_publisher_count /scan)"
  [[ "${publishers:-0}" -gt 0 ]]
}

flatscan_publisher_exists() {
  local publishers
  publishers="$(topic_publisher_count /flatscan)"
  [[ "${publishers:-0}" -gt 0 ]]
}

flatscan_hz_ok() {
  local output
  local hz
  output="$(timeout --kill-after=1 "${FLATSCAN_MESSAGE_CONFIRM_TIMEOUT_SEC}" \
    ros2 topic hz /flatscan --window 3 2>/dev/null || true)"
  hz="$(awk '/average rate:/ {value=$3} END {if (value != "") print value}' <<<"${output}")"
  if [[ -z "${hz}" ]]; then
    echo "[pointcloud-accel] FAIL /flatscan publisher exists but hz could not be measured" >&2
    return 1
  fi
  if awk -v hz="${hz}" -v min_hz="${FLATSCAN_MIN_HZ}" 'BEGIN {exit !(hz >= min_hz)}'; then
    echo "[pointcloud-accel] /flatscan ready: hz=${hz} min=${FLATSCAN_MIN_HZ}" >&2
    return 0
  fi
  echo "[pointcloud-accel] FAIL /flatscan hz=${hz} below min=${FLATSCAN_MIN_HZ}" >&2
  return 1
}

note_flatscan_healthy() {
  local now
  local healthy_elapsed
  now="$(date +%s)"
  flatscan_helper_graph_miss_count=0
  flatscan_helper_health_state="healthy"
  if [[ "${flatscan_helper_healthy_since_epoch}" -eq 0 ]]; then
    flatscan_helper_healthy_since_epoch="${now}"
  fi
  healthy_elapsed=$((now - flatscan_helper_healthy_since_epoch))
  if [[ "${flatscan_helper_restart_count}" -gt 0 ]] && \
      [[ "${healthy_elapsed}" -ge "${FLATSCAN_HELPER_HEALTHY_RESET_SEC}" ]]; then
    echo "[pointcloud-accel] /flatscan stable health reset restart budget after ${healthy_elapsed}s" >&2
    flatscan_helper_restart_count=0
    flatscan_helper_restart_cooldown_until_epoch=0
  fi
  write_flatscan_helper_status
}

note_flatscan_graph_miss() {
  flatscan_helper_graph_miss_count=$((flatscan_helper_graph_miss_count + 1))
  flatscan_helper_health_state="graph_suspect"
  flatscan_helper_healthy_since_epoch=0
  write_flatscan_helper_status
  echo "[pointcloud-accel] WARN /flatscan graph probe miss ${flatscan_helper_graph_miss_count}/${FLATSCAN_HELPER_MISSING_CONFIRMATIONS}; no restart before confirmation" >&2
}

confirm_flatscan_stream_after_graph_misses() {
  if flatscan_hz_ok; then
    echo "[pointcloud-accel] /flatscan graph misses confirmed but /flatscan messages are flowing; keeping helper pid=${flatscan_pid}" >&2
    note_flatscan_healthy
    return 0
  fi
  return 1
}

wait_for_pid_exit() {
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

flatscan_helper_running() {
  [[ -n "${flatscan_pid}" ]] && kill -0 "${flatscan_pid}" 2>/dev/null
}

stop_flatscan_helper() {
  [[ -n "${flatscan_pid}" ]] || return 0
  if ! kill -0 "${flatscan_pid}" 2>/dev/null; then
    wait "${flatscan_pid}" 2>/dev/null || true
    flatscan_pid=""
    write_flatscan_helper_status
    return 0
  fi
  echo "[pointcloud-accel] stopping flatscan helper pid=${flatscan_pid}" >&2
  kill -INT "${flatscan_pid}" 2>/dev/null || true
  wait_for_pid_exit "${flatscan_pid}" 20 || {
    kill -TERM "${flatscan_pid}" 2>/dev/null || true
    wait_for_pid_exit "${flatscan_pid}" 20 || true
  }
  wait "${flatscan_pid}" 2>/dev/null || true
  flatscan_pid=""
  write_flatscan_helper_status
}

start_flatscan_helper() {
  local helper_prefix
  local helper_bin
  if ! helper_prefix="$(ros2 pkg prefix jt128_nav_tools 2>/dev/null)"; then
    echo "[pointcloud-accel] FAIL jt128_nav_tools laser_scan_to_flatscan is unavailable; /scan may publish but /flatscan cannot be restored" >&2
    return 1
  fi
  helper_bin="${helper_prefix}/lib/jt128_nav_tools/laser_scan_to_flatscan"
  if [[ ! -x "${helper_bin}" ]]; then
    echo "[pointcloud-accel] FAIL jt128_nav_tools laser_scan_to_flatscan binary missing or not executable: ${helper_bin}" >&2
    return 1
  fi
  echo "[pointcloud-accel] starting supervised laser_scan_to_flatscan helper" >&2
  njrh_start_affined_background flatscan_pid laser_scan_to_flatscan \
    "${helper_bin}" \
    --ros-args --params-file "${FLATSCAN_PARAMS}" \
    -r scan:=/scan -r flatscan:=/flatscan
  flatscan_helper_mode="standalone"
  flatscan_helper_graph_miss_count=0
  flatscan_helper_health_state="starting"
  flatscan_helper_healthy_since_epoch=0
  write_flatscan_helper_status
}

wait_for_scan_ready() {
  if runtime_readiness_probe topic /scan "${FLATSCAN_WAIT_SEC}"; then
    return 0
  fi
  echo "[pointcloud-accel] FAIL /scan publisher is not ready; /flatscan helper startup is blocked" >&2
  return 1
}

wait_for_flatscan_ready() {
  if ! truthy "${FLATSCAN_HELPER_REQUIRED}"; then
    echo "[pointcloud-accel] /flatscan helper requirement disabled by NJRH_FLATSCAN_HELPER_REQUIRED=false" >&2
    return 0
  fi
  if ! runtime_readiness_probe topic /flatscan "${FLATSCAN_WAIT_SEC}"; then
    if scan_publisher_exists; then
      echo "[pointcloud-accel] FAIL CASE_FLATSCAN_HELPER_DEAD: /scan has a publisher but /flatscan is missing" >&2
    else
      echo "[pointcloud-accel] FAIL /flatscan missing and /scan publisher is not ready" >&2
    fi
    return 1
  fi
  flatscan_hz_ok
}

restart_flatscan_helper_if_allowed() {
  local reason="$1"
  local now
  now="$(date +%s)"

  if ! truthy "${FLATSCAN_HELPER_RESTART}"; then
    flatscan_helper_health_state="restart_disabled"
    write_flatscan_helper_status
    echo "[pointcloud-accel] FAIL ${reason}; helper restart disabled, keeping supervisor and /scan owner alive" >&2
    return 0
  fi

  if [[ "${flatscan_helper_restart_cooldown_until_epoch}" -gt "${now}" ]]; then
    flatscan_helper_health_state="restart_cooldown"
    write_flatscan_helper_status
    echo "[pointcloud-accel] WARN ${reason}; restart cooldown active until ${flatscan_helper_restart_cooldown_until_epoch}, keeping supervisor alive" >&2
    return 0
  fi

  if [[ "${flatscan_helper_restart_cooldown_until_epoch}" -ne 0 ]]; then
    echo "[pointcloud-accel] /flatscan restart cooldown expired; opening a new bounded restart window" >&2
    flatscan_helper_restart_count=0
    flatscan_helper_restart_cooldown_until_epoch=0
  fi

  if [[ "${flatscan_helper_restart_count}" -ge "${FLATSCAN_HELPER_MAX_RESTARTS}" ]]; then
    flatscan_helper_restart_cooldown_until_epoch=$((now + FLATSCAN_HELPER_RESTART_COOLDOWN_SEC))
    flatscan_helper_health_state="restart_cooldown"
    write_flatscan_helper_status
    echo "[pointcloud-accel] WARN ${reason}; restart budget exhausted; keeping supervisor alive and cooling down for ${FLATSCAN_HELPER_RESTART_COOLDOWN_SEC}s" >&2
    return 0
  fi

  flatscan_helper_restart_count=$((flatscan_helper_restart_count + 1))
  flatscan_helper_graph_miss_count=0
  flatscan_helper_health_state="restarting"
  flatscan_helper_healthy_since_epoch=0
  write_flatscan_helper_status
  echo "[pointcloud-accel] WARN ${reason}; restarting laser_scan_to_flatscan count=${flatscan_helper_restart_count}/${FLATSCAN_HELPER_MAX_RESTARTS}" >&2
  stop_flatscan_helper || true
  sleep "${FLATSCAN_HELPER_RESTART_BACKOFF_SEC}"
  if ! start_flatscan_helper; then
    flatscan_helper_health_state="restart_start_failed"
    write_flatscan_helper_status
    echo "[pointcloud-accel] FAIL helper restart could not start; supervisor remains active for retry" >&2
    return 0
  fi
  if ! wait_for_flatscan_ready; then
    flatscan_helper_health_state="restart_unverified"
    write_flatscan_helper_status
    echo "[pointcloud-accel] WARN helper restart did not pass readiness yet; keeping process and supervisor active for recheck" >&2
    return 0
  fi
  note_flatscan_healthy
}

supervise_flatscan_helper() {
  if ! truthy "${FLATSCAN_HELPER_REQUIRED}"; then
    wait "${driver_pid}"
    return $?
  fi

  while kill -0 "${driver_pid}" 2>/dev/null; do
    case "${flatscan_helper_mode}" in
      standalone)
        if ! flatscan_helper_running; then
          flatscan_helper_graph_miss_count=0
          flatscan_helper_health_state="process_missing"
          flatscan_helper_healthy_since_epoch=0
          write_flatscan_helper_status
          restart_flatscan_helper_if_allowed "laser_scan_to_flatscan exited"
        elif ! flatscan_publisher_exists; then
          note_flatscan_graph_miss
          if [[ "${flatscan_helper_graph_miss_count}" -ge "${FLATSCAN_HELPER_MISSING_CONFIRMATIONS}" ]]; then
            if confirm_flatscan_stream_after_graph_misses; then
              :
            elif scan_publisher_exists; then
              restart_flatscan_helper_if_allowed "CASE_FLATSCAN_HELPER_DEAD: standalone /scan exists but /flatscan publisher is missing while laser_scan_to_flatscan pid=${flatscan_pid} is still alive"
            else
              flatscan_helper_graph_miss_count=0
              flatscan_helper_health_state="upstream_scan_suspect"
              write_flatscan_helper_status
              echo "[pointcloud-accel] WARN standalone scan chain temporarily lacks /flatscan while /scan publisher is not ready; keeping laser_scan_to_flatscan pid=${flatscan_pid} alive and retrying" >&2
            fi
          fi
        else
          note_flatscan_healthy
        fi
        ;;
    esac
    sleep "${FLATSCAN_SUPERVISE_PERIOD_SEC}"
  done

  wait "${driver_pid}"
}

cleanup() {
  stop_flatscan_helper
  [[ -n "${local_perception_pid}" ]] && kill -INT "${local_perception_pid}" 2>/dev/null || true
  [[ -n "${driver_pid}" ]] && kill -INT "${driver_pid}" 2>/dev/null || true
  [[ -n "${local_perception_pid}" ]] && wait "${local_perception_pid}" 2>/dev/null || true
  [[ -n "${driver_pid}" ]] && wait "${driver_pid}" 2>/dev/null || true
}
trap cleanup EXIT

if truthy "${RESTART}"; then
  echo "[pointcloud-accel] restart requested; stopping pointcloud-only profile processes with SIGINT" >&2
  stop_pointcloud_profile_processes
fi

njrh_print_pointcloud_accel_profile

case "${PROFILE}" in
  legacy)
    echo "[pointcloud-accel] FAIL legacy profile removed: do not start robot_local_perception or /perception/* obstacle clouds" >&2
    echo "[pointcloud-accel] Use NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker; /scan feeds Nav2 standard marking+clearing" >&2
    exit 2
    ;;
  ipc_worker)
    echo "[pointcloud-accel] starting ipc_worker fast trunk + same-process worker pipeline ingress=${INGRESS_PROFILE}" >&2
    if [[ "${INGRESS_PROFILE}" == "driver_integrated" ]]; then
      echo "[pointcloud-accel] driver_integrated ingress selected; run_driver.sh will not start standalone hesai_ros_driver_node or pointcloud_accel_axis_node" >&2
      env \
        NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker \
        NJRH_POINTCLOUD_INGRESS_PROFILE=driver_integrated \
        NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}" \
        bash "${SCRIPT_DIR}/run_driver.sh" &
      driver_pid=$!
      wait_for_scan_ready
      start_flatscan_helper
      echo "[pointcloud-accel] final topology: hesai_accel_driver_node decodes JT128 and feeds PointCloudAccelCore in-process; /jt128/vendor/points_raw is debug-only; /scan feeds Nav2 local costmap/collision_monitor and /flatscan helper" >&2
    else
      env \
        NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker \
        NJRH_POINTCLOUD_INGRESS_PROFILE="${INGRESS_PROFILE}" \
        NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}" \
        bash "${SCRIPT_DIR}/run_driver.sh" &
      driver_pid=$!
      wait_for_scan_ready
      start_flatscan_helper
      echo "[pointcloud-accel] final topology: /lidar_points full trunk; pointcloud_accel_axis_node scan worker publishes /scan; compact PointCloud2 local/nav branches disabled by default; /points_nav is not production" >&2
    fi
    ;;
  nitros)
    echo "[pointcloud-accel] validating NITROS environment before startup" >&2
    if ! bash "${SCRIPT_DIR}/check_isaac_ros_nitros_env.sh"; then
      echo "[pointcloud-accel] NITROS profile not started. Use: set_pointcloud_accel_profile.sh --profile ipc_worker --restart" >&2
      exit 3
    fi
    echo "[pointcloud-accel] NITROS skeleton available; launching ROS-compatible worker outputs while NITROS components are integrated" >&2
    env \
      NJRH_POINTCLOUD_ACCEL_PROFILE=nitros \
      NJRH_POINTCLOUD_INGRESS_PROFILE="${INGRESS_PROFILE}" \
      NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}" \
      bash "${SCRIPT_DIR}/run_driver.sh" &
    driver_pid=$!
    wait_for_scan_ready
    start_flatscan_helper
    echo "[pointcloud-accel] final topology: /lidar_points full trunk; NITROS navigation-branch skeleton guarded by environment check; /scan output remains compatible" >&2
    ;;
esac

wait_for_flatscan_ready
note_flatscan_healthy
supervise_flatscan_helper

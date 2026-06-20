#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset NJRH_COMMON_ENV_SETUP_DONE
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
FLATSCAN_WAIT_SEC="${NJRH_FLATSCAN_WAIT_SEC:-30}"
FLATSCAN_MIN_HZ="${NJRH_FLATSCAN_MIN_HZ:-5.0}"
FLATSCAN_SUPERVISE_PERIOD_SEC="${NJRH_FLATSCAN_SUPERVISE_PERIOD_SEC:-5.0}"
FLATSCAN_STATUS_FILE="${NJRH_FLATSCAN_HELPER_STATUS_FILE:-${NJRH_RUNTIME_LOG_DIR}/flatscan_helper_status.env}"

driver_pid=""
local_perception_pid=""
flatscan_pid=""
flatscan_helper_mode="none"
flatscan_helper_restart_count=0

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
    printf 'FLATSCAN_HELPER_REQUIRED=%q\n' "${FLATSCAN_HELPER_REQUIRED}"
    printf 'FLATSCAN_HELPER_RESTART=%q\n' "${FLATSCAN_HELPER_RESTART}"
    printf 'FLATSCAN_HELPER_UPDATED_AT=%q\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } >"${FLATSCAN_STATUS_FILE}"
}

topic_publisher_count() {
  local topic="$1"
  timeout 4 ros2 topic info -v "${topic}" 2>/dev/null \
    | awk -F: '/Publisher count/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}'
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
  output="$(timeout 10 ros2 topic hz /flatscan --window 3 2>/dev/null || true)"
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

restart_flatscan_helper_or_fail() {
  local reason="$1"
  flatscan_helper_restart_count=$((flatscan_helper_restart_count + 1))
  write_flatscan_helper_status
  if ! truthy "${FLATSCAN_HELPER_RESTART}" || [[ "${flatscan_helper_restart_count}" -gt "${FLATSCAN_HELPER_MAX_RESTARTS}" ]]; then
    echo "[pointcloud-accel] FAIL ${reason}; restart=${FLATSCAN_HELPER_RESTART} count=${flatscan_helper_restart_count} max=${FLATSCAN_HELPER_MAX_RESTARTS}" >&2
    return 1
  fi
  echo "[pointcloud-accel] WARN ${reason}; restarting laser_scan_to_flatscan count=${flatscan_helper_restart_count}/${FLATSCAN_HELPER_MAX_RESTARTS}" >&2
  stop_flatscan_helper || true
  sleep "${FLATSCAN_HELPER_RESTART_BACKOFF_SEC}"
  start_flatscan_helper || return 1
  wait_for_flatscan_ready || return 1
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
          restart_flatscan_helper_or_fail "laser_scan_to_flatscan exited" || return 1
        elif ! flatscan_publisher_exists; then
          if scan_publisher_exists; then
            restart_flatscan_helper_or_fail "CASE_FLATSCAN_HELPER_DEAD: standalone /scan exists but /flatscan publisher is missing while laser_scan_to_flatscan pid=${flatscan_pid} is still alive" || return 1
          else
            echo "[pointcloud-accel] FAIL standalone scan chain lost /flatscan and /scan publisher is not ready" >&2
            return 1
          fi
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
supervise_flatscan_helper

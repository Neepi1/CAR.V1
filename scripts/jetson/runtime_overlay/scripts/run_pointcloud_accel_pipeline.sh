#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset NJRH_COMMON_ENV_SETUP_DONE
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile

PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE}"
RESTART="${NJRH_POINTCLOUD_ACCEL_RESTART:-false}"
FLATSCAN_PARAMS="${NJRH_POINTCLOUD_ACCEL_FLATSCAN_PARAMS:-${NJRH_OVERLAY_ROOT}/config/jt128_flatscan.yaml}"
LEGACY_SCAN_PREPROCESSOR_PARAMS="${NJRH_LEGACY_SCAN_PREPROCESSOR_PARAMS:-${NJRH_OVERLAY_ROOT}/config/jt128_nav_cloud_preprocessor.yaml}"
LEGACY_SCAN_PARAMS="${NJRH_LEGACY_SCAN_PARAMS:-${NJRH_OVERLAY_ROOT}/config/jt128_scan_slam2d.yaml}"
LEGACY_SCAN_FLATSCAN_PARAMS="${NJRH_LEGACY_SCAN_FLATSCAN_PARAMS:-${NJRH_OVERLAY_ROOT}/config/jt128_flatscan.yaml}"
LEGACY_SCAN_POINTS_TOPIC="${NJRH_LEGACY_SCAN_POINTS_TOPIC:-/lidar_points_nav}"
LEGACY_SCAN_NAV_POINTS_TOPIC="${NJRH_LEGACY_SCAN_NAV_POINTS_TOPIC:-/points_nav}"
LEGACY_SCAN_TOPIC="${NJRH_LEGACY_SCAN_TOPIC:-/scan}"
LEGACY_SCAN_FLATSCAN_TOPIC="${NJRH_LEGACY_SCAN_FLATSCAN_TOPIC:-/flatscan}"

driver_pid=""
local_perception_pid=""
legacy_scan_pid=""
flatscan_pid=""

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

stop_pointcloud_profile_processes() {
  local patterns=(
    "pointcloud_accel_axis_node"
    "pointcloud_axis_remap_node"
    "pointcloud_axis_remap"
    "pointcloud_perception_pipeline.launch.py"
    "component_container_mt.*pointcloud_perception_pipeline"
    "pointcloud_downsample"
    "robot_local_perception/local_perception_node"
    "install/robot_local_perception/.*/local_perception_node"
    "nav_cloud_preprocessor"
    "pointcloud_to_laserscan_node"
    "pointcloud_to_laserscan"
    "scan_republisher_node"
    "laser_scan_to_flatscan"
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

legacy_scan_chain_running() {
  profile_process_running "nav_cloud_preprocessor" \
    && profile_process_running "pointcloud_to_laserscan" \
    && profile_process_running "scan_republisher" \
    && profile_process_running "laser_scan_to_flatscan"
}

legacy_scan_chain_partial_running() {
  local running=0
  profile_process_running "nav_cloud_preprocessor" && running=$((running + 1))
  profile_process_running "pointcloud_to_laserscan" && running=$((running + 1))
  profile_process_running "scan_republisher" && running=$((running + 1))
  profile_process_running "laser_scan_to_flatscan" && running=$((running + 1))
  [[ "${running}" -gt 0 && "${running}" -lt 4 ]]
}

cleanup() {
  [[ -n "${legacy_scan_pid}" ]] && kill -INT "${legacy_scan_pid}" 2>/dev/null || true
  [[ -n "${flatscan_pid}" ]] && kill -INT "${flatscan_pid}" 2>/dev/null || true
  [[ -n "${local_perception_pid}" ]] && kill -INT "${local_perception_pid}" 2>/dev/null || true
  [[ -n "${driver_pid}" ]] && kill -INT "${driver_pid}" 2>/dev/null || true
  [[ -n "${legacy_scan_pid}" ]] && wait "${legacy_scan_pid}" 2>/dev/null || true
  [[ -n "${flatscan_pid}" ]] && wait "${flatscan_pid}" 2>/dev/null || true
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
    echo "[pointcloud-accel] starting legacy trunk/local-branch pipeline" >&2
    env \
      NJRH_POINTCLOUD_ACCEL_PROFILE=legacy \
      NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}" \
      bash "${SCRIPT_DIR}/run_driver.sh" &
    driver_pid=$!
    sleep 5
    env NJRH_POINTCLOUD_ACCEL_PROFILE=legacy bash "${SCRIPT_DIR}/run_local_perception.sh" &
    local_perception_pid=$!
    if legacy_scan_chain_running; then
      echo "[pointcloud-accel] legacy scan chain already running; reusing" >&2
    elif legacy_scan_chain_partial_running; then
      echo "[pointcloud-accel] WARN legacy scan chain is partially running; use --restart to stop stale pointcloud profile processes before recovery" >&2
    elif ros2 pkg prefix jt128_nav_tools >/dev/null 2>&1 && ros2 pkg prefix pointcloud_to_laserscan >/dev/null 2>&1; then
      ros2 launch "${NJRH_OVERLAY_ROOT}/launch/jt128_localization_sensing.launch.py" \
        preprocessor_params:="${LEGACY_SCAN_PREPROCESSOR_PARAMS}" \
        scan_params:="${LEGACY_SCAN_PARAMS}" \
        flatscan_params:="${LEGACY_SCAN_FLATSCAN_PARAMS}" \
        points_topic:="${LEGACY_SCAN_POINTS_TOPIC}" \
        nav_points_topic:="${LEGACY_SCAN_NAV_POINTS_TOPIC}" \
        scan_topic:="${LEGACY_SCAN_TOPIC}" \
        flatscan_topic:="${LEGACY_SCAN_FLATSCAN_TOPIC}" &
      legacy_scan_pid=$!
    else
      echo "[pointcloud-accel] WARN legacy sensing dependencies are unavailable; /scan and /flatscan will not be restored by this profile restart" >&2
    fi
    echo "[pointcloud-accel] final topology: /lidar_points full trunk; /_internal/lidar_points_local -> robot_local_perception -> /perception/*; /lidar_points_nav -> /points_nav -> /scan -> /flatscan" >&2
    ;;
  ipc_worker)
    echo "[pointcloud-accel] starting ipc_worker fast trunk + same-process worker pipeline" >&2
    env \
      NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker \
      NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}" \
      bash "${SCRIPT_DIR}/run_driver.sh" &
    driver_pid=$!
    if ros2 pkg prefix jt128_nav_tools >/dev/null 2>&1; then
      njrh_start_affined_background flatscan_pid laser_scan_to_flatscan \
        ros2 run jt128_nav_tools laser_scan_to_flatscan \
        --ros-args --params-file "${FLATSCAN_PARAMS}" \
        -r scan:=/scan -r flatscan:=/flatscan
    else
      echo "[pointcloud-accel] WARN jt128_nav_tools laser_scan_to_flatscan is unavailable; /scan will publish but /flatscan will not" >&2
    fi
    echo "[pointcloud-accel] final topology: /lidar_points full trunk; pointcloud_accel_axis_node workers publish /perception/* and /scan; /_internal/lidar_points_local and /lidar_points_nav are compact debug/compat only; /points_nav is not production" >&2
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
      NJRH_FORCE_RESTART_DRIVER="${NJRH_FORCE_RESTART_DRIVER:-false}" \
      bash "${SCRIPT_DIR}/run_driver.sh" &
    driver_pid=$!
    if ros2 pkg prefix jt128_nav_tools >/dev/null 2>&1; then
      njrh_start_affined_background flatscan_pid laser_scan_to_flatscan \
        ros2 run jt128_nav_tools laser_scan_to_flatscan \
        --ros-args --params-file "${FLATSCAN_PARAMS}" \
        -r scan:=/scan -r flatscan:=/flatscan
    fi
    echo "[pointcloud-accel] final topology: /lidar_points full trunk; NITROS navigation-branch skeleton guarded by environment check; ROS /perception/* and /scan outputs remain compatible" >&2
    ;;
esac

wait "${driver_pid}"

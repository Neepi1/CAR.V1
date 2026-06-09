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

driver_pid=""
local_perception_pid=""
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
}

cleanup() {
  [[ -n "${flatscan_pid}" ]] && kill -INT "${flatscan_pid}" 2>/dev/null || true
  [[ -n "${local_perception_pid}" ]] && kill -INT "${local_perception_pid}" 2>/dev/null || true
  [[ -n "${driver_pid}" ]] && kill -INT "${driver_pid}" 2>/dev/null || true
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
    echo "[pointcloud-accel] final topology: /lidar_points full trunk; /_internal/lidar_points_local -> robot_local_perception -> /perception/*; localization scan chain remains legacy-owned by occupancy runtime" >&2
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

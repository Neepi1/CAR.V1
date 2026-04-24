#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

CONFIG_FILE="${NJRH_HESAI_CONFIG_FILE:-${UPSTREAM_WS}/src/hesai_lidar_ros2/config/config.yaml}"
export DRIVER_PROFILE="${DRIVER_PROFILE:-mapping}"
export LIDAR_FRAME="${LIDAR_FRAME:-lidar_link}"
export IMU_FRAME="${IMU_FRAME:-imu_link}"
export POINTS_TOPIC="${NJRH_JT128_POINTS_TOPIC:-/lidar_points}"
export IMU_TOPIC="${NJRH_JT128_IMU_TOPIC:-/lidar_imu}"
export VENDOR_POINTS_TOPIC="${NJRH_JT128_VENDOR_POINTS_TOPIC:-/jt128/vendor/points_raw}"
export VENDOR_IMU_TOPIC="${NJRH_JT128_VENDOR_IMU_TOPIC:-/jt128/vendor/imu_raw}"
export POINTCLOUD_REMAP_CONFIG="${NJRH_JT128_CANONICAL_POINTCLOUD_REMAP_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_canonical_pointcloud_remap.yaml}"
export IMU_REMAP_CONFIG="${NJRH_JT128_CANONICAL_IMU_REMAP_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_canonical_imu_remap.yaml}"
export POINTCLOUD_REMAP_CPP_BIN="${NJRH_POINTCLOUD_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/pointcloud_axis_remap_node}"
export IMU_REMAP_CPP_BIN="${NJRH_IMU_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/imu_axis_remap_node}"
UPSTREAM_DRIVER_PROFILE="${DRIVER_PROFILE}"

# JT128 on the current Jetson stack drops live point/IMU output when the
# upstream helper switches to its "navigation" timestamp mode. Keep the
# repository-owned canonical ingress on the verified live profile by default.
if [[ "${DRIVER_PROFILE}" == "navigation" ]]; then
  UPSTREAM_DRIVER_PROFILE="${NJRH_HESAI_NAV_UPSTREAM_PROFILE:-mapping}"
fi

[[ -f "${CONFIG_FILE}" ]] || {
  echo "[runtime-overlay] driver config missing: ${CONFIG_FILE}" >&2
  exit 1
}

RUNTIME_CONFIG_FILE="$(mktemp /tmp/njrh_driver_config_XXXX.yaml)"
sed -E \
  -e "s|^([[:space:]]*ros_frame_id:[[:space:]]*).*|\\1hesai_lidar|" \
  -e "s|^([[:space:]]*ros_send_point_cloud_topic:[[:space:]]*).*|\\1${VENDOR_POINTS_TOPIC}|" \
  -e "s|^([[:space:]]*ros_send_imu_topic:[[:space:]]*).*|\\1${VENDOR_IMU_TOPIC}|" \
  "${CONFIG_FILE}" > "${RUNTIME_CONFIG_FILE}"
export CONFIG_FILE="${RUNTIME_CONFIG_FILE}"

driver_pid=""
pointcloud_remap_pid=""
imu_remap_pid=""

cleanup() {
  if [[ -n "${pointcloud_remap_pid}" ]]; then
    kill -INT "${pointcloud_remap_pid}" 2>/dev/null || true
    wait "${pointcloud_remap_pid}" 2>/dev/null || true
  fi
  if [[ -n "${imu_remap_pid}" ]]; then
    kill -INT "${imu_remap_pid}" 2>/dev/null || true
    wait "${imu_remap_pid}" 2>/dev/null || true
  fi
  if [[ -n "${driver_pid}" ]]; then
    kill -INT "${driver_pid}" 2>/dev/null || true
    wait "${driver_pid}" 2>/dev/null || true
  fi
  rm -f "${RUNTIME_CONFIG_FILE}"
}

trap cleanup EXIT

for pattern in "hesai_ros_driver_node" "ros2 run hesai_ros_driver" "imu_axis_remap" "pointcloud_axis_remap"; do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in "hesai_ros_driver_node" "ros2 run hesai_ros_driver" "imu_axis_remap" "pointcloud_axis_remap"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

[[ -f "${POINTCLOUD_REMAP_CONFIG}" ]] || {
  echo "[runtime-overlay] canonical pointcloud remap config missing: ${POINTCLOUD_REMAP_CONFIG}" >&2
  exit 1
}
[[ -f "${IMU_REMAP_CONFIG}" ]] || {
  echo "[runtime-overlay] canonical imu remap config missing: ${IMU_REMAP_CONFIG}" >&2
  exit 1
}
[[ -x "${POINTCLOUD_REMAP_CPP_BIN}" ]] || {
  echo "[runtime-overlay] compiled pointcloud remap missing or not executable: ${POINTCLOUD_REMAP_CPP_BIN}" >&2
  echo "[runtime-overlay] build robot_hesai_jt128; Python remap fallback has been removed." >&2
  exit 1
}
echo "[runtime-overlay] using compiled pointcloud remap: ${POINTCLOUD_REMAP_CPP_BIN}" >&2
"${POINTCLOUD_REMAP_CPP_BIN}" --ros-args --params-file "${POINTCLOUD_REMAP_CONFIG}" &
pointcloud_remap_pid=$!

[[ -x "${IMU_REMAP_CPP_BIN}" ]] || {
  echo "[runtime-overlay] compiled imu remap missing or not executable: ${IMU_REMAP_CPP_BIN}" >&2
  echo "[runtime-overlay] build robot_hesai_jt128; Python remap fallback has been removed." >&2
  exit 1
}
echo "[runtime-overlay] using compiled imu remap: ${IMU_REMAP_CPP_BIN}" >&2
"${IMU_REMAP_CPP_BIN}" --ros-args --params-file "${IMU_REMAP_CONFIG}" &
imu_remap_pid=$!

DRIVER_PROFILE="${UPSTREAM_DRIVER_PROFILE}" bash "$(require_upstream_script run_driver.sh)" &
driver_pid=$!
wait "${driver_pid}"

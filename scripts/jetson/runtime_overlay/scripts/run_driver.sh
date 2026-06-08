#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# run_driver may be spawned by a long-lived parent runtime script. Refresh the
# ROS overlay here so component containers see newly installed packages.
unset NJRH_COMMON_ENV_SETUP_DONE
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

CONFIG_FILE="${NJRH_HESAI_CONFIG_FILE:-${UPSTREAM_WS}/src/hesai_lidar_ros2/config/config.yaml}"
export DRIVER_PROFILE="${DRIVER_PROFILE:-mapping}"
export NJRH_HESAI_UPSTREAM_DRIVER_PROFILE="${NJRH_HESAI_UPSTREAM_DRIVER_PROFILE:-navigation}"
export LIDAR_FRAME="${LIDAR_FRAME:-lidar_link}"
export IMU_FRAME="${IMU_FRAME:-imu_link}"
export POINTS_TOPIC="${NJRH_JT128_POINTS_TOPIC:-/lidar_points}"
export IMU_TOPIC="${NJRH_JT128_IMU_TOPIC:-/lidar_imu}"
export VENDOR_POINTS_TOPIC="${NJRH_JT128_VENDOR_POINTS_TOPIC:-/jt128/vendor/points_raw}"
export VENDOR_IMU_TOPIC="${NJRH_JT128_VENDOR_IMU_TOPIC:-/jt128/vendor/imu_raw}"
export POINTCLOUD_REMAP_CONFIG="${NJRH_JT128_CANONICAL_POINTCLOUD_REMAP_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_canonical_pointcloud_remap.yaml}"
export POINTCLOUD_DOWNSAMPLE_CONFIG="${NJRH_JT128_POINTCLOUD_DOWNSAMPLE_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_pointcloud_downsample.yaml}"
export IMU_REMAP_CONFIG="${NJRH_JT128_CANONICAL_IMU_REMAP_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_canonical_imu_remap.yaml}"
export POINTCLOUD_PIPELINE_LAUNCH="${NJRH_POINTCLOUD_PIPELINE_LAUNCH:-${NJRH_OVERLAY_ROOT}/launch/pointcloud_perception_pipeline.launch.py}"
export LOCAL_PERCEPTION_CONFIG="${LOCAL_PERCEPTION_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_perception.yaml}"
export POINTCLOUD_REMAP_CPP_BIN="${NJRH_POINTCLOUD_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/pointcloud_axis_remap_node}"
export POINTCLOUD_DOWNSAMPLE_CPP_BIN="${NJRH_POINTCLOUD_DOWNSAMPLE_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/pointcloud_downsample_node}"
export IMU_REMAP_CPP_BIN="${NJRH_IMU_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/imu_axis_remap_node}"
export NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER="${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER:-false}"
export NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE="${NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE:-false}"
UPSTREAM_DRIVER_PROFILE="${NJRH_HESAI_UPSTREAM_DRIVER_PROFILE}"

[[ -f "${CONFIG_FILE}" ]] || {
  echo "[runtime-overlay] driver config missing: ${CONFIG_FILE}" >&2
  exit 1
}

RUNTIME_CONFIG_FILE="$(mktemp /tmp/njrh_driver_config_XXXX.yaml)"
sed -E \
  -e "s|^([[:space:]]*ros_frame_id:[[:space:]]*).*|\\1hesai_lidar|" \
  -e "s|^([[:space:]]*ros_send_point_cloud_topic:[[:space:]]*).*|\\1${VENDOR_POINTS_TOPIC}|" \
  -e "s|^([[:space:]]*ros_send_imu_topic:[[:space:]]*).*|\\1${VENDOR_IMU_TOPIC}|" \
  -e "s|^([[:space:]]*)use_timestamp_type:.*|\\1use_timestamp_type: 1                 # 0 use lidar point cloud timestamp; 1 use host receive timestamp|" \
  "${CONFIG_FILE}" > "${RUNTIME_CONFIG_FILE}"
export CONFIG_FILE="${RUNTIME_CONFIG_FILE}"

driver_pid=""
pointcloud_pipeline_pid=""
pointcloud_remap_pid=""
pointcloud_downsample_pid=""
imu_remap_pid=""
stopped_disabled_pointcloud_downsample="false"

cleanup() {
  if [[ -n "${pointcloud_pipeline_pid}" ]]; then
    kill -INT "${pointcloud_pipeline_pid}" 2>/dev/null || true
    wait "${pointcloud_pipeline_pid}" 2>/dev/null || true
  fi
  if [[ -n "${pointcloud_remap_pid}" ]]; then
    kill -INT "${pointcloud_remap_pid}" 2>/dev/null || true
    wait "${pointcloud_remap_pid}" 2>/dev/null || true
  fi
  if [[ -n "${pointcloud_downsample_pid}" ]]; then
    kill -INT "${pointcloud_downsample_pid}" 2>/dev/null || true
    wait "${pointcloud_downsample_pid}" 2>/dev/null || true
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

jt128_driver_process_running() {
  pgrep -f "hesai_ros_driver_node" >/dev/null 2>&1
}

jt128_pointcloud_remap_running() {
  if [[ "${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER}" == "true" ]]; then
    pgrep -f "pointcloud_perception_pipeline.launch.py|component_container_mt.*pointcloud_perception_pipeline|pointcloud_perception_pipeline" >/dev/null 2>&1
  else
    pgrep -f "pointcloud_axis_remap" >/dev/null 2>&1
  fi
}

jt128_pointcloud_downsample_running() {
  pgrep -f "pointcloud_downsample" >/dev/null 2>&1
}

jt128_imu_remap_running() {
  pgrep -f "imu_axis_remap" >/dev/null 2>&1
}

canonical_jt128_ingress_running() {
  jt128_driver_process_running || return 1
  jt128_pointcloud_remap_running || return 1
  jt128_imu_remap_running || return 1
  if [[ "${NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE}" == "true" ]]; then
    jt128_pointcloud_downsample_running || return 1
  fi
  return 0
}

any_jt128_ingress_process_running() {
  jt128_driver_process_running ||
    jt128_pointcloud_remap_running ||
    jt128_pointcloud_downsample_running ||
    jt128_imu_remap_running
}

stop_jt128_ingress_processes() {
  for pattern in "hesai_ros_driver_node" "ros2 run hesai_ros_driver" "imu_axis_remap" "pointcloud_perception_pipeline.launch.py" "component_container_mt.*pointcloud_perception_pipeline" "pointcloud_axis_remap" "pointcloud_fastlio_remap" "pointcloud_downsample"; do
    pkill -INT -f "$pattern" 2>/dev/null || true
  done
  sleep 1
  for pattern in "hesai_ros_driver_node" "ros2 run hesai_ros_driver" "imu_axis_remap" "pointcloud_perception_pipeline.launch.py" "component_container_mt.*pointcloud_perception_pipeline" "pointcloud_axis_remap" "pointcloud_fastlio_remap" "pointcloud_downsample"; do
    pkill -9 -f "$pattern" 2>/dev/null || true
  done
}

if [[ "${NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE}" != "true" ]] && jt128_pointcloud_downsample_running; then
  echo "[runtime-overlay] stopping diagnostic pointcloud_downsample; production /lidar_points is produced by pointcloud_axis_remap" >&2
  pkill -INT -f "pointcloud_downsample" 2>/dev/null || true
  sleep 1
  pkill -9 -f "pointcloud_downsample" 2>/dev/null || true
  stopped_disabled_pointcloud_downsample="true"
fi

if any_jt128_ingress_process_running; then
  if [[ "${NJRH_FORCE_RESTART_DRIVER:-false}" != "true" ]]; then
    if [[ "${stopped_disabled_pointcloud_downsample}" != "true" ]] && canonical_jt128_ingress_running; then
      echo "[runtime-overlay] canonical JT128 driver/remap chain already running; reusing existing ingress" >&2
      while canonical_jt128_ingress_running; do
        sleep 2
      done
      exit 0
    fi
    echo "[runtime-overlay] incomplete JT128 ingress detected; restarting driver plus canonical pointcloud/IMU remaps" >&2
  fi
  stop_jt128_ingress_processes
fi

[[ -f "${POINTCLOUD_REMAP_CONFIG}" ]] || {
  echo "[runtime-overlay] canonical pointcloud remap config missing: ${POINTCLOUD_REMAP_CONFIG}" >&2
  exit 1
}
if [[ "${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER}" == "true" ]]; then
  [[ -f "${POINTCLOUD_PIPELINE_LAUNCH}" ]] || {
    echo "[runtime-overlay] pointcloud perception pipeline launch missing: ${POINTCLOUD_PIPELINE_LAUNCH}" >&2
    exit 1
  }
  [[ -f "${LOCAL_PERCEPTION_CONFIG}" ]] || {
    echo "[runtime-overlay] local perception config missing: ${LOCAL_PERCEPTION_CONFIG}" >&2
    exit 1
  }
fi
[[ -f "${IMU_REMAP_CONFIG}" ]] || {
  echo "[runtime-overlay] canonical imu remap config missing: ${IMU_REMAP_CONFIG}" >&2
  exit 1
}
if [[ "${NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE}" == "true" ]]; then
  [[ -f "${POINTCLOUD_DOWNSAMPLE_CONFIG}" ]] || {
    echo "[runtime-overlay] pointcloud downsample config missing: ${POINTCLOUD_DOWNSAMPLE_CONFIG}" >&2
    exit 1
  }
fi
if [[ "${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER}" == "true" ]]; then
  echo "[runtime-overlay] starting component pointcloud perception pipeline: ${POINTCLOUD_PIPELINE_LAUNCH}" >&2
  njrh_run_affined pointcloud_perception_pipeline \
    ros2 launch "${POINTCLOUD_PIPELINE_LAUNCH}" \
      pointcloud_params:="${POINTCLOUD_REMAP_CONFIG}" \
      local_perception_params:="${LOCAL_PERCEPTION_CONFIG}" &
  pointcloud_pipeline_pid=$!
else
  [[ -x "${POINTCLOUD_REMAP_CPP_BIN}" ]] || {
    echo "[runtime-overlay] compiled pointcloud remap missing or not executable: ${POINTCLOUD_REMAP_CPP_BIN}" >&2
    echo "[runtime-overlay] build robot_hesai_jt128; Python remap fallback has been removed." >&2
    exit 1
  }
  echo "[runtime-overlay] using compiled pointcloud remap: ${POINTCLOUD_REMAP_CPP_BIN}" >&2
  njrh_run_affined pointcloud_axis_remap \
    "${POINTCLOUD_REMAP_CPP_BIN}" --ros-args --params-file "${POINTCLOUD_REMAP_CONFIG}" &
  pointcloud_remap_pid=$!
fi

if [[ "${NJRH_JT128_ENABLE_POINTCLOUD_DOWNSAMPLE}" == "true" ]]; then
  [[ -x "${POINTCLOUD_DOWNSAMPLE_CPP_BIN}" ]] || {
    echo "[runtime-overlay] compiled pointcloud downsample missing or not executable: ${POINTCLOUD_DOWNSAMPLE_CPP_BIN}" >&2
    echo "[runtime-overlay] build robot_hesai_jt128; Python fallback has been removed." >&2
    exit 1
  }
  echo "[runtime-overlay] using compiled pointcloud downsample: ${POINTCLOUD_DOWNSAMPLE_CPP_BIN}" >&2
  njrh_run_affined pointcloud_downsample \
    "${POINTCLOUD_DOWNSAMPLE_CPP_BIN}" --ros-args --params-file "${POINTCLOUD_DOWNSAMPLE_CONFIG}" &
  pointcloud_downsample_pid=$!
else
  echo "[runtime-overlay] diagnostic pointcloud_downsample disabled; pointcloud ingress publishes only canonical /lidar_points" >&2
fi

[[ -x "${IMU_REMAP_CPP_BIN}" ]] || {
  echo "[runtime-overlay] compiled imu remap missing or not executable: ${IMU_REMAP_CPP_BIN}" >&2
  echo "[runtime-overlay] build robot_hesai_jt128; Python remap fallback has been removed." >&2
  exit 1
}
echo "[runtime-overlay] using compiled imu remap: ${IMU_REMAP_CPP_BIN}" >&2
njrh_run_affined imu_axis_remap \
  "${IMU_REMAP_CPP_BIN}" --ros-args --params-file "${IMU_REMAP_CONFIG}" &
imu_remap_pid=$!

export DRIVER_PROFILE="${UPSTREAM_DRIVER_PROFILE}"
njrh_run_affined hesai_ros_driver bash "$(require_upstream_script run_driver.sh)" &
driver_pid=$!
wait "${driver_pid}"

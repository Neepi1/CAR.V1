#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"

export CONFIG_FILE="${NJRH_FASTLIO_CONFIG_FILE:-${NJRH_OVERLAY_ROOT}/config/fastlio.yaml}"
export FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"
export PUBLISH_ODOM_TF="false"
export ODOM_TOPIC="${ODOM_TOPIC:-/wheel/odom}"

[[ -f "${CONFIG_FILE}" ]] || {
  echo "[runtime-overlay] missing FAST-LIO runtime file: ${CONFIG_FILE}" >&2
  exit 1
}

if [[ "${ENABLE_FASTLIO_IMU_REMAP:-false}" == "true" || "${ENABLE_FASTLIO_POINTCLOUD_REMAP:-false}" == "true" ]]; then
  echo "[runtime-overlay] Fast-LIO must consume the canonical TF-tree inputs /lidar_points and /lidar_imu under lidar_link. Legacy Fast-LIO-only remap paths are no longer supported." >&2
  exit 1
fi

require_can_interface_up

for pattern in "laser_mapping" "ros2 launch fast_lio" "hesai_lidar_state_publisher"; do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in "laser_mapping" "ros2 launch fast_lio" "hesai_lidar_state_publisher"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

stop_existing_canonical_tf_publishers

fastlio_helper_pids=()
fastlio_pid=""
fastlio_exit_code=0

start_fastlio_helper() {
  local helper_name="$1"
  shift
  local helper_log="${NJRH_RUNTIME_LOG_DIR}/${helper_name}.log"
  echo "[runtime-overlay] starting ${helper_name}" >&2
  "$@" >>"${helper_log}" 2>&1 &
  local helper_pid=$!
  fastlio_helper_pids+=("${helper_pid}")
  sleep 1
  if ! kill -0 "${helper_pid}" 2>/dev/null; then
    echo "[runtime-overlay] helper failed to stay alive: ${helper_name}. Check ${helper_log}" >&2
    return 1
  fi
}

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${fastlio_pid}" ]]; then
    kill -INT "${fastlio_pid}" 2>/dev/null || true
    wait "${fastlio_pid}" 2>/dev/null || true
  fi
  local helper_pid
  for helper_pid in "${fastlio_helper_pids[@]:-}"; do
    kill -INT "${helper_pid}" 2>/dev/null || true
  done
  sleep 1
  for helper_pid in "${fastlio_helper_pids[@]:-}"; do
    kill -9 "${helper_pid}" 2>/dev/null || true
  done
  cleanup_canonical_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

start_canonical_helper "ranger_chassis" bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
start_canonical_helper "robot_description_static_tf" bash "${SCRIPT_DIR}/run_robot_description.sh"
start_canonical_helper "robot_local_state" bash "${SCRIPT_DIR}/run_local_state.sh"

ros2 run fast_lio fastlio_mapping \
  --ros-args \
  --params-file "${CONFIG_FILE}" \
  -p use_sim_time:=false \
  -r /tf:=/tf_fastlio_internal \
  -r /tf_static:=/tf_static_fastlio_internal &
fastlio_pid=$!
wait "${fastlio_pid}" || fastlio_exit_code=$?
exit "${fastlio_exit_code}"

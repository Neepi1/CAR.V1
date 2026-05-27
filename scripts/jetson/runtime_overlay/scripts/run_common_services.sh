#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"

common_pids=()
NAV_LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-passthrough}"

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
  sleep 1
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[runtime-overlay] common service failed to stay alive: ${name}. Check ${log_file}" >&2
    return 1
  fi
  echo "[runtime-overlay] common service ready: ${name} (pid=${pid})" >&2
}

canonical_jt128_ingress_running() {
  pgrep -f "hesai_ros_driver_node" >/dev/null 2>&1 &&
    pgrep -f "pointcloud_axis_remap" >/dev/null 2>&1 &&
    pgrep -f "imu_axis_remap" >/dev/null 2>&1
}

cleanup() {
  trap - EXIT INT TERM
  local pid
  for pid in "${common_pids[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  cleanup_overlay_helpers
  cleanup_canonical_helpers
  sleep 1
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

if reuse_common_services_enabled && canonical_jt128_ingress_running; then
  echo "[runtime-overlay] reusing existing jt128_driver; canonical driver/remap chain is complete" >&2
else
  start_common_process "jt128_driver" "__njrh_force_start_jt128_driver_chain__" \
    bash "${SCRIPT_DIR}/run_driver.sh"
fi
start_canonical_helper "ranger_chassis_common" bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
start_canonical_helper "robot_description_static_tf_common" bash "${SCRIPT_DIR}/run_robot_description.sh"
if [[ "${NJRH_GS2_AUTOSTART:-true}" == "true" ]]; then
  start_common_process "gs2_driver" "robot_eai_gs2/gs2_driver_node|gs2_driver_node --ros-args|ros2 launch robot_eai_gs2 gs2.launch.py" \
    bash "${SCRIPT_DIR}/run_gs2_driver.sh"
fi
if [[ "${NAV_LOCAL_STATE_MODE}" == "passthrough" || "${NAV_LOCAL_STATE_MODE}" == "legacy" ]]; then
  # Temporary navigation field mode: keep the canonical /local_state/odometry
  # and odom->base_link owner, but back it directly with /wheel/odom instead
  # of the wheel+JT128-IMU EKF until EKF yaw drift is resolved.
  kill_canonical_pattern "robot_localization/ekf_node"
  kill_canonical_pattern "ekf_node --ros-args.*__node:=robot_local_state"
fi
start_canonical_helper \
  "robot_local_state_common" \
  env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"
start_overlay_helper "local_perception_common" bash "${SCRIPT_DIR}/run_local_perception.sh"
start_overlay_helper "floor_manager_common" bash "${SCRIPT_DIR}/run_floor_manager.sh"
start_overlay_helper "robot_safety_common" bash "${SCRIPT_DIR}/run_robot_safety.sh"
start_overlay_helper "ranger_mini3_mode_controller_common" bash "${SCRIPT_DIR}/run_ranger_mini3_mode_controller.sh"
start_common_process "robot_api_server" "run_robot_api_server_supervised.sh|robot_api_server/robot_api_server_node|robot_api_server_node --ros-args" \
  bash "${SCRIPT_DIR}/run_robot_api_server_supervised.sh"

echo "[runtime-overlay] common services are running; start mapping or navigation scripts in reuse mode" >&2
while true; do
  sleep 3600
done

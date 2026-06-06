#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

MODE="${LOCAL_STATE_MODE:-ekf}"
NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_local_state/lib/robot_local_state/local_state_node"

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
  local int_attempts="${3:-${LOCAL_STATE_STOP_INT_ATTEMPTS:-20}}"
  local term_attempts="${4:-${LOCAL_STATE_STOP_TERM_ATTEMPTS:-20}}"
  [[ -n "${pid}" ]] || return 0
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi
  echo "[runtime-overlay] stopping ${label} pid=${pid}" >&2
  kill -INT "${pid}" 2>/dev/null || true
  wait_for_child_exit "${pid}" "${int_attempts}" || {
    kill -TERM "${pid}" 2>/dev/null || true
    wait_for_child_exit "${pid}" "${term_attempts}" || {
      kill -KILL "${pid}" 2>/dev/null || true
    }
  }
  wait "${pid}" 2>/dev/null || true
}

if [[ "${MODE}" == "fastlio" ]]; then
  PARAMS_FILE="${LOCAL_STATE_FASTLIO_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_state_fastlio.yaml}"
  [[ -f "${PARAMS_FILE}" ]] || {
    echo "[runtime-overlay] local state FAST-LIO params file missing: ${PARAMS_FILE}" >&2
    exit 1
  }
  [[ -x "${NODE_BIN}" ]] || {
    echo "[runtime-overlay] compiled local state node missing or not executable: ${NODE_BIN}" >&2
    exit 1
  }

  FASTLIO_BRIDGE_BIN="${NJRH_PROJECT_ROOT}/install/robot_fastlio_mapping/lib/robot_fastlio_mapping/fastlio_odom_bridge_node"
  [[ -x "${FASTLIO_BRIDGE_BIN}" ]] || {
    echo "[runtime-overlay] compiled FAST-LIO odom bridge missing or not executable: ${FASTLIO_BRIDGE_BIN}" >&2
    exit 1
  }

  bridge_pid=""
  local_state_pid=""
  cleanup_fastlio_mode() {
    trap - EXIT INT TERM
    terminate_child "${local_state_pid}" "robot_local_state"
    terminate_child "${bridge_pid}" "FAST-LIO odom bridge"
  }
  on_signal() {
    cleanup_fastlio_mode
    exit 130
  }
  trap cleanup_fastlio_mode EXIT
  trap on_signal INT TERM

  njrh_start_affined_background bridge_pid fastlio_odom_bridge "${FASTLIO_BRIDGE_BIN}" --ros-args \
    -p input_topic:="${LOCAL_STATE_FASTLIO_INPUT_TOPIC:-/Odometry}" \
    -p output_topic:="${LOCAL_STATE_FASTLIO_BASE_ODOM_TOPIC:-/fastlio/base_odometry}" \
    -p output_odom_frame:=odom \
    -p output_base_frame:=base_link \
    -p sensor_frame:=lidar_link \
    -p anchor_on_first_sample:=true \
    -p flatten_to_2d:=true \
    -p restamp_output_to_now:="${LOCAL_STATE_FASTLIO_RESTAMP_OUTPUT_TO_NOW:-true}" \
    -p output_stamp_offset_sec:="${LOCAL_STATE_FASTLIO_OUTPUT_STAMP_OFFSET_SEC:-0.0}" \
    -p input_reliable:="${LOCAL_STATE_FASTLIO_INPUT_RELIABLE:-false}" \
    -p input_qos_depth:="${LOCAL_STATE_FASTLIO_INPUT_QOS_DEPTH:-1}" \
    -p output_reliable:="${LOCAL_STATE_FASTLIO_OUTPUT_RELIABLE:-true}" \
    -p output_qos_depth:="${LOCAL_STATE_FASTLIO_OUTPUT_QOS_DEPTH:-20}" \
    -p publish_tf:=false
  sleep 1
  if ! kill -0 "${bridge_pid}" 2>/dev/null; then
    echo "[runtime-overlay] FAST-LIO odom bridge failed to stay alive" >&2
    exit 1
  fi

  njrh_start_affined_background local_state_pid robot_local_state "${NODE_BIN}" --ros-args \
    --params-file "${PARAMS_FILE}" \
    -r __node:=robot_local_state
  sleep "${LOCAL_STATE_LAUNCH_SETTLE_SEC:-1}"
  if ! kill -0 "${local_state_pid}" 2>/dev/null; then
    echo "[runtime-overlay] robot_local_state failed to stay alive" >&2
    exit 1
  fi

  echo "[runtime-overlay] FAST-LIO local-state launched; startup readiness probes are disabled" >&2
  fastlio_mode_exit_code=0
  wait -n "${bridge_pid}" "${local_state_pid}" || fastlio_mode_exit_code=$?
  exit "${fastlio_mode_exit_code}"
fi

if [[ "${MODE}" == "passthrough" || "${MODE}" == "legacy" ]]; then
  PARAMS_FILE="${LOCAL_STATE_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_state.yaml}"
  [[ -f "${PARAMS_FILE}" ]] || {
    echo "[runtime-overlay] local state params file missing: ${PARAMS_FILE}" >&2
    exit 1
  }

  [[ -x "${NODE_BIN}" ]] || {
    echo "[runtime-overlay] compiled local state node missing or not executable: ${NODE_BIN}" >&2
    echo "[runtime-overlay] build robot_local_state; Python fallback has been removed." >&2
    exit 1
  }

  njrh_exec_affined robot_local_state "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"
fi

EKF_PARAMS_FILE="${LOCAL_STATE_EKF_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_state_ekf.yaml}"
[[ -f "${EKF_PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] local state EKF params file missing: ${EKF_PARAMS_FILE}" >&2
  exit 1
}
WHEEL_ODOM_EKF_PARAMS_FILE="${LOCAL_STATE_WHEEL_ODOM_EKF_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_state_wheel_odom_ekf.yaml}"
[[ -f "${WHEEL_ODOM_EKF_PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] local state wheel odom EKF params file missing: ${WHEEL_ODOM_EKF_PARAMS_FILE}" >&2
  exit 1
}
IMU_BIAS_FILTER_PARAMS_FILE="${LOCAL_STATE_IMU_BIAS_FILTER_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_state_imu_bias_filter.yaml}"
[[ -f "${IMU_BIAS_FILTER_PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] local state IMU bias filter params file missing: ${IMU_BIAS_FILTER_PARAMS_FILE}" >&2
  exit 1
}

if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  echo "[runtime-overlay] ROS package missing: robot_localization" >&2
  echo "[runtime-overlay] install ros-humble-robot-localization in the NJRH-car image/container." >&2
  exit 1
fi

[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] compiled local state node missing or not executable: ${NODE_BIN}" >&2
  exit 1
}

cleanup_stale_ekf_mode_processes() {
  pkill -INT -f "__node:=wheel_odom_ekf_input" 2>/dev/null || true
  pkill -INT -f "robot_local_state/imu_gyro_bias_filter_node" 2>/dev/null || true
  pkill -INT -f "imu_gyro_bias_filter_node --ros-args" 2>/dev/null || true
  pkill -INT -f "robot_localization/ekf_node" 2>/dev/null || true
  pkill -INT -f "ekf_node --ros-args.*__node:=robot_local_state" 2>/dev/null || true
  sleep 1
  pkill -9 -f "__node:=wheel_odom_ekf_input" 2>/dev/null || true
  pkill -9 -f "robot_local_state/imu_gyro_bias_filter_node" 2>/dev/null || true
  pkill -9 -f "imu_gyro_bias_filter_node --ros-args" 2>/dev/null || true
  pkill -9 -f "robot_localization/ekf_node" 2>/dev/null || true
  pkill -9 -f "ekf_node --ros-args.*__node:=robot_local_state" 2>/dev/null || true
}

if [[ "${LOCAL_STATE_CLEAN_STALE_EKF_MODE:-true}" == "true" ]]; then
  cleanup_stale_ekf_mode_processes
fi

wheel_odom_pid=""
imu_bias_pid=""
ekf_pid=""

cleanup_ekf_mode() {
  trap - EXIT INT TERM
  terminate_child "${ekf_pid}" "robot_local_state EKF"
  terminate_child "${wheel_odom_pid}" "wheel odom EKF input preprocessor"
  terminate_child "${imu_bias_pid}" "IMU gyro bias filter"
}

on_signal() {
  cleanup_ekf_mode
  exit 130
}

trap cleanup_ekf_mode EXIT
trap on_signal INT TERM

njrh_start_affined_background wheel_odom_pid robot_local_state_odom_preprocessor "${NODE_BIN}" --ros-args \
  --params-file "${WHEEL_ODOM_EKF_PARAMS_FILE}" \
  -r __node:=wheel_odom_ekf_input
sleep 1
if ! kill -0 "${wheel_odom_pid}" 2>/dev/null; then
  echo "[runtime-overlay] wheel odom EKF input preprocessor failed to stay alive" >&2
  exit 1
fi

IMU_BIAS_NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_local_state/lib/robot_local_state/imu_gyro_bias_filter_node"
[[ -x "${IMU_BIAS_NODE_BIN}" ]] || {
  echo "[runtime-overlay] compiled IMU gyro bias filter missing or not executable: ${IMU_BIAS_NODE_BIN}" >&2
  exit 1
}

njrh_start_affined_background imu_bias_pid robot_local_state_imu_bias_filter "${IMU_BIAS_NODE_BIN}" --ros-args \
  --params-file "${IMU_BIAS_FILTER_PARAMS_FILE}" \
  -r __node:=imu_gyro_bias_filter
sleep 1
if ! kill -0 "${imu_bias_pid}" 2>/dev/null; then
  echo "[runtime-overlay] IMU gyro bias filter failed to stay alive" >&2
  exit 1
fi

njrh_start_affined_background ekf_pid robot_local_state ros2 run robot_localization ekf_node --ros-args \
  --params-file "${EKF_PARAMS_FILE}" \
  -r __node:=robot_local_state \
  -r /odometry/filtered:=/local_state/odometry
wait "${ekf_pid}"

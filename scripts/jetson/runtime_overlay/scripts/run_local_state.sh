#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MODE="${LOCAL_STATE_MODE:-ekf}"

if [[ "${MODE}" == "passthrough" || "${MODE}" == "legacy" ]]; then
  PARAMS_FILE="${LOCAL_STATE_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_state.yaml}"
  [[ -f "${PARAMS_FILE}" ]] || {
    echo "[runtime-overlay] local state params file missing: ${PARAMS_FILE}" >&2
    exit 1
  }

  NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_local_state/lib/robot_local_state/local_state_node"
  [[ -x "${NODE_BIN}" ]] || {
    echo "[runtime-overlay] compiled local state node missing or not executable: ${NODE_BIN}" >&2
    echo "[runtime-overlay] build robot_local_state; Python fallback has been removed." >&2
    exit 1
  }

  exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"
fi

EKF_PARAMS_FILE="${LOCAL_STATE_EKF_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_state_ekf.yaml}"
[[ -f "${EKF_PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] local state EKF params file missing: ${EKF_PARAMS_FILE}" >&2
  exit 1
}

if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  echo "[runtime-overlay] ROS package missing: robot_localization" >&2
  echo "[runtime-overlay] install ros-humble-robot-localization in the NJRH-car image/container." >&2
  exit 1
fi

exec ros2 run robot_localization ekf_node --ros-args \
  --params-file "${EKF_PARAMS_FILE}" \
  -r __node:=robot_local_state \
  -r /odometry/filtered:=/local_state/odometry

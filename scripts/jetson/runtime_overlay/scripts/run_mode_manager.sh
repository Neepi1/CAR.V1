#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PARAMS_FILE="${ROBOT_MODE_MANAGER_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/mode_manager.yaml}"
[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] robot mode-manager params file missing: ${PARAMS_FILE}" >&2
  exit 1
}

NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_mode_manager/lib/robot_mode_manager/mode_manager_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] robot_mode_manager binary missing: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build it with: colcon build --packages-select robot_interfaces robot_mode_manager" >&2
  exit 1
}

exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"

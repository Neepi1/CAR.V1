#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_global_localization/lib/robot_global_localization/global_localization_node.py"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] robot_global_localization executable missing: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build it with: colcon build --packages-select robot_interfaces robot_global_localization" >&2
  exit 1
}

PARAMS_FILE="${GLOBAL_LOCALIZATION_PARAMS_FILE:-${NJRH_PROJECT_ROOT}/install/robot_global_localization/share/robot_global_localization/config/global_localization.yaml}"
if [[ -f "${PARAMS_FILE}" ]]; then
  exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"
fi

exec "${NODE_BIN}"

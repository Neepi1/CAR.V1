#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

if [[ -f "${NJRH_PROJECT_ROOT}/install/setup.bash" ]]; then
  set +u
  source "${NJRH_PROJECT_ROOT}/install/setup.bash"
  set -u
else
  echo "[runtime-overlay] project install missing: ${NJRH_PROJECT_ROOT}/install/setup.bash" >&2
  echo "[runtime-overlay] build robot_docking_manager before starting docking." >&2
  exit 1
fi

PARAMS_FILE="${DOCKING_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/docking.yaml}"
NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_docking_manager/lib/robot_docking_manager/docking_manager_node"

[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] docking params missing: ${PARAMS_FILE}" >&2
  exit 1
}

[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] build robot_docking_manager; Python fallback has been removed." >&2
  exit 1
}

echo "[runtime-overlay] starting robot_docking_manager with ${PARAMS_FILE}" >&2
njrh_exec_affined docking_manager "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"

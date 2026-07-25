#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

OWNER_LOCK_FILE="${NJRH_ORBBEC_PERCEPTION_OWNER_LOCK_FILE:-/tmp/njrh_orbbec_docking_perception.lock}"
exec 9>"${OWNER_LOCK_FILE}"
if ! flock -n 9; then
  echo "[runtime-overlay] Orbbec docking perception owner already running; refusing duplicate observer" >&2
  exit 73
fi

set +u
source /opt/ros/humble/setup.bash
source "${NJRH_PROJECT_ROOT}/install/setup.bash"
set -u

PARAMS_FILE="${ORBBEC_DOCKING_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/orbbec_docking_perception.yaml}"
NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_docking_perception/lib/robot_docking_perception/orbbec_depth_dock_node"

[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] Orbbec docking params missing: ${PARAMS_FILE}" >&2
  exit 1
}
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] build robot_interfaces robot_docking_perception first" >&2
  exit 1
}

echo "[runtime-overlay] starting Orbbec depth docking observer with ${PARAMS_FILE}" >&2
njrh_exec_affined docking_vision "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"

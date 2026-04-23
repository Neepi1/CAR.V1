#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
REPO_ROOT="$(cd "${NJRH_OVERLAY_ROOT}/../../.." && pwd)"

PARAMS_FILE="${LOCAL_PERCEPTION_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_perception.yaml}"
[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] local perception params file missing: ${PARAMS_FILE}" >&2
  exit 1
}

NODE_SCRIPT="${REPO_ROOT}/src/robot_local_perception/scripts/local_perception_node.py"
[[ -f "${NODE_SCRIPT}" ]] || {
  echo "[runtime-overlay] local perception node missing: ${NODE_SCRIPT}" >&2
  exit 1
}

NODE_BIN="${REPO_ROOT}/install/robot_local_perception/lib/robot_local_perception/local_perception_node"
if [[ "${NJRH_USE_CPP_LOCAL_PERCEPTION:-auto}" == "1" && -x "${NODE_BIN}" ]]; then
  exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"
fi

if [[ "${NJRH_USE_CPP_LOCAL_PERCEPTION:-auto}" == "auto" && -x "${NODE_BIN}" ]]; then
  exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"
fi

exec env PYTHONUNBUFFERED=1 python3 "${NODE_SCRIPT}" --ros-args --params-file "${PARAMS_FILE}"

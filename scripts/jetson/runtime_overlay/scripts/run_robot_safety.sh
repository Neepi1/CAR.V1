#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
REPO_ROOT="$(cd "${NJRH_OVERLAY_ROOT}/../../.." && pwd)"

PARAMS_FILE="${ROBOT_SAFETY_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/robot_safety.yaml}"
[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] robot safety params file missing: ${PARAMS_FILE}" >&2
  exit 1
}

NODE_SCRIPT="${REPO_ROOT}/src/robot_safety/scripts/robot_safety_node.py"
[[ -f "${NODE_SCRIPT}" ]] || {
  echo "[runtime-overlay] robot safety node missing: ${NODE_SCRIPT}" >&2
  exit 1
}

exec env PYTHONUNBUFFERED=1 python3 "${NODE_SCRIPT}" --ros-args --params-file "${PARAMS_FILE}"

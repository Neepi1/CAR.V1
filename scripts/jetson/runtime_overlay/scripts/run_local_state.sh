#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

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

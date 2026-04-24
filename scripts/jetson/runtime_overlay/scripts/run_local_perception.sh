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

NODE_BIN="${REPO_ROOT}/install/robot_local_perception/lib/robot_local_perception/local_perception_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] compiled local perception node missing or not executable: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build robot_local_perception; Python fallback has been removed." >&2
  exit 1
}

exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"

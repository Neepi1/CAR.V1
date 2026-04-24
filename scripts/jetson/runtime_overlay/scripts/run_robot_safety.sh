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

NODE_BIN="${REPO_ROOT}/install/robot_safety/lib/robot_safety/robot_safety_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] compiled robot safety node missing or not executable: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build robot_safety; Python fallback has been removed." >&2
  exit 1
}

exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"

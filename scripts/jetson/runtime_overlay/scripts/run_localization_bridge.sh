#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PARAMS_FILE="${LOCALIZATION_BRIDGE_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/localization_bridge.yaml}"
[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] localization bridge params file missing: ${PARAMS_FILE}" >&2
  exit 1
}

NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_localization_bridge/lib/robot_localization_bridge/localization_bridge_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] compiled localization bridge node missing or not executable: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build robot_localization_bridge; Python fallback has been removed." >&2
  exit 1
}

exec "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}"

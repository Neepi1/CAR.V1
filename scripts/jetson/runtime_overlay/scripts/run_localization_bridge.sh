#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PARAMS_FILE="${LOCALIZATION_BRIDGE_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/localization_bridge.yaml}"
[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] localization bridge params file missing: ${PARAMS_FILE}" >&2
  exit 1
}

exec env PYTHONUNBUFFERED=1 python3 "${SCRIPT_DIR}/localization_bridge_node.py" --ros-args --params-file "${PARAMS_FILE}"

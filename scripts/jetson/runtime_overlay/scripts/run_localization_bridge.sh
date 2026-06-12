#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

PARAMS_FILE="${LOCALIZATION_BRIDGE_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/localization_bridge.yaml}"
[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] localization bridge params file missing: ${PARAMS_FILE}" >&2
  exit 1
}
ISAAC_LOCALIZATION_MODE="${NJRH_ISAAC_LOCALIZATION_MODE:-triggered}"
case "${ISAAC_LOCALIZATION_MODE}" in
  triggered)
    ;;
  *)
    echo "[runtime-overlay] invalid NJRH_ISAAC_LOCALIZATION_MODE=${ISAAC_LOCALIZATION_MODE}; expected triggered. Isaac continuous localization has been removed; use NJRH_AMCL_LOCALIZATION_MODE=shadow|gated for continuous correction candidates." >&2
    exit 2
    ;;
esac
AMCL_LOCALIZATION_MODE="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
case "${AMCL_LOCALIZATION_MODE}" in
  disabled|shadow|gated)
    ;;
  *)
    echo "[runtime-overlay] invalid NJRH_AMCL_LOCALIZATION_MODE=${AMCL_LOCALIZATION_MODE}; expected disabled, shadow, or gated" >&2
    exit 2
    ;;
esac
AMCL_INPUT_ENABLED="false"
AMCL_GATE_MODE="shadow"
if [[ "${AMCL_LOCALIZATION_MODE}" == "shadow" || "${AMCL_LOCALIZATION_MODE}" == "gated" ]]; then
  AMCL_INPUT_ENABLED="true"
  AMCL_GATE_MODE="${AMCL_LOCALIZATION_MODE}"
fi

NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_localization_bridge/lib/robot_localization_bridge/localization_bridge_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] compiled localization bridge node missing or not executable: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build robot_localization_bridge; Python fallback has been removed." >&2
  exit 1
}

njrh_exec_affined robot_localization_bridge "${NODE_BIN}" --ros-args \
  --params-file "${PARAMS_FILE}" \
  -p "continuous_localization_mode:=triggered" \
  -p "amcl_input_enabled:=${AMCL_INPUT_ENABLED}" \
  -p "amcl_gate_mode:=${AMCL_GATE_MODE}" \
  -p "amcl_pose_topic:=${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}" \
  -p "amcl_initial_pose_topic:=${NJRH_AMCL_INITIAL_POSE_TOPIC:-/initialpose}"

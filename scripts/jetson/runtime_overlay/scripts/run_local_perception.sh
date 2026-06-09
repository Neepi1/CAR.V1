#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"
njrh_load_local_perception_input_profile
REPO_ROOT="$(cd "${NJRH_OVERLAY_ROOT}/../../.." && pwd)"

PARAMS_FILE="${LOCAL_PERCEPTION_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/local_perception.yaml}"
INPUT_TOPIC="${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}"
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

echo "[runtime-overlay] starting local perception without startup topic/TF probes" >&2
njrh_print_local_perception_profile

njrh_exec_affined robot_local_perception "${NODE_BIN}" \
  --ros-args \
  --params-file "${PARAMS_FILE}" \
  -p "input_topic:=${INPUT_TOPIC}"

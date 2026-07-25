#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

if [[ -f "${NJRH_PROJECT_ROOT}/install/setup.bash" ]]; then
  set +u
  source "${NJRH_PROJECT_ROOT}/install/setup.bash"
  set -u
else
  echo "[runtime-overlay] project install missing: ${NJRH_PROJECT_ROOT}/install/setup.bash" >&2
  echo "[runtime-overlay] build robot_docking_manager before starting docking." >&2
  exit 1
fi

PARAMS_FILE="${DOCKING_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/docking.yaml}"
NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_docking_manager/lib/robot_docking_manager/docking_manager_node"
DOCKING_SENSOR_BACKEND="${NJRH_DOCKING_SENSOR_BACKEND:-orbbec_336l}"
backend_args=()

[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] docking params missing: ${PARAMS_FILE}" >&2
  exit 1
}

[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] build robot_docking_manager; Python fallback has been removed." >&2
  exit 1
}

case "${DOCKING_SENSOR_BACKEND}" in
  gs2)
    backend_args+=(
      -p observation_backend:=gs2_scan
      -p approach.allow_blind_approach:=true
      -p approach.final_target_distance_m:=0.05
      -p controller.lateral_command_sign:=-1.0
      -p controller.contact_verify_max_distance_m:=0.12
      -p controller.contact_verify_retry_enabled:=true
      -p tolerances.contact_confirm_timeout_s:=3.0
    )
    ;;
  orbbec_336l)
    backend_args+=(
      -p observation_backend:=target_observation
      -p target_observation_topic:=/dock/target_observation
      -p target_observation_source:=orbbec_336l_depth
      -p approach.allow_blind_approach:=false
      -p approach.final_target_distance_m:=0.34
      -p controller.lateral_command_sign:=1.0
      -p controller.contact_verify_max_distance_m:=0.31
      -p controller.contact_verify_retry_enabled:=true
      -p controller.contact_retry_max_count:=2
      -p controller.contact_retry_backoff_distance_m:=0.60
      -p controller.contact_retry_backoff_speed_mps:=0.06
      -p controller.contact_retry_backoff_timeout_s:=20.0
      -p tolerances.contact_confirm_timeout_s:=16.0
    )
    ;;
  *)
    echo "[runtime-overlay] unsupported NJRH_DOCKING_SENSOR_BACKEND=${DOCKING_SENSOR_BACKEND}" >&2
    exit 1
    ;;
esac

echo "[runtime-overlay] starting robot_docking_manager backend=${DOCKING_SENSOR_BACKEND} with ${PARAMS_FILE}" >&2
njrh_exec_affined docking_manager "${NODE_BIN}" --ros-args --params-file "${PARAMS_FILE}" "${backend_args[@]}"

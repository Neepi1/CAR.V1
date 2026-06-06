#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"

export NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"

stop_existing_overlay_nav_helpers

nav_pid=""
nav_exit_code=0
cleanup_started=0

cleanup() {
  if [[ "${cleanup_started}" -eq 1 ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  if [[ -n "${nav_pid}" ]]; then
    kill -INT "${nav_pid}" 2>/dev/null || true
    wait "${nav_pid}" 2>/dev/null || true
  fi
  cleanup_overlay_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

if [[ "${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER:-true}" == "true" ]]; then
  echo "[runtime-overlay] local_perception is owned by pointcloud_perception_pipeline; skipping standalone rapid local_perception" >&2
else
  start_overlay_helper "local_perception_rapid" bash "${SCRIPT_DIR}/run_local_perception.sh"
fi
start_overlay_helper "robot_safety_rapid" bash "${SCRIPT_DIR}/run_robot_safety.sh"
start_overlay_helper "ranger_mini3_mode_controller_rapid" bash "${SCRIPT_DIR}/run_ranger_mini3_mode_controller.sh"

bash "$(require_upstream_script run_nav2_rapid_avoidance.sh)" &
nav_pid=$!
wait "${nav_pid}" || nav_exit_code=$?
exit "${nav_exit_code}"

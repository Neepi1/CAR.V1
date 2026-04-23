#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"

export NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
LAUNCH_FILE="${NJRH_PROJECT_ROOT}/src/robot_bringup/launch/standard_navigation.launch.py"

[[ -f "${LAUNCH_FILE}" ]] || {
  echo "[runtime-overlay] missing repository launch file: ${LAUNCH_FILE}" >&2
  exit 1
}

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

start_overlay_helper "local_perception" bash "${SCRIPT_DIR}/run_local_perception.sh"
start_overlay_helper "robot_safety" bash "${SCRIPT_DIR}/run_robot_safety.sh"

ros2 launch "${LAUNCH_FILE}" \
  use_sim_time:=false \
  autostart:=true \
  params_file:="${NAV2_PARAMS_FILE}" &
nav_pid=$!
wait "${nav_pid}" || nav_exit_code=$?
exit "${nav_exit_code}"

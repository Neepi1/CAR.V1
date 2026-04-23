#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"

export NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
LAUNCH_FILE="${NJRH_PROJECT_ROOT}/src/robot_bringup/launch/local_costmap_debug.launch.py"

[[ -f "${LAUNCH_FILE}" ]] || {
  echo "[runtime-overlay] missing repository launch file: ${LAUNCH_FILE}" >&2
  exit 1
}

stop_existing_overlay_nav_helpers
stop_existing_canonical_tf_publishers
kill_overlay_pattern "ros2 launch .*local_costmap_debug.launch.py"
kill_overlay_pattern "lifecycle_manager_local_costmap_debug"
kill_overlay_pattern "controller_server"

costmap_pid=""
costmap_exit_code=0
cleanup_started=0

cleanup() {
  if [[ "${cleanup_started}" -eq 1 ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  if [[ -n "${costmap_pid}" ]]; then
    kill -INT "${costmap_pid}" 2>/dev/null || true
    wait "${costmap_pid}" 2>/dev/null || true
  fi
  cleanup_overlay_helpers
  cleanup_canonical_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

start_canonical_helper "robot_description" bash "${SCRIPT_DIR}/run_robot_description.sh"
start_canonical_helper "local_state" bash "${SCRIPT_DIR}/run_local_state.sh"
start_overlay_helper "local_perception" bash "${SCRIPT_DIR}/run_local_perception.sh"

ros2 launch "${LAUNCH_FILE}" \
  use_sim_time:=false \
  autostart:=true \
  params_file:="${NAV2_PARAMS_FILE}" &
costmap_pid=$!
wait "${costmap_pid}" || costmap_exit_code=$?
exit "${costmap_exit_code}"

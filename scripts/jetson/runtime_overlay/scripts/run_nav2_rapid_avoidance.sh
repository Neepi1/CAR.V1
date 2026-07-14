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

echo "[runtime-overlay] local_perception rapid helper disabled; rapid avoidance consumes /scan through Nav2 standard layers" >&2
start_overlay_helper "robot_safety_rapid" bash "${SCRIPT_DIR}/run_robot_safety.sh"

bash "$(require_upstream_script run_nav2_rapid_avoidance.sh)" &
nav_pid=$!
wait "${nav_pid}" || nav_exit_code=$?
exit "${nav_exit_code}"

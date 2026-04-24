#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

helper_pids=()

kill_overlay_pattern() {
  local pattern="$1"
  pkill -INT -f "$pattern" 2>/dev/null || true
  sleep 1
  pkill -9 -f "$pattern" 2>/dev/null || true
}

stop_existing_overlay_nav_helpers() {
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_local_perception.sh"
  kill_overlay_pattern "/install/robot_local_perception/lib/robot_local_perception/local_perception_node"
  kill_overlay_pattern "robot_local_perception/local_perception_node"
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_robot_safety.sh"
  kill_overlay_pattern "/install/robot_safety/lib/robot_safety/robot_safety_node"
  kill_overlay_pattern "robot_safety/robot_safety_node"
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_ranger_mini3_mode_controller.sh"
  kill_overlay_pattern "ranger_mini3_mode_controller/mode_controller_node.py"
  kill_overlay_pattern "python3 .*mode_controller_node.py"
  kill_overlay_pattern "/install/ranger_mini3_mode_controller/lib/ranger_mini3_mode_controller/mode_controller_node"
  kill_overlay_pattern "ranger_mini3_mode_controller/mode_controller_node"
}

start_overlay_helper() {
  local helper_name="$1"
  shift
  local helper_log="${NJRH_RUNTIME_LOG_DIR}/${helper_name}.log"
  echo "[runtime-overlay] starting ${helper_name}" >&2
  "$@" >>"${helper_log}" 2>&1 &
  local helper_pid=$!
  helper_pids+=("${helper_pid}")
  sleep 1
  if ! kill -0 "${helper_pid}" 2>/dev/null; then
    echo "[runtime-overlay] helper failed to stay alive: ${helper_name}. Check ${helper_log}" >&2
    return 1
  fi
  echo "[runtime-overlay] helper ready: ${helper_name} (pid=${helper_pid})" >&2
}

cleanup_overlay_helpers() {
  local helper_pid
  for helper_pid in "${helper_pids[@]:-}"; do
    kill -INT "${helper_pid}" 2>/dev/null || true
  done
  sleep 1
  for helper_pid in "${helper_pids[@]:-}"; do
    kill -9 "${helper_pid}" 2>/dev/null || true
  done
  helper_pids=()
}

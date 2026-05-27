#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/map_server_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"

export NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
LAUNCH_FILE="${NJRH_PROJECT_ROOT}/src/robot_bringup/launch/standard_navigation.launch.py"

if [[ -n "${NJRH_FLOOR_ID:-}" || -n "${NAV2_FLOOR_ID:-}" ]]; then
  resolve_floor_assets "${NJRH_BUILDING_ID:-${NAV2_BUILDING_ID:-building_1}}" "${NJRH_FLOOR_ID:-${NAV2_FLOOR_ID:-}}"
fi

[[ -f "${LAUNCH_FILE}" ]] || {
  echo "[runtime-overlay] missing repository launch file: ${LAUNCH_FILE}" >&2
  exit 1
}

stop_existing_overlay_nav_helpers
stop_existing_standard_nav_stack

ensure_map_server_active "${NAV2_MAP_YAML:-}" 30 || {
  echo "[runtime-overlay] map_server is not active. Start localization and load a Nav2 map before navigation." >&2
  exit 1
}

wait_for_occupancy_grid "/map" 30 || {
  echo "[runtime-overlay] /map is not available. Navigation cannot build the global costmap." >&2
  exit 1
}

ensure_costmap_filter_masks() {
  local keepout="${NAV2_KEEP_OUT_MASK_YAML:-}"
  local speed="${NAV2_SPEED_MASK_YAML:-}"
  if [[ -n "${keepout}" && -f "${keepout}" && -n "${speed}" && -f "${speed}" ]]; then
    export NAV2_KEEP_OUT_MASK_YAML="${keepout}"
    export NAV2_SPEED_MASK_YAML="${speed}"
    return 0
  fi

  local neutral_dir="${NJRH_OVERLAY_ROOT}/filters/runtime_neutral"
  local generator="${SCRIPT_DIR}/ensure_costmap_filter_masks.py"
  [[ -f "${generator}" ]] || {
    echo "[runtime-overlay] missing costmap filter mask generator: ${generator}" >&2
    return 1
  }

  if [[ -n "${NAV2_MAP_YAML:-}" && -f "${NAV2_MAP_YAML}" ]]; then
    eval "$(python3 "${generator}" --nav-yaml "${NAV2_MAP_YAML}" --output-dir "${neutral_dir}")"
  else
    eval "$(python3 "${generator}" --output-dir "${neutral_dir}")"
  fi
  export NAV2_KEEP_OUT_MASK_YAML NAV2_SPEED_MASK_YAML NAV2_BINARY_MASK_YAML
}

ensure_costmap_filter_masks || {
  echo "[runtime-overlay] failed to prepare costmap filter masks" >&2
  exit 1
}

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
start_overlay_helper "floor_manager" bash "${SCRIPT_DIR}/run_floor_manager.sh"
start_overlay_helper "robot_safety" bash "${SCRIPT_DIR}/run_robot_safety.sh"
start_overlay_helper "ranger_mini3_mode_controller" bash "${SCRIPT_DIR}/run_ranger_mini3_mode_controller.sh"

ros2 launch "${LAUNCH_FILE}" \
  use_sim_time:=false \
  autostart:=true \
  params_file:="${NAV2_PARAMS_FILE}" \
  keepout_mask_yaml:="${NAV2_KEEP_OUT_MASK_YAML}" \
  speed_mask_yaml:="${NAV2_SPEED_MASK_YAML}" &
nav_pid=$!

wait_for_global_costmap_static 45 || {
  echo "[runtime-overlay] global costmap did not receive the static map; stopping navigation startup" >&2
  exit 1
}

wait "${nav_pid}" || nav_exit_code=$?
exit "${nav_exit_code}"

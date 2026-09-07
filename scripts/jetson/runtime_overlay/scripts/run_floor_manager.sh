#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_floor_manager/lib/robot_floor_manager/floor_manager_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] robot_floor_manager binary missing: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build it with: colcon build --packages-select robot_map_asset_identity robot_interfaces robot_floor_manager" >&2
  exit 1
}

exec "${NODE_BIN}" --ros-args \
  -p maps_root:="${NJRH_RELEASE_ASSETS_DIR}" \
  -p default_building_id:="${NJRH_BUILDING_ID:-building_1}" \
  -p live_floor_switch_enabled:="${NJRH_LIVE_FLOOR_SWITCH_ENABLED:-true}" \
  -p speed_filter_enabled:="${NJRH_ENABLE_SPEED_FILTER:-false}" \
  -p localizer_apply_timeout_sec:="${NJRH_FLOOR_LOCALIZER_APPLY_TIMEOUT_SEC:-30.0}" \
  -p localization_trigger_timeout_sec:="${NJRH_FLOOR_LOCALIZATION_TRIGGER_TIMEOUT_SEC:-75.0}" \
  -p navigate_to_pose_action:="${NJRH_NAVIGATE_TO_POSE_ACTION:-/navigate_to_pose}" \
  -p nav_idle_bootstrap_grace_sec:="${NJRH_NAV_IDLE_BOOTSTRAP_GRACE_SEC:-2.0}"

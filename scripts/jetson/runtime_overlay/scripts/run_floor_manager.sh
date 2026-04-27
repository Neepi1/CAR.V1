#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_floor_manager/lib/robot_floor_manager/floor_manager_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] robot_floor_manager binary missing: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build it with: colcon build --packages-select robot_interfaces robot_floor_manager" >&2
  exit 1
}

exec "${NODE_BIN}" --ros-args \
  -p maps_root:="${NJRH_RELEASE_ASSETS_DIR}" \
  -p default_building_id:="${NJRH_BUILDING_ID:-building_1}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/map_server_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"

building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
floor_id="${2:-${NJRH_FLOOR_ID:-}}"

[[ -n "${floor_id}" ]] || {
  echo "[runtime-overlay] floor_id is required for floor navigation resume" >&2
  exit 1
}

resolve_floor_assets "${building_id}" "${floor_id}"

stop_existing_navigation_stack() {
  local patterns=(
    "run_nav2_navigation.sh"
    "standard_navigation.launch.py"
    "controller_server"
    "planner_server"
    "bt_navigator"
    "behavior_server"
    "smoother_server"
    "waypoint_follower"
    "velocity_smoother"
    "collision_monitor"
    "lifecycle_manager_navigation"
    "keepout_filter_mask_server"
    "speed_filter_mask_server"
    "keepout_costmap_filter_info_server"
    "speed_costmap_filter_info_server"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -INT -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
}

localization_pid=""
navigation_pid=""
exit_code=0
cleanup_started=0

cleanup() {
  if [[ "${cleanup_started}" -eq 1 ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  if [[ -n "${navigation_pid}" ]]; then
    kill -INT "${navigation_pid}" 2>/dev/null || true
  fi
  if [[ -n "${localization_pid}" ]]; then
    kill -INT "${localization_pid}" 2>/dev/null || true
  fi
  cleanup_overlay_helpers
  sleep 1
  if [[ -n "${navigation_pid}" ]]; then
    kill -9 "${navigation_pid}" 2>/dev/null || true
    wait "${navigation_pid}" 2>/dev/null || true
  fi
  if [[ -n "${localization_pid}" ]]; then
    kill -9 "${localization_pid}" 2>/dev/null || true
    wait "${localization_pid}" 2>/dev/null || true
  fi
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

write_runtime_map_context() {
  local state="$1"
  local confirmed="$2"
  local message="$3"
  [[ -n "${NJRH_RUNTIME_MAP_CONTEXT_FILE:-}" && -n "${NJRH_MAP_ID:-}" ]] || return 0
  python3 - "$state" "$confirmed" "$message" <<'PY'
import json
import os
import sys
import time

path = os.environ.get("NJRH_RUNTIME_MAP_CONTEXT_FILE", "")
if not path:
    raise SystemExit(0)

state = sys.argv[1]
confirmed = sys.argv[2].lower() == "true"
message = sys.argv[3]
data = {
    "schema": "njrh.runtime_map_context.v1",
    "state": state,
    "confirmed": confirmed,
    "message": message,
    "map_id": os.environ.get("NJRH_MAP_ID", ""),
    "display_name": os.environ.get("NJRH_MAP_DISPLAY_NAME", ""),
    "building_id": os.environ.get("NJRH_MAP_CONTEXT_BUILDING_ID") or os.environ.get("NJRH_BUILDING_ID", ""),
    "floor_id": os.environ.get("NJRH_MAP_CONTEXT_FLOOR_ID") or os.environ.get("NJRH_FLOOR_ID", ""),
    "updated_at": time.time(),
}
os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
tmp_path = f"{path}.tmp"
with open(tmp_path, "w", encoding="utf-8") as file:
    json.dump(data, file, ensure_ascii=False, separators=(",", ":"))
    file.write("\n")
os.replace(tmp_path, path)
PY
}

echo "[runtime-overlay] floor navigation resume requested: ${NJRH_BUILDING_ID}/${NJRH_FLOOR_ID}" >&2
write_runtime_map_context "starting" "false" "floor navigation runtime starting"
stop_existing_navigation_stack

NJRH_BUILDING_ID="${NJRH_BUILDING_ID}" \
NJRH_FLOOR_ID="${NJRH_FLOOR_ID}" \
bash "${SCRIPT_DIR}/run_occupancy_grid_localization.sh" &
localization_pid=$!

wait_for_ros_service "/global_localization/apply_floor_assets" 30 || {
  echo "[runtime-overlay] /global_localization/apply_floor_assets did not become ready" >&2
  exit 1
}

wait_for_ros_service "/global_localization/trigger" 30 || {
  echo "[runtime-overlay] /global_localization/trigger did not become ready" >&2
  exit 1
}

wait_for_ros_service "/trigger_grid_search_localization" 45 || {
  echo "[runtime-overlay] Isaac /trigger_grid_search_localization did not become ready" >&2
  exit 1
}

wait_for_occupancy_grid "/map" 45 || {
  echo "[runtime-overlay] /map did not become ready after starting localization" >&2
  exit 1
}

wait_for_topic_message "/flatscan" 45 || {
  echo "[runtime-overlay] /flatscan did not become ready after starting localization" >&2
  exit 1
}

start_overlay_helper "floor_manager" bash "${SCRIPT_DIR}/run_floor_manager.sh"
wait_for_ros_service "/floor_manager/switch_floor" 30 || {
  echo "[runtime-overlay] floor_manager switch service did not become ready" >&2
  exit 1
}

payload="{building_id: '${NJRH_BUILDING_ID}', floor_id: '${NJRH_FLOOR_ID}', resume_navigation: true}"
timeout 45 ros2 service call /floor_manager/switch_floor robot_interfaces/srv/SwitchFloor "${payload}" >/dev/null || {
  echo "[runtime-overlay] floor_manager switch/localization trigger failed before Nav2 startup" >&2
  exit 1
}

if wait_for_topic_message "/localization_result" 8; then
  echo "[runtime-overlay] Isaac localization_result observed after floor switch" >&2
else
  echo "[runtime-overlay] localization_result was not observed directly; checking map->odom bridge state" >&2
fi

wait_for_tf_transform "map" "odom" 60 || {
  echo "[runtime-overlay] map->odom did not become ready after floor switch localization" >&2
  exit 1
}

echo "[runtime-overlay] map->odom ready; starting standard Nav2 navigation stack" >&2

NJRH_BUILDING_ID="${NJRH_BUILDING_ID}" \
NJRH_FLOOR_ID="${NJRH_FLOOR_ID}" \
bash "${SCRIPT_DIR}/run_nav2_navigation.sh" &
navigation_pid=$!

wait_for_global_costmap_static 60 || {
  echo "[runtime-overlay] global costmap did not become ready after starting Nav2" >&2
  exit 1
}
write_runtime_map_context "ready" "true" "map->odom and Nav2 global costmap are ready"

while true; do
  if [[ -n "${localization_pid}" ]] && ! kill -0 "${localization_pid}" 2>/dev/null; then
    wait "${localization_pid}" || exit_code=$?
    echo "[runtime-overlay] localization process exited with ${exit_code}" >&2
    exit "${exit_code}"
  fi
  if [[ -n "${navigation_pid}" ]] && ! kill -0 "${navigation_pid}" 2>/dev/null; then
    wait "${navigation_pid}" || exit_code=$?
    echo "[runtime-overlay] navigation process exited with ${exit_code}" >&2
    exit "${exit_code}"
  fi
  sleep 2
done

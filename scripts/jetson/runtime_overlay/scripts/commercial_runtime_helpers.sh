#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/map_server_helpers.sh"

critical_nav2_lifecycle_nodes() {
  printf '%s\n' \
    /controller_server \
    /planner_server \
    /bt_navigator \
    /behavior_server \
    /velocity_smoother \
    /collision_monitor
}

write_runtime_map_context() {
  local state="$1"
  local confirmed="$2"
  local message="$3"
  local context_map_id="${NJRH_MAP_ID:-${NJRH_NAV_MAP_ID:-}}"
  local context_display_name="${NJRH_MAP_DISPLAY_NAME:-${NJRH_NAV_MAP_NAME:-}}"
  [[ -n "${NJRH_RUNTIME_MAP_CONTEXT_FILE:-}" ]] || return 0
  if [[ -z "${context_map_id}" ]]; then
    echo "[runtime-overlay] WARN: runtime map context skipped because no map_id is available from resolved floor assets" >&2
    return 0
  fi
  export NJRH_MAP_ID="${context_map_id}"
  export NJRH_MAP_DISPLAY_NAME="${context_display_name}"
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
tmp_path = f"{path}.{os.getpid()}.tmp"
try:
    with open(tmp_path, "w", encoding="utf-8") as file:
        json.dump(data, file, ensure_ascii=False, separators=(",", ":"))
        file.write("\n")
    os.replace(tmp_path, path)
finally:
    try:
        os.unlink(tmp_path)
    except FileNotFoundError:
        pass
PY
}

nav2_lifecycle_node_active() {
  local node_name="$1"
  local timeout_sec="${2:-8}"
  runtime_readiness_probe lifecycle-active "${node_name}" "${timeout_sec}"
}

map_server_asset_matches_current_floor() {
  [[ -n "${NAV2_MAP_YAML:-}" ]] || return 1
  map_server_publishing_requested_map "${NAV2_MAP_YAML}"
}

runtime_map_context_matches_current_floor() {
  [[ -n "${NJRH_RUNTIME_MAP_CONTEXT_FILE:-}" && -f "${NJRH_RUNTIME_MAP_CONTEXT_FILE}" ]] || return 1
  local context_map_id="${NJRH_MAP_ID:-${NJRH_NAV_MAP_ID:-}}"
  [[ -n "${context_map_id}" ]] || return 1
  export NJRH_MAP_ID="${context_map_id}"
  export NJRH_MAP_DISPLAY_NAME="${NJRH_MAP_DISPLAY_NAME:-${NJRH_NAV_MAP_NAME:-}}"
  python3 <<'PY'
import json
import os

path = os.environ.get("NJRH_RUNTIME_MAP_CONTEXT_FILE", "")
try:
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
except Exception:
    raise SystemExit(1)

expected = {
    "map_id": os.environ.get("NJRH_MAP_ID", ""),
    "building_id": os.environ.get("NJRH_MAP_CONTEXT_BUILDING_ID") or os.environ.get("NJRH_BUILDING_ID", ""),
    "floor_id": os.environ.get("NJRH_MAP_CONTEXT_FLOOR_ID") or os.environ.get("NJRH_FLOOR_ID", ""),
}
ok = (
    data.get("confirmed") is True
    and data.get("state") == "ready"
    and data.get("map_id") == expected["map_id"]
    and data.get("building_id") == expected["building_id"]
    and data.get("floor_id") == expected["floor_id"]
)
raise SystemExit(0 if ok else 1)
PY
}

navigation_map_source_diagnostics() {
  local quick_timeout="${1:-3}"
  if map_server_asset_matches_current_floor >/dev/null 2>&1; then
    echo "[runtime-overlay] map source diagnostic: map_server asset matches selected floor" >&2
  else
    echo "[runtime-overlay] map source diagnostic: map_server asset is not directly confirmable; using global costmap static-map readiness as the final map gate" >&2
  fi

  if wait_for_occupancy_grid "/map" "${quick_timeout}" >/dev/null 2>&1; then
    echo "[runtime-overlay] map source diagnostic: /map OccupancyGrid is observable" >&2
  else
    echo "[runtime-overlay] map source diagnostic: /map was not observed during the short final check; global costmap remains authoritative after Nav2 activation" >&2
  fi
}

standard_nav_stack_lifecycle_active() {
  local timeout_sec="${1:-8}"
  local node_name
  for node_name in $(critical_nav2_lifecycle_nodes); do
    nav2_lifecycle_node_active "${node_name}" "${timeout_sec}" || return 1
  done
}

navigation_ready_gate() {
  local label="$1"
  shift
  local output
  if output="$("$@" 2>&1)"; then
    [[ -z "${output}" ]] || echo "${output}" >&2
    return 0
  fi
  [[ -z "${output}" ]] || echo "${output}" >&2
  echo "[runtime-overlay] navigation readiness failed: ${label}" >&2
  return 1
}

navigation_runtime_ready_for_current_floor() {
  local quick_timeout="${1:-3}"
  navigation_map_source_diagnostics "${quick_timeout}" || true
  navigation_ready_gate "local_state_endpoint" local_state_endpoint_ready "${quick_timeout}" || return 1
  navigation_ready_gate "local_odom_topic" wait_for_fresh_header_topic_message \
    "/local_state/odometry" \
    "${quick_timeout}" \
    "${NJRH_NAV_LOCAL_ODOM_MAX_AGE_SEC:-0.75}" \
    "${NJRH_NAV_LOCAL_ODOM_MAX_FUTURE_SEC:-0.25}" || return 1
  navigation_ready_gate "safety_status_topic" wait_for_topic_message "/safety/status" "${quick_timeout}" || return 1
  navigation_ready_gate "tf_odom_base_link" wait_for_fresh_tf_transform "odom" "base_link" \
    "${quick_timeout}" "${NJRH_NAV_TF_MAX_AGE_SEC:-0.25}" || return 1
  navigation_ready_gate "tf_map_odom" wait_for_tf_transform "map" "odom" "${quick_timeout}" || return 1
  navigation_ready_gate "global_costmap_static" wait_for_global_costmap_static "${quick_timeout}" || return 1
  navigation_ready_gate "local_costmap_observation" wait_for_local_costmap_observation_ready "${quick_timeout}" || return 1
  navigation_ready_gate "global_localization_trigger_service" wait_for_ros_service "/global_localization/trigger" "${quick_timeout}" || return 1
  navigation_ready_gate "isaac_grid_search_trigger_service" wait_for_ros_service "/trigger_grid_search_localization" "${quick_timeout}" || return 1
  navigation_ready_gate "nav2_lifecycle_active" standard_nav_stack_lifecycle_active "${quick_timeout}" || return 1
}

wait_for_navigation_global_costmap_ready() {
  local timeout_sec="${1:-120}"
  local deadline=$((SECONDS + timeout_sec))
  local probe_output

  while (( SECONDS < deadline )); do
    if probe_output="$(wait_for_global_costmap_static 2 2>&1)"; then
      echo "${probe_output}" >&2
      return 0
    fi
    sleep 0.5
  done

  wait_for_global_costmap_static 2 || true
  return 1
}

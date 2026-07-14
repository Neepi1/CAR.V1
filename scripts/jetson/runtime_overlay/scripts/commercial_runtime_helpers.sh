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
    /behavior_server \
    /smoother_server \
    /bt_navigator \
    /velocity_smoother \
    /collision_monitor
}

nav2_lifecycle_node_active() {
  local node_name="$1"
  local timeout_sec="${2:-8}"
  runtime_readiness_probe lifecycle-active "${node_name}" "${timeout_sec}"
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
    "startup_stage": os.environ.get("NJRH_RUNTIME_STARTUP_STAGE", ""),
    "confirmed": confirmed,
    "message": message,
    "map_id": os.environ.get("NJRH_MAP_ID", ""),
    "display_name": os.environ.get("NJRH_MAP_DISPLAY_NAME", ""),
    "building_id": os.environ.get("NJRH_MAP_CONTEXT_BUILDING_ID") or os.environ.get("NJRH_BUILDING_ID", ""),
    "floor_id": os.environ.get("NJRH_MAP_CONTEXT_FLOOR_ID") or os.environ.get("NJRH_FLOOR_ID", ""),
    "updated_at": time.time(),
}
for key, env_key in (
    ("failure_code", "NJRH_RUNTIME_FAILURE_CODE"),
    ("localization_mode", "NJRH_RUNTIME_LOCALIZATION_MODE"),
    ("last_triggered_relocalization_ok", "NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK"),
    ("map_to_odom_age_ms", "NJRH_RUNTIME_MAP_TO_ODOM_AGE_MS"),
    ("startup_elapsed_sec", "NJRH_RUNTIME_STARTUP_ELAPSED_SEC"),
):
    value = os.environ.get(env_key, "")
    if value == "":
        continue
    if key == "last_triggered_relocalization_ok":
        data[key] = value.lower() in ("1", "true", "yes", "on")
    elif key in ("map_to_odom_age_ms", "startup_elapsed_sec"):
        try:
            data[key] = float(value)
        except ValueError:
            data[key] = value
    else:
        data[key] = value
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

nav2_lifecycle_ready_status_file() {
  printf '%s\n' "${NJRH_NAV2_LIFECYCLE_READY_STATUS_FILE:-/tmp/njrh_nav2_lifecycle_ready.env}"
}

clear_nav2_lifecycle_ready_status() {
  rm -f "$(nav2_lifecycle_ready_status_file)" 2>/dev/null || true
}

write_nav2_lifecycle_ready_status() {
  local owner_pid="$1"
  local source_label="${2:-unknown}"
  local status_file
  local tmp_file
  status_file="$(nav2_lifecycle_ready_status_file)"
  tmp_file="${status_file}.$$"
  {
    printf 'NAV2_LIFECYCLE_READY=true\n'
    printf 'NAV2_LIFECYCLE_READY_STAMP_SEC=%q\n' "$(date +%s)"
    printf 'NAV2_LIFECYCLE_READY_OWNER_PID=%q\n' "${owner_pid}"
    printf 'NAV2_LIFECYCLE_READY_SOURCE=%q\n' "${source_label}"
  } >"${tmp_file}"
  mv -f "${tmp_file}" "${status_file}"
}

nav2_lifecycle_ready_status_matches() (
  local status_file
  local now_sec
  local age_sec
  local max_age_sec="${NJRH_NAV2_LIFECYCLE_READY_STATUS_MAX_AGE_SEC:-600}"
  status_file="$(nav2_lifecycle_ready_status_file)"
  [[ -r "${status_file}" ]] || return 1
  NAV2_LIFECYCLE_READY=""
  NAV2_LIFECYCLE_READY_STAMP_SEC=""
  NAV2_LIFECYCLE_READY_OWNER_PID=""
  NAV2_LIFECYCLE_READY_SOURCE=""
  # shellcheck disable=SC1090
  source "${status_file}" 2>/dev/null || return 1
  [[ "${NAV2_LIFECYCLE_READY:-false}" == "true" ]] || return 1
  [[ "${NAV2_LIFECYCLE_READY_OWNER_PID:-}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${NAV2_LIFECYCLE_READY_OWNER_PID}" 2>/dev/null || return 1
  now_sec="$(date +%s)"
  age_sec=$((now_sec - ${NAV2_LIFECYCLE_READY_STAMP_SEC:-0}))
  (( age_sec >= 0 && age_sec <= max_age_sec ))
)

nav2_lifecycle_manager_reported_active() {
  nav2_lifecycle_ready_status_matches
}

nav2_point_navigation_core_reported_active() {
  nav2_lifecycle_ready_status_matches
}

nav2_critical_processes_running() {
  local patterns=(
    "__node:=controller_server"
    "__node:=planner_server"
    "__node:=behavior_server"
    "__node:=smoother_server"
    "__node:=bt_navigator"
    "__node:=velocity_smoother"
    "__node:=collision_monitor"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pgrep -f "${pattern}" >/dev/null 2>&1 || return 1
  done
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
  local deadline=$((SECONDS + timeout_sec))
  local node_name
  local core_active
  while (( SECONDS < deadline )); do
    core_active=1
    if nav2_critical_processes_running && nav2_point_navigation_core_reported_active; then
      echo "[runtime-overlay] Nav2 repo lifecycle sequence reported point-navigation core nodes active and critical processes are running" >&2
      return 0
    fi
    if nav2_critical_processes_running; then
      while IFS= read -r node_name; do
        nav2_lifecycle_node_active "${node_name}" "${NJRH_NAV2_CORE_LIFECYCLE_ACTIVE_CHECK_TIMEOUT_SEC:-1}" >/dev/null 2>&1 || {
          core_active=0
          break
        }
      done < <(critical_nav2_lifecycle_nodes)
      if [[ "${core_active}" -eq 1 ]]; then
        echo "[runtime-overlay] Nav2 point-navigation core lifecycle nodes are active and critical processes are running" >&2
        return 0
      fi
    fi
    if nav2_critical_processes_running && nav2_lifecycle_manager_reported_active; then
      echo "[runtime-overlay] Nav2 lifecycle manager reported managed nodes active and critical processes are running" >&2
      return 0
    fi
    sleep 0.5
  done
  echo "[runtime-overlay] Nav2 point-navigation core lifecycle nodes did not become active before timeout" >&2
  return 1
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

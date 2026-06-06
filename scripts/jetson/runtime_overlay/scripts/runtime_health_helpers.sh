#!/usr/bin/env bash

runtime_health_file() {
  printf '%s\n' "${NJRH_RUNTIME_HEALTH_FILE:-/tmp/njrh_runtime_health.json}"
}

runtime_health_max_age_sec() {
  printf '%s\n' "${NJRH_RUNTIME_HEALTH_MAX_AGE_SEC:-2.0}"
}

runtime_health_available() {
  local health_file
  health_file="$(runtime_health_file)"
  [[ -s "${health_file}" ]] || return 1
  python3 - "${health_file}" "$(runtime_health_max_age_sec)" <<'PY'
import json
import sys
import time

path = sys.argv[1]
max_age = float(sys.argv[2])
try:
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
except Exception:
    raise SystemExit(1)

updated_at = float(data.get("updated_at") or 0.0)
raise SystemExit(0 if time.time() - updated_at <= max_age else 1)
PY
}

runtime_health_check() {
  local check_name="$1"
  local health_file
  health_file="$(runtime_health_file)"
  [[ -s "${health_file}" ]] || return 1
  python3 - "${health_file}" "$(runtime_health_max_age_sec)" "${check_name}" <<'PY'
import json
import sys
import time

path = sys.argv[1]
max_age = float(sys.argv[2])
check = sys.argv[3]
try:
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
except Exception:
    raise SystemExit(1)

if time.time() - float(data.get("updated_at") or 0.0) > max_age:
    raise SystemExit(1)

summary = data.get("summary") or {}
topics = data.get("topics") or {}
tf_edges = data.get("tf") or {}
services = data.get("services") or {}

checks = {
    "local_state_endpoint": bool(summary.get("local_state_endpoint_ready")),
    "local_state_fastlio_endpoint": bool(summary.get("local_state_fastlio_endpoint_ready")),
    "local_state_ready": bool(summary.get("local_state_ready")),
    "local_odom_fresh": bool(summary.get("local_odom_fresh")),
    "odom_base_tf_fresh": bool(summary.get("odom_base_tf_fresh")),
    "map_odom_tf_ready": bool(summary.get("map_odom_tf_ready")),
    "localization_bridge_endpoint": bool(summary.get("localization_bridge_endpoint_ready")),
    "safety_status_fresh": bool(summary.get("safety_status_fresh")),
    "perception_obstacle_fresh": bool(summary.get("perception_obstacle_fresh")),
    "local_costmap_fresh": bool(summary.get("local_costmap_fresh")),
    "global_costmap_fresh": bool(summary.get("global_costmap_fresh")),
    "map_fresh": bool(summary.get("map_fresh")),
    "global_localization_trigger_service": bool(summary.get("global_localization_trigger_service"))
        or bool(services.get("/global_localization/trigger")),
    "isaac_grid_search_trigger_service": bool(summary.get("isaac_grid_search_trigger_service"))
        or bool(services.get("/trigger_grid_search_localization")),
    "floor_switch_service": bool(summary.get("floor_switch_service"))
        or bool(services.get("/floor_manager/switch_floor")),
    "obstacle_topic_seen": bool((topics.get("/perception/obstacle_points") or {}).get("last_received_at")),
    "odom_base_tf_seen": "odom->base_link" in tf_edges,
    "map_odom_tf_seen": "map->odom" in tf_edges,
}

raise SystemExit(0 if checks.get(check, False) else 1)
PY
}

runtime_health_topic_message_ready() {
  local topic="$1"
  local health_file
  health_file="$(runtime_health_file)"
  [[ -s "${health_file}" ]] || return 1
  python3 - "${health_file}" "$(runtime_health_max_age_sec)" "${topic}" <<'PY'
import json
import sys
import time

path = sys.argv[1]
max_age = float(sys.argv[2])
topic = sys.argv[3]
try:
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
except Exception:
    raise SystemExit(1)
if time.time() - float(data.get("updated_at") or 0.0) > max_age:
    raise SystemExit(1)
item = (data.get("topics") or {}).get(topic) or {}
if item.get("last_received_at") is None:
    raise SystemExit(1)
ok = int(item.get("publishers") or 0) > 0
raise SystemExit(0 if ok else 1)
PY
}

runtime_health_fresh_tf_ready() {
  local parent_frame="$1"
  local child_frame="$2"
  local max_age_sec="$3"
  local health_file
  health_file="$(runtime_health_file)"
  [[ -s "${health_file}" ]] || return 1
  python3 - "${health_file}" "$(runtime_health_max_age_sec)" "${parent_frame}" "${child_frame}" "${max_age_sec}" <<'PY'
import json
import sys
import time

path = sys.argv[1]
max_snapshot_age = float(sys.argv[2])
parent = sys.argv[3].strip().lstrip("/")
child = sys.argv[4].strip().lstrip("/")
max_age = float(sys.argv[5])
try:
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
except Exception:
    raise SystemExit(1)
if time.time() - float(data.get("updated_at") or 0.0) > max_snapshot_age:
    raise SystemExit(1)
edge = (data.get("tf") or {}).get(f"{parent}->{child}") or {}
age = edge.get("last_age_sec")
try:
    age = float(age)
except (TypeError, ValueError):
    raise SystemExit(1)
raise SystemExit(0 if -0.25 <= age <= max_age else 1)
PY
}

runtime_health_tf_seen() {
  local parent_frame="$1"
  local child_frame="$2"
  local max_age_sec="${NJRH_RUNTIME_HEALTH_TF_SEEN_MAX_AGE_SEC:-5.0}"
  local health_file
  health_file="$(runtime_health_file)"
  [[ -s "${health_file}" ]] || return 1
  python3 - "${health_file}" "$(runtime_health_max_age_sec)" "${parent_frame}" "${child_frame}" "${max_age_sec}" <<'PY'
import json
import sys
import time

path = sys.argv[1]
max_snapshot_age = float(sys.argv[2])
parent = sys.argv[3].strip().lstrip("/")
child = sys.argv[4].strip().lstrip("/")
max_age = float(sys.argv[5])
try:
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
except Exception:
    raise SystemExit(1)
if time.time() - float(data.get("updated_at") or 0.0) > max_snapshot_age:
    raise SystemExit(1)
edge = (data.get("tf") or {}).get(f"{parent}->{child}") or {}
age = edge.get("last_age_sec")
try:
    age = float(age)
except (TypeError, ValueError):
    raise SystemExit(1)
raise SystemExit(0 if -0.25 <= age <= max_age else 1)
PY
}

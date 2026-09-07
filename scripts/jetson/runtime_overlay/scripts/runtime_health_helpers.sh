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
    "local_state_topic_ready": bool(summary.get("local_state_topic_ready")),
    "local_state_ready": bool(summary.get("local_state_ready")),
    "local_odom_fresh": bool(summary.get("local_odom_fresh")),
    "odom_base_tf_fresh": bool(summary.get("odom_base_tf_fresh")),
    "map_odom_tf_ready": bool(summary.get("map_odom_tf_ready")),
    "localization_bridge_endpoint": bool(summary.get("localization_bridge_endpoint_ready")),
    "safety_status_fresh": bool(summary.get("safety_status_fresh")),
    "local_scan_fresh": bool(summary.get("local_scan_fresh")),
    "local_costmap_fresh": bool(summary.get("local_costmap_fresh")),
    "global_costmap_fresh": bool(summary.get("global_costmap_fresh")),
    "map_fresh": bool(summary.get("map_fresh")),
    "docking_observation_fresh": bool(summary.get("docking_observation_fresh")),
    "docking_sensor_healthy": bool(summary.get("docking_sensor_healthy")),
    "global_localization_trigger_service": bool(summary.get("global_localization_trigger_service"))
        or bool(services.get("/global_localization/trigger")),
    "isaac_grid_search_trigger_service": bool(summary.get("isaac_grid_search_trigger_service"))
        or bool(services.get("/trigger_grid_search_localization")),
    "floor_switch_service": bool(summary.get("floor_switch_service"))
        or bool(services.get("/floor_manager/switch_floor")),
    "scan_topic_seen": bool((topics.get("/scan") or {}).get("last_received_at")),
    "odom_base_tf_seen": "odom->base_link" in tf_edges,
    "map_odom_tf_seen": "map->odom" in tf_edges,
}

raise SystemExit(0 if checks.get(check, False) else 1)
PY
}

# Classify local-state health without conflating an unavailable observer with a
# failed robot_local_state producer. The caller may only authorize complete
# runtime recovery for the 50-59 range, which is derived from a fresh snapshot.
#
# Exit codes:
#   0      ready
#   40-49  observer unavailable/invalid/stale (never proves local-state loss)
#   50-59  fresh-snapshot local-state fault candidate
runtime_health_local_state_diagnostic() {
  local health_file
  health_file="$(runtime_health_file)"
  python3 - \
    "${health_file}" \
    "$(runtime_health_max_age_sec)" \
    "${NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC:-0.75}" <<'PY'
import json
import math
import sys
import time

path = sys.argv[1]
snapshot_max_age = float(sys.argv[2])
odom_max_age = float(sys.argv[3])
now = time.time()


def number(value, default=None):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def integer(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def field(value):
    if value is None:
        return "none"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value).replace(" ", "_")


def emit(status, code, **values):
    ordered = {"status": status, **values}
    print(" ".join(f"{key}={field(value)}" for key, value in ordered.items()))
    raise SystemExit(code)


try:
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
except FileNotFoundError:
    emit("observer_missing", 40, snapshot_age_sec=None)
except Exception as exc:
    emit("observer_invalid", 41, snapshot_age_sec=None, error=type(exc).__name__)

updated_at = number(data.get("updated_at"))
if updated_at is None:
    emit("observer_invalid", 41, snapshot_age_sec=None, error="updated_at_invalid")

snapshot_age = now - updated_at
if snapshot_age < -0.25:
    emit("observer_clock_invalid", 43, snapshot_age_sec=snapshot_age)
if snapshot_age > snapshot_max_age:
    emit("observer_stale", 42, snapshot_age_sec=snapshot_age)

summary = data.get("summary") or {}
topic = (data.get("topics") or {}).get("/local_state/odometry") or {}
publishers = integer(topic.get("publishers"))
message_count = integer(topic.get("message_count"))
last_received_at = number(topic.get("last_received_at"))
last_stamp_sec = number(topic.get("last_stamp_sec"))
odom_age = number(topic.get("last_age_sec"))
if odom_age is None and last_stamp_sec is not None:
    odom_age = now - last_stamp_sec
received_age = None if last_received_at is None else now - last_received_at
endpoint_ready = bool(summary.get("local_state_endpoint_ready"))
delivery_max_age = max(snapshot_max_age, odom_max_age * 3.0)
delivery_fresh = (
    received_age is not None
    and -0.25 <= received_age <= delivery_max_age
    and message_count > 0
)
stamp_fresh = odom_age is not None and -0.25 <= odom_age <= odom_max_age

details = {
    "snapshot_age_sec": snapshot_age,
    "endpoint_ready": endpoint_ready,
    "publishers": publishers,
    "message_count": message_count,
    "odom_age_sec": odom_age,
    "received_age_sec": received_age,
}

# A resident callback seeing current odometry is stronger evidence than a
# periodic ROS-graph sample. Discovery can temporarily report zero publishers
# or omit the node while the already-matched BEST_EFFORT stream is still
# arriving. Classify that contradiction as observer-side graph lag; it must not
# consume the real local-state failure budget.
if delivery_fresh and stamp_fresh and (
    not endpoint_ready
    or publishers <= 0
    or not bool(summary.get("local_state_topic_ready"))
    or not bool(summary.get("local_odom_fresh"))
):
    emit("observer_graph_inconsistent", 44, **details)

if not endpoint_ready:
    emit("endpoint_missing", 50, **details)
if publishers <= 0:
    emit("publisher_missing", 51, **details)
if last_received_at is None or message_count <= 0:
    emit("odom_unseen", 52, **details)

if received_age is None or received_age < -0.25 or received_age > delivery_max_age:
    emit("odom_delivery_stale", 54, **details)
if (
    odom_age is None
    or odom_age < -0.25
    or odom_age > odom_max_age
    or not bool(summary.get("local_odom_fresh"))
):
    emit("odom_stamp_stale", 53, **details)
if not bool(summary.get("local_state_topic_ready")):
    emit("local_state_summary_inconsistent", 55, **details)

emit("ready", 0, **details)
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

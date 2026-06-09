#!/usr/bin/env bash
set -euo pipefail

API_URL="${API_URL:-http://127.0.0.1:8080}"
CURL_TIMEOUT_SEC="${CURL_TIMEOUT_SEC:-5}"
BUILDING_ID=""
FLOOR_ID=""
MAP_ID=""
POSE_ID=""
DIRECT_X=""
DIRECT_Y=""
DIRECT_YAW=""
EXECUTE_GOAL=false

usage() {
  cat <<'USAGE'
Usage: verify_pre_navigation_undock_gate.sh [options]

Read-only by default. It checks /status, /docking/state, and
/navigation/pre_goal_check without moving the robot.

Options:
  --api-url URL          API base URL, default http://127.0.0.1:8080
  --building-id ID       Building id for pose_id precheck
  --floor-id ID          Floor id for pose_id precheck
  --map-id ID            Optional map id for pose_id precheck
  --pose-id ID           Pose id to resolve from poses.yaml
  --x VALUE              Direct map-frame goal x for precheck/execute
  --y VALUE              Direct map-frame goal y for precheck/execute
  --yaw VALUE            Direct map-frame goal yaw for precheck/execute
  --execute-goal         POST /api/v1/navigation/goal after read-only checks
  -h, --help             Show this help

Set ROBOT_API_TOKEN to send X-Robot-Token.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --api-url)
      API_URL="$2"
      shift 2
      ;;
    --building-id)
      BUILDING_ID="$2"
      shift 2
      ;;
    --floor-id)
      FLOOR_ID="$2"
      shift 2
      ;;
    --map-id)
      MAP_ID="$2"
      shift 2
      ;;
    --pose-id)
      POSE_ID="$2"
      shift 2
      ;;
    --x)
      DIRECT_X="$2"
      shift 2
      ;;
    --y)
      DIRECT_Y="$2"
      shift 2
      ;;
    --yaw|--theta)
      DIRECT_YAW="$2"
      shift 2
      ;;
    --execute-goal)
      EXECUTE_GOAL=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

API_URL="${API_URL%/}"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

curl_json() {
  local method="$1"
  local url="$2"
  local body="${3:-}"
  local -a args=(-fsS --max-time "${CURL_TIMEOUT_SEC}" -H "Content-Type: application/json")
  if [[ -n "${ROBOT_API_TOKEN:-}" ]]; then
    args+=(-H "X-Robot-Token: ${ROBOT_API_TOKEN}")
  fi
  if [[ "${method}" == "GET" ]]; then
    curl "${args[@]}" "${url}"
  else
    curl "${args[@]}" -X "${method}" --data "${body}" "${url}"
  fi
}

build_precheck_url() {
  python3 - "$API_URL" "$BUILDING_ID" "$FLOOR_ID" "$MAP_ID" "$POSE_ID" "$DIRECT_X" "$DIRECT_Y" "$DIRECT_YAW" <<'PY'
import sys
from urllib.parse import urlencode

base, building_id, floor_id, map_id, pose_id, x, y, yaw = sys.argv[1:]
params = {}
if building_id:
    params["building_id"] = building_id
if floor_id:
    params["floor_id"] = floor_id
if map_id:
    params["map_id"] = map_id
if pose_id:
    params["pose_id"] = pose_id
if x:
    params["x"] = x
if y:
    params["y"] = y
if yaw:
    params["yaw"] = yaw
query = urlencode(params)
print(f"{base}/api/v1/navigation/pre_goal_check" + (f"?{query}" if query else ""))
PY
}

goal_body() {
  python3 - "$BUILDING_ID" "$FLOOR_ID" "$MAP_ID" "$POSE_ID" "$DIRECT_X" "$DIRECT_Y" "$DIRECT_YAW" <<'PY'
import json
import sys

building_id, floor_id, map_id, pose_id, x, y, yaw = sys.argv[1:]
body = {}
if pose_id:
    body["pose_id"] = pose_id
    if building_id:
        body["building_id"] = building_id
    if floor_id:
        body["floor_id"] = floor_id
    if map_id:
        body["map_id"] = map_id
elif x and y and yaw:
    body.update({"x": float(x), "y": float(y), "yaw": float(yaw), "frame_id": "map"})
else:
    raise SystemExit("execute-goal requires --pose-id or --x --y --yaw")
print(json.dumps(body, separators=(",", ":")))
PY
}

require_fields() {
  local label="$1"
  local file="$2"
  shift 2
  python3 - "$label" "$file" "$@" <<'PY'
import json
import sys

label, path, *fields = sys.argv[1:]
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)
missing = []
for field in fields:
    obj = data
    ok = True
    for part in field.split("."):
        if isinstance(obj, dict) and part in obj:
            obj = obj[part]
        else:
            ok = False
            break
    if not ok:
        missing.append(field)
if missing:
    raise SystemExit(f"{label}: missing fields: {', '.join(missing)}")
PY
}

print_summary() {
  python3 - "$1" "$2" "$3" <<'PY'
import json
import sys

status_path, docking_path, precheck_path = sys.argv[1:]
with open(status_path, "r", encoding="utf-8") as f:
    status = json.load(f)
with open(docking_path, "r", encoding="utf-8") as f:
    docking = json.load(f)
with open(precheck_path, "r", encoding="utf-8") as f:
    precheck = json.load(f)

dock_check = precheck.get("pre_navigation_dock_check", {})
bms = dock_check.get("bms", {})
docking_check = dock_check.get("docking", {})
pose = precheck.get("pose_resolution", {})

print("pre-navigation undock gate:")
print(f"  status.mode={status.get('mode')} navigation.active={status.get('navigation', {}).get('active')}")
print(f"  docking.state={docking.get('state')} last_status={docking.get('last_status')}")
print(f"  bms.contact={bms.get('contact')} reason={bms.get('reason')} age_sec={bms.get('age_sec')}")
print(f"  bms.status={bms.get('power_supply_status')} current={bms.get('current')} voltage={bms.get('voltage')}")
print(f"  final_is_docked_or_charging={dock_check.get('final_is_docked_or_charging')}")
print(f"  would_auto_undock={precheck.get('would_auto_undock')} can_auto_undock={precheck.get('can_auto_undock')}")
print(f"  auto_undock_reason={dock_check.get('auto_undock_reason')}")
print(f"  docking_status_docked={docking_check.get('status_indicates_docked')} charging={docking_check.get('status_indicates_charging')}")
print(f"  pose_resolution.status={pose.get('status')} ok={pose.get('ok')} detail={pose.get('detail')}")
PY
}

STATUS_JSON="${TMP_DIR}/status.json"
DOCKING_JSON="${TMP_DIR}/docking_state.json"
PRECHECK_JSON="${TMP_DIR}/pre_goal_check.json"

curl_json GET "${API_URL}/api/v1/status" > "${STATUS_JSON}"
curl_json GET "${API_URL}/api/v1/docking/state" > "${DOCKING_JSON}"
curl_json GET "$(build_precheck_url)" > "${PRECHECK_JSON}"

require_fields "status" "${STATUS_JSON}" \
  "bms.charging_contact" \
  "bms.charging_contact_reason" \
  "bms.contact_snapshot.reason" \
  "docking.pre_navigation_dock_check.final_auto_undock_required"
require_fields "docking_state" "${DOCKING_JSON}" \
  "charging_contact" \
  "charging_contact_reason" \
  "can_auto_undock" \
  "pre_navigation_dock_check.auto_undock_reason"
require_fields "pre_goal_check" "${PRECHECK_JSON}" \
  "read_only" \
  "would_auto_undock" \
  "pre_navigation_dock_check.api_bms_charging_contact" \
  "pre_navigation_dock_check.bms.power_supply_status" \
  "pre_navigation_dock_check.final_auto_undock_required" \
  "pose_resolution.status"

print_summary "${STATUS_JSON}" "${DOCKING_JSON}" "${PRECHECK_JSON}"

if [[ "${EXECUTE_GOAL}" == "true" ]]; then
  GOAL_JSON="${TMP_DIR}/navigation_goal.json"
  BODY="$(goal_body)"
  echo "posting /api/v1/navigation/goal with body: ${BODY}"
  curl_json POST "${API_URL}/api/v1/navigation/goal" "${BODY}" > "${GOAL_JSON}"
  require_fields "navigation_goal" "${GOAL_JSON}" \
    "pre_navigation_undock" \
    "pre_navigation_undock_detail" \
    "pre_navigation_dock_check.final_auto_undock_required"
  python3 - "$GOAL_JSON" <<'PY'
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = json.load(f)
print("navigation_goal response:")
print(f"  ok={data.get('ok')} accepted={data.get('accepted')} error={data.get('error')}")
print(f"  pre_navigation_undock={data.get('pre_navigation_undock')}")
print(f"  pre_navigation_undock_detail={data.get('pre_navigation_undock_detail')}")
PY
else
  echo "read-only check complete. Use --execute-goal only when movement is intended."
fi

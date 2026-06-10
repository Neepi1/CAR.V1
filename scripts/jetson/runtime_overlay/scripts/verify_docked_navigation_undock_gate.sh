#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  source "${PROJECT_ROOT}/install/setup.bash"
fi
set -u

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
TEST_NORMAL_CMD_BLOCK=false
TEST_DOCKING_CMD_ALLOWED=false
CONFIRM_LATCH=false
CLEAR_LATCH=false
PRINT_LATCH=false

usage() {
  cat <<'USAGE'
Usage: verify_docked_navigation_undock_gate.sh [options]

Read-only by default. It checks API dock/contact admission state, safety state,
and velocity-chain topology. It moves the robot only with --execute-goal.

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
  --confirm-latch        Maintenance action: confirm physical docked state in persistent latch
  --clear-latch          Maintenance action: clear persistent docked latch
  --print-latch          Print latch fields from pre_goal_check/status snapshots
  --test-normal-cmd-block
                         DANGEROUS: publish one low angular normal cmd and
                         verify robot_safety reports DOCKED_CONTACT_BLOCK.
  --test-docking-cmd-allowed
                         DANGEROUS: publish one low docking cmd and verify it
                         is not blocked by DOCKED_CONTACT_BLOCK.
  -h, --help             Show this help

Set ROBOT_API_TOKEN to send X-Robot-Token.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --api-url) API_URL="$2"; shift 2 ;;
    --building-id) BUILDING_ID="$2"; shift 2 ;;
    --floor-id) FLOOR_ID="$2"; shift 2 ;;
    --map-id) MAP_ID="$2"; shift 2 ;;
    --pose-id) POSE_ID="$2"; shift 2 ;;
    --x) DIRECT_X="$2"; shift 2 ;;
    --y) DIRECT_Y="$2"; shift 2 ;;
    --yaw|--theta) DIRECT_YAW="$2"; shift 2 ;;
    --execute-goal) EXECUTE_GOAL=true; shift ;;
    --confirm-latch) CONFIRM_LATCH=true; shift ;;
    --clear-latch) CLEAR_LATCH=true; shift ;;
    --print-latch) PRINT_LATCH=true; shift ;;
    --test-normal-cmd-block) TEST_NORMAL_CMD_BLOCK=true; shift ;;
    --test-docking-cmd-allowed) TEST_DOCKING_CMD_ALLOWED=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

API_URL="${API_URL%/}"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

pass_count=0
warn_count=0
fail_count=0

pass() { echo "PASS: $*"; pass_count=$((pass_count + 1)); }
warn() { echo "WARN: $*"; warn_count=$((warn_count + 1)); }
fail() { echo "FAIL: $*"; fail_count=$((fail_count + 1)); }

dock_evidence_active() {
  [[ "${bms_contact}" == "true" || "${latch_docked}" == "true" || "${latched_docked:-}" == "true" ||
    "${docking_state}" == "docked" || "${docking_state}" == "charging" ||
    "${docking_status}" == docked* || "${docking_status}" == charging* ]]
}

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

latch_body() {
  python3 - "$BUILDING_ID" "$FLOOR_ID" "$MAP_ID" "$POSE_ID" <<'PY'
import json
import sys

building_id, floor_id, map_id, pose_id = sys.argv[1:]
body = {"reason": "field_maintenance_confirmed_physical_dock", "note": "set by verify_docked_navigation_undock_gate.sh"}
if building_id:
    body["building_id"] = building_id
if floor_id:
    body["floor_id"] = floor_id
if map_id:
    body["map_id"] = map_id
if pose_id:
    body["dock_id"] = pose_id if pose_id.startswith("dock") else ""
print(json.dumps(body, separators=(",", ":")))
PY
}

json_get() {
  python3 - "$1" "$2" <<'PY'
import json
import sys

path, dotted = sys.argv[1:]
with open(path, "r", encoding="utf-8") as f:
    obj = json.load(f)
for part in dotted.split("."):
    if isinstance(obj, dict) and part in obj:
        obj = obj[part]
    else:
        print("")
        raise SystemExit(0)
if isinstance(obj, bool):
    print("true" if obj else "false")
else:
    print(obj)
PY
}

twist_file_is_zero() {
  python3 - "$1" <<'PY'
import math
import re
import sys

with open(sys.argv[1], "r", encoding="utf-8", errors="ignore") as f:
    text = f.read()
values = [float(v) for v in re.findall(r":\s*(-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", text)]
if len(values) < 6:
    raise SystemExit(1)
print("true" if all(math.isfinite(v) and abs(v) <= 1e-6 for v in values[:6]) else "false")
PY
}

twist_file_is_nonzero() {
  python3 - "$1" <<'PY'
import math
import re
import sys

with open(sys.argv[1], "r", encoding="utf-8", errors="ignore") as f:
    text = f.read()
values = [float(v) for v in re.findall(r":\s*(-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", text)]
if len(values) < 6:
    raise SystemExit(1)
print("true" if any(math.isfinite(v) and abs(v) > 1e-6 for v in values[:6]) else "false")
PY
}

STATUS_JSON="${TMP_DIR}/status.json"
DOCKING_JSON="${TMP_DIR}/docking_state.json"
PRECHECK_JSON="${TMP_DIR}/pre_goal_check.json"
NAV_STATE_JSON="${TMP_DIR}/navigation_state.json"

curl_json GET "${API_URL}/api/v1/status" > "${STATUS_JSON}"
curl_json GET "${API_URL}/api/v1/docking/state" > "${DOCKING_JSON}"
curl_json GET "${API_URL}/api/v1/navigation/state" > "${NAV_STATE_JSON}"
curl_json GET "$(build_precheck_url)" > "${PRECHECK_JSON}"

if [[ "${CONFIRM_LATCH}" == "true" && "${CLEAR_LATCH}" == "true" ]]; then
  fail "--confirm-latch and --clear-latch are mutually exclusive"
fi

if [[ "${CONFIRM_LATCH}" == "true" ]]; then
  CONFIRM_JSON="${TMP_DIR}/confirm_latch.json"
  BODY="$(latch_body)"
  echo "confirming persistent docked latch via /api/v1/docking/confirm_docked"
  if curl_json POST "${API_URL}/api/v1/docking/confirm_docked" "${BODY}" > "${CONFIRM_JSON}"; then
    pass "confirm_docked endpoint accepted latch update"
    curl_json GET "$(build_precheck_url)" > "${PRECHECK_JSON}"
    curl_json GET "${API_URL}/api/v1/navigation/state" > "${NAV_STATE_JSON}"
    curl_json GET "${API_URL}/api/v1/status" > "${STATUS_JSON}"
  else
    fail "confirm_docked endpoint failed"
  fi
fi

if [[ "${CLEAR_LATCH}" == "true" ]]; then
  CLEAR_JSON="${TMP_DIR}/clear_latch.json"
  echo "clearing persistent docked latch via /api/v1/docking/clear_docked_latch"
  if curl_json POST "${API_URL}/api/v1/docking/clear_docked_latch" \
    '{"reason":"field_maintenance_clear","note":"cleared by verify_docked_navigation_undock_gate.sh"}' > "${CLEAR_JSON}"; then
    pass "clear_docked_latch endpoint accepted latch update"
    curl_json GET "$(build_precheck_url)" > "${PRECHECK_JSON}"
    curl_json GET "${API_URL}/api/v1/navigation/state" > "${NAV_STATE_JSON}"
    curl_json GET "${API_URL}/api/v1/status" > "${STATUS_JSON}"
  else
    fail "clear_docked_latch endpoint failed"
  fi
fi

bms_contact="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.api_bms_charging_contact")"
docking_state="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.docking.state")"
docking_status="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.docking.last_status")"
latch_docked="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.dock_contact_snapshot.docked")"
latched_docked="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.dock_contact_snapshot.latched_docked")"
docked_state_class="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.docked_state_class")"
latch_source="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.dock_contact_snapshot.source")"
would_auto="$(json_get "${PRECHECK_JSON}" "would_auto_undock")"
auto_required="$(json_get "${PRECHECK_JSON}" "auto_undock_required")"
can_auto="$(json_get "${PRECHECK_JSON}" "can_auto_undock")"
reason="$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.auto_undock_reason")"
safety_status="$(timeout 2 ros2 topic echo /safety/status --once --field data 2>/dev/null | sed '/^---$/d' | head -n 1 || true)"
motion_allowed="$(timeout 2 ros2 topic echo /safety/motion_allowed --once --field data 2>/dev/null | sed '/^---$/d' | head -n 1 || true)"

echo "docked navigation undock gate:"
echo "  bms_contact=${bms_contact}"
echo "  docking_state=${docking_state}"
echo "  docking_status=${docking_status}"
echo "  latch_docked=${latch_docked}"
echo "  latched_docked=${latched_docked}"
echo "  latch_source=${latch_source}"
echo "  docked_state_class=${docked_state_class}"
echo "  would_auto_undock=${would_auto}"
echo "  auto_undock_required=${auto_required}"
echo "  can_auto_undock=${can_auto}"
echo "  auto_undock_reason=${reason}"
echo "  safety_status=${safety_status:-unavailable}"
echo "  motion_allowed=${motion_allowed:-unavailable}"

if [[ "${PRINT_LATCH}" == "true" ]]; then
  echo "latch snapshot:"
  python3 - "${PRECHECK_JSON}" <<'PY'
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = json.load(f)
print(json.dumps(data.get("pre_navigation_dock_check", {}).get("dock_contact_snapshot", {}), indent=2, ensure_ascii=False))
PY
fi

if [[ "${bms_contact}" == "true" || "${latch_docked}" == "true" || "${latched_docked}" == "true" ||
  "${docking_state}" == "docked" || "${docking_state}" == "charging" ||
  "${docking_status}" == docked* || "${docking_status}" == charging* ]]; then
  [[ "${would_auto}" == "true" ]] && pass "dock/contact evidence forces pre-navigation auto-undock" \
    || fail "dock/contact evidence exists but would_auto_undock=false"
else
  warn "no dock/contact evidence in current API snapshot; on-dock PASS requires BMS/status/latch evidence"
fi

[[ "$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.dock_contact_snapshot.valid")" != "" ]] \
  && pass "pre_goal_check exposes dock_contact_snapshot" \
  || fail "pre_goal_check missing dock_contact_snapshot"

[[ "$(json_get "${PRECHECK_JSON}" "pre_navigation_dock_check.docked_state_class")" != "" ]] \
  && pass "pre_goal_check exposes docked_state_class" \
  || fail "pre_goal_check missing docked_state_class"

[[ "$(json_get "${NAV_STATE_JSON}" "pre_navigation_dock_check.final_auto_undock_required")" != "" ]] \
  && pass "navigation/state exposes pre_navigation_dock_check" \
  || fail "navigation/state missing pre_navigation_dock_check"

if [[ "${safety_status}" == "DOCKED_CONTACT_BLOCK" ]]; then
  pass "robot_safety reports DOCKED_CONTACT_BLOCK"
elif [[ "${would_auto}" == "true" ]]; then
  warn "pre_goal_check requires undock, but safety status is ${safety_status:-unavailable}; publish a normal cmd only in a controlled bench test to verify zeroing"
else
  warn "safety dock block not active because no dock/contact evidence is active"
fi

ros2 topic info -v /cmd_vel_collision_checked >/dev/null 2>&1 && pass "/cmd_vel_collision_checked exists" || warn "/cmd_vel_collision_checked not visible"
ros2 topic info -v /cmd_vel_safe >/dev/null 2>&1 && pass "/cmd_vel_safe exists" || fail "/cmd_vel_safe not visible"
ros2 topic info -v /cmd_vel_docking >/dev/null 2>&1 && pass "/cmd_vel_docking exists" || warn "/cmd_vel_docking not visible until docking manager starts"
ros2 topic info -v /cmd_vel >/dev/null 2>&1 && pass "/cmd_vel exists" || warn "/cmd_vel not visible"

[[ "$(json_get "${STATUS_JSON}" "safety.status")" != "" ]] \
  && pass "/api/v1/status exposes safety.status" \
  || warn "/api/v1/status does not expose safety.status"
[[ "$(json_get "${NAV_STATE_JSON}" "safety.status")" != "" ]] \
  && pass "/api/v1/navigation/state exposes safety.status" \
  || warn "/api/v1/navigation/state does not expose safety.status"

if [[ "${TEST_NORMAL_CMD_BLOCK}" == "true" ]]; then
  if ! dock_evidence_active; then
    warn "--test-normal-cmd-block skipped: no dock/contact/latch evidence is active"
  else
    echo "DANGEROUS optional test: publishing one low normal angular command to /cmd_vel_collision_checked"
    NORMAL_SAFE_ECHO="${TMP_DIR}/normal_cmd_vel_safe.txt"
    timeout 4 ros2 topic echo /cmd_vel_safe --once > "${NORMAL_SAFE_ECHO}" 2>/dev/null &
    normal_echo_pid=$!
    sleep 0.3
    timeout 2 ros2 topic pub --once /cmd_vel_collision_checked geometry_msgs/msg/Twist \
      "{angular: {z: 0.12}}" >/dev/null 2>&1 || true
    wait "${normal_echo_pid}" || true
    timeout 2 ros2 topic pub --once /cmd_vel_collision_checked geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
    safety_status_after_normal="$(
      timeout 2 ros2 topic echo /safety/status --once --field data 2>/dev/null | sed '/^---$/d' | head -n 1 || true
    )"
    if [[ "${safety_status_after_normal}" == "DOCKED_CONTACT_BLOCK" ]] &&
      [[ -s "${NORMAL_SAFE_ECHO}" ]] &&
      [[ "$(twist_file_is_zero "${NORMAL_SAFE_ECHO}" 2>/dev/null || echo false)" == "true" ]]; then
      pass "normal cmd is zeroed while dock/contact evidence is active"
    else
      fail "normal cmd block test did not observe zero /cmd_vel_safe with DOCKED_CONTACT_BLOCK"
    fi
  fi
fi

if [[ "${TEST_DOCKING_CMD_ALLOWED}" == "true" ]]; then
  if ! dock_evidence_active; then
    warn "--test-docking-cmd-allowed skipped: no dock/contact/latch evidence is active"
  else
    echo "DANGEROUS optional test: publishing one low docking command to /cmd_vel_docking"
    DOCKING_SAFE_ECHO="${TMP_DIR}/docking_cmd_vel_safe.txt"
    timeout 4 ros2 topic echo /cmd_vel_safe --once > "${DOCKING_SAFE_ECHO}" 2>/dev/null &
    docking_echo_pid=$!
    sleep 0.3
    timeout 2 ros2 topic pub --once /cmd_vel_docking geometry_msgs/msg/Twist \
      "{linear: {x: -0.02}}" >/dev/null 2>&1 || true
    wait "${docking_echo_pid}" || true
    timeout 2 ros2 topic pub --once /cmd_vel_docking geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
    safety_status_after_docking="$(
      timeout 2 ros2 topic echo /safety/status --once --field data 2>/dev/null | sed '/^---$/d' | head -n 1 || true
    )"
    if [[ "${safety_status_after_docking}" == "DOCKED_CONTACT_BLOCK" ]]; then
      fail "docking cmd was blocked by DOCKED_CONTACT_BLOCK"
    elif [[ -s "${DOCKING_SAFE_ECHO}" ]] &&
      [[ "$(twist_file_is_nonzero "${DOCKING_SAFE_ECHO}" 2>/dev/null || echo false)" == "true" ]]; then
      pass "docking cmd remains allowed while dock/contact evidence is active"
    else
      warn "docking cmd allowed test did not observe nonzero /cmd_vel_safe; check estop/localization/watchdog status"
    fi
  fi
fi

if [[ "${EXECUTE_GOAL}" == "true" ]]; then
  GOAL_JSON="${TMP_DIR}/navigation_goal.json"
  BODY="$(goal_body)"
  echo "posting /api/v1/navigation/goal with body: ${BODY}"
  if curl_json POST "${API_URL}/api/v1/navigation/goal" "${BODY}" > "${GOAL_JSON}"; then
    goal_undock="$(json_get "${GOAL_JSON}" "pre_navigation_undock")"
    goal_required="$(json_get "${GOAL_JSON}" "pre_navigation_dock_check.final_auto_undock_required")"
    accepted="$(json_get "${GOAL_JSON}" "accepted")"
    echo "  accepted=${accepted}"
    echo "  pre_navigation_undock=${goal_undock}"
    echo "  final_auto_undock_required=${goal_required}"
    if [[ "${goal_required}" == "true" && "${goal_undock}" != "true" ]]; then
      fail "goal required auto-undock but response did not perform pre_navigation_undock"
    else
      pass "goal response preserves pre-navigation undock contract"
    fi
  else
    fail "POST /api/v1/navigation/goal failed"
  fi
else
  echo "read-only check complete. Use --execute-goal only when movement is intended."
fi

echo "summary: PASS=${pass_count} WARN=${warn_count} FAIL=${fail_count}"
[[ "${fail_count}" -eq 0 ]]

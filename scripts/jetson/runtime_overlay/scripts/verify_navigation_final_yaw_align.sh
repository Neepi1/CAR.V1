#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
API_URL="${API_URL:-http://127.0.0.1:8080}"
ROBOT_API_CONFIG="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/robot_api_server.yaml"
NAV2_CONFIG="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/nav2.yaml"
EXECUTE_GOAL=false
GOAL_JSON=""

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() { echo "PASS: $*"; PASS_COUNT=$((PASS_COUNT + 1)); }
warn() { echo "WARN: $*"; WARN_COUNT=$((WARN_COUNT + 1)); }
fail() { echo "FAIL: $*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/verify_navigation_final_yaw_align.sh
  bash scripts/jetson/runtime_overlay/scripts/verify_navigation_final_yaw_align.sh --execute-goal --goal-json '{"x":1.0,"y":0.0,"yaw":0.0}'

Default mode is dry-run and never sends a navigation goal.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute-goal)
      EXECUTE_GOAL=true
      shift
      ;;
    --goal-json)
      GOAL_JSON="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

yaml_scalar() {
  local file="$1"
  local key="$2"
  python3 - "$file" "$key" <<'PY'
import sys
path, key = sys.argv[1], sys.argv[2]
try:
    with open(path, encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if stripped.startswith(key + ":"):
                print(stripped.split(":", 1)[1].strip().strip('"').strip("'"))
                raise SystemExit(0)
except FileNotFoundError:
    raise SystemExit(2)
raise SystemExit(1)
PY
}

nav2_local_costmap_global_frame() {
  python3 - "$NAV2_CONFIG" <<'PY'
import sys
path = sys.argv[1]
stack = []
try:
    lines = open(path, encoding="utf-8").read().splitlines()
except FileNotFoundError:
    raise SystemExit(2)
in_root = in_child = in_params = False
for raw in lines:
    if not raw.strip() or raw.lstrip().startswith("#"):
        continue
    indent = len(raw) - len(raw.lstrip(" "))
    text = raw.strip()
    if indent == 0:
        in_root = text == "local_costmap:"
        in_child = False
        in_params = False
        continue
    if in_root and indent == 2:
        in_child = text == "local_costmap:"
        in_params = False
        continue
    if in_root and in_child and indent == 4:
        in_params = text == "ros__parameters:"
        continue
    if in_root and in_child and in_params and indent == 6 and text.startswith("global_frame:"):
        print(text.split(":", 1)[1].strip().strip('"').strip("'"))
        raise SystemExit(0)
raise SystemExit(1)
PY
}

param_value() {
  local node="$1"
  local param="$2"
  local output
  if ! output="$(ros2 param get "$node" "$param" 2>&1)"; then
    echo ""
    return 1
  fi
  echo "$output" | awk -F': ' 'NF > 1 {print $NF}' | tail -n 1 | tr -d '"'
}

float_check() {
  python3 - "$@" <<'PY'
import sys
expr = sys.argv[1]
raise SystemExit(0 if eval(expr, {"__builtins__": {}}, {}) else 1)
PY
}

if [[ -f "$ROBOT_API_CONFIG" ]]; then
  pass "robot_api_server overlay config exists"
else
  fail "missing robot_api_server overlay config: $ROBOT_API_CONFIG"
fi

if [[ -f "$NAV2_CONFIG" ]]; then
  pass "nav2 overlay config exists"
else
  fail "missing nav2 overlay config: $NAV2_CONFIG"
fi

cfg_tolerance="$(yaml_scalar "$ROBOT_API_CONFIG" navigation_final_yaw_tolerance_rad || true)"
cfg_trigger="$(yaml_scalar "$ROBOT_API_CONFIG" navigation_final_yaw_align_trigger_rad || true)"
cfg_timeout="$(yaml_scalar "$ROBOT_API_CONFIG" navigation_final_yaw_align_timeout_sec || true)"
cfg_cmd_topic="$(yaml_scalar "$ROBOT_API_CONFIG" navigation_final_yaw_align_cmd_topic || true)"
cfg_bypass="$(yaml_scalar "$ROBOT_API_CONFIG" navigation_final_yaw_align_bypass_collision_monitor || true)"
cfg_local_frame="$(nav2_local_costmap_global_frame || true)"

[[ "$cfg_tolerance" == "0.05" || "$cfg_tolerance" == "0.050" ]] && pass "config final yaw tolerance is $cfg_tolerance" || warn "config final yaw tolerance is $cfg_tolerance"
[[ "$cfg_trigger" == "0.08" || "$cfg_trigger" == "0.080" ]] && pass "config final yaw trigger is $cfg_trigger" || warn "config final yaw trigger is $cfg_trigger"
[[ "$cfg_timeout" == "8.0" || "$cfg_timeout" == "8" ]] && pass "config final yaw timeout is $cfg_timeout" || warn "config final yaw timeout is $cfg_timeout"
[[ "$cfg_cmd_topic" != "/cmd_vel_safe" && "$cfg_cmd_topic" != "/cmd_vel" ]] && pass "config final yaw cmd topic is $cfg_cmd_topic" || fail "final yaw cmd topic must not be $cfg_cmd_topic"
[[ "$cfg_local_frame" == "odom" ]] && pass "config local_costmap global_frame is odom" || fail "config local_costmap global_frame is ${cfg_local_frame:-missing}"

runtime_tolerance="$(param_value /robot_api_server navigation_final_yaw_tolerance_rad || true)"
runtime_trigger="$(param_value /robot_api_server navigation_final_yaw_align_trigger_rad || true)"
runtime_timeout="$(param_value /robot_api_server navigation_final_yaw_align_timeout_sec || true)"
runtime_cmd_topic="$(param_value /robot_api_server navigation_final_yaw_align_cmd_topic || true)"
runtime_bypass="$(param_value /robot_api_server navigation_final_yaw_align_bypass_collision_monitor || true)"

if [[ -n "$runtime_tolerance" ]]; then
  float_check "${runtime_tolerance} <= 0.06" && pass "runtime final yaw tolerance <= 0.06 (${runtime_tolerance})" || warn "runtime final yaw tolerance is ${runtime_tolerance}"
else
  fail "missing runtime param navigation_final_yaw_tolerance_rad"
fi
if [[ -n "$runtime_trigger" && -n "$runtime_tolerance" ]]; then
  float_check "${runtime_trigger} >= ${runtime_tolerance}" && pass "runtime trigger >= tolerance (${runtime_trigger} >= ${runtime_tolerance})" || fail "runtime trigger < tolerance"
else
  fail "missing runtime param navigation_final_yaw_align_trigger_rad"
fi
if [[ -n "$runtime_timeout" ]]; then
  float_check "${runtime_timeout} >= 6.0" && pass "runtime final yaw timeout >= 6s (${runtime_timeout})" || warn "runtime final yaw timeout is ${runtime_timeout}"
else
  fail "missing runtime param navigation_final_yaw_align_timeout_sec"
fi
if [[ -n "$runtime_cmd_topic" ]]; then
  if [[ "$runtime_cmd_topic" == "/cmd_vel_safe" || "$runtime_cmd_topic" == "/cmd_vel" ]]; then
    fail "runtime final yaw cmd topic must not be $runtime_cmd_topic"
  elif [[ "$runtime_cmd_topic" == "/cmd_vel_collision_checked" ]]; then
    warn "runtime final yaw uses /cmd_vel_collision_checked: robot_safety is kept, collision_monitor is bypassed"
  else
    pass "runtime final yaw cmd topic is $runtime_cmd_topic"
  fi
else
  fail "missing runtime param navigation_final_yaw_align_cmd_topic"
fi
[[ "$runtime_bypass" == "True" || "$runtime_bypass" == "true" ]] && warn "final_yaw_align_bypass_collision_monitor=true" || pass "final yaw does not bypass collision_monitor"

runtime_lc_frame="$(param_value /local_costmap/local_costmap global_frame || true)"
[[ "$runtime_lc_frame" == "odom" ]] && pass "runtime local_costmap global_frame is odom" || fail "runtime local_costmap global_frame is ${runtime_lc_frame:-missing}"

controller_state="$(ros2 lifecycle get /controller_server 2>&1 || true)"
if echo "$controller_state" | grep -q "active"; then
  pass "controller_server active"
else
  fail "controller_server is not active: $controller_state"
fi

if ros2 node list 2>/dev/null | grep -Eiq 'fast.?lio|laserMapping'; then
  fail "FAST-LIO2-like node is present during navigation"
else
  pass "no FAST-LIO2-like navigation residue detected"
fi

obstacle_info="$(ros2 topic info -v /perception/obstacle_points 2>&1 || true)"
echo "$obstacle_info" | grep -q "Node name: local_costmap" && pass "local_costmap subscribes /perception/obstacle_points" || fail "local_costmap subscriber missing on /perception/obstacle_points"
echo "$obstacle_info" | grep -q "Node name: collision_monitor" && pass "collision_monitor subscribes /perception/obstacle_points" || fail "collision_monitor subscriber missing on /perception/obstacle_points"

state_json="$(curl -fsS "${API_URL}/api/v1/navigation/state" 2>/dev/null || true)"
if [[ -n "$state_json" ]]; then
  python3 - "$state_json" <<'PY'
import json, sys
payload = json.loads(sys.argv[1])
goal = payload.get("navigation_goal", {})
required = [
    "final_yaw_align_attempted",
    "final_yaw_align_blocked_reason",
    "final_yaw_align_duration_sec",
    "final_yaw_align_timeout_sec",
    "final_yaw_align_target_yaw_rad",
    "final_yaw_align_initial_yaw_error_rad",
    "final_yaw_align_final_yaw_error_rad",
    "final_yaw_align_max_xy_drift_m",
    "final_yaw_align_observed_xy_drift_m",
    "final_yaw_align_cmd_topic",
    "final_yaw_align_bypass_collision_monitor",
    "final_pose_verified",
    "final_pose_verify_reason",
]
missing = [key for key in required if key not in goal]
print("missing=" + ",".join(missing))
raise SystemExit(1 if missing else 0)
PY
  [[ $? -eq 0 ]] && pass "navigation_goal final yaw fields are present" || fail "navigation_goal final yaw fields are missing"
else
  fail "cannot read ${API_URL}/api/v1/navigation/state"
fi

if [[ "$EXECUTE_GOAL" == "true" ]]; then
  if [[ -z "$GOAL_JSON" ]]; then
    fail "--execute-goal requires --goal-json"
  else
    echo "INFO: sending navigation goal through POST /api/v1/navigation/goal"
    goal_response="$(curl -fsS -X POST "${API_URL}/api/v1/navigation/goal" -H 'Content-Type: application/json' --data "$GOAL_JSON" 2>&1 || true)"
    echo "$goal_response"
    python3 - "${API_URL}" <<'PY'
import json
import sys
import time
import urllib.request

api_url = sys.argv[1].rstrip("/")
terminal = {"succeeded", "failed", "canceled"}
deadline = time.time() + 240.0
last = {}
while time.time() < deadline:
    try:
        with urllib.request.urlopen(api_url + "/api/v1/navigation/state", timeout=2.0) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
            last = payload.get("navigation_goal", {})
    except Exception as exc:
        last = {"error": str(exc)}
    state = last.get("state")
    if state in terminal:
        break
    time.sleep(1.0)
print(json.dumps({
    key: last.get(key)
    for key in [
        "state",
        "phase",
        "final_distance_m",
        "final_yaw_error_rad",
        "position_reached",
        "final_pose_verified",
        "final_pose_verify_reason",
        "final_yaw_align_requested",
        "final_yaw_align_attempted",
        "final_yaw_align_succeeded",
        "final_yaw_align_blocked",
        "final_yaw_align_blocked_reason",
        "final_yaw_align_duration_sec",
        "final_yaw_align_cmd_topic",
        "final_yaw_align_bypass_collision_monitor",
    ]
}, ensure_ascii=False, indent=2))
raise SystemExit(0 if last.get("state") in terminal else 1)
PY
    [[ $? -eq 0 ]] && pass "execute-goal reached terminal navigation_goal state" || fail "execute-goal did not reach terminal state"
  fi
else
  pass "dry-run mode; no navigation goal was sent"
fi

echo "SUMMARY: PASS=${PASS_COUNT} WARN=${WARN_COUNT} FAIL=${FAIL_COUNT}"
[[ "$FAIL_COUNT" -eq 0 ]]

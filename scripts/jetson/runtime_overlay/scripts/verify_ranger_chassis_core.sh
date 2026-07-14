#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

failures=0

pass() {
  echo "[ranger-core] PASS $*"
}

fail() {
  echo "[ranger-core] FAIL $*" >&2
  failures=$((failures + 1))
}

topic_has_node() {
  local topic="$1"
  local pattern="$2"
  local info=""
  info="$(timeout 5 ros2 topic info -v "${topic}" 2>/dev/null || true)"
  grep -Eq "${pattern}" <<<"${info}"
}

if topic_has_node /cmd_vel "Node name: (robot_safety|robot_safety_node)"; then
  pass "/cmd_vel publisher is robot_safety"
else
  fail "/cmd_vel publisher is not robot_safety"
fi

if topic_has_node /cmd_vel "Node name: (ranger_base|ranger_base_node)"; then
  pass "/cmd_vel subscriber is ranger_base"
else
  fail "/cmd_vel ranger_base subscriber missing"
fi

status="$(
  timeout 5 ros2 topic echo --once --field data /ranger_base/status 2>/dev/null |
    awk '/^\{/{print; exit}' || true
)"
if [[ -z "${status}" ]]; then
  fail "/ranger_base/status sample missing"
else
  if STATUS_JSON="${status}" python3 - <<'PY'
import json
import os

payload = json.loads(os.environ["STATUS_JSON"])
assert payload["owner"] == "ranger_base"
assert payload["mode_switch_handshake_enabled"] is True
assert "desired_motion_mode" in payload
assert "actual_motion_mode" in payload
assert "mode_changing" in payload["actual_motion_mode"]
PY
  then
    pass "/ranger_base/status owns desired/actual mode handshake"
  else
    fail "/ranger_base/status is not valid chassis-core JSON"
  fi
fi

retired_processes="$(
  ps -eo args= |
    grep -E 'ros2 run ranger_mini3_mode_controller|/ranger_mini3_mode_controller/lib/|mode_controller_node --ros-args' |
    grep -v -E 'grep -E|verify_ranger_chassis_core' || true
)"
if [[ -n "${retired_processes}" ]]; then
  fail "retired ranger_mini3_mode_controller process is still running"
else
  pass "retired ranger_mini3_mode_controller process absent"
fi

shadow_info="$(timeout 5 ros2 topic info -v /ranger_mini3/mode_controller_shadow_cmd_vel 2>/dev/null || true)"
if grep -q "Publisher count: 0" <<<"${shadow_info}" || [[ -z "${shadow_info}" ]]; then
  pass "shadow cmd_vel publisher absent"
else
  fail "unexpected shadow cmd_vel publisher remains"
fi

if [[ "${failures}" -ne 0 ]]; then
  exit 1
fi

echo "[ranger-core] verification complete"

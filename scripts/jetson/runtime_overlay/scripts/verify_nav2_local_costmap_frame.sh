#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
PREFIX="[nav2-local-costmap-frame]"
FAILURES=0
WARNINGS=0

pass() {
  echo "${PREFIX} PASS $*"
}

warn() {
  echo "${PREFIX} WARN $*"
  WARNINGS=$((WARNINGS + 1))
}

fail() {
  echo "${PREFIX} FAIL $*"
  FAILURES=$((FAILURES + 1))
}

read_nav2_value() {
  local dotted_key="$1"
  python3 - "$NAV2_PARAMS_FILE" "$dotted_key" <<'PY'
import sys

path, dotted_key = sys.argv[1], sys.argv[2]
target = dotted_key.split(".")
stack = []

with open(path, encoding="utf-8") as f:
    for raw in f:
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or ":" not in line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        name, value = line.strip().split(":", 1)
        while stack and indent <= stack[-1][0]:
            stack.pop()
        current = [item[1] for item in stack] + [name]
        if current == target:
            print(value.strip().strip('"').strip("'"))
            sys.exit(0)
        if value.strip() == "":
            stack.append((indent, name))

sys.exit(1)
PY
}

param_value() {
  local node="$1"
  local name="$2"
  local output
  output="$(timeout 5 ros2 param get "${node}" "${name}" 2>&1 || true)"
  if [[ "${output}" =~ is:\ (.*)$ ]]; then
    printf '%s\n' "${BASH_REMATCH[1]}" | tr -d '"'
    return 0
  fi
  printf '%s\n' "${output}"
  return 1
}

check_static_config() {
  if [[ ! -f "${NAV2_PARAMS_FILE}" ]]; then
    fail "missing NAV2 params file: ${NAV2_PARAMS_FILE}"
    return
  fi

  local static_global_frame=""
  local static_robot_base_frame=""
  static_global_frame="$(read_nav2_value "local_costmap.local_costmap.ros__parameters.global_frame" || true)"
  static_robot_base_frame="$(read_nav2_value "local_costmap.local_costmap.ros__parameters.robot_base_frame" || true)"

  if [[ "${static_global_frame}" == "odom" ]]; then
    pass "static local_costmap.global_frame=odom (${NAV2_PARAMS_FILE})"
  elif [[ "${static_global_frame}" == "base_link" ]]; then
    fail "static local_costmap.global_frame is still base_link (${NAV2_PARAMS_FILE})"
  else
    fail "static local_costmap.global_frame is '${static_global_frame:-missing}', expected odom"
  fi

  if [[ "${static_robot_base_frame}" == "base_link" ]]; then
    pass "static local_costmap.robot_base_frame=base_link"
  else
    fail "static local_costmap.robot_base_frame is '${static_robot_base_frame:-missing}', expected base_link"
  fi
}

check_runtime_params() {
  local runtime_global_frame
  local runtime_robot_base_frame

  runtime_global_frame="$(param_value /local_costmap/local_costmap global_frame || true)"
  runtime_robot_base_frame="$(param_value /local_costmap/local_costmap robot_base_frame || true)"

  if [[ "${runtime_global_frame}" == "odom" ]]; then
    pass "runtime /local_costmap/local_costmap global_frame=odom"
  elif [[ "${runtime_global_frame}" == "base_link" ]]; then
    fail "runtime /local_costmap/local_costmap global_frame is still base_link; restart Nav2 runtime with updated nav2.yaml"
  else
    fail "runtime /local_costmap/local_costmap global_frame is '${runtime_global_frame:-unavailable}', expected odom"
  fi

  if [[ "${runtime_robot_base_frame}" == "base_link" ]]; then
    pass "runtime /local_costmap/local_costmap robot_base_frame=base_link"
  else
    fail "runtime /local_costmap/local_costmap robot_base_frame is '${runtime_robot_base_frame:-unavailable}', expected base_link"
  fi
}

tf_translation_sample() {
  timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>&1 |
    awk '/Translation:/ {getline; gsub(/^[[:space:]]+/, "", $0); print; exit}'
}

check_tf() {
  local tf_output
  tf_output="$(timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>&1 || true)"
  if grep -q "Translation:" <<<"${tf_output}"; then
    pass "tf odom -> base_link is available"
  else
    fail "tf odom -> base_link is unavailable"
    return
  fi

  if [[ "${NJRH_VERIFY_ROBOT_MOVING:-false}" == "true" ]]; then
    local first_sample=""
    local second_sample=""
    first_sample="$(tf_translation_sample || true)"
    sleep "${NJRH_VERIFY_TF_MOVE_SAMPLE_SEC:-2}"
    second_sample="$(tf_translation_sample || true)"
    if [[ -n "${first_sample}" && -n "${second_sample}" && "${first_sample}" != "${second_sample}" ]]; then
      pass "tf odom -> base_link changed while robot was moving"
    else
      fail "tf odom -> base_link did not change while NJRH_VERIFY_ROBOT_MOVING=true"
    fi
  else
    warn "tf is available; run with NJRH_VERIFY_ROBOT_MOVING=true during a short goal to verify movement change"
  fi
}

check_local_state_odom() {
  if timeout 6 ros2 topic echo /local_state/odometry --once >/dev/null 2>&1; then
    pass "/local_state/odometry is publishing"
  else
    fail "/local_state/odometry is not publishing"
  fi
}

check_obstacle_subscribers() {
  local info
  info="$(timeout 8 ros2 topic info -v /perception/obstacle_points 2>&1 || true)"

  if grep -Eq "Node name: local_costmap|/local_costmap/local_costmap" <<<"${info}"; then
    pass "/perception/obstacle_points has local_costmap subscriber"
  else
    fail "/perception/obstacle_points is missing local_costmap subscriber"
  fi

  if grep -Eq "Node name: collision_monitor|/collision_monitor" <<<"${info}"; then
    pass "/perception/obstacle_points has collision_monitor subscriber"
  else
    fail "/perception/obstacle_points is missing collision_monitor subscriber"
  fi
}

check_controller_active() {
  local state
  state="$(timeout 5 ros2 lifecycle get /controller_server 2>&1 || true)"
  if grep -qi "active" <<<"${state}"; then
    pass "controller_server lifecycle is active"
  else
    fail "controller_server lifecycle is not active: ${state:-unavailable}"
  fi
}

check_static_config
check_runtime_params
check_tf
check_local_state_odom
check_obstacle_subscribers
check_controller_active

if [[ "${FAILURES}" -gt 0 ]]; then
  echo "${PREFIX} FAIL failures=${FAILURES} warnings=${WARNINGS}"
  exit 1
fi

if [[ "${WARNINGS}" -gt 0 ]]; then
  echo "${PREFIX} WARN failures=0 warnings=${WARNINGS}"
  exit 0
fi

echo "${PREFIX} PASS failures=0 warnings=0"

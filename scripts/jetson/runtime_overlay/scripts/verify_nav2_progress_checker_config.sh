#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
PREFIX="[nav2-progress-checker]"
FAILURES=0
WARNINGS=0

pass() { echo "${PREFIX} PASS $*"; }
warn() { echo "${PREFIX} WARN $*"; WARNINGS=$((WARNINGS + 1)); }
fail() { echo "${PREFIX} FAIL $*"; FAILURES=$((FAILURES + 1)); }

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
            raise SystemExit(0)
        if value.strip() == "":
            stack.append((indent, name))
raise SystemExit(1)
PY
}

numeric_between() {
  python3 - "$1" "$2" "$3" <<'PY'
import sys
value = float(sys.argv[1])
low = float(sys.argv[2])
high = float(sys.argv[3])
raise SystemExit(0 if low <= value <= high else 1)
PY
}

param_value() {
  local node="$1"
  local name="$2"
  local output
  output="$(timeout 6 ros2 param get "${node}" "${name}" 2>&1 || true)"
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

  local plugin radius angle timeout rotate threshold local_frame yaw_tol
  plugin="$(read_nav2_value "controller_server.ros__parameters.progress_checker.plugin" || true)"
  radius="$(read_nav2_value "controller_server.ros__parameters.progress_checker.required_movement_radius" || true)"
  angle="$(read_nav2_value "controller_server.ros__parameters.progress_checker.required_movement_angle" || true)"
  timeout="$(read_nav2_value "controller_server.ros__parameters.progress_checker.movement_time_allowance" || true)"
  rotate="$(read_nav2_value "controller_server.ros__parameters.FollowPath.rotate_to_goal_heading" || true)"
  threshold="$(read_nav2_value "controller_server.ros__parameters.FollowPath.angular_dist_threshold" || true)"
  local_frame="$(read_nav2_value "local_costmap.local_costmap.ros__parameters.global_frame" || true)"
  yaw_tol="$(read_nav2_value "controller_server.ros__parameters.goal_checker.yaw_goal_tolerance" || true)"

  [[ "${plugin}" == "nav2_controller::PoseProgressChecker" ]] \
    && pass "static progress_checker.plugin=${plugin}" \
    || fail "static progress_checker.plugin=${plugin:-missing}, expected nav2_controller::PoseProgressChecker"

  [[ "${radius}" == "0.10" || "${radius}" == "0.1" ]] \
    && pass "static required_movement_radius=${radius}" \
    || fail "static required_movement_radius=${radius:-missing}, expected 0.10"

  if [[ -n "${angle}" ]] && numeric_between "${angle}" 0.05 0.20; then
    pass "static required_movement_angle=${angle}"
  else
    fail "static required_movement_angle=${angle:-missing}, expected 0.05..0.20"
  fi

  [[ "${timeout}" == "12.0" || "${timeout}" == "12" ]] \
    && pass "static movement_time_allowance=${timeout}" \
    || fail "static movement_time_allowance=${timeout:-missing}, expected unchanged 12.0"

  [[ "${rotate}" == "true" ]] \
    && pass "static FollowPath.rotate_to_goal_heading=true" \
    || fail "static FollowPath.rotate_to_goal_heading=${rotate:-missing}, expected true"

  if [[ -n "${threshold}" ]]; then
    pass "static FollowPath.angular_dist_threshold=${threshold}"
  else
    fail "missing FollowPath.angular_dist_threshold"
  fi

  [[ "${local_frame}" == "odom" ]] \
    && pass "static local_costmap.global_frame=odom" \
    || fail "static local_costmap.global_frame=${local_frame:-missing}, expected odom"

  if [[ -n "${yaw_tol}" ]] && numeric_between "${yaw_tol}" 0.10 3.14; then
    pass "static yaw_goal_tolerance=${yaw_tol}"
  else
    fail "static yaw_goal_tolerance=${yaw_tol:-missing}, must not be made tiny"
  fi
}

check_pose_progress_plugin_available() {
  if grep -R "PoseProgressChecker" /opt/ros/humble/share/nav2_controller /opt/ros/humble/lib >/dev/null 2>&1; then
    pass "Humble nav2_controller exposes PoseProgressChecker"
  else
    fail "PoseProgressChecker not found in /opt/ros/humble; do not start Nav2 with this config"
  fi
}

check_runtime_config() {
  local lifecycle plugin angle rotate threshold frame
  lifecycle="$(timeout 6 ros2 lifecycle get /controller_server 2>&1 || true)"
  if [[ "${lifecycle}" == *"active [3]"* ]]; then
    pass "controller_server active"
  else
    fail "controller_server is not active: ${lifecycle}"
  fi

  plugin="$(param_value /controller_server progress_checker.plugin || true)"
  [[ "${plugin}" == "nav2_controller::PoseProgressChecker" ]] \
    && pass "runtime progress_checker.plugin=${plugin}" \
    || warn "runtime progress_checker.plugin unavailable or unexpected: ${plugin}"

  angle="$(param_value /controller_server progress_checker.required_movement_angle || true)"
  if [[ "${angle}" =~ ^[-+0-9.]+$ ]] && numeric_between "${angle}" 0.05 0.20; then
    pass "runtime required_movement_angle=${angle}"
  else
    warn "runtime required_movement_angle unavailable or outside expected range: ${angle}"
  fi

  rotate="$(param_value /controller_server FollowPath.rotate_to_goal_heading || true)"
  [[ "${rotate}" == "True" || "${rotate}" == "true" ]] \
    && pass "runtime FollowPath.rotate_to_goal_heading=${rotate}" \
    || warn "runtime FollowPath.rotate_to_goal_heading unavailable or unexpected: ${rotate}"

  threshold="$(param_value /controller_server FollowPath.angular_dist_threshold || true)"
  [[ -n "${threshold}" ]] \
    && pass "runtime FollowPath.angular_dist_threshold=${threshold}" \
    || warn "runtime angular_dist_threshold unavailable"

  frame="$(param_value /local_costmap/local_costmap global_frame || true)"
  [[ "${frame}" == "odom" ]] \
    && pass "runtime local_costmap.global_frame=odom" \
    || fail "runtime local_costmap.global_frame=${frame}, expected odom"
}

check_cmd_chain() {
  local nav_info collision_info safe_info final_info
  nav_info="$(timeout 8 ros2 topic info -v /cmd_vel_nav 2>&1 || true)"
  collision_info="$(timeout 8 ros2 topic info -v /cmd_vel_collision_checked 2>&1 || true)"
  safe_info="$(timeout 8 ros2 topic info -v /cmd_vel_safe 2>&1 || true)"
  final_info="$(timeout 8 ros2 topic info -v /cmd_vel 2>&1 || true)"

  [[ "${nav_info}" == *"Node name: velocity_smoother"* && "${nav_info}" == *"Node name: collision_monitor"* ]] \
    && pass "/cmd_vel_nav connects velocity_smoother to collision_monitor" \
    || warn "/cmd_vel_nav chain not fully visible"

  [[ "${collision_info}" == *"Node name: collision_monitor"* && "${collision_info}" == *"Node name: robot_safety"* ]] \
    && pass "/cmd_vel_collision_checked connects collision_monitor to robot_safety" \
    || warn "/cmd_vel_collision_checked chain not fully visible"

  [[ "${safe_info}" == *"Node name: robot_safety"* ]] \
    && pass "/cmd_vel_safe is published by robot_safety" \
    || warn "/cmd_vel_safe publisher not visible"

  [[ "${final_info}" == *"Node name: ranger_base_node"* ]] \
    && pass "/cmd_vel reaches ranger_base_node" \
    || warn "/cmd_vel ranger_base_node subscriber not visible"
}

check_static_config
check_pose_progress_plugin_available
check_runtime_config
check_cmd_chain

echo "${PREFIX} SUMMARY failures=${FAILURES} warnings=${WARNINGS}"
[[ "${FAILURES}" -eq 0 ]] || exit 1

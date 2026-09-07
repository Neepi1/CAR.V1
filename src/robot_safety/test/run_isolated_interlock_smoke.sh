#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
workspace="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
test_setup="${NJRH_ROBOT_SAFETY_TEST_SETUP:-${workspace}/install/setup.bash}"
test_binary="${NJRH_ROBOT_SAFETY_TEST_BIN:-${workspace}/install/robot_safety/lib/robot_safety/robot_safety_node}"
source "${test_setup}"
set -u

export ROS_DOMAIN_ID=181
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

log_file="/tmp/p6_isolated_safety_${$}.log"
node_pid=

cleanup() {
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill -TERM "${node_pid}"
    wait "${node_pid}" 2>/dev/null || true
  fi
  rm -f "${log_file}"
}
trap cleanup EXIT

"${test_binary}" \
  --ros-args \
  -r __node:=p6_isolated_safety \
  -p cmd_vel_in_topic:=/p6_test/input \
  -p api_cmd_vel_in_topic:=/p6_test/api \
  -p docking_cmd_vel_in_topic:=/p6_test/docking \
  -p cmd_vel_out_topic:=/p6_test/output \
  -p cmd_vel_mirror_topic:=/p6_test/mirror \
  -p execution_mode_state_topic:=/p6_test/mode \
  -p reverse_enable_topic:=/p6_test/reverse_enable \
  -p docking_reverse_enable_topic:=/p6_test/docking_reverse_enable \
  -p teleop_reverse_enable_topic:=/p6_test/teleop_reverse_enable \
  -p nav_terminal_reverse_enable_topic:=/p6_test/nav_terminal_reverse_enable \
  -p nav_terminal_lateral_enable_topic:=/p6_test/nav_terminal_lateral_enable \
  -p require_localization_health:=false \
  -p allow_reverse:=true \
  -p bms_docking_interlock_enabled:=false \
  -p enable_bms_contact_guard:=false \
  -p enable_docking_status_guard:=false \
  -p enable_docked_latch_file_guard:=false \
  -p spin_to_drive_settle_enabled:=false \
  -p mode_exit_guard_enabled:=false \
  -p publish_rate_hz:=1.0 \
  >"${log_file}" 2>&1 &
node_pid=$!

sleep 1
if ! kill -0 "${node_pid}" 2>/dev/null; then
  cat "${log_file}" >&2
  exit 1
fi
python3 \
  "${workspace}/src/robot_safety/test/isolated_interlock_smoke.py"

#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
workspace="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
test_setup="${NJRH_ROBOT_SAFETY_TEST_SETUP:-${workspace}/install/setup.bash}"
test_binary="${NJRH_ROBOT_SAFETY_TEST_BIN:-${workspace}/install/robot_safety/lib/robot_safety/robot_safety_node}"
source "${test_setup}"
set -u

export ROS_DOMAIN_ID=182
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

log_file="/tmp/normal_lateral_watchdog_safety_${$}.log"
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
  -r __node:=normal_lateral_watchdog_safety \
  -p cmd_vel_in_topic:=/lateral_guard_test/input \
  -p api_cmd_vel_in_topic:=/lateral_guard_test/api \
  -p docking_cmd_vel_in_topic:=/lateral_guard_test/docking \
  -p cmd_vel_out_topic:=/lateral_guard_test/output \
  -p cmd_vel_mirror_topic:=/lateral_guard_test/mirror \
  -p mode_controller_status_topic:=/lateral_guard_test/ranger_status \
  -p nav_terminal_lateral_enable_topic:=/lateral_guard_test/nav_terminal_lateral_enable \
  -p require_localization_health:=false \
  -p block_normal_motion_when_docked:=false \
  -p bms_docking_interlock_enabled:=false \
  -p enable_bms_contact_guard:=false \
  -p enable_docking_status_guard:=false \
  -p enable_docked_latch_file_guard:=false \
  -p spin_to_drive_settle_enabled:=false \
  -p mode_exit_guard_enabled:=true \
  -p zero_cmd_priority_enabled:=false \
  -p publish_zero_on_startup:=false \
  -p watchdog_timeout_sec:=0.35 \
  -p publish_rate_hz:=10.0 \
  >"${log_file}" 2>&1 &
node_pid=$!

sleep 1
if ! kill -0 "${node_pid}" 2>/dev/null; then
  cat "${log_file}" >&2
  exit 1
fi

if ! python3 \
  "${workspace}/src/robot_safety/test/isolated_normal_lateral_watchdog_smoke.py"
then
  cat "${log_file}" >&2
  exit 1
fi

#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /workspaces/njrh-v3/workspace1/install/setup.bash
if [[ -n "${NJRH_TEST_INSTALL_PREFIX:-}" ]]; then
  source "${NJRH_TEST_INSTALL_PREFIX}/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${NJRH_TEST_ROS_DOMAIN_ID:-182}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

install_prefix="${NJRH_TEST_INSTALL_PREFIX:-/workspaces/njrh-v3/workspace1/install}"
source_root="${NJRH_TEST_SOURCE_ROOT:-/workspaces/njrh-v3/workspace1/src/robot_localization_bridge}"
log_file="/tmp/p6_disabled_floor_transition_${$}.log"
node_pid=

cleanup() {
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill -TERM "${node_pid}"
    wait "${node_pid}" 2>/dev/null || true
  fi
  if [[ "${log_file}" == /tmp/p6_disabled_floor_transition_*.log ]]; then
    rm -f -- "${log_file}"
  fi
}
trap cleanup EXIT

"${install_prefix}/robot_localization_bridge/lib/robot_localization_bridge/localization_bridge_node" \
  --ros-args \
  -r __node:=p6_disabled_floor_transition_bridge \
  -p publish_tf:=false \
  -p amcl_input_enabled:=false \
  >"${log_file}" 2>&1 &
node_pid=$!

sleep 1
if ! kill -0 "${node_pid}" 2>/dev/null; then
  cat "${log_file}" >&2
  exit 1
fi

python3 "${source_root}/test/isolated_floor_transition_disabled_smoke.py"

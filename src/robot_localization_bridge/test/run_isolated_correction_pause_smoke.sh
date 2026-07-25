#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /workspaces/njrh-v3/workspace1/install/setup.bash
set -u

export ROS_DOMAIN_ID=182
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

log_file="/tmp/p6_isolated_bridge_${$}.log"
node_pid=

cleanup() {
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill -TERM "${node_pid}"
    wait "${node_pid}" 2>/dev/null || true
  fi
  rm -f "${log_file}"
}
trap cleanup EXIT

/workspaces/njrh-v3/workspace1/install/robot_localization_bridge/lib/robot_localization_bridge/localization_bridge_node \
  --ros-args \
  -r __node:=p6_isolated_bridge \
  -p publish_tf:=false \
  -p amcl_input_enabled:=false \
  >"${log_file}" 2>&1 &
node_pid=$!

sleep 1
if ! kill -0 "${node_pid}" 2>/dev/null; then
  cat "${log_file}" >&2
  exit 1
fi
python3 \
  /workspaces/njrh-v3/workspace1/src/robot_localization_bridge/test/isolated_correction_pause_smoke.py

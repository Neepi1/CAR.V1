#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 2 ]] || { echo "usage: $0 LIBRARY_DIRECTORY REPORT_DIRECTORY" >&2; exit 2; }
library_dir="$(realpath "$1")"
report="$(realpath "$2")"
[[ -f "$library_dir/libfastrtps.so.2.6" && "$report" == /tmp/njrh_reports/* ]]
# Never source production common_env: this is an isolated, non-TF-publishing
# instance of the already-installed bridge, using its existing package test.
exec timeout --kill-after=3 50 unshare --net --mount --ipc --fork bash -c '
  set -eo pipefail
  mount --make-rprivate /
  mount -t tmpfs -o size=64m tmpfs /dev/shm
  ip link set lo up
  source /opt/ros/humble/setup.bash
  source /workspaces/njrh-v3/workspace1/install/setup.bash
  set -u
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  export SKIP_DEFAULT_XML=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export ROS_DOMAIN_ID=182 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export LD_LIBRARY_PATH="$1:${LD_LIBRARY_PATH:-/opt/ros/humble/lib}"
  echo "network_namespace=$(readlink /proc/self/ns/net)"
  bridge=/workspaces/njrh-v3/workspace1/install/robot_localization_bridge/lib/robot_localization_bridge/localization_bridge_node
  # rclcpp loads its selected RMW at runtime, so inspect that RMW dependency
  # here and prove the bridge actual mapping after its service test below.
  ldd /opt/ros/humble/lib/librmw_fastrtps_cpp.so | grep -E "fastrtps|fastcdr"
  "$bridge" --ros-args -r __node:=p6_isolated_bridge \
    -p publish_tf:=false -p amcl_input_enabled:=false > "$2/bridge-smoke-node.log" 2>&1 &
  node_pid=$!
  cleanup() {
    if kill -0 "$node_pid" 2>/dev/null; then
      kill -TERM "$node_pid"
      wait "$node_pid" || true
    fi
  }
  trap cleanup EXIT
  python3 /workspaces/njrh-v3/workspace1/src/robot_localization_bridge/test/isolated_correction_pause_smoke.py
  grep /libfastrtps.so /proc/"$node_pid"/maps
  kill -INT "$node_pid"
  wait "$node_pid"
  trap - EXIT
  echo BRIDGE_SMOKE_PASS
' isolated-bridge-dds "$library_dir" "$report"

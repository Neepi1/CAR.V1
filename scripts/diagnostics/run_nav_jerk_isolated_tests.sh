#!/usr/bin/env bash
# Only test-owned processes are stopped. No installed production code is invoked.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" != "--inside" ]]; then
  export NAV_JERK_PARENT_NET="$(readlink /proc/self/ns/net)"
  export NAV_JERK_PARENT_IPC="$(readlink /proc/self/ns/ipc)"
  exec unshare --net --ipc --mount --fork bash "$0" --inside "$@"
fi
shift
[[ "$(readlink /proc/self/ns/net)" != "${NAV_JERK_PARENT_NET}" ]]
[[ "$(readlink /proc/self/ns/ipc)" != "${NAV_JERK_PARENT_IPC}" ]]
# Private /dev/shm rules out Fast DDS shared-memory connectivity to the robot.
mount --make-rprivate /
mount -t tmpfs -o size=128m tmpfs /dev/shm
ip link set lo up
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
python3 -m unittest discover -s "$script_dir" -p test_nav_jerk_capture.py -v
python3 "$script_dir/test_nav_jerk_required_evidence.py" -v
python3 "$script_dir/test_nav_jerk_ros_isolated.py" -v "$@"

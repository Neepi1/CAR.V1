#!/usr/bin/env bash
# Execute only this test binary, isolated from all real robot DDS and SHM.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspaces/njrh-v3/workspace1/install/setup.bash
set -u
binary=$(realpath "$1")
shift
exec unshare --net --ipc --pid --fork --mount --mount-proc --propagation private \
  bash -c '
    set -euo pipefail
    mount -t tmpfs tmpfs /dev/shm
    ip link set lo up
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export NJRH_ELEVATOR_TEST_ISOLATED=1
    exec timeout 90 "$@"
  ' bash "$binary" "$@"

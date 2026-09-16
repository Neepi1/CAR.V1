#!/usr/bin/env bash
# Test binaries only; never launch a production API or connect to the robot DDS.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspaces/njrh-v3/workspace1/install/setup.bash
set -u
build=$(realpath "$1")
exec unshare --net --ipc --pid --fork --mount --mount-proc --propagation private \
  bash -c '
    set -euo pipefail
    mount -t tmpfs tmpfs /dev/shm
    ip link set lo up
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
    export ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    timeout 120 "$1/test_elevator_ros_executor" --gtest_repeat="${NJRH_EXECUTOR_TEST_REPEAT:-1}"
    if test -x "$1/test_elevator_ros_events"; then
      timeout 180 "$1/test_elevator_ros_events" --gtest_repeat="${NJRH_EXECUTOR_TEST_REPEAT:-1}"
    fi
    for test in test_elevator_test_module test_elevator_runtime_policy test_elevator_recovery_action_barrier; do
      if test -x "$1/$test"; then timeout 120 "$1/$test"; fi
    done
    if test -x /workspaces/njrh-v3/workspace1/build/robot_elevator_manager/test_elevator_execution_module; then
      timeout 120 /workspaces/njrh-v3/workspace1/build/robot_elevator_manager/test_elevator_execution_module
    fi
  ' bash "$build"

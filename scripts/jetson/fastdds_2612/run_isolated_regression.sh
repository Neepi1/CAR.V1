#!/usr/bin/env bash
set -euo pipefail
# A private network AND /dev/shm prevent discovery/data-sharing with the robot.
[[ $# -ge 2 && $# -le 3 ]] || {
  echo "usage: $0 TEST_BINARY LIBRARY_DIRECTORY [REPEAT=10]" >&2; exit 2;
}
test_binary="$(realpath "$1")"
library_dir="$(realpath "$2")"
repeat="${3:-10}"
[[ -x "$test_binary" && -f "$library_dir/libfastrtps.so.2.6" ]]
[[ "$repeat" =~ ^[1-9][0-9]*$ ]] && (( repeat <= 20 ))
exec timeout --kill-after=3 150 unshare --net --mount --ipc --fork bash -c '
  set -euo pipefail
  mount --make-rprivate /
  mount -t tmpfs -o size=64m tmpfs /dev/shm
  ip link set lo up
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  export SKIP_DEFAULT_XML=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export LD_LIBRARY_PATH="$2:/opt/ros/humble/lib"
  echo "network_namespace=$(readlink /proc/self/ns/net)"
  ldd "$1" | grep -E "fastrtps|fastcdr"
  exec "$1" --gtest_filter=PubSubFlowControllers.AsyncPubSubReaderRemovalWhileDeliveringDoesNotDeadlock \
    --gtest_repeat="$3" --gtest_break_on_failure
' isolated-fastdds "$test_binary" "$library_dir" "$repeat"

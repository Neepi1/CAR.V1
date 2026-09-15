#!/usr/bin/env bash
# Run inside NJRH-car as root; never creates a listener on the vehicle network.
set -eo pipefail
if [[ $# -ne 2 ]]; then
  echo "usage: $0 APPROVED_API_BINARY REPORT_OUTPUT_DIR" >&2
  exit 2
fi
script_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/humble/setup.bash
source "$script_root/install/local_setup.bash"
export API_TEST_ORIGINAL_NET=$(readlink /proc/self/ns/net)
export API_TEST_ORIGINAL_IPC=$(readlink /proc/self/ns/ipc)
export API_TEST_ORIGINAL_MNT=$(readlink /proc/self/ns/mnt)
export API_TEST_ORIGINAL_PID=$(readlink /proc/self/ns/pid)
exec unshare --mount --pid --fork --mount-proc --net --ipc bash -c '
  mount --make-rprivate /
  mount -t tmpfs tmpfs /dev/shm
  ip link set lo up
  exec timeout --kill-after=3 90 taskset -c 0,1,4 python3 "$1/src/robot_api_server/test/infrastructure/http/api_pose_query_regression.py" "$2" --output "$3"
' _ "$script_root" "$1" "$2"

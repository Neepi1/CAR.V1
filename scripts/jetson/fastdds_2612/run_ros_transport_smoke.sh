#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 2 ]] || { echo "usage: $0 LIBRARY_DIRECTORY UDPv4|SHM" >&2; exit 2; }
library_dir="$(realpath "$1")"
transport="$2"
[[ "$transport" == UDPv4 || "$transport" == SHM ]]
[[ -f "$library_dir/libfastrtps.so.2.6" ]]
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec timeout --kill-after=3 60 unshare --net --mount --ipc --fork bash -c '
  set -eo pipefail
  mount --make-rprivate /
  mount -t tmpfs -o size=128m tmpfs /dev/shm
  ip link set lo up
  source /opt/ros/humble/setup.bash
  set -u
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  export SKIP_DEFAULT_XML=1 NJRH_DDS_ISOLATED_TEST=1
  export ROS_DOMAIN_ID=183 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS="$2"
  export LD_LIBRARY_PATH="$1:${LD_LIBRARY_PATH:-/opt/ros/humble/lib}"
  echo "transport=$2 namespace=$(readlink /proc/self/ns/net)"
  exec python3 "$3/test/ros_transport_smoke.py"
' isolated-ros-dds "$library_dir" "$transport" "$script_dir"

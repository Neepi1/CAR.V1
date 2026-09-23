#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 2 ]]; then echo "usage: $0 ABS_NATIVE_BINARY NEW_REPORT_DIRECTORY [unittest arguments]" >&2; exit 2; fi
binary=$(realpath "$1")
reports=$(realpath -m "$2")
shift 2
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p -- "$reports"
exec unshare --net --ipc --mount --fork bash -c '
  set -euo pipefail
  mount --make-rprivate /
  mount -t tmpfs tmpfs /dev/shm
  ip link set lo up
  export NAVLITE_ISOLATED=1 ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1
  export FASTRTPS_DEFAULT_PROFILES_FILE="$1/isolated_fastdds.xml"
  export FASTDDS_DEFAULT_PROFILES_FILE="$1/isolated_fastdds.xml"
  export ROS_HOME="$3/ros_home"
  script_dir=$1; binary=$2; reports=$3; shift 3
  exec python3 -B "$script_dir/test_native_capture.py" --binary "$binary" --output "$reports" "$@"
' bash "$script_dir" "$binary" "$reports" "$@"

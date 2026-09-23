#!/usr/bin/env bash
# Builds diagnostic-only candidates in /tmp, never installs or talks to the robot ROS domain.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
nav_source="$script_dir/../../src/robot_nav_config"
output=""
while (($#)); do
  case "$1" in
    --source-nav) nav_source=$2; shift 2 ;;
    --output) output=$2; shift 2 ;;
    *) printf 'usage: %s [--source-nav package_dir] [--output new_/tmp/njrh_reports/directory]\n' "$0" >&2; exit 2 ;;
  esac
done
[[ $EUID -eq 0 ]] || { echo 'Run inside NJRH-car as root: tests require private net/IPC/mount namespaces.' >&2; exit 2; }
[[ -r /opt/ros/humble/setup.bash && -r "$nav_source/tools/navlite_mppi_replay.cpp" ]] || exit 2
if [[ -z $output ]]; then
  mkdir -p /tmp/njrh_reports
  output=$(mktemp -d /tmp/njrh_reports/navlite_snapshot_tests_XXXXXX)
else
  [[ $output == /tmp/njrh_reports/* && ! -e $output ]] || { echo 'Output must be a new /tmp/njrh_reports subdirectory.' >&2; exit 2; }
  mkdir -p -- "$output"
fi
mkdir -- "$output/project"
set +u
source /opt/ros/humble/setup.bash
set -u
cp -- "$script_dir/navlite_snapshot_tests.CMakeLists.txt" "$output/project/CMakeLists.txt"
cmake -S "$output/project" -B "$output/build" -DNAV_SOURCE="$nav_source" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' > "$output/configure.log" 2>&1
cmake --build "$output/build" -j1 > "$output/build.log" 2>&1
"$output/build/test_transport" > "$output/transport.log" 2>&1
unshare --net --ipc --mount --fork bash -c '
  set -euo pipefail
  mount --make-rprivate /
  mount -t tmpfs -o size=64m tmpfs /dev/shm
  ip link set lo up
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export NAVLITE_ISOLATED=1 ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  exec "$1"
' bash "$output/build/test_integration" > "$output/integration.log" 2>&1
sha256sum "$nav_source/src/chassis_dynamics/mppi_controller.cpp" "$nav_source/include/robot_nav_config/"navlite_snapshot*.hpp "$output/build/"*candidate.so > "$output/hashes.txt"
printf 'PASS isolated snapshots: %s\n' "$output"
tail -8 "$output/transport.log"
tail -3 "$output/integration.log"

#!/usr/bin/env bash
# Tests the supplied built archive, never recompiles policy source or starts ROS.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "usage: bash $0 /absolute/path/libelevator_entry_collision_bypass_policy.a" >&2
  exit 2
fi
archive=$(realpath -e -- "$1")
package=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p /tmp/njrh_reports
report=$(mktemp -d /tmp/njrh_reports/elevator_bypass_archive_XXXXXXXX)
echo "report_dir=$report"
sha256sum "$archive" "$package/test/test_elevator_entry_collision_bypass_policy.cpp" \
  | tee "$report/inputs.sha256"
"${CXX:-c++}" -std=c++17 -I"$package/include" \
  "$package/test/test_elevator_entry_collision_bypass_policy.cpp" "$archive" \
  -lgtest_main -lgtest -pthread -o "$report/check_archive" \
  >"$report/compile.log" 2>&1
"$report/check_archive" | tee "$report/result.log"

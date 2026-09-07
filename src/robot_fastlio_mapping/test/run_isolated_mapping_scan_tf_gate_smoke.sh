#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
GATE_BIN="${NJRH_MAPPING_SCAN_TF_GATE_BIN:-${PROJECT_ROOT}/install/robot_fastlio_mapping/lib/robot_fastlio_mapping/mapping_scan_tf_gate_node}"
LOG_FILE="$(mktemp /tmp/njrh_mapping_scan_tf_gate_smoke.XXXXXX.log)"
GATE_PID=""

cleanup() {
  if [[ -n "${GATE_PID}" ]] && kill -0 "${GATE_PID}" 2>/dev/null; then
    kill -INT "${GATE_PID}" 2>/dev/null || true
    wait "${GATE_PID}" 2>/dev/null || true
  fi
  rm -f "${LOG_FILE}"
}
trap cleanup EXIT INT TERM

[[ -x "${GATE_BIN}" ]] || {
  echo "missing mapping scan/TF gate binary: ${GATE_BIN}" >&2
  exit 1
}

export ROS_DOMAIN_ID="${NJRH_MAPPING_SCAN_TF_GATE_TEST_DOMAIN_ID:-196}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

"${GATE_BIN}" --ros-args \
  -p input_topic:=/scan_tf_gate_test/scan_raw \
  -p output_topic:=/scan_tf_gate_test/scan \
  -p status_topic:=/scan_tf_gate_test/status \
  -p target_frame:=mapping_odom \
  -p required_frame:=lidar_level_link \
  -p tf_topic:=/scan_tf_gate_test/tf \
  -p tf_static_topic:=/scan_tf_gate_test/tf_static \
  -p max_wait_sec:=2.0 \
  -p post_tf_settle_ms:=20.0 >"${LOG_FILE}" 2>&1 &
GATE_PID=$!

python3 "${SCRIPT_DIR}/isolated_mapping_scan_tf_gate_smoke.py" || {
  cat "${LOG_FILE}" >&2
  exit 1
}

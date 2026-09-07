#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PROBE_BIN="${NJRH_RUNTIME_READINESS_PROBE_BIN:-${PROJECT_ROOT}/install/robot_bringup/lib/robot_bringup/runtime_readiness_probe}"
SUCCESS_LOG="$(mktemp /tmp/njrh_stamped_scan_tf_success.XXXXXX.log)"
FAILURE_LOG="$(mktemp /tmp/njrh_stamped_scan_tf_failure.XXXXXX.log)"
PROBE_PID=""

cleanup() {
  if [[ -n "${PROBE_PID}" ]] && kill -0 "${PROBE_PID}" 2>/dev/null; then
    kill -INT "${PROBE_PID}" 2>/dev/null || true
    wait "${PROBE_PID}" 2>/dev/null || true
  fi
  rm -f "${SUCCESS_LOG}" "${FAILURE_LOG}"
}
trap cleanup EXIT INT TERM

[[ -x "${PROBE_BIN}" ]] || {
  echo "missing runtime readiness probe: ${PROBE_BIN}" >&2
  exit 1
}

export ROS_DOMAIN_ID="${NJRH_STAMPED_SCAN_TF_TEST_DOMAIN_ID:-197}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

"${PROBE_BIN}" stamped-scan-tf \
  /stamped_scan_tf_test/scan \
  /stamped_scan_tf_test/tf \
  mapping_odom 5 3 >"${SUCCESS_LOG}" 2>&1 &
PROBE_PID=$!
python3 "${SCRIPT_DIR}/stamped_scan_tf_fixture.py" \
  --scan-topic /stamped_scan_tf_test/scan \
  --tf-topic /stamped_scan_tf_test/tf \
  --include-dynamic-tf
wait "${PROBE_PID}" || {
  cat "${SUCCESS_LOG}" >&2
  exit 1
}
PROBE_PID=""
grep -q "original-stamp scan TF ready" "${SUCCESS_LOG}" || {
  cat "${SUCCESS_LOG}" >&2
  exit 1
}

"${PROBE_BIN}" stamped-scan-tf \
  /stamped_scan_tf_test/scan_missing \
  /stamped_scan_tf_test/tf_missing \
  mapping_odom 1.5 2 >"${FAILURE_LOG}" 2>&1 &
PROBE_PID=$!
python3 "${SCRIPT_DIR}/stamped_scan_tf_fixture.py" \
  --scan-topic /stamped_scan_tf_test/scan_missing \
  --tf-topic /stamped_scan_tf_test/tf_missing
set +e
wait "${PROBE_PID}"
failure_rc=$?
set -e
PROBE_PID=""
if [[ "${failure_rc}" -eq 0 ]]; then
  echo "probe incorrectly accepted scans without dynamic TF" >&2
  cat "${FAILURE_LOG}" >&2
  exit 1
fi
grep -q "original-stamp scan TF not ready" "${FAILURE_LOG}" || {
  cat "${FAILURE_LOG}" >&2
  exit 1
}

echo "PASS original-stamp scan TF admission accepts an exact chain and rejects missing dynamic TF"

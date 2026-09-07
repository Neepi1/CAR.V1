#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${NJRH_FLATSCAN_STARTUP_CONTRACT_TARGET:-${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh}"

fail() {
  echo "[flatscan-startup-contract] FAIL $*" >&2
  exit 1
}

function_source="$(sed -n '/^flatscan_startup_rate_confirmed() {$/,/^}$/p' "${TARGET}")"
[[ -n "${function_source}" ]] || fail "flatscan_startup_rate_confirmed function is missing"
eval "${function_source}"

FLATSCAN_STARTUP_RATE_ATTEMPTS=3
FLATSCAN_STARTUP_RATE_RETRY_SEC=0
FLATSCAN_STARTUP_RATE_SAMPLE_SEC=1

probe_index=0
probe_results=(1 0)
flatscan_hz_ok() {
  local result="${probe_results[${probe_index}]:-1}"
  probe_index=$((probe_index + 1))
  return "${result}"
}
sleep() { :; }

flatscan_startup_rate_confirmed || fail "a transient first rate miss must recover on a later confirmation"
[[ "${probe_index}" -eq 2 ]] || fail "expected two rate probes, got ${probe_index}"

probe_index=0
probe_results=(1 1 1)
if flatscan_startup_rate_confirmed; then
  fail "all failed rate probes must remain an unverified startup result"
fi
[[ "${probe_index}" -eq 3 ]] || fail "expected three bounded rate probes, got ${probe_index}"

grep -Fq 'if wait_for_flatscan_ready; then' "${TARGET}" || \
  fail "startup readiness must be guarded under set -e"
grep -Fq 'flatscan_helper_health_state="startup_degraded"' "${TARGET}" || \
  fail "startup failure must enter degraded supervision"
if grep -Fxq 'wait_for_flatscan_ready' "${TARGET}"; then
  fail "unguarded startup readiness call would terminate the supervisor"
fi

echo "[flatscan-startup-contract] PASS"

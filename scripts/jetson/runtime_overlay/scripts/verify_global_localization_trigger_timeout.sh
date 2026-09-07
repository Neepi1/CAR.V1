#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_SCRIPT="${1:-${SCRIPT_DIR}/run_navigation_runtime_services.sh}"

fail() {
  echo "[global-localization-trigger-timeout] FAIL: $*" >&2
  exit 1
}

[[ -f "${RUNTIME_SCRIPT}" ]] || fail "runtime script not found: ${RUNTIME_SCRIPT}"

grep -Fq 'process_timeout_sec=$((this_timeout + process_grace_sec))' "${RUNTIME_SCRIPT}" ||
  fail "per-attempt OS process budget is not derived from the wrapper attempt budget"
grep -Fq 'timeout --signal=TERM --kill-after="${process_kill_after_sec}s" "${process_timeout_sec}s"' "${RUNTIME_SCRIPT}" ||
  fail "global localization trigger is not guarded by TERM plus bounded KILL escalation"
grep -Fq -- '--timeout-sec "${this_timeout}"' "${RUNTIME_SCRIPT}" ||
  fail "the existing wrapper-level timeout argument was changed or removed"

started_at="$(date +%s)"
set +e
{
  timeout --signal=TERM --kill-after=1s 1s \
    bash -c 'trap "" TERM; sleep 30'
} >/dev/null 2>&1
timeout_rc=$?
set -e
elapsed_sec=$(( $(date +%s) - started_at ))

[[ "${timeout_rc}" -ne 0 ]] ||
  fail "OS timeout unexpectedly reported success for a TERM-resistant process"
(( elapsed_sec <= 5 )) ||
  fail "OS timeout did not force-kill the TERM-resistant process: elapsed=${elapsed_sec}s"

echo "[global-localization-trigger-timeout] PASS: production call is bounded and TERM-resistant children are force-killed"

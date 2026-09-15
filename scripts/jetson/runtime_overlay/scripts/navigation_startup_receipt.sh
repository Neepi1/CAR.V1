#!/usr/bin/env bash
# One-shot scheduling receipt, private to this common-start invocation. This is
# not runtime readiness, a lease, or permission to navigate. No ROS calls.
report_navigation_startup_finished() {
  case "$1" in
    ready|reused|waiting_for_localization)
      if declare -F njrh_finish_startup_cpu_boost >/dev/null; then
        njrh_finish_startup_cpu_boost "$1"
      fi
      ;;
  esac
  [[ -n "${NJRH_NAVIGATION_STARTUP_RECEIPT:-}" ]] || return 0
  local temporary="${NJRH_NAVIGATION_STARTUP_RECEIPT}.${BASHPID}.tmp"
  if printf '%s\n' "$1" > "${temporary}" &&
    mv -f "${temporary}" "${NJRH_NAVIGATION_STARTUP_RECEIPT}"; then
    echo "[runtime-overlay] navigation initialization finished: $1" >&2
  else
    echo "[runtime-overlay] cannot write navigation initialization receipt" >&2
    rm -f "${temporary}" || true
    # Scheduling telemetry must not turn a healthy navigation process into a
    # new fatal startup failure. The common owner leaves docking deferred.
    return 0
  fi
}

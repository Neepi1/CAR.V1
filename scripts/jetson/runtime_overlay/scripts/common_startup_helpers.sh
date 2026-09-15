#!/usr/bin/env bash
# Private common-runtime startup jobs. Producer PIDs stay registered in the
# parent; only the existing bounded readiness checks run in child shells.
# No ROS participant, persistent lease, restart policy, or additional admission.
declare -A common_startup_waiters=()
declare -A common_startup_producers=()

common_navigation_initialization_finished() {
  [[ "${resident_navigation_autostart_started:-0}" == 1 ]] || return 0
  local phase=""
  if [[ -s "${NJRH_NAVIGATION_STARTUP_RECEIPT:-}" ]]; then
    read -r phase < "${NJRH_NAVIGATION_STARTUP_RECEIPT}" || true
    case "${phase}" in
      ready|waiting_for_localization|reused) return 0 ;;
    esac
  fi
  # A reused process needs no new cold start. An exited owned startup has also
  # finished, unsuccessfully; its actual failure remains in the nav context.
  [[ -n "${resident_navigation_autostart_pid:-}" ]] || return 0
  ! common_startup_child_running "${resident_navigation_autostart_pid}"
}

common_api_http_ready() {
  # A short metadata GET, only during startup. No ROS graph probe or mutation;
  # credentials stay in the environment, never in argv/logs.
  python3 - <<'PY'
import http.client
import json
import os
connection = http.client.HTTPConnection(
    "127.0.0.1", int(os.environ.get("ROBOT_API_SERVER_PORT", "8080")), timeout=1.0)
try:
    connection.request("GET", "/api/v1/openapi",
                       headers={"X-Robot-Token": os.environ.get("ROBOT_API_TOKEN", "")})
    response = connection.getresponse()
    payload = json.loads(response.read(65537))
    ready = (response.status == 200 and payload.get("ok") is True and
             "GET /api/v1/navigation/state" in payload.get("endpoints", []))
except Exception:
    ready = False
finally:
    connection.close()
raise SystemExit(0 if ready else 1)
PY
}

common_startup_child_running() {
  local candidate
  # The parent's live job table also excludes an exited child's recycled PID.
  while read -r candidate; do
    [[ "${candidate}" != "$1" ]] || return 0
  done < <(jobs -pr)
  return 1
}

start_common_canonical_helper_background() {
  local name="$1"
  [[ -z "${common_startup_waiters[${name}]:-}" ]] || return 0
  if [[ -n "${common_startup_producers[${name}]:-}" ]]; then
    canonical_helper_launched_pid="${common_startup_producers[${name}]}"
    common_startup_child_running "${canonical_helper_launched_pid}"
    return $?
  fi
  launch_canonical_helper "$@" || return $?
  local producer_pid="${canonical_helper_launched_pid}"
  [[ -n "${producer_pid}" ]] || return 0
  common_startup_producers["${name}"]="${producer_pid}"
  complete_canonical_helper_start "${name}" "${producer_pid}" &
  common_startup_waiters["${name}"]=$!
}

start_common_startup_check() {
  local name="$1"
  shift
  [[ -z "${common_startup_waiters[${name}]:-}" ]] || return 0
  "$@" &
  common_startup_waiters["${name}"]=$!
}

wait_for_common_startup_job() {
  local name="$1"
  local waiter_pid="${common_startup_waiters[${name}]:-}"
  [[ -n "${waiter_pid}" ]] || return 0
  local rc=0
  wait "${waiter_pid}" || rc=$?
  unset 'common_startup_waiters['"${name}"']'
  if (( rc == 0 )); then
    local producer_pid="${common_startup_producers[${name}]:-}"
    if [[ -z "${producer_pid}" ]] || common_startup_child_running "${producer_pid}"; then
      return 0
    fi
  fi
  echo "[runtime-overlay] common startup failed: ${name}, readiness_exit=${rc}" >&2
  return 1
}

cleanup_common_startup_helpers() {
  local pid name descendants
  # Cancel readiness subprocesses before producers. Capture descendants before
  # signalling their parent so a blocked readiness client cannot be orphaned.
  for name in "${!common_startup_waiters[@]}"; do
    pid="${common_startup_waiters[${name}]}"
    if ! common_startup_child_running "${pid}"; then
      wait "${pid}" 2>/dev/null || true
      continue
    fi
    descendants="$(canonical_descendant_pids "${pid}")"
    kill -TERM ${descendants} "${pid}" 2>/dev/null || true
    canonical_wait_for_pid_exit "${pid}" 5 || true
    [[ -z "${descendants}" ]] || kill -KILL ${descendants} 2>/dev/null || true
    kill -KILL "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
  common_startup_waiters=()
  for name in "${!common_startup_producers[@]}"; do
    pid="${common_startup_producers[${name}]}"
    if common_startup_child_running "${pid}"; then
      terminate_canonical_helper_pid "${name}" "${pid}" 0 5
    else
      wait "${pid}" 2>/dev/null || true
    fi
    forget_canonical_helper_pid "${pid}"
  done
  common_startup_producers=()
}

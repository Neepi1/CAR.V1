#!/usr/bin/env bash

# Canonical /scan has exactly one publisher at a time:
#   navigation -> pointcloud_accel_axis_node
#   mapping    -> pointcloud_to_laserscan (FAST-LIO2 corrected cloud slice)
# This helper performs the graph-proven handoff.  It never stops the resident
# pointcloud trunk, so /lidar_points continues feeding FAST-LIO2.

RESIDENT_SCAN_OWNER_NODE="${NJRH_RESIDENT_SCAN_OWNER_NODE:-pointcloud_accel_axis_node}"
RESIDENT_SCAN_CONTROL_SERVICE="${NJRH_RESIDENT_SCAN_CONTROL_SERVICE:-/pointcloud_accel_axis_node/set_scan_output_enabled}"
SCAN_OWNERSHIP_TIMEOUT_SEC="${NJRH_SCAN_OWNERSHIP_TIMEOUT_SEC:-12}"
SCAN_OWNER_OBSERVER_PID=""
SCAN_OWNER_OBSERVER_LOG=""

wait_for_scan_publisher_count() {
  local topic="$1"
  local expected_count="$2"
  local timeout_sec="${3:-${SCAN_OWNERSHIP_TIMEOUT_SEC}}"
  runtime_readiness_probe publisher-count "${topic}" "${expected_count}" "${timeout_sec}"
}

wait_for_scan_owner() {
  local topic="$1"
  local owner_node="$2"
  local expected_count="$3"
  local timeout_sec="${4:-${SCAN_OWNERSHIP_TIMEOUT_SEC}}"
  runtime_readiness_probe exact-publisher-owner \
    "${topic}" "${owner_node}" "${expected_count}" "${timeout_sec}"
}

stop_scan_owner_observer() {
  local pid="${SCAN_OWNER_OBSERVER_PID:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
  SCAN_OWNER_OBSERVER_PID=""
  if [[ -n "${SCAN_OWNER_OBSERVER_LOG:-}" ]]; then
    rm -f "${SCAN_OWNER_OBSERVER_LOG}" 2>/dev/null || true
  fi
  SCAN_OWNER_OBSERVER_LOG=""
}

start_scan_owner_observer() {
  local topic="$1"
  local owner_node="$2"
  local expected_count="$3"
  local timeout_sec="${4:-${SCAN_OWNERSHIP_TIMEOUT_SEC}}"

  stop_scan_owner_observer
  SCAN_OWNER_OBSERVER_LOG="/tmp/njrh_scan_owner_observer_${BASHPID:-$$}.log"
  runtime_readiness_probe exact-publisher-owner \
    "${topic}" "${owner_node}" "${expected_count}" "${timeout_sec}" \
    >"${SCAN_OWNER_OBSERVER_LOG}" 2>&1 &
  SCAN_OWNER_OBSERVER_PID=$!
  echo "[runtime-overlay] pre-armed scan owner observer pid=${SCAN_OWNER_OBSERVER_PID} topic=${topic} owner=${owner_node} timeout_sec=${timeout_sec}" >&2
}

wait_for_scan_owner_observer() {
  local pid="${SCAN_OWNER_OBSERVER_PID:-}"
  local log_file="${SCAN_OWNER_OBSERVER_LOG:-}"
  local rc=1

  if [[ -z "${pid}" ]]; then
    echo "[runtime-overlay] scan owner observer was not started" >&2
    return 1
  fi
  set +e
  wait "${pid}"
  rc=$?
  set -e
  [[ -z "${log_file}" || ! -f "${log_file}" ]] || cat "${log_file}" >&2
  SCAN_OWNER_OBSERVER_PID=""
  [[ -z "${log_file}" ]] || rm -f "${log_file}" 2>/dev/null || true
  SCAN_OWNER_OBSERVER_LOG=""
  return "${rc}"
}

set_resident_scan_output() {
  local enabled="$1"
  local response

  runtime_readiness_probe service "${RESIDENT_SCAN_CONTROL_SERVICE}" "${SCAN_OWNERSHIP_TIMEOUT_SEC}" || {
    echo "[runtime-overlay] resident scan ownership service is unavailable: ${RESIDENT_SCAN_CONTROL_SERVICE}" >&2
    return 1
  }
  if ! response="$(timeout --kill-after=2 "${SCAN_OWNERSHIP_TIMEOUT_SEC}" \
    ros2 service call \
      "${RESIDENT_SCAN_CONTROL_SERVICE}" \
      std_srvs/srv/SetBool \
      "{data: ${enabled}}" 2>&1)"; then
    echo "[runtime-overlay] resident scan ownership request outcome is unknown enabled=${enabled}: ${response}" >&2
    if resident_scan_output_matches_requested_state "${enabled}"; then
      echo "[runtime-overlay] resident scan ownership reached requested state after uncertain service result enabled=${enabled}" >&2
      return 0
    fi
    return 1
  fi
  if ! grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${response}"; then
    echo "[runtime-overlay] resident scan ownership request was rejected enabled=${enabled}: ${response}" >&2
    return 1
  fi
  echo "[runtime-overlay] resident scan output enabled=${enabled} service=${RESIDENT_SCAN_CONTROL_SERVICE}" >&2
}

resident_scan_output_matches_requested_state() {
  local enabled="$1"
  local topic="${2:-/scan}"
  case "${enabled}" in
    true|TRUE|1)
      wait_for_scan_owner \
        "${topic}" "${RESIDENT_SCAN_OWNER_NODE}" 1 "${SCAN_OWNERSHIP_TIMEOUT_SEC}"
      ;;
    false|FALSE|0)
      wait_for_scan_publisher_count \
        "${topic}" 0 "${SCAN_OWNERSHIP_TIMEOUT_SEC}"
      ;;
    *)
      echo "[runtime-overlay] invalid resident scan requested state: ${enabled}" >&2
      return 2
      ;;
  esac
}

restore_navigation_scan_owner() {
  local topic="${1:-/scan}"
  # Fast idempotent check first. During a completed mapping teardown the graph
  # is already proven empty, so do not spend the full ownership timeout waiting
  # for an owner that intentionally does not exist.
  if wait_for_scan_owner \
      "${topic}" "${RESIDENT_SCAN_OWNER_NODE}" 1 1; then
    return 0
  fi

  if wait_for_scan_publisher_count "${topic}" 0 1; then
    set_resident_scan_output true || return 1
    wait_for_scan_owner "${topic}" "${RESIDENT_SCAN_OWNER_NODE}" 1 || {
      echo "[runtime-overlay] canonical navigation scan owner was not restored on ${topic}" >&2
      return 1
    }
    return 0
  fi

  # A discovered endpoint can temporarily have UNKNOWN node metadata under
  # Fast DDS load. Keep one continuous full-timeout proof before deciding that
  # a different publisher owns the canonical topic.
  if wait_for_scan_owner \
      "${topic}" "${RESIDENT_SCAN_OWNER_NODE}" 1 "${SCAN_OWNERSHIP_TIMEOUT_SEC}"; then
    return 0
  fi
  if ! wait_for_scan_publisher_count "${topic}" 0 1; then
    echo "[runtime-overlay] refusing navigation scan restore: another publisher still owns ${topic}" >&2
    return 1
  fi
  set_resident_scan_output true || return 1
  wait_for_scan_owner "${topic}" "${RESIDENT_SCAN_OWNER_NODE}" 1 || {
    echo "[runtime-overlay] canonical navigation scan owner was not restored on ${topic}" >&2
    return 1
  }
}

#!/usr/bin/env bash

runtime_health_file() {
  printf '%s\n' "${NJRH_RUNTIME_HEALTH_FILE:-/tmp/njrh_runtime_health.json}"
}

runtime_health_max_age_sec() {
  printf '%s\n' "${NJRH_RUNTIME_HEALTH_MAX_AGE_SEC:-2.0}"
}

runtime_health_query() {
  local root="${PROJECT_ROOT:-}"
  if [[ -z "${root}" ]]; then
    root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
  fi
  local binary="${NJRH_RUNTIME_HEALTH_CHECK_BIN:-${root}/install/robot_bringup/lib/robot_bringup/runtime_health_check}"
  if [[ ! -x "${binary}" ]]; then
    echo "status=observer_unavailable detail=missing_cpp_health_check" >&2
    return 40
  fi
  "${binary}" "$(runtime_health_file)" "$(runtime_health_max_age_sec)" "$@"
}

runtime_health_available() { runtime_health_query available; }
runtime_health_check() { runtime_health_query check "$1"; }
# 40-49: observer failure, never authorizes recovery. 50-59: independently confirm.
runtime_health_local_state_diagnostic() {
  runtime_health_query diagnostic "${NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC:-0.75}"
}
runtime_health_topic_message_ready() { runtime_health_query topic "$1"; }
runtime_health_fresh_tf_ready() { runtime_health_query tf "$1" "$2" "$3"; }
runtime_health_tf_seen() {
  runtime_health_query tf "$1" "$2" "${NJRH_RUNTIME_HEALTH_TF_SEEN_MAX_AGE_SEC:-5.0}"
}

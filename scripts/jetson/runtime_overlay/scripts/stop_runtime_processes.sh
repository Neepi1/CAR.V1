#!/usr/bin/env bash
set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/runtime_process_patterns.sh"

cleanup_shell_pid="$$"
cleanup_parent_pid="${PPID}"

pids_by_pattern() {
  local pattern="$1"
  ps -eo pid=,args= \
    | awk \
      -v pattern="${pattern}" \
      -v cleanup_shell_pid="${cleanup_shell_pid}" \
      -v cleanup_parent_pid="${cleanup_parent_pid}" '
        $0 ~ pattern &&
        $1 != cleanup_shell_pid &&
        $1 != cleanup_parent_pid &&
        $0 !~ /awk -v pattern/ &&
        $0 !~ /pids_by_pattern/ &&
        $0 !~ /stop_exact_process_set/ &&
        $0 !~ /stop_runtime_processes[.]sh/ {print $1}
      '
}

wait_pids_gone() {
  local timeout_sec="$1"
  shift || true
  local pids=("$@")
  local deadline=$((SECONDS + timeout_sec))
  local pid
  while (( SECONDS < deadline )); do
    local alive=0
    for pid in "${pids[@]}"; do
      [[ -n "${pid}" && -d "/proc/${pid}" ]] && alive=1
    done
    [[ "${alive}" -eq 0 ]] && return 0
    sleep 0.2
  done
  return 1
}

stop_exact_process_set() {
  local label="$1"
  local pattern="$2"
  local pids=()
  mapfile -t pids < <(pids_by_pattern "${pattern}")
  [[ "${#pids[@]}" -gt 0 ]] || return 0
  echo "[njrh-systemd] stopping ${label} pids=${pids[*]}" >&2
  kill -INT "${pids[@]}" 2>/dev/null || true
  wait_pids_gone 2 "${pids[@]}" && return 0

  mapfile -t pids < <(pids_by_pattern "${pattern}")
  [[ "${#pids[@]}" -gt 0 ]] || return 0
  kill -TERM "${pids[@]}" 2>/dev/null || true
  wait_pids_gone 3 "${pids[@]}" && return 0

  mapfile -t pids < <(pids_by_pattern "${pattern}")
  [[ "${#pids[@]}" -gt 0 ]] || return 0
  echo "[njrh-systemd] killing exact stale ${label} pids=${pids[*]}" >&2
  kill -KILL "${pids[@]}" 2>/dev/null || true
  wait_pids_gone 2 "${pids[@]}" || true

  mapfile -t pids < <(pids_by_pattern "${pattern}")
  if [[ "${#pids[@]}" -gt 0 ]]; then
    echo "[njrh-systemd] failed to stop ${label}; remaining pids=${pids[*]}" >&2
    return 1
  fi
  return 0
}

remaining_runtime_pids() {
  pids_by_pattern "${NJRH_RUNTIME_ALL_PATTERN}"
}

if [[ "${1:-}" == "--check" ]]; then
  mapfile -t remaining < <(remaining_runtime_pids)
  if [[ "${#remaining[@]}" -gt 0 ]]; then
    echo "[njrh-systemd] runtime processes remain: ${remaining[*]}" >&2
    exit 1
  fi
  exit 0
fi

cleanup_failed=0
stop_exact_process_set \
  "stale ros2 diagnostics cli" \
  "${NJRH_RUNTIME_ROS2_CLI_PATTERN}" || cleanup_failed=1
stop_exact_process_set \
  "common services" \
  "${NJRH_RUNTIME_COMMON_PATTERN}" || cleanup_failed=1
stop_exact_process_set \
  "runtime nodes" \
  "${NJRH_RUNTIME_NODE_PATTERN}" || cleanup_failed=1

mapfile -t remaining < <(remaining_runtime_pids)
if [[ "${#remaining[@]}" -gt 0 ]]; then
  echo "[njrh-systemd] runtime cleanup left residual pids=${remaining[*]}" >&2
  cleanup_failed=1
fi

rm -f \
  /tmp/njrh_runtime_map_context.json \
  /tmp/njrh_runtime_health.json \
  /tmp/njrh_amcl_runtime_status.env \
  /tmp/njrh_nav2_launch_hold_ready.env \
  /tmp/njrh_nav2_lifecycle_ready.env \
  2>/dev/null || true

exit "${cleanup_failed}"

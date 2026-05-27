#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESTART_DELAY_SEC="${ROBOT_API_SERVER_RESTART_DELAY_SEC:-2}"

child_pid=""
stopping=0

stop_child() {
  stopping=1
  if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
    kill -INT "${child_pid}" 2>/dev/null || true
    sleep 1
    if kill -0 "${child_pid}" 2>/dev/null; then
      kill -TERM "${child_pid}" 2>/dev/null || true
    fi
    wait "${child_pid}" 2>/dev/null || true
  fi
}

trap stop_child EXIT
trap 'stop_child; exit 130' INT TERM

echo "[runtime-overlay] robot_api_server supervisor starting" >&2
while true; do
  child_pid=""
  bash "${SCRIPT_DIR}/run_robot_api_server.sh" &
  child_pid=$!
  set +e
  wait "${child_pid}"
  status=$?
  set -e
  child_pid=""

  if [[ "${stopping}" -eq 1 ]]; then
    exit "${status}"
  fi

  echo "[runtime-overlay] robot_api_server exited with ${status}; restarting in ${RESTART_DELAY_SEC}s" >&2
  sleep "${RESTART_DELAY_SEC}"
done

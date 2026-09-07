#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
child_pid=""
stopping=0

cleanup_stale_api_processes() {
  local pattern
  for pattern in \
    "/install/robot_api_server/lib/robot_api_server/robot_api_server_node" \
    "robot_api_server_node --ros-args" \
    "ros2 run robot_api_server robot_api_server_node"
  do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
  for pattern in \
    "/install/robot_api_server/lib/robot_api_server/robot_api_server_node" \
    "robot_api_server_node --ros-args" \
    "ros2 run robot_api_server robot_api_server_node"
  do
    pkill -KILL -f "${pattern}" 2>/dev/null || true
  done
}

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
child_pid=""
cleanup_stale_api_processes
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

# Never restart the API process in isolation. It owns in-memory admission
# evidence for ROS requests whose client-side wait timed out while Nav2,
# localization, docking, or floor services may still execute. Losing that
# evidence while those servers remain resident could admit overlapping
# motion. Exiting the supervisor lets run_common_services fail and systemd
# restart the complete runtime chain.
echo "[runtime-overlay] robot_api_server exited with ${status}; refusing isolated restart so systemd restarts the complete runtime chain" >&2
# Wake the common owner immediately instead of waiting for its health period.
# Its TERM trap latches safety stop first and then tears down every runtime
# motion publisher before this supervisor exits.
kill -TERM "${PPID}" 2>/dev/null || true
if [[ "${status}" -eq 0 ]]; then
  exit 1
fi
exit "${status}"

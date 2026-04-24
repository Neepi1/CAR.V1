#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

CAN_IFACE="${CAN_IFACE:-can0}"
export PUBLISH_ODOM_TF="${PUBLISH_ODOM_TF:-false}"
export ODOM_TOPIC="${ODOM_TOPIC:-/wheel/odom}"
export ODOM_FRAME="${ODOM_FRAME:-odom}"
export BASE_FRAME="${BASE_FRAME:-base_link}"
ROBOT_MODEL="${ROBOT_MODEL:-ranger_mini_v3}"
LOCK_DIR="/tmp/njrh_ranger_chassis_${CAN_IFACE}.lock"
REAL_NODE_PATTERN="/ranger_base/lib/ranger_base/ranger_base_node.*port_name:=${CAN_IFACE}"
WRAPPER_PATTERN="ros2 run ranger_base ranger_base_node.*port_name:=${CAN_IFACE}"

monitor_existing_ranger() {
  echo "[runtime-overlay] ranger_base_node already owns ${CAN_IFACE}; monitoring existing instance instead of starting a duplicate" >&2
  while pgrep -f "${REAL_NODE_PATTERN}" >/dev/null || pgrep -f "${WRAPPER_PATTERN}" >/dev/null; do
    sleep 2
  done
  echo "[runtime-overlay] existing ranger_base_node for ${CAN_IFACE} exited" >&2
}

if ! mkdir "${LOCK_DIR}" 2>/dev/null; then
  if pgrep -f "${REAL_NODE_PATTERN}" >/dev/null || pgrep -f "${WRAPPER_PATTERN}" >/dev/null; then
    monitor_existing_ranger
    exit 0
  fi
  echo "[runtime-overlay] removing stale Ranger chassis lock: ${LOCK_DIR}" >&2
  rm -rf "${LOCK_DIR}"
  mkdir "${LOCK_DIR}" 2>/dev/null || {
    echo "[runtime-overlay] failed to acquire Ranger chassis lock: ${LOCK_DIR}" >&2
    exit 1
  }
fi

cleanup_lock() {
  rm -rf "${LOCK_DIR}" 2>/dev/null || true
}
chassis_pid=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${chassis_pid}" ]]; then
    kill -INT "${chassis_pid}" 2>/dev/null || true
    wait "${chassis_pid}" 2>/dev/null || true
  fi
  cleanup_lock
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

ros2 run ranger_base ranger_base_node \
  --ros-args \
  -p use_sim_time:=false \
  -p "port_name:=${CAN_IFACE}" \
  -p "odom_frame:=${ODOM_FRAME}" \
  -p "base_frame:=${BASE_FRAME}" \
  -p "odom_topic_name:=${ODOM_TOPIC}" \
  -p simulated_robot:=false \
  -p "publish_odom_tf:=${PUBLISH_ODOM_TF}" \
  -p "robot_model:=${ROBOT_MODEL}" \
  -r /tf:=/tf_ranger_internal \
  -r /tf_static:=/tf_static_ranger_internal &
chassis_pid=$!
wait "${chassis_pid}"

#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
CAN_WAIT_TIMEOUT_SEC="${CAN_WAIT_TIMEOUT_SEC:-120}"
UPSTREAM_WORKSPACE_HOST="${NJRH_UPSTREAM_WORKSPACE_HOST:-/home/nvidia/workspaces/isaac_ros-dev}"
CAN_BRINGUP_SCRIPT="${CAN_BRINGUP_SCRIPT:-${UPSTREAM_WORKSPACE_HOST}/scripts/bringup_ranger_can_host.sh}"

deadline=$((SECONDS + CAN_WAIT_TIMEOUT_SEC))
while [[ ! -d "/sys/class/net/${CAN_IFACE}" ]]; do
  if (( SECONDS >= deadline )); then
    echo "[njrh-can] CAN interface ${CAN_IFACE} did not appear within ${CAN_WAIT_TIMEOUT_SEC}s" >&2
    exit 1
  fi
  sleep 1
done

if [[ ! -x "${CAN_BRINGUP_SCRIPT}" ]]; then
  echo "[njrh-can] missing executable CAN bringup script: ${CAN_BRINGUP_SCRIPT}" >&2
  exit 1
fi

CAN_IFACE="${CAN_IFACE}" CAN_BITRATE="${CAN_BITRATE}" bash "${CAN_BRINGUP_SCRIPT}"

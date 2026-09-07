#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
CAN_WAIT_TIMEOUT_SEC="${CAN_WAIT_TIMEOUT_SEC:-120}"
WORKSPACE_HOST="${NJRH_WORKSPACE_HOST:-/home/nvidia/workspaces/njrh-v3/workspace1}"
CAN_BRINGUP_SCRIPT="${CAN_BRINGUP_SCRIPT:-${WORKSPACE_HOST}/scripts/jetson/runtime_overlay/scripts/bringup_ranger_can.sh}"

deadline=$((SECONDS + CAN_WAIT_TIMEOUT_SEC))
while [[ ! -d "/sys/class/net/${CAN_IFACE}" ]]; do
  if (( SECONDS >= deadline )); then
    echo "[njrh-can] CAN interface ${CAN_IFACE} did not appear within ${CAN_WAIT_TIMEOUT_SEC}s" >&2
    exit 1
  fi
  sleep 1
done

if [[ ! -f "${CAN_BRINGUP_SCRIPT}" || ! -r "${CAN_BRINGUP_SCRIPT}" ]]; then
  echo "[njrh-can] missing readable CAN bringup script: ${CAN_BRINGUP_SCRIPT}" >&2
  exit 1
fi

CAN_IFACE="${CAN_IFACE}" CAN_BITRATE="${CAN_BITRATE}" bash "${CAN_BRINGUP_SCRIPT}"

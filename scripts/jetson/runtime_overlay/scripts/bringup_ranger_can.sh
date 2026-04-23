#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"

IP_CMD=()
for candidate in /usr/sbin/ip /sbin/ip /usr/bin/ip /bin/ip; do
  if [[ -x "${candidate}" ]]; then
    IP_CMD=("${candidate}")
    break
  fi
done

if [[ ${#IP_CMD[@]} -eq 0 ]] && command -v ip >/dev/null 2>&1; then
  IP_CMD=("$(command -v ip)")
fi

if [[ ${#IP_CMD[@]} -eq 0 ]] && [[ -x /proc/1/root/lib/ld-linux-aarch64.so.1 ]]; then
  for candidate in /proc/1/root/usr/sbin/ip /proc/1/root/usr/bin/ip /proc/1/root/sbin/ip /proc/1/root/bin/ip; do
    if [[ -x "${candidate}" ]]; then
      IP_CMD=(
        /proc/1/root/lib/ld-linux-aarch64.so.1
        --library-path
        /proc/1/root/lib/aarch64-linux-gnu:/proc/1/root/usr/lib/aarch64-linux-gnu
        "${candidate}"
      )
      break
    fi
  done
fi

if [[ ${#IP_CMD[@]} -eq 0 ]]; then
  echo "ip command is not available in the current container." >&2
  exit 1
fi

if [[ ! -d "/sys/class/net/${CAN_IFACE}" ]]; then
  echo "CAN interface ${CAN_IFACE} is not visible in the current container." >&2
  exit 1
fi

"${IP_CMD[@]}" link set "${CAN_IFACE}" down 2>/dev/null || true
"${IP_CMD[@]}" link set "${CAN_IFACE}" up type can bitrate "${CAN_BITRATE}"
"${IP_CMD[@]}" -details link show "${CAN_IFACE}"

#!/usr/bin/env bash
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-njrh-gs2.rules"
RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="gs2", GROUP="dialout", MODE="0666"'

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo bash scripts/jetson/install_gs2_udev.sh" >&2
  exit 1
fi

printf '%s\n' "${RULE}" > "${RULE_FILE}"
usermod -aG dialout nvidia 2>/dev/null || true
udevadm control --reload-rules
udevadm trigger

current_device="$(readlink -f /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 2>/dev/null || true)"
if [[ -n "${current_device}" && -e "${current_device}" ]]; then
  ln -sf "${current_device}" /dev/gs2
  chmod 0666 "${current_device}"
  echo "[gs2-udev] current GS2 adapter: /dev/gs2 -> ${current_device}"
fi

echo "[gs2-udev] installed ${RULE_FILE}"
echo "[gs2-udev] unplug/replug the GS2 USB adapter if /dev/gs2 does not appear immediately"

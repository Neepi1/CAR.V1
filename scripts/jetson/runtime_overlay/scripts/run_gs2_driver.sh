#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

if [[ -f "${NJRH_PROJECT_ROOT}/install/setup.bash" ]]; then
  # GS2 is a repository-owned package, not part of the upstream Isaac workspace.
  set +u
  source "${NJRH_PROJECT_ROOT}/install/setup.bash"
  set -u
else
  echo "[runtime-overlay] project install missing: ${NJRH_PROJECT_ROOT}/install/setup.bash" >&2
  echo "[runtime-overlay] build robot_eai_gs2 before starting the GS2 driver." >&2
  exit 1
fi

SERIAL_PORT="${NJRH_GS2_SERIAL_PORT:-/dev/gs2}"
SERIAL_BAUDRATE="${NJRH_GS2_SERIAL_BAUDRATE:-921600}"
FRAME_ID="${NJRH_GS2_FRAME_ID:-gs2_link}"
SCAN_TOPIC="${NJRH_GS2_SCAN_TOPIC:-/dock/gs2_scan}"
POINT_CLOUD_TOPIC="${NJRH_GS2_POINTS_TOPIC:-/dock/gs2_points}"
CONFIG_FILE="${NJRH_GS2_CONFIG_FILE:-}"

if [[ ! -e "${SERIAL_PORT}" && "${SERIAL_PORT}" == "/dev/gs2" ]]; then
  if compgen -G "/dev/serial/by-id/*CP2102*" >/dev/null; then
    SERIAL_PORT="$(readlink -f /dev/serial/by-id/*CP2102* | head -n 1)"
    echo "[runtime-overlay] /dev/gs2 missing inside container; using CP2102 alias ${SERIAL_PORT}" >&2
  elif [[ -e "/dev/ttyUSB0" ]]; then
    SERIAL_PORT="/dev/ttyUSB0"
    echo "[runtime-overlay] /dev/gs2 missing inside container; using current GS2 tty fallback ${SERIAL_PORT}" >&2
  fi
fi

if [[ ! -e "${SERIAL_PORT}" ]]; then
  echo "[runtime-overlay] GS2 serial device does not exist in container: ${SERIAL_PORT}" >&2
  echo "[runtime-overlay] check /dev/gs2 udev alias or set NJRH_GS2_SERIAL_PORT to the host-resolved ttyUSB device." >&2
  exit 1
fi

if [[ ! -r "${SERIAL_PORT}" || ! -w "${SERIAL_PORT}" ]]; then
  echo "[runtime-overlay] GS2 serial device is not readable/writable by $(id -un): ${SERIAL_PORT}" >&2
  ls -l "${SERIAL_PORT}" >&2 || true
  exit 1
fi

args=(
  serial_port:="${SERIAL_PORT}"
  serial_baudrate:="${SERIAL_BAUDRATE}"
  frame_id:="${FRAME_ID}"
  scan_topic:="${SCAN_TOPIC}"
  point_cloud_topic:="${POINT_CLOUD_TOPIC}"
)

if [[ -n "${CONFIG_FILE}" ]]; then
  [[ -f "${CONFIG_FILE}" ]] || {
    echo "[runtime-overlay] GS2 config missing: ${CONFIG_FILE}" >&2
    exit 1
  }
  args+=(config_file:="${CONFIG_FILE}")
fi

echo "[runtime-overlay] starting GS2 driver on ${SERIAL_PORT} @ ${SERIAL_BAUDRATE}, frame=${FRAME_ID}, scan=${SCAN_TOPIC}" >&2
ros2 launch robot_eai_gs2 gs2.launch.py "${args[@]}"

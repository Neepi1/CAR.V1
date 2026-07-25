#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

OWNER_LOCK_FILE="${NJRH_ORBBEC_CAMERA_OWNER_LOCK_FILE:-/tmp/njrh_orbbec_336l_depth.lock}"
exec 9>"${OWNER_LOCK_FILE}"
if ! flock -n 9; then
  echo "[runtime-overlay] Orbbec 336L depth owner already running; refusing duplicate driver" >&2
  exit 73
fi

set +u
source /opt/ros/humble/setup.bash
set -u

CAMERA_NAME="${NJRH_ORBBEC_CAMERA_NAME:-camera336l}"
SERIAL_NUMBER="${NJRH_ORBBEC_SERIAL_NUMBER:-CPC8563000LM}"
CONNECTION_DELAY_SEC="${NJRH_ORBBEC_CONNECTION_DELAY_SEC:-1}"
ENABLE_HEARTBEAT="${NJRH_ORBBEC_ENABLE_HEARTBEAT:-false}"
UVC_BACKEND="${NJRH_ORBBEC_UVC_BACKEND:-libuvc}"
DIAGNOSTIC_PERIOD_SEC="${NJRH_ORBBEC_DIAGNOSTIC_PERIOD_SEC:-1.0}"
SDK_CONFIG_PREPARER="${SCRIPT_DIR}/prepare_orbbec_336l_sdk_config.py"
SDK_OVERLAY_PREFIX="${NJRH_ORBBEC_SDK_OVERLAY_PREFIX:-/tmp/njrh_orbbec_336l_ament_overlay}"
PRESET_CONFIG_FILE="${NJRH_ORBBEC_PRESET_CONFIG_FILE:-${SCRIPT_DIR}/../config/orbbec_depth_preset.env}"
AMR_PRESET_NAME="G336X AMR Default"
AMR_PRESET_FILE="${SCRIPT_DIR}/../config/orbbec_presets/G336X_AMR_Default_v0.0.5.bin"
AMR_PRESET_SHA256="db8f907e14380207b20b9f08018ac9a1aa2597894e234154141e2186fa355e3e"

if [[ ! -f "${PRESET_CONFIG_FILE}" ]]; then
  echo "[runtime-overlay] Orbbec preset config is missing: ${PRESET_CONFIG_FILE}" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${PRESET_CONFIG_FILE}"

DEVICE_PRESET="${NJRH_ORBBEC_DEVICE_PRESET:-${AMR_PRESET_NAME}}"
PRESET_FIRMWARE_PATH="${NJRH_ORBBEC_PRESET_FIRMWARE_PATH:-}"
DEPTH_WIDTH="${NJRH_ORBBEC_DEPTH_WIDTH:-848}"
DEPTH_HEIGHT="${NJRH_ORBBEC_DEPTH_HEIGHT:-480}"
DEPTH_FPS="${NJRH_ORBBEC_DEPTH_FPS:-30}"
if [[ -z "${PRESET_FIRMWARE_PATH}" && "${DEVICE_PRESET}" == "${AMR_PRESET_NAME}" ]]; then
  PRESET_FIRMWARE_PATH="${AMR_PRESET_FILE}"
fi

PRESET_LAUNCH_ARGS=()
if [[ -n "${PRESET_FIRMWARE_PATH}" ]]; then
  if [[ ! -f "${PRESET_FIRMWARE_PATH}" ]]; then
    echo "[runtime-overlay] Orbbec preset firmware is missing: ${PRESET_FIRMWARE_PATH}" >&2
    exit 1
  fi
  if [[ "${PRESET_FIRMWARE_PATH}" == "${AMR_PRESET_FILE}" ]]; then
    actual_preset_sha256="$(sha256sum "${PRESET_FIRMWARE_PATH}")"
    actual_preset_sha256="${actual_preset_sha256%% *}"
    if [[ "${actual_preset_sha256}" != "${AMR_PRESET_SHA256}" ]]; then
      echo "[runtime-overlay] Orbbec AMR preset checksum mismatch: ${actual_preset_sha256}" >&2
      exit 1
    fi
  fi
  PRESET_LAUNCH_ARGS+=(preset_firmware_path:="${PRESET_FIRMWARE_PATH}")
fi
PRESET_LAUNCH_ARGS+=(device_preset:="${DEVICE_PRESET}")

if ! ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
  echo "[runtime-overlay] orbbec_camera is not installed in the container" >&2
  exit 1
fi
ORBBEC_PACKAGE_PREFIX="$(ros2 pkg prefix orbbec_camera)"
python3 "${SDK_CONFIG_PREPARER}" \
  --package-prefix "${ORBBEC_PACKAGE_PREFIX}" \
  --overlay-prefix "${SDK_OVERLAY_PREFIX}"
export AMENT_PREFIX_PATH="${SDK_OVERLAY_PREFIX}:${AMENT_PREFIX_PATH}"

echo "[runtime-overlay] starting Orbbec 336L depth-only serial=${SERIAL_NUMBER} preset=${DEVICE_PRESET} profile=${DEPTH_WIDTH}x${DEPTH_HEIGHT}@${DEPTH_FPS}" >&2
njrh_exec_affined docking_camera ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:="${CAMERA_NAME}" \
  serial_number:="${SERIAL_NUMBER}" \
  "${PRESET_LAUNCH_ARGS[@]}" \
  connection_delay:="${CONNECTION_DELAY_SEC}" \
  uvc_backend:="${UVC_BACKEND}" \
  enumerate_net_device:=false \
  enable_heartbeat:="${ENABLE_HEARTBEAT}" \
  diagnostic_period:="${DIAGNOSTIC_PERIOD_SEC}" \
  enable_depth:=true \
  depth_width:="${DEPTH_WIDTH}" \
  depth_height:="${DEPTH_HEIGHT}" \
  depth_fps:="${DEPTH_FPS}" \
  depth_format:=Y16 \
  depth_qos:=sensor_data \
  depth_camera_info_qos:=sensor_data \
  enable_color:=false \
  enable_left_ir:=false \
  enable_right_ir:=false \
  enable_accel:=false \
  enable_gyro:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_d2c_viewer:=false \
  depth_registration:=false \
  publish_tf:=false \
  enable_publish_extrinsic:=false

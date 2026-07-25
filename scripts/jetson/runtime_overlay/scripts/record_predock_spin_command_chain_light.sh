#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"

# A long-lived VSCode shell can retain the common-env loaded marker after its
# ROS overlay variables were replaced. This script is a child process, so force
# one complete local setup without changing the caller's environment.
unset NJRH_COMMON_ENV_LOADED
unset NJRH_COMMON_ENV_SETUP_DONE
export NJRH_COMMON_ENV_PARENT_READY=0
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +e

DURATION_SEC=90
SAMPLE_HZ=20
LABEL="predock_spin_chain"
OUTPUT_DIR=""
PREFIX="[predock-spin-chain]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/record_predock_spin_command_chain_light.sh \
    --duration-sec 90 \
    --label predock_spin_01

Start this read-only recorder, then trigger return-to-dock from the App. Press
Ctrl+C after the fault occurs; the CSV files and summary are still finalized.

The recorder subscribes only to low-bandwidth command, mode, odom, and IMU
topics. It does not record pointcloud, scan, rosbag, or publish any command.

Options:
  --duration-sec N  Maximum capture duration. Default: 90.
  --sample-hz N     Snapshot rate, 2..50 Hz. Default: 20.
  --label LABEL     Report label. Default: predock_spin_chain.
  --output-dir DIR  Explicit output directory. Default: unique directory in /tmp.
  -h, --help        Show this help.
EOF
}

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --sample-hz)
      SAMPLE_HZ="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "${PREFIX} FAIL unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} FAIL --duration-sec must be numeric" >&2
  exit 2
fi
if ! [[ "${SAMPLE_HZ}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} FAIL --sample-hz must be numeric" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="/tmp/njrh_predock_spin_chain_${TIMESTAMP}_${LABEL}"
fi
if ! mkdir -p "${OUTPUT_DIR}/logs_since_start" 2>/dev/null; then
  echo "${PREFIX} FAIL cannot create output directory: ${OUTPUT_DIR}" >&2
  echo "${PREFIX} hint: omit --output-dir to use a unique /tmp directory" >&2
  exit 1
fi

LOG_FILES=(
  "${NJRH_RUNTIME_LOG_DIR}/resident_navigation_runtime.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_api_server.log"
  "${NJRH_RUNTIME_LOG_DIR}/ranger_chassis_common.log"
  "${NJRH_RUNTIME_LOG_DIR}/ranger_base_common.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_safety_common.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_local_state_common.log"
)
LOG_STARTS=()
for log_file in "${LOG_FILES[@]}"; do
  if [[ -f "${log_file}" ]]; then
    LOG_STARTS+=("$(wc -l <"${log_file}")")
  else
    LOG_STARTS+=("-1")
  fi
done

{
  echo "captured_at_utc=${TIMESTAMP}"
  echo "duration_sec=${DURATION_SEC}"
  echo "sample_hz=${SAMPLE_HZ}"
  echo "label=${LABEL}"
  echo "read_only=true"
} >"${OUTPUT_DIR}/metadata.env"

curl -m 2 -fsS "http://127.0.0.1:8080/api/v1/status" \
  >"${OUTPUT_DIR}/api_status_before.json" 2>"${OUTPUT_DIR}/api_status_before.err" || true

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} read-only; wait for READY before triggering return-to-dock"

INTERRUPTED=false
trap 'INTERRUPTED=true' INT TERM
python3 "${SCRIPT_DIR}/record_predock_spin_command_chain.py" \
  --output-dir "${OUTPUT_DIR}" \
  --duration-sec "${DURATION_SEC}" \
  --sample-hz "${SAMPLE_HZ}" \
  --warmup-sec 8.0
RC=$?

curl -m 2 -fsS "http://127.0.0.1:8080/api/v1/status" \
  >"${OUTPUT_DIR}/api_status_after.json" 2>"${OUTPUT_DIR}/api_status_after.err" || true

for index in "${!LOG_FILES[@]}"; do
  log_file="${LOG_FILES[$index]}"
  start_line="${LOG_STARTS[$index]}"
  if [[ "${start_line}" -ge 0 && -f "${log_file}" ]]; then
    output_name="$(basename "${log_file}")"
    tail -n "+$((start_line + 1))" "${log_file}" 2>/dev/null |
      grep -Ei 'spin|rotation|mode|progress|cmd_vel|safety|dock|abort|fail|cancel|yaw' \
        >"${OUTPUT_DIR}/logs_since_start/${output_name}" || true
  fi
done

if [[ "${INTERRUPTED}" == "true" ]]; then
  echo "interrupted=true" >>"${OUTPUT_DIR}/metadata.env"
fi
if [[ "${RC}" -ne 0 ]]; then
  echo "${PREFIX} FAIL recorder exited with rc=${RC}" >&2
  exit "${RC}"
fi

echo "${PREFIX} complete: ${OUTPUT_DIR}"

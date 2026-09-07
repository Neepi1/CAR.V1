#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"

# Force a complete child-shell setup. Long-lived SSH/VSCode shells can retain
# a stale loaded marker even after their ROS overlay variables were replaced.
unset NJRH_COMMON_ENV_LOADED
unset NJRH_COMMON_ENV_SETUP_DONE
export NJRH_COMMON_ENV_PARENT_READY=0
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +e

DURATION_SEC=240
SAMPLE_HZ=20
API_HZ=4
WARMUP_SEC=8
LABEL="return_dock"
OUTPUT_DIR=""
API_URL="${API_URL:-http://127.0.0.1:8080}"
PREFIX="[docking-rotation-trace]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/record_docking_rotation_trace.sh \
    --duration-sec 240 \
    --label return_dock_01

Wait for READY, then trigger the normal return-to-dock operation from the App.
Press Ctrl+C after the rotate-stop-rotate symptom; the report is finalized.

This recorder is read-only. It creates no command publisher, sends no goal,
changes no parameter, and calls no ROS service. It does not subscribe to scan
or pointcloud topics. Output is always below /tmp/njrh_reports.

Options:
  --duration-sec N  Capture time after READY, 5..900 seconds. Default: 240.
  --sample-hz N     Timeline sample rate, 2..30 Hz. Default: 20.
  --api-hz N        Read-only API poll rate, 1..10 Hz. Default: 4.
  --warmup-sec N    DDS discovery warmup, 0..15 seconds. Default: 8.
  --label LABEL     Safe report label. Default: return_dock.
  --output-dir DIR  Explicit unique directory below /tmp/njrh_reports.
  --api-url URL     Local read-only API base URL. Default: http://127.0.0.1:8080.
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
    --api-hz)
      API_HZ="${2:-}"
      shift 2
      ;;
    --warmup-sec)
      WARMUP_SEC="${2:-}"
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
    --api-url)
      API_URL="${2:-}"
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

for numeric_value in "${DURATION_SEC}" "${SAMPLE_HZ}" "${API_HZ}" "${WARMUP_SEC}"; do
  if ! [[ "${numeric_value}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "${PREFIX} FAIL duration/sample/api/warmup values must be numeric" >&2
    exit 2
  fi
done

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${LABEL}" ]]; then
  LABEL="return_dock"
fi
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="/tmp/njrh_reports/docking_rotation_trace_${TIMESTAMP}_${LABEL}"
fi
case "${OUTPUT_DIR}" in
  /tmp/njrh_reports/*)
    ;;
  *)
    echo "${PREFIX} FAIL --output-dir must be below /tmp/njrh_reports" >&2
    exit 2
    ;;
esac
if [[ -e "${OUTPUT_DIR}" ]]; then
  echo "${PREFIX} FAIL output directory already exists: ${OUTPUT_DIR}" >&2
  exit 2
fi
if ! mkdir -p "${OUTPUT_DIR}/logs_since_start"; then
  echo "${PREFIX} FAIL cannot create report directory: ${OUTPUT_DIR}" >&2
  exit 1
fi

LOG_FILES=(
  "${NJRH_RUNTIME_LOG_DIR}/resident_navigation_runtime.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_api_server.log"
  "${NJRH_RUNTIME_LOG_DIR}/robot_docking_manager.log"
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
  echo "started_at_device_clock=$(date --iso-8601=ns)"
  echo "started_at_device_utc_label=$(date -u --iso-8601=ns)"
  echo "duration_sec=${DURATION_SEC}"
  echo "sample_hz=${SAMPLE_HZ}"
  echo "api_hz=${API_HZ}"
  echo "warmup_sec=${WARMUP_SEC}"
  echo "label=${LABEL}"
  echo "read_only=true"
  echo "pointcloud_subscribed=false"
  echo "scan_subscribed=false"
  echo "shell_api_token_present=$([[ -n "${ROBOT_API_TOKEN:-}" ]] && echo true || echo false)"
} >"${OUTPUT_DIR}/shell_metadata.env"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} read-only; wait for READY before triggering return-to-dock"

python3 "${SCRIPT_DIR}/record_docking_rotation_trace.py" \
  --output-dir "${OUTPUT_DIR}" \
  --duration-sec "${DURATION_SEC}" \
  --sample-hz "${SAMPLE_HZ}" \
  --api-hz "${API_HZ}" \
  --warmup-sec "${WARMUP_SEC}" \
  --api-url "${API_URL}" \
  --label "${LABEL}"
RC=$?

for index in "${!LOG_FILES[@]}"; do
  log_file="${LOG_FILES[$index]}"
  start_line="${LOG_STARTS[$index]}"
  if [[ "${start_line}" -ge 0 && -f "${log_file}" ]]; then
    output_name="$(basename "${log_file}")"
    tail -n "+$((start_line + 1))" "${log_file}" 2>/dev/null |
      grep -Ei \
        'dock|predock|yaw|spin|rotation|terminal handoff|bridge.*settle|cmd_vel|collision|safety|mode|abort|fail|cancel' \
        >"${OUTPUT_DIR}/logs_since_start/${output_name}" || true
  fi
done

{
  echo "completed_at_device_clock=$(date --iso-8601=ns)"
  echo "completed_at_device_utc_label=$(date -u --iso-8601=ns)"
  echo "recorder_exit_code=${RC}"
} >"${OUTPUT_DIR}/completed.env"

(
  cd "${OUTPUT_DIR}" || exit 1
  find . -type f ! -name SHA256SUMS -print0 |
    sort -z |
    xargs -0 sha256sum
) >"${OUTPUT_DIR}/SHA256SUMS"

if [[ "${RC}" -ne 0 ]]; then
  echo "${PREFIX} FAIL recorder exited with rc=${RC}; partial report kept at ${OUTPUT_DIR}" >&2
  exit "${RC}"
fi

echo "${PREFIX} complete=${OUTPUT_DIR}"
echo "${PREFIX} summary=${OUTPUT_DIR}/summary.md"

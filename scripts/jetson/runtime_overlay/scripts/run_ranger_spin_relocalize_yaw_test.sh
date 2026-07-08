#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

ANGLES_DEG="180"
ANGULAR_SPEED_RADPS="0.60"
COUNTDOWN_SEC="5"
SETTLE_SEC="3.0"
LABEL="spin180_relocalize_yaw"
OUTPUT_ROOT="${NJRH_TEST_OUTPUT_ROOT:-/tmp/ranger_spin_relocalize_yaw_test}"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
ANGLE_TOLERANCE_DEG="1.0"
MAX_EXTRA_SEC="8.0"
RELOCALIZE_TIMEOUT_SEC="60.0"
RELOCALIZE_SETTLE_TIMEOUT_SEC="15.0"
RUN_RELOCALIZE="true"

usage() {
  cat <<'EOF'
Usage: run_ranger_spin_relocalize_yaw_test.sh [options]

Runs a spin yaw truth test:
  1. command an odom-controlled spin using the existing Ranger spin odom test
  2. after the robot stops, trigger explicit relocalization
  3. record how much map->odom had to correct yaw/translation

This is the right test when wheel odom yaw itself is suspect. The spin segment
still stops when /wheel/odom reaches the target, then relocalization tells us
whether the physical/map yaw actually matched.

Options:
  --angles-deg LIST           Comma-separated signed spin targets. Default: 180
  --angular-speed RADPS       Absolute spin command. Default: 0.60
  --countdown-sec N           Countdown before motion. Default: 5
  --settle-sec SEC            Stop/settle after spin. Default: 3.0
  --label NAME                Report label. Default: spin180_relocalize_yaw
  --output-root DIR           Report root. Default: /tmp/ranger_spin_relocalize_yaw_test
  --cmd-topic TOPIC           Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC          Odom feedback topic. Default: /wheel/odom
  --local-odom-topic TOPIC    Local odom topic. Default: /local_state/odometry
  --no-relocalize             Only run the spin report, do not trigger relocalization
  -h, --help                  Show this help
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --angles-deg|--spin-deg)
      ANGLES_DEG="${2:-}"
      shift 2
      ;;
    --angular-speed)
      ANGULAR_SPEED_RADPS="${2:-}"
      shift 2
      ;;
    --countdown-sec)
      COUNTDOWN_SEC="${2:-}"
      shift 2
      ;;
    --settle-sec)
      SETTLE_SEC="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --cmd-topic)
      CMD_TOPIC="${2:-}"
      shift 2
      ;;
    --odom-topic)
      ODOM_TOPIC="${2:-}"
      shift 2
      ;;
    --local-odom-topic)
      LOCAL_ODOM_TOPIC="${2:-}"
      shift 2
      ;;
    --no-relocalize)
      RUN_RELOCALIZE="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-spin-yaw-truth] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}"
SEGMENT_ROOT="${OUT_DIR}/segments"
mkdir -p "${SEGMENT_ROOT}"

SPIN_SCRIPT="${SCRIPT_DIR}/run_ranger_spin_odom_test.sh"
RELOCALIZE_SCRIPT="${SCRIPT_DIR}/capture_relocalize_correction_compare.sh"

if [[ ! -f "${SPIN_SCRIPT}" ]]; then
  echo "[ranger-spin-yaw-truth] missing required script: ${SPIN_SCRIPT}" >&2
  exit 1
fi
if [[ "${RUN_RELOCALIZE}" == "true" && ! -f "${RELOCALIZE_SCRIPT}" ]]; then
  echo "[ranger-spin-yaw-truth] missing required script: ${RELOCALIZE_SCRIPT}" >&2
  exit 1
fi

{
  echo "# Ranger Spin Relocalize Yaw Test"
  echo
  echo "- timestamp_utc: ${timestamp}"
  echo "- angles_deg: ${ANGLES_DEG}"
  echo "- angular_speed_radps: ${ANGULAR_SPEED_RADPS}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- settle_sec: ${SETTLE_SEC}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- odom_topic: ${ODOM_TOPIC}"
  echo "- local_odom_topic: ${LOCAL_ODOM_TOPIC}"
  echo "- run_relocalize: ${RUN_RELOCALIZE}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## Topic Info"
  for topic in \
    "${CMD_TOPIC}" \
    /cmd_vel_safe \
    /cmd_vel \
    "${ODOM_TOPIC}" \
    "${LOCAL_ODOM_TOPIC}" \
    /localization/bridge_status \
    /safety/status \
    /ranger_mini3_mode_controller/status; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
} >"${OUT_DIR}/environment.md"

set +e
bash "${SPIN_SCRIPT}" \
  --angles-deg "${ANGLES_DEG}" \
  --angular-speed "${ANGULAR_SPEED_RADPS}" \
  --repeat 1 \
  --sample-hz 20.0 \
  --countdown-sec "${COUNTDOWN_SEC}" \
  --settle-sec "${SETTLE_SEC}" \
  --label "${safe_label}_spin_${ANGLES_DEG}" \
  --cmd-topic "${CMD_TOPIC}" \
  --odom-topic "${ODOM_TOPIC}" \
  --local-odom-topic "${LOCAL_ODOM_TOPIC}" \
  --output-root "${SEGMENT_ROOT}" \
  --no-pause-correction \
  --angle-tolerance-deg "${ANGLE_TOLERANCE_DEG}" \
  --max-extra-sec "${MAX_EXTRA_SEC}" | tee "${OUT_DIR}/spin_segment.log"
spin_rc=${PIPESTATUS[0]}
set -e

spin_dir="$(sed -n 's/^\[ranger-spin-test\] report: //p' "${OUT_DIR}/spin_segment.log" | tail -1 || true)"

relocalize_rc=0
relocalize_dir=""
if [[ "${spin_rc}" -eq 0 && "${RUN_RELOCALIZE}" == "true" ]]; then
  set +e
  bash "${RELOCALIZE_SCRIPT}" \
    --test-dir "${spin_dir}" \
    --reason "spin_yaw_truth_after_odom_controlled_spin" \
    --timeout-sec "${RELOCALIZE_TIMEOUT_SEC}" \
    --settle-timeout-sec "${RELOCALIZE_SETTLE_TIMEOUT_SEC}" | tee "${OUT_DIR}/relocalize_compare.log"
  relocalize_rc=${PIPESTATUS[0]}
  set -e
  relocalize_dir="$(sed -n 's/^\[relocalize-capture\] output: //p' "${OUT_DIR}/relocalize_compare.log" | tail -1 || true)"
fi

{
  echo "# Ranger Spin Relocalize Yaw Test Summary"
  echo
  echo "- report_dir: \`${OUT_DIR}\`"
  echo "- angles_deg: \`${ANGLES_DEG}\`"
  echo "- angular_speed_radps: \`${ANGULAR_SPEED_RADPS}\`"
  echo "- spin_rc: \`${spin_rc}\`"
  echo "- relocalize_rc: \`${relocalize_rc}\`"
  echo "- spin_report: \`${spin_dir}\`"
  if [[ -n "${relocalize_dir}" ]]; then
    echo "- relocalize_report: \`${relocalize_dir}/summary.md\`"
  else
    echo "- relocalize_report: \`not_run\`"
  fi
} >"${OUT_DIR}/summary.md"

echo "[ranger-spin-yaw-truth] summary: ${OUT_DIR}/summary.md"
echo "[ranger-spin-yaw-truth] complete: ${OUT_DIR}"

if [[ "${spin_rc}" -ne 0 ]]; then
  exit "${spin_rc}"
fi
exit "${relocalize_rc}"

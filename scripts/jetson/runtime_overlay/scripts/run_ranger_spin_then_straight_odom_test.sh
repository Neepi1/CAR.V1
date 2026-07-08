#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

SPIN_DEG="180"
STRAIGHT_M="8.0"
ANGULAR_SPEED_RADPS="0.60"
LINEAR_SPEED_MPS="1.20"
COUNTDOWN_SEC="5"
SPIN_SETTLE_SEC="2.0"
STRAIGHT_SETTLE_SEC="4.0"
LABEL="spin180_straight8m_no_amcl"
OUTPUT_ROOT="${NJRH_TEST_OUTPUT_ROOT:-/tmp/ranger_spin_then_straight_odom_test}"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
ANGLE_TOLERANCE_DEG="1.0"
DISTANCE_TOLERANCE_M="0.03"
SPIN_MAX_EXTRA_SEC="8.0"
STRAIGHT_MAX_EXTRA_SEC="12.0"
RECORD_FULL_CHAIN="true"
RECORD_DURATION_SEC="60"
SAMPLE_PERIOD_SEC="0.25"
API_URL="${NJRH_API_URL:-http://127.0.0.1:8080}"
PRE_RELOCALIZE="false"
POST_RELOCALIZE_COMPARE="false"
RELOCALIZE_TIMEOUT_SEC="60.0"
RELOCALIZE_SETTLE_TIMEOUT_SEC="15.0"

usage() {
  cat <<'EOF'
Usage: run_ranger_spin_then_straight_odom_test.sh [options]

Runs a controlled odometry diagnosis:
  1. optional pre-test explicit relocalization
  2. signed in-place spin
  3. signed straight-line drive
  4. optional post-test relocalization compare

The motion path remains:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base

By default this script does not pause localization correction and does not
trigger relocalization. It assumes AMCL/bridge correction policy is already set
by the runtime. Use --post-relocalize-compare only when you intentionally want
to measure the final map->odom correction after the robot has stopped.

Options:
  --spin-deg DEG              Signed spin target. Default: 180
  --straight-m M              Signed straight target. Default: 8.0
  --angular-speed RADPS       Absolute spin command. Default: 0.60
  --linear-speed MPS          Absolute straight command. Default: 1.20
  --countdown-sec N           Countdown before motion. Default: 5
  --spin-settle-sec SEC       Stop/settle after spin. Default: 2.0
  --straight-settle-sec SEC   Stop/settle after straight. Default: 4.0
  --label NAME                Report label. Default: spin180_straight8m_no_amcl
  --output-root DIR           Report root. Default: /tmp/ranger_spin_then_straight_odom_test
  --cmd-topic TOPIC           Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC          Odom feedback topic. Default: /wheel/odom
  --local-odom-topic TOPIC    Local odom topic. Default: /local_state/odometry
  --record-duration-sec SEC   Full-chain recorder duration. Default: 60
  --sample-period-sec SEC     Full-chain recorder sample period. Default: 0.25
  --no-full-chain-record      Do not run record_navigation_amcl_odom_correlation.sh
  --pre-relocalize            Trigger explicit relocalization before motion
  --post-relocalize-compare   Trigger explicit relocalization after motion and record correction
  --api-url URL               API URL passed to full-chain recorder
  -h, --help                  Show this help
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --spin-deg)
      SPIN_DEG="${2:-}"
      shift 2
      ;;
    --straight-m)
      STRAIGHT_M="${2:-}"
      shift 2
      ;;
    --angular-speed)
      ANGULAR_SPEED_RADPS="${2:-}"
      shift 2
      ;;
    --linear-speed|--linear-speed-mps)
      LINEAR_SPEED_MPS="${2:-}"
      shift 2
      ;;
    --countdown-sec)
      COUNTDOWN_SEC="${2:-}"
      shift 2
      ;;
    --spin-settle-sec)
      SPIN_SETTLE_SEC="${2:-}"
      shift 2
      ;;
    --straight-settle-sec)
      STRAIGHT_SETTLE_SEC="${2:-}"
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
    --record-duration-sec)
      RECORD_DURATION_SEC="${2:-}"
      shift 2
      ;;
    --sample-period-sec)
      SAMPLE_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --no-full-chain-record)
      RECORD_FULL_CHAIN="false"
      shift
      ;;
    --pre-relocalize)
      PRE_RELOCALIZE="true"
      shift
      ;;
    --post-relocalize-compare)
      POST_RELOCALIZE_COMPARE="true"
      shift
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
      echo "[ranger-spin-straight] unknown argument: $1" >&2
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
FULL_CHAIN_DIR="${OUT_DIR}/full_chain_record"
mkdir -p "${SEGMENT_ROOT}"

SPIN_SCRIPT="${SCRIPT_DIR}/run_ranger_spin_odom_test.sh"
STRAIGHT_SCRIPT="${SCRIPT_DIR}/run_ranger_straight_odom_test.sh"
RECORD_SCRIPT="${SCRIPT_DIR}/record_navigation_amcl_odom_correlation.sh"
RELOCALIZE_SCRIPT="${SCRIPT_DIR}/capture_relocalize_correction_compare.sh"

for required in "${SPIN_SCRIPT}" "${STRAIGHT_SCRIPT}"; do
  if [[ ! -x "${required}" && ! -f "${required}" ]]; then
    echo "[ranger-spin-straight] missing required script: ${required}" >&2
    exit 1
  fi
done

{
  echo "# Ranger Spin Then Straight Odom Test"
  echo
  echo "- timestamp_utc: ${timestamp}"
  echo "- spin_deg: ${SPIN_DEG}"
  echo "- straight_m: ${STRAIGHT_M}"
  echo "- angular_speed_radps: ${ANGULAR_SPEED_RADPS}"
  echo "- linear_speed_mps: ${LINEAR_SPEED_MPS}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- spin_settle_sec: ${SPIN_SETTLE_SEC}"
  echo "- straight_settle_sec: ${STRAIGHT_SETTLE_SEC}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- odom_topic: ${ODOM_TOPIC}"
  echo "- local_odom_topic: ${LOCAL_ODOM_TOPIC}"
  echo "- record_full_chain: ${RECORD_FULL_CHAIN}"
  echo "- record_duration_sec: ${RECORD_DURATION_SEC}"
  echo "- sample_period_sec: ${SAMPLE_PERIOD_SEC}"
  echo "- pre_relocalize: ${PRE_RELOCALIZE}"
  echo "- post_relocalize_compare: ${POST_RELOCALIZE_COMPARE}"
  echo "- api_url: ${API_URL}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## ROS Nodes"
  ros2 node list 2>&1 || true
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
    /ranger_mini3_mode_controller/status \
    /motion_state \
    /system_state \
    /actuator_state; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
} >"${OUT_DIR}/environment.md"

recorder_pid=""
cleanup() {
  if [[ -n "${recorder_pid}" ]] && kill -0 "${recorder_pid}" 2>/dev/null; then
    kill "${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ "${RECORD_FULL_CHAIN}" == "true" ]]; then
  if [[ -f "${RECORD_SCRIPT}" ]]; then
    bash "${RECORD_SCRIPT}" \
      --duration-sec "${RECORD_DURATION_SEC}" \
      --sample-period-sec "${SAMPLE_PERIOD_SEC}" \
      --label "${safe_label}_full_chain" \
      --no-rosout \
      --output-dir "${FULL_CHAIN_DIR}" \
      --api-url "${API_URL}" >"${OUT_DIR}/full_chain_record.log" 2>&1 &
    recorder_pid="$!"
    echo "[ranger-spin-straight] full-chain recorder pid=${recorder_pid} dir=${FULL_CHAIN_DIR}"
    sleep 1
  else
    echo "[ranger-spin-straight] recorder script not found: ${RECORD_SCRIPT}" | tee "${OUT_DIR}/full_chain_record.log"
  fi
fi

if [[ "${PRE_RELOCALIZE}" == "true" ]]; then
  if [[ ! -f "${RELOCALIZE_SCRIPT}" ]]; then
    echo "[ranger-spin-straight] cannot pre-relocalize; missing ${RELOCALIZE_SCRIPT}" >&2
    exit 1
  fi
  echo "[ranger-spin-straight] pre-test relocalization..."
  bash "${RELOCALIZE_SCRIPT}" \
    --test-dir "${OUT_DIR}" \
    --output-dir "${OUT_DIR}/pre_relocalize_compare" \
    --reason "spin_then_straight_pre_relocalize" \
    --timeout-sec "${RELOCALIZE_TIMEOUT_SEC}" \
    --settle-timeout-sec "${RELOCALIZE_SETTLE_TIMEOUT_SEC}" | tee "${OUT_DIR}/pre_relocalize.log"
fi

if [[ "${COUNTDOWN_SEC}" != "0" ]]; then
  echo "[ranger-spin-straight] motion starts in ${COUNTDOWN_SEC}s. Ensure the path is clear and E-stop is available."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-spin-straight] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

spin_rc=0
straight_rc=0

echo "[ranger-spin-straight] running spin segment..."
set +e
bash "${SPIN_SCRIPT}" \
  --angles-deg "${SPIN_DEG}" \
  --angular-speed "${ANGULAR_SPEED_RADPS}" \
  --repeat 1 \
  --sample-hz 20.0 \
  --countdown-sec 0 \
  --settle-sec "${SPIN_SETTLE_SEC}" \
  --label "${safe_label}_spin_${SPIN_DEG}" \
  --cmd-topic "${CMD_TOPIC}" \
  --odom-topic "${ODOM_TOPIC}" \
  --local-odom-topic "${LOCAL_ODOM_TOPIC}" \
  --output-root "${SEGMENT_ROOT}" \
  --no-pause-correction \
  --angle-tolerance-deg "${ANGLE_TOLERANCE_DEG}" \
  --max-extra-sec "${SPIN_MAX_EXTRA_SEC}" | tee "${OUT_DIR}/spin_segment.log"
spin_rc=${PIPESTATUS[0]}
set -e

if [[ "${spin_rc}" -ne 0 ]]; then
  echo "[ranger-spin-straight] spin segment failed rc=${spin_rc}" >&2
else
  echo "[ranger-spin-straight] running straight segment..."
  set +e
  bash "${STRAIGHT_SCRIPT}" \
    --distance-m "${STRAIGHT_M}" \
    --linear-speed "${LINEAR_SPEED_MPS}" \
    --repeat 1 \
    --sample-hz 20.0 \
    --countdown-sec 0 \
    --settle-sec "${STRAIGHT_SETTLE_SEC}" \
    --label "${safe_label}_straight_${STRAIGHT_M}" \
    --cmd-topic "${CMD_TOPIC}" \
    --odom-topic "${ODOM_TOPIC}" \
    --local-odom-topic "${LOCAL_ODOM_TOPIC}" \
    --output-root "${SEGMENT_ROOT}" \
    --no-pause-correction \
    --distance-tolerance-m "${DISTANCE_TOLERANCE_M}" \
    --max-extra-sec "${STRAIGHT_MAX_EXTRA_SEC}" | tee "${OUT_DIR}/straight_segment.log"
  straight_rc=${PIPESTATUS[0]}
  set -e
fi

post_relocalize_rc=0
if [[ "${POST_RELOCALIZE_COMPARE}" == "true" ]]; then
  if [[ ! -f "${RELOCALIZE_SCRIPT}" ]]; then
    echo "[ranger-spin-straight] cannot post-relocalize; missing ${RELOCALIZE_SCRIPT}" >&2
    post_relocalize_rc=1
  else
    echo "[ranger-spin-straight] post-test relocalization compare..."
    set +e
    bash "${RELOCALIZE_SCRIPT}" \
      --test-dir "${OUT_DIR}" \
      --reason "spin_then_straight_post_relocalize" \
      --timeout-sec "${RELOCALIZE_TIMEOUT_SEC}" \
      --settle-timeout-sec "${RELOCALIZE_SETTLE_TIMEOUT_SEC}" | tee "${OUT_DIR}/post_relocalize.log"
    post_relocalize_rc=${PIPESTATUS[0]}
    set -e
  fi
fi

if [[ -n "${recorder_pid}" ]]; then
  echo "[ranger-spin-straight] waiting for full-chain recorder..."
  set +e
  wait "${recorder_pid}"
  recorder_rc=$?
  recorder_pid=""
  set -e
else
  recorder_rc=0
fi

spin_dir="$(sed -n 's/^\[ranger-spin-test\] report: //p' "${OUT_DIR}/spin_segment.log" | tail -1 || true)"
if [[ -f "${OUT_DIR}/straight_segment.log" ]]; then
  straight_dir="$(sed -n 's/^\[ranger-straight-test\] report: //p' "${OUT_DIR}/straight_segment.log" | tail -1 || true)"
else
  straight_dir=""
fi
post_dir="$(sed -n 's/^\[relocalize-capture\] output: //p' "${OUT_DIR}/post_relocalize.log" 2>/dev/null | tail -1 || true)"

{
  echo "# Ranger Spin Then Straight Odom Test Summary"
  echo
  echo "- report_dir: \`${OUT_DIR}\`"
  echo "- spin_deg: \`${SPIN_DEG}\`"
  echo "- straight_m: \`${STRAIGHT_M}\`"
  echo "- angular_speed_radps: \`${ANGULAR_SPEED_RADPS}\`"
  echo "- linear_speed_mps: \`${LINEAR_SPEED_MPS}\`"
  echo "- spin_rc: \`${spin_rc}\`"
  echo "- straight_rc: \`${straight_rc}\`"
  echo "- recorder_rc: \`${recorder_rc}\`"
  echo "- post_relocalize_rc: \`${post_relocalize_rc}\`"
  echo "- spin_report: \`${spin_dir}\`"
  echo "- straight_report: \`${straight_dir}\`"
  echo "- full_chain_report: \`${FULL_CHAIN_DIR}/summary.md\`"
  if [[ -n "${post_dir}" ]]; then
    echo "- post_relocalize_report: \`${post_dir}/summary.md\`"
  else
    echo "- post_relocalize_report: \`not_run\`"
  fi
  echo
  echo "## Next Step"
  echo
  if [[ "${POST_RELOCALIZE_COMPARE}" != "true" ]]; then
    echo "To measure final map->odom pull-back after the robot is fully stopped, run:"
    echo
    echo '```bash'
    echo "bash ${RELOCALIZE_SCRIPT} --test-dir ${OUT_DIR} --reason spin_then_straight_manual_post_relocalize"
    echo '```'
  else
    echo "Post relocalization compare was requested and is listed above."
  fi
} >"${OUT_DIR}/summary.md"

echo "[ranger-spin-straight] summary: ${OUT_DIR}/summary.md"
echo "[ranger-spin-straight] complete: ${OUT_DIR}"

if [[ "${spin_rc}" -ne 0 ]]; then
  exit "${spin_rc}"
fi
if [[ "${straight_rc}" -ne 0 ]]; then
  exit "${straight_rc}"
fi
if [[ "${post_relocalize_rc}" -ne 0 ]]; then
  exit "${post_relocalize_rc}"
fi
exit "${recorder_rc}"

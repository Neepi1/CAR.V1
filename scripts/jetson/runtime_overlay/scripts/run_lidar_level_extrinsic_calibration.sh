#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

ANGLES_DEG="0,90,180,270"
REPEAT="2"
ANGULAR_SPEED_RADPS="0.20"
COUNTDOWN_SEC="3"
SETTLE_SEC="3.0"
LABEL="lidar_level_static_4heading"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/lidar_extrinsic_calibration"
CMD_TOPIC="/cmd_vel_collision_checked"
AUTO_SPIN="true"
EXPECTED_FIRST_HEADING_DEG=""
EXPECTED_YAWS_DEG=""
TIMEOUT_SEC="60.0"
SETTLE_TIMEOUT_SEC="20.0"

usage() {
  cat <<'EOF'
Usage: run_lidar_level_extrinsic_calibration.sh [options]

Captures static Isaac relocalization samples at repeated headings, then fits
base_link -> lidar_level_link planar extrinsics. Movement, when enabled, stays
on the safety chain:
  script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base

Each relocalization sample also writes optional read-only scan-map diagnostics
under sample_*/scan_map_alignment/ when LaserScan topics and the active map are
available.

Options:
  --angles-deg LIST             Relative heading labels. Default: 0,90,180,270
  --repeat N                    Repeat heading list. Default: 2
  --angular-speed RADPS         Spin command speed. Default: 0.20
  --countdown-sec N             Countdown before each spin. Default: 3
  --settle-sec SEC              Stop/settle after each spin. Default: 3.0
  --label NAME                  Report label. Default: lidar_level_static_4heading
  --output-root DIR             Report root. Default: reports/lidar_extrinsic_calibration
  --cmd-topic TOPIC             Safety-chain input topic. Default: /cmd_vel_collision_checked
  --manual-step                 Do not publish spin commands; wait for Enter between headings.
  --expected-first-heading-deg DEG
                                Absolute map yaw for heading_0. Enables yaw fit.
  --expected-yaws-deg LIST      Absolute expected map yaws for every captured sample.
  --timeout-sec SEC             Relocalization service timeout. Default: 60.0
  --settle-timeout-sec SEC      Bridge smoothing settle timeout. Default: 20.0

If no expected yaw is provided, the script still fits XY. Absolute yaw is not
observable from same-spot four-heading samples alone.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --angles-deg)
      ANGLES_DEG="${2:-}"
      shift 2
      ;;
    --repeat)
      REPEAT="${2:-}"
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
    --manual-step)
      AUTO_SPIN="false"
      shift
      ;;
    --expected-first-heading-deg)
      EXPECTED_FIRST_HEADING_DEG="${2:-}"
      shift 2
      ;;
    --expected-yaws-deg)
      EXPECTED_YAWS_DEG="${2:-}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --settle-timeout-sec)
      SETTLE_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[lidar-extrinsic-calib] unknown argument: $1" >&2
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
mkdir -p "${OUT_DIR}"

split_headings() {
  python3 - "$ANGLES_DEG" "$REPEAT" <<'PY'
import sys
angles = [part.strip() for part in sys.argv[1].split(",") if part.strip()]
repeat = int(sys.argv[2])
if not angles:
    raise SystemExit("angles list is empty")
if repeat < 1:
    raise SystemExit("repeat must be >= 1")
for _ in range(repeat):
    for angle in angles:
        print(angle)
PY
}

normal_delta() {
  python3 - "$1" "$2" <<'PY'
import math
import sys
current = float(sys.argv[1])
target = float(sys.argv[2])
delta = (target - current + 180.0) % 360.0 - 180.0
if delta <= -180.0:
    delta += 360.0
print(f"{delta:.9f}")
PY
}

sanitize_heading() {
  printf '%s' "$1" | tr '+' 'p' | tr '-' 'm' | tr '.' 'p'
}

mapfile -t headings < <(split_headings)

{
  echo "# Lidar Level Extrinsic Calibration Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- label: ${LABEL}"
  echo "- angles_deg: ${ANGLES_DEG}"
  echo "- repeat: ${REPEAT}"
  echo "- angular_speed_radps: ${ANGULAR_SPEED_RADPS}"
  echo "- auto_spin: ${AUTO_SPIN}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- expected_first_heading_deg: ${EXPECTED_FIRST_HEADING_DEG}"
  echo "- expected_yaws_deg: ${EXPECTED_YAWS_DEG}"
  echo "- current_config: ${OVERLAY_ROOT}/config/sensors.yaml"
  echo
  echo "## Current sensors.yaml"
  sed -n '1,80p' "${OVERLAY_ROOT}/config/sensors.yaml" || true
  echo
  echo "## ROS Nodes"
  ros2 node list 2>&1 || true
  echo
  echo "## TF"
  timeout 5s ros2 run tf2_ros tf2_echo base_link lidar_level_link 2>&1 || true
} >"${OUT_DIR}/environment.md"

echo "[lidar-extrinsic-calib] output: ${OUT_DIR}"
echo "[lidar-extrinsic-calib] headings: ${headings[*]}"
if [[ "${AUTO_SPIN}" == "true" ]]; then
  echo "[lidar-extrinsic-calib] ensure the robot has clear spin space and E-stop is available."
fi

sample_dirs=()
for ((i = 0; i < ${#headings[@]}; i++)); do
  heading="${headings[$i]}"
  sample_dir="${OUT_DIR}/sample_$(printf '%02d' "$((i + 1))")_heading_$(sanitize_heading "${heading}")"
  reason="lidar_level_extrinsic_heading_${heading}_sample_$((i + 1))"
  echo "[lidar-extrinsic-calib] capture sample $((i + 1))/${#headings[@]} heading=${heading}"
  bash "${SCRIPT_DIR}/capture_relocalize_correction_compare.sh" \
    --output-dir "${sample_dir}" \
    --reason "${reason}" \
    --timeout-sec "${TIMEOUT_SEC}" \
    --settle-timeout-sec "${SETTLE_TIMEOUT_SEC}"
  sample_dirs+=("${sample_dir}")

  if (( i + 1 >= ${#headings[@]} )); then
    break
  fi

  next_heading="${headings[$((i + 1))]}"
  delta="$(normal_delta "${heading}" "${next_heading}")"
  if [[ "${AUTO_SPIN}" == "true" ]]; then
    echo "[lidar-extrinsic-calib] spin delta=${delta} deg to next heading=${next_heading}"
    bash "${SCRIPT_DIR}/run_ranger_spin_odom_test.sh" \
      --angles-deg "${delta}" \
      --repeat 1 \
      --angular-speed "${ANGULAR_SPEED_RADPS}" \
      --countdown-sec "${COUNTDOWN_SEC}" \
      --settle-sec "${SETTLE_SEC}" \
      --label "${safe_label}_spin_$((i + 1))_${delta}" \
      --cmd-topic "${CMD_TOPIC}" \
      --output-root "${OUT_DIR}/spin_reports"
  else
    echo "[lidar-extrinsic-calib] manually rotate to heading ${next_heading}, then press Enter."
    read -r _
  fi
done

fit_args=(
  --root "${OUT_DIR}"
  --current-config "${OVERLAY_ROOT}/config/sensors.yaml"
  --output-json "${OUT_DIR}/calibration_fit.json"
  --summary-md "${OUT_DIR}/summary.md"
)
if [[ -n "${EXPECTED_FIRST_HEADING_DEG}" ]]; then
  fit_args+=(--expected-first-heading-deg "${EXPECTED_FIRST_HEADING_DEG}")
fi
if [[ -n "${EXPECTED_YAWS_DEG}" ]]; then
  fit_args+=(--expected-yaws-deg "${EXPECTED_YAWS_DEG}")
fi

python3 "${SCRIPT_DIR}/fit_lidar_level_extrinsic_from_relocalize_samples.py" "${fit_args[@]}"
tar -czf "${OUT_DIR}.tgz" -C "$(dirname "${OUT_DIR}")" "$(basename "${OUT_DIR}")"
echo "summary: ${OUT_DIR#${PROJECT_ROOT}/}/summary.md"
echo "archive: ${OUT_DIR#${PROJECT_ROOT}/}.tgz"

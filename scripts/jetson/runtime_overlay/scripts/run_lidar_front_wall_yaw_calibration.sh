#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

LABEL="front_wall_yaw"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/lidar_front_wall_yaw_calibration"
SCAN_TOPIC="/scan"
BASE_FRAME="base_link"
CURRENT_CONFIG="${OVERLAY_ROOT}/config/sensors.yaml"
COLLECT_SEC="3.0"
MAX_SCANS="60"
FRONT_HALF_ANGLE_DEG="15.0"
RANGE_MIN_M="0.5"
RANGE_MAX_M="8.0"
EXPECTED_NORMAL_DEG="0.0"
LINE_INLIER_THRESHOLD_M="0.03"
RANSAC_ITERATIONS="1200"

usage() {
  cat <<'EOF'
Usage: run_lidar_front_wall_yaw_calibration.sh [options]

Estimate base_link -> lidar_level_link yaw error from a static front-wall scan.
The robot must be physically square to a straight wall segment. This is a
read-only diagnostic: it does not publish velocity, trigger localization, or
modify configuration.

Options:
  --label NAME                    Report label. Default: front_wall_yaw
  --output-root DIR               Report root. Default: reports/lidar_front_wall_yaw_calibration
  --scan-topic TOPIC              LaserScan topic. Default: /scan
  --base-frame FRAME              Target frame for fitting. Default: base_link
  --current-config FILE           sensors.yaml to read current lidar_yaw.
                                  Default: runtime_overlay/config/sensors.yaml
  --collect-sec SEC               Scan collection window. Default: 3.0
  --max-scans N                   Maximum scans to collect. Default: 60
  --front-half-angle-deg DEG       Keep points inside +/- DEG from base +X. Default: 15
  --range-min-m M                 Minimum range. Default: 0.5
  --range-max-m M                 Maximum range. Default: 8.0
  --expected-normal-deg DEG        Expected wall normal in base_link. Default: 0
                                  Use 0 when the robot front faces the wall.
  --line-inlier-threshold-m M      RANSAC line inlier threshold. Default: 0.03
  --ransac-iterations N           RANSAC iterations. Default: 1200
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --scan-topic)
      SCAN_TOPIC="${2:-}"
      shift 2
      ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --current-config)
      CURRENT_CONFIG="${2:-}"
      shift 2
      ;;
    --collect-sec)
      COLLECT_SEC="${2:-}"
      shift 2
      ;;
    --max-scans)
      MAX_SCANS="${2:-}"
      shift 2
      ;;
    --front-half-angle-deg)
      FRONT_HALF_ANGLE_DEG="${2:-}"
      shift 2
      ;;
    --range-min-m)
      RANGE_MIN_M="${2:-}"
      shift 2
      ;;
    --range-max-m)
      RANGE_MAX_M="${2:-}"
      shift 2
      ;;
    --expected-normal-deg)
      EXPECTED_NORMAL_DEG="${2:-}"
      shift 2
      ;;
    --line-inlier-threshold-m)
      LINE_INLIER_THRESHOLD_M="${2:-}"
      shift 2
      ;;
    --ransac-iterations)
      RANSAC_ITERATIONS="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[front-wall-yaw] unknown argument: $1" >&2
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

{
  echo "# Front Wall Yaw Calibration Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- scan_topic: ${SCAN_TOPIC}"
  echo "- base_frame: ${BASE_FRAME}"
  echo "- current_config: ${CURRENT_CONFIG}"
  echo "- collect_sec: ${COLLECT_SEC}"
  echo "- max_scans: ${MAX_SCANS}"
  echo "- front_half_angle_deg: ${FRONT_HALF_ANGLE_DEG}"
  echo "- range_min_m: ${RANGE_MIN_M}"
  echo "- range_max_m: ${RANGE_MAX_M}"
  echo "- expected_normal_deg: ${EXPECTED_NORMAL_DEG}"
  echo "- line_inlier_threshold_m: ${LINE_INLIER_THRESHOLD_M}"
  echo
  echo "## Current sensors.yaml"
  sed -n '1,80p' "${CURRENT_CONFIG}" || true
  echo
  echo "## Scan Topic"
  timeout 3s ros2 topic info "${SCAN_TOPIC}" 2>&1 || true
  echo
  echo "## TF ${BASE_FRAME} -> lidar_level_link"
  timeout 5s ros2 run tf2_ros tf2_echo "${BASE_FRAME}" lidar_level_link 2>&1 || true
} >"${OUT_DIR}/environment.md"

python3 "${SCRIPT_DIR}/fit_front_wall_lidar_yaw.py" \
  --scan-topic "${SCAN_TOPIC}" \
  --base-frame "${BASE_FRAME}" \
  --current-config "${CURRENT_CONFIG}" \
  --collect-sec "${COLLECT_SEC}" \
  --max-scans "${MAX_SCANS}" \
  --front-half-angle-deg "${FRONT_HALF_ANGLE_DEG}" \
  --range-min-m "${RANGE_MIN_M}" \
  --range-max-m "${RANGE_MAX_M}" \
  --expected-normal-deg "${EXPECTED_NORMAL_DEG}" \
  --line-inlier-threshold-m "${LINE_INLIER_THRESHOLD_M}" \
  --ransac-iterations "${RANSAC_ITERATIONS}" \
  --output-json "${OUT_DIR}/front_wall_yaw_fit.json" \
  --summary-md "${OUT_DIR}/summary.md"

tar -czf "${OUT_DIR}.tgz" -C "$(dirname "${OUT_DIR}")" "$(basename "${OUT_DIR}")"
echo "summary: ${OUT_DIR#${PROJECT_ROOT}/}/summary.md"
echo "archive: ${OUT_DIR#${PROJECT_ROOT}/}.tgz"

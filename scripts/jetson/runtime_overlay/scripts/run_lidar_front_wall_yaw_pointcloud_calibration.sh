#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

LABEL="front_wall_yaw_3d"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/lidar_front_wall_yaw_calibration"
CLOUD_TOPIC="/lidar_points"
BASE_FRAME="base_link"
CURRENT_CONFIG="${OVERLAY_ROOT}/config/sensors.yaml"
COLLECT_SEC="3.0"
MAX_CLOUDS="8"
X_MIN_M="1.0"
X_MAX_M="8.0"
Y_ABS_MAX_M="1.2"
Z_MIN_M="0.5"
Z_MAX_M="2.6"
POINT_STRIDE="8"
MAX_POINTS="8000"
EXPECTED_NORMAL_DEG="0.0"
LINE_INLIER_THRESHOLD_M="0.04"
RANSAC_ITERATIONS="1500"

usage() {
  cat <<'EOF'
Usage: run_lidar_front_wall_yaw_pointcloud_calibration.sh [options]

Estimate base_link -> lidar_level_link yaw error from a 3D front-wall point
cloud fit. The robot must be physically square to a straight vertical wall
segment. This is read-only: no velocity, localization trigger, or config edit.

Options:
  --label NAME                    Report label. Default: front_wall_yaw_3d
  --output-root DIR               Report root. Default: reports/lidar_front_wall_yaw_calibration
  --cloud-topic TOPIC             PointCloud2 topic. Default: /lidar_points
  --base-frame FRAME              Target frame for fitting. Default: base_link
  --current-config FILE           sensors.yaml to read current lidar_yaw.
  --collect-sec SEC               Point cloud collection window. Default: 3.0
  --max-clouds N                  Maximum clouds to collect. Default: 8
  --x-min-m M                     Front ROI min x in base_link. Default: 1.0
  --x-max-m M                     Front ROI max x in base_link. Default: 8.0
  --y-abs-max-m M                 Front ROI lateral half width. Default: 1.2
  --z-min-m M                     Front ROI min height. Default: 0.5
  --z-max-m M                     Front ROI max height. Default: 2.6
  --point-stride N                Use every Nth point from each cloud. Default: 8
  --max-points N                  Max cropped points retained. Default: 8000
  --expected-normal-deg DEG        Expected wall normal in base_link. Default: 0
                                  Use 0 when the robot front faces the wall.
  --line-inlier-threshold-m M      RANSAC line inlier threshold. Default: 0.04
  --ransac-iterations N           RANSAC iterations. Default: 1500
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
    --cloud-topic)
      CLOUD_TOPIC="${2:-}"
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
    --max-clouds)
      MAX_CLOUDS="${2:-}"
      shift 2
      ;;
    --x-min-m)
      X_MIN_M="${2:-}"
      shift 2
      ;;
    --x-max-m)
      X_MAX_M="${2:-}"
      shift 2
      ;;
    --y-abs-max-m)
      Y_ABS_MAX_M="${2:-}"
      shift 2
      ;;
    --z-min-m)
      Z_MIN_M="${2:-}"
      shift 2
      ;;
    --z-max-m)
      Z_MAX_M="${2:-}"
      shift 2
      ;;
    --point-stride)
      POINT_STRIDE="${2:-}"
      shift 2
      ;;
    --max-points)
      MAX_POINTS="${2:-}"
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
      echo "[front-wall-yaw-3d] unknown argument: $1" >&2
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
  echo "# Front Wall 3D Yaw Calibration Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- cloud_topic: ${CLOUD_TOPIC}"
  echo "- base_frame: ${BASE_FRAME}"
  echo "- current_config: ${CURRENT_CONFIG}"
  echo "- collect_sec: ${COLLECT_SEC}"
  echo "- max_clouds: ${MAX_CLOUDS}"
  echo "- x_min_m: ${X_MIN_M}"
  echo "- x_max_m: ${X_MAX_M}"
  echo "- y_abs_max_m: ${Y_ABS_MAX_M}"
  echo "- z_min_m: ${Z_MIN_M}"
  echo "- z_max_m: ${Z_MAX_M}"
  echo "- point_stride: ${POINT_STRIDE}"
  echo "- expected_normal_deg: ${EXPECTED_NORMAL_DEG}"
  echo "- line_inlier_threshold_m: ${LINE_INLIER_THRESHOLD_M}"
  echo
  echo "## Current sensors.yaml"
  sed -n '1,80p' "${CURRENT_CONFIG}" || true
  echo
  echo "## Cloud Topic"
  timeout 3s ros2 topic info "${CLOUD_TOPIC}" 2>&1 || true
  echo
  echo "## TF ${BASE_FRAME} -> lidar_level_link"
  timeout 5s ros2 run tf2_ros tf2_echo "${BASE_FRAME}" lidar_level_link 2>&1 || true
} >"${OUT_DIR}/environment.md"

PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}" python3 "${SCRIPT_DIR}/fit_front_wall_lidar_yaw_pointcloud.py" \
  --cloud-topic "${CLOUD_TOPIC}" \
  --base-frame "${BASE_FRAME}" \
  --current-config "${CURRENT_CONFIG}" \
  --collect-sec "${COLLECT_SEC}" \
  --max-clouds "${MAX_CLOUDS}" \
  --x-min-m "${X_MIN_M}" \
  --x-max-m "${X_MAX_M}" \
  --y-abs-max-m "${Y_ABS_MAX_M}" \
  --z-min-m "${Z_MIN_M}" \
  --z-max-m "${Z_MAX_M}" \
  --point-stride "${POINT_STRIDE}" \
  --max-points "${MAX_POINTS}" \
  --expected-normal-deg "${EXPECTED_NORMAL_DEG}" \
  --line-inlier-threshold-m "${LINE_INLIER_THRESHOLD_M}" \
  --ransac-iterations "${RANSAC_ITERATIONS}" \
  --output-json "${OUT_DIR}/front_wall_yaw_3d_fit.json" \
  --summary-md "${OUT_DIR}/summary.md"

tar -czf "${OUT_DIR}.tgz" -C "$(dirname "${OUT_DIR}")" "$(basename "${OUT_DIR}")"
echo "summary: ${OUT_DIR#${PROJECT_ROOT}/}/summary.md"
echo "archive: ${OUT_DIR#${PROJECT_ROOT}/}.tgz"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MEASURE_SEC="${NJRH_VERIFY_POINTCLOUD_RATE_SEC:-8}"
RAW_TOPIC="${NJRH_VERIFY_POINTCLOUD_RAW_TOPIC:-/jt128/vendor/points_raw}"
LIDAR_TOPIC="${NJRH_VERIFY_POINTCLOUD_LIDAR_TOPIC:-/lidar_points}"
MIN_RAW_HZ="${NJRH_VERIFY_POINTCLOUD_MIN_RAW_HZ:-18.0}"
MIN_LIDAR_HZ="${NJRH_VERIFY_POINTCLOUD_MIN_LIDAR_HZ:-18.0}"
MIN_LIDAR_WARN_HZ="${NJRH_VERIFY_POINTCLOUD_MIN_LIDAR_WARN_HZ:-10.0}"
OPTIONAL_TOPICS="${NJRH_VERIFY_POINTCLOUD_OPTIONAL_TOPICS:-/lidar_points_nav /points_nav /scan /flatscan /perception/obstacle_points /perception/clearing_points}"

tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_rates_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

float_ge() {
  awk -v value="$1" -v minimum="$2" 'BEGIN { exit(value + 0.0 >= minimum + 0.0 ? 0 : 1) }'
}

topic_exists() {
  timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

measure_topic() {
  local topic="$1"
  local label="$2"
  local required="$3"
  local output_file="${tmp_dir}/${label}.txt"
  local rate=""

  echo "[pointcloud-rates] measuring ${topic} for ${MEASURE_SEC}s"
  if ! topic_exists "${topic}"; then
    echo "[pointcloud-rates] ${topic}: NOT_PRESENT"
    [[ "${required}" == "false" ]] && return 0
    return 1
  fi

  timeout "${MEASURE_SEC}" ros2 topic hz "${topic}" >"${output_file}" 2>&1 || true
  rate="$(awk '/average rate:/ {rate=$3} END {print rate}' "${output_file}")"
  if [[ -z "${rate}" ]]; then
    sed 's/^/[pointcloud-rates]   /' "${output_file}" >&2 || true
    echo "[pointcloud-rates] ${topic}: NO_SAMPLES"
    [[ "${required}" == "false" ]] && return 0
    return 1
  fi

  echo "[pointcloud-rates] ${topic}: ${rate} Hz"
  printf '%s\n' "${rate}" >"${tmp_dir}/${label}.rate"
}

status=0

measure_topic "${RAW_TOPIC}" "raw" "true" || status=1
measure_topic "${LIDAR_TOPIC}" "lidar" "true" || status=1

for topic in ${OPTIONAL_TOPICS}; do
  safe_label="$(printf '%s' "${topic#/}" | tr '/:' '__')"
  measure_topic "${topic}" "${safe_label}" "false" || true
done

if [[ -s "${tmp_dir}/raw.rate" ]]; then
  raw_rate="$(<"${tmp_dir}/raw.rate")"
  if float_ge "${raw_rate}" "${MIN_RAW_HZ}"; then
    echo "[pointcloud-rates] PASS raw ${raw_rate} >= ${MIN_RAW_HZ} Hz"
  else
    echo "[pointcloud-rates] FAIL raw ${raw_rate} < ${MIN_RAW_HZ} Hz"
    status=1
  fi
else
  echo "[pointcloud-rates] FAIL raw rate unavailable"
  status=1
fi

if [[ -s "${tmp_dir}/lidar.rate" ]]; then
  lidar_rate="$(<"${tmp_dir}/lidar.rate")"
  if float_ge "${lidar_rate}" "${MIN_LIDAR_HZ}"; then
    echo "[pointcloud-rates] PASS /lidar_points ${lidar_rate} >= ${MIN_LIDAR_HZ} Hz"
  else
    echo "[pointcloud-rates] FAIL /lidar_points ${lidar_rate} < ${MIN_LIDAR_HZ} Hz"
    status=1
  fi
  if ! float_ge "${lidar_rate}" "${MIN_LIDAR_WARN_HZ}"; then
    echo "[pointcloud-rates] FAIL /lidar_points dropped below ${MIN_LIDAR_WARN_HZ} Hz while local perception may be active"
    status=1
  fi
else
  echo "[pointcloud-rates] FAIL /lidar_points rate unavailable"
  status=1
fi

exit "${status}"

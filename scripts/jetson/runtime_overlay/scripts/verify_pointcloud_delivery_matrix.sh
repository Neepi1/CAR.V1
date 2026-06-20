#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

AXIS_STATUS_TOPIC="${NJRH_VERIFY_MATRIX_AXIS_STATUS_TOPIC:-/lidar/axis_remap_status}"
NAV_STATUS_TOPIC="${NJRH_VERIFY_MATRIX_NAV_STATUS_TOPIC:-/lidar/nav_cloud_preprocessor_status}"
LIDAR_POINTS_TOPIC="${NJRH_VERIFY_MATRIX_LIDAR_POINTS_TOPIC:-/lidar_points}"
SCAN_TOPIC="${NJRH_VERIFY_MATRIX_SCAN_TOPIC:-/scan}"
FLATSCAN_TOPIC="${NJRH_VERIFY_MATRIX_FLATSCAN_TOPIC:-/flatscan}"
STATUS_TIMEOUT_SEC="${NJRH_VERIFY_MATRIX_STATUS_TIMEOUT_SEC:-8}"
LIGHT_TOPIC_HZ_SEC="${NJRH_VERIFY_MATRIX_LIGHT_TOPIC_HZ_SEC:-6}"
MIN_AXIS_HZ="${NJRH_VERIFY_MATRIX_MIN_AXIS_HZ:-18.0}"
MIN_NAV_INPUT_HZ="${NJRH_VERIFY_MATRIX_MIN_NAV_INPUT_HZ:-8.0}"
MIN_NAV_OUTPUT_HZ="${NJRH_VERIFY_MATRIX_MIN_NAV_OUTPUT_HZ:-8.0}"
MIN_SCAN_HZ="${NJRH_VERIFY_MATRIX_MIN_SCAN_HZ:-8.0}"
DROP_RATIO="${NJRH_VERIFY_MATRIX_DROP_RATIO:-0.70}"
LIDAR_POINTS_CLI_HZ="${NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ:-}"

tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_matrix_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

float_ge() {
  awk -v value="${1:-nan}" -v minimum="${2:-0}" 'BEGIN { exit(value + 0.0 >= minimum + 0.0 ? 0 : 1) }'
}

float_lt() {
  awk -v value="${1:-nan}" -v limit="${2:-0}" 'BEGIN { exit(value + 0.0 < limit + 0.0 ? 0 : 1) }'
}

float_drop_from_baseline() {
  awk -v current="${1:-nan}" -v baseline="${2:-nan}" -v ratio="${3:-0.70}" \
    'BEGIN { exit((baseline + 0.0) > 0.0 && (current + 0.0) < (baseline + 0.0) * (ratio + 0.0) ? 0 : 1) }'
}

topic_exists() {
  timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

topic_graph_counts() {
  local topic="$1"
  local label="$2"
  local info_file="${tmp_dir}/${label}.topic_info"
  timeout 8 ros2 topic info -v "${topic}" >"${info_file}" 2>&1 || true
  local publisher_count
  local subscription_count
  publisher_count="$(awk -F: '$1 == "Publisher count" {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}' "${info_file}")"
  subscription_count="$(awk -F: '$1 == "Subscription count" {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}' "${info_file}")"
  publisher_count="${publisher_count:-0}"
  subscription_count="${subscription_count:-0}"
  printf '%s\n' "${publisher_count}" >"${tmp_dir}/${label}.publisher_count"
  printf '%s\n' "${subscription_count}" >"${tmp_dir}/${label}.subscription_count"
  echo "[pointcloud-matrix] ${topic}: publishers=${publisher_count} subscribers=${subscription_count}"
}

status_once() {
  local topic="$1"
  local label="$2"
  local raw_file="${tmp_dir}/${label}.status.raw"
  local samples_file="${tmp_dir}/${label}.status.samples"
  local latest_file="${tmp_dir}/${label}.status"
  timeout "${STATUS_TIMEOUT_SEC}" ros2 topic echo "${topic}" --field data >"${raw_file}" 2>&1 || true
  local data
  awk 'NF && $0 != "---" {print}' "${raw_file}" >"${samples_file}"
  data="$(tail -n 1 "${samples_file}" 2>/dev/null || true)"
  if [[ -z "${data}" ]]; then
    sed 's/^/[pointcloud-matrix]   /' "${raw_file}" >&2 || true
    if topic_exists "${topic}"; then
      echo "[pointcloud-matrix] ${topic}: NO_STATUS_SAMPLE"
    else
      echo "[pointcloud-matrix] ${topic}: NOT_PRESENT"
    fi
    return 1
  fi
  printf '%s\n' "${data}" >"${latest_file}"
  echo "[pointcloud-matrix] ${topic}: samples=$(wc -l <"${samples_file}") latest=${data}"
}

field_value() {
  local text="$1"
  local key="$2"
  printf '%s\n' "${text}" | tr ' ' '\n' | awk -F= -v key="${key}" '$1 == key {print substr($0, length(key) + 2); exit}'
}

status_field() {
  local label="$1"
  local key="$2"
  local file="${tmp_dir}/${label}.status"
  [[ -s "${file}" ]] || return 1
  field_value "$(<"${file}")" "${key}"
}

status_field_avg() {
  local label="$1"
  local key="$2"
  local file="${tmp_dir}/${label}.status.samples"
  [[ -s "${file}" ]] || return 1
  awk -v key="${key}" '
    {
      for (i = 1; i <= NF; i += 1) {
        split($i, kv, "=")
        if (kv[1] == key && kv[2] ~ /^-?[0-9]+([.][0-9]+)?$/) {
          sum += kv[2]
          count += 1
        }
      }
    }
    END {
      if (count > 0) {
        printf "%.3f", sum / count
      } else {
        exit 1
      }
    }
  ' "${file}"
}

check_ge() {
  local label="$1"
  local value="$2"
  local minimum="$3"
  if [[ -z "${value}" ]]; then
    echo "[pointcloud-matrix] FAIL ${label}: missing"
    return 1
  fi
  if float_ge "${value}" "${minimum}"; then
    echo "[pointcloud-matrix] PASS ${label}: ${value} >= ${minimum}"
    return 0
  fi
  echo "[pointcloud-matrix] FAIL ${label}: ${value} < ${minimum}"
  return 1
}

measure_light_topic_hz() {
  local topic="$1"
  local label="$2"
  local output_file="${tmp_dir}/${label}.hz"
  if ! topic_exists "${topic}"; then
    echo "[pointcloud-matrix] ${topic}: NOT_PRESENT"
    return 1
  fi
  timeout "${LIGHT_TOPIC_HZ_SEC}" ros2 topic hz "${topic}" >"${output_file}" 2>&1 || true
  local rate
  rate="$(awk '/average rate:/ {rate=$3} END {print rate}' "${output_file}")"
  if [[ -z "${rate}" ]]; then
    sed 's/^/[pointcloud-matrix]   /' "${output_file}" >&2 || true
    echo "[pointcloud-matrix] ${topic}: NO_SAMPLES"
    return 1
  fi
  printf '%s\n' "${rate}" >"${tmp_dir}/${label}.rate"
  echo "[pointcloud-matrix] ${topic}: ${rate} Hz"
}

echo "[pointcloud-matrix] This script only observes status topics and light scan topics."
echo "[pointcloud-matrix] It does not start/stop Nav2, change QoS, change stamps, or record rosbag."
echo "[pointcloud-matrix] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[pointcloud-matrix] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"
echo "[pointcloud-matrix] NJRH_FASTDDS_PROFILE_ENABLED=${NJRH_FASTDDS_PROFILE_ENABLED:-unset}"
echo "[pointcloud-matrix] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"
echo "[pointcloud-matrix] FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-unset}"
echo "[pointcloud-matrix] retired local PointCloud2 obstacle branch is not sampled"
echo "[pointcloud-matrix] Manual A/B modes to run:"
echo "[pointcloud-matrix]   mode1: driver + axis_remap only"
echo "[pointcloud-matrix]   mode2: driver + axis_remap + scan worker"
echo "[pointcloud-matrix]   mode3: driver + axis_remap + scan + flatscan helper"

status=0
axis_ok=0
nav_ok=0

topic_graph_counts "${LIDAR_POINTS_TOPIC}" "lidar_points"

fastlio_resident="$(pgrep -af 'fastlio|fast_lio|fast-lio' 2>/dev/null || true)"
if [[ -n "${fastlio_resident}" && "${NJRH_MAPPING_ACTIVE:-false}" != "true" && "${NJRH_ALLOW_FASTLIO_NAV_RESIDUE:-false}" != "true" ]]; then
  echo "[pointcloud-matrix] WARN FAST-LIO2-like process is present outside explicit mapping/diagnostic mode:"
  printf '%s\n' "${fastlio_resident}" | sed 's/^/[pointcloud-matrix]   /'
else
  echo "[pointcloud-matrix] FAST-LIO2 navigation residue: none"
fi

status_once "${AXIS_STATUS_TOPIC}" "axis" && axis_ok=1 || status=1
status_once "${NAV_STATUS_TOPIC}" "nav" && nav_ok=1 || true

axis_raw_hz="$(status_field_avg axis raw_input_hz || true)"
axis_lidar_hz="$(status_field_avg axis lidar_points_publish_hz || true)"
axis_subscribers="$(status_field axis output_subscription_count || true)"
lidar_points_publishers="$(<"${tmp_dir}/lidar_points.publisher_count")"
lidar_points_subscribers="$(<"${tmp_dir}/lidar_points.subscription_count")"
nav_input_hz="$(status_field_avg nav input_callback_hz || true)"
nav_output_hz="$(status_field_avg nav output_points_nav_hz || true)"

if [[ "${axis_ok}" -eq 1 ]]; then
  check_ge "axis raw_input_hz" "${axis_raw_hz}" "${MIN_AXIS_HZ}" || status=1
  check_ge "axis lidar_points_publish_hz" "${axis_lidar_hz}" "${MIN_AXIS_HZ}" || status=1
  echo "[pointcloud-matrix] axis output_subscription_count=${axis_subscribers:-missing}"
  echo "[pointcloud-matrix] ${LIDAR_POINTS_TOPIC} graph_subscription_count=${lidar_points_subscribers:-missing}"
fi

if [[ "${nav_ok}" -eq 1 ]]; then
  check_ge "nav input_callback_hz" "${nav_input_hz}" "${MIN_NAV_INPUT_HZ}" || status=1
  check_ge "nav output_points_nav_hz" "${nav_output_hz}" "${MIN_NAV_OUTPUT_HZ}" || status=1
fi

if [[ "${nav_ok}" -eq 1 ]]; then
  measure_light_topic_hz "${SCAN_TOPIC}" "scan" || true
  if [[ -s "${tmp_dir}/scan.rate" ]]; then
    check_ge "scan hz" "$(<"${tmp_dir}/scan.rate")" "${MIN_SCAN_HZ}" || status=1
  fi
  measure_light_topic_hz "${FLATSCAN_TOPIC}" "flatscan" || true
  if [[ -s "${tmp_dir}/flatscan.rate" ]]; then
    check_ge "flatscan hz" "$(<"${tmp_dir}/flatscan.rate")" "${MIN_SCAN_HZ}" || status=1
  fi
fi

echo "[pointcloud-matrix] diagnosis:"
case_result="CASE_INCONCLUSIVE"
if [[ -n "${axis_lidar_hz}" ]] && float_lt "${axis_lidar_hz}" "${MIN_AXIS_HZ}"; then
  case_result="CASE_A_MAIN_TRUNK_LOW"
elif [[ -n "${axis_lidar_hz}" ]] &&
  float_ge "${axis_lidar_hz}" "${MIN_AXIS_HZ}" &&
  [[ "${lidar_points_subscribers:-0}" -gt 2 ]]
then
  case_result="CASE_E_TOO_MANY_FULL_DENSITY_SUBSCRIBERS"
elif [[ -n "${axis_lidar_hz}" && -n "${LIDAR_POINTS_CLI_HZ}" ]] &&
  float_ge "${axis_lidar_hz}" "${MIN_AXIS_HZ}" &&
  [[ "${lidar_points_subscribers:-0}" -le 2 ]] &&
  float_lt "${LIDAR_POINTS_CLI_HZ}" "${MIN_AXIS_HZ}"
then
  case_result="CASE_G_DDS_TRANSPORT_SUSPECT"
fi

local_baseline="${NJRH_VERIFY_MATRIX_LOCAL_ONLY_INPUT_HZ:-}"
nav_baseline="${NJRH_VERIFY_MATRIX_NAV_ONLY_INPUT_HZ:-}"
if [[ -n "${local_baseline}" && -n "${nav_baseline}" && -n "${axis_lidar_hz}" && -n "${nav_input_hz}" ]] &&
  float_drop_from_baseline "${axis_lidar_hz}" "${local_baseline}" "${DROP_RATIO}" &&
  float_drop_from_baseline "${nav_input_hz}" "${nav_baseline}" "${DROP_RATIO}"
then
  case_result="CASE_D_FANOUT_PRESSURE"
fi

echo "[pointcloud-matrix] ${case_result}"
echo "[pointcloud-matrix] CASE_A_MAIN_TRUNK_LOW: axis lidar_points_publish_hz < ${MIN_AXIS_HZ}"
echo "[pointcloud-matrix] CASE_D_FANOUT_PRESSURE: compare mode2/mode3 baselines using NJRH_VERIFY_MATRIX_LOCAL_ONLY_INPUT_HZ and NJRH_VERIFY_MATRIX_NAV_ONLY_INPUT_HZ"
echo "[pointcloud-matrix] CASE_E_TOO_MANY_FULL_DENSITY_SUBSCRIBERS: axis >= ${MIN_AXIS_HZ}, /lidar_points subscribers > 2"
echo "[pointcloud-matrix] CASE_G_DDS_TRANSPORT_SUSPECT: axis >= ${MIN_AXIS_HZ}, /lidar_points subscribers <= 2, and manual /lidar_points CLI hz is low"
if [[ -z "${LIDAR_POINTS_CLI_HZ}" ]]; then
  echo "[pointcloud-matrix] CASE_G helper: set NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ=<manual ros2 topic hz /lidar_points average> to classify DDS transport suspicion without this script creating a full-density subscriber."
fi
if [[ "${case_result}" == "CASE_G_DDS_TRANSPORT_SUSPECT" && "${lidar_points_subscribers:-0}" -le 2 ]]; then
  echo "[pointcloud-matrix] Suggestion: subscriber count is not high; run run_pointcloud_dds_transport_ab.sh for controlled DDS transport A/B before changing QoS or timestamps."
fi

exit "${status}"

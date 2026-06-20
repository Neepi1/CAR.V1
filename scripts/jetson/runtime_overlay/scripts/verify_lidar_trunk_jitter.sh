#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

RAW_TOPIC="${NJRH_VERIFY_TRUNK_RAW_TOPIC:-/jt128/vendor/points_raw}"
AXIS_STATUS_TOPIC="${NJRH_VERIFY_TRUNK_AXIS_STATUS_TOPIC:-/lidar/axis_remap_status}"
RAW_HZ_SECONDS="${NJRH_VERIFY_TRUNK_RAW_HZ_SECONDS:-6}"
STATUS_SECONDS="${NJRH_VERIFY_TRUNK_STATUS_SECONDS:-10}"
MIN_RAW_HZ="${NJRH_VERIFY_TRUNK_MIN_RAW_HZ:-18.0}"
MIN_LIDAR_HZ="${NJRH_VERIFY_TRUNK_MIN_LIDAR_HZ:-18.0}"
MIN_NAV_BRANCH_HZ="${NJRH_VERIFY_TRUNK_MIN_NAV_BRANCH_HZ:-8.0}"
MAX_NAV_BRANCH_HZ="${NJRH_VERIFY_TRUNK_MAX_NAV_BRANCH_HZ:-12.0}"

tmp_dir="$(mktemp -d /tmp/njrh_lidar_trunk_jitter_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

float_ge() {
  awk -v value="${1:-nan}" -v minimum="${2:-0}" 'BEGIN { exit(value + 0.0 >= minimum + 0.0 ? 0 : 1) }'
}

float_le() {
  awk -v value="${1:-nan}" -v maximum="${2:-0}" 'BEGIN { exit(value + 0.0 <= maximum + 0.0 ? 0 : 1) }'
}

field_value() {
  local text="$1"
  local key="$2"
  printf '%s\n' "${text}" | tr ' ' '\n' | awk -F= -v key="${key}" '$1 == key {print substr($0, length(key) + 2); exit}'
}

field_avg() {
  local file="$1"
  local key="$2"
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
    echo "[lidar-trunk-jitter] FAIL ${label}: missing"
    return 1
  fi
  if float_ge "${value}" "${minimum}"; then
    echo "[lidar-trunk-jitter] PASS ${label}: ${value} >= ${minimum}"
    return 0
  fi
  echo "[lidar-trunk-jitter] FAIL ${label}: ${value} < ${minimum}"
  return 1
}

check_between() {
  local label="$1"
  local value="$2"
  local minimum="$3"
  local maximum="$4"
  if [[ -z "${value}" ]]; then
    echo "[lidar-trunk-jitter] FAIL ${label}: missing"
    return 1
  fi
  if float_ge "${value}" "${minimum}" && float_le "${value}" "${maximum}"; then
    echo "[lidar-trunk-jitter] PASS ${label}: ${value} in [${minimum}, ${maximum}]"
    return 0
  fi
  echo "[lidar-trunk-jitter] FAIL ${label}: ${value} outside [${minimum}, ${maximum}]"
  return 1
}

measure_raw_hz() {
  local output_file="${tmp_dir}/raw.hz"
  echo "[lidar-trunk-jitter] Measuring ${RAW_TOPIC} with ros2 topic hz for ${RAW_HZ_SECONDS}s."
  echo "[lidar-trunk-jitter] This is a temporary PointCloud2 subscriber; stop it after field checks."
  timeout "${RAW_HZ_SECONDS}" ros2 topic hz "${RAW_TOPIC}" >"${output_file}" 2>&1 || true
  awk '/average rate:/ {rate=$3} END {print rate}' "${output_file}"
}

collect_axis_status() {
  local raw_file="${tmp_dir}/axis.raw"
  local samples_file="${tmp_dir}/axis.samples"
  timeout "${STATUS_SECONDS}" ros2 topic echo "${AXIS_STATUS_TOPIC}" --field data >"${raw_file}" 2>&1 || true
  awk 'NF && $0 != "---" {print}' "${raw_file}" >"${samples_file}"
  if [[ ! -s "${samples_file}" ]]; then
    sed 's/^/[lidar-trunk-jitter]   /' "${raw_file}" >&2 || true
    return 1
  fi
  printf '%s\n' "${samples_file}"
}

status=0
raw_rate="$(measure_raw_hz)"
if [[ -z "${raw_rate}" ]]; then
  echo "[lidar-trunk-jitter] WARN raw ${RAW_TOPIC} CLI hz unavailable; relying on axis raw_input_hz"
elif float_ge "${raw_rate}" "${MIN_RAW_HZ}"; then
  echo "[lidar-trunk-jitter] PASS raw ${RAW_TOPIC} CLI hz: ${raw_rate} >= ${MIN_RAW_HZ}"
else
  echo "[lidar-trunk-jitter] WARN raw ${RAW_TOPIC} CLI hz: ${raw_rate} < ${MIN_RAW_HZ}"
  echo "[lidar-trunk-jitter] WARN raw CLI measurement is a temporary full-density subscriber; final source-side judgment uses axis raw_input_hz"
fi

axis_samples="$(collect_axis_status || true)"
if [[ -z "${axis_samples}" ]]; then
  echo "[lidar-trunk-jitter] FAIL ${AXIS_STATUS_TOPIC}: no status samples"
  exit 1
fi

latest="$(tail -n 1 "${axis_samples}")"
raw_input_hz="$(field_avg "${axis_samples}" raw_input_hz || true)"
lidar_publish_hz="$(field_avg "${axis_samples}" lidar_points_publish_hz || true)"
nav_publish_hz="$(field_avg "${axis_samples}" nav_branch_publish_hz || true)"

echo "[lidar-trunk-jitter] ${AXIS_STATUS_TOPIC}: samples=$(wc -l <"${axis_samples}")"
echo "[lidar-trunk-jitter] latest=${latest}"
check_ge "axis raw_input_hz" "${raw_input_hz}" "${MIN_RAW_HZ}" || status=1
check_ge "axis lidar_points_publish_hz" "${lidar_publish_hz}" "${MIN_LIDAR_HZ}" || status=1

nav_enabled="$(field_value "${latest}" nav_branch_enabled)"
nav_subscribers="$(field_value "${latest}" nav_branch_subscription_count)"
if [[ "${nav_enabled}" == "true" && "${nav_subscribers:-0}" -gt 0 ]]; then
  check_between "nav_branch_publish_hz" "${nav_publish_hz}" "${MIN_NAV_BRANCH_HZ}" "${MAX_NAV_BRANCH_HZ}" || status=1
fi

echo "[lidar-trunk-jitter] axis output_subscription_count=$(field_value "${latest}" output_subscription_count)"
echo "[lidar-trunk-jitter] nav_branch_enabled=${nav_enabled:-missing}"
echo "[lidar-trunk-jitter] nav_branch_subscription_count=${nav_subscribers:-missing}"
echo "[lidar-trunk-jitter] nav_output_stride=$(field_value "${latest}" nav_output_stride)"
echo "[lidar-trunk-jitter] nav_output_publish_every_n=$(field_value "${latest}" nav_output_publish_every_n)"
echo "[lidar-trunk-jitter] last_trunk_publish_duration_ms=$(field_value "${latest}" last_trunk_publish_duration_ms)"
echo "[lidar-trunk-jitter] last_branch_publish_duration_ms=$(field_value "${latest}" last_branch_publish_duration_ms)"
echo "[lidar-trunk-jitter] last_total_publish_outputs_duration_ms=$(field_value "${latest}" last_total_publish_outputs_duration_ms)"

cat <<'EOF'
[lidar-trunk-jitter] Optional CLI receive checks:
[lidar-trunk-jitter]   ros2 topic hz /lidar_points
[lidar-trunk-jitter] This creates a full-density subscriber and can add pressure. Prefer /lidar/axis_remap_status for final publish-side judgment.
[lidar-trunk-jitter] Obstacle follow-up:
[lidar-trunk-jitter]   ros2 topic echo /lidar/pointcloud_accel_status --field data
[lidar-trunk-jitter]   ros2 topic hz /scan
[lidar-trunk-jitter] Watch scan_publish_hz, scan_output_header_age_ms, and scan_output_source_age_ms.
EOF

if [[ "${status}" -eq 0 ]]; then
  echo "[lidar-trunk-jitter] PASS"
else
  echo "[lidar-trunk-jitter] FAIL"
fi
exit "${status}"

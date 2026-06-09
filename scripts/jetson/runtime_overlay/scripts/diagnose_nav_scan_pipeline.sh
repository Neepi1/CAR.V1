#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

STATUS_SECONDS="${NJRH_NAV_SCAN_DIAG_STATUS_SECONDS:-12}"
TOPIC_HZ_SECONDS="${NJRH_NAV_SCAN_DIAG_TOPIC_HZ_SECONDS:-8}"
AXIS_STATUS_TOPIC="${NJRH_NAV_SCAN_DIAG_AXIS_STATUS_TOPIC:-/lidar/axis_remap_status}"
NAV_STATUS_TOPIC="${NJRH_NAV_SCAN_DIAG_STATUS_TOPIC:-/lidar/nav_cloud_preprocessor_status}"
LIDAR_POINTS_NAV_TOPIC="${NJRH_NAV_SCAN_DIAG_LIDAR_POINTS_NAV_TOPIC:-/lidar_points_nav}"
POINTS_NAV_TOPIC="${NJRH_NAV_SCAN_DIAG_POINTS_NAV_TOPIC:-/points_nav}"
SCAN_TOPIC="${NJRH_NAV_SCAN_DIAG_SCAN_TOPIC:-/scan}"
FLATSCAN_TOPIC="${NJRH_NAV_SCAN_DIAG_FLATSCAN_TOPIC:-/flatscan}"

tmp_dir="$(mktemp -d /tmp/njrh_nav_scan_diag_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

float_ge() {
  awk -v value="${1:-nan}" -v minimum="${2:-0}" 'BEGIN {exit(value + 0.0 >= minimum + 0.0 ? 0 : 1)}'
}

float_lt() {
  awk -v value="${1:-nan}" -v limit="${2:-0}" 'BEGIN {exit(value + 0.0 < limit + 0.0 ? 0 : 1)}'
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

field_latest() {
  local file="$1"
  local key="$2"
  awk -v key="${key}" '
    {
      for (i = 1; i <= NF; i += 1) {
        split($i, kv, "=")
        if (kv[1] == key) {
          value = substr($i, length(key) + 2)
        }
      }
    }
    END {
      if (value != "") {
        print value
      } else {
        exit 1
      }
    }
  ' "${file}"
}

field_delta() {
  local file="$1"
  local key="$2"
  awk -v key="${key}" '
    {
      for (i = 1; i <= NF; i += 1) {
        split($i, kv, "=")
        if (kv[1] == key && kv[2] ~ /^-?[0-9]+([.][0-9]+)?$/) {
          if (!seen) {
            first = kv[2]
            seen = 1
          }
          last = kv[2]
        }
      }
    }
    END {
      if (seen) {
        printf "%.3f", last - first
      } else {
        exit 1
      }
    }
  ' "${file}"
}

status_samples() {
  local topic="$1"
  local label="$2"
  local raw_file="${tmp_dir}/${label}.raw"
  local samples_file="${tmp_dir}/${label}.samples"
  timeout "${STATUS_SECONDS}" ros2 topic echo "${topic}" --field data >"${raw_file}" 2>&1 || true
  awk 'NF && $0 != "---" {print}' "${raw_file}" >"${samples_file}"
  if [[ ! -s "${samples_file}" ]]; then
    echo "[nav-scan-diag] WARN ${topic}: no status samples" >&2
    sed 's/^/[nav-scan-diag]   /' "${raw_file}" >&2 || true
  fi
  printf '%s\n' "${samples_file}"
}

topic_exists() {
  timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

count_from_topic_info() {
  local file="$1"
  local label="$2"
  awk -F: -v label="${label}" '$1 == label {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}' "${file}"
}

topic_info_file() {
  local topic="$1"
  local label="$2"
  local file="${tmp_dir}/${label}.info"
  ros2 topic info -v "${topic}" >"${file}" 2>&1 || true
  printf '%s\n' "${file}"
}

measure_topic_hz() {
  local topic="$1"
  local label="$2"
  local file="${tmp_dir}/${label}.hz"
  if ! topic_exists "${topic}"; then
    echo "[nav-scan-diag] ${topic}: NOT_PRESENT"
    return 1
  fi
  timeout "${TOPIC_HZ_SECONDS}" ros2 topic hz "${topic}" >"${file}" 2>&1 || true
  local rate
  rate="$(awk '/average rate:/ {rate=$3} END {print rate}' "${file}")"
  if [[ -z "${rate}" ]]; then
    echo "[nav-scan-diag] ${topic}: NO_HZ_SAMPLE"
    sed 's/^/[nav-scan-diag]   /' "${file}" >&2 || true
    return 1
  fi
  printf '%s\n' "${rate}" >"${tmp_dir}/${label}.rate"
  echo "[nav-scan-diag] ${topic}: ${rate} Hz"
}

pid_csv_for_pattern() {
  local pattern="$1"
  pgrep -f "${pattern}" 2>/dev/null | sort -n | paste -sd ',' -
}

cpus_for_pattern() {
  local pattern="$1"
  local pids
  pids="$(pid_csv_for_pattern "${pattern}")"
  [[ -n "${pids}" ]] || return 0
  ps -T -p "${pids}" -o psr --no-headers 2>/dev/null |
    awk 'NF {seen[$1]=1} END {for (cpu in seen) print cpu}' | sort -n | paste -sd ',' -
}

sets_overlap() {
  local lhs="$1"
  local rhs="$2"
  awk -v lhs="${lhs}" -v rhs="${rhs}" '
    BEGIN {
      split(lhs, a, ",")
      split(rhs, b, ",")
      for (i in a) seen[a[i]] = 1
      for (i in b) {
        if (b[i] in seen) exit 0
      }
      exit 1
    }'
}

check_tf() {
  local target="$1"
  local source="$2"
  if runtime_readiness_probe tf "${target}" "${source}" 3 >/tmp/njrh_nav_scan_tf.out 2>&1; then
    echo "[nav-scan-diag] TF ${target}<-${source}: PASS"
    return 0
  fi
  echo "[nav-scan-diag] TF ${target}<-${source}: WARN"
  sed 's/^/[nav-scan-diag]   /' /tmp/njrh_nav_scan_tf.out >&2 || true
  return 1
}

echo "[nav-scan-diag] Observes reduced nav/scan topics only; it does not subscribe to /lidar_points."
echo "[nav-scan-diag] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"

axis_samples="$(status_samples "${AXIS_STATUS_TOPIC}" axis)"
nav_samples="$(status_samples "${NAV_STATUS_TOPIC}" nav)"

for item in \
  "${LIDAR_POINTS_NAV_TOPIC}:lidar_points_nav" \
  "${POINTS_NAV_TOPIC}:points_nav" \
  "${SCAN_TOPIC}:scan" \
  "${FLATSCAN_TOPIC}:flatscan"; do
  topic="${item%%:*}"
  label="${item##*:}"
  info_file="$(topic_info_file "${topic}" "${label}")"
  publishers="$(count_from_topic_info "${info_file}" "Publisher count")"
  subscribers="$(count_from_topic_info "${info_file}" "Subscription count")"
  echo "[nav-scan-diag] ${topic}: publishers=${publishers:-0} subscribers=${subscribers:-0}"
done

measure_topic_hz "${LIDAR_POINTS_NAV_TOPIC}" lidar_points_nav || true
measure_topic_hz "${POINTS_NAV_TOPIC}" points_nav || true
measure_topic_hz "${SCAN_TOPIC}" scan || true
measure_topic_hz "${FLATSCAN_TOPIC}" flatscan || true

check_tf "lidar_level_link" "lidar_link" || true
check_tf "base_link" "lidar_level_link" || true
check_tf "map" "base_link" || true

axis_nav_hz="$(field_avg "${axis_samples}" nav_branch_publish_hz 2>/dev/null || true)"
axis_nav_enabled="$(field_latest "${axis_samples}" nav_branch_enabled 2>/dev/null || true)"
nav_input_topic="$(field_latest "${nav_samples}" input_topic 2>/dev/null || true)"
nav_output_topic="$(field_latest "${nav_samples}" output_topic 2>/dev/null || true)"
nav_input_hz="$(field_avg "${nav_samples}" input_callback_hz 2>/dev/null || true)"
nav_accept_hz="$(field_avg "${nav_samples}" input_accept_hz 2>/dev/null || field_avg "${nav_samples}" input_callback_hz 2>/dev/null || true)"
nav_output_hz="$(field_avg "${nav_samples}" output_points_nav_hz 2>/dev/null || true)"
nav_processing_avg="$(field_avg "${nav_samples}" processing_ms_avg 2>/dev/null || true)"
nav_processing_max="$(field_avg "${nav_samples}" processing_ms_max 2>/dev/null || true)"
nav_last_output_points="$(field_latest "${nav_samples}" last_output_points 2>/dev/null || field_latest "${nav_samples}" output_cloud_points 2>/dev/null || true)"
nav_target_frame="$(field_latest "${nav_samples}" target_frame 2>/dev/null || true)"
nav_source_frame="$(field_latest "${nav_samples}" source_frame 2>/dev/null || true)"
skip_transform_delta="$(field_delta "${nav_samples}" skipped_transform 2>/dev/null || true)"
skip_empty_delta="$(field_delta "${nav_samples}" skipped_empty 2>/dev/null || true)"
skip_filter_empty_delta="$(field_delta "${nav_samples}" skipped_filter_empty 2>/dev/null || true)"
points_nav_hz="$(cat "${tmp_dir}/points_nav.rate" 2>/dev/null || true)"
scan_hz="$(cat "${tmp_dir}/scan.rate" 2>/dev/null || true)"
flatscan_hz="$(cat "${tmp_dir}/flatscan.rate" 2>/dev/null || true)"

echo "[nav-scan-diag] status axis_nav_branch_enabled=${axis_nav_enabled:-missing} axis_nav_branch_publish_hz=${axis_nav_hz:-missing}"
echo "[nav-scan-diag] status input_topic=${nav_input_topic:-missing} output_topic=${nav_output_topic:-missing} input_hz=${nav_input_hz:-missing} accept_hz=${nav_accept_hz:-missing} output_hz=${nav_output_hz:-missing}"
echo "[nav-scan-diag] status frames source=${nav_source_frame:-missing} target=${nav_target_frame:-missing} last_output_points=${nav_last_output_points:-missing}"
echo "[nav-scan-diag] status processing_ms_avg=${nav_processing_avg:-missing} processing_ms_max=${nav_processing_max:-missing}"
echo "[nav-scan-diag] status deltas skipped_transform=${skip_transform_delta:-missing} skipped_empty=${skip_empty_delta:-missing} skipped_filter_empty=${skip_filter_empty_delta:-missing}"

nav_cpus="$(cpus_for_pattern "nav_cloud_preprocessor|pointcloud_to_laserscan")"
localizer_cpus="$(cpus_for_pattern "scan_republisher|laser_scan_to_flatscan|occupancy_grid_localizer|isaac.*localizer|robot_global_localization")"
cpu_overlap=false
if [[ -n "${nav_cpus}" && -n "${localizer_cpus}" ]] && sets_overlap "${nav_cpus}" "${localizer_cpus}"; then
  cpu_overlap=true
fi
echo "[nav-scan-diag] cpu_quick nav_preprocessor_or_converter_cpus=${nav_cpus:-missing} scan_or_localizer_cpus=${localizer_cpus:-missing} overlap=${cpu_overlap}"

case_result="CASE_NAV_UNCLASSIFIED"
if [[ -n "${axis_nav_hz}" && -n "${nav_input_hz}" ]] &&
  float_ge "${axis_nav_hz}" 8.5 && float_lt "${nav_input_hz}" 7.0
then
  case_result="CASE_NAV_A_BRANCH_INPUT_LOW"
elif [[ -n "${nav_input_hz}" && -n "${nav_output_hz}" ]] &&
  float_ge "${nav_input_hz}" 8.0 && float_lt "${nav_output_hz}" 7.0
then
  case_result="CASE_NAV_B_PREPROCESSOR_OUTPUT_LOW"
elif [[ -n "${points_nav_hz}" ]] &&
  float_ge "${points_nav_hz}" 8.0 &&
  { float_lt "${scan_hz:-0}" 7.0 || float_lt "${flatscan_hz:-0}" 7.0; }
then
  case_result="CASE_NAV_C_SCAN_CONVERSION_LOW"
elif float_ge "${skip_transform_delta:-0}" 1.0
then
  case_result="CASE_NAV_D_TF_SKIPS"
elif [[ -n "${nav_input_hz}" && -n "${nav_last_output_points}" ]] &&
  float_ge "${nav_input_hz}" 8.0 &&
  { float_lt "${nav_last_output_points}" 1.0 || float_ge "${skip_filter_empty_delta:-0}" 1.0; }
then
  case_result="CASE_NAV_F_EMPTY_FILTER"
elif [[ -n "${nav_processing_avg}" && -n "${nav_input_hz}" && -n "${nav_output_hz}" ]] &&
  float_lt "${nav_processing_avg}" 10.0 &&
  { float_lt "${nav_input_hz}" 8.0 || float_lt "${nav_output_hz}" 8.0; } &&
  [[ "${cpu_overlap}" == "true" ]]
then
  case_result="CASE_NAV_E_CPU_CONTENTION"
fi

echo "[nav-scan-diag] CASE=${case_result}"
echo "[nav-scan-diag] next: run diagnose_pointcloud_cpu_pressure.sh if CPU contention is suspected."

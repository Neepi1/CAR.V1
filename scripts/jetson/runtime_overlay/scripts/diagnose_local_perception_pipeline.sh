#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"
njrh_load_local_perception_input_profile

STATUS_SECONDS="${NJRH_LOCAL_PERCEPTION_DIAG_STATUS_SECONDS:-12}"
TOPIC_HZ_SECONDS="${NJRH_LOCAL_PERCEPTION_DIAG_TOPIC_HZ_SECONDS:-8}"
AXIS_STATUS_TOPIC="${NJRH_LOCAL_PERCEPTION_DIAG_AXIS_STATUS_TOPIC:-/lidar/axis_remap_status}"
LOCAL_STATUS_TOPIC="${NJRH_LOCAL_PERCEPTION_DIAG_STATUS_TOPIC:-/perception/local_perception_status}"
LOCAL_BRANCH_TOPIC="${NJRH_LOCAL_PERCEPTION_DIAG_LOCAL_BRANCH_TOPIC:-/_internal/lidar_points_local}"
OBSTACLE_TOPIC="${NJRH_LOCAL_PERCEPTION_DIAG_OBSTACLE_TOPIC:-/perception/obstacle_points}"
CLEARING_TOPIC="${NJRH_LOCAL_PERCEPTION_DIAG_CLEARING_TOPIC:-/perception/clearing_points}"

tmp_dir="$(mktemp -d /tmp/njrh_local_perception_diag_XXXXXX)"
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
    echo "[local-perception-diag] WARN ${topic}: no status samples" >&2
    sed 's/^/[local-perception-diag]   /' "${raw_file}" >&2 || true
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
    echo "[local-perception-diag] ${topic}: NOT_PRESENT"
    return 1
  fi
  timeout "${TOPIC_HZ_SECONDS}" ros2 topic hz "${topic}" >"${file}" 2>&1 || true
  local rate
  rate="$(awk '/average rate:/ {rate=$3} END {print rate}' "${file}")"
  if [[ -z "${rate}" ]]; then
    echo "[local-perception-diag] ${topic}: NO_HZ_SAMPLE"
    sed 's/^/[local-perception-diag]   /' "${file}" >&2 || true
    return 1
  fi
  printf '%s\n' "${rate}" >"${tmp_dir}/${label}.rate"
  echo "[local-perception-diag] ${topic}: ${rate} Hz"
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

echo "[local-perception-diag] Observes local/reduced branches only; it does not subscribe to /lidar_points."
echo "[local-perception-diag] profile=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE} resolved_input=${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}"
echo "[local-perception-diag] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"

axis_samples="$(status_samples "${AXIS_STATUS_TOPIC}" axis)"
local_samples="$(status_samples "${LOCAL_STATUS_TOPIC}" local)"

for item in \
  "${LOCAL_BRANCH_TOPIC}:local_branch" \
  "${OBSTACLE_TOPIC}:obstacle" \
  "${CLEARING_TOPIC}:clearing"; do
  topic="${item%%:*}"
  label="${item##*:}"
  info_file="$(topic_info_file "${topic}" "${label}")"
  publishers="$(count_from_topic_info "${info_file}" "Publisher count")"
  subscribers="$(count_from_topic_info "${info_file}" "Subscription count")"
  echo "[local-perception-diag] ${topic}: publishers=${publishers:-0} subscribers=${subscribers:-0}"
done

measure_topic_hz "${LOCAL_BRANCH_TOPIC}" local_branch || true
measure_topic_hz "${OBSTACLE_TOPIC}" obstacle || true
measure_topic_hz "${CLEARING_TOPIC}" clearing || true

axis_local_hz="$(field_avg "${axis_samples}" local_branch_publish_hz 2>/dev/null || true)"
axis_local_enabled="$(field_latest "${axis_samples}" local_branch_enabled 2>/dev/null || true)"
local_input_topic="$(field_latest "${local_samples}" input_topic 2>/dev/null || true)"
local_input_hz="$(field_avg "${local_samples}" input_callback_hz 2>/dev/null || true)"
local_accept_hz="$(field_avg "${local_samples}" input_cloud_accept_hz 2>/dev/null || true)"
local_timer_hz="$(field_avg "${local_samples}" timer_tick_hz 2>/dev/null || field_avg "${local_samples}" process_timer_hz 2>/dev/null || true)"
local_processed_hz="$(field_avg "${local_samples}" processed_cloud_hz 2>/dev/null || true)"
local_obstacle_hz="$(field_avg "${local_samples}" published_obstacle_hz 2>/dev/null || true)"
local_clearing_hz="$(field_avg "${local_samples}" published_clearing_hz 2>/dev/null || true)"
local_no_new_hz="$(field_avg "${local_samples}" no_new_hz 2>/dev/null || true)"
local_processing_avg="$(field_avg "${local_samples}" processing_ms_avg 2>/dev/null || true)"
local_processing_max="$(field_avg "${local_samples}" processing_ms_max 2>/dev/null || true)"
last_obstacle_points="$(field_latest "${local_samples}" last_obstacle_points 2>/dev/null || true)"
last_filtered_points="$(field_latest "${local_samples}" last_filtered_points 2>/dev/null || true)"
last_mode="$(field_latest "${local_samples}" last_mode 2>/dev/null || true)"
active_profile="$(field_latest "${local_samples}" active_profile_name 2>/dev/null || true)"
skip_transform_delta="$(field_delta "${local_samples}" skipped_transform 2>/dev/null || true)"
skip_empty_obstacle_delta="$(field_delta "${local_samples}" skipped_empty_obstacle 2>/dev/null || true)"
skip_publish_gating_delta="$(field_delta "${local_samples}" skipped_publish_gating 2>/dev/null || true)"

echo "[local-perception-diag] status axis_local_branch_enabled=${axis_local_enabled:-missing} axis_local_branch_publish_hz=${axis_local_hz:-missing}"
echo "[local-perception-diag] status input_topic=${local_input_topic:-missing} input_hz=${local_input_hz:-missing} accept_hz=${local_accept_hz:-missing} timer_hz=${local_timer_hz:-missing}"
echo "[local-perception-diag] status processed_hz=${local_processed_hz:-missing} obstacle_hz=${local_obstacle_hz:-missing} clearing_hz=${local_clearing_hz:-missing} no_new_hz=${local_no_new_hz:-missing}"
echo "[local-perception-diag] status last_filtered_points=${last_filtered_points:-missing} last_obstacle_points=${last_obstacle_points:-missing} mode=${last_mode:-missing} profile=${active_profile:-missing}"
echo "[local-perception-diag] status processing_ms_avg=${local_processing_avg:-missing} processing_ms_max=${local_processing_max:-missing}"
echo "[local-perception-diag] status deltas skipped_transform=${skip_transform_delta:-missing} skipped_empty_obstacle=${skip_empty_obstacle_delta:-missing} skipped_publish_gating=${skip_publish_gating_delta:-missing}"

local_cpus="$(cpus_for_pattern "local_perception_node|robot_local_perception")"
nav_cpus="$(cpus_for_pattern "nav_cloud_preprocessor|pointcloud_to_laserscan|occupancy_grid_localizer|isaac.*localizer|robot_global_localization")"
cpu_overlap=false
if [[ -n "${local_cpus}" && -n "${nav_cpus}" ]] && sets_overlap "${local_cpus}" "${nav_cpus}"; then
  cpu_overlap=true
fi
echo "[local-perception-diag] cpu_quick local_cpus=${local_cpus:-missing} nav_or_localizer_cpus=${nav_cpus:-missing} overlap=${cpu_overlap}"

case_result="CASE_LOCAL_UNCLASSIFIED"
if [[ -n "${axis_local_hz}" && -n "${local_input_hz}" ]] &&
  float_ge "${axis_local_hz}" 15.0 && float_lt "${local_input_hz}" 10.0
then
  case_result="CASE_LOCAL_A_BRANCH_INPUT_LOW"
elif [[ -n "${local_input_hz}" && -n "${local_processed_hz}" ]] &&
  float_ge "${local_input_hz}" 10.0 && float_lt "${local_processed_hz}" 10.0
then
  case_result="CASE_LOCAL_B_PROCESSING_LOW"
elif [[ -n "${local_processed_hz}" && -n "${local_obstacle_hz}" ]] &&
  float_ge "${local_processed_hz}" 10.0 && float_lt "${local_obstacle_hz}" 10.0 &&
  { float_ge "${skip_empty_obstacle_delta:-0}" 1.0 || float_ge "${skip_publish_gating_delta:-0}" 1.0; }
then
  case_result="CASE_LOCAL_C_OBSTACLE_PUBLISH_GATING"
elif [[ -n "${local_processed_hz}" && -n "${last_obstacle_points}" ]] &&
  float_ge "${local_processed_hz}" 10.0 && float_lt "${last_obstacle_points}" 1.0
then
  case_result="CASE_LOCAL_D_FILTER_EMPTY"
elif [[ -n "${local_processing_avg}" && -n "${local_timer_hz}" && -n "${local_processed_hz}" ]] &&
  float_lt "${local_processing_avg}" 10.0 &&
  { float_lt "${local_timer_hz}" 10.0 || float_lt "${local_processed_hz}" 10.0; } &&
  [[ "${cpu_overlap}" == "true" ]]
then
  case_result="CASE_LOCAL_E_CPU_CONTENTION"
elif [[ -n "${local_input_hz}" && -n "${local_processed_hz}" && -n "${local_obstacle_hz}" ]] &&
  float_ge "${local_input_hz}" 10.0 && float_ge "${local_processed_hz}" 10.0 && float_ge "${local_obstacle_hz}" 10.0 &&
  float_lt "${local_no_new_hz:-0}" 1.0 && float_lt "${skip_transform_delta:-0}" 1.0
then
  case_result="CASE_LOCAL_F_OK"
fi

echo "[local-perception-diag] CASE=${case_result}"
echo "[local-perception-diag] next: run diagnose_pointcloud_cpu_pressure.sh if CASE_LOCAL_E_CPU_CONTENTION or rates are low with normal processing_ms."

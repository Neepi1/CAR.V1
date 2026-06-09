#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"
njrh_load_local_perception_input_profile

INCLUDE_CLI_HZ=false
STATUS_SECONDS="${NJRH_DIAG_LIDAR_STATUS_SECONDS:-12}"
CLI_HZ_SECONDS="${NJRH_DIAG_LIDAR_CLI_HZ_SECONDS:-10}"
MIN_AXIS_HZ="${NJRH_DIAG_LIDAR_MIN_AXIS_HZ:-18.0}"
MIN_LOCAL_HZ="${NJRH_DIAG_LIDAR_MIN_LOCAL_HZ:-10.0}"
LOW_CLI_HZ="${NJRH_DIAG_LIDAR_LOW_CLI_HZ:-18.0}"

usage() {
  cat <<'EOF'
Usage: diagnose_lidar_points_jitter.sh [--include-cli-hz]

Default mode does not subscribe to full-density /lidar_points. It reads status
topics and ROS graph state to distinguish publish-side jitter from subscriber
delivery jitter.

Options:
  --include-cli-hz  Also run ros2 topic hz /lidar_points for comparison.
                   This creates a temporary full-density subscriber.
  -h, --help       Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --include-cli-hz)
      INCLUDE_CLI_HZ=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[lidar-jitter] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

tmp_dir="$(mktemp -d /tmp/njrh_lidar_jitter_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

float_ge() {
  awk -v value="${1:-nan}" -v minimum="${2:-0}" 'BEGIN { exit(value + 0.0 >= minimum + 0.0 ? 0 : 1) }'
}

float_lt() {
  awk -v value="${1:-nan}" -v minimum="${2:-0}" 'BEGIN { exit(value + 0.0 < minimum + 0.0 ? 0 : 1) }'
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

status_samples() {
  local topic="$1"
  local label="$2"
  local raw_file="${tmp_dir}/${label}.raw"
  local samples_file="${tmp_dir}/${label}.samples"
  timeout "${STATUS_SECONDS}" ros2 topic echo "${topic}" --field data >"${raw_file}" 2>&1 || true
  awk 'NF && $0 != "---" {print}' "${raw_file}" >"${samples_file}"
  if [[ ! -s "${samples_file}" ]]; then
    echo "[lidar-jitter] WARN ${topic}: no status samples" >&2
    sed 's/^/[lidar-jitter]   /' "${raw_file}" >&2 || true
  fi
  printf '%s\n' "${samples_file}"
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

param_value() {
  local node="$1"
  local name="$2"
  ros2 param get "${node}" "${name}" 2>/dev/null | sed -E 's/^[^:]+: //'
}

print_audit_row() {
  local item="$1"
  local expected="$2"
  local actual="$3"
  local status="$4"
  local risk="$5"
  printf '[lidar-jitter] AUDIT item=%s expected="%s" actual="%s" status=%s risk="%s"\n' \
    "${item}" "${expected}" "${actual}" "${status}" "${risk}"
}

echo "[lidar-jitter] This default run does not subscribe to full-density /lidar_points."
echo "[lidar-jitter] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[lidar-jitter] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"
echo "[lidar-jitter] NJRH_FASTDDS_PROFILE_ENABLED=${NJRH_FASTDDS_PROFILE_ENABLED:-unset}"
echo "[lidar-jitter] profile=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE} source=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE_SOURCE}"
echo "[lidar-jitter] resolved_local_input=${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}"
echo "[lidar-jitter] resolved_axis_local_output_topic=${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC:-<disabled>}"

axis_samples="$(status_samples /lidar/axis_remap_status axis)"
local_samples="$(status_samples /perception/local_perception_status local)"
nav_samples="$(status_samples /lidar/nav_cloud_preprocessor_status nav)"

lidar_info="$(topic_info_file /lidar_points lidar_points)"
local_branch_info="$(topic_info_file /_internal/lidar_points_local local_branch)"
nav_branch_info="$(topic_info_file /lidar_points_nav nav_branch)"
obstacle_info="$(topic_info_file /perception/obstacle_points obstacle)"

lidar_publishers="$(count_from_topic_info "${lidar_info}" "Publisher count")"
lidar_subscribers="$(count_from_topic_info "${lidar_info}" "Subscription count")"
local_branch_publishers="$(count_from_topic_info "${local_branch_info}" "Publisher count")"
local_branch_subscribers="$(count_from_topic_info "${local_branch_info}" "Subscription count")"
nav_branch_publishers="$(count_from_topic_info "${nav_branch_info}" "Publisher count")"
nav_branch_subscribers="$(count_from_topic_info "${nav_branch_info}" "Subscription count")"
obstacle_publishers="$(count_from_topic_info "${obstacle_info}" "Publisher count")"
obstacle_subscribers="$(count_from_topic_info "${obstacle_info}" "Subscription count")"

axis_raw_hz="$(field_avg "${axis_samples}" raw_input_hz 2>/dev/null || true)"
axis_publish_hz="$(field_avg "${axis_samples}" lidar_points_publish_hz 2>/dev/null || true)"
axis_local_hz="$(field_avg "${axis_samples}" local_branch_publish_hz 2>/dev/null || true)"
axis_nav_hz="$(field_avg "${axis_samples}" nav_branch_publish_hz 2>/dev/null || true)"
axis_local_enabled="$(field_latest "${axis_samples}" local_branch_enabled 2>/dev/null || true)"
axis_nav_enabled="$(field_latest "${axis_samples}" nav_branch_enabled 2>/dev/null || true)"
axis_raw_interarrival_avg="$(field_avg "${axis_samples}" raw_interarrival_ms_avg 2>/dev/null || true)"
axis_publish_interval_avg="$(field_avg "${axis_samples}" lidar_points_publish_interval_ms_avg 2>/dev/null || true)"
axis_gap100="$(field_latest "${axis_samples}" trunk_publish_gap_over_100ms_count 2>/dev/null || true)"
axis_gap150="$(field_latest "${axis_samples}" trunk_publish_gap_over_150ms_count 2>/dev/null || true)"
axis_gap200="$(field_latest "${axis_samples}" trunk_publish_gap_over_200ms_count 2>/dev/null || true)"
local_input_topic="$(field_latest "${local_samples}" input_topic 2>/dev/null || true)"
local_input_hz="$(field_avg "${local_samples}" input_callback_hz 2>/dev/null || true)"
local_processed_hz="$(field_avg "${local_samples}" processed_cloud_hz 2>/dev/null || true)"
local_obstacle_hz="$(field_avg "${local_samples}" published_obstacle_hz 2>/dev/null || true)"
nav_input_hz="$(field_avg "${nav_samples}" input_callback_hz 2>/dev/null || true)"

nav_output_stride="$(param_value /pointcloud_axis_remap nav_output_stride || true)"
nav_output_publish_every_n="$(param_value /pointcloud_axis_remap nav_output_publish_every_n || true)"
local_output_topic="$(param_value /pointcloud_axis_remap local_output_topic || true)"
local_output_stride="$(param_value /pointcloud_axis_remap local_output_stride || true)"
local_output_publish_every_n="$(param_value /pointcloud_axis_remap local_output_publish_every_n || true)"
robot_local_input_param="$(param_value /robot_local_perception input_topic || true)"

echo "[lidar-jitter] axis avg raw_input_hz=${axis_raw_hz:-missing} lidar_points_publish_hz=${axis_publish_hz:-missing}"
echo "[lidar-jitter] axis avg local_branch_publish_hz=${axis_local_hz:-missing} nav_branch_publish_hz=${axis_nav_hz:-missing}"
echo "[lidar-jitter] axis timing raw_interarrival_ms_avg=${axis_raw_interarrival_avg:-missing} lidar_points_publish_interval_ms_avg=${axis_publish_interval_avg:-missing}"
echo "[lidar-jitter] axis trunk gap counters: >100ms=${axis_gap100:-missing} >150ms=${axis_gap150:-missing} >200ms=${axis_gap200:-missing}"
echo "[lidar-jitter] local status input_topic=${local_input_topic:-missing} input_hz=${local_input_hz:-missing} processed_hz=${local_processed_hz:-missing} obstacle_hz=${local_obstacle_hz:-missing}"
echo "[lidar-jitter] nav preprocessor input_hz=${nav_input_hz:-missing}"
echo "[lidar-jitter] graph /lidar_points publishers=${lidar_publishers:-0} subscribers=${lidar_subscribers:-0}"
echo "[lidar-jitter] graph /_internal/lidar_points_local publishers=${local_branch_publishers:-0} subscribers=${local_branch_subscribers:-0}"
echo "[lidar-jitter] graph /lidar_points_nav publishers=${nav_branch_publishers:-0} subscribers=${nav_branch_subscribers:-0}"
echo "[lidar-jitter] graph /perception/obstacle_points publishers=${obstacle_publishers:-0} subscribers=${obstacle_subscribers:-0}"

print_audit_row "profile" "local_branch production default" \
  "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" \
  "$([[ "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" == "local_branch" ]] && echo PASS || echo WARN)" \
  "trunk profile makes local perception consume full-density /lidar_points"
print_audit_row "robot_local_perception_input" "/_internal/lidar_points_local" \
  "${robot_local_input_param:-missing}" \
  "$([[ "${robot_local_input_param:-}" == "/_internal/lidar_points_local" ]] && echo PASS || echo FAIL)" \
  "profile regression can increase trunk fan-out"
print_audit_row "lidar_points_publishers" "1" "${lidar_publishers:-0}" \
  "$([[ "${lidar_publishers:-0}" -eq 1 ]] && echo PASS || echo FAIL)" \
  "multiple trunk publishers break canonical ownership"
print_audit_row "lidar_points_subscribers" "0 during local_branch navigation unless explicit diagnostics" \
  "${lidar_subscribers:-0}" \
  "$([[ "${lidar_subscribers:-0}" -eq 0 ]] && echo PASS || echo WARN)" \
  "full-density subscribers can lower CLI delivery or add CPU/DDS pressure"
print_audit_row "local_branch_graph" "1 publisher / 1 subscriber" \
  "${local_branch_publishers:-0}/${local_branch_subscribers:-0}" \
  "$([[ "${local_branch_publishers:-0}" -eq 1 && "${local_branch_subscribers:-0}" -eq 1 ]] && echo PASS || echo FAIL)" \
  "local_branch not effective"
print_audit_row "axis_params" "nav stride=4 every=2 local topic=/_internal/lidar_points_local stride=2 every=1" \
  "nav_stride=${nav_output_stride:-missing} nav_every=${nav_output_publish_every_n:-missing} local_topic=${local_output_topic:-missing} local_stride=${local_output_stride:-missing} local_every=${local_output_publish_every_n:-missing}" \
  "$([[ "${nav_output_stride:-}" == "4" && "${nav_output_publish_every_n:-}" == "2" && "${local_output_topic:-}" == "/_internal/lidar_points_local" && "${local_output_stride:-}" == "2" && "${local_output_publish_every_n:-}" == "1" ]] && echo PASS || echo FAIL)" \
  "old binary or profile override"
print_audit_row "dds_transport" "UDPv4 default, no implicit DDS A/B" \
  "RMW=${RMW_IMPLEMENTATION:-unset} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}" \
  "$([[ "${RMW_IMPLEMENTATION:-}" == "rmw_fastrtps_cpp" && "${FASTDDS_BUILTIN_TRANSPORTS:-}" == "UDPv4" ]] && echo PASS || echo WARN)" \
  "DDS transport changes must be explicit experiments"

cli_rate=""
cli_rate_min=""
cli_rate_max=""
if [[ "${INCLUDE_CLI_HZ}" == "true" ]]; then
  echo "[lidar-jitter] WARNING: ros2 topic hz /lidar_points is a full-density subscriber."
  echo "[lidar-jitter] WARNING: prefer /lidar/axis_remap_status lidar_points_publish_hz for publish-side judgment."
  cli_file="${tmp_dir}/lidar_points_cli_hz.txt"
  timeout "${CLI_HZ_SECONDS}" ros2 topic hz /lidar_points >"${cli_file}" 2>&1 || true
  cli_values="$(
    awk '
      /average rate:/ {
        rate=$3
        sum += rate
        count += 1
        if (min == "" || rate < min) min = rate
        if (max == "" || rate > max) max = rate
      }
      END {
        if (count > 0) printf "%.3f %.3f %.3f", sum / count, min, max
      }
    ' "${cli_file}" || true
  )"
  read -r cli_rate cli_rate_min cli_rate_max <<<"${cli_values:-}" || true
  echo "[lidar-jitter] CLI /lidar_points average_of_averages=${cli_rate:-missing} min_average=${cli_rate_min:-missing} max_average=${cli_rate_max:-missing}"
fi

process_snapshot="$(pgrep -af 'pointcloud_axis_remap|pointcloud_perception_pipeline|component_container|pointcloud_downsample|fastlio|laser_mapping|rviz|foxglove|rosbag' || true)"
if [[ -n "${process_snapshot}" ]]; then
  echo "[lidar-jitter] process snapshot:"
  printf '%s\n' "${process_snapshot}" | sed 's/^/[lidar-jitter]   /'
fi

case_result="CASE_UNCLASSIFIED"
case_detail="insufficient data"

profile_regression=false
if [[ "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" == "local_branch" ]]; then
  if [[ "${robot_local_input_param:-}" == "/lidar_points" || "${local_input_topic:-}" == "/lidar_points" ]]; then
    profile_regression=true
  fi
  if grep -q 'Node name: robot_local_perception' "${lidar_info}"; then
    profile_regression=true
  fi
  if [[ "${local_branch_publishers:-0}" -eq 0 || "${local_branch_subscribers:-0}" -eq 0 ]]; then
    profile_regression=true
  fi
fi

old_binary=false
if [[ -z "${nav_output_publish_every_n}" || -z "${local_output_publish_every_n}" ]]; then
  old_binary=true
fi

if [[ "${profile_regression}" == "true" ]]; then
  case_result="CASE_C_PROFILE_REGRESSION"
  case_detail="local_branch is selected but local perception or graph does not use the local branch"
elif [[ "${old_binary}" == "true" ]]; then
  case_result="CASE_D_STALE_PROCESS_OR_OLD_BINARY"
  case_detail="running pointcloud_axis_remap lacks Phase 1.6/1.8 parameters"
elif [[ -n "${axis_raw_hz}" && -n "${axis_publish_hz}" ]] &&
  float_ge "${axis_raw_hz}" "${MIN_AXIS_HZ}" && float_lt "${axis_publish_hz}" "${MIN_AXIS_HZ}"
then
  case_result="CASE_A_AXIS_PUBLISH_LOW"
  case_detail="axis input is healthy but /lidar_points publish-side is below ${MIN_AXIS_HZ}Hz"
elif [[ -n "${axis_publish_hz}" ]] && float_lt "${axis_publish_hz}" "${MIN_AXIS_HZ}" &&
  [[ "${axis_local_enabled:-}" == "true" || "${axis_nav_enabled:-}" == "true" ]]
then
  case_result="CASE_G_AXIS_LOW_WITH_PURE_TRUNK_NEEDED"
  case_detail="axis publish is low while derived branches are enabled; run pure trunk A/B"
elif [[ "${lidar_subscribers:-0}" -gt 0 && "${INCLUDE_CLI_HZ}" != "true" ]]; then
  case_result="CASE_E_TOO_MANY_TRUNK_SUBSCRIBERS"
  case_detail="/lidar_points has full-density subscribers during navigation"
elif [[ "${INCLUDE_CLI_HZ}" == "true" && -n "${axis_publish_hz}" && -n "${cli_rate}" ]] &&
  float_ge "${axis_publish_hz}" "${MIN_AXIS_HZ}" && float_lt "${cli_rate}" "${LOW_CLI_HZ}" &&
  [[ "${lidar_publishers:-0}" -eq 1 ]]
then
  if [[ -n "${axis_local_hz}" && -n "${local_input_hz}" && -n "${local_processed_hz}" && -n "${local_obstacle_hz}" ]] &&
    float_ge "${axis_local_hz}" "${MIN_LOCAL_HZ}" &&
    float_ge "${local_input_hz}" "${MIN_LOCAL_HZ}" &&
    float_ge "${local_processed_hz}" "${MIN_LOCAL_HZ}" &&
    float_ge "${local_obstacle_hz}" "${MIN_LOCAL_HZ}"
  then
    case_result="CASE_F_LOCAL_BRANCH_OK_BUT_TRUNK_CLI_LOW"
    case_detail="navigation local branch is healthy; full-density CLI delivery is low"
  else
    case_result="CASE_B_CLI_DELIVERY_LOW_ONLY"
    case_detail="axis publish is healthy but /lidar_points CLI receives below ${LOW_CLI_HZ}Hz"
  fi
elif [[ -n "${axis_publish_hz}" ]] && float_ge "${axis_publish_hz}" "${MIN_AXIS_HZ}"; then
  case_result="CASE_AXIS_PUBLISH_HEALTHY"
  case_detail="publish-side /lidar_points is healthy; inspect local branch or optional CLI delivery if needed"
fi

echo "[lidar-jitter] diagnosis=${case_result}"
echo "[lidar-jitter] detail=${case_detail}"

if [[ "${case_result}" == "CASE_AXIS_PUBLISH_HEALTHY" || "${case_result}" == "CASE_F_LOCAL_BRANCH_OK_BUT_TRUNK_CLI_LOW" || "${case_result}" == "CASE_B_CLI_DELIVERY_LOW_ONLY" ]]; then
  echo "[lidar-jitter] DDS transport A/B may be useful for full-density debug/mapping subscribers."
  echo "[lidar-jitter] Suggested helper: bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_dds_transport_ab.sh"
fi

if [[ "${case_result}" == CASE_A_* || "${case_result}" == CASE_C_* || "${case_result}" == CASE_D_* || "${case_result}" == CASE_G_* ]]; then
  exit 1
fi
exit 0

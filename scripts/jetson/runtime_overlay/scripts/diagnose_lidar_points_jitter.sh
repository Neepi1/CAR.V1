#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

INCLUDE_CLI_HZ=false
STATUS_SECONDS="${NJRH_DIAG_LIDAR_STATUS_SECONDS:-12}"
CLI_HZ_SECONDS="${NJRH_DIAG_LIDAR_CLI_HZ_SECONDS:-10}"
MIN_AXIS_HZ="${NJRH_DIAG_LIDAR_MIN_AXIS_HZ:-18.0}"
LOW_CLI_HZ="${NJRH_DIAG_LIDAR_LOW_CLI_HZ:-18.0}"

usage() {
  cat <<'EOF'
Usage: diagnose_lidar_points_jitter.sh [--include-cli-hz]

Default mode does not subscribe to full-density /lidar_points. It reads status
topics and ROS graph state to distinguish JT128 publish-side jitter from
subscriber delivery jitter. The retired robot_local_perception PointCloud2
obstacle path is not sampled.

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
echo "[lidar-jitter] Retired robot_local_perception PointCloud2 obstacle topics are not sampled."
echo "[lidar-jitter] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[lidar-jitter] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"
echo "[lidar-jitter] NJRH_FASTDDS_PROFILE_ENABLED=${NJRH_FASTDDS_PROFILE_ENABLED:-unset}"

axis_samples="$(status_samples /lidar/axis_remap_status axis)"
accel_samples="$(status_samples /lidar/pointcloud_accel_status accel)"
nav_samples="$(status_samples /lidar/nav_cloud_preprocessor_status nav)"

lidar_info="$(topic_info_file /lidar_points lidar_points)"
nav_branch_info="$(topic_info_file /lidar_points_nav nav_branch)"
scan_info="$(topic_info_file /scan scan)"
flatscan_info="$(topic_info_file /flatscan flatscan)"

lidar_publishers="$(count_from_topic_info "${lidar_info}" "Publisher count")"
lidar_subscribers="$(count_from_topic_info "${lidar_info}" "Subscription count")"
nav_branch_publishers="$(count_from_topic_info "${nav_branch_info}" "Publisher count")"
nav_branch_subscribers="$(count_from_topic_info "${nav_branch_info}" "Subscription count")"
scan_publishers="$(count_from_topic_info "${scan_info}" "Publisher count")"
scan_subscribers="$(count_from_topic_info "${scan_info}" "Subscription count")"
flatscan_publishers="$(count_from_topic_info "${flatscan_info}" "Publisher count")"
flatscan_subscribers="$(count_from_topic_info "${flatscan_info}" "Subscription count")"

axis_raw_hz="$(field_avg "${axis_samples}" raw_input_hz 2>/dev/null || true)"
axis_publish_hz="$(field_avg "${axis_samples}" lidar_points_publish_hz 2>/dev/null || true)"
axis_nav_hz="$(field_avg "${axis_samples}" nav_branch_publish_hz 2>/dev/null || true)"
axis_raw_interarrival_avg="$(field_avg "${axis_samples}" raw_interarrival_ms_avg 2>/dev/null || true)"
axis_publish_interval_avg="$(field_avg "${axis_samples}" lidar_points_publish_interval_ms_avg 2>/dev/null || true)"
axis_gap100="$(field_latest "${axis_samples}" trunk_publish_gap_over_100ms_count 2>/dev/null || true)"
axis_gap150="$(field_latest "${axis_samples}" trunk_publish_gap_over_150ms_count 2>/dev/null || true)"
axis_gap200="$(field_latest "${axis_samples}" trunk_publish_gap_over_200ms_count 2>/dev/null || true)"
accel_scan_hz="$(field_avg "${accel_samples}" scan_publish_hz 2>/dev/null || true)"
accel_scan_header_age="$(field_avg "${accel_samples}" scan_output_header_age_ms 2>/dev/null || true)"
accel_scan_source_age="$(field_avg "${accel_samples}" scan_output_source_age_ms 2>/dev/null || true)"
nav_input_hz="$(field_avg "${nav_samples}" input_callback_hz 2>/dev/null || true)"

nav_output_stride="$(param_value /pointcloud_axis_remap nav_output_stride || true)"
nav_output_publish_every_n="$(param_value /pointcloud_axis_remap nav_output_publish_every_n || true)"
local_output_topic="$(param_value /pointcloud_axis_remap local_output_topic || true)"

echo "[lidar-jitter] axis avg raw_input_hz=${axis_raw_hz:-missing} lidar_points_publish_hz=${axis_publish_hz:-missing}"
echo "[lidar-jitter] axis avg nav_branch_publish_hz=${axis_nav_hz:-missing}"
echo "[lidar-jitter] axis timing raw_interarrival_ms_avg=${axis_raw_interarrival_avg:-missing} lidar_points_publish_interval_ms_avg=${axis_publish_interval_avg:-missing}"
echo "[lidar-jitter] axis trunk gap counters: >100ms=${axis_gap100:-missing} >150ms=${axis_gap150:-missing} >200ms=${axis_gap200:-missing}"
echo "[lidar-jitter] accel scan_publish_hz=${accel_scan_hz:-missing} scan_output_header_age_ms=${accel_scan_header_age:-missing} scan_output_source_age_ms=${accel_scan_source_age:-missing}"
echo "[lidar-jitter] nav preprocessor input_hz=${nav_input_hz:-missing}"
echo "[lidar-jitter] graph /lidar_points publishers=${lidar_publishers:-0} subscribers=${lidar_subscribers:-0}"
echo "[lidar-jitter] graph /lidar_points_nav publishers=${nav_branch_publishers:-0} subscribers=${nav_branch_subscribers:-0}"
echo "[lidar-jitter] graph /scan publishers=${scan_publishers:-0} subscribers=${scan_subscribers:-0}"
echo "[lidar-jitter] graph /flatscan publishers=${flatscan_publishers:-0} subscribers=${flatscan_subscribers:-0}"

print_audit_row "local_pointcloud_branch" "disabled" \
  "local_output_topic=${local_output_topic:-missing}" \
  "$([[ -z "${local_output_topic:-}" || "${local_output_topic:-}" == '""' || "${local_output_topic:-}" == "''" ]] && echo PASS || echo FAIL)" \
  "retired PointCloud2 local obstacle branch must not publish"
print_audit_row "lidar_points_publishers" "1" "${lidar_publishers:-0}" \
  "$([[ "${lidar_publishers:-0}" -eq 1 ]] && echo PASS || echo FAIL)" \
  "multiple trunk publishers break canonical ownership"
print_audit_row "lidar_points_subscribers" "<=2 during navigation unless explicit diagnostics" \
  "${lidar_subscribers:-0}" \
  "$([[ "${lidar_subscribers:-0}" -le 2 ]] && echo PASS || echo WARN)" \
  "full-density subscribers can add CPU/DDS pressure"
print_audit_row "scan_graph" ">=1 publisher and local_costmap/collision_monitor subscribers" \
  "${scan_publishers:-0}/${scan_subscribers:-0}" \
  "$([[ "${scan_publishers:-0}" -ge 1 && "${scan_subscribers:-0}" -ge 2 ]] && echo PASS || echo WARN)" \
  "Nav2 standard obstacle marking/clearing depends on /scan"
print_audit_row "axis_params" "nav stride=4 every=2 local topic disabled" \
  "nav_stride=${nav_output_stride:-missing} nav_every=${nav_output_publish_every_n:-missing} local_topic=${local_output_topic:-missing}" \
  "$([[ "${nav_output_stride:-}" == "4" && "${nav_output_publish_every_n:-}" == "2" && ( -z "${local_output_topic:-}" || "${local_output_topic:-}" == '""' || "${local_output_topic:-}" == "''" ) ]] && echo PASS || echo FAIL)" \
  "old binary or profile override"
print_audit_row "dds_transport" "UDPv4 default, no implicit DDS A/B" \
  "RMW=${RMW_IMPLEMENTATION:-unset} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}" \
  "$([[ "${RMW_IMPLEMENTATION:-}" == "rmw_fastrtps_cpp" && "${FASTDDS_BUILTIN_TRANSPORTS:-}" == "UDPv4" ]] && echo PASS || echo WARN)" \
  "DDS transport changes must be explicit experiments"

cli_rate=""
if [[ "${INCLUDE_CLI_HZ}" == "true" ]]; then
  echo "[lidar-jitter] WARNING: ros2 topic hz /lidar_points is a full-density subscriber."
  echo "[lidar-jitter] WARNING: prefer /lidar/axis_remap_status lidar_points_publish_hz for publish-side judgment."
  cli_file="${tmp_dir}/lidar_points_cli_hz.txt"
  timeout "${CLI_HZ_SECONDS}" ros2 topic hz /lidar_points >"${cli_file}" 2>&1 || true
  cli_rate="$(
    awk '
      /average rate:/ {
        rate=$3
        sum += rate
        count += 1
      }
      END {
        if (count > 0) printf "%.3f", sum / count
      }
    ' "${cli_file}" || true
  )"
  echo "[lidar-jitter] CLI /lidar_points average_of_averages=${cli_rate:-missing}"
fi

process_snapshot="$(pgrep -af 'pointcloud_axis_remap|pointcloud_perception_pipeline|component_container|pointcloud_downsample|fastlio|laser_mapping|rviz|foxglove|rosbag' || true)"
if [[ -n "${process_snapshot}" ]]; then
  echo "[lidar-jitter] process snapshot:"
  printf '%s\n' "${process_snapshot}" | sed 's/^/[lidar-jitter]   /'
fi

case_result="CASE_UNCLASSIFIED"
case_detail="insufficient data"

old_binary=false
if [[ -z "${nav_output_publish_every_n}" ]]; then
  old_binary=true
fi

if [[ "${old_binary}" == "true" ]]; then
  case_result="CASE_D_STALE_PROCESS_OR_OLD_BINARY"
  case_detail="running pointcloud_axis_remap lacks expected parameters"
elif [[ -n "${local_output_topic:-}" && "${local_output_topic:-}" != '""' && "${local_output_topic:-}" != "''" ]]; then
  case_result="CASE_C_RETIRED_LOCAL_POINTCLOUD_BRANCH_ENABLED"
  case_detail="local PointCloud2 obstacle branch should be disabled; Nav2 uses /scan"
elif [[ -n "${axis_raw_hz}" && -n "${axis_publish_hz}" ]] &&
  float_ge "${axis_raw_hz}" "${MIN_AXIS_HZ}" && float_lt "${axis_publish_hz}" "${MIN_AXIS_HZ}"
then
  case_result="CASE_A_AXIS_PUBLISH_LOW"
  case_detail="axis input is healthy but /lidar_points publish-side is below ${MIN_AXIS_HZ}Hz"
elif [[ "${lidar_subscribers:-0}" -gt 2 && "${INCLUDE_CLI_HZ}" != "true" ]]; then
  case_result="CASE_E_TOO_MANY_TRUNK_SUBSCRIBERS"
  case_detail="/lidar_points has unexpected full-density subscribers during navigation"
elif [[ "${INCLUDE_CLI_HZ}" == "true" && -n "${axis_publish_hz}" && -n "${cli_rate}" ]] &&
  float_ge "${axis_publish_hz}" "${MIN_AXIS_HZ}" && float_lt "${cli_rate}" "${LOW_CLI_HZ}" &&
  [[ "${lidar_publishers:-0}" -eq 1 ]]
then
  case_result="CASE_B_CLI_DELIVERY_LOW_ONLY"
  case_detail="axis publish-side is healthy but CLI subscriber observes low delivery"
elif [[ -n "${axis_publish_hz}" && -n "${accel_scan_hz}" ]] &&
  float_ge "${axis_publish_hz}" "${MIN_AXIS_HZ}" && float_ge "${accel_scan_hz}" 8.0
then
  case_result="CASE_OK_SCAN_OBSTACLE_PATH"
  case_detail="/lidar_points trunk and /scan obstacle path are healthy"
fi

echo "[lidar-jitter] RESULT ${case_result}: ${case_detail}"
echo "[lidar-jitter] CASE_A_AXIS_PUBLISH_LOW: JT128 ingress is healthy but /lidar_points publish-side is low."
echo "[lidar-jitter] CASE_B_CLI_DELIVERY_LOW_ONLY: axis status is healthy but a temporary CLI subscriber sees low /lidar_points rate."
echo "[lidar-jitter] CASE_C_RETIRED_LOCAL_POINTCLOUD_BRANCH_ENABLED: local PointCloud2 obstacle branch is unexpectedly enabled."
echo "[lidar-jitter] CASE_D_STALE_PROCESS_OR_OLD_BINARY: running binary lacks expected pointcloud parameters."
echo "[lidar-jitter] CASE_E_TOO_MANY_TRUNK_SUBSCRIBERS: unexpected full-density /lidar_points subscribers are attached."
echo "[lidar-jitter] CASE_OK_SCAN_OBSTACLE_PATH: trunk publish and standard /scan obstacle path are healthy."

if [[ "${case_result}" == "CASE_B_CLI_DELIVERY_LOW_ONLY" ]]; then
  echo "[lidar-jitter] Set NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ=${cli_rate:-missing} and run verify_pointcloud_delivery_matrix.sh to classify DDS transport suspicion."
fi

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"
njrh_load_local_perception_input_profile

DURATION_SEC=1200
SAMPLE_PERIOD_SEC="${NJRH_NAV_ACCEPTANCE_SAMPLE_PERIOD_SEC:-5}"
OUTPUT_DIR="${NJRH_NAV_ACCEPTANCE_REPORT_DIR:-${NJRH_PROJECT_ROOT}/reports}"
INCLUDE_TF="${NJRH_NAV_ACCEPTANCE_INCLUDE_TF:-false}"

usage() {
  cat <<'EOF'
Usage: record_pointcloud_nav_acceptance.sh [--duration-sec SECONDS] [--output-dir DIR]

Records lightweight navigation pointcloud diagnostics and writes a markdown
summary. It does not record full-density PointCloud2 topics unless
NJRH_RECORD_HEAVY_POINTCLOUDS=true is explicitly set.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      [[ "$#" -ge 2 ]] || { echo "[nav-acceptance] --duration-sec requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --output-dir)
      [[ "$#" -ge 2 ]] || { echo "[nav-acceptance] --output-dir requires a value" >&2; exit 2; }
      OUTPUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[nav-acceptance] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || { echo "[nav-acceptance] invalid duration: ${DURATION_SEC}" >&2; exit 2; }
[[ "${SAMPLE_PERIOD_SEC}" =~ ^[0-9]+$ ]] || { echo "[nav-acceptance] invalid sample period: ${SAMPLE_PERIOD_SEC}" >&2; exit 2; }

mkdir -p "${OUTPUT_DIR}"
timestamp="$(date +%Y%m%d_%H%M%S)"
report_path="${OUTPUT_DIR}/pointcloud_nav_acceptance_${timestamp}.md"
tmp_dir="$(mktemp -d /tmp/njrh_nav_acceptance_XXXXXX)"

field_value() {
  local text="$1"
  local key="$2"
  printf '%s\n' "${text}" | tr ' ' '\n' | awk -F= -v key="${key}" '$1 == key {print substr($0, length(key) + 2); exit}'
}

status_once() {
  local topic="$1"
  local raw_file="$2"
  timeout 4 ros2 topic echo "${topic}" --field data >"${raw_file}.raw" 2>&1 || true
  awk 'NF && $0 != "---" {line=$0} END {print line}' "${raw_file}.raw"
}

append_metric() {
  local file="$1"
  local value="$2"
  if [[ "${value}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    printf '%s\n' "${value}" >>"${file}"
  fi
}

metric_stats() {
  local file="$1"
  awk '
    NF {
      value = $1 + 0.0
      if (count == 0 || value < min) min = value
      sum += value
      count += 1
    }
    END {
      if (count > 0) {
        printf "min=%.3f avg=%.3f samples=%d", min, sum / count, count
      } else {
        printf "missing"
      }
    }
  ' "${file}" 2>/dev/null || printf 'missing'
}

metric_max() {
  local file="$1"
  awk 'NF {if (count == 0 || $1 + 0.0 > max) max = $1 + 0.0; count += 1} END {if (count > 0) printf "%.3f", max; else printf "missing"}' "${file}" 2>/dev/null || printf 'missing'
}

topic_info_snapshot() {
  local topic="$1"
  local file="$2"
  {
    echo "### ${topic}"
    timeout 8 ros2 topic info -v "${topic}" 2>&1 || true
    echo
  } >>"${file}"
}

echo "[nav-acceptance] Writing ${report_path}"
echo "[nav-acceptance] duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "[nav-acceptance] heavy pointcloud recording=${NJRH_RECORD_HEAVY_POINTCLOUDS:-false}"

topic_info_file="${tmp_dir}/topic_info.md"
topic_info_snapshot /lidar_points "${topic_info_file}"
topic_info_snapshot /_internal/lidar_points_local "${topic_info_file}"
topic_info_snapshot /perception/obstacle_points "${topic_info_file}"

start_epoch="$(date +%s)"
end_epoch=$((start_epoch + DURATION_SEC))
sample_index=0
case_i_seen="false"
fastlio_seen="false"
heavy_bag_pid=""

cleanup() {
  if [[ -n "${heavy_bag_pid}" ]]; then
    kill -INT "${heavy_bag_pid}" 2>/dev/null || true
    wait "${heavy_bag_pid}" 2>/dev/null || true
  fi
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

if [[ "${NJRH_RECORD_HEAVY_POINTCLOUDS:-false}" == "true" ]]; then
  heavy_bag_dir="${OUTPUT_DIR}/pointcloud_nav_acceptance_${timestamp}_heavy_bag"
  echo "[nav-acceptance] heavy pointcloud bag output=${heavy_bag_dir}"
  ros2 bag record \
    /lidar_points \
    /_internal/lidar_points_local \
    /points_nav \
    /perception/obstacle_points \
    -o "${heavy_bag_dir}" >/dev/null 2>&1 &
  heavy_bag_pid=$!
fi

while (( $(date +%s) < end_epoch )); do
  axis_status="$(status_once /lidar/axis_remap_status "${tmp_dir}/axis_${sample_index}")"
  local_status="$(status_once /perception/local_perception_status "${tmp_dir}/local_${sample_index}")"
  nav_status="$(status_once /lidar/nav_cloud_preprocessor_status "${tmp_dir}/nav_${sample_index}")"
  safety_status="$(status_once /safety/status "${tmp_dir}/safety_${sample_index}")"

  printf '%s\n' "${axis_status}" >>"${tmp_dir}/axis_status.log"
  printf '%s\n' "${local_status}" >>"${tmp_dir}/local_status.log"
  printf '%s\n' "${nav_status}" >>"${tmp_dir}/nav_status.log"
  printf '%s\n' "${safety_status}" >>"${tmp_dir}/safety_status.log"

  append_metric "${tmp_dir}/axis_lidar_hz.metric" "$(field_value "${axis_status}" lidar_points_publish_hz || true)"
  append_metric "${tmp_dir}/local_branch_hz.metric" "$(field_value "${axis_status}" local_branch_publish_hz || true)"
  append_metric "${tmp_dir}/local_input_hz.metric" "$(field_value "${local_status}" input_callback_hz || true)"
  append_metric "${tmp_dir}/local_processed_hz.metric" "$(field_value "${local_status}" processed_cloud_hz || true)"
  append_metric "${tmp_dir}/local_obstacle_hz.metric" "$(field_value "${local_status}" published_obstacle_hz || true)"
  append_metric "${tmp_dir}/local_interarrival.metric" "$(field_value "${local_status}" input_interarrival_ms_max || true)"
  append_metric "${tmp_dir}/local_no_new.metric" "$(field_value "${local_status}" no_new_count || true)"

  axis_local_branch_enabled="$(field_value "${axis_status}" local_branch_enabled || true)"
  axis_lidar_hz="$(field_value "${axis_status}" lidar_points_publish_hz || true)"
  if [[ "${axis_local_branch_enabled}" == "true" && -n "${axis_lidar_hz}" ]] &&
    awk -v value="${axis_lidar_hz}" 'BEGIN {exit(value + 0.0 < 18.0 ? 0 : 1)}'
  then
    case_i_seen="true"
  fi

  if pgrep -af 'fastlio|fast_lio|fast-lio' >/dev/null 2>&1; then
    fastlio_seen="true"
  fi

  timeout 3 ros2 topic echo /local_state/odometry --field header >"${tmp_dir}/local_state_odometry_${sample_index}.txt" 2>&1 || true
  timeout 3 ros2 topic echo /localization_result >"${tmp_dir}/localization_result_${sample_index}.txt" 2>&1 || true
  if [[ "${INCLUDE_TF}" == "true" ]]; then
    timeout 3 ros2 topic echo /tf >"${tmp_dir}/tf_${sample_index}.txt" 2>&1 || true
    timeout 3 ros2 topic echo /tf_static >"${tmp_dir}/tf_static_${sample_index}.txt" 2>&1 || true
  fi

  sample_index=$((sample_index + 1))
  sleep "${SAMPLE_PERIOD_SEC}"
done

topic_info_snapshot /lidar_points "${topic_info_file}"
topic_info_snapshot /_internal/lidar_points_local "${topic_info_file}"
topic_info_snapshot /perception/obstacle_points "${topic_info_file}"

nav2_active="false"
if timeout 8 ros2 lifecycle get /controller_server 2>/dev/null | grep -q 'active' &&
  timeout 8 ros2 lifecycle get /planner_server 2>/dev/null | grep -q 'active' &&
  timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q 'active'
then
  nav2_active="true"
fi

obstacle_info="$(timeout 8 ros2 topic info -v /perception/obstacle_points 2>&1 || true)"
obstacle_has_local_costmap="false"
obstacle_has_collision_monitor="false"
printf '%s\n' "${obstacle_info}" | grep -q "Node name: local_costmap" && obstacle_has_local_costmap="true"
printf '%s\n' "${obstacle_info}" | grep -q "Node name: collision_monitor" && obstacle_has_collision_monitor="true"

first_no_new="$(awk 'NF {print; exit}' "${tmp_dir}/local_no_new.metric" 2>/dev/null || true)"
last_no_new="$(awk 'NF {value=$1} END {print value}' "${tmp_dir}/local_no_new.metric" 2>/dev/null || true)"
no_new_delta="missing"
if [[ "${first_no_new}" =~ ^[0-9]+([.][0-9]+)?$ && "${last_no_new}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  no_new_delta="$(awk -v first="${first_no_new}" -v last="${last_no_new}" 'BEGIN {printf "%.0f", last - first}')"
fi

{
  echo "# Pointcloud Navigation Acceptance ${timestamp}"
  echo
  echo "- duration_sec: ${DURATION_SEC}"
  echo "- sample_period_sec: ${SAMPLE_PERIOD_SEC}"
  echo "- profile: ${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}"
  echo "- resolved_local_input: ${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}"
  echo "- resolved_axis_local_output_topic: ${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC:-<disabled>}"
  echo "- heavy_pointcloud_recording: ${NJRH_RECORD_HEAVY_POINTCLOUDS:-false}"
  echo
  echo "## Metrics"
  echo
  echo "- axis lidar_points_publish_hz: $(metric_stats "${tmp_dir}/axis_lidar_hz.metric")"
  echo "- axis local_branch_publish_hz: $(metric_stats "${tmp_dir}/local_branch_hz.metric")"
  echo "- local input_callback_hz: $(metric_stats "${tmp_dir}/local_input_hz.metric")"
  echo "- local processed_cloud_hz: $(metric_stats "${tmp_dir}/local_processed_hz.metric")"
  echo "- local published_obstacle_hz: $(metric_stats "${tmp_dir}/local_obstacle_hz.metric")"
  echo "- local no_new_count_delta: ${no_new_delta}"
  echo "- local max input_interarrival_ms: $(metric_max "${tmp_dir}/local_interarrival.metric")"
  echo "- CASE_I_LOCAL_BRANCH_DRAGS_TRUNK seen: ${case_i_seen}"
  echo "- FAST-LIO2 navigation residue seen: ${fastlio_seen}"
  echo "- Nav2 active: ${nav2_active}"
  echo "- /perception/obstacle_points has local_costmap subscriber: ${obstacle_has_local_costmap}"
  echo "- /perception/obstacle_points has collision_monitor subscriber: ${obstacle_has_collision_monitor}"
  echo
  echo "## Heavy PointCloud2 Recording"
  if [[ "${NJRH_RECORD_HEAVY_POINTCLOUDS:-false}" == "true" ]]; then
    echo "Heavy pointcloud recording was explicitly enabled by environment. Review operator storage and DDS load before using this during navigation."
  else
    echo "Heavy pointcloud topics were not recorded. Set NJRH_RECORD_HEAVY_POINTCLOUDS=true only for explicit short diagnostic captures."
  fi
  echo
  echo "## Topic Info Snapshots"
  cat "${topic_info_file}"
  echo
  echo "## Latest Axis Status"
  tail -n 5 "${tmp_dir}/axis_status.log" 2>/dev/null || true
  echo
  echo "## Latest Local Perception Status"
  tail -n 5 "${tmp_dir}/local_status.log" 2>/dev/null || true
  echo
  echo "## Latest Nav Cloud Preprocessor Status"
  tail -n 5 "${tmp_dir}/nav_status.log" 2>/dev/null || true
  echo
  echo "## Latest Safety Status"
  tail -n 5 "${tmp_dir}/safety_status.log" 2>/dev/null || true
} >"${report_path}"

echo "[nav-acceptance] wrote ${report_path}"

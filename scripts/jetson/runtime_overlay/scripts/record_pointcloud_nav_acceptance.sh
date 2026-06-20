#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=1200
SAMPLE_PERIOD_SEC="${NJRH_NAV_ACCEPTANCE_SAMPLE_PERIOD_SEC:-5}"
OUTPUT_DIR="${NJRH_NAV_ACCEPTANCE_REPORT_DIR:-${NJRH_PROJECT_ROOT}/reports}"
INCLUDE_TF="${NJRH_NAV_ACCEPTANCE_INCLUDE_TF:-false}"

usage() {
  cat <<'EOF'
Usage: record_pointcloud_nav_acceptance.sh [--duration-sec SECONDS] [--output-dir DIR]

Records lightweight navigation sensor diagnostics for the standard Nav2
LaserScan obstacle path. It does not record or subscribe to retired local
PointCloud2 obstacle topics.
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
report_path="${OUTPUT_DIR}/scan_nav_acceptance_${timestamp}.md"
tmp_dir="$(mktemp -d /tmp/njrh_scan_nav_acceptance_XXXXXX)"

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
      if (count == 0 || value > max) max = value
      sum += value
      count += 1
    }
    END {
      if (count > 0) {
        printf "min=%.3f avg=%.3f max=%.3f samples=%d", min, sum / count, max, count
      } else {
        printf "missing"
      }
    }
  ' "${file}" 2>/dev/null || printf 'missing'
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

hz_once() {
  local topic="$1"
  local label="$2"
  local file="${tmp_dir}/${label}_hz.txt"
  timeout 8 ros2 topic hz "${topic}" >"${file}" 2>&1 || true
  awk '/average rate:/ {rate=$3} END {print rate}' "${file}"
}

echo "[nav-acceptance] Writing ${report_path}"
echo "[nav-acceptance] duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "[nav-acceptance] standard obstacle path=/scan"
echo "[nav-acceptance] no PointCloud2 topics are recorded"

topic_info_file="${tmp_dir}/topic_info.md"
for topic in /lidar_points /scan /flatscan /local_costmap/costmap; do
  topic_info_snapshot "${topic}" "${topic_info_file}"
done

start_epoch="$(date +%s)"
end_epoch=$((start_epoch + DURATION_SEC))
sample_index=0
fastlio_seen="false"

cleanup() {
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

while (( $(date +%s) < end_epoch )); do
  axis_status="$(status_once /lidar/axis_remap_status "${tmp_dir}/axis_${sample_index}")"
  accel_status="$(status_once /lidar/pointcloud_accel_status "${tmp_dir}/accel_${sample_index}")"
  safety_status="$(status_once /safety/status "${tmp_dir}/safety_${sample_index}")"

  printf '%s\n' "${axis_status}" >>"${tmp_dir}/axis_status.log"
  printf '%s\n' "${accel_status}" >>"${tmp_dir}/accel_status.log"
  printf '%s\n' "${safety_status}" >>"${tmp_dir}/safety_status.log"

  append_metric "${tmp_dir}/axis_lidar_hz.metric" "$(field_value "${axis_status}" lidar_points_publish_hz || true)"
  append_metric "${tmp_dir}/accel_scan_hz.metric" "$(field_value "${accel_status}" scan_publish_hz || true)"
  append_metric "${tmp_dir}/scan_source_age.metric" "$(field_value "${accel_status}" scan_output_source_age_ms || true)"
  append_metric "${tmp_dir}/scan_header_age.metric" "$(field_value "${accel_status}" scan_output_header_age_ms || true)"

  if pgrep -af 'fastlio|fast_lio|fast-lio' >/dev/null 2>&1; then
    fastlio_seen="true"
  fi

  timeout 3 ros2 topic echo /local_state/odometry --field header >"${tmp_dir}/local_state_odometry_${sample_index}.txt" 2>&1 || true
  timeout 3 ros2 topic echo /localization/bridge_status --field data >"${tmp_dir}/bridge_status_${sample_index}.txt" 2>&1 || true
  timeout 3 ros2 topic echo /local_costmap/costmap --field header >"${tmp_dir}/local_costmap_${sample_index}.txt" 2>&1 || true
  if [[ "${INCLUDE_TF}" == "true" ]]; then
    timeout 3 ros2 topic echo /tf >"${tmp_dir}/tf_${sample_index}.txt" 2>&1 || true
    timeout 3 ros2 topic echo /tf_static >"${tmp_dir}/tf_static_${sample_index}.txt" 2>&1 || true
  fi

  sample_index=$((sample_index + 1))
  sleep "${SAMPLE_PERIOD_SEC}"
done

for topic in /lidar_points /scan /flatscan /local_costmap/costmap; do
  topic_info_snapshot "${topic}" "${topic_info_file}"
done

scan_hz="$(hz_once /scan scan)"
flatscan_hz="$(hz_once /flatscan flatscan)"

nav2_active="false"
if timeout 8 ros2 lifecycle get /controller_server 2>/dev/null | grep -q 'active' &&
  timeout 8 ros2 lifecycle get /planner_server 2>/dev/null | grep -q 'active' &&
  timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q 'active'
then
  nav2_active="true"
fi

scan_info="$(timeout 8 ros2 topic info -v /scan 2>&1 || true)"
scan_has_local_costmap="false"
scan_has_collision_monitor="false"
printf '%s\n' "${scan_info}" | grep -q "Node name: local_costmap" && scan_has_local_costmap="true"
printf '%s\n' "${scan_info}" | grep -q "Node name: collision_monitor" && scan_has_collision_monitor="true"

{
  echo "# Scan Navigation Acceptance ${timestamp}"
  echo
  echo "- duration_sec: ${DURATION_SEC}"
  echo "- sample_period_sec: ${SAMPLE_PERIOD_SEC}"
  echo "- standard_obstacle_topic: /scan"
  echo "- pointcloud2_recording: false"
  echo
  echo "## Metrics"
  echo
  echo "- axis lidar_points_publish_hz: $(metric_stats "${tmp_dir}/axis_lidar_hz.metric")"
  echo "- accel scan_publish_hz: $(metric_stats "${tmp_dir}/accel_scan_hz.metric")"
  echo "- accel scan_output_source_age_ms: $(metric_stats "${tmp_dir}/scan_source_age.metric")"
  echo "- accel scan_output_header_age_ms: $(metric_stats "${tmp_dir}/scan_header_age.metric")"
  echo "- ros2 topic hz /scan: ${scan_hz:-missing}"
  echo "- ros2 topic hz /flatscan: ${flatscan_hz:-missing}"
  echo "- FAST-LIO2 navigation residue seen: ${fastlio_seen}"
  echo "- Nav2 active: ${nav2_active}"
  echo "- /scan has local_costmap subscriber: ${scan_has_local_costmap}"
  echo "- /scan has collision_monitor subscriber: ${scan_has_collision_monitor}"
  echo
  echo "## Topic Info Snapshots"
  cat "${topic_info_file}"
  echo
  echo "## Latest Axis Status"
  tail -n 5 "${tmp_dir}/axis_status.log" 2>/dev/null || true
  echo
  echo "## Latest Pointcloud Accel Status"
  tail -n 5 "${tmp_dir}/accel_status.log" 2>/dev/null || true
  echo
  echo "## Latest Safety Status"
  tail -n 5 "${tmp_dir}/safety_status.log" 2>/dev/null || true
} >"${report_path}"

echo "[nav-acceptance] wrote ${report_path}"

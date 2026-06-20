#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

TOPICS=(
  /lidar_points
  /lidar_points_nav
  /points_nav
  /scan
  /flatscan
)

status=0

count_from_info() {
  local file="$1"
  local label="$2"
  awk -F: -v label="${label}" '$1 == label {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}' "${file}"
}

node_names_from_info() {
  local file="$1"
  sed -n 's/^[[:space:]]*Node name:[[:space:]]*//p' "${file}" | sort -u | paste -sd ',' -
}

qos_summary_from_info() {
  local file="$1"
  awk '
    /^[[:space:]]*Reliability:/ {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2)
      reliabilities[$2] = 1
    }
    /^[[:space:]]*Depth:/ {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2)
      depths[$2] = 1
    }
    END {
      reliability = ""
      for (item in reliabilities) {
        reliability = reliability (reliability ? "," : "") item
      }
      depth = ""
      for (item in depths) {
        depth = depth (depth ? "," : "") item
      }
      printf "reliability=%s depth=%s", (reliability ? reliability : "unknown"), (depth ? depth : "unknown")
    }
  ' "${file}"
}

topic_info() {
  local topic="$1"
  local file="$2"
  ros2 topic info -v "${topic}" >"${file}" 2>&1 || true
}

echo "[pointcloud-subscribers] Observing ROS graph only; this script does not subscribe to PointCloud2 topics."
echo "[pointcloud-subscribers] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[pointcloud-subscribers] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"
echo "[pointcloud-subscribers] NJRH_FASTDDS_PROFILE_ENABLED=${NJRH_FASTDDS_PROFILE_ENABLED:-unset}"

tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_subscribers_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

for topic in "${TOPICS[@]}"; do
  safe_name="${topic//\//_}"
  info_file="${tmp_dir}/${safe_name}.info"
  topic_info "${topic}" "${info_file}"

  publisher_count="$(count_from_info "${info_file}" "Publisher count")"
  subscription_count="$(count_from_info "${info_file}" "Subscription count")"
  publisher_count="${publisher_count:-0}"
  subscription_count="${subscription_count:-0}"
  nodes="$(node_names_from_info "${info_file}")"
  qos_summary="$(qos_summary_from_info "${info_file}")"

  echo "[pointcloud-subscribers] ${topic}: publishers=${publisher_count} subscribers=${subscription_count} nodes=${nodes:-none} ${qos_summary}"

  if [[ "${topic}" == "/lidar_points" ]]; then
    if [[ "${publisher_count}" -ne 1 ]]; then
      echo "[pointcloud-subscribers] FAIL /lidar_points publisher count must be exactly 1, got ${publisher_count}"
      status=1
    fi
    if [[ "${subscription_count}" -gt 2 ]]; then
      echo "[pointcloud-subscribers] WARN /lidar_points has ${subscription_count} subscribers; inspect full-density fan-out"
    fi
    if grep -Eiq 'rviz|foxglove|rosbag|debug|probe|runtime_health_guard|dashboard|robot_api_server' "${info_file}"; then
      echo "[pointcloud-subscribers] WARN debug/dashboard/runtime observer is attached to /lidar_points"
    fi
    if grep -Eiq 'fast[-_]?lio|fastlio' "${info_file}" &&
      [[ "${NJRH_MAPPING_ACTIVE:-false}" != "true" && "${NJRH_ALLOW_FASTLIO_LIDAR_SUBSCRIBER:-false}" != "true" ]]
    then
      echo "[pointcloud-subscribers] WARN FAST-LIO appears attached to /lidar_points outside an explicit mapping/diagnostic run"
    fi
  fi
done

if [[ "${status}" -eq 0 ]]; then
  echo "[pointcloud-subscribers] PASS"
else
  echo "[pointcloud-subscribers] FAIL"
fi
exit "${status}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

CONFIG_FILE="${NJRH_POINTCLOUD_TOPOLOGY_CONFIG:-${REPO_ROOT}/scripts/jetson/runtime_overlay/config/jt128_canonical_pointcloud_remap.yaml}"
NAV2_CONFIG_FILE="${NJRH_NAV2_TOPOLOGY_CONFIG:-${REPO_ROOT}/scripts/jetson/runtime_overlay/config/nav2.yaml}"

status=0

fail() {
  echo "[pointcloud-topology] FAIL $*"
  status=1
}

pass() {
  echo "[pointcloud-topology] PASS $*"
}

param_value() {
  local key="$1"
  awk -v key="${key}" '
    $1 == key ":" {
      value = $0
      sub("^[[:space:]]*" key ":[[:space:]]*", "", value)
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      print value
      exit
    }
  ' "${CONFIG_FILE}"
}

require_value() {
  local key="$1"
  local expected="$2"
  local value
  value="$(param_value "${key}")"
  if [[ "${value}" == "${expected}" ]]; then
    pass "${key}=${expected}"
  else
    fail "${key} expected ${expected}, got ${value:-missing}"
  fi
}

require_int_ge() {
  local key="$1"
  local minimum="$2"
  local value
  value="$(param_value "${key}")"
  if awk -v value="${value:-nan}" -v minimum="${minimum}" 'BEGIN {exit(value + 0 >= minimum + 0 ? 0 : 1)}'; then
    pass "${key}=${value} >= ${minimum}"
  else
    fail "${key} expected >= ${minimum}, got ${value:-missing}"
  fi
}

count_from_info() {
  awk -F: -v key="$2" '$1 == key {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}' "$1"
}

[[ -f "${CONFIG_FILE}" ]] || {
  echo "[pointcloud-topology] FAIL missing config: ${CONFIG_FILE}"
  exit 1
}
[[ -f "${NAV2_CONFIG_FILE}" ]] || {
  echo "[pointcloud-topology] FAIL missing Nav2 config: ${NAV2_CONFIG_FILE}"
  exit 1
}

require_value input_topic /jt128/vendor/points_raw
require_value output_topic /lidar_points
require_value local_output_topic '""'
require_value input_reliable false
require_value output_reliable false
require_int_ge nav_output_stride 2
require_int_ge nav_output_publish_every_n 2

if grep -Eq '^[[:space:]]*local_output_topic:[[:space:]]*/[^[:space:]]+[[:space:]]*$' "${CONFIG_FILE}"; then
  fail "baseline config must not enable a local PointCloud2 branch"
else
  pass "baseline config keeps retired local PointCloud2 branch disabled"
fi

if grep -Eq '^[[:space:]]*(input_reliable|output_reliable):[[:space:]]*true[[:space:]]*$' "${CONFIG_FILE}"; then
  fail "production pointcloud QoS must not be reliable"
else
  pass "production pointcloud QoS remains best-effort"
fi

if grep -q 'plugin: "nav2_costmap_2d::ObstacleLayer"' "${NAV2_CONFIG_FILE}" &&
  grep -q 'observation_sources: scan' "${NAV2_CONFIG_FILE}" &&
  grep -q 'topic: /scan' "${NAV2_CONFIG_FILE}" &&
  grep -q 'data_type: LaserScan' "${NAV2_CONFIG_FILE}" &&
  grep -q 'inf_is_valid: true' "${NAV2_CONFIG_FILE}"
then
  pass "Nav2 local costmap uses standard /scan ObstacleLayer marking and clearing"
else
  fail "Nav2 local costmap does not match standard /scan ObstacleLayer contract"
fi

if grep -q '/perception/' "${NAV2_CONFIG_FILE}"; then
  fail "Nav2 config must not consume retired perception cloud topics"
else
  pass "Nav2 config does not consume retired perception cloud topics"
fi

publisher_matches="$(grep -R -n -E '^[[:space:]]*output_topic:[[:space:]]*/lidar_points[[:space:]]*$' \
  "${REPO_ROOT}/scripts/jetson/runtime_overlay/config" \
  "${REPO_ROOT}/src/robot_hesai_jt128/config" 2>/dev/null || true)"
unexpected_publisher_matches="$(
  printf '%s\n' "${publisher_matches}" | awk '
    NF &&
      $0 !~ /jt128_canonical_pointcloud_remap[.]yaml:/ &&
      $0 !~ /hesai_accel_driver[.]yaml:/ &&
      $0 !~ /pointcloud_accel_axis[.]yaml:/ {print}
  '
)"
publisher_count="$(printf '%s\n' "${publisher_matches}" | awk 'NF {count += 1} END {print count + 0}')"
if [[ "${publisher_count}" -ge 1 && -z "${unexpected_publisher_matches}" ]]; then
  pass "only canonical driver/axis pointcloud profiles publish /lidar_points"
else
  fail "unexpected /lidar_points output_topic matches: ${unexpected_publisher_matches:-none}"
fi

graph_mode="${NJRH_POINTCLOUD_TOPOLOGY_CHECK_GRAPH:-auto}"
if [[ "${graph_mode}" != "false" ]] && command -v ros2 >/dev/null 2>&1; then
  graph_available="false"
  if timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "/lidar_points"; then
    graph_available="true"
  fi
  if [[ "${graph_available}" == "true" ]]; then
    tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_topology_XXXXXX)"
    trap 'rm -rf "${tmp_dir}"' EXIT
    for topic in /lidar_points /scan; do
      safe="${topic//\//_}"
      timeout 8 ros2 topic info -v "${topic}" >"${tmp_dir}/${safe}.info" 2>&1 || true
    done

    lidar_publishers="$(count_from_info "${tmp_dir}/_lidar_points.info" "Publisher count")"
    scan_publishers="$(count_from_info "${tmp_dir}/_scan.info" "Publisher count")"
    scan_subscribers="$(count_from_info "${tmp_dir}/_scan.info" "Subscription count")"

    [[ "${lidar_publishers:-0}" -eq 1 ]] && pass "/lidar_points graph publisher count is 1" || fail "/lidar_points graph publisher count expected 1, got ${lidar_publishers:-0}"
    [[ "${scan_publishers:-0}" -ge 1 ]] && pass "/scan graph publisher is present" || fail "/scan graph publisher missing"
    [[ "${scan_subscribers:-0}" -ge 2 ]] && pass "/scan graph has Nav2/collision subscribers" || fail "/scan graph subscriber count too low: ${scan_subscribers:-0}"
  elif [[ "${graph_mode}" == "true" ]]; then
    fail "ROS graph check requested but /lidar_points is not present"
  else
    pass "ROS graph pointcloud topics not present; static topology checks only"
  fi
fi

if [[ "${status}" -eq 0 ]]; then
  echo "[pointcloud-topology] PASS"
else
  echo "[pointcloud-topology] FAIL"
fi
exit "${status}"

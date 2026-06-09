#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"
njrh_load_local_perception_input_profile
CONFIG_FILE="${NJRH_POINTCLOUD_TOPOLOGY_CONFIG:-${REPO_ROOT}/scripts/jetson/runtime_overlay/config/jt128_canonical_pointcloud_remap.yaml}"

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

[[ -f "${CONFIG_FILE}" ]] || {
  echo "[pointcloud-topology] FAIL missing config: ${CONFIG_FILE}"
  exit 1
}

require_value input_topic /jt128/vendor/points_raw
require_value output_topic /lidar_points
require_value local_output_topic '""'
require_value input_reliable false
require_value output_reliable false
require_int_ge nav_output_stride 2
require_int_ge nav_output_publish_every_n 2

if [[ "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" == "local_branch" ]]; then
  if [[ "${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC}" == "/_internal/lidar_points_local" ]]; then
    pass "local_branch profile derives /_internal/lidar_points_local"
  else
    fail "local_branch profile expected /_internal/lidar_points_local, got ${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC:-empty}"
  fi
  if [[ "${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}" == "/_internal/lidar_points_local" ]]; then
    pass "local_branch profile routes robot_local_perception to /_internal/lidar_points_local"
  else
    fail "local_branch profile expected robot_local_perception input /_internal/lidar_points_local, got ${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC:-missing}"
  fi
  if awk -v value="${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE}" 'BEGIN {exit(value + 0 >= 2 ? 0 : 1)}'; then
    pass "local_branch local_output_stride=${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE} >= 2"
  else
    fail "local_branch local_output_stride expected >= 2, got ${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE:-missing}"
  fi
  if awk -v value="${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N}" 'BEGIN {exit(value + 0 >= 1 ? 0 : 1)}'; then
    pass "local_branch local_output_publish_every_n=${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N} >= 1"
  else
    fail "local_branch local_output_publish_every_n expected >= 1, got ${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N:-missing}"
  fi
else
  if [[ -z "${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC}" ]]; then
    pass "trunk profile disables local internal branch"
  else
    fail "trunk profile must disable local_output_topic, got ${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC}"
  fi
  if [[ "${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}" == "/lidar_points" ]]; then
    pass "trunk profile routes robot_local_perception to /lidar_points"
  else
    fail "trunk profile expected robot_local_perception input /lidar_points, got ${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC:-missing}"
  fi
fi

if grep -Eq '^[[:space:]]*local_output_topic:[[:space:]]*/_internal/lidar_points_local[[:space:]]*$' "${CONFIG_FILE}"; then
  fail "baseline config must not directly enable /_internal/lidar_points_local; use NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=local_branch"
else
  pass "baseline config keeps local internal branch disabled"
fi

if grep -Eq '^[[:space:]]*(input_reliable|output_reliable):[[:space:]]*true[[:space:]]*$' "${CONFIG_FILE}"; then
  fail "production pointcloud QoS must not be reliable"
else
  pass "production pointcloud QoS remains best-effort"
fi

publisher_matches="$(grep -R -n -E '^[[:space:]]*output_topic:[[:space:]]*/lidar_points[[:space:]]*$' \
  "${REPO_ROOT}/scripts/jetson/runtime_overlay/config" \
  "${REPO_ROOT}/src/robot_hesai_jt128/config" 2>/dev/null || true)"
unexpected_publisher_matches="$(
  printf '%s\n' "${publisher_matches}" | awk '
    NF && $0 !~ /jt128_canonical_pointcloud_remap(_local_branch)?[.]yaml:/ {print}
  '
)"
publisher_count="$(printf '%s\n' "${publisher_matches}" | awk 'NF {count += 1} END {print count + 0}')"
if [[ "${publisher_count}" -ge 1 && -z "${unexpected_publisher_matches}" ]]; then
  pass "only canonical pointcloud_axis_remap profiles publish /lidar_points"
else
  fail "unexpected /lidar_points output_topic matches: ${unexpected_publisher_matches:-none}"
fi

if grep -Eq '^[[:space:]]*nav_output_topic:[[:space:]]*/lidar_points_nav[[:space:]]*$' "${CONFIG_FILE}"; then
  pass "/lidar_points_nav is present as a derived branch"
else
  fail "missing derived /lidar_points_nav branch"
fi

graph_mode="${NJRH_POINTCLOUD_TOPOLOGY_CHECK_GRAPH:-auto}"
if [[ "${graph_mode}" != "false" ]] && command -v ros2 >/dev/null 2>&1; then
  graph_available="false"
  if timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "/lidar_points"; then
    graph_available="true"
  fi
  if [[ "${graph_available}" == "true" ]]; then
    lidar_info="$(timeout 8 ros2 topic info -v /lidar_points 2>&1 || true)"
    lidar_publishers="$(printf '%s\n' "${lidar_info}" | awk -F: '$1 == "Publisher count" {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}')"
    lidar_subscribers="$(printf '%s\n' "${lidar_info}" | awk -F: '$1 == "Subscription count" {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}')"
    lidar_publishers="${lidar_publishers:-0}"
    lidar_subscribers="${lidar_subscribers:-0}"
    if [[ "${lidar_publishers}" -eq 1 ]]; then
      pass "/lidar_points graph publisher count is 1"
    else
      fail "/lidar_points graph publisher count expected 1, got ${lidar_publishers}"
    fi
    if printf '%s\n' "${lidar_info}" | grep -q "Node name: pointcloud_axis_remap"; then
      pass "/lidar_points graph includes pointcloud_axis_remap"
    else
      fail "/lidar_points graph missing pointcloud_axis_remap"
    fi
    if [[ "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" == "local_branch" ]]; then
      if [[ "${lidar_subscribers}" -le 1 ]]; then
        pass "/lidar_points graph subscriber count is low for local_branch: ${lidar_subscribers}"
      else
        fail "/lidar_points has ${lidar_subscribers} subscribers in local_branch profile"
      fi
      local_info="$(timeout 8 ros2 topic info -v /_internal/lidar_points_local 2>&1 || true)"
      local_publishers="$(printf '%s\n' "${local_info}" | awk -F: '$1 == "Publisher count" {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}')"
      local_subscribers="$(printf '%s\n' "${local_info}" | awk -F: '$1 == "Subscription count" {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2; exit}')"
      local_publishers="${local_publishers:-0}"
      local_subscribers="${local_subscribers:-0}"
      if [[ "${local_publishers}" -eq 1 && "${local_subscribers}" -eq 1 ]]; then
        pass "/_internal/lidar_points_local graph publisher/subscriber count is 1/1"
      else
        fail "/_internal/lidar_points_local graph expected publishers=1 subscribers=1, got publishers=${local_publishers} subscribers=${local_subscribers}"
      fi
      if printf '%s\n' "${local_info}" | grep -q "Node name: pointcloud_axis_remap"; then
        pass "/_internal/lidar_points_local publisher is pointcloud_axis_remap"
      else
        fail "/_internal/lidar_points_local graph missing pointcloud_axis_remap"
      fi
      if printf '%s\n' "${local_info}" | grep -q "Node name: robot_local_perception"; then
        pass "/_internal/lidar_points_local subscriber includes robot_local_perception"
      else
        fail "/_internal/lidar_points_local graph missing robot_local_perception"
      fi
    fi
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

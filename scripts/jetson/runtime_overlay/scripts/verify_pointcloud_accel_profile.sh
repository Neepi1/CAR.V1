#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile

AXIS_STATUS_TOPIC="${NJRH_POINTCLOUD_ACCEL_AXIS_STATUS_TOPIC:-/lidar/axis_remap_status}"
ACCEL_STATUS_TOPIC="${NJRH_POINTCLOUD_ACCEL_STATUS_TOPIC:-/lidar/pointcloud_accel_status}"
LOCAL_STATUS_TOPIC="${NJRH_POINTCLOUD_ACCEL_LOCAL_STATUS_TOPIC:-/perception/local_perception_status}"
MIN_TRUNK_HZ="${NJRH_POINTCLOUD_ACCEL_MIN_TRUNK_HZ:-18.0}"
MIN_OBSTACLE_HZ="${NJRH_POINTCLOUD_ACCEL_MIN_OBSTACLE_HZ:-10.0}"
MIN_SCAN_HZ="${NJRH_POINTCLOUD_ACCEL_MIN_SCAN_HZ:-8.0}"
TMP_DIR="$(mktemp -d /tmp/njrh_pointcloud_accel_verify_XXXX)"
trap 'rm -rf "${TMP_DIR}"' EXIT

status=0

field_value() {
  local text="$1"
  local key="$2"
  awk -v k="${key}" '{
    for (i = 1; i <= NF; ++i) {
      split($i, kv, "=")
      if (kv[1] == k) {
        print substr($i, length(k) + 2)
        exit
      }
    }
  }' <<<"${text}"
}

float_ge() {
  awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN {exit !(a >= b)}'
}

float_lt() {
  awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN {exit !(a < b)}'
}

topic_count() {
  local topic="$1"
  local label="$2"
  timeout 8 ros2 topic info -v "${topic}" >"${TMP_DIR}/${label}.info" 2>&1 || true
  awk -F: -v key="$3" '$1 ~ key {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}' "${TMP_DIR}/${label}.info"
}

status_once() {
  local topic="$1"
  timeout 8 ros2 topic echo "${topic}" --field data --once 2>/dev/null || true
}

light_hz() {
  local topic="$1"
  local output
  output="$(timeout 14 ros2 topic hz "${topic}" 2>/dev/null || true)"
  awk '/average rate:/ {value=$3} END {if (value != "") print value}' <<<"${output}"
}

echo "[pointcloud-accel-verify] current NJRH_POINTCLOUD_ACCEL_PROFILE=${NJRH_POINTCLOUD_ACCEL_PROFILE}"
echo "[pointcloud-accel-verify] profile_source=${NJRH_POINTCLOUD_ACCEL_PROFILE_SOURCE}"
echo "[pointcloud-accel-verify] DDS transport env: RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"
echo "[pointcloud-accel-verify] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[pointcloud-accel-verify] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"

lidar_publishers="$(topic_count /lidar_points lidar "Publisher count" || true)"
lidar_subscribers="$(topic_count /lidar_points lidar "Subscription count" || true)"
echo "[pointcloud-accel-verify] /lidar_points publishers=${lidar_publishers:-0} subscribers=${lidar_subscribers:-0}"
if [[ "${lidar_publishers:-0}" != "1" ]]; then
  echo "[pointcloud-accel-verify] FAIL /lidar_points publisher count must be 1"
  status=1
fi

axis_status="$(status_once "${AXIS_STATUS_TOPIC}")"
accel_status="$(status_once "${ACCEL_STATUS_TOPIC}")"
local_status="$(status_once "${LOCAL_STATUS_TOPIC}")"
if [[ -n "${axis_status}" ]]; then
  echo "[pointcloud-accel-verify] axis_status=${axis_status}"
fi
if [[ -n "${accel_status}" ]]; then
  echo "[pointcloud-accel-verify] accel_status=${accel_status}"
fi
if [[ -n "${local_status}" && "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
  echo "[pointcloud-accel-verify] local_status=${local_status}"
fi

axis_hz="$(field_value "${axis_status}" fast_path_lidar_points_publish_hz)"
[[ -z "${axis_hz}" ]] && axis_hz="$(field_value "${axis_status}" lidar_points_publish_hz)"
echo "[pointcloud-accel-verify] axis publish hz=${axis_hz:-missing}"
if [[ -z "${axis_hz}" ]] || ! float_ge "${axis_hz}" "${MIN_TRUNK_HZ}"; then
  echo "[pointcloud-accel-verify] FAIL /lidar_points full trunk is below ${MIN_TRUNK_HZ}Hz or status is missing"
  status=1
fi

obstacle_hz="$(field_value "${accel_status}" local_worker_obstacle_publish_hz)"
if [[ -z "${obstacle_hz}" ]]; then
  obstacle_hz="$(field_value "${local_status}" published_obstacle_hz)"
fi
clearing_hz="$(field_value "${accel_status}" local_worker_clearing_publish_hz)"
if [[ -z "${clearing_hz}" ]]; then
  clearing_hz="$(field_value "${local_status}" published_clearing_hz)"
fi
scan_hz="$(field_value "${accel_status}" scan_worker_scan_publish_hz)"
if [[ -z "${scan_hz}" ]]; then
  scan_hz="$(light_hz /scan)"
fi
flatscan_hz="$(light_hz /flatscan)"

echo "[pointcloud-accel-verify] obstacle_hz=${obstacle_hz:-missing}"
echo "[pointcloud-accel-verify] clearing_hz=${clearing_hz:-missing}"
echo "[pointcloud-accel-verify] scan_hz=${scan_hz:-missing}"
echo "[pointcloud-accel-verify] flatscan_hz=${flatscan_hz:-missing}"

if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]]; then
  if [[ -z "${obstacle_hz}" ]] || ! float_ge "${obstacle_hz}" "${MIN_OBSTACLE_HZ}"; then
    if [[ -n "${obstacle_hz}" ]] && float_ge "${obstacle_hz}" 9.0; then
      echo "[pointcloud-accel-verify] WARN obstacle is 9-10Hz but stable"
    else
      echo "[pointcloud-accel-verify] FAIL obstacle < ${MIN_OBSTACLE_HZ}Hz"
      status=1
    fi
  fi
  if [[ -z "${scan_hz}" ]] || ! float_ge "${scan_hz}" "${MIN_SCAN_HZ}"; then
    if [[ -n "${scan_hz}" ]] && float_ge "${scan_hz}" 7.0; then
      echo "[pointcloud-accel-verify] WARN scan is 7-8Hz"
    else
      echo "[pointcloud-accel-verify] FAIL scan < ${MIN_SCAN_HZ}Hz"
      status=1
    fi
  fi
fi

fastlio_residual=false
if pgrep -f "fast_lio|fastlio|laser_mapping" >/dev/null 2>&1; then
  fastlio_residual=true
fi
echo "[pointcloud-accel-verify] FAST-LIO2 residual=${fastlio_residual}"
if [[ "${fastlio_residual}" == "true" ]]; then
  echo "[pointcloud-accel-verify] FAIL FAST-LIO2 navigation residue detected"
  status=1
else
  echo "[pointcloud-accel-verify] PASS no FAST-LIO2 navigation residue"
fi

if timeout 5 ros2 lifecycle get /bt_navigator >/dev/null 2>&1; then
  echo "[pointcloud-accel-verify] Nav2 lifecycle: $(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
else
  echo "[pointcloud-accel-verify] WARN Nav2 lifecycle unavailable"
fi

timeout 8 ros2 topic info -v /perception/obstacle_points >"${TMP_DIR}/obstacle.info" 2>&1 || true
if grep -q "/local_costmap" "${TMP_DIR}/obstacle.info"; then
  echo "[pointcloud-accel-verify] PASS local_costmap subscribes /perception/obstacle_points"
else
  echo "[pointcloud-accel-verify] WARN local_costmap subscriber not observed on /perception/obstacle_points"
fi
if grep -q "collision_monitor" "${TMP_DIR}/obstacle.info"; then
  echo "[pointcloud-accel-verify] PASS collision_monitor subscribes /perception/obstacle_points"
else
  echo "[pointcloud-accel-verify] WARN collision_monitor subscriber not observed on /perception/obstacle_points"
fi

if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
  echo "[pointcloud-accel-verify] production_hop_internal_local_branch=true"
  echo "[pointcloud-accel-verify] production_hop_points_nav=true"
else
  echo "[pointcloud-accel-verify] production_hop_internal_local_branch=false"
  echo "[pointcloud-accel-verify] production_hop_points_nav=false"
fi

echo "[pointcloud-accel-verify] PointCloud2 QoS snapshot:"
grep -E "Reliability|Depth|Publisher count|Subscription count" "${TMP_DIR}/lidar.info" || true
echo "[pointcloud-accel-verify] CPU per-core snapshot:"
top -b -n1 | awk '/^%Cpu/ || /^Cpu/ {print}' || true

if [[ "${status}" -eq 0 ]]; then
  echo "[pointcloud-accel-verify] PASS"
else
  echo "[pointcloud-accel-verify] FAIL"
fi
exit "${status}"

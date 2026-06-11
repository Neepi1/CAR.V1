#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"

REQUESTED_PROFILE="${NJRH_POINTCLOUD_ACCEL_REQUESTED_PROFILE:-${NJRH_POINTCLOUD_ACCEL_PROFILE:-profile-file}}"
njrh_load_pointcloud_accel_profile
njrh_load_pointcloud_ingress_profile
RESOLVED_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE}"
RESOLVED_INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE}"

AXIS_STATUS_TOPIC="${NJRH_POINTCLOUD_ACCEL_AXIS_STATUS_TOPIC:-/lidar/axis_remap_status}"
ACCEL_STATUS_TOPIC="${NJRH_POINTCLOUD_ACCEL_STATUS_TOPIC:-/lidar/pointcloud_accel_status}"
LOCAL_STATUS_TOPIC="${NJRH_POINTCLOUD_ACCEL_LOCAL_STATUS_TOPIC:-/perception/local_perception_status}"
ACCEL_CONFIG="${NJRH_POINTCLOUD_ACCEL_AXIS_CONFIG:-${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_axis.yaml}"
MIN_TRUNK_HZ="${NJRH_POINTCLOUD_ACCEL_MIN_TRUNK_HZ:-18.0}"
MIN_OBSTACLE_HZ="${NJRH_POINTCLOUD_ACCEL_MIN_OBSTACLE_HZ:-10.0}"
MIN_SCAN_HZ="${NJRH_POINTCLOUD_ACCEL_MIN_SCAN_HZ:-8.0}"
MIN_FLATSCAN_HZ="${NJRH_FLATSCAN_MIN_HZ:-5.0}"
FLATSCAN_STATUS_FILE="${NJRH_FLATSCAN_HELPER_STATUS_FILE:-${NJRH_RUNTIME_LOG_DIR}/flatscan_helper_status.env}"
ALLOW_LOCAL_PERCEPTION_FALLBACK="${NJRH_POINTCLOUD_ACCEL_ALLOW_LOCAL_PERCEPTION_FALLBACK:-false}"
ALLOW_LEGACY_SCAN_FALLBACK="${NJRH_POINTCLOUD_ACCEL_ALLOW_LEGACY_SCAN_FALLBACK:-false}"
TMP_DIR="$(mktemp -d /tmp/njrh_pointcloud_accel_verify_XXXX)"
trap 'rm -rf "${TMP_DIR}"' EXIT

status=0
profile_owner_contract_ok=true
legacy_scan_chain_ok=true
ipc_worker_owner_ok=true
trunk_full_density_ok=true
nav2_compat_topics_ok=true
flatscan_owner_ok=false
flatscan_hz_ok=false
flatscan_nav_startup_gate_ok=false

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

topic_info() {
  local topic="$1"
  local label="$2"
  timeout 8 ros2 topic info -v "${topic}" >"${TMP_DIR}/${label}.info" 2>&1 || true
}

topic_count() {
  local label="$1"
  local key="$2"
  local value
  value="$(awk -F: -v k="${key}" '$1 ~ k {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}' "${TMP_DIR}/${label}.info" 2>/dev/null || true)"
  echo "${value:-0}"
}

publisher_nodes() {
  local label="$1"
  awk '
    /^[[:space:]]*Node name:/ {
      node=$0
      sub(/^[[:space:]]*Node name:[[:space:]]*/, "", node)
    }
    /^[[:space:]]*Endpoint type:/ {
      endpoint=$0
      if (endpoint ~ /PUBLISHER/ && node != "") {
        print node
      }
    }
  ' "${TMP_DIR}/${label}.info" 2>/dev/null | sort -u
}

nodes_csv() {
  awk 'NF {if (out != "") out = out ", " $0; else out = $0} END {print out}'
}

node_list_has() {
  local nodes="$1"
  local pattern="$2"
  grep -Eq "${pattern}" <<<"${nodes}"
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

fail() {
  echo "[pointcloud-accel-verify] FAIL $*"
  status=1
}

warn() {
  echo "[pointcloud-accel-verify] WARN $*"
}

mark_profile_fail() {
  profile_owner_contract_ok=false
}

mark_legacy_scan_fail() {
  legacy_scan_chain_ok=false
  nav2_compat_topics_ok=false
}

mark_ipc_fail() {
  ipc_worker_owner_ok=false
}

mark_trunk_fail() {
  trunk_full_density_ok=false
}

mark_nav2_fail() {
  nav2_compat_topics_ok=false
}

for spec in \
  "/lidar_points:lidar" \
  "/jt128/vendor/points_raw:vendor_raw" \
  "/perception/obstacle_points:obstacle" \
  "/perception/clearing_points:clearing" \
  "/points_nav:points_nav" \
  "/lidar_points_nav:lidar_points_nav" \
  "/_internal/lidar_points_local:internal_local" \
  "/scan:scan" \
  "/flatscan:flatscan"; do
  topic="${spec%%:*}"
  label="${spec##*:}"
  topic_info "${topic}" "${label}"
done

echo "[pointcloud-accel-verify] requested_profile=${REQUESTED_PROFILE}"
echo "[pointcloud-accel-verify] resolved_profile=${RESOLVED_PROFILE}"
echo "[pointcloud-accel-verify] ingress_profile=${RESOLVED_INGRESS_PROFILE}"
echo "[pointcloud-accel-verify] current NJRH_POINTCLOUD_ACCEL_PROFILE=${RESOLVED_PROFILE}"
echo "[pointcloud-accel-verify] profile_source=${NJRH_POINTCLOUD_ACCEL_PROFILE_SOURCE}"
echo "[pointcloud-accel-verify] ingress_profile_source=${NJRH_POINTCLOUD_INGRESS_PROFILE_SOURCE}"
echo "[pointcloud-accel-verify] DDS transport env: RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"
echo "[pointcloud-accel-verify] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[pointcloud-accel-verify] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"

FLATSCAN_HELPER_MODE="missing"
FLATSCAN_HELPER_PID=""
FLATSCAN_HELPER_RESTART_COUNT=""
if [[ -f "${FLATSCAN_STATUS_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${FLATSCAN_STATUS_FILE}"
fi
echo "[pointcloud-accel-verify] flatscan_helper_status_file=${FLATSCAN_STATUS_FILE}"
echo "[pointcloud-accel-verify] flatscan_helper_mode=${FLATSCAN_HELPER_MODE:-missing}"
echo "[pointcloud-accel-verify] flatscan_helper_pid=${FLATSCAN_HELPER_PID:-missing}"
echo "[pointcloud-accel-verify] flatscan_helper_restart_count=${FLATSCAN_HELPER_RESTART_COUNT:-missing}"

lidar_publishers="$(topic_count lidar "Publisher count")"
lidar_subscribers="$(topic_count lidar "Subscription count")"
vendor_raw_publishers="$(topic_count vendor_raw "Publisher count")"
vendor_raw_subscribers="$(topic_count vendor_raw "Subscription count")"
obstacle_publishers="$(topic_count obstacle "Publisher count")"
obstacle_subscribers="$(topic_count obstacle "Subscription count")"
clearing_publishers="$(topic_count clearing "Publisher count")"
clearing_subscribers="$(topic_count clearing "Subscription count")"
points_nav_publishers="$(topic_count points_nav "Publisher count")"
points_nav_subscribers="$(topic_count points_nav "Subscription count")"
lidar_points_nav_publishers="$(topic_count lidar_points_nav "Publisher count")"
internal_local_publishers="$(topic_count internal_local "Publisher count")"
scan_publishers="$(topic_count scan "Publisher count")"
scan_subscribers="$(topic_count scan "Subscription count")"
flatscan_publishers="$(topic_count flatscan "Publisher count")"
flatscan_subscribers="$(topic_count flatscan "Subscription count")"

lidar_publisher_nodes="$(publisher_nodes lidar || true)"
vendor_raw_publisher_nodes="$(publisher_nodes vendor_raw || true)"
obstacle_publisher_nodes="$(publisher_nodes obstacle || true)"
clearing_publisher_nodes="$(publisher_nodes clearing || true)"
points_nav_publisher_nodes="$(publisher_nodes points_nav || true)"
scan_publisher_nodes="$(publisher_nodes scan || true)"
flatscan_publisher_nodes="$(publisher_nodes flatscan || true)"

lidar_owner="$(nodes_csv <<<"${lidar_publisher_nodes}")"
vendor_raw_owner="$(nodes_csv <<<"${vendor_raw_publisher_nodes}")"
obstacle_owner="$(nodes_csv <<<"${obstacle_publisher_nodes}")"
clearing_owner="$(nodes_csv <<<"${clearing_publisher_nodes}")"
points_nav_owner="$(nodes_csv <<<"${points_nav_publisher_nodes}")"
scan_owner="$(nodes_csv <<<"${scan_publisher_nodes}")"
flatscan_owner="$(nodes_csv <<<"${flatscan_publisher_nodes}")"

echo "[pointcloud-accel-verify] /lidar_points publisher count=${lidar_publishers} subscriber count=${lidar_subscribers}"
echo "[pointcloud-accel-verify] /lidar_points publisher_nodes=${lidar_owner}"
echo "[pointcloud-accel-verify] actual trunk owner=${lidar_owner:-missing}"
echo "[pointcloud-accel-verify] /jt128/vendor/points_raw publishers=${vendor_raw_publishers} subscribers=${vendor_raw_subscribers} publisher_nodes=${vendor_raw_owner}"
echo "[pointcloud-accel-verify] /perception/obstacle_points publishers=${obstacle_publishers} subscribers=${obstacle_subscribers} publisher_nodes=${obstacle_owner}"
echo "[pointcloud-accel-verify] actual obstacle owner=${obstacle_owner:-missing}"
echo "[pointcloud-accel-verify] /perception/clearing_points publishers=${clearing_publishers} subscribers=${clearing_subscribers} publisher_nodes=${clearing_owner}"
echo "[pointcloud-accel-verify] actual clearing owner=${clearing_owner:-missing}"
echo "[pointcloud-accel-verify] /points_nav publishers=${points_nav_publishers} subscribers=${points_nav_subscribers} publisher_nodes=${points_nav_owner}"
echo "[pointcloud-accel-verify] actual points_nav owner=${points_nav_owner:-missing}"
echo "[pointcloud-accel-verify] /lidar_points_nav publishers=${lidar_points_nav_publishers}"
echo "[pointcloud-accel-verify] /_internal/lidar_points_local publishers=${internal_local_publishers}"
echo "[pointcloud-accel-verify] /scan publishers=${scan_publishers} subscribers=${scan_subscribers} publisher_nodes=${scan_owner}"
echo "[pointcloud-accel-verify] actual scan owner=${scan_owner:-missing}"
echo "[pointcloud-accel-verify] /flatscan publishers=${flatscan_publishers} subscribers=${flatscan_subscribers} publisher_nodes=${flatscan_owner}"
echo "[pointcloud-accel-verify] actual flatscan owner=${flatscan_owner:-missing}"

if [[ "${lidar_publishers}" != "1" ]]; then
  fail "/lidar_points publisher count must be 1"
  mark_profile_fail
  mark_trunk_fail
fi

if [[ -f "${ACCEL_CONFIG}" ]] && grep -Eq '^[[:space:]]*(output_stride|output_downsample|output_compact|output_compact_fields|output_compact_stride)[[:space:]]*:' "${ACCEL_CONFIG}"; then
  fail "/lidar_points trunk appears compact/downsampled by config ${ACCEL_CONFIG}"
  mark_trunk_fail
else
  echo "[pointcloud-accel-verify] PASS /lidar_points trunk config has no compact/downsample output knobs"
fi

case "${RESOLVED_PROFILE}" in
  legacy)
    if node_list_has "${lidar_publisher_nodes}" '(^|[[:space:]])pointcloud_axis_remap(_node)?($|[[:space:]])'; then
      echo "[pointcloud-accel-verify] PASS legacy trunk owner is pointcloud_axis_remap"
    else
      fail "legacy /lidar_points publisher must be pointcloud_axis_remap_node or pointcloud_axis_remap"
      mark_profile_fail
      mark_trunk_fail
    fi
    if node_list_has "${lidar_publisher_nodes}" '(^|[[:space:]])pointcloud_accel_axis_node($|[[:space:]])'; then
      fail "legacy must not use pointcloud_accel_axis_node as trunk owner"
      mark_profile_fail
    fi
    ;;
  ipc_worker|nitros)
    if node_list_has "${lidar_publisher_nodes}" '(^|[[:space:]])pointcloud_axis_remap(_node)?($|[[:space:]])'; then
      fail "${RESOLVED_PROFILE} is still using legacy pointcloud_axis_remap as trunk owner"
      mark_profile_fail
      mark_ipc_fail
    fi
    if [[ "${RESOLVED_INGRESS_PROFILE}" == "driver_integrated" ]]; then
      if node_list_has "${lidar_publisher_nodes}" '(^|[[:space:]])hesai_accel_driver_node($|[[:space:]])|(^|[[:space:]])jt128_accel_driver_node($|[[:space:]])'; then
        echo "[pointcloud-accel-verify] PASS ${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} trunk owner is integrated driver"
      else
        fail "${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} /lidar_points publisher must be hesai_accel_driver_node"
        mark_profile_fail
        mark_ipc_fail
        mark_trunk_fail
      fi
    elif node_list_has "${lidar_publisher_nodes}" '(^|[[:space:]])pointcloud_accel_axis_node($|[[:space:]])'; then
      echo "[pointcloud-accel-verify] PASS ${RESOLVED_PROFILE} trunk owner is pointcloud_accel_axis_node"
    else
      fail "${RESOLVED_PROFILE} /lidar_points publisher must be pointcloud_accel_axis_node"
      mark_profile_fail
      mark_ipc_fail
      mark_trunk_fail
    fi
    ;;
esac

if [[ "${RESOLVED_PROFILE}" == "nitros" ]] && ! bash "${SCRIPT_DIR}/check_isaac_ros_nitros_env.sh" >/dev/null 2>&1; then
  fail "NITROS_ENV_GUARDED_UNAVAILABLE; nitros cannot half-pass without the required Isaac ROS environment"
  mark_profile_fail
  mark_ipc_fail
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
if [[ -n "${local_status}" && "${RESOLVED_PROFILE}" == "legacy" ]]; then
  echo "[pointcloud-accel-verify] local_status=${local_status}"
fi

local_worker_enabled="$(field_value "${accel_status}" local_worker_enabled)"
[[ -z "${local_worker_enabled}" ]] && local_worker_enabled="$(field_value "${accel_status}" worker_local_enabled)"
scan_worker_enabled="$(field_value "${accel_status}" scan_worker_enabled)"
[[ -z "${scan_worker_enabled}" ]] && scan_worker_enabled="$(field_value "${accel_status}" worker_scan_enabled)"
internal_zero_copy_profile="$(field_value "${accel_status}" internal_zero_copy_profile)"
status_accel_ingress_profile="$(field_value "${accel_status}" accel_ingress_profile)"
status_input_path="$(field_value "${accel_status}" input_path)"
status_vendor_raw_ros_hop_required="$(field_value "${accel_status}" vendor_raw_ros_hop_required)"
status_vendor_raw_debug_publish_enabled="$(field_value "${accel_status}" vendor_raw_debug_publish_enabled)"
status_driver_integrated_process="$(field_value "${accel_status}" driver_integrated_process)"
status_accel_core_process_pointcloud2_count="$(field_value "${accel_status}" accel_core_process_pointcloud2_count)"
status_accel_core_process_decoded_view_count="$(field_value "${accel_status}" accel_core_process_decoded_view_count)"
latest_internal_buffer_points="$(field_value "${accel_status}" latest_internal_buffer_points)"
local_worker_full_cloud_copy_count="$(field_value "${accel_status}" local_worker_full_cloud_copy_count)"
scan_worker_full_cloud_copy_count="$(field_value "${accel_status}" scan_worker_full_cloud_copy_count)"
local_worker_intermediate_pointcloud_build_count="$(field_value "${accel_status}" local_worker_intermediate_pointcloud_build_count)"
scan_worker_intermediate_pointcloud_build_count="$(field_value "${accel_status}" scan_worker_intermediate_pointcloud_build_count)"
local_worker_lock_wait_ms_max="$(field_value "${accel_status}" local_worker_lock_wait_ms_max)"
scan_worker_lock_wait_ms_max="$(field_value "${accel_status}" scan_worker_lock_wait_ms_max)"
echo "[pointcloud-accel-verify] local_worker_enabled=${local_worker_enabled:-missing}"
echo "[pointcloud-accel-verify] scan_worker_enabled=${scan_worker_enabled:-missing}"
echo "[pointcloud-accel-verify] internal_zero_copy_profile=${internal_zero_copy_profile:-missing}"
echo "[pointcloud-accel-verify] status_accel_ingress_profile=${status_accel_ingress_profile:-missing}"
echo "[pointcloud-accel-verify] status_input_path=${status_input_path:-missing}"
echo "[pointcloud-accel-verify] status_vendor_raw_ros_hop_required=${status_vendor_raw_ros_hop_required:-missing}"
echo "[pointcloud-accel-verify] status_vendor_raw_debug_publish_enabled=${status_vendor_raw_debug_publish_enabled:-missing}"
echo "[pointcloud-accel-verify] status_driver_integrated_process=${status_driver_integrated_process:-missing}"
echo "[pointcloud-accel-verify] status_accel_core_process_pointcloud2_count=${status_accel_core_process_pointcloud2_count:-missing}"
echo "[pointcloud-accel-verify] status_accel_core_process_decoded_view_count=${status_accel_core_process_decoded_view_count:-missing}"
echo "[pointcloud-accel-verify] latest_internal_buffer_points=${latest_internal_buffer_points:-missing}"
echo "[pointcloud-accel-verify] local_worker_full_cloud_copy_count=${local_worker_full_cloud_copy_count:-missing}"
echo "[pointcloud-accel-verify] scan_worker_full_cloud_copy_count=${scan_worker_full_cloud_copy_count:-missing}"
echo "[pointcloud-accel-verify] local_worker_intermediate_pointcloud_build_count=${local_worker_intermediate_pointcloud_build_count:-missing}"
echo "[pointcloud-accel-verify] scan_worker_intermediate_pointcloud_build_count=${scan_worker_intermediate_pointcloud_build_count:-missing}"
echo "[pointcloud-accel-verify] local_worker_lock_wait_ms_max=${local_worker_lock_wait_ms_max:-missing}"
echo "[pointcloud-accel-verify] scan_worker_lock_wait_ms_max=${scan_worker_lock_wait_ms_max:-missing}"

if [[ "${RESOLVED_PROFILE}" == "legacy" ]]; then
  legacy_input_topic="$(field_value "${local_status}" input_topic)"
  if [[ -z "${axis_status}" ]]; then
    fail "${AXIS_STATUS_TOPIC} is required for legacy"
    mark_profile_fail
  fi
  if [[ -z "${local_status}" ]]; then
    fail "${LOCAL_STATUS_TOPIC} is required for legacy"
    mark_profile_fail
  elif [[ "${legacy_input_topic}" == "/_internal/lidar_points_local" ]]; then
    echo "[pointcloud-accel-verify] PASS legacy robot_local_perception input_topic=/_internal/lidar_points_local"
  else
    fail "legacy robot_local_perception input_topic is ${legacy_input_topic:-missing}, expected /_internal/lidar_points_local"
    mark_profile_fail
  fi
else
  if [[ -z "${accel_status}" ]]; then
    fail "${ACCEL_STATUS_TOPIC} is required for ${RESOLVED_PROFILE}"
    mark_profile_fail
    mark_ipc_fail
  fi
  if [[ "${local_worker_enabled:-false}" != "true" ]]; then
    fail "local_worker_enabled must be true for ${RESOLVED_PROFILE}"
    mark_profile_fail
    mark_ipc_fail
  fi
  if [[ "${scan_worker_enabled:-false}" != "true" ]]; then
    fail "scan_worker_enabled must be true for ${RESOLVED_PROFILE}"
    mark_profile_fail
    mark_ipc_fail
  fi
  if [[ "${RESOLVED_INGRESS_PROFILE}" == "separate_process" && "${status_vendor_raw_ros_hop_required:-missing}" == "false" ]]; then
    fail "separate_process status must not claim vendor_raw_ros_hop_required=false"
    mark_profile_fail
  fi
  if [[ "${RESOLVED_INGRESS_PROFILE}" == "driver_integrated" ]]; then
    if [[ "${status_vendor_raw_ros_hop_required:-missing}" != "false" ]]; then
      fail "driver_integrated selected but status does not report vendor_raw_ros_hop_required=false"
      mark_profile_fail
      mark_ipc_fail
    fi
    if [[ "${status_driver_integrated_process:-missing}" != "true" ]]; then
      fail "driver_integrated selected but accel status is not from integrated process"
      mark_profile_fail
      mark_ipc_fail
    fi
  fi
  if [[ "${internal_zero_copy_profile:-false}" != "true" ]]; then
    fail "internal_zero_copy_profile must be true for ${RESOLVED_PROFILE}"
    mark_profile_fail
    mark_ipc_fail
  fi
  if [[ -z "${latest_internal_buffer_points}" ]] || ! float_ge "${latest_internal_buffer_points}" 1.0; then
    fail "latest_internal_buffer_points must be present and nonzero for ${RESOLVED_PROFILE}"
    mark_profile_fail
    mark_ipc_fail
  fi
  if [[ "${local_worker_full_cloud_copy_count:-missing}" != "0" || "${scan_worker_full_cloud_copy_count:-missing}" != "0" ]]; then
    fail "worker full PointCloud2 copy counters must stay zero"
    mark_profile_fail
    mark_ipc_fail
  fi
  if [[ "${local_worker_intermediate_pointcloud_build_count:-missing}" != "0" || "${scan_worker_intermediate_pointcloud_build_count:-missing}" != "0" ]]; then
    fail "worker intermediate PointCloud2 build counters must stay zero"
    mark_profile_fail
    mark_ipc_fail
  fi
fi

axis_hz="$(field_value "${axis_status}" fast_path_lidar_points_publish_hz)"
[[ -z "${axis_hz}" ]] && axis_hz="$(field_value "${axis_status}" lidar_points_publish_hz)"
echo "[pointcloud-accel-verify] axis publish hz=${axis_hz:-missing}"
if [[ -z "${axis_hz}" ]]; then
  fail "/lidar_points full trunk status is missing"
  mark_trunk_fail
elif ! float_ge "${axis_hz}" "${MIN_TRUNK_HZ}"; then
  warn "/lidar_points full trunk publish rate ${axis_hz}Hz is below ${MIN_TRUNK_HZ}Hz; owner/full-density contract remains separate from this performance warning"
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

if [[ "${scan_publishers}" != "0" && "${flatscan_publishers}" == "0" ]]; then
  fail "CASE_FLATSCAN_HELPER_DEAD: /scan has publisher_nodes=${scan_owner:-missing} but /flatscan publisher is missing"
  mark_nav2_fail
fi
if [[ "${flatscan_publishers}" != "0" ]]; then
  if [[ -z "${flatscan_hz}" ]]; then
    fail "/flatscan publisher exists but hz could not be measured"
    mark_nav2_fail
  elif float_ge "${flatscan_hz}" "${MIN_FLATSCAN_HZ}"; then
    flatscan_hz_ok=true
    echo "[pointcloud-accel-verify] PASS FLATSCAN_HZ_OK hz=${flatscan_hz} min=${MIN_FLATSCAN_HZ}"
  else
    fail "/flatscan hz ${flatscan_hz} below ${MIN_FLATSCAN_HZ}"
    mark_nav2_fail
  fi
else
  fail "/flatscan publisher count is 0"
  mark_nav2_fail
fi

if [[ "${RESOLVED_PROFILE}" != "legacy" ]]; then
  if [[ -z "${obstacle_hz}" ]] || ! float_ge "${obstacle_hz}" "${MIN_OBSTACLE_HZ}"; then
    if [[ -n "${obstacle_hz}" ]] && float_ge "${obstacle_hz}" 9.0; then
      warn "obstacle is 9-10Hz but stable"
    else
      fail "obstacle < ${MIN_OBSTACLE_HZ}Hz"
      mark_nav2_fail
    fi
  fi
  if [[ -z "${scan_hz}" ]] || ! float_ge "${scan_hz}" "${MIN_SCAN_HZ}"; then
    if [[ -n "${scan_hz}" ]] && float_ge "${scan_hz}" 7.0; then
      warn "scan is 7-8Hz"
    else
      fail "scan < ${MIN_SCAN_HZ}Hz"
      mark_nav2_fail
    fi
  fi
fi

expected_accel_owner='(^|[[:space:]])pointcloud_accel_axis_node($|[[:space:]])'
if [[ "${RESOLVED_INGRESS_PROFILE}" == "driver_integrated" ]]; then
  expected_accel_owner='(^|[[:space:]])hesai_accel_driver_node($|[[:space:]])|(^|[[:space:]])jt128_accel_driver_node($|[[:space:]])'
fi

if [[ "${RESOLVED_PROFILE}" == "legacy" ]]; then
  if node_list_has "${obstacle_publisher_nodes}" '(^|[[:space:]])robot_local_perception($|[[:space:]])'; then
    echo "[pointcloud-accel-verify] PASS legacy obstacle owner is robot_local_perception"
  else
    fail "legacy obstacle owner must be robot_local_perception"
    mark_profile_fail
  fi
  if node_list_has "${clearing_publisher_nodes}" '(^|[[:space:]])robot_local_perception($|[[:space:]])'; then
    echo "[pointcloud-accel-verify] PASS legacy clearing owner is robot_local_perception"
  else
    fail "legacy clearing owner must be robot_local_perception"
    mark_profile_fail
  fi
  if [[ "${points_nav_publishers}" != "0" ]] && node_list_has "${points_nav_publisher_nodes}" '(^|[[:space:]])nav_cloud_preprocessor($|[[:space:]])'; then
    echo "[pointcloud-accel-verify] PASS legacy points_nav owner is nav_cloud_preprocessor"
  else
    fail "legacy missing /points_nav publisher from nav_cloud_preprocessor"
    mark_legacy_scan_fail
  fi
  if [[ "${scan_publishers}" != "0" ]] && node_list_has "${scan_publisher_nodes}" '(^|[[:space:]])scan_republisher($|[[:space:]])'; then
    echo "[pointcloud-accel-verify] PASS legacy scan owner is scan_republisher"
  else
    fail "legacy missing /scan publisher from scan_republisher"
    mark_legacy_scan_fail
  fi
  if [[ "${flatscan_publishers}" != "0" ]] && node_list_has "${flatscan_publisher_nodes}" '(^|[[:space:]])laser_scan_to_flatscan($|[[:space:]])'; then
    flatscan_owner_ok=true
    echo "[pointcloud-accel-verify] PASS legacy flatscan owner is laser_scan_to_flatscan"
  else
    fail "legacy missing /flatscan publisher from laser_scan_to_flatscan"
    mark_legacy_scan_fail
  fi
else
  if node_list_has "${obstacle_publisher_nodes}" '(^|[[:space:]])robot_local_perception($|[[:space:]])'; then
    if [[ "${ALLOW_LOCAL_PERCEPTION_FALLBACK}" == "true" ]]; then
      warn "${RESOLVED_PROFILE} obstacle owner includes standalone robot_local_perception fallback"
    else
      fail "${RESOLVED_PROFILE} obstacle owner is still standalone robot_local_perception"
      mark_profile_fail
      mark_ipc_fail
    fi
  fi
  if node_list_has "${obstacle_publisher_nodes}" "${expected_accel_owner}"; then
    echo "[pointcloud-accel-verify] PASS ${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} obstacle owner is accel core process"
  else
    fail "${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} obstacle owner must be the selected accel core process unless an explicit fallback is documented"
    mark_profile_fail
    mark_ipc_fail
  fi
  if node_list_has "${clearing_publisher_nodes}" "${expected_accel_owner}"; then
    echo "[pointcloud-accel-verify] PASS ${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} clearing owner is accel core process"
  else
    fail "${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} clearing owner must be the selected accel core process unless an explicit fallback is documented"
    mark_profile_fail
    mark_ipc_fail
  fi
  if node_list_has "${scan_publisher_nodes}" "${expected_accel_owner}"; then
    echo "[pointcloud-accel-verify] PASS ${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} scan owner is accel core process"
  else
    fail "${RESOLVED_PROFILE}/${RESOLVED_INGRESS_PROFILE} scan owner must be the selected accel core process or an explicit accel scan worker"
    mark_profile_fail
    mark_ipc_fail
  fi
  if [[ "${points_nav_publishers}" != "0" ]] || node_list_has "${points_nav_publisher_nodes}" '(^|[[:space:]])nav_cloud_preprocessor($|[[:space:]])'; then
    if [[ "${ALLOW_LEGACY_SCAN_FALLBACK}" == "true" ]]; then
      warn "/points_nav legacy scan fallback observed in ${RESOLVED_PROFILE}"
    else
      fail "/points_nav still has a production publisher in ${RESOLVED_PROFILE}; it must not be the production scan hop"
      mark_profile_fail
      mark_ipc_fail
    fi
  else
    echo "[pointcloud-accel-verify] PASS /points_nav is not a production hop for ${RESOLVED_PROFILE}"
  fi
  if [[ "${internal_local_publishers}" != "0" ]]; then
    echo "[pointcloud-accel-verify] /_internal/lidar_points_local debug_compat_publishers=${internal_local_publishers}"
  fi
  if [[ "${flatscan_publishers}" != "0" ]] && node_list_has "${flatscan_publisher_nodes}" '(^|[[:space:]])laser_scan_to_flatscan($|[[:space:]])'; then
    flatscan_owner_ok=true
    echo "[pointcloud-accel-verify] PASS ${RESOLVED_PROFILE} flatscan compatibility owner is laser_scan_to_flatscan"
  elif [[ "${flatscan_publishers}" != "0" ]] && node_list_has "${flatscan_publisher_nodes}" '(^|[[:space:]])pointcloud_accel_axis_node($|[[:space:]])'; then
    flatscan_owner_ok=true
    echo "[pointcloud-accel-verify] PASS ${RESOLVED_PROFILE} flatscan owner is pointcloud_accel_axis_node direct publisher"
  elif [[ "${flatscan_publishers}" == "0" ]]; then
    fail "${RESOLVED_PROFILE} /flatscan compatibility publisher is missing"
    mark_nav2_fail
  else
    fail "${RESOLVED_PROFILE} /flatscan publisher owner is unexpected: ${flatscan_owner:-missing}"
    mark_nav2_fail
  fi
fi

hesai_driver_running=false
hesai_accel_driver_running=false
pointcloud_accel_axis_running=false
if pgrep -f "[h]esai_ros_driver_node" >/dev/null 2>&1; then
  hesai_driver_running=true
fi
if pgrep -f "[h]esai_accel_driver_node|[j]t128_accel_driver_node" >/dev/null 2>&1; then
  hesai_accel_driver_running=true
fi
if pgrep -f "[p]ointcloud_accel_axis_node" >/dev/null 2>&1; then
  pointcloud_accel_axis_running=true
fi
lidar_driver_owner="missing"
if [[ "${hesai_accel_driver_running}" == "true" ]]; then
  lidar_driver_owner="hesai_accel_driver_node"
elif [[ "${hesai_driver_running}" == "true" ]]; then
  lidar_driver_owner="hesai_ros_driver_node"
fi
echo "[pointcloud-accel-verify] lidar_driver_owner=${lidar_driver_owner}"
echo "[pointcloud-accel-verify] hesai_ros_driver_node_running=${hesai_driver_running}"
echo "[pointcloud-accel-verify] hesai_accel_driver_node_running=${hesai_accel_driver_running}"
echo "[pointcloud-accel-verify] pointcloud_accel_axis_node_running=${pointcloud_accel_axis_running}"
if [[ "${RESOLVED_INGRESS_PROFILE}" == "separate_process" ]]; then
  [[ "${hesai_driver_running}" == "true" ]] || { fail "separate_process requires hesai_ros_driver_node"; mark_profile_fail; }
  if [[ "${RESOLVED_PROFILE}" != "legacy" ]]; then
    [[ "${pointcloud_accel_axis_running}" == "true" ]] || { fail "separate_process accel profile requires pointcloud_accel_axis_node"; mark_profile_fail; mark_ipc_fail; }
    [[ "${vendor_raw_publishers}" == "1" ]] || { fail "separate_process requires /jt128/vendor/points_raw publisher=1"; mark_profile_fail; }
    [[ "${vendor_raw_subscribers}" != "0" ]] || { fail "separate_process requires /jt128/vendor/points_raw subscriber from accel node"; mark_profile_fail; }
  fi
else
  [[ "${hesai_accel_driver_running}" == "true" ]] || { fail "driver_integrated requires hesai_accel_driver_node running"; mark_profile_fail; mark_ipc_fail; }
  [[ "${hesai_driver_running}" == "false" ]] || { fail "driver_integrated must not run standalone hesai_ros_driver_node"; mark_profile_fail; mark_ipc_fail; }
  [[ "${pointcloud_accel_axis_running}" == "false" ]] || { fail "driver_integrated must not run standalone pointcloud_accel_axis_node"; mark_profile_fail; mark_ipc_fail; }
  [[ "${vendor_raw_subscribers}" == "0" ]] || { fail "driver_integrated must not require /jt128/vendor/points_raw subscribers"; mark_profile_fail; mark_ipc_fail; }
fi

if [[ "${flatscan_owner_ok}" == "true" ]]; then
  echo "[pointcloud-accel-verify] PASS FLATSCAN_OWNER_OK owner=${flatscan_owner}"
else
  echo "[pointcloud-accel-verify] FAIL FLATSCAN_OWNER_OK=false owner=${flatscan_owner:-missing}"
  status=1
fi
if [[ "${flatscan_owner_ok}" == "true" && "${flatscan_hz_ok}" == "true" ]]; then
  flatscan_nav_startup_gate_ok=true
  echo "[pointcloud-accel-verify] PASS FLATSCAN_NAV_STARTUP_GATE_OK"
else
  echo "[pointcloud-accel-verify] FAIL FLATSCAN_NAV_STARTUP_GATE_OK=false"
  status=1
fi

fastlio_residual=false
if pgrep -f "[f]ast[_]lio|[f]ast[l]io|[l]aser[_]mapping" >/dev/null 2>&1; then
  fastlio_residual=true
fi
echo "[pointcloud-accel-verify] FAST-LIO2 residual=${fastlio_residual}"
if [[ "${fastlio_residual}" == "true" ]]; then
  fail "FAST-LIO2 navigation residue detected"
  mark_nav2_fail
else
  echo "[pointcloud-accel-verify] PASS no FAST-LIO2 navigation residue"
fi

controller_state="$(timeout 5 ros2 lifecycle get /controller_server 2>/dev/null || true)"
bt_state="$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
if grep -qi "active" <<<"${controller_state}"; then
  echo "[pointcloud-accel-verify] PASS controller_server active: ${controller_state}"
else
  fail "controller_server is not active: ${controller_state:-unavailable}"
  mark_nav2_fail
fi
if [[ -n "${bt_state}" ]]; then
  echo "[pointcloud-accel-verify] Nav2 lifecycle: ${bt_state}"
else
  warn "bt_navigator lifecycle unavailable"
fi

if grep -q "/local_costmap" "${TMP_DIR}/obstacle.info"; then
  echo "[pointcloud-accel-verify] PASS local_costmap subscribes /perception/obstacle_points"
else
  fail "local_costmap subscriber not observed on /perception/obstacle_points"
  mark_nav2_fail
fi
if grep -q "collision_monitor" "${TMP_DIR}/obstacle.info"; then
  echo "[pointcloud-accel-verify] PASS collision_monitor subscribes /perception/obstacle_points"
else
  fail "collision_monitor subscriber not observed on /perception/obstacle_points"
  mark_nav2_fail
fi

if [[ "${RESOLVED_PROFILE}" == "legacy" ]]; then
  echo "[pointcloud-accel-verify] production_hop_internal_local_branch=true"
  echo "[pointcloud-accel-verify] production_hop_points_nav=true"
else
  echo "[pointcloud-accel-verify] production_hop_internal_local_branch=false"
  echo "[pointcloud-accel-verify] production_hop_points_nav=false"
fi

echo "[pointcloud-accel-verify] PointCloud2 QoS snapshot:"
grep -E "Reliability|Depth|Publisher count|Subscription count" "${TMP_DIR}/lidar.info" || true
echo "[pointcloud-accel-verify] /jt128/vendor/points_raw QoS snapshot:"
grep -E "Reliability|Depth|Publisher count|Subscription count" "${TMP_DIR}/vendor_raw.info" || true
echo "[pointcloud-accel-verify] socket drop snapshot:"
ss -u -n -a -i 2>/dev/null | grep -E "2368|7400|7417|7463|rcvbuf|drops" || true
awk 'NR<=5 || /:09[4-9][0-9]|:1C[Ff][0-9]|:1D[0-9A-Fa-f][0-9A-Fa-f]/ {print}' /proc/net/udp 2>/dev/null || true
echo "[pointcloud-accel-verify] CPU per-core snapshot:"
top -b -n1 | awk '/^%Cpu/ || /^Cpu/ {print}' || true

echo "[pointcloud-accel-verify] PROFILE_OWNER_CONTRACT_OK=${profile_owner_contract_ok}"
echo "[pointcloud-accel-verify] INGRESS_PROFILE=${RESOLVED_INGRESS_PROFILE}"
echo "[pointcloud-accel-verify] LEGACY_SCAN_CHAIN_OK=${legacy_scan_chain_ok}"
echo "[pointcloud-accel-verify] IPC_WORKER_OWNER_OK=${ipc_worker_owner_ok}"
echo "[pointcloud-accel-verify] TRUNK_FULL_DENSITY_OK=${trunk_full_density_ok}"
echo "[pointcloud-accel-verify] NAV2_COMPAT_TOPICS_OK=${nav2_compat_topics_ok}"
echo "[pointcloud-accel-verify] FLATSCAN_OWNER_OK=${flatscan_owner_ok}"
echo "[pointcloud-accel-verify] FLATSCAN_HZ_OK=${flatscan_hz_ok}"
echo "[pointcloud-accel-verify] FLATSCAN_NAV_STARTUP_GATE_OK=${flatscan_nav_startup_gate_ok}"

if [[ "${status}" -eq 0 ]]; then
  echo "[pointcloud-accel-verify] PASS"
else
  echo "[pointcloud-accel-verify] FAIL"
fi
exit "${status}"

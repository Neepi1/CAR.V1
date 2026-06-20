#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/map_server_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile

export PUBLISH_LIDAR_TF="${PUBLISH_LIDAR_TF:-false}"
if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
  LAUNCH_FILE="${NJRH_OVERLAY_ROOT}/launch/occupancy_localization_stack.launch.py"
else
  LAUNCH_FILE="${NJRH_OVERLAY_ROOT}/launch/occupancy_localization.launch.py"
fi
NAV_LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}"
POINTS_TOPIC="${NJRH_LOCALIZATION_POINTS_TOPIC:-/lidar_points_nav}"
FASTLIO_POINTS_TOPIC="${NJRH_FASTLIO_POINTS_TOPIC:-/cloud_registered_body}"
FASTLIO_ODOM_TOPIC="${NJRH_FASTLIO_ODOM_TOPIC:-/Odometry}"
FASTLIO_POINTS_READY_TIMEOUT="${NJRH_FASTLIO_POINTS_READY_TIMEOUT:-30}"
FASTLIO_ODOM_READY_TIMEOUT="${NJRH_FASTLIO_ODOM_READY_TIMEOUT:-30}"
FASTLIO_TOPIC_FRESH_TIMEOUT="${NJRH_FASTLIO_TOPIC_FRESH_TIMEOUT:-8}"
FASTLIO_TOPIC_MAX_AGE_SEC="${NJRH_FASTLIO_TOPIC_MAX_AGE_SEC:-1.0}"
FASTLIO_TOPIC_MAX_FUTURE_SEC="${NJRH_FASTLIO_TOPIC_MAX_FUTURE_SEC:-0.25}"
LOCALIZATION_POINTS_INITIAL_WAIT_SEC="${NJRH_LOCALIZATION_POINTS_INITIAL_WAIT_SEC:-20}"
LOCALIZATION_POINTS_RETRY_WAIT_SEC="${NJRH_LOCALIZATION_POINTS_RETRY_WAIT_SEC:-20}"
LOCALIZATION_POINTS_REPAIR_WAIT_SEC="${NJRH_LOCALIZATION_POINTS_REPAIR_WAIT_SEC:-60}"
LOCALIZATION_POINTS_DRIVER_REPAIR="${NJRH_LOCALIZATION_POINTS_DRIVER_REPAIR:-true}"
LOCALIZATION_FLATSCAN_READY_TIMEOUT="${NJRH_LOCALIZATION_FLATSCAN_READY_TIMEOUT_SEC:-75}"
MAP_SERVER_READY_TIMEOUT="${NJRH_LOCALIZATION_MAP_SERVER_READY_TIMEOUT:-75}"
LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP="${NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP:-true}"
LOCALIZATION_MAP_LIFECYCLE_BRINGUP_TIMEOUT_SEC="${NJRH_LOCALIZATION_MAP_LIFECYCLE_BRINGUP_TIMEOUT_SEC:-90}"
LOCALIZATION_REUSE_READY_STACK="${NJRH_LOCALIZATION_REUSE_READY_STACK:-false}"
ISAAC_LOCALIZATION_MODE="${NJRH_ISAAC_LOCALIZATION_MODE:-triggered}"
case "${ISAAC_LOCALIZATION_MODE}" in
  triggered)
    ;;
  *)
    echo "[runtime-overlay] invalid NJRH_ISAAC_LOCALIZATION_MODE=${ISAAC_LOCALIZATION_MODE}; expected triggered. Isaac continuous localization has been removed; use NJRH_AMCL_LOCALIZATION_MODE=shadow|gated for continuous correction candidates." >&2
    exit 2
    ;;
esac
LOCALIZER_FLATSCAN_TOPIC="${NJRH_ISAAC_LOCALIZER_FLATSCAN_TOPIC:-/flatscan}"
export NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP="${LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP}"

if [[ -n "${NJRH_FLOOR_ID:-}" || -n "${NAV2_FLOOR_ID:-}" ]]; then
  resolve_floor_assets "${NJRH_BUILDING_ID:-${NAV2_BUILDING_ID:-building_1}}" "${NJRH_FLOOR_ID:-${NAV2_FLOOR_ID:-}}"
fi

NAV2_MAP_YAML="${NAV2_MAP_YAML:-}"
NAV2_LOCALIZER_PARAMS="${NAV2_LOCALIZER_PARAMS:-}"
NAV2_LOCALIZER_MAP_YAML="${NAV2_LOCALIZER_MAP_YAML:-}"
LOCALIZER_MAP_PREPARE_SCRIPT="${NJRH_OVERLAY_ROOT}/scripts/prepare_localizer_map.py"
NITROS_TMP_DIR="${ISAAC_ROS_NITROS_TMP_DIR:-/tmp/isaac_ros_nitros}"

[[ -n "${NAV2_MAP_YAML}" ]] || {
  echo "NAV2_MAP_YAML is not set. Please choose a saved 2D map before starting occupancy localization." >&2
  exit 1
}
[[ -f "${NAV2_MAP_YAML}" ]] || {
  echo "Nav2 map yaml does not exist: ${NAV2_MAP_YAML}" >&2
  exit 1
}
[[ -f "${LAUNCH_FILE}" ]] || {
  echo "[runtime-overlay] launch file missing: ${LAUNCH_FILE}" >&2
  exit 1
}
[[ -f "${LOCALIZER_MAP_PREPARE_SCRIPT}" ]] || {
  echo "[runtime-overlay] localizer map prepare script missing: ${LOCALIZER_MAP_PREPARE_SCRIPT}" >&2
  exit 1
}

require_can_interface_up

mkdir -p "${NITROS_TMP_DIR}/graphs" 2>/dev/null || {
  echo "[runtime-overlay] Isaac NITROS tmp is not writable: ${NITROS_TMP_DIR}" >&2
  echo "[runtime-overlay] restart with scripts/jetson/njrh_container.sh start-runtime to repair root-owned 1777 permissions." >&2
  exit 1
}
if [[ ! -w "${NITROS_TMP_DIR}" || ! -w "${NITROS_TMP_DIR}/graphs" ]]; then
  echo "[runtime-overlay] Isaac NITROS graph directory is not writable: ${NITROS_TMP_DIR}/graphs" >&2
  echo "[runtime-overlay] expected root:root owner with 1777 mode so the admin runtime can create graph files." >&2
  exit 1
fi

if [[ -n "${NAV2_LOCALIZER_MAP_YAML}" ]]; then
  LOCALIZER_MAP_YAML="${NAV2_LOCALIZER_MAP_YAML}"
else
  LOCALIZER_MAP_YAML="$(python3 "${LOCALIZER_MAP_PREPARE_SCRIPT}" --nav-yaml "${NAV2_MAP_YAML}")"
fi
[[ -f "${LOCALIZER_MAP_YAML}" ]] || {
  echo "[runtime-overlay] prepared localizer yaml does not exist: ${LOCALIZER_MAP_YAML}" >&2
  exit 1
}

localization_stack_ready_for_current_floor() {
  pgrep -f "occupancy_localization_stack.launch.py|occupancy_grid_localizer_container|occupancy_grid_localizer" >/dev/null 2>&1
}

if [[ "${LOCALIZATION_REUSE_READY_STACK}" == "true" ]] && localization_stack_ready_for_current_floor; then
  echo "[runtime-overlay] occupancy localization stack already ready for ${NAV2_MAP_YAML}; reusing existing stack" >&2
  while true; do
    sleep 3600
  done
fi

patterns=(
  "ros2 launch .*occupancy_localization_stack.launch.py"
  "occupancy_localization_stack.launch.py"
  "occupancy_localization.launch.py"
  "ros2 launch .*jt128_occupancy_localization_stack.launch.py"
  "jt128_occupancy_localization_stack.launch.py"
  "jt128_occupancy_localization.launch.py"
  "occupancy_grid_localizer_container"
  "occupancy_grid_localizer"
  # Stale cleanup only: these names existed before Phase A2 removed the Isaac
  # continuous-localization path from runtime startup.
  "continuous_flatscan_forwarder.py"
  "isaac_continuous_flatscan_forwarder"
  "map_to_odom_tf_bridge"
  "map_server"
  "lifecycle_manager_map"
  "/opt/ros/humble/lib/nav2_util/lifecycle_bringup map_server"
)
if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
  patterns+=(
    "jt128_nav_sensing.launch.py"
    "pointcloud_to_laserscan_node"
    "pointcloud_to_laserscan"
    "robot_hesai_jt128/scan_republisher_node"
    "scan_republisher_node"
    "nav_cloud_preprocessor"
  )
else
  patterns+=(
    "jt128_nav_sensing.launch.py"
    "pointcloud_to_laserscan_node"
    "pointcloud_to_laserscan"
    "robot_hesai_jt128/scan_republisher_node"
    "scan_republisher_node"
    "nav_cloud_preprocessor"
  )
fi

for pattern in "${patterns[@]}"; do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in "${patterns[@]}"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

kill_localization_stack_patterns() {
  local signal="$1"
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill "-${signal}" -f "$pattern" 2>/dev/null || true
  done
}

cleanup_localization_stack_patterns() {
  kill_localization_stack_patterns INT
  sleep "${NJRH_LOCALIZATION_PATTERN_STOP_INT_WAIT_SEC:-1}"
  kill_localization_stack_patterns TERM
  sleep "${NJRH_LOCALIZATION_PATTERN_STOP_TERM_WAIT_SEC:-1}"
  kill_localization_stack_patterns KILL
}

stop_existing_canonical_tf_publishers

localization_pid=""
map_lifecycle_bringup_pid=""
localization_exit_code=0
launch_args=(
  "map_yaml:=${NAV2_MAP_YAML}"
  "localizer_map_yaml:=${LOCALIZER_MAP_YAML}"
  "use_sim_time:=false"
)
if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
  launch_args+=(
    "publish_lidar_tf:=${PUBLISH_LIDAR_TF}"
    "points_topic:=${POINTS_TOPIC}"
  )
else
  launch_args+=(
    "start_map_server:=true"
    "map_frame:=map"
    "flatscan_topic:=${LOCALIZER_FLATSCAN_TOPIC}"
  )
  echo "[runtime-overlay] pointcloud accel profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}; occupancy localization reuses /scan and /flatscan from pointcloud_accel_pipeline instead of launching /points_nav legacy sensing; localizer_flatscan_topic=${LOCALIZER_FLATSCAN_TOPIC}" >&2
  if [[ "${LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP}" == "true" ]]; then
    launch_args+=("map_lifecycle_manager_enabled:=false")
    echo "[runtime-overlay] localization map_server lifecycle manager disabled; external lifecycle_bringup will activate map_server" >&2
  else
    launch_args+=("map_lifecycle_manager_enabled:=true")
  fi
fi

wait_for_child_exit() {
  local pid="$1"
  local attempts="${2:-20}"
  local i
  for ((i = 0; i < attempts; i += 1)); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

terminate_child() {
  local pid="$1"
  local label="$2"
  [[ -n "${pid}" ]] || return 0
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi
  echo "[runtime-overlay] stopping ${label} pid=${pid}" >&2
  kill -INT "${pid}" 2>/dev/null || true
  wait_for_child_exit "${pid}" "${NJRH_LOCALIZATION_STOP_INT_ATTEMPTS:-20}" || {
    kill -TERM "${pid}" 2>/dev/null || true
    wait_for_child_exit "${pid}" "${NJRH_LOCALIZATION_STOP_TERM_ATTEMPTS:-20}" || {
      kill -KILL "${pid}" 2>/dev/null || true
    }
  }
  wait "${pid}" 2>/dev/null || true
}

localization_map_external_lifecycle_bringup_enabled() {
  [[ "${LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP}" == "true" ]]
}

start_map_server_lifecycle_with_nav2_util() {
  localization_map_external_lifecycle_bringup_enabled || return 0
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || {
    echo "[runtime-overlay] localization map_server external lifecycle bringup skipped for legacy localization profile" >&2
    return 0
  }

  local node_timeout="${NJRH_LOCALIZATION_MAP_LIFECYCLE_NODE_TIMEOUT_SEC:-30}"
  local sequence_args=(--per-node-timeout-sec "${node_timeout}")
  if [[ "${NJRH_LOCALIZATION_MAP_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE:-true}" == "true" ]]; then
    sequence_args+=(--trust-change-state-response)
  fi

  echo "[runtime-overlay] starting localization map_server lifecycle with repo lifecycle sequence timeout=${LOCALIZATION_MAP_LIFECYCLE_BRINGUP_TIMEOUT_SEC}s node_timeout=${node_timeout}s" >&2
  timeout --kill-after="${NJRH_LOCALIZATION_MAP_LIFECYCLE_BRINGUP_KILL_AFTER_SEC:-5}" \
    "${LOCALIZATION_MAP_LIFECYCLE_BRINGUP_TIMEOUT_SEC}" \
    python3 "${SCRIPT_DIR}/nav2_lifecycle_sequence.py" \
      "${sequence_args[@]}" \
      map_server &
  map_lifecycle_bringup_pid=$!
  if wait "${map_lifecycle_bringup_pid}"; then
    map_lifecycle_bringup_pid=""
    echo "[runtime-overlay] localization map_server repo lifecycle sequence: map_server active" >&2
    return 0
  fi
  map_lifecycle_bringup_pid=""
  echo "[runtime-overlay] localization map_server repo lifecycle sequence failed or timed out" >&2
  return 1
}

repair_jt128_navigation_points() {
  if [[ "${LOCALIZATION_POINTS_DRIVER_REPAIR}" != "true" ]]; then
    return 1
  fi
  if [[ "${POINTS_TOPIC}" != "/lidar_points_nav" && "${POINTS_TOPIC}" != "/lidar_points" ]]; then
    return 1
  fi

  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  echo "[runtime-overlay] attempting JT128 driver/remap repair for localization pointcloud: ${POINTS_TOPIC}" >&2
  nohup env \
    DRIVER_PROFILE="${NJRH_LOCALIZATION_DRIVER_PROFILE:-navigation}" \
    NJRH_FORCE_RESTART_DRIVER="${NJRH_LOCALIZATION_REPAIR_FORCE_RESTART_DRIVER:-true}" \
    bash "${SCRIPT_DIR}/run_driver.sh" \
    >>"${NJRH_RUNTIME_LOG_DIR}/driver_repair_for_localization.log" 2>&1 &
  sleep "${NJRH_LOCALIZATION_DRIVER_REPAIR_SETTLE_SEC:-3}"
}

ensure_localization_pointcloud_ready() {
  echo "[runtime-overlay] localization pointcloud startup probe disabled: ${POINTS_TOPIC}" >&2
  return 0
}

pointcloud_accel_pipeline_runtime_running() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  pgrep -f "run_pointcloud_accel_pipeline.sh|pointcloud_accel_axis_node|laser_scan_to_flatscan" >/dev/null 2>&1
}

flatscan_publisher_ready_for_localization() {
  wait_for_topic_publisher_from_node "${LOCALIZER_FLATSCAN_TOPIC}" "laser_scan_to_flatscan" "$1"
}

ensure_pointcloud_accel_pipeline_for_localization() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  if pointcloud_accel_pipeline_runtime_running; then
    if flatscan_publisher_ready_for_localization "${LOCALIZATION_FLATSCAN_READY_TIMEOUT}"; then
      echo "[runtime-overlay] pointcloud accel pipeline already running with ${LOCALIZER_FLATSCAN_TOPIC} publisher for localization profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}" >&2
      return 0
    fi
    echo "[runtime-overlay] pointcloud accel pipeline is running but ${LOCALIZER_FLATSCAN_TOPIC} publisher is missing; restarting profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}" >&2
    bash "${SCRIPT_DIR}/set_pointcloud_accel_profile.sh" --profile "${NJRH_POINTCLOUD_ACCEL_PROFILE}" --restart >&2
    flatscan_publisher_ready_for_localization "${LOCALIZATION_FLATSCAN_READY_TIMEOUT}"
    return $?
  fi
  echo "[runtime-overlay] starting pointcloud accel pipeline for localization profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}" >&2
  start_overlay_helper "pointcloud_accel_pipeline_localization" env \
    NJRH_FORCE_RESTART_DRIVER="${NJRH_POINTCLOUD_ACCEL_FORCE_RESTART_DRIVER:-false}" \
    bash "${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh"
  flatscan_publisher_ready_for_localization "${LOCALIZATION_FLATSCAN_READY_TIMEOUT}"
}

if [[ -n "${NAV2_LOCALIZER_PARAMS}" ]]; then
  launch_args+=("localizer_params:=${NAV2_LOCALIZER_PARAMS}")
fi

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${map_lifecycle_bringup_pid}" ]]; then
    kill -INT "${map_lifecycle_bringup_pid}" 2>/dev/null || true
    sleep 0.5
    kill -TERM "${map_lifecycle_bringup_pid}" 2>/dev/null || true
    sleep 0.5
    kill -KILL "${map_lifecycle_bringup_pid}" 2>/dev/null || true
    wait "${map_lifecycle_bringup_pid}" 2>/dev/null || true
    map_lifecycle_bringup_pid=""
  fi
  terminate_child "${localization_pid}" "occupancy localization launch"
  cleanup_localization_stack_patterns
  cleanup_overlay_helpers
  # Canonical TF/local-state is a common-service dependency, not owned by the
  # occupancy localization mode. Killing it here can leave Nav2 active with no
  # /local_state/odometry or odom->base_link publisher during stop/restart.
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

resident_fastlio_runtime_running() {
  pgrep -f "fast_lio[[:space:]]+fastlio_mapping|fastlio_mapping --ros-args|laser_mapping" >/dev/null 2>&1
}

ensure_resident_fastlio_for_local_state() {
  resident_fastlio_runtime_running || {
    echo "[runtime-overlay] managed FAST-LIO runtime is not running; explicit fastlio local-state mode must start it before occupancy localization" >&2
    return 1
  }
  echo "[runtime-overlay] managed FAST-LIO process exists for explicit fastlio local-state mode; startup topic probes are disabled" >&2
}

ensure_resident_local_state_for_localization() {
  LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" local_state_required_processes_running || {
    echo "[runtime-overlay] resident local_state ${NAV_LOCAL_STATE_MODE} process is not running; common services must start it before occupancy localization" >&2
    return 1
  }
  echo "[runtime-overlay] resident local_state ${NAV_LOCAL_STATE_MODE} process exists for occupancy localization; startup odom/TF probes are disabled" >&2
}

start_canonical_helper "ranger_chassis_localization" bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
start_canonical_helper "robot_description_static_tf_localization" bash "${SCRIPT_DIR}/run_robot_description.sh"
ensure_pointcloud_accel_pipeline_for_localization
ensure_localization_pointcloud_ready
if [[ "${NAV_LOCAL_STATE_MODE}" == "fastlio" ]]; then
  ensure_resident_fastlio_for_local_state || exit 1
fi
ensure_resident_local_state_for_localization || exit 1
start_canonical_helper "robot_localization_bridge" bash "${SCRIPT_DIR}/run_localization_bridge.sh"
start_overlay_helper "global_localization_localization" bash "${SCRIPT_DIR}/run_global_localization.sh"
echo "[runtime-overlay] Isaac localization mode=triggered; AMCL owns continuous localization candidates when NJRH_AMCL_LOCALIZATION_MODE=shadow|gated" >&2

ros2 launch "${LAUNCH_FILE}" "${launch_args[@]}" &
localization_pid=$!

start_map_server_lifecycle_with_nav2_util || exit 1

wait "${localization_pid}" || localization_exit_code=$?
exit "${localization_exit_code}"

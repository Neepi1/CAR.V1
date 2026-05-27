#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/map_server_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"

export PUBLISH_LIDAR_TF="${PUBLISH_LIDAR_TF:-false}"
LAUNCH_FILE="${NJRH_OVERLAY_ROOT}/launch/occupancy_localization_stack.launch.py"
NAV_LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-passthrough}"
POINTS_TOPIC="${NJRH_LOCALIZATION_POINTS_TOPIC:-/lidar_points}"

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

patterns=(
  "laser_mapping"
  "fastlio_mapping"
  "ros2 launch .*occupancy_localization_stack.launch.py"
  "occupancy_localization_stack.launch.py"
  "occupancy_localization.launch.py"
  "ros2 launch .*jt128_occupancy_localization_stack.launch.py"
  "jt128_occupancy_localization_stack.launch.py"
  "jt128_occupancy_localization.launch.py"
  "jt128_nav_sensing.launch.py"
  "occupancy_grid_localizer_container"
  "occupancy_grid_localizer"
  "map_to_odom_tf_bridge"
  "laser_scan_to_flatscan"
  "pointcloud_to_laserscan_node"
  "pointcloud_to_laserscan"
  "robot_hesai_jt128/scan_republisher_node"
  "scan_republisher_node"
  "nav_cloud_preprocessor"
  "map_server"
  "lifecycle_manager_map"
)

for pattern in "${patterns[@]}"; do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in "${patterns[@]}"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

stop_existing_canonical_tf_publishers

localization_pid=""
localization_exit_code=0
launch_args=(
  "map_yaml:=${NAV2_MAP_YAML}"
  "localizer_map_yaml:=${LOCALIZER_MAP_YAML}"
  "use_sim_time:=false"
  "publish_lidar_tf:=${PUBLISH_LIDAR_TF}"
  "points_topic:=${POINTS_TOPIC}"
)

if [[ -n "${NAV2_LOCALIZER_PARAMS}" ]]; then
  launch_args+=("localizer_params:=${NAV2_LOCALIZER_PARAMS}")
fi

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${localization_pid}" ]]; then
    kill -INT "${localization_pid}" 2>/dev/null || true
    wait "${localization_pid}" 2>/dev/null || true
  fi
  cleanup_overlay_helpers
  cleanup_canonical_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

start_canonical_helper "ranger_chassis_localization" bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
start_canonical_helper "robot_description_static_tf_localization" bash "${SCRIPT_DIR}/run_robot_description.sh"
if [[ "${NAV_LOCAL_STATE_MODE}" == "passthrough" || "${NAV_LOCAL_STATE_MODE}" == "legacy" ]]; then
  kill_canonical_pattern "robot_localization/ekf_node"
  kill_canonical_pattern "ekf_node --ros-args.*__node:=robot_local_state"
fi
start_canonical_helper \
  "robot_local_state_localization" \
  env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"
start_canonical_helper "robot_localization_bridge" bash "${SCRIPT_DIR}/run_localization_bridge.sh"
start_overlay_helper "global_localization_localization" bash "${SCRIPT_DIR}/run_global_localization.sh"

wait_for_tf_edge "base_link" "lidar_level_link" 10 || {
  echo "[runtime-overlay] base_link -> lidar_level_link TF did not become ready for occupancy localization" >&2
  exit 1
}

wait_for_topic_message "/local_state/odometry" 12 || {
  echo "[runtime-overlay] /local_state/odometry did not become ready for occupancy localization" >&2
  exit 1
}

wait_for_topic_message "${POINTS_TOPIC}" 20 || {
  echo "[runtime-overlay] timed out waiting for localization pointcloud: ${POINTS_TOPIC}" >&2
  echo "[runtime-overlay] check canonical /lidar_points input stream and JT128 driver/remap chain." >&2
  exit 1
}

ros2 launch "${LAUNCH_FILE}" "${launch_args[@]}" &
localization_pid=$!

ensure_map_server_active "${NAV2_MAP_YAML}" 30 || {
  echo "[runtime-overlay] map_server did not become active for occupancy localization" >&2
  exit 1
}

wait_for_occupancy_grid "/map" 30 || {
  echo "[runtime-overlay] /map did not become available for occupancy localization" >&2
  exit 1
}

wait "${localization_pid}" || localization_exit_code=$?
exit "${localization_exit_code}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"

UPSTREAM_SLAM_SCRIPT="${NJRH_UPSTREAM_ROOT}/scripts/run_jt128_2d_mapping.sh"
USE_UPSTREAM_SLAM_SCRIPT="${NJRH_SLAM2D_USE_UPSTREAM_SCRIPT:-false}"
SLAM_LAUNCH_FILE="${NJRH_SLAM2D_LAUNCH_FILE:-${NJRH_OVERLAY_ROOT}/launch/jt128_slam_toolbox_mapping.launch.py}"
SLAM_PARAMS_FILE="${NJRH_SLAM2D_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_slam_toolbox_mapping.yaml}"
SCAN_PARAMS_FILE="${NJRH_SLAM2D_SCAN_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_scan_slam2d.yaml}"
FASTLIO_CONFIG_FILE="${NJRH_SLAM2D_FASTLIO_CONFIG:-${NJRH_OVERLAY_ROOT}/config/fastlio.yaml}"
POINTS_TOPIC="${NJRH_SLAM2D_POINTS_TOPIC:-/cloud_registered_body}"
SLAM2D_LOCAL_STATE_MODE="${NJRH_SLAM2D_LOCAL_STATE_MODE:-passthrough}"

for pattern in \
  "run_jt128_2d_mapping.sh" \
  "projected_occupancy_mapper.py" \
  "occupancy_builder_live_node.py" \
  "robot_occupancy_builder_live" \
  "frontend_pose_from_odometry.py" \
  "slam_toolbox" \
  "jt128_2d_mapping.launch.py" \
  "jt128_slam_toolbox_mapping.launch.py" \
  "nav_cloud_preprocessor" \
  "pointcloud_to_laserscan_node" \
  "robot_hesai_jt128/scan_republisher_node" \
  "scan_republisher_node"
do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in \
  "run_jt128_2d_mapping.sh" \
  "projected_occupancy_mapper.py" \
  "occupancy_builder_live_node.py" \
  "robot_occupancy_builder_live" \
  "frontend_pose_from_odometry.py" \
  "slam_toolbox" \
  "jt128_2d_mapping.launch.py" \
  "jt128_slam_toolbox_mapping.launch.py" \
  "nav_cloud_preprocessor" \
  "pointcloud_to_laserscan_node" \
  "robot_hesai_jt128/scan_republisher_node" \
  "scan_republisher_node"
do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

require_can_interface_up
stop_existing_canonical_tf_publishers

projected_map_pid=""
fastlio_deskew_pid=""
fastlio_reused_for_slam2d="false"
projected_map_exit_code=0

stop_fastlio_deskew_sources() {
  for pattern in "laser_mapping" "fastlio_mapping" "ros2 run fast_lio fastlio_mapping" "run_fastlio_tf.sh"; do
    pkill -INT -f "$pattern" 2>/dev/null || true
  done
  sleep 1
  for pattern in "laser_mapping" "fastlio_mapping" "ros2 run fast_lio fastlio_mapping" "run_fastlio_tf.sh"; do
    pkill -9 -f "$pattern" 2>/dev/null || true
  done
}

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${projected_map_pid}" ]]; then
    kill -INT "${projected_map_pid}" 2>/dev/null || true
    wait "${projected_map_pid}" 2>/dev/null || true
  fi
  if [[ -n "${fastlio_deskew_pid}" ]]; then
    kill -INT "${fastlio_deskew_pid}" 2>/dev/null || true
    wait "${fastlio_deskew_pid}" 2>/dev/null || true
  fi
  if [[ "${fastlio_reused_for_slam2d}" == "true" ]]; then
    stop_fastlio_deskew_sources
  fi
  cleanup_canonical_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

start_canonical_helper "ranger_chassis_projected_map" bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
start_canonical_helper "robot_description_static_tf_projected_map" bash "${SCRIPT_DIR}/run_robot_description.sh"
if [[ "${SLAM2D_LOCAL_STATE_MODE}" == "passthrough" || "${SLAM2D_LOCAL_STATE_MODE}" == "legacy" ]]; then
  # Field 2D mapping diagnostics use wheel odom directly. If common services
  # already started the EKF owner, stop only local-state publishers so the
  # passthrough node can remain the unique odom->base_link source for mapping.
  kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_local_state.sh"
  kill_canonical_pattern "robot_localization/ekf_node"
  kill_canonical_pattern "ekf_node --ros-args.*__node:=robot_local_state"
  kill_canonical_pattern "robot_local_state/local_state_node"
  kill_canonical_pattern "local_state_node --ros-args"
fi
start_canonical_helper \
  "robot_local_state_projected_map" \
  env LOCAL_STATE_MODE="${SLAM2D_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"

wait_for_tf_edge "base_link" "lidar_level_link" 10 || {
  echo "[runtime-overlay] base_link -> lidar_level_link TF did not become ready for slam_toolbox mapping" >&2
  exit 1
}

wait_for_topic_message "/local_state/odometry" 12 || {
  echo "[runtime-overlay] /local_state/odometry did not become ready for slam_toolbox mapping" >&2
  exit 1
}

if [[ "${USE_UPSTREAM_SLAM_SCRIPT}" == "true" && -f "${UPSTREAM_SLAM_SCRIPT}" ]]; then
  bash -lc "PUBLISH_LIDAR_TF=false bash '${UPSTREAM_SLAM_SCRIPT}'" &
  projected_map_pid=$!
  wait "${projected_map_pid}" || projected_map_exit_code=$?
  exit "${projected_map_exit_code}"
fi

for required_file in "${SLAM_LAUNCH_FILE}" "${SLAM_PARAMS_FILE}" "${SCAN_PARAMS_FILE}" "${FASTLIO_CONFIG_FILE}"; do
  [[ -f "${required_file}" ]] || {
    echo "[runtime-overlay] missing slam_toolbox runtime file: ${required_file}" >&2
    exit 1
  }
done

fastlio_log="${NJRH_RUNTIME_LOG_DIR}/fastlio_slam2d_deskew.log"
if wait_for_topic_message "${POINTS_TOPIC}" 2; then
  fastlio_reused_for_slam2d="true"
  echo "[runtime-overlay] reusing existing FAST-LIO2 deskew source: ${POINTS_TOPIC}" >&2
else
  stop_fastlio_deskew_sources
  : >"${fastlio_log}" 2>/dev/null || {
    echo "[runtime-overlay] FAST-LIO log is not writable: ${fastlio_log}" >&2
    exit 1
  }
  echo "[runtime-overlay] starting FAST-LIO2 deskew source for slam_toolbox: ${POINTS_TOPIC}" >&2
  ros2 run fast_lio fastlio_mapping \
    --ros-args \
    --params-file "${FASTLIO_CONFIG_FILE}" \
    -p use_sim_time:=false \
    -r /tf:=/tf_fastlio_internal \
    -r /tf_static:=/tf_static_fastlio_internal >>"${fastlio_log}" 2>&1 &
  fastlio_deskew_pid=$!
  sleep 2
  if ! kill -0 "${fastlio_deskew_pid}" 2>/dev/null; then
    echo "[runtime-overlay] FAST-LIO2 deskew source failed to stay alive. Check ${fastlio_log}" >&2
    exit 1
  fi
fi

wait_for_topic_message "${POINTS_TOPIC}" 20 || {
  echo "[runtime-overlay] timed out waiting for FAST-LIO2 deskewed pointcloud: ${POINTS_TOPIC}" >&2
  echo "[runtime-overlay] check ${fastlio_log} and canonical /lidar_points + /lidar_imu input streams." >&2
  exit 1
}

ros2 launch "${SLAM_LAUNCH_FILE}" \
  slam_params:="${SLAM_PARAMS_FILE}" \
  scan_params:="${SCAN_PARAMS_FILE}" \
  points_topic:="${POINTS_TOPIC}" &
projected_map_pid=$!
wait "${projected_map_pid}" || projected_map_exit_code=$?
exit "${projected_map_exit_code}"

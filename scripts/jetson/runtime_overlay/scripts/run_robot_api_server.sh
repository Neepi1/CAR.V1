#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
umask 0002

CONFIG_FILE="${ROBOT_API_SERVER_CONFIG:-${OVERLAY_ROOT}/config/robot_api_server.yaml}"
PORT="${ROBOT_API_SERVER_PORT:-8080}"
DOCKING_SENSOR_BACKEND="${NJRH_DOCKING_SENSOR_BACKEND:-orbbec_336l}"
docking_backend_args=()

case "${DOCKING_SENSOR_BACKEND}" in
  gs2)
    docking_backend_args+=(
      -p docking_observation_backend:=gs2_scan
      -p docking_default_dock_profile_id:=gs2_rear_charging_dock
      -p docking_default_dock_profile_type:=gs2_near_field
      -p docking_default_sensor_frame:=gs2_link
    )
    ;;
  orbbec_336l)
    docking_backend_args+=(
      -p docking_observation_backend:=target_observation
      -p docking_target_observation_topic:=/dock/target_observation
      -p docking_target_observation_source:=orbbec_336l_depth
      -p docking_default_dock_profile_id:=orbbec_336l_rear_charging_dock
      -p docking_default_dock_profile_type:=depth_geometry
      -p docking_default_sensor_frame:=camera336l_depth_optical_frame
    )
    ;;
  *)
    echo "[runtime-overlay] unsupported NJRH_DOCKING_SENSOR_BACKEND=${DOCKING_SENSOR_BACKEND}" >&2
    exit 1
    ;;
esac

cd "${PROJECT_ROOT}"
set +u
source /opt/ros/humble/setup.bash

if [[ ! -x "${PROJECT_ROOT}/install/robot_api_server/lib/robot_api_server/robot_api_server_node" ]]; then
  colcon build --packages-select robot_map_asset_identity robot_interfaces robot_elevator_manager robot_api_server --symlink-install
fi

source "${PROJECT_ROOT}/install/setup.bash"
set -u

if [[ -n "${ROBOT_API_TOKEN:-}" ]]; then
  njrh_exec_affined robot_api_server ros2 run robot_api_server robot_api_server_node --ros-args \
    --params-file "${CONFIG_FILE}" \
    -p port:="${PORT}" \
    -p api_token:="${ROBOT_API_TOKEN}" \
    "${docking_backend_args[@]}"
fi

njrh_exec_affined robot_api_server ros2 run robot_api_server robot_api_server_node --ros-args \
  --params-file "${CONFIG_FILE}" \
  -p port:="${PORT}" \
  "${docking_backend_args[@]}"

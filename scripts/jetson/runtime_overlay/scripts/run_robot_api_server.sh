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

# The common owner starts the manager after main-stack initialization. Reuse
# the existing empty-command policy so an early App request cannot race it.
# Standalone API launch retains its existing on-demand manager behavior.
if [[ "${NJRH_COMMON_SERVICES_MANAGED:-false}" == "true" &&
  "${NJRH_DOCKING_MANAGER_AUTOSTART:-true}" == "true" ]]; then
  docking_backend_args+=(-p "docking_manager_start_command:=''")
fi

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
# common_env already prepares both layers for cold entries and exports them
# to resident children. Only restore an actually missing layer here.
if [[ "${ROS_VERSION:-}" != 2 || "${ROS_DISTRO:-}" != humble ||
  ":${AMENT_PREFIX_PATH:-}:" != *":/opt/ros/humble:"* ||
  ":${LD_LIBRARY_PATH:-}:" != *":/opt/ros/humble/lib:"* ]]; then
  source /opt/ros/humble/setup.bash
fi

API_BIN="${PROJECT_ROOT}/install/robot_api_server/lib/robot_api_server/robot_api_server_node"
api_built=false
if [[ ! -x "${API_BIN}" ]]; then
  colcon build --packages-select robot_map_asset_identity robot_interfaces robot_elevator_manager robot_api_server --symlink-install
  api_built=true
fi

api_overlay_ready=false
if [[ "${NJRH_COMMON_ENV_SETUP_DONE:-}" == 1 ]]; then
  api_overlay_ready=true
  for package in robot_api_server robot_interfaces; do
    package_ready=false
    for prefix in "${PROJECT_ROOT}/install" "${PROJECT_ROOT}/install/${package}"; do
      if [[ ":${AMENT_PREFIX_PATH:-}:" == *":${prefix}:"* &&
        ":${LD_LIBRARY_PATH:-}:" == *":${prefix}/lib:"* ]]; then
        package_ready=true
        break
      fi
    done
    [[ "${package_ready}" == true ]] || api_overlay_ready=false
  done
fi
if [[ "${api_built}" == true || "${api_overlay_ready}" != true ]]; then
  source "${PROJECT_ROOT}/install/local_setup.bash"
fi
set -u

njrh_exec_affined robot_api_server "${API_BIN}" --ros-args \
  --params-file "${CONFIG_FILE}" \
  -p port:="${PORT}" \
  "${docking_backend_args[@]}"

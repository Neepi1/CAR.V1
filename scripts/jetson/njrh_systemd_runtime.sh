#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-run}"

WORKSPACE_HOST="${NJRH_WORKSPACE_HOST:-/home/nvidia/workspaces/njrh-v3/workspace1}"
WORKSPACE_CONTAINER="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
UPSTREAM_WORKSPACE_HOST="${NJRH_UPSTREAM_WORKSPACE_HOST:-/home/nvidia/workspaces/isaac_ros-dev}"
UPSTREAM_WORKSPACE_CONTAINER="${NJRH_UPSTREAM_WORKSPACE_CONTAINER:-/workspaces/isaac_ros-dev}"
CONTAINER_NAME="${NJRH_CONTAINER_NAME:-NJRH-car}"
RUNTIME_USER="${NJRH_RUNTIME_USER:-root}"
RUNTIME_GROUP="${NJRH_RUNTIME_GROUP:-${RUNTIME_USER}}"
RUNTIME_HOME="${NJRH_RUNTIME_HOME:-}"
if [[ -z "${RUNTIME_HOME}" ]]; then
  if [[ "${RUNTIME_USER}" == "root" ]]; then
    RUNTIME_HOME="/root"
  else
    RUNTIME_HOME="/home/${RUNTIME_USER}"
  fi
fi
OVERLAY_CONTAINER="${WORKSPACE_CONTAINER}/scripts/jetson/runtime_overlay"

resolve_gs2_serial_port() {
  local configured="${NJRH_GS2_SERIAL_PORT:-}"
  if [[ -n "${configured}" ]]; then
    echo "${configured}"
    return 0
  fi

  if [[ -e /dev/gs2 ]]; then
    readlink -f /dev/gs2
    return 0
  fi

  local by_id=""
  if [[ -d /dev/serial/by-id ]]; then
    by_id="$(find /dev/serial/by-id -maxdepth 1 -type l -name '*CP2102*' -print -quit 2>/dev/null || true)"
  fi
  if [[ -n "${by_id}" ]]; then
    readlink -f "${by_id}"
    return 0
  fi

  echo "/dev/gs2"
}

GS2_SERIAL_PORT="$(resolve_gs2_serial_port)"

container_env=(
  "-e" "ROBOT_API_TOKEN=${ROBOT_API_TOKEN:-}"
  "-e" "NJRH_REUSE_COMMON_SERVICES=${NJRH_REUSE_COMMON_SERVICES:-true}"
  "-e" "NJRH_PROJECT_ROOT=${WORKSPACE_CONTAINER}"
  "-e" "NJRH_UPSTREAM_ROOT=${UPSTREAM_WORKSPACE_CONTAINER}"
  "-e" "NJRH_UPSTREAM_HOST_ROOT=${UPSTREAM_WORKSPACE_HOST}"
  "-e" "NJRH_GS2_SERIAL_PORT=${GS2_SERIAL_PORT}"
  "-e" "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  "-e" "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  "-e" "USER=${RUNTIME_USER}"
  "-e" "HOME=${RUNTIME_HOME}"
)

prepare_container_permissions() {
  docker exec -u root "${CONTAINER_NAME}" /bin/bash -lc "
    set -e
    mkdir -p \
      '${WORKSPACE_CONTAINER}/maps_release' \
      '${OVERLAY_CONTAINER}/web_dashboard/runtime_logs' \
      '${OVERLAY_CONTAINER}/maps' \
      '${OVERLAY_CONTAINER}/maps3d' \
      '${OVERLAY_CONTAINER}/waypoints'
    chown -R '${RUNTIME_USER}:${RUNTIME_GROUP}' \
      '${WORKSPACE_CONTAINER}/maps_release' \
      '${OVERLAY_CONTAINER}/web_dashboard/runtime_logs' \
      '${OVERLAY_CONTAINER}/maps' \
      '${OVERLAY_CONTAINER}/maps3d' \
      '${OVERLAY_CONTAINER}/waypoints'
    find \
      '${WORKSPACE_CONTAINER}/maps_release' \
      '${OVERLAY_CONTAINER}/web_dashboard/runtime_logs' \
      '${OVERLAY_CONTAINER}/maps' \
      '${OVERLAY_CONTAINER}/maps3d' \
      '${OVERLAY_CONTAINER}/waypoints' \
      -type d -exec chmod 2775 {} +
    find \
      '${WORKSPACE_CONTAINER}/maps_release' \
      '${OVERLAY_CONTAINER}/web_dashboard/runtime_logs' \
      '${OVERLAY_CONTAINER}/maps' \
      '${OVERLAY_CONTAINER}/maps3d' \
      '${OVERLAY_CONTAINER}/waypoints' \
      -type f -exec chmod 664 {} +
  " >/dev/null
}

stop_container_common_processes() {
  if ! docker ps --format '{{.Names}}' | grep -Fx "${CONTAINER_NAME}" >/dev/null 2>&1; then
    return 0
  fi
  docker exec "${CONTAINER_NAME}" /bin/bash -lc '
    set +e
    stop_by_pattern() {
      local pattern="$1"
      ps -eo pid=,args= \
        | awk -v pattern="${pattern}" '"'"'$0 ~ pattern && $0 !~ /awk/ {print $1}'"'"' \
        | xargs -r kill -INT 2>/dev/null || true
    }
    kill_by_pattern() {
      local pattern="$1"
      ps -eo pid=,args= \
        | awk -v pattern="${pattern}" '"'"'$0 ~ pattern && $0 !~ /awk/ {print $1}'"'"' \
        | xargs -r kill -9 2>/dev/null || true
    }
    common_pattern="run_common_services.sh"
    node_pattern="hesai_ros_driver_node|pointcloud_axis_remap|imu_axis_remap|ranger_base_node|robot_description_static_tf_node|robot_eai_gs2/gs2_driver_node|gs2_driver_node --ros-args|ros2 launch robot_eai_gs2 gs2.launch.py|ekf_node --ros-args.*__node:=robot_local_state|robot_localization/ekf_node|robot_local_perception/local_perception_node|robot_floor_manager/floor_manager_node|robot_safety/robot_safety_node|ranger_mini3_mode_controller/mode_controller_node|run_robot_api_server_supervised.sh|robot_api_server/robot_api_server_node|robot_api_server_node --ros-args"
    stop_by_pattern "${common_pattern}"
    sleep 2
    stop_by_pattern "${node_pattern}"
    sleep 1
    kill_by_pattern "${common_pattern}"
    kill_by_pattern "${node_pattern}"
  ' || true
}

cd "${WORKSPACE_HOST}"

case "${ACTION}" in
  run)
    NJRH_WORKSPACE_HOST="${WORKSPACE_HOST}" \
    NJRH_WORKSPACE_CONTAINER="${WORKSPACE_CONTAINER}" \
    NJRH_UPSTREAM_WORKSPACE_HOST="${UPSTREAM_WORKSPACE_HOST}" \
    NJRH_UPSTREAM_WORKSPACE_CONTAINER="${UPSTREAM_WORKSPACE_CONTAINER}" \
    NJRH_CONTAINER_NAME="${CONTAINER_NAME}" \
      bash scripts/jetson/njrh_container.sh start

    prepare_container_permissions
    stop_container_common_processes

    echo "[njrh-systemd] GS2 serial port resolved to ${GS2_SERIAL_PORT}" >&2
    exec docker exec -u "${RUNTIME_USER}" --workdir "${OVERLAY_CONTAINER}" "${container_env[@]}" "${CONTAINER_NAME}" \
      /bin/bash -lc "cd '${OVERLAY_CONTAINER}' && exec bash scripts/run_common_services.sh"
    ;;
  stop)
    stop_container_common_processes
    ;;
  status)
    NJRH_WORKSPACE_HOST="${WORKSPACE_HOST}" \
    NJRH_WORKSPACE_CONTAINER="${WORKSPACE_CONTAINER}" \
    NJRH_UPSTREAM_WORKSPACE_HOST="${UPSTREAM_WORKSPACE_HOST}" \
    NJRH_UPSTREAM_WORKSPACE_CONTAINER="${UPSTREAM_WORKSPACE_CONTAINER}" \
    NJRH_CONTAINER_NAME="${CONTAINER_NAME}" \
      bash scripts/jetson/njrh_container.sh status
    ;;
  *)
    echo "unsupported action: ${ACTION}" >&2
    exit 1
    ;;
esac

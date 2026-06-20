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
  "-e" "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=${NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-false}"
  "-e" "NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=${NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION:-false}"
  "-e" "NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}"
  "-e" "NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=${NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false}"
  "-e" "NJRH_NAV2_LIFECYCLE_PARALLEL_BT=${NJRH_NAV2_LIFECYCLE_PARALLEL_BT:-true}"
  "-e" "NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=${NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false}"
  "-e" "NJRH_COMMON_LOCAL_STATE_START_READY_MODE=${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}"
  "-e" "NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=${NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE:-true}"
  "-e" "NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST=${NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false}"
  "-e" "NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC=${NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC:-15}"
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
  local mode="${NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE:-once}"
  local marker="${NJRH_RUNTIME_PERMISSIONS_MARKER:-${WORKSPACE_CONTAINER}/.njrh_runtime_permissions_ready}"
  case "${mode}" in
    skip)
      echo "[njrh-systemd] runtime permission preparation skipped by NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=skip" >&2
      return 0
      ;;
    once|always)
      ;;
    *)
      echo "[njrh-systemd] unsupported NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=${mode}; expected once|always|skip" >&2
      exit 1
      ;;
  esac
  if [[ "${mode}" == "once" ]] \
    && docker exec "${CONTAINER_NAME}" test -f "${marker}" >/dev/null 2>&1; then
    echo "[njrh-systemd] runtime permission preparation already complete marker=${marker}" >&2
    return 0
  fi

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
    touch '${marker}'
  " >/dev/null
}

stop_container_common_processes() {
  if ! docker ps --format '{{.Names}}' | grep -Fx "${CONTAINER_NAME}" >/dev/null 2>&1; then
    return 0
  fi
  docker exec "${CONTAINER_NAME}" /bin/bash -lc '
    set +e
    pids_by_pattern() {
      local pattern="$1"
      ps -eo pid=,args= \
        | awk -v pattern="${pattern}" '"'"'
          $0 ~ pattern &&
          $0 !~ /awk -v pattern/ &&
          $0 !~ /pids_by_pattern/ &&
          $0 !~ /stop_exact_process_set/ &&
          $0 !~ /common_pattern=/ &&
          $0 !~ /node_pattern=/ &&
          $0 !~ /ros2_cli_pattern=/ {print $1}
        '"'"'
    }
    wait_pids_gone() {
      local timeout_sec="$1"
      shift || true
      local pids=("$@")
      local deadline=$((SECONDS + timeout_sec))
      local pid
      while (( SECONDS < deadline )); do
        local alive=0
        for pid in "${pids[@]}"; do
          [[ -n "${pid}" && -d "/proc/${pid}" ]] && alive=1
        done
        [[ "${alive}" -eq 0 ]] && return 0
        sleep 0.2
      done
      return 1
    }
    stop_exact_process_set() {
      local label="$1"
      local pattern="$2"
      mapfile -t pids < <(pids_by_pattern "${pattern}")
      [[ "${#pids[@]}" -gt 0 ]] || return 0
      echo "[njrh-systemd] stopping ${label} pids=${pids[*]}" >&2
      kill -INT "${pids[@]}" 2>/dev/null || true
      wait_pids_gone 2 "${pids[@]}" && return 0
      mapfile -t pids < <(pids_by_pattern "${pattern}")
      [[ "${#pids[@]}" -gt 0 ]] || return 0
      kill -TERM "${pids[@]}" 2>/dev/null || true
      wait_pids_gone 3 "${pids[@]}" && return 0
      mapfile -t pids < <(pids_by_pattern "${pattern}")
      [[ "${#pids[@]}" -gt 0 ]] || return 0
      echo "[njrh-systemd] killing exact stale ${label} pids=${pids[*]}" >&2
      kill -KILL "${pids[@]}" 2>/dev/null || true
    }
    common_pattern="run_common_services.sh"
    node_pattern="hesai_ros_driver_node|pointcloud_axis_remap|imu_axis_remap|ranger_base_node|robot_description_static_tf_node|robot_eai_gs2/gs2_driver_node|gs2_driver_node --ros-args|ros2 launch robot_eai_gs2 gs2.launch.py|ekf_node --ros-args.*__node:=robot_local_state|robot_localization/ekf_node|robot_local_perception/local_perception_node|robot_floor_manager/floor_manager_node|robot_safety/robot_safety_node|ranger_mini3_mode_controller/mode_controller_node|run_robot_api_server_supervised.sh|robot_api_server/robot_api_server_node|robot_api_server_node --ros-args|run_navigation_runtime_services.sh|nav2_lifecycle_sequence.py|call_global_localization_trigger.py|run_nav2_navigation.sh|run_occupancy_grid_localization.sh|standard_navigation.launch.py|occupancy_localization_stack.launch.py|occupancy_grid_localizer_container|occupancy_grid_localizer|robot_localization_bridge/localization_bridge_node|localization_bridge_node --ros-args|amcl --ros-args|nav2_amcl|amcl_scan_admission|__node:=map_server|__node:=controller_server|__node:=planner_server|__node:=bt_navigator|__node:=behavior_server|__node:=velocity_smoother|__node:=collision_monitor|__node:=lifecycle_manager_navigation|__node:=lifecycle_manager_costmap_filters"
    ros2_cli_pattern="/opt/ros/humble/bin/ros2 (lifecycle get|topic echo|topic hz|topic info|node info|service call /amcl/(change_state|get_state))|ros2 (lifecycle get|topic echo|topic hz|topic info|node info|service call /amcl/(change_state|get_state))"
    stop_exact_process_set "stale ros2 diagnostics cli" "${ros2_cli_pattern}"
    stop_exact_process_set "common services" "${common_pattern}"
    stop_exact_process_set "runtime nodes" "${node_pattern}"
    rm -f \
      /tmp/njrh_runtime_map_context.json \
      /tmp/njrh_amcl_runtime_status.env \
      /tmp/njrh_nav2_launch_hold_ready.env \
      2>/dev/null || true
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

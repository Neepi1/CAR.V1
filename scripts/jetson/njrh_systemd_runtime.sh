#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-run}"

WORKSPACE_HOST="${NJRH_WORKSPACE_HOST:-/home/nvidia/workspaces/njrh-v3/workspace1}"
WORKSPACE_CONTAINER="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
UPSTREAM_WORKSPACE_HOST="${NJRH_UPSTREAM_WORKSPACE_HOST:-/home/nvidia/workspaces/isaac_ros-dev}"
UPSTREAM_WORKSPACE_CONTAINER="${NJRH_UPSTREAM_WORKSPACE_CONTAINER:-/workspaces/isaac_ros-dev}"
CONTAINER_NAME="${NJRH_CONTAINER_NAME:-NJRH-car}"
PROVISION_MOTION_LOCK="${NJRH_PROVISION_MOTION_LOCK:-/var/lib/njrh/provision/motion.lock}"
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

RUNTIME_OVERRIDE_ENV="${NJRH_RUNTIME_OVERRIDE_ENV:-/tmp/njrh_runtime_override.env}"
if [[ -f "${RUNTIME_OVERRIDE_ENV}" ]]; then
  # shellcheck source=/dev/null
  set -a
  source "${RUNTIME_OVERRIDE_ENV}"
  set +a
fi

DOCKING_SENSOR_BACKEND="${NJRH_DOCKING_SENSOR_BACKEND:-orbbec_336l}"
GS2_AUTOSTART="${NJRH_GS2_AUTOSTART:-false}"
if [[ "${DOCKING_SENSOR_BACKEND}" == "gs2" ]]; then
  GS2_SERIAL_PORT="$(resolve_gs2_serial_port)"
else
  GS2_SERIAL_PORT=""
fi

container_env=(
  "-e" "ROBOT_API_TOKEN"
  "-e" "NJRH_REUSE_COMMON_SERVICES=${NJRH_REUSE_COMMON_SERVICES:-true}"
  "-e" "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=${NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION:-true}"
  "-e" "NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE=${NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE:-true}"
  "-e" "NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=${NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION:-true}"
  "-e" "NJRH_NAV2_PRESTART_AFTER_LOCALIZATION_STACK=${NJRH_NAV2_PRESTART_AFTER_LOCALIZATION_STACK:-false}"
  "-e" "NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=${NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START:-false}"
  "-e" "NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=${NJRH_NAV2_LIFECYCLE_PARALLEL_CORE:-false}"
  "-e" "NJRH_NAV2_LIFECYCLE_PARALLEL_BT=${NJRH_NAV2_LIFECYCLE_PARALLEL_BT:-true}"
  "-e" "NJRH_NAV2_LIFECYCLE_BACKGROUND_START=${NJRH_NAV2_LIFECYCLE_BACKGROUND_START:-false}"
  "-e" "NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=${NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK:-false}"
  "-e" "NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE=${NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE:-true}"
  "-e" "NJRH_NAV2_PLANNER_PROFILE=${NJRH_NAV2_PLANNER_PROFILE:-smac2d}"
  "-e" "NJRH_AMCL_LOCALIZATION_MODE=${NJRH_AMCL_LOCALIZATION_MODE:-gated}"
  "-e" "NJRH_ORBBEC_SERIAL_NUMBER=${NJRH_ORBBEC_SERIAL_NUMBER:-}"
  "-e" "NJRH_NAV2_PLANNER_PROFILE_FILE=${NJRH_NAV2_PLANNER_PROFILE_FILE:-}"
  "-e" "NJRH_RANGER_LATTICE_FILE=${NJRH_RANGER_LATTICE_FILE:-}"
  "-e" "NJRH_NAV2_BT_XML=${NJRH_NAV2_BT_XML:-}"
  "-e" "NJRH_COMMON_LOCAL_STATE_START_READY_MODE=${NJRH_COMMON_LOCAL_STATE_START_READY_MODE:-endpoint}"
  "-e" "NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=${NJRH_COMMON_LOCAL_STATE_BACKGROUND_START:-true}"
  "-e" "NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=${NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE:-false}"
  "-e" "LOCAL_STATE_PROCESS_START_TIMEOUT_SEC=${LOCAL_STATE_PROCESS_START_TIMEOUT_SEC:-}"
  "-e" "LOCAL_STATE_START_READY_TIMEOUT_SEC=${LOCAL_STATE_START_READY_TIMEOUT_SEC:-}"
  "-e" "LOCAL_STATE_READY_RECHECK_TIMEOUT_SEC=${LOCAL_STATE_READY_RECHECK_TIMEOUT_SEC:-}"
  "-e" "LOCAL_STATE_IMU_BIAS_FILTER_READY_CHECK=${LOCAL_STATE_IMU_BIAS_FILTER_READY_CHECK:-}"
  "-e" "LOCAL_STATE_IMU_BIAS_FILTER_READY_TIMEOUT_SEC=${LOCAL_STATE_IMU_BIAS_FILTER_READY_TIMEOUT_SEC:-}"
  "-e" "LOCAL_STATE_LAUNCH_SETTLE_SEC=${LOCAL_STATE_LAUNCH_SETTLE_SEC:-}"
  "-e" "NJRH_INITIAL_LOCALIZATION_SERVICE_WAIT_SEC=${NJRH_INITIAL_LOCALIZATION_SERVICE_WAIT_SEC:-}"
  "-e" "NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST=${NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false}"
  "-e" "NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC=${NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC:-15}"
  "-e" "NJRH_NAV_LOCAL_STATE_MODE=${NJRH_NAV_LOCAL_STATE_MODE:-}"
  "-e" "NJRH_LOCAL_STATE_EKF_PROFILE=${NJRH_LOCAL_STATE_EKF_PROFILE:-}"
  "-e" "LOCAL_STATE_EKF_PROFILE=${LOCAL_STATE_EKF_PROFILE:-}"
  "-e" "NJRH_FORCE_RESTART_CANONICAL_TF=${NJRH_FORCE_RESTART_CANONICAL_TF:-}"
  "-e" "NJRH_FORCE_RESTART_NAV_HELPERS=${NJRH_FORCE_RESTART_NAV_HELPERS:-}"
  "-e" "NJRH_DOCKING_SENSOR_BACKEND=${DOCKING_SENSOR_BACKEND}"
  "-e" "NJRH_GS2_AUTOSTART=${GS2_AUTOSTART}"
  "-e" "NJRH_PROJECT_ROOT=${WORKSPACE_CONTAINER}"
  "-e" "NJRH_UPSTREAM_ROOT=${UPSTREAM_WORKSPACE_CONTAINER}"
  "-e" "NJRH_UPSTREAM_HOST_ROOT=${UPSTREAM_WORKSPACE_HOST}"
  "-e" "NJRH_GS2_SERIAL_PORT=${GS2_SERIAL_PORT}"
  "-e" "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  "-e" "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  "-e" "USER=${RUNTIME_USER}"
  "-e" "HOME=${RUNTIME_HOME}"
)

clear_runtime_status_files() {
  rm -f \
    /tmp/njrh_runtime_map_context.json \
    /tmp/njrh_runtime_health.json \
    /tmp/njrh_amcl_runtime_status.env \
    /tmp/njrh_nav2_launch_hold_ready.env \
    /tmp/njrh_nav2_lifecycle_ready.env \
    2>/dev/null || true
}

verify_container_runtime_processes_absent() {
  docker exec "${CONTAINER_NAME}" /bin/bash \
    "${OVERLAY_CONTAINER}/scripts/stop_runtime_processes.sh" --check
}

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

  docker exec -u root "${CONTAINER_NAME}" /bin/bash -c "
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
    clear_runtime_status_files
    return 0
  fi
  # Always run the idempotent complete-runtime sweep. A pre-check must never
  # skip cleanup because a stale service omitted from a host regex could still
  # execute a timed-out ROS request after the API process restarts.
  if ! docker exec "${CONTAINER_NAME}" /bin/bash \
    "${OVERLAY_CONTAINER}/scripts/stop_runtime_processes.sh"; then
    echo "[njrh-systemd] runtime cleanup command failed" >&2
    clear_runtime_status_files
    return 1
  fi
  clear_runtime_status_files
  if verify_container_runtime_processes_absent; then
    return 0
  fi
  echo "[njrh-systemd] unable to prove that every runtime process stopped" >&2
  return 1
}

cd "${WORKSPACE_HOST}"

case "${ACTION}" in
  run)
    if [[ -e "${PROVISION_MOTION_LOCK}" ]]; then
      echo "[njrh-systemd] provisioning motion lock is present: ${PROVISION_MOTION_LOCK}" >&2
      echo "[njrh-systemd] refusing to start the production runtime" >&2
      exit 1
    fi
    NJRH_WORKSPACE_HOST="${WORKSPACE_HOST}" \
    NJRH_WORKSPACE_CONTAINER="${WORKSPACE_CONTAINER}" \
    NJRH_UPSTREAM_WORKSPACE_HOST="${UPSTREAM_WORKSPACE_HOST}" \
    NJRH_UPSTREAM_WORKSPACE_CONTAINER="${UPSTREAM_WORKSPACE_CONTAINER}" \
    NJRH_CONTAINER_NAME="${CONTAINER_NAME}" \
      bash scripts/jetson/njrh_container.sh start

    prepare_container_permissions
    stop_container_common_processes

    if [[ "${DOCKING_SENSOR_BACKEND}" == "gs2" ]]; then
      echo "[njrh-systemd] docking backend=gs2 serial_port=${GS2_SERIAL_PORT} autostart=${GS2_AUTOSTART}" >&2
    else
      echo "[njrh-systemd] docking backend=${DOCKING_SENSOR_BACKEND}; GS2 disabled" >&2
    fi
    exec docker exec -u "${RUNTIME_USER}" --workdir "${OVERLAY_CONTAINER}" "${container_env[@]}" "${CONTAINER_NAME}" \
      /bin/bash -c "cd '${OVERLAY_CONTAINER}' && exec bash scripts/run_common_services.sh"
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

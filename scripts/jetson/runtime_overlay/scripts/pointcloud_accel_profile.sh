#!/usr/bin/env bash
# Shared resolver for Phase 1.13 pointcloud acceleration profiles.

njrh_load_pointcloud_accel_profile() {
  local profile_file="${NJRH_POINTCLOUD_ACCEL_PROFILE_FILE:-${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env}"
  local profile_source="default"
  if [[ -z "${NJRH_POINTCLOUD_ACCEL_PROFILE:-}" && -f "${profile_file}" ]]; then
    # shellcheck source=/dev/null
    source "${profile_file}"
    profile_source="${profile_file}"
  elif [[ -n "${NJRH_POINTCLOUD_ACCEL_PROFILE:-}" ]]; then
    profile_source="environment"
  fi

  local profile="${NJRH_POINTCLOUD_ACCEL_PROFILE:-legacy}"
  case "${profile}" in
    legacy|ipc_worker|nitros) ;;
    *)
      echo "[runtime-overlay] invalid NJRH_POINTCLOUD_ACCEL_PROFILE=${profile}; expected legacy, ipc_worker, or nitros" >&2
      return 2
      ;;
  esac

  export NJRH_POINTCLOUD_ACCEL_PROFILE="${profile}"
  export NJRH_POINTCLOUD_ACCEL_PROFILE_SOURCE="${profile_source}"
  export NJRH_POINTCLOUD_ACCEL_PROFILE_FILE_RESOLVED="${profile_file}"
}

njrh_print_pointcloud_accel_profile() {
  echo "[runtime-overlay] pointcloud accel profile=${NJRH_POINTCLOUD_ACCEL_PROFILE} source=${NJRH_POINTCLOUD_ACCEL_PROFILE_SOURCE}" >&2
  case "${NJRH_POINTCLOUD_ACCEL_PROFILE}" in
    legacy)
      echo "[runtime-overlay] topology: /lidar_points full trunk; /_internal/lidar_points_local -> robot_local_perception; /lidar_points_nav -> /points_nav -> /scan -> /flatscan" >&2
      ;;
    ipc_worker)
      echo "[runtime-overlay] topology: /lidar_points full trunk; same-process workers publish /perception/* and /scan; /flatscan comes from laser_scan_to_flatscan" >&2
      ;;
    nitros)
      echo "[runtime-overlay] topology: NITROS navigation-branch skeleton only; /lidar_points full trunk remains non-NITROS mapping input" >&2
      ;;
  esac
}

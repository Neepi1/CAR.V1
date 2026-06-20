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

  local profile="${NJRH_POINTCLOUD_ACCEL_PROFILE:-ipc_worker}"
  case "${profile}" in
    ipc_worker|nitros) ;;
    legacy)
      echo "[runtime-overlay] NJRH_POINTCLOUD_ACCEL_PROFILE=legacy has been removed from production." >&2
      echo "[runtime-overlay] Use ipc_worker: Nav2 local costmap/collision_monitor consume /scan for standard marking+clearing." >&2
      return 2
      ;;
    *)
      echo "[runtime-overlay] invalid NJRH_POINTCLOUD_ACCEL_PROFILE=${profile}; expected ipc_worker or nitros" >&2
      return 2
      ;;
  esac

  export NJRH_POINTCLOUD_ACCEL_PROFILE="${profile}"
  export NJRH_POINTCLOUD_ACCEL_PROFILE_SOURCE="${profile_source}"
  export NJRH_POINTCLOUD_ACCEL_PROFILE_FILE_RESOLVED="${profile_file}"
}

njrh_load_pointcloud_ingress_profile() {
  local profile_file="${NJRH_POINTCLOUD_INGRESS_PROFILE_FILE:-${NJRH_OVERLAY_ROOT}/config/pointcloud_ingress_profile.env}"
  local profile_source="default"
  if [[ -z "${NJRH_POINTCLOUD_INGRESS_PROFILE:-}" && -f "${profile_file}" ]]; then
    # shellcheck source=/dev/null
    source "${profile_file}"
    profile_source="${profile_file}"
  elif [[ -n "${NJRH_POINTCLOUD_INGRESS_PROFILE:-}" ]]; then
    profile_source="environment"
  fi

  local ingress_profile="${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}"
  case "${ingress_profile}" in
    separate_process|driver_integrated) ;;
    *)
      echo "[runtime-overlay] invalid NJRH_POINTCLOUD_INGRESS_PROFILE=${ingress_profile}; expected separate_process or driver_integrated" >&2
      return 2
      ;;
  esac

  export NJRH_POINTCLOUD_INGRESS_PROFILE="${ingress_profile}"
  export NJRH_POINTCLOUD_INGRESS_PROFILE_SOURCE="${profile_source}"
  export NJRH_POINTCLOUD_INGRESS_PROFILE_FILE_RESOLVED="${profile_file}"
}

njrh_print_pointcloud_accel_profile() {
  echo "[runtime-overlay] pointcloud accel profile=${NJRH_POINTCLOUD_ACCEL_PROFILE} source=${NJRH_POINTCLOUD_ACCEL_PROFILE_SOURCE}" >&2
  if [[ -n "${NJRH_POINTCLOUD_INGRESS_PROFILE:-}" ]]; then
    echo "[runtime-overlay] pointcloud ingress profile=${NJRH_POINTCLOUD_INGRESS_PROFILE} source=${NJRH_POINTCLOUD_INGRESS_PROFILE_SOURCE:-unknown}" >&2
  fi
  case "${NJRH_POINTCLOUD_ACCEL_PROFILE}" in
    ipc_worker)
      if [[ "${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}" == "driver_integrated" ]]; then
        echo "[runtime-overlay] topology: driver_integrated ingress; Hesai decode feeds AccelCore; local costmap/collision_monitor consume /scan" >&2
      else
        echo "[runtime-overlay] topology: /lidar_points full trunk; same-process scan worker publishes /scan; /flatscan comes from laser_scan_to_flatscan" >&2
      fi
      ;;
    nitros)
      echo "[runtime-overlay] topology: NITROS navigation-branch skeleton only; /lidar_points full trunk remains non-NITROS mapping input" >&2
      ;;
  esac
}

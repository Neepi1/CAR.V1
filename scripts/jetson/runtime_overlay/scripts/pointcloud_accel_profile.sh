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

  if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE:-legacy}" == "legacy" && "${ingress_profile}" != "separate_process" ]]; then
    echo "[runtime-overlay] forcing pointcloud ingress separate_process for legacy accel profile" >&2
    ingress_profile="separate_process"
    profile_source="legacy_forced_separate_process"
  fi

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
    legacy)
      echo "[runtime-overlay] topology: /lidar_points full trunk; /_internal/lidar_points_local -> robot_local_perception; /lidar_points_nav -> /points_nav -> /scan -> /flatscan" >&2
      ;;
    ipc_worker)
      if [[ "${NJRH_POINTCLOUD_INGRESS_PROFILE:-separate_process}" == "driver_integrated" ]]; then
        echo "[runtime-overlay] topology: driver_integrated ingress; Hesai decode feeds AccelCore in the same process" >&2
      else
        echo "[runtime-overlay] topology: /lidar_points full trunk; same-process workers publish /perception/* and /scan; /flatscan comes from laser_scan_to_flatscan" >&2
      fi
      ;;
    nitros)
      echo "[runtime-overlay] topology: NITROS navigation-branch skeleton only; /lidar_points full trunk remains non-NITROS mapping input" >&2
      ;;
  esac
}

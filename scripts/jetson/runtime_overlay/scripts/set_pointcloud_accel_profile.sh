#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"

PROFILE=""
INGRESS_PROFILE=""
DO_PRINT=false
DO_RESTART=false
PROFILE_FILE="${NJRH_POINTCLOUD_ACCEL_PROFILE_FILE:-${NJRH_OVERLAY_ROOT}/config/pointcloud_accel_profile.env}"
INGRESS_PROFILE_FILE="${NJRH_POINTCLOUD_INGRESS_PROFILE_FILE:-${NJRH_OVERLAY_ROOT}/config/pointcloud_ingress_profile.env}"

usage() {
  cat <<'EOF'
Usage: set_pointcloud_accel_profile.sh [--profile legacy|ipc_worker|nitros] [--ingress-profile separate_process|driver_integrated] [--print] [--restart]

Profiles:
  legacy      Current verified branch topology and one-command rollback.
  ipc_worker  Same-process navigation workers; /lidar_points remains full trunk.
  nitros      Isaac ROS NITROS navigation-branch skeleton; never replaces /lidar_points.

Ingress:
  separate_process   Current production path through /jt128/vendor/points_raw.
  driver_integrated  Guarded until a repo-owned Hesai driver overlay is available.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --profile)
      [[ "$#" -ge 2 ]] || { echo "[pointcloud-accel] --profile requires a value" >&2; exit 2; }
      PROFILE="$2"
      shift 2
      ;;
    --ingress-profile)
      [[ "$#" -ge 2 ]] || { echo "[pointcloud-accel] --ingress-profile requires a value" >&2; exit 2; }
      INGRESS_PROFILE="$2"
      shift 2
      ;;
    --print)
      DO_PRINT=true
      shift
      ;;
    --restart)
      DO_RESTART=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[pointcloud-accel] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -n "${PROFILE}" ]]; then
  case "${PROFILE}" in
    legacy|ipc_worker|nitros) ;;
    *)
      echo "[pointcloud-accel] invalid profile: ${PROFILE}" >&2
      usage >&2
      exit 2
      ;;
  esac
  if [[ "${PROFILE}" == "nitros" ]]; then
    if ! bash "${SCRIPT_DIR}/check_isaac_ros_nitros_env.sh"; then
      echo "[pointcloud-accel] NITROS profile was not written. Use --profile ipc_worker or --profile legacy." >&2
      exit 3
    fi
  fi
  mkdir -p "$(dirname "${PROFILE_FILE}")"
  {
    echo "# Runtime-selected pointcloud acceleration profile."
    echo "# Valid values: legacy, ipc_worker, nitros"
    printf 'export NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE:-%s}"\n' "${PROFILE}"
  } >"${PROFILE_FILE}"
  export NJRH_POINTCLOUD_ACCEL_PROFILE="${PROFILE}"
  echo "[pointcloud-accel] wrote ${PROFILE_FILE}: ${PROFILE}"
fi

if [[ -n "${INGRESS_PROFILE}" ]]; then
  case "${INGRESS_PROFILE}" in
    separate_process|driver_integrated) ;;
    *)
      echo "[pointcloud-accel] invalid ingress profile: ${INGRESS_PROFILE}" >&2
      usage >&2
      exit 2
      ;;
  esac
  mkdir -p "$(dirname "${INGRESS_PROFILE_FILE}")"
  {
    echo "# Runtime-selected pointcloud ingress profile."
    echo "# Valid values: separate_process, driver_integrated"
    printf 'export NJRH_POINTCLOUD_INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE:-%s}"\n' "${INGRESS_PROFILE}"
  } >"${INGRESS_PROFILE_FILE}"
  export NJRH_POINTCLOUD_INGRESS_PROFILE="${INGRESS_PROFILE}"
  echo "[pointcloud-accel] wrote ${INGRESS_PROFILE_FILE}: ${INGRESS_PROFILE}"
fi

njrh_load_pointcloud_accel_profile
njrh_load_pointcloud_ingress_profile

if [[ "${DO_PRINT}" == "true" || -z "${PROFILE}" ]]; then
  njrh_print_pointcloud_accel_profile
  echo "[pointcloud-accel] profile_file=${PROFILE_FILE}"
  echo "[pointcloud-accel] ingress_profile_file=${INGRESS_PROFILE_FILE}"
fi

cat <<'EOF'
[pointcloud-accel] Validation commands:
[pointcloud-accel]   bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_accel_profile.sh
[pointcloud-accel]   timeout 12 ros2 topic echo /lidar/axis_remap_status --field data
[pointcloud-accel]   ros2 topic hz /perception/obstacle_points
[pointcloud-accel]   ros2 topic hz /scan
[pointcloud-accel]   ros2 topic hz /flatscan
EOF

if [[ "${DO_RESTART}" != "true" ]]; then
  echo "[pointcloud-accel] --restart not requested; no production process was stopped."
  exit 0
fi

mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
echo "[pointcloud-accel] restarting pointcloud profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}" >&2
nohup env \
  NJRH_POINTCLOUD_ACCEL_PROFILE="${NJRH_POINTCLOUD_ACCEL_PROFILE}" \
  NJRH_POINTCLOUD_INGRESS_PROFILE="${NJRH_POINTCLOUD_INGRESS_PROFILE}" \
  NJRH_POINTCLOUD_ACCEL_RESTART=true \
  NJRH_FORCE_RESTART_DRIVER=true \
  bash "${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh" \
  >"${NJRH_RUNTIME_LOG_DIR}/run_pointcloud_accel_pipeline.log" 2>&1 &
echo "[pointcloud-accel] run_pointcloud_accel_pipeline pid=$!"

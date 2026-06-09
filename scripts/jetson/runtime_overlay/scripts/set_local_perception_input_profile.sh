#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"

PROFILE=""
DO_PRINT=false
DO_RESTART=false
PROFILE_FILE="${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE_FILE:-${NJRH_OVERLAY_ROOT}/config/local_perception_input_profile.env}"

usage() {
  cat <<'EOF'
Usage: set_local_perception_input_profile.sh [--profile local_branch|trunk] [--print] [--restart]

Profiles:
  local_branch  pointcloud_axis_remap derives /_internal/lidar_points_local and robot_local_perception consumes it
  trunk         robot_local_perception consumes /lidar_points directly and the hidden local branch is disabled

This script updates the runtime overlay profile file only. It does not restart
processes unless --restart is explicitly provided.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --profile)
      [[ "$#" -ge 2 ]] || { echo "[local-profile] --profile requires a value" >&2; exit 2; }
      PROFILE="$2"
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
      echo "[local-profile] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -n "${PROFILE}" ]]; then
  case "${PROFILE}" in
    local_branch|trunk) ;;
    *)
      echo "[local-profile] invalid profile: ${PROFILE}" >&2
      usage >&2
      exit 2
      ;;
  esac
  mkdir -p "$(dirname "${PROFILE_FILE}")"
  {
    echo "# Runtime-selected local perception input profile."
    echo "# Valid values: local_branch, trunk"
    printf 'export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=%s\n' "${PROFILE}"
  } >"${PROFILE_FILE}"
  export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE="${PROFILE}"
  echo "[local-profile] wrote ${PROFILE_FILE}: ${PROFILE}"
fi

njrh_load_local_perception_input_profile

if [[ "${DO_PRINT}" == "true" || -z "${PROFILE}" ]]; then
  njrh_print_local_perception_profile
  echo "[local-profile] profile_file=${PROFILE_FILE}"
fi

cat <<'EOF'
[local-profile] Services that must be restarted for the profile to take effect:
[local-profile]   - robot_hesai_jt128 / pointcloud_axis_remap
[local-profile]   - robot_local_perception
[local-profile] Validation commands:
[local-profile]   bash scripts/jetson/runtime_overlay/scripts/inspect_pointcloud_subscribers.sh
[local-profile]   bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_delivery_matrix.sh
[local-profile]   timeout 12 ros2 topic echo /lidar/axis_remap_status --field data
[local-profile]   timeout 12 ros2 topic echo /perception/local_perception_status --field data
[local-profile]   ros2 topic hz /perception/obstacle_points
EOF

if [[ "${DO_RESTART}" != "true" ]]; then
  echo "[local-profile] --restart not requested; no process was restarted."
  exit 0
fi

mkdir -p "${NJRH_RUNTIME_LOG_DIR}"

echo "[local-profile] restarting JT128 driver/remap with profile=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" >&2
nohup env \
  NJRH_LOCAL_PERCEPTION_INPUT_PROFILE="${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" \
  NJRH_FORCE_RESTART_DRIVER=true \
  bash "${SCRIPT_DIR}/run_driver.sh" \
  >"${NJRH_RUNTIME_LOG_DIR}/run_driver_local_profile.log" 2>&1 &
echo "[local-profile] run_driver restart pid=$!"

sleep 5

echo "[local-profile] restarting robot_local_perception with profile=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" >&2
pkill -INT -f 'install/robot_local_perception/.*/local_perception_node|robot_local_perception/local_perception_node|/local_perception_node' 2>/dev/null || true
sleep 1
pkill -9 -f 'install/robot_local_perception/.*/local_perception_node|robot_local_perception/local_perception_node|/local_perception_node' 2>/dev/null || true
nohup env \
  NJRH_LOCAL_PERCEPTION_INPUT_PROFILE="${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" \
  bash "${SCRIPT_DIR}/run_local_perception.sh" \
  >"${NJRH_RUNTIME_LOG_DIR}/run_local_perception_local_profile.log" 2>&1 &
echo "[local-profile] run_local_perception restart pid=$!"


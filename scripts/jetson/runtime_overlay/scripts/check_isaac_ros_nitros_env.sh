#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

missing=0

check_cmd() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    echo "[nitros-env] PASS ${label}"
  else
    echo "[nitros-env] FAIL ${label}"
    missing=$((missing + 1))
  fi
}

check_optional_file() {
  local label="$1"
  local path="$2"
  if [[ -e "${path}" ]]; then
    echo "[nitros-env] PASS ${label}: ${path}"
  else
    echo "[nitros-env] WARN ${label}: missing ${path}"
  fi
}

echo "[nitros-env] ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "[nitros-env] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
if [[ -r /etc/nv_tegra_release ]]; then
  echo "[nitros-env] L4T=$(head -1 /etc/nv_tegra_release)"
else
  echo "[nitros-env] WARN L4T release file is not readable"
fi

if command -v nvcc >/dev/null 2>&1; then
  echo "[nitros-env] PASS CUDA nvcc: $(nvcc --version | tail -1)"
elif command -v nvidia-smi >/dev/null 2>&1; then
  echo "[nitros-env] PASS CUDA nvidia-smi available"
else
  echo "[nitros-env] WARN CUDA command not found"
fi

check_optional_file "Isaac ROS workspace" "/workspaces/isaac_ros-dev"
check_cmd "isaac_ros_nitros package" ros2 pkg prefix isaac_ros_nitros
check_cmd "isaac_ros_managed_nitros package" ros2 pkg prefix isaac_ros_managed_nitros
check_cmd "isaac_ros_nitros_point_cloud_type package" ros2 pkg prefix isaac_ros_nitros_point_cloud_type

nitros_search_roots=(
  "${AMENT_PREFIX_PATH:-}"
  "/workspaces/isaac_ros-dev/install"
  "/opt/ros/humble"
)
nitros_found=false
for root_group in "${nitros_search_roots[@]}"; do
  IFS=':' read -ra roots <<<"${root_group}"
  for root in "${roots[@]}"; do
    [[ -n "${root}" && -d "${root}" ]] || continue
    if find "${root}" -maxdepth 8 -type f \( -name '*.hpp' -o -name '*.h' -o -name '*.cpp' \) \
      -print 2>/dev/null | xargs -r grep -l "NitrosPointCloud" >/dev/null 2>&1
    then
      nitros_found=true
      break 2
    fi
  done
done

if [[ "${nitros_found}" == "true" ]]; then
  echo "[nitros-env] PASS NitrosPointCloud symbol found"
else
  echo "[nitros-env] FAIL NitrosPointCloud symbol not found"
  missing=$((missing + 1))
fi

if [[ "${missing}" -gt 0 ]]; then
  echo "[nitros-env] NITROS navigation branch is unavailable; use NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker or legacy." >&2
  exit 3
fi

echo "[nitros-env] NITROS navigation branch environment looks available."

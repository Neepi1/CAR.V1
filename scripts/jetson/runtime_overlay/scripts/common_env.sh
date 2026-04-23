#!/usr/bin/env bash
set -euo pipefail

OVERLAY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-/workspaces/njrh-v3/workspace1}"
UPSTREAM_ROOT="${NJRH_UPSTREAM_ROOT:-/workspaces/isaac_ros-dev}"
UPSTREAM_HOST_ROOT="${NJRH_UPSTREAM_HOST_ROOT:-/home/nvidia/workspaces/isaac_ros-dev}"
UPSTREAM_SCRIPTS="${UPSTREAM_ROOT}/scripts"
UPSTREAM_WS="${UPSTREAM_ROOT}/ros2_ws"

export NJRH_OVERLAY_ROOT="$OVERLAY_ROOT"
export NJRH_PROJECT_ROOT="$PROJECT_ROOT"
export NJRH_UPSTREAM_ROOT="$UPSTREAM_ROOT"
export NJRH_UPSTREAM_HOST_ROOT="$UPSTREAM_HOST_ROOT"
export NJRH_MAPS_DIR="${NJRH_MAPS_DIR:-${UPSTREAM_ROOT}/maps}"
export NJRH_MAPS3D_DIR="${NJRH_MAPS3D_DIR:-${UPSTREAM_ROOT}/maps3d}"
export NJRH_RELEASE_ASSETS_DIR="${NJRH_RELEASE_ASSETS_DIR:-${UPSTREAM_ROOT}/maps_release}"
export NJRH_WAYPOINTS_DIR="${NJRH_WAYPOINTS_DIR:-${UPSTREAM_ROOT}/waypoints}"
export NJRH_RUNTIME_LOG_DIR="${NJRH_RUNTIME_LOG_DIR:-${OVERLAY_ROOT}/web_dashboard/runtime_logs}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${UPSTREAM_WS}/install/local_setup.bash" ]]; then
  source "${UPSTREAM_WS}/install/local_setup.bash"
fi
if [[ -f "${UPSTREAM_ROOT}/install/local_setup.bash" ]]; then
  source "${UPSTREAM_ROOT}/install/local_setup.bash"
fi
set -u

mkdir -p "${NJRH_RUNTIME_LOG_DIR}"

require_upstream_script() {
  local script_name="$1"
  local script_path="${UPSTREAM_SCRIPTS}/${script_name}"
  [[ -f "$script_path" ]] || {
    echo "[runtime-overlay] missing upstream script: ${script_path}" >&2
    exit 1
  }
  printf '%s\n' "$script_path"
}

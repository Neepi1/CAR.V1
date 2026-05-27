#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/common_env.sh"

UPSTREAM_ROOT="${NJRH_UPSTREAM_ROOT:-/workspaces/isaac_ros-dev}"
UPSTREAM_WEB="${UPSTREAM_ROOT}/web_dashboard"

export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

mkdir -p "${ROOT_DIR}/web_dashboard"

rm -f "${ROOT_DIR}/web_dashboard/dashboard_server.py" "${ROOT_DIR}/web_dashboard/index.html" "${ROOT_DIR}/web_dashboard/map2d_view.html" "${ROOT_DIR}/web_dashboard/lidar_view.html" "${ROOT_DIR}/web_dashboard/nvblox_view.html"
cp "${UPSTREAM_WEB}/dashboard_server.py" "${ROOT_DIR}/web_dashboard/dashboard_server.py"
cp "${UPSTREAM_WEB}/index.html" "${ROOT_DIR}/web_dashboard/index.html"
cp "${UPSTREAM_WEB}/map2d_view.html" "${ROOT_DIR}/web_dashboard/map2d_view.html"
cp "${UPSTREAM_WEB}/lidar_view.html" "${ROOT_DIR}/web_dashboard/lidar_view.html"
cp "${UPSTREAM_WEB}/nvblox_view.html" "${ROOT_DIR}/web_dashboard/nvblox_view.html"

for asset in slam2d_view.html chassis_control.html vendor; do
  if [[ -e "${ROOT_DIR}/web_dashboard/${asset}" ]]; then
    rm -rf "${ROOT_DIR}/web_dashboard/${asset}"
  fi
  ln -s "${UPSTREAM_WEB}/${asset}" "${ROOT_DIR}/web_dashboard/${asset}"
done

seed_runtime_dir() {
  local name="$1"
  local upstream_dir="$2"
  local local_dir="${ROOT_DIR}/${name}"
  if [[ -L "${local_dir}" ]]; then
    rm -f "${local_dir}"
  fi
  mkdir -p "${local_dir}"
  if [[ -d "${upstream_dir}" ]]; then
    cp -an "${upstream_dir}/." "${local_dir}/" 2>/dev/null || true
  fi
}

seed_runtime_dir "maps" "${UPSTREAM_ROOT}/maps"
seed_runtime_dir "maps3d" "${UPSTREAM_ROOT}/maps3d"
seed_runtime_dir "waypoints" "${UPSTREAM_ROOT}/waypoints"

mkdir -p "${ROOT_DIR}/web_dashboard/runtime_logs"
python3 "${ROOT_DIR}/scripts/patch_dashboard_runtime_v2.py" \
  --dashboard-server "${ROOT_DIR}/web_dashboard/dashboard_server.py" \
  --index-html "${ROOT_DIR}/web_dashboard/index.html" \
  --map2d-view-html "${ROOT_DIR}/web_dashboard/map2d_view.html" \
  --lidar-view-html "${ROOT_DIR}/web_dashboard/lidar_view.html" \
  --nvblox-view-html "${ROOT_DIR}/web_dashboard/nvblox_view.html"

ps -ef | grep '[d]ashboard_server.py' | awk '{print $2}' | xargs -r kill -INT 2>/dev/null || true
sleep 1
ps -ef | grep '[d]ashboard_server.py' | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true

exec python3 "${ROOT_DIR}/web_dashboard/dashboard_server.py"

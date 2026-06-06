#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
umask 0002

CONFIG_FILE="${ROBOT_API_SERVER_CONFIG:-${OVERLAY_ROOT}/config/robot_api_server.yaml}"
PORT="${ROBOT_API_SERVER_PORT:-8080}"

cd "${PROJECT_ROOT}"
set +u
source /opt/ros/humble/setup.bash

if [[ ! -x "${PROJECT_ROOT}/install/robot_api_server/lib/robot_api_server/robot_api_server_node" ]]; then
  colcon build --packages-select robot_interfaces robot_api_server --symlink-install
fi

source "${PROJECT_ROOT}/install/setup.bash"
set -u

if [[ -n "${ROBOT_API_TOKEN:-}" ]]; then
  njrh_exec_affined robot_api_server ros2 run robot_api_server robot_api_server_node --ros-args \
    --params-file "${CONFIG_FILE}" \
    -p port:="${PORT}" \
    -p api_token:="${ROBOT_API_TOKEN}"
fi

njrh_exec_affined robot_api_server ros2 run robot_api_server robot_api_server_node --ros-args \
  --params-file "${CONFIG_FILE}" \
  -p port:="${PORT}"

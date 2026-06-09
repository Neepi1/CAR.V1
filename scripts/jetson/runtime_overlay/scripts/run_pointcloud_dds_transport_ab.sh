#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

TRANSPORT="UDPv4"
RESTART_RUNTIME=false
EXECUTE=false

usage() {
  cat <<'EOF'
Usage: run_pointcloud_dds_transport_ab.sh --transport UDPv4|DEFAULT|LARGE_DATA [--restart-runtime] [--execute]

Print a controlled DDS transport A/B profile for pointcloud delivery diagnosis.
By default this script does not kill or restart production runtime.

Options:
  --transport VALUE   Candidate FASTDDS_BUILTIN_TRANSPORTS value.
  --restart-runtime  Print restart commands that must be applied before ROS participants start.
  --execute          Execute the restart command. Requires --restart-runtime.
  -h, --help         Show this help.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --transport)
      [[ "$#" -ge 2 ]] || { echo "[dds-ab] --transport requires a value" >&2; exit 2; }
      TRANSPORT="$2"
      shift 2
      ;;
    --restart-runtime)
      RESTART_RUNTIME=true
      shift
      ;;
    --execute)
      EXECUTE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[dds-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${TRANSPORT}" in
  UDPv4)
    TRANSPORT_VALUE="UDPv4"
    ;;
  DEFAULT)
    TRANSPORT_VALUE="DEFAULT"
    ;;
  LARGE_DATA)
    TRANSPORT_VALUE="LARGE_DATA?max_msg_size=${NJRH_FASTDDS_LARGE_DATA_MAX_MSG_SIZE:-10485760}&sockets_size=${NJRH_FASTDDS_LARGE_DATA_SOCKETS_SIZE:-10485760}&non_blocking=true"
    ;;
  *)
    echo "[dds-ab] unsupported transport: ${TRANSPORT}" >&2
    usage >&2
    exit 2
    ;;
esac

cat <<EOF
[dds-ab] Current environment:
[dds-ab]   RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}
[dds-ab]   FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}
[dds-ab]   NJRH_FASTDDS_PROFILE_ENABLED=${NJRH_FASTDDS_PROFILE_ENABLED:-unset}
[dds-ab]   FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}
[dds-ab]   FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-unset}
[dds-ab]   SKIP_DEFAULT_XML=${SKIP_DEFAULT_XML:-unset}
[dds-ab] Candidate profile:
[dds-ab]   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[dds-ab]   export FASTDDS_BUILTIN_TRANSPORTS='${TRANSPORT_VALUE}'
[dds-ab]   export NJRH_FASTDDS_PROFILE_ENABLED='${NJRH_FASTDDS_PROFILE_ENABLED:-true}'
[dds-ab]
[dds-ab] IMPORTANT: FASTDDS_BUILTIN_TRANSPORTS must be set before every ROS participant starts.
[dds-ab] Do not compare profiles by changing this variable while Nav2/driver/perception are already running.
EOF

cat <<'EOF'
[dds-ab] Suggested validation sequence after starting a candidate profile:
[dds-ab]   bash scripts/jetson/runtime_overlay/scripts/inspect_pointcloud_subscribers.sh
[dds-ab]   bash scripts/jetson/runtime_overlay/scripts/verify_lidar_trunk_jitter.sh
[dds-ab]   bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_delivery_matrix.sh
[dds-ab] Optional CLI sample for CASE_G only, run manually because it creates a full-density temporary subscriber:
[dds-ab]   timeout 8 ros2 topic hz /lidar_points
[dds-ab]   NJRH_VERIFY_MATRIX_LIDAR_POINTS_CLI_HZ=<average_rate> bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_delivery_matrix.sh
EOF

if [[ "${RESTART_RUNTIME}" != "true" ]]; then
  echo "[dds-ab] Runtime restart not requested; no production process was stopped."
  exit 0
fi

if [[ "${EXECUTE}" != "true" ]]; then
  cat <<EOF
[dds-ab] --restart-runtime requested, but --execute was not provided.
[dds-ab] Restart from the host shell with the candidate environment, for example:
[dds-ab]   cd '${PROJECT_ROOT}'
[dds-ab]   env RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS='${TRANSPORT_VALUE}' NJRH_FASTDDS_PROFILE_ENABLED='${NJRH_FASTDDS_PROFILE_ENABLED:-true}' bash scripts/jetson/njrh_container.sh start-runtime
[dds-ab] No runtime process was stopped.
EOF
  exit 0
fi

if [[ ! -x "${PROJECT_ROOT}/scripts/jetson/njrh_container.sh" && ! -f "${PROJECT_ROOT}/scripts/jetson/njrh_container.sh" ]]; then
  echo "[dds-ab] restart helper missing: ${PROJECT_ROOT}/scripts/jetson/njrh_container.sh" >&2
  exit 1
fi

echo "[dds-ab] Executing explicit runtime restart with FASTDDS_BUILTIN_TRANSPORTS=${TRANSPORT_VALUE}" >&2
cd "${PROJECT_ROOT}"
env \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  FASTDDS_BUILTIN_TRANSPORTS="${TRANSPORT_VALUE}" \
  NJRH_FASTDDS_PROFILE_ENABLED="${NJRH_FASTDDS_PROFILE_ENABLED:-true}" \
  bash scripts/jetson/njrh_container.sh start-runtime

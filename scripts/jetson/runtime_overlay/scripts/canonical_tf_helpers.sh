#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/runtime_health_helpers.sh"

canonical_helper_pids=()

reuse_common_services_enabled() {
  [[ "${NJRH_REUSE_COMMON_SERVICES:-true}" == "true" ]]
}

force_restart_canonical_tf_enabled() {
  [[ "${NJRH_FORCE_RESTART_CANONICAL_TF:-false}" == "true" ]]
}

forget_canonical_helper_pid() {
  local pid_to_forget="$1"
  local kept_pids=()
  local helper_pid
  for helper_pid in "${canonical_helper_pids[@]:-}"; do
    [[ -n "${helper_pid}" ]] || continue
    if [[ "${helper_pid}" != "${pid_to_forget}" ]]; then
      kept_pids+=("${helper_pid}")
    fi
  done
  canonical_helper_pids=("${kept_pids[@]}")
}

canonical_helper_process_pattern() {
  local helper_name="$1"
  case "${helper_name}" in
    ranger_chassis*)
      printf '%s\n' "ranger_base_node.*port_name:=${CAN_IFACE:-can0}|ros2 run ranger_base ranger_base_node.*port_name:=${CAN_IFACE:-can0}"
      ;;
    robot_description*|robot_description_static_tf*)
      printf '%s\n' "robot_description_static_tf_node"
      ;;
    local_state*|robot_local_state*)
      printf '%s\n' "${NJRH_OVERLAY_ROOT}/scripts/run_local_state.sh|__node:=wheel_odom_ekf_input|robot_local_state/imu_gyro_bias_filter_node|imu_gyro_bias_filter_node --ros-args|__node:=imu_gyro_bias_filter|robot_localization/ekf_node|ekf_node --ros-args.*__node:=robot_local_state|robot_local_state/local_state_node|local_state_node --ros-args|robot_fastlio_mapping/fastlio_odom_bridge_node|fastlio_odom_bridge_node --ros-args"
      ;;
    robot_localization_bridge*)
      printf '%s\n' "robot_localization_bridge/localization_bridge_node|localization_bridge_node --ros-args"
      ;;
    *)
      return 1
      ;;
  esac
}

canonical_process_running() {
  local pattern="$1"
  pgrep -f "${pattern}" >/dev/null 2>&1
}

local_state_node_process_running() {
  pgrep -f "robot_local_state/local_state_node|local_state_node --ros-args" >/dev/null 2>&1
}

fastlio_odom_bridge_process_running() {
  pgrep -f "robot_fastlio_mapping/fastlio_odom_bridge_node|fastlio_odom_bridge_node --ros-args" >/dev/null 2>&1
}

ekf_local_state_process_running() {
  pgrep -f "robot_localization/ekf_node|ekf_node --ros-args.*__node:=robot_local_state" >/dev/null 2>&1
}

local_state_required_processes_running() {
  local mode="${LOCAL_STATE_MODE:-${NAV_LOCAL_STATE_MODE:-${NJRH_NAV_LOCAL_STATE_MODE:-ekf}}}"
  case "${mode}" in
    fastlio)
      fastlio_odom_bridge_process_running && local_state_node_process_running
      ;;
    passthrough|legacy)
      local_state_node_process_running
      ;;
    *)
      ekf_local_state_process_running
      ;;
  esac
}

wait_for_local_state_required_processes() {
  local timeout_sec="${1:-${LOCAL_STATE_PROCESS_START_TIMEOUT_SEC:-8}}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if local_state_required_processes_running; then
      return 0
    fi
    sleep 0.2
  done
  local_state_required_processes_running
}

local_state_endpoint_ready() {
  local timeout_sec="${1:-8}"
  local mode="${LOCAL_STATE_MODE:-${NAV_LOCAL_STATE_MODE:-${NJRH_NAV_LOCAL_STATE_MODE:-ekf}}}"
  if runtime_health_available; then
    if [[ "${mode}" == "fastlio" ]]; then
      runtime_health_check "local_state_fastlio_endpoint" >/dev/null 2>&1 && return 0
    else
      runtime_health_check "local_state_endpoint" >/dev/null 2>&1 && return 0
    fi
  fi
  runtime_readiness_probe local-state-endpoint "${timeout_sec}" "${mode}"
}

canonical_helper_ready() {
  # Startup path must not create ROS graph probes. If the helper process exists,
  # lifecycle/API layers decide whether it is usable.
  local helper_name="$1"
  case "${helper_name}" in
    local_state*|robot_local_state*)
      local_state_required_processes_running
      ;;
    *)
      return 0
      ;;
  esac
}

canonical_helper_start_ready() {
  local helper_name="$1"
  case "${helper_name}" in
    local_state*|robot_local_state*)
      wait_for_local_state_required_processes "${LOCAL_STATE_PROCESS_START_TIMEOUT_SEC:-8}"
      ;;
    *)
      # A just-started helper only has to stay alive. Topic/TF/service checks
      # are diagnostics, not startup gates.
      return 0
      ;;
  esac
}

kill_canonical_pattern() {
  local pattern="$1"
  pkill -INT -f "$pattern" 2>/dev/null || true
  sleep 1
  pkill -9 -f "$pattern" 2>/dev/null || true
}

cleanup_stale_canonical_helper() {
  local helper_name="$1"
  local helper_pattern="${2:-}"
  case "${helper_name}" in
    local_state*|robot_local_state*)
      [[ -n "${helper_pattern}" ]] && kill_canonical_pattern "${helper_pattern}"
      kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_local_state.sh"
      kill_canonical_pattern "robot_fastlio_mapping/fastlio_odom_bridge_node"
      kill_canonical_pattern "fastlio_odom_bridge_node --ros-args"
      kill_canonical_pattern "robot_fastlio_mapping/fastlio_odom_bridge_node.py"
      kill_canonical_pattern "fastlio_odom_bridge_node.py --ros-args"
      ;;
    *)
      [[ -n "${helper_pattern}" ]] && kill_canonical_pattern "${helper_pattern}"
      ;;
  esac
}

stop_existing_canonical_tf_publishers() {
  if ! force_restart_canonical_tf_enabled; then
    echo "[runtime-overlay] reusing canonical TF/local-state helpers; set NJRH_FORCE_RESTART_CANONICAL_TF=true to restart them" >&2
    return 0
  fi
  kill_canonical_pattern "hesai_lidar_state_publisher"
  kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_robot_description.sh"
  kill_canonical_pattern "robot_description_static_tf_node"
  kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_local_state.sh"
  kill_canonical_pattern "__node:=wheel_odom_ekf_input"
  kill_canonical_pattern "robot_localization/ekf_node"
  kill_canonical_pattern "ekf_node --ros-args.*__node:=robot_local_state"
  kill_canonical_pattern "robot_local_state/local_state_node"
  kill_canonical_pattern "local_state_node --ros-args"
  kill_canonical_pattern "robot_local_state/imu_gyro_bias_filter_node"
  kill_canonical_pattern "imu_gyro_bias_filter_node --ros-args"
  kill_canonical_pattern "__node:=imu_gyro_bias_filter"
  kill_canonical_pattern "robot_fastlio_mapping/fastlio_odom_bridge_node"
  kill_canonical_pattern "fastlio_odom_bridge_node --ros-args"
  kill_canonical_pattern "robot_fastlio_mapping/fastlio_odom_bridge_node.py"
  kill_canonical_pattern "fastlio_odom_bridge_node.py --ros-args"
  kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_localization_bridge.sh"
  kill_canonical_pattern "robot_localization_bridge/localization_bridge_node"
  kill_canonical_pattern "localization_bridge_node --ros-args"
  kill_canonical_pattern "map_to_odom_tf_bridge"
}

start_canonical_helper() {
  local helper_name="$1"
  shift
  local helper_log="${NJRH_RUNTIME_LOG_DIR}/${helper_name}.log"
  local helper_pattern=""
  if helper_pattern="$(canonical_helper_process_pattern "${helper_name}")"; then
    if reuse_common_services_enabled && canonical_process_running "${helper_pattern}"; then
      if canonical_helper_ready "${helper_name}"; then
        echo "[runtime-overlay] reusing existing ${helper_name}; pattern=${helper_pattern}" >&2
        return 0
      fi
      echo "[runtime-overlay] existing ${helper_name} process will be restarted" >&2
      cleanup_stale_canonical_helper "${helper_name}" "${helper_pattern}"
    fi
  fi
  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  if [[ -e "${helper_log}" && ! -w "${helper_log}" ]]; then
    rm -f "${helper_log}" 2>/dev/null || {
      echo "[runtime-overlay] helper log is not writable and could not be removed: ${helper_log}" >&2
      return 1
    }
  fi
  : >"${helper_log}" 2>/dev/null || {
    echo "[runtime-overlay] helper log is not writable: ${helper_log}" >&2
    return 1
  }
  echo "[runtime-overlay] starting ${helper_name}" >&2
  "$@" >>"${helper_log}" 2>&1 &
  local helper_pid=$!
  canonical_helper_pids+=("${helper_pid}")
  sleep 1
  if ! kill -0 "${helper_pid}" 2>/dev/null; then
    echo "[runtime-overlay] helper failed to stay alive: ${helper_name}. Check ${helper_log}" >&2
    return 1
  fi
  if ! canonical_helper_start_ready "${helper_name}"; then
    echo "[runtime-overlay] helper child process did not become ready: ${helper_name}. Check ${helper_log}" >&2
    kill -INT "${helper_pid}" 2>/dev/null || true
    wait "${helper_pid}" 2>/dev/null || true
    return 1
  fi
  # Canonical TF/local-state helpers are common-service dependencies. Keep a
  # helper process alive even when a mode wrapper exits on a later
  # Nav2/localizer startup failure.
  disown "${helper_pid}" 2>/dev/null || true
  forget_canonical_helper_pid "${helper_pid}"
  echo "[runtime-overlay] helper launched: ${helper_name} (pid=${helper_pid}, cleanup_owner=common)" >&2
}

cleanup_canonical_helpers() {
  local helper_pid
  for helper_pid in "${canonical_helper_pids[@]:-}"; do
    kill -INT "${helper_pid}" 2>/dev/null || true
  done
  sleep 1
  for helper_pid in "${canonical_helper_pids[@]:-}"; do
    kill -9 "${helper_pid}" 2>/dev/null || true
  done
  canonical_helper_pids=()
}

require_can_interface_up() {
  local can_interface="${CAN_INTERFACE:-can0}"
  local operstate_file="/sys/class/net/${can_interface}/operstate"
  [[ -r "${operstate_file}" ]] || {
    echo "[runtime-overlay] CAN interface not found: ${can_interface}" >&2
    return 1
  }

  local operstate
  operstate="$(tr -d '[:space:]' < "${operstate_file}")"
  if [[ "${operstate}" != "up" ]]; then
    echo "[runtime-overlay] CAN interface ${can_interface} is ${operstate}." >&2
    echo "[runtime-overlay] Bring it up on the Jetson host first:" >&2
    echo "  sudo bash ${NJRH_UPSTREAM_HOST_ROOT}/scripts/bringup_ranger_can_host.sh" >&2
    return 1
  fi
}

wait_for_topic_message() {
  local topic_name="$1"
  local timeout_sec="${2:-10}"
  runtime_readiness_probe topic "${topic_name}" "${timeout_sec}"
}

wait_for_fresh_header_topic_message() {
  local topic_name="$1"
  local timeout_sec="${2:-10}"
  local max_age_sec="${3:-1.0}"
  local max_future_sec="${4:-0.25}"
  runtime_readiness_probe fresh-header-topic "${topic_name}" "${timeout_sec}" "${max_age_sec}" "${max_future_sec}"
}

wait_for_topic_publisher() {
  local topic_name="$1"
  local timeout_sec="${2:-10}"
  runtime_readiness_probe topic-publisher "${topic_name}" "${timeout_sec}"
}

wait_for_node_name() {
  local expected_node="$1"
  local timeout_sec="${2:-10}"
  if [[ "${expected_node}" != /* ]]; then
    expected_node="/${expected_node}"
  fi
  runtime_readiness_probe node "${expected_node}" "${timeout_sec}"
}

wait_for_topic_publisher_from_node() {
  local topic_name="$1"
  local node_name="$2"
  local timeout_sec="${3:-10}"
  runtime_readiness_probe publisher-from-node "${topic_name}" "${node_name}" "${timeout_sec}"
}

wait_for_service_name() {
  local service_name="$1"
  local timeout_sec="${2:-10}"
  runtime_readiness_probe service "${service_name}" "${timeout_sec}"
}

localization_bridge_graph_ready() {
  local timeout_sec="${1:-8}"
  wait_for_node_name "/robot_localization_bridge" "${timeout_sec}" &&
    wait_for_topic_publisher_from_node "/tf" "robot_localization_bridge" "${timeout_sec}" &&
    wait_for_service_name "/robot_localization_bridge/force_accept_next_localization" "${timeout_sec}"
}

localization_bridge_runtime_ready() {
  local timeout_sec="${1:-3}"
  localization_bridge_graph_ready "${timeout_sec}" &&
    wait_for_tf_edge "map" "odom" "${timeout_sec}" >/dev/null 2>&1
}

wait_for_tf_edge() {
  local parent_frame="$1"
  local child_frame="$2"
  local timeout_sec="${3:-10}"
  runtime_readiness_probe tf "${parent_frame}" "${child_frame}" "${timeout_sec}"
}

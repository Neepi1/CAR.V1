#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/commercial_runtime_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile

export NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
LAUNCH_FILE="${NJRH_PROJECT_ROOT}/src/robot_bringup/launch/standard_navigation.launch.py"
map_server_ready_timeout_sec="${NJRH_NAV_MAP_SERVER_READY_TIMEOUT:-75}"
global_costmap_ready_timeout_sec="${NJRH_NAV_GLOBAL_COSTMAP_READY_TIMEOUT:-90}"

controller_server_pids() {
  ps -eo pid=,args= | awk '
    /controller_server/ &&
    (/nav2_controller/ || /__node:=controller_server/ || /\/controller_server/) &&
    $0 !~ /ros2 lifecycle|get \/controller_server|ros2 param|awk/ {
      print $1
    }
  '
}

read_proc_cpuset() {
  local pid="$1"
  awk -F: '/^Cpus_allowed_list:/ {gsub(/^[ \t]+/, "", $2); print $2; exit}' "/proc/${pid}/status" 2>/dev/null || true
}

controller_threads_match_cpuset() {
  local pid="$1"
  local expected="$2"
  local task_path
  local tid
  local allowed
  for task_path in /proc/"${pid}"/task/*; do
    [[ -e "${task_path}" ]] || continue
    tid="${task_path##*/}"
    allowed="$(awk -F: '/^Cpus_allowed_list:/ {gsub(/^[ \t]+/, "", $2); print $2; exit}' "${task_path}/status" 2>/dev/null || true)"
    if [[ "${allowed}" != "${expected}" ]]; then
      echo "[runtime-overlay] controller_server affinity mismatch tid=${tid} expected=${expected} actual=${allowed:-missing}" >&2
      return 1
    fi
  done
  return 0
}

wait_for_controller_server_affinity() {
  local expected="${NJRH_CPUSET_CONTROLLER_SERVER:-}"
  local profile="${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current}"
  local timeout_sec="${NJRH_NAV2_CONTROLLER_AFFINITY_CHECK_TIMEOUT_SEC:-15}"
  local deadline=$((SECONDS + timeout_sec))
  local pid=""
  local allowed=""

  if ! njrh_affinity_enabled; then
    echo "[runtime-overlay] controller_server affinity check skipped because CPU affinity is disabled" >&2
    return 0
  fi
  if [[ -z "${expected}" ]]; then
    echo "[runtime-overlay] controller_server expected CPU set is empty for profile=${profile}" >&2
    return 1
  fi

  while (( SECONDS <= deadline )); do
    pid="$(controller_server_pids | tail -n 1 || true)"
    if [[ -n "${pid}" && -r "/proc/${pid}/status" ]]; then
      allowed="$(read_proc_cpuset "${pid}")"
      if [[ "${allowed}" == "${expected}" ]] && controller_threads_match_cpuset "${pid}" "${expected}"; then
        echo "[runtime-overlay] controller_server cpu profile=${profile} cpuset=${expected} pid=${pid} allowed=${allowed}" >&2
        return 0
      fi
      echo "[runtime-overlay] waiting for controller_server affinity profile=${profile} expected=${expected} pid=${pid} actual=${allowed:-missing}" >&2
    fi
    sleep 0.5
  done

  echo "[runtime-overlay] controller_server affinity check failed profile=${profile} expected=${expected} pid=${pid:-missing} actual=${allowed:-missing}" >&2
  return 1
}

if [[ -n "${NJRH_FLOOR_ID:-}" || -n "${NAV2_FLOOR_ID:-}" ]]; then
  resolve_floor_assets "${NJRH_BUILDING_ID:-${NAV2_BUILDING_ID:-building_1}}" "${NJRH_FLOOR_ID:-${NAV2_FLOOR_ID:-}}"
fi

[[ -f "${LAUNCH_FILE}" ]] || {
  echo "[runtime-overlay] missing repository launch file: ${LAUNCH_FILE}" >&2
  exit 1
}

standard_nav_stack_ready() {
  [[ "${NJRH_NAV2_REUSE_READY_STACK:-false}" == "true" ]] || return 1
  pgrep -f "standard_navigation.launch.py|__node:=controller_server|__node:=bt_navigator" >/dev/null 2>&1
}

if standard_nav_stack_ready; then
  echo "[runtime-overlay] standard Nav2 navigation stack already ready; reusing existing stack" >&2
  while true; do
    sleep 2
  done
fi

stop_existing_overlay_nav_helpers
if [[ "${NJRH_SKIP_PRESTART_NAV2_STOP:-false}" != "true" ]]; then
  stop_existing_standard_nav_stack
else
  echo "[runtime-overlay] skipping pre-start Nav2 stop because NJRH_SKIP_PRESTART_NAV2_STOP=true" >&2
fi

echo "[runtime-overlay] starting Nav2 without blocking map/topic/TF readiness probes" >&2
echo "[runtime-overlay] Nav2 controller CPU profile=${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current} cpuset=${NJRH_CPUSET_CONTROLLER_SERVER:-unset}" >&2

ensure_costmap_filter_masks() {
  local generator="${SCRIPT_DIR}/ensure_costmap_filter_masks.py"
  [[ -f "${generator}" ]] || {
    echo "[runtime-overlay] missing costmap filter mask generator: ${generator}" >&2
    return 1
  }

  local source_keepout="${NAV2_KEEP_OUT_MASK_YAML:-}"
  local source_speed="${NAV2_SPEED_MASK_YAML:-}"
  local source_binary="${NAV2_BINARY_MASK_YAML:-}"
  local runtime_key="${NJRH_BUILDING_ID:-building_1}_${NJRH_FLOOR_ID:-floor}_$$"
  runtime_key="${runtime_key//[^A-Za-z0-9_.-]/_}"
  local runtime_dir="${NJRH_OVERLAY_ROOT}/filters/runtime_nav2/${runtime_key}"

  local args=(--output-dir "${runtime_dir}")
  if [[ -n "${NAV2_MAP_YAML:-}" && -f "${NAV2_MAP_YAML}" ]]; then
    args+=(--nav-yaml "${NAV2_MAP_YAML}")
  fi
  if [[ -n "${source_keepout}" ]]; then
    args+=(--keepout-yaml "${source_keepout}")
  fi
  if [[ -n "${source_speed}" ]]; then
    args+=(--speed-yaml "${source_speed}")
  fi
  if [[ -n "${source_binary}" ]]; then
    args+=(--binary-yaml "${source_binary}")
  fi
  args+=(--stable-wait-sec "${NJRH_COSTMAP_FILTER_MASK_STABLE_WAIT_SEC:-3.0}")

  eval "$(python3 "${generator}" "${args[@]}")"
  export NAV2_KEEP_OUT_MASK_YAML NAV2_SPEED_MASK_YAML NAV2_BINARY_MASK_YAML
  [[ -s "${NAV2_KEEP_OUT_MASK_YAML}" && -s "${NAV2_SPEED_MASK_YAML}" ]] || return 1
  [[ "${NAV2_KEEP_OUT_MASK_YAML}" == "${runtime_dir}/"* ]] || return 1
  [[ "${NAV2_SPEED_MASK_YAML}" == "${runtime_dir}/"* ]] || return 1
}

ensure_costmap_filter_masks || {
  echo "[runtime-overlay] failed to prepare costmap filter masks" >&2
  exit 1
}

nav_pid=""
nav_exit_code=0
cleanup_started=0

cleanup() {
  if [[ "${cleanup_started}" -eq 1 ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  if [[ -n "${nav_pid}" ]]; then
    kill -INT "${nav_pid}" 2>/dev/null || true
    wait "${nav_pid}" 2>/dev/null || true
  fi
  stop_existing_standard_nav_stack
  cleanup_overlay_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

ensure_resident_overlay_helper_process() {
  local helper_name="$1"
  local label="$2"
  shift 2
  local helper_pattern=""
  helper_pattern="$(helper_process_pattern "${helper_name}" 2>/dev/null || true)"
  if [[ -n "${helper_pattern}" ]] && helper_process_running "${helper_pattern}"; then
    echo "[runtime-overlay] resident ${label} process exists; skipping readiness probe" >&2
    return 0
  fi
  echo "[runtime-overlay] resident ${label} process not found; starting it without readiness probe" >&2
  "$@" >>"${NJRH_RUNTIME_LOG_DIR}/${helper_name}.log" 2>&1 &
  helper_pids+=("$!")
  sleep "${NJRH_NAV_HELPER_START_SETTLE_SEC:-0.5}"
}

ensure_resident_overlay_helper_process "floor_manager" "floor_manager" bash "${SCRIPT_DIR}/run_floor_manager.sh"
ensure_resident_overlay_helper_process "robot_safety" "robot_safety" bash "${SCRIPT_DIR}/run_robot_safety.sh"
ensure_resident_overlay_helper_process "ranger_mini3_mode_controller" "ranger_mini3_mode_controller" bash "${SCRIPT_DIR}/run_ranger_mini3_mode_controller.sh"
if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]]; then
  echo "[runtime-overlay] local_perception is owned by pointcloud accel profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}; skipping standalone local_perception helper" >&2
elif [[ "${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER:-false}" == "true" ]]; then
  echo "[runtime-overlay] local_perception is owned by pointcloud_perception_pipeline; skipping standalone local_perception helper" >&2
else
  ensure_resident_overlay_helper_process "local_perception" "local_perception" bash "${SCRIPT_DIR}/run_local_perception.sh"
fi

ros2 launch "${LAUNCH_FILE}" \
  use_sim_time:=false \
  autostart:=true \
  params_file:="${NAV2_PARAMS_FILE}" \
  keepout_mask_yaml:="${NAV2_KEEP_OUT_MASK_YAML}" \
  speed_mask_yaml:="${NAV2_SPEED_MASK_YAML}" &
nav_pid=$!
sleep "${NJRH_NAV2_LAUNCH_SETTLE_SEC:-3}"
if ! kill -0 "${nav_pid}" 2>/dev/null; then
  wait "${nav_pid}" || nav_exit_code=$?
  echo "[runtime-overlay] Nav2 launch exited during initial settle with ${nav_exit_code}" >&2
  exit "${nav_exit_code}"
fi
wait_for_controller_server_affinity || exit 1
echo "[runtime-overlay] Nav2 launch process is running; blocking readiness probes are disabled" >&2

wait "${nav_pid}" || nav_exit_code=$?
exit "${nav_exit_code}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/commercial_runtime_helpers.sh"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile

export NJRH_NAV_LIFECYCLE_START_DELAY_SEC="${NJRH_NAV_LIFECYCLE_START_DELAY_SEC:-2.0}"
export NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
export NJRH_NAV2_HOLD_READY_FILE="${NJRH_NAV2_HOLD_READY_FILE:-/tmp/njrh_nav2_launch_hold_ready.env}"
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

nav2_external_lifecycle_bringup_enabled() {
  [[ "${NJRH_NAV2_EXTERNAL_LIFECYCLE_BRINGUP:-true}" == "true" ]]
}

nav2_lifecycle_hold_enabled() {
  [[ "${NJRH_NAV2_LIFECYCLE_HOLD:-false}" == "true" ]]
}

nav2_speed_filter_enabled() {
  [[ "${NJRH_ENABLE_SPEED_FILTER:-false}" =~ ^(1|true|TRUE|yes|YES|on|ON)$ ]]
}

write_nav2_hold_ready_status() {
  local controller_pid=""
  local tmp_path="${NJRH_NAV2_HOLD_READY_FILE}.$$"
  controller_pid="$(controller_server_pids | tail -n 1 || true)"
  {
    printf 'NAV2_HOLD_READY="true"\n'
    printf 'NAV2_HOLD_READY_STAMP_SEC="%s"\n' "$(date +%s)"
    printf 'NAV2_HOLD_READY_WRAPPER_PID="%s"\n' "$$"
    printf 'NAV2_HOLD_READY_BASHPID="%s"\n' "${BASHPID:-$$}"
    printf 'NAV2_HOLD_READY_LAUNCH_PID="%s"\n' "${nav_pid:-}"
    printf 'NAV2_HOLD_READY_CONTROLLER_PID="%s"\n' "${controller_pid}"
  } >"${tmp_path}"
  mv -f "${tmp_path}" "${NJRH_NAV2_HOLD_READY_FILE}"
}

start_navigation_lifecycle_with_nav2_util() {
  local timeout_sec="${NJRH_NAV2_LIFECYCLE_BRINGUP_TIMEOUT_SEC:-180}"
  local nodes=(
    planner_server
    controller_server
    velocity_smoother
    collision_monitor
    bt_navigator
    smoother_server
    behavior_server
    waypoint_follower
  )
  echo "[runtime-overlay] starting Nav2 core lifecycle with nav2_util lifecycle_bringup timeout=${timeout_sec}s" >&2
  timeout --kill-after="${NJRH_NAV2_LIFECYCLE_BRINGUP_KILL_AFTER_SEC:-5}" "${timeout_sec}" \
    /opt/ros/humble/lib/nav2_util/lifecycle_bringup "${nodes[@]}" &
  nav_lifecycle_bringup_pid=$!
  if wait "${nav_lifecycle_bringup_pid}"; then
    nav_lifecycle_bringup_pid=""
    echo "[runtime-overlay] lifecycle_manager_navigation external lifecycle_bringup: Managed nodes are active" >&2
    return 0
  fi
  nav_lifecycle_bringup_pid=""
  echo "[runtime-overlay] Nav2 core lifecycle_bringup failed or timed out" >&2
  return 1
}

start_navigation_lifecycle_with_repo_sequence() {
  local timeout_sec="${NJRH_NAV2_LIFECYCLE_BRINGUP_TIMEOUT_SEC:-180}"
  local node_timeout="${NJRH_NAV2_LIFECYCLE_NODE_TIMEOUT_SEC:-60}"
  local kill_after="${NJRH_NAV2_LIFECYCLE_BRINGUP_KILL_AFTER_SEC:-5}"
  local core_nodes=(
    planner_server
    controller_server
    velocity_smoother
    collision_monitor
    bt_navigator
  )
  local background_nodes=(
    smoother_server
    behavior_server
    waypoint_follower
  )
  local sequence_args=(--per-node-timeout-sec "${node_timeout}")
  if [[ "${NJRH_NAV2_LIFECYCLE_TRUST_CHANGE_STATE_RESPONSE:-true}" == "true" ]]; then
    sequence_args+=(--trust-change-state-response)
  fi
  if [[ "${NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false}" == "true" ]]; then
    sequence_args+=(--configure-all-before-activate)
  fi

  echo "[runtime-overlay] starting Nav2 point-navigation core lifecycle with repo sequence timeout=${timeout_sec}s nodes=${core_nodes[*]}" >&2
  timeout --kill-after="${kill_after}" "${timeout_sec}" \
    python3 "${SCRIPT_DIR}/nav2_lifecycle_sequence.py" \
      "${sequence_args[@]}" \
      "${core_nodes[@]}" || {
        echo "[runtime-overlay] Nav2 point-navigation core lifecycle sequence failed or timed out" >&2
        return 1
      }
  echo "[runtime-overlay] lifecycle_manager_navigation external repo lifecycle sequence: point-navigation core nodes are active" >&2

  if [[ "${NJRH_NAV2_BACKGROUND_NONCRITICAL_LIFECYCLE:-true}" == "true" ]]; then
    echo "[runtime-overlay] starting Nav2 noncritical lifecycle sequence in background nodes=${background_nodes[*]}" >&2
    (
      timeout --kill-after="${kill_after}" "${timeout_sec}" \
        python3 "${SCRIPT_DIR}/nav2_lifecycle_sequence.py" \
          "${sequence_args[@]}" \
          "${background_nodes[@]}"
    ) &
    nav_lifecycle_bringup_pid=$!
  fi
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
rm -f "${NJRH_NAV2_HOLD_READY_FILE}" 2>/dev/null || true
if [[ "${NJRH_SKIP_PRESTART_NAV2_STOP:-false}" != "true" ]]; then
  stop_existing_standard_nav_stack
else
  echo "[runtime-overlay] skipping pre-start Nav2 stop because NJRH_SKIP_PRESTART_NAV2_STOP=true" >&2
fi

echo "[runtime-overlay] starting Nav2 without blocking map/topic/TF readiness probes" >&2
echo "[runtime-overlay] Nav2 controller CPU profile=${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current} cpuset=${NJRH_CPUSET_CONTROLLER_SERVER:-unset}" >&2
echo "[runtime-overlay] Nav2 speed filter enabled=${NJRH_ENABLE_SPEED_FILTER:-false}" >&2
if nav2_lifecycle_hold_enabled; then
  navigation_lifecycle_autostart="false"
  echo "[runtime-overlay] Nav2 core lifecycle autostart disabled; resident runtime is holding lifecycle activation" >&2
elif nav2_external_lifecycle_bringup_enabled; then
  navigation_lifecycle_autostart="false"
  echo "[runtime-overlay] Nav2 core lifecycle autostart disabled; lifecycle_bringup will manage core nodes" >&2
else
  navigation_lifecycle_autostart="true"
fi

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
  args+=(--stable-wait-sec "${NJRH_COSTMAP_FILTER_MASK_STABLE_WAIT_SEC:-0.5}")

  eval "$(python3 "${generator}" "${args[@]}")"
  export NAV2_KEEP_OUT_MASK_YAML NAV2_SPEED_MASK_YAML NAV2_BINARY_MASK_YAML
  [[ -s "${NAV2_KEEP_OUT_MASK_YAML}" ]] || return 1
  [[ "${NAV2_KEEP_OUT_MASK_YAML}" == "${runtime_dir}/"* ]] || return 1
  if nav2_speed_filter_enabled; then
    [[ -s "${NAV2_SPEED_MASK_YAML}" ]] || return 1
    [[ "${NAV2_SPEED_MASK_YAML}" == "${runtime_dir}/"* ]] || return 1
  fi
}

costmap_filter_mask_is_neutral() {
  local mask_yaml="$1"
  python3 - "${mask_yaml}" <<'PY'
import re
import sys
from pathlib import Path

yaml_path = Path(sys.argv[1])
try:
    text = yaml_path.read_text(encoding="utf-8", errors="ignore")
except OSError:
    raise SystemExit(2)

image_ref = ""
for line in text.splitlines():
    if re.match(r"^\s*image\s*:", line):
        image_ref = line.split(":", 1)[1].split("#", 1)[0].strip().strip("\"'")
        break
if not image_ref:
    raise SystemExit(2)
image_path = Path(image_ref)
if not image_path.is_absolute():
    image_path = yaml_path.parent / image_path

try:
    data = image_path.read_bytes()
except OSError:
    raise SystemExit(2)

tokens = []
index = 0
while index < len(data) and len(tokens) < 4:
    while index < len(data) and data[index:index + 1].isspace():
        index += 1
    if index < len(data) and data[index:index + 1] == b"#":
        while index < len(data) and data[index:index + 1] not in (b"\n", b"\r"):
            index += 1
        continue
    start = index
    while index < len(data) and not data[index:index + 1].isspace():
        index += 1
    if start != index:
        tokens.append(data[start:index])

if len(tokens) < 4 or tokens[0] not in (b"P5", b"P2"):
    raise SystemExit(2)
width = int(tokens[1])
height = int(tokens[2])
count = width * height
while index < len(data) and data[index:index + 1].isspace():
    index += 1

if tokens[0] == b"P5":
    pixels = data[index:index + count]
    if len(pixels) < count:
        raise SystemExit(2)
    neutral = all(value >= 250 for value in pixels)
else:
    values = re.findall(rb"\d+", data[index:])
    if len(values) < count:
        raise SystemExit(2)
    neutral = all(int(value) >= 250 for value in values[:count])

raise SystemExit(0 if neutral else 1)
PY
}

disable_neutral_costmap_filters_if_needed() {
  [[ "${NJRH_NAV2_DISABLE_NEUTRAL_COSTMAP_FILTERS:-true}" == "true" ]] || return 0
  [[ -n "${NAV2_KEEP_OUT_MASK_YAML:-}" && -f "${NAV2_KEEP_OUT_MASK_YAML}" ]] || return 0

  if ! costmap_filter_mask_is_neutral "${NAV2_KEEP_OUT_MASK_YAML}"; then
    echo "[runtime-overlay] keepout filter mask has active cells; keeping Nav2 global costmap filters enabled" >&2
    return 0
  fi

  local runtime_dir
  runtime_dir="$(dirname "${NAV2_KEEP_OUT_MASK_YAML}")"
  local runtime_params="${runtime_dir}/nav2.neutral_keepout_disabled.yaml"
  python3 - "${NAV2_PARAMS_FILE}" "${runtime_params}" <<'PY'
import sys
from pathlib import Path

source = Path(sys.argv[1])
target = Path(sys.argv[2])
lines = source.read_text(encoding="utf-8").splitlines()
out = []
section = []
disabled_filter_manager = False
removed_global_keepout_filter_list = False
disabled_keepout_plugin = False

for line in lines:
    if line and not line.startswith((" ", "\t")):
        section = [line.split(":", 1)[0]]
    stripped = line.strip()

    if section == ["lifecycle_manager_costmap_filters"] and stripped == "autostart: true":
        out.append(line.replace("true", "false", 1))
        disabled_filter_manager = True
        continue

    if stripped == 'filters: ["keepout_filter"]':
        removed_global_keepout_filter_list = True
        continue

    if stripped == "enabled: true" and out and out[-1].strip() == 'plugin: "nav2_costmap_2d::KeepoutFilter"':
        out.append(line.replace("true", "false", 1))
        disabled_keepout_plugin = True
        continue

    out.append(line)

missing = []
if not disabled_filter_manager:
    missing.append("lifecycle_manager_costmap_filters autostart")
if not removed_global_keepout_filter_list:
    missing.append('global_costmap filters: ["keepout_filter"]')
if not disabled_keepout_plugin:
    missing.append("keepout_filter enabled")
if missing:
    raise SystemExit(f"expected neutral keepout disable targets not found in {source}: {', '.join(missing)}")

target.write_text("\n".join(out) + "\n", encoding="utf-8")
PY
  export NAV2_PARAMS_FILE="${runtime_params}"
  if ! nav2_speed_filter_enabled; then
    export NJRH_NAV2_COSTMAP_FILTER_SERVERS_ENABLED=false
  fi
  echo "[runtime-overlay] neutral keepout filter mask detected; global costmap keepout filter removed and filter lifecycle autostart disabled for this Nav2 launch params=${NAV2_PARAMS_FILE} filter_servers_enabled=${NJRH_NAV2_COSTMAP_FILTER_SERVERS_ENABLED:-true}" >&2
}

ensure_costmap_filter_masks || {
  echo "[runtime-overlay] failed to prepare costmap filter masks" >&2
  exit 1
}
disable_neutral_costmap_filters_if_needed

nav_pid=""
nav_lifecycle_bringup_pid=""
nav_exit_code=0
cleanup_started=0

cleanup() {
  if [[ "${cleanup_started}" -eq 1 ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  if [[ -n "${nav_lifecycle_bringup_pid}" ]]; then
    kill -INT "${nav_lifecycle_bringup_pid}" 2>/dev/null || true
    sleep 0.5
    kill -TERM "${nav_lifecycle_bringup_pid}" 2>/dev/null || true
    sleep 0.5
    kill -KILL "${nav_lifecycle_bringup_pid}" 2>/dev/null || true
    wait "${nav_lifecycle_bringup_pid}" 2>/dev/null || true
    nav_lifecycle_bringup_pid=""
  fi
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
echo "[runtime-overlay] local_perception helper disabled; local costmap/collision_monitor consume /scan for standard marking+clearing" >&2

ros2 launch "${LAUNCH_FILE}" \
  use_sim_time:=false \
  autostart:=true \
  navigation_lifecycle_autostart:="${navigation_lifecycle_autostart}" \
  params_file:="${NAV2_PARAMS_FILE}" \
  keepout_mask_yaml:="${NAV2_KEEP_OUT_MASK_YAML}" \
  speed_mask_yaml:="${NAV2_SPEED_MASK_YAML}" &
nav_pid=$!
sleep "${NJRH_NAV2_LAUNCH_SETTLE_SEC:-1}"
if ! kill -0 "${nav_pid}" 2>/dev/null; then
  wait "${nav_pid}" || nav_exit_code=$?
  echo "[runtime-overlay] Nav2 launch exited during initial settle with ${nav_exit_code}" >&2
  exit "${nav_exit_code}"
fi
wait_for_controller_server_affinity || exit 1
if nav2_external_lifecycle_bringup_enabled && ! nav2_lifecycle_hold_enabled; then
  if [[ "${NJRH_NAV2_USE_REPO_LIFECYCLE_SEQUENCE:-true}" == "true" ]]; then
    start_navigation_lifecycle_with_repo_sequence || exit 1
  else
    start_navigation_lifecycle_with_nav2_util || exit 1
  fi
elif nav2_lifecycle_hold_enabled; then
  write_nav2_hold_ready_status
  echo "[runtime-overlay] Nav2 launch process is running with lifecycle activation held" >&2
fi
echo "[runtime-overlay] Nav2 launch process is running; blocking readiness probes are disabled" >&2

wait "${nav_pid}" || nav_exit_code=$?
exit "${nav_exit_code}"

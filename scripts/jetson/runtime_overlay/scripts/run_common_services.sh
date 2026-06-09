#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/nav_runtime_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"
njrh_load_pointcloud_accel_profile

common_pids=()
NAV_LOCAL_STATE_MODE="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}"
# FAST-LIO2 is mapping-owned by default. Daily navigation uses wheel+IMU EKF
# local odom, so common services must not keep the lidar-inertial frontend
# resident unless an explicit diagnostic FAST-LIO local-state mode is selected.
FASTLIO_AUTOSTART="${NJRH_FASTLIO_AUTOSTART:-false}"
FASTLIO_CONFIG_FILE="${NJRH_FASTLIO_CONFIG_FILE:-${NJRH_OVERLAY_ROOT}/config/fastlio.yaml}"
FASTLIO_POINTS_TOPIC="${NJRH_FASTLIO_POINTS_TOPIC:-/cloud_registered_body}"
FASTLIO_ODOM_TOPIC="${NJRH_FASTLIO_ODOM_TOPIC:-/Odometry}"
FASTLIO_TOPIC_FRESH_TIMEOUT="${NJRH_FASTLIO_TOPIC_FRESH_TIMEOUT:-8}"
FASTLIO_TOPIC_MAX_AGE_SEC="${NJRH_FASTLIO_TOPIC_MAX_AGE_SEC:-1.0}"
FASTLIO_TOPIC_MAX_FUTURE_SEC="${NJRH_FASTLIO_TOPIC_MAX_FUTURE_SEC:-0.25}"
FASTLIO_ODOM_FRESH_TIMEOUT="${NJRH_FASTLIO_ODOM_FRESH_TIMEOUT:-8}"
FASTLIO_ODOM_MAX_AGE_SEC="${NJRH_FASTLIO_ODOM_MAX_AGE_SEC:-1.0}"
FASTLIO_ODOM_MAX_FUTURE_SEC="${NJRH_FASTLIO_ODOM_MAX_FUTURE_SEC:-0.25}"
LAST_NAVIGATION_MAP_FILE="${NJRH_LAST_NAVIGATION_MAP_FILE:-${NJRH_RELEASE_ASSETS_DIR}/last_navigation_map.json}"

start_common_process() {
  local name="$1"
  local pattern="$2"
  shift 2
  local log_file="${NJRH_RUNTIME_LOG_DIR}/${name}.log"

  if reuse_common_services_enabled && pgrep -f "${pattern}" >/dev/null 2>&1; then
    echo "[runtime-overlay] reusing existing ${name}; pattern=${pattern}" >&2
    return 0
  fi

  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  echo "[runtime-overlay] starting ${name}" >&2
  "$@" >>"${log_file}" 2>&1 &
  local pid=$!
  common_pids+=("${pid}")
  sleep 1
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[runtime-overlay] common service failed to stay alive: ${name}. Check ${log_file}" >&2
    return 1
  fi
  echo "[runtime-overlay] common service ready: ${name} (pid=${pid})" >&2
}

canonical_jt128_ingress_running() {
  local pointcloud_pipeline_pattern="pointcloud_perception_pipeline.launch.py|component_container_mt.*pointcloud_perception_pipeline|pointcloud_perception_pipeline"
  local pointcloud_standalone_pattern="pointcloud_axis_remap|pointcloud_accel_axis"
  pgrep -f "hesai_ros_driver_node" >/dev/null 2>&1 &&
    { pgrep -f "${pointcloud_pipeline_pattern}" >/dev/null 2>&1 || pgrep -f "${pointcloud_standalone_pattern}" >/dev/null 2>&1; } &&
    pgrep -f "imu_axis_remap" >/dev/null 2>&1
}

pointcloud_accel_pipeline_aux_running() {
  [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]] || return 0
  pgrep -f "run_pointcloud_accel_pipeline.sh|laser_scan_to_flatscan" >/dev/null 2>&1
}

canonical_jt128_runtime_complete() {
  canonical_jt128_ingress_running && pointcloud_accel_pipeline_aux_running
}

fastlio_runtime_running() {
  pgrep -f "ros2 run fast_lio fastlio_mapping|fast_lio fastlio_mapping|laser_mapping" >/dev/null 2>&1
}

fastlio_runtime_output_fresh() {
  fastlio_runtime_running || return 1
  runtime_readiness_probe \
    fresh-header-topic \
    "${FASTLIO_ODOM_TOPIC}" \
    "${FASTLIO_ODOM_FRESH_TIMEOUT}" \
    "${FASTLIO_ODOM_MAX_AGE_SEC}" \
    "${FASTLIO_ODOM_MAX_FUTURE_SEC}" >/dev/null 2>&1
}

wait_for_fastlio_runtime_output() {
  runtime_readiness_probe \
    fresh-header-topic \
    "${FASTLIO_ODOM_TOPIC}" \
    "${FASTLIO_ODOM_FRESH_TIMEOUT}" \
    "${FASTLIO_ODOM_MAX_AGE_SEC}" \
    "${FASTLIO_ODOM_MAX_FUTURE_SEC}"
}

stop_fastlio_runtime_processes() {
  local patterns=(
    "ros2 run fast_lio fastlio_mapping"
    "fast_lio/lib/fast_lio/fastlio_mapping"
    "fast_lio/fastlio_mapping"
    "fastlio_mapping --ros-args"
    "laser_mapping"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -INT -f "${pattern}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_INT_WAIT_SEC:-1}"
  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_TERM_WAIT_SEC:-1}"
  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
}

fastlio_pid_is_mapping_owned() {
  local pid="$1"
  [[ -r "/proc/${pid}/environ" ]] || return 1
  tr '\0' '\n' <"/proc/${pid}/environ" | grep -qx "NJRH_SLAM2D_PRIVATE_FASTLIO=1"
}

stop_non_mapping_fastlio_runtime_processes() {
  local pids=()
  local proc pid
  for proc in /proc/[0-9]*; do
    [[ -r "${proc}/cmdline" ]] || continue
    pid="${proc##*/}"
    tr '\0' ' ' <"${proc}/cmdline" | grep -Eq "ros2 run fast_lio fastlio_mapping|fast_lio/lib/fast_lio/fastlio_mapping|fast_lio/fastlio_mapping|fastlio_mapping --ros-args|laser_mapping" || continue
    fastlio_pid_is_mapping_owned "${pid}" && continue
    pids+=("${pid}")
  done
  [[ ${#pids[@]} -gt 0 ]] || return 0
  echo "[runtime-overlay] FAST-LIO2 common autostart disabled; stopping non-mapping FAST-LIO leftovers: ${pids[*]}" >&2
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_INT_WAIT_SEC:-1}"
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_FASTLIO_STOP_TERM_WAIT_SEC:-1}"
  for pid in "${pids[@]}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
}

start_fastlio_common() {
  [[ -f "${FASTLIO_CONFIG_FILE}" ]] || {
    echo "[runtime-overlay] missing FAST-LIO runtime file: ${FASTLIO_CONFIG_FILE}" >&2
    return 1
  }

  if reuse_common_services_enabled && fastlio_runtime_running; then
    if fastlio_runtime_output_fresh; then
      echo "[runtime-overlay] reusing existing fastlio_mapping common runtime; ${FASTLIO_ODOM_TOPIC} is fresh" >&2
      return 0
    fi
    echo "[runtime-overlay] existing fastlio_mapping process has stale/missing ${FASTLIO_ODOM_TOPIC}; restarting FAST-LIO" >&2
    stop_fastlio_runtime_processes
  fi

  if reuse_common_services_enabled && fastlio_runtime_running; then
    echo "[runtime-overlay] reusing existing fastlio_mapping common runtime after stale-output cleanup" >&2
  else
    start_common_process "fastlio_mapping" "ros2 run fast_lio fastlio_mapping|fast_lio fastlio_mapping|laser_mapping" \
      njrh_run_affined fastlio_mapping ros2 run fast_lio fastlio_mapping \
        --ros-args \
        --params-file "${FASTLIO_CONFIG_FILE}" \
        -p use_sim_time:=false \
          -r /tf:=/tf_fastlio_internal \
          -r /tf_static:=/tf_static_fastlio_internal
  fi

  if ! wait_for_fastlio_runtime_output; then
    echo "[runtime-overlay] FAST-LIO failed to publish fresh ${FASTLIO_ODOM_TOPIC}; stopping stale runtime" >&2
    stop_fastlio_runtime_processes
    return 1
  fi
}

load_last_navigation_map_selection() {
  [[ -f "${LAST_NAVIGATION_MAP_FILE}" ]] || return 1
  python3 - "${LAST_NAVIGATION_MAP_FILE}" "${NJRH_RELEASE_ASSETS_DIR}" <<'PY'
import json
import re
import sys
from pathlib import Path

state_path = Path(sys.argv[1])
maps_root = Path(sys.argv[2])
safe = re.compile(r"^[A-Za-z0-9_.-]+$")

try:
    data = json.loads(state_path.read_text(encoding="utf-8"))
except Exception as exc:
    print(f"[runtime-overlay] cannot read last navigation map file {state_path}: {exc}", file=sys.stderr)
    raise SystemExit(1)

map_id = str(data.get("map_id") or "")
building_id = str(data.get("building_id") or "")
floor_id = str(data.get("floor_id") or "")
display_name = str(data.get("display_name") or map_id).replace("\t", " ").replace("\n", " ")
if not (safe.fullmatch(map_id) and safe.fullmatch(building_id) and safe.fullmatch(floor_id)):
    print("[runtime-overlay] last navigation map file has invalid ids", file=sys.stderr)
    raise SystemExit(1)

current_root = maps_root / building_id / floor_id / "current"
current_manifest = current_root / "manifest.json"
required = [
    current_manifest,
    current_root / "nav" / "nav_map.yaml",
    current_root / "localizer" / "localizer_params.yaml",
    current_root / "localizer" / "localizer_map.png",
]
missing = [str(path) for path in required if not path.is_file()]
if missing:
    print("[runtime-overlay] last navigation map is not selected in current/: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)

try:
    current_data = json.loads(current_manifest.read_text(encoding="utf-8"))
except Exception as exc:
    print(f"[runtime-overlay] cannot read current manifest {current_manifest}: {exc}", file=sys.stderr)
    raise SystemExit(1)

if str(current_data.get("map_id") or "") != map_id:
    print(
        f"[runtime-overlay] last navigation map {map_id} does not match current manifest "
        f"{current_data.get('map_id')}",
        file=sys.stderr,
    )
    raise SystemExit(1)

print("\t".join([building_id, floor_id, map_id, display_name]))
PY
}

cleanup() {
  trap - EXIT INT TERM
  local pid
  for pid in "${common_pids[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  cleanup_overlay_helpers
  cleanup_canonical_helpers
  sleep 1
  for pid in "${common_pids[@]:-}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

require_can_interface_up

if reuse_common_services_enabled && canonical_jt128_runtime_complete; then
  echo "[runtime-overlay] reusing existing jt128_driver; canonical driver/remap chain is complete" >&2
else
  if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" == "legacy" ]]; then
    start_common_process "jt128_driver" "__njrh_force_start_jt128_driver_chain__" \
      bash "${SCRIPT_DIR}/run_driver.sh"
  else
    start_common_process "pointcloud_accel_pipeline" "__njrh_force_start_pointcloud_accel_pipeline__" \
      bash "${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh"
  fi
fi
start_canonical_helper "ranger_chassis_common" bash "${SCRIPT_DIR}/run_ranger_chassis.sh"
start_canonical_helper "robot_description_static_tf_common" bash "${SCRIPT_DIR}/run_robot_description.sh"
if [[ "${FASTLIO_AUTOSTART}" == "true" ]] || { [[ "${NAV_LOCAL_STATE_MODE}" == "fastlio" ]] && fastlio_runtime_running; }; then
  start_fastlio_common
elif [[ "${NAV_LOCAL_STATE_MODE}" == "fastlio" ]]; then
  echo "[runtime-overlay] NJRH_NAV_LOCAL_STATE_MODE=fastlio requires NJRH_FASTLIO_AUTOSTART=true or an already managed FAST-LIO runtime" >&2
  exit 1
else
  stop_non_mapping_fastlio_runtime_processes
  echo "[runtime-overlay] FAST-LIO2 common autostart disabled; mapping starts FAST-LIO2 only while mapping is active" >&2
fi
if [[ "${NJRH_GS2_AUTOSTART:-true}" == "true" ]]; then
  start_common_process "gs2_driver" "robot_eai_gs2/gs2_driver_node|gs2_driver_node --ros-args|ros2 launch robot_eai_gs2 gs2.launch.py" \
    bash "${SCRIPT_DIR}/run_gs2_driver.sh"
fi
if [[ "${NAV_LOCAL_STATE_MODE}" == "passthrough" || "${NAV_LOCAL_STATE_MODE}" == "legacy" ]]; then
  # Explicit diagnostic fallback: keep the canonical /local_state/odometry
  # and odom->base_link owner, but back it directly with /wheel/odom.
  kill_canonical_pattern "robot_localization/ekf_node"
  kill_canonical_pattern "ekf_node --ros-args.*__node:=robot_local_state"
fi
start_canonical_helper \
  "robot_local_state_common" \
  env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_local_state.sh"
if [[ "${NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART:-false}" == "true" ]]; then
  start_common_process "runtime_health_guard" "runtime_health_guard.py|run_runtime_health_guard.sh" \
    bash "${SCRIPT_DIR}/run_runtime_health_guard.sh"
else
  echo "[runtime-overlay] runtime_health_guard autostart disabled; startup readiness probes are disabled" >&2
fi
if [[ "${NJRH_POINTCLOUD_ACCEL_PROFILE}" != "legacy" ]]; then
  echo "[runtime-overlay] local_perception is owned by pointcloud accel profile=${NJRH_POINTCLOUD_ACCEL_PROFILE}; skipping standalone local_perception_common" >&2
elif [[ "${NJRH_JT128_USE_POINTCLOUD_PIPELINE_CONTAINER:-false}" == "true" ]]; then
  echo "[runtime-overlay] local_perception is owned by pointcloud_perception_pipeline; skipping standalone local_perception_common" >&2
else
  start_overlay_helper "local_perception_common" bash "${SCRIPT_DIR}/run_local_perception.sh"
fi
start_overlay_helper "floor_manager_common" bash "${SCRIPT_DIR}/run_floor_manager.sh"
start_overlay_helper "robot_safety_common" bash "${SCRIPT_DIR}/run_robot_safety.sh"
start_overlay_helper "ranger_mini3_mode_controller_common" bash "${SCRIPT_DIR}/run_ranger_mini3_mode_controller.sh"
start_common_process "robot_api_server" "run_robot_api_server.sh|run_robot_api_server_supervised.sh|robot_api_server/robot_api_server_node|robot_api_server_node --ros-args" \
  bash "${SCRIPT_DIR}/run_robot_api_server_supervised.sh"

RESIDENT_NAVIGATION_AUTOSTART="${NJRH_RESIDENT_NAVIGATION_AUTOSTART:-auto}"
if [[ "${RESIDENT_NAVIGATION_AUTOSTART}" != "false" ]]; then
  autostart_building_id=""
  autostart_floor_id=""
  autostart_map_id=""
  autostart_display_name=""

  if [[ "${RESIDENT_NAVIGATION_AUTOSTART}" == "true" && -n "${NJRH_FLOOR_ID:-}" ]]; then
    autostart_building_id="${NJRH_BUILDING_ID:-building_1}"
    autostart_floor_id="${NJRH_FLOOR_ID}"
    echo "[runtime-overlay] resident navigation autostart uses explicit floor ${autostart_building_id}/${autostart_floor_id}" >&2
  else
    if selection="$(load_last_navigation_map_selection)"; then
      IFS=$'\t' read -r autostart_building_id autostart_floor_id autostart_map_id autostart_display_name <<<"${selection}"
      echo "[runtime-overlay] resident navigation autostart selected last map ${autostart_building_id}/${autostart_floor_id}/${autostart_map_id}" >&2
    else
      echo "[runtime-overlay] no valid last navigation map; common services stay alive in NO_MAP mode" >&2
    fi
  fi

  if [[ -n "${autostart_floor_id}" ]]; then
    start_common_process "resident_navigation_runtime" "run_navigation_runtime_services.sh" \
      env \
        NJRH_MAP_ID="${autostart_map_id}" \
        NJRH_MAP_DISPLAY_NAME="${autostart_display_name}" \
        NJRH_MAP_CONTEXT_BUILDING_ID="${autostart_building_id}" \
        NJRH_MAP_CONTEXT_FLOOR_ID="${autostart_floor_id}" \
        bash "${SCRIPT_DIR}/run_navigation_runtime_services.sh" "${autostart_building_id}" "${autostart_floor_id}"
  fi
fi

echo "[runtime-overlay] common services are running; start mapping or resident navigation scripts in reuse mode" >&2
while true; do
  sleep 3600
done

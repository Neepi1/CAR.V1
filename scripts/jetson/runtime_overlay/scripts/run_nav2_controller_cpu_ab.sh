#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PROFILE="${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current}"
DURATION_SEC=180
APPLY=false
RESTART_NAV2=false
LABEL="nav2_controller_cpu_ab"

usage() {
  cat <<'USAGE'
Usage: run_nav2_controller_cpu_ab.sh --profile current|control_wide [--duration-sec N] [--apply] [--restart-nav2] [--label LABEL]

Runs the Phase C1 controller_server CPU-set A/B observation. Default mode is a
dry-run plan. With --apply, this script sets the selected profile and records a
report. With --restart-nav2, it restarts only the Nav2 navigation layer using a
PID-specific INT/TERM stop path; it does not use forced broad pattern kills.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --apply)
      APPLY=true
      shift
      ;;
    --restart-nav2)
      RESTART_NAV2=true
      shift
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[nav2-controller-cpu-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${PROFILE}" in
  current|control_wide) ;;
  *)
    echo "[nav2-controller-cpu-ab] --profile must be current or control_wide" >&2
    exit 2
    ;;
esac
case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[nav2-controller-cpu-ab] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

export NJRH_NAV2_CONTROLLER_CPU_PROFILE="${PROFILE}"
source "${SCRIPT_DIR}/cpu_affinity.sh"

expected_cpuset="${NJRH_CPUSET_CONTROLLER_SERVER:-}"
case "${PROFILE}" in
  current)
    [[ "${expected_cpuset}" == "${NJRH_CPUSET_NAV2_CONTROLLER_CURRENT:-3}" ]] || {
      echo "[nav2-controller-cpu-ab] current profile resolved to unexpected cpuset=${expected_cpuset}" >&2
      exit 1
    }
    ;;
  control_wide)
    [[ "${expected_cpuset}" == "${NJRH_CPUSET_NAV2_CONTROLLER_WIDE:-3,5}" ]] || {
      echo "[nav2-controller-cpu-ab] control_wide profile resolved to unexpected cpuset=${expected_cpuset}" >&2
      exit 1
    }
    ;;
esac

safe_label="$(printf '%s' "${LABEL}_${PROFILE}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_path="${NJRH_PROJECT_ROOT}/reports/nav2_controller_cpu_ab_${timestamp}.md"
log_dir="${NJRH_PROJECT_ROOT}/reports/nav2_controller_cpu_ab_logs"
mkdir -p "$(dirname "${report_path}")" "${log_dir}"

controller_server_pids() {
  ps -eo pid=,args= | awk '
    /controller_server/ &&
    (/nav2_controller/ || /__node:=controller_server/ || /\/controller_server/) &&
    $0 !~ /run_nav2_controller_cpu_ab|ros2 lifecycle|get \/controller_server|ros2 param|awk/ {
      print $1
    }
  '
}

read_cpuset() {
  local pid="$1"
  awk -F: '/^Cpus_allowed_list:/ {gsub(/^[ \t]+/, "", $2); print $2; exit}' "/proc/${pid}/status" 2>/dev/null || true
}

nav2_stop_pids() {
  ps -eo pid=,args= | awk '
    /standard_navigation.launch.py|run_nav2_navigation.sh|__node:=keepout_filter_mask_server|__node:=speed_filter_mask_server|__node:=keepout_costmap_filter_info_server|__node:=speed_costmap_filter_info_server|__node:=controller_server|__node:=smoother_server|__node:=planner_server|__node:=behavior_server|__node:=bt_navigator|__node:=waypoint_follower|__node:=velocity_smoother|__node:=collision_monitor|__node:=lifecycle_manager_costmap_filters|__node:=lifecycle_manager_navigation/ &&
    $0 !~ /run_nav2_controller_cpu_ab|awk/ {
      print $1
    }
  ' | sort -n -u
}

safe_stop_nav2_stack() {
  mapfile -t pids < <(nav2_stop_pids)
  if [[ "${#pids[@]}" -eq 0 ]]; then
    echo "[nav2-controller-cpu-ab] no existing Nav2 navigation PIDs to stop" >&2
    return 0
  fi
  echo "[nav2-controller-cpu-ab] stopping Nav2 PIDs with INT/TERM only: ${pids[*]}" >&2
  local pid
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_NAV2_AB_STOP_INT_WAIT_SEC:-2}"
  for pid in "${pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  local deadline=$((SECONDS + ${NJRH_NAV2_AB_STOP_GRACE_SEC:-12}))
  local still_running=()
  while (( SECONDS <= deadline )); do
    still_running=()
    for pid in "${pids[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        still_running+=("${pid}")
      fi
    done
    [[ "${#still_running[@]}" -eq 0 ]] && return 0
    sleep "${NJRH_NAV2_AB_STOP_TERM_WAIT_SEC:-1}"
  done
  echo "[nav2-controller-cpu-ab] Nav2 PIDs still running after INT/TERM grace: ${still_running[*]}" >&2
  return 1
}

apply_controller_affinity_to_existing() {
  local pid
  local task_path
  local tid
  pid="$(controller_server_pids | tail -n 1 || true)"
  [[ -n "${pid}" ]] || {
    echo "[nav2-controller-cpu-ab] controller_server is not running" >&2
    return 1
  }
  for task_path in /proc/"${pid}"/task/*; do
    [[ -e "${task_path}" ]] || continue
    tid="${task_path##*/}"
    taskset -pc "${expected_cpuset}" "${tid}" >/dev/null
  done
}

wait_controller_affinity() {
  local deadline=$((SECONDS + ${NJRH_NAV2_AB_CONTROLLER_WAIT_SEC:-30}))
  local pid=""
  local allowed=""
  while (( SECONDS <= deadline )); do
    pid="$(controller_server_pids | tail -n 1 || true)"
    if [[ -n "${pid}" && -r "/proc/${pid}/status" ]]; then
      allowed="$(read_cpuset "${pid}")"
      if [[ "${allowed}" == "${expected_cpuset}" ]]; then
        echo "${pid}"
        return 0
      fi
      echo "[nav2-controller-cpu-ab] waiting controller affinity pid=${pid} expected=${expected_cpuset} actual=${allowed:-missing}" >&2
    fi
    sleep 0.5
  done
  echo "[nav2-controller-cpu-ab] controller affinity not ready expected=${expected_cpuset} pid=${pid:-missing} actual=${allowed:-missing}" >&2
  return 1
}

echo "[nav2-controller-cpu-ab] profile=${PROFILE} cpuset=${expected_cpuset} duration_sec=${DURATION_SEC} apply=${APPLY} restart_nav2=${RESTART_NAV2}" >&2

if [[ "${APPLY}" != "true" ]]; then
  cat <<PLAN
# Nav2 Controller CPU A/B Dry Run

- profile: ${PROFILE}
- controller_server cpuset: ${expected_cpuset}
- duration_sec: ${DURATION_SEC}
- restart_nav2: ${RESTART_NAV2}

Run with --apply to execute. This script does not change Nav2 parameters,
transform_tolerance, max_odom_tf_age_ms, pointcloud QoS/DDS, FAST-LIO2,
Ranger odom, or EKF production fusion.
PLAN
  exit 0
fi

nav2_runner_pid=""
if [[ "${RESTART_NAV2}" == "true" ]]; then
  safe_stop_nav2_stack
  export NJRH_SKIP_PRESTART_NAV2_STOP=true
  nav_log="${log_dir}/${timestamp}_${safe_label}_run_nav2_navigation.log"
  echo "[nav2-controller-cpu-ab] starting Nav2 navigation; log=${nav_log}" >&2
  nohup bash "${SCRIPT_DIR}/run_nav2_navigation.sh" >"${nav_log}" 2>&1 &
  nav2_runner_pid="$!"
else
  apply_controller_affinity_to_existing
fi

controller_pid="$(wait_controller_affinity)"

inspect_output="$(bash "${SCRIPT_DIR}/inspect_nav2_controller_threads.sh" 2>&1 || true)"
observation_dir="$(bash "${SCRIPT_DIR}/observe_controller_tf_backlog_180s.sh" --duration-sec "${DURATION_SEC}" --label "${safe_label}" | awk '/^\// {path=$0} END {print path}')"
if [[ -z "${observation_dir}" ]]; then
  echo "[nav2-controller-cpu-ab] observe_controller_tf_backlog_180s.sh did not return a report path" >&2
  exit 1
fi
summary_json="${observation_dir}/summary.json"

python3 - "${report_path}" "${summary_json}" "${observation_dir}" "${PROFILE}" "${expected_cpuset}" "${controller_pid}" "${nav2_runner_pid}" <<'PY'
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1])
summary_json = Path(sys.argv[2])
observation_dir = sys.argv[3]
profile = sys.argv[4]
expected_cpuset = sys.argv[5]
controller_pid = sys.argv[6]
nav2_runner_pid = sys.argv[7]

summary = {}
if summary_json.exists():
    summary = json.loads(summary_json.read_text(encoding="utf-8"))

lag = summary.get("controller_requested_latest_lag", {})
rosout = summary.get("rosout_counts", {})
tf_series = summary.get("tf_series", {})
cmd_vel = summary.get("cmd_vel", {})

def value(path, default="n/a"):
    cur = summary
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur

lines = [
    "# Nav2 Controller CPU A/B Report",
    "",
    f"- profile: {profile}",
    f"- expected_controller_cpuset: {expected_cpuset}",
    f"- controller_pid: {controller_pid}",
    f"- controller_cpus_allowed_list: {summary.get('controller_cpus_allowed_list')}",
    f"- nav2_runner_pid: {nav2_runner_pid or 'not_restarted'}",
    f"- observation_dir: {observation_dir}",
    f"- tf_future_extrapolation_count: {rosout.get('tf_future_extrapolation', 0)}",
    f"- local_costmap_message_filter_drop_count: {rosout.get('local_costmap_message_filter_drop', 0)}",
    f"- requested_latest_lag_count: {lag.get('count', 0)}",
    f"- requested_latest_lag_p99_ms: {lag.get('p99_ms')}",
    f"- requested_latest_lag_max_ms: {lag.get('max_ms')}",
    "",
    "## TF",
    "",
    f"- map_to_odom_hz: {value(['tf_series', 'tf:map->odom', 'hz'])}",
    f"- map_to_odom_recv_gap_max_ms: {value(['tf_series', 'tf:map->odom', 'recv_gap', 'max_ms'])}",
    f"- odom_to_base_hz: {value(['tf_series', 'tf:odom->base_link', 'hz'])}",
    f"- odom_to_base_recv_gap_max_ms: {value(['tf_series', 'tf:odom->base_link', 'recv_gap', 'max_ms'])}",
    "",
    "## Cmd Vel",
    "",
]
for topic, data in sorted(cmd_vel.items()):
    lines.append(
        f"- {topic}: count={data.get('count', 0)} hz={data.get('hz', 0.0):.3f} "
        f"nonzero={data.get('nonzero_count', 0)} max_linear_x={data.get('max_abs_linear_x', 0.0):.4f} "
        f"max_angular_z={data.get('max_abs_angular_z', 0.0):.4f}"
    )
lines.extend([
    "",
    "## Contract",
    "",
    "- changed_tf_gate: no",
    "- changed_nav2_controller_or_planner_params: no",
    "- changed_pointcloud_qos_or_dds: no",
    "- changed_fastlio: no",
    "- changed_ranger_odom_or_ekf: no",
])
report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

{
  echo
  echo "## Inspect Output"
  echo
  echo '```text'
  printf '%s\n' "${inspect_output}"
  echo '```'
} >>"${report_path}"

echo "[nav2-controller-cpu-ab] report ${report_path}"
echo "[nav2-controller-cpu-ab] observation ${observation_dir}"

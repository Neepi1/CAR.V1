#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

controller_server_pids() {
  ps -eo pid=,args= | awk '
    /controller_server/ &&
    (/nav2_controller/ || /__node:=controller_server/ || /\/controller_server/) &&
    $0 !~ /ros2 lifecycle|get \/controller_server|ros2 param|awk/ {
      print $1
    }
  '
}

read_cpuset() {
  local pid="$1"
  awk -F: '/^Cpus_allowed_list:/ {gsub(/^[ \t]+/, "", $2); print $2; exit}' "/proc/${pid}/status" 2>/dev/null || true
}

read_proc_env_value() {
  local pid="$1"
  local key="$2"
  tr '\0' '\n' <"/proc/${pid}/environ" 2>/dev/null | awk -F= -v key="${key}" '$1 == key {print $2; exit}' || true
}

cpuset_for_controller_profile() {
  case "$1" in
    control_wide)
      printf '%s\n' "${NJRH_CPUSET_NAV2_CONTROLLER_WIDE:-3,5}"
      ;;
    current|*)
      printf '%s\n' "${NJRH_CPUSET_NAV2_CONTROLLER_CURRENT:-3}"
      ;;
  esac
}

print_process_rows() {
  local label="$1"
  local pattern="$2"
  local expected="${3:-}"
  ps -eo pid=,psr=,pcpu=,comm=,args= | awk -v label="${label}" -v pattern="${pattern}" -v expected="${expected}" '
    $0 ~ pattern && $0 !~ /inspect_nav2_controller_threads|awk/ {
      print label "|" $1 "|" $2 "|" $3 "|" expected "|" substr($0, index($0, $5))
    }
  ' | while IFS='|' read -r name pid psr pcpu want cmd; do
    [[ -n "${pid}" ]] || continue
    printf '%-34s pid=%-8s psr=%-2s cpu=%-6s allowed=%-8s expected=%-8s cmd=%s\n' \
      "${name}" "${pid}" "${psr}" "${pcpu}" "$(read_cpuset "${pid}")" "${want:-n/a}" "${cmd}"
  done
}

controller_pid="$(controller_server_pids | tail -n 1 || true)"
live_profile="${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current}"
if [[ -n "${controller_pid}" ]]; then
  controller_env_profile="$(read_proc_env_value "${controller_pid}" "NJRH_NAV2_CONTROLLER_CPU_PROFILE")"
  [[ -n "${controller_env_profile}" ]] && live_profile="${controller_env_profile}"
fi
controller_expected_cpuset="$(cpuset_for_controller_profile "${live_profile}")"

echo "# Nav2 Controller Thread Inspection"
echo
echo "- shell profile: ${NJRH_NAV2_CONTROLLER_CPU_PROFILE:-current}"
echo "- controller live profile: ${live_profile}"
echo "- expected current cpuset: ${NJRH_CPUSET_NAV2_CONTROLLER_CURRENT:-3}"
echo "- expected control_wide cpuset: ${NJRH_CPUSET_NAV2_CONTROLLER_WIDE:-3,5}"
echo "- expected controller_server cpuset: ${controller_expected_cpuset}"
echo

if [[ -z "${controller_pid}" ]]; then
  echo "controller_server_pid=missing"
else
  echo "controller_server_pid=${controller_pid}"
  echo "controller_server_Cpus_allowed_list=$(read_cpuset "${controller_pid}")"
  echo
  echo "## controller_server threads"
  ps -L -p "${controller_pid}" -o pid,tid,psr,pcpu,pmem,comm,wchan:28 --sort=-pcpu || true
fi

echo
echo "## local_costmap ownership"
if timeout 5 ros2 node list 2>/dev/null | grep -qx "/local_costmap/local_costmap"; then
  echo "local_costmap_node=/local_costmap/local_costmap"
  echo "local_costmap_same_pid=expected_yes_controller_server_hosts_local_costmap"
else
  echo "local_costmap_node=missing"
fi
if ps -eo pid=,args= | grep -E "local_costmap" | grep -vE "inspect_nav2_controller_threads|grep" >/dev/null 2>&1; then
  echo "local_costmap_process_hint:"
  ps -eo pid=,psr=,pcpu=,args= | grep -E "local_costmap" | grep -vE "inspect_nav2_controller_threads|grep" || true
else
  echo "separate_local_costmap_process=none_detected"
fi

echo
echo "## key process CPU status"
print_process_rows "controller_server" "controller_server" "${controller_expected_cpuset}"
print_process_rows "robot_local_state_ekf" "ekf_node|robot_local_state" "${NJRH_CPUSET_ROBOT_LOCAL_STATE:-2}"
print_process_rows "robot_localization_bridge" "localization_bridge_node" "${NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE:-7}"
print_process_rows "amcl_scan_admission_node" "amcl_scan_admission_node|amcl_scan_admission_relay.py" "${NJRH_CPUSET_AMCL_SCAN_ADMISSION:-6}"
print_process_rows "hesai_accel_driver_node" "hesai_accel_driver_node" "${NJRH_CPUSET_HESAI_ROS_DRIVER:-4}"
print_process_rows "collision_monitor" "collision_monitor" "${NJRH_CPUSET_COLLISION_MONITOR:-1}"
print_process_rows "velocity_smoother" "velocity_smoother" "${NJRH_CPUSET_VELOCITY_SMOOTHER:-1}"
print_process_rows "bt_navigator" "bt_navigator" "${NJRH_CPUSET_BT_NAVIGATOR:-3,5}"
print_process_rows "planner_server" "planner_server" "${NJRH_CPUSET_PLANNER_SERVER:-3,5}"

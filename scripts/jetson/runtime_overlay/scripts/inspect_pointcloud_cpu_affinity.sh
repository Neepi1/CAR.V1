#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"
njrh_load_local_perception_input_profile

tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_cpu_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

field_value() {
  local text="$1"
  local key="$2"
  printf '%s\n' "${text}" | tr ' ' '\n' | awk -F= -v key="${key}" '$1 == key {print substr($0, length(key) + 2); exit}'
}

status_once() {
  local topic="$1"
  local raw_file="${tmp_dir}/status.raw"
  timeout 5 ros2 topic echo "${topic}" --field data >"${raw_file}" 2>&1 || true
  awk 'NF && $0 != "---" {line=$0} END {print line}' "${raw_file}"
}

pid_list_for_pattern() {
  local pattern="$1"
  pgrep -f "${pattern}" 2>/dev/null | sort -n | paste -sd ' ' -
}

pid_csv_for_pattern() {
  local pattern="$1"
  pgrep -f "${pattern}" 2>/dev/null | sort -n | paste -sd ',' -
}

print_threads_for_pattern() {
  local label="$1"
  local pattern="$2"
  local pids
  local pid_csv
  pids="$(pid_list_for_pattern "${pattern}")"
  pid_csv="$(pid_csv_for_pattern "${pattern}")"
  if [[ -z "${pids}" ]]; then
    echo "[pointcloud-cpu] ${label}: not running"
    return 0
  fi
  echo "[pointcloud-cpu] ${label}: pids=${pids}"
  ps -T -p "${pid_csv}" -o pid,tid,psr,pcpu,comm --no-headers 2>/dev/null |
    awk -v label="${label}" '{printf "[pointcloud-cpu]   %-28s pid=%s tid=%s cpu=%s pcpu=%s comm=%s\n", label, $1, $2, $3, $4, $5}'
}

cpus_for_pattern() {
  local pattern="$1"
  local pid_csv
  pid_csv="$(pid_csv_for_pattern "${pattern}")"
  [[ -n "${pid_csv}" ]] || return 0
  ps -T -p "${pid_csv}" -o psr --no-headers 2>/dev/null | awk 'NF {seen[$1]=1} END {for (cpu in seen) print cpu}' | sort -n | paste -sd ',' -
}

print_thermal_and_clock_snapshot() {
  echo "[pointcloud-cpu] thermal and clock snapshot:"
  if command -v tegrastats >/dev/null 2>&1; then
    timeout 3 tegrastats --interval 1000 2>/dev/null | sed -n '1,3p' | sed 's/^/[pointcloud-cpu]   tegrastats /' || true
  else
    echo "[pointcloud-cpu]   tegrastats unavailable"
  fi

  if command -v nvpmodel >/dev/null 2>&1; then
    nvpmodel -q 2>/dev/null | sed -n '1,8p' | sed 's/^/[pointcloud-cpu]   /' || true
  else
    echo "[pointcloud-cpu]   nvpmodel unavailable"
  fi

  if command -v jetson_clocks >/dev/null 2>&1; then
    timeout 5 jetson_clocks --show 2>/dev/null | sed -n '1,16p' | sed 's/^/[pointcloud-cpu]   /' || true
  else
    echo "[pointcloud-cpu]   jetson_clocks unavailable"
  fi

  local zone
  for zone in /sys/devices/virtual/thermal/thermal_zone*; do
    [[ -d "${zone}" ]] || continue
    local name="unknown"
    local temp="unknown"
    [[ -r "${zone}/type" ]] && name="$(cat "${zone}/type" 2>/dev/null || echo unknown)"
    if [[ -r "${zone}/temp" ]]; then
      temp="$(awk '{printf "%.1fC", $1 / 1000.0}' "${zone}/temp" 2>/dev/null || echo unknown)"
    fi
    echo "[pointcloud-cpu]   thermal ${name}=${temp}"
  done

  local cpu
  for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
    [[ -d "${cpu}/cpufreq" ]] || continue
    local core="${cpu##*/}"
    local cur="unknown"
    local max="unknown"
    local governor="unknown"
    [[ -r "${cpu}/cpufreq/scaling_cur_freq" ]] && cur="$(cat "${cpu}/cpufreq/scaling_cur_freq" 2>/dev/null || echo unknown)"
    [[ -r "${cpu}/cpufreq/scaling_max_freq" ]] && max="$(cat "${cpu}/cpufreq/scaling_max_freq" 2>/dev/null || echo unknown)"
    [[ -r "${cpu}/cpufreq/scaling_governor" ]] && governor="$(cat "${cpu}/cpufreq/scaling_governor" 2>/dev/null || echo unknown)"
    echo "[pointcloud-cpu]   ${core} cur_khz=${cur} max_khz=${max} governor=${governor}"
  done
}

echo "[pointcloud-cpu] profile=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}"
echo "[pointcloud-cpu] resolved local input=${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC}"
echo "[pointcloud-cpu] resolved axis local_output_topic=${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC:-<disabled>}"
echo "[pointcloud-cpu] current NJRH_CPUSET_* values:"
env | awk -F= '/^NJRH_CPUSET_/ {print "[pointcloud-cpu]   " $1 "=" $2}' | sort

print_threads_for_pattern "pointcloud_axis_remap" "pointcloud_axis_remap"
print_threads_for_pattern "robot_local_perception" "local_perception_node|robot_local_perception"
print_threads_for_pattern "nav_cloud_preprocessor" "nav_cloud_preprocessor"
print_threads_for_pattern "pointcloud_to_laserscan" "pointcloud_to_laserscan"
print_threads_for_pattern "scan_republisher" "scan_republisher_node"
print_threads_for_pattern "global_localization/localizer" "robot_global_localization|occupancy_grid_localizer|isaac.*localizer"

echo "[pointcloud-cpu] per-core utilization snapshot:"
if command -v mpstat >/dev/null 2>&1; then
  mpstat -P ALL 1 1 | sed 's/^/[pointcloud-cpu]   /'
else
  top -b -n1 | sed -n '1,12p' | sed 's/^/[pointcloud-cpu]   /'
fi

print_thermal_and_clock_snapshot

local_status="$(status_once /perception/local_perception_status)"
local_input_topic="$(field_value "${local_status}" input_topic || true)"
local_input_hz="$(field_value "${local_status}" input_callback_hz || true)"
echo "[pointcloud-cpu] local input_topic=${local_input_topic:-missing} input_callback_hz=${local_input_hz:-missing}"

local_cpus="$(cpus_for_pattern "local_perception_node|robot_local_perception")"
nav_cpus="$(cpus_for_pattern "nav_cloud_preprocessor")"
laser_cpus="$(cpus_for_pattern "pointcloud_to_laserscan")"
localizer_cpus="$(cpus_for_pattern "robot_global_localization|occupancy_grid_localizer|isaac.*localizer")"

if [[ "${local_input_topic:-}" == "${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC}" && -n "${local_input_hz:-}" ]] &&
  awk -v value="${local_input_hz}" 'BEGIN {exit(value + 0.0 < 10.0 ? 0 : 1)}'
then
  if [[ -n "${local_cpus}" && "${local_cpus}" == "${nav_cpus}" && "${local_cpus}" == "${laser_cpus}" && "${local_cpus}" == "${localizer_cpus}" ]]; then
    echo "[pointcloud-cpu] WARN local_branch is enabled but local input <10Hz and pointcloud/localizer workers are concentrated on CPU ${local_cpus}"
  else
    echo "[pointcloud-cpu] WARN local_branch is enabled but local input <10Hz; inspect CPU contention and DDS transport"
  fi
fi

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

SAMPLE_SECONDS="${NJRH_POINTCLOUD_CPU_PRESSURE_SECONDS:-12}"
STATUS_SECONDS="${NJRH_POINTCLOUD_CPU_STATUS_SECONDS:-6}"

tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_cpu_pressure_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

float_ge() {
  awk -v value="${1:-nan}" -v minimum="${2:-0}" 'BEGIN {exit(value + 0.0 >= minimum + 0.0 ? 0 : 1)}'
}

float_lt() {
  awk -v value="${1:-nan}" -v limit="${2:-0}" 'BEGIN {exit(value + 0.0 < limit + 0.0 ? 0 : 1)}'
}

field_latest() {
  local text="$1"
  local key="$2"
  printf '%s\n' "${text}" | tr ' ' '\n' |
    awk -F= -v key="${key}" '$1 == key {print substr($0, length(key) + 2); exit}'
}

status_once() {
  local topic="$1"
  local raw_file="${tmp_dir}/$(basename "${topic}").raw"
  timeout "${STATUS_SECONDS}" ros2 topic echo "${topic}" --field data >"${raw_file}" 2>&1 || true
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
    echo "[pointcloud-cpu-pressure] ${label}: not running"
    return 0
  fi
  echo "[pointcloud-cpu-pressure] ${label}: pids=${pids}"
  ps -T -p "${pid_csv}" -o pid,tid,psr,pcpu,pri,ni,comm --no-headers 2>/dev/null |
    awk -v label="${label}" '{printf "[pointcloud-cpu-pressure]   %-32s pid=%s tid=%s cpu=%s pcpu=%s pri=%s ni=%s comm=%s\n", label, $1, $2, $3, $4, $5, $6, $7}'
}

cpus_for_pattern() {
  local pattern="$1"
  local pids
  pids="$(pid_csv_for_pattern "${pattern}")"
  [[ -n "${pids}" ]] || return 0
  ps -T -p "${pids}" -o psr --no-headers 2>/dev/null |
    awk 'NF {seen[$1]=1} END {for (cpu in seen) print cpu}' | sort -n | paste -sd ',' -
}

sets_overlap() {
  local lhs="$1"
  local rhs="$2"
  awk -v lhs="${lhs}" -v rhs="${rhs}" '
    BEGIN {
      split(lhs, a, ",")
      split(rhs, b, ",")
      for (i in a) seen[a[i]] = 1
      for (i in b) {
        if (b[i] in seen) exit 0
      }
      exit 1
    }'
}

echo "[pointcloud-cpu-pressure] This script only observes CPU/thermal/process state."
echo "[pointcloud-cpu-pressure] sample_seconds=${SAMPLE_SECONDS}"
echo "[pointcloud-cpu-pressure] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}"
echo "[pointcloud-cpu-pressure] NJRH_CPU_AFFINITY_ENABLED=${NJRH_CPU_AFFINITY_ENABLED:-unset}"
echo "[pointcloud-cpu-pressure] current NJRH_CPUSET_* values:"
env | awk -F= '/^NJRH_CPUSET_/ {print "[pointcloud-cpu-pressure]   " $1 "=" $2}' | sort

echo "[pointcloud-cpu-pressure] memory and swap:"
free -h 2>/dev/null | sed 's/^/[pointcloud-cpu-pressure]   /' || true

echo "[pointcloud-cpu-pressure] nvpmodel:"
if command -v nvpmodel >/dev/null 2>&1; then
  nvpmodel -q 2>/dev/null | sed -n '1,12p' | sed 's/^/[pointcloud-cpu-pressure]   /' || true
else
  echo "[pointcloud-cpu-pressure]   unavailable"
fi

echo "[pointcloud-cpu-pressure] jetson_clocks:"
if command -v jetson_clocks >/dev/null 2>&1; then
  timeout 6 jetson_clocks --show 2>/dev/null | sed -n '1,18p' | sed 's/^/[pointcloud-cpu-pressure]   /' || true
else
  echo "[pointcloud-cpu-pressure]   unavailable"
fi

echo "[pointcloud-cpu-pressure] tegrastats:"
tegrastats_file="${tmp_dir}/tegrastats.txt"
if command -v tegrastats >/dev/null 2>&1; then
  timeout "$((SAMPLE_SECONDS + 2))" tegrastats --interval 1000 >"${tegrastats_file}" 2>/dev/null || true
  sed 's/^/[pointcloud-cpu-pressure]   /' "${tegrastats_file}" || true
else
  echo "[pointcloud-cpu-pressure]   unavailable"
fi

echo "[pointcloud-cpu-pressure] thermal zones:"
for zone in /sys/devices/virtual/thermal/thermal_zone*; do
  [[ -d "${zone}" ]] || continue
  name="$(cat "${zone}/type" 2>/dev/null || echo unknown)"
  temp="unknown"
  if [[ -r "${zone}/temp" ]]; then
    temp="$(awk '{printf "%.1fC", $1 / 1000.0}' "${zone}/temp" 2>/dev/null || echo unknown)"
  fi
  echo "[pointcloud-cpu-pressure]   ${name}=${temp}"
done

echo "[pointcloud-cpu-pressure] CPU frequencies:"
for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
  [[ -d "${cpu}/cpufreq" ]] || continue
  core="${cpu##*/}"
  cur="$(cat "${cpu}/cpufreq/scaling_cur_freq" 2>/dev/null || echo unknown)"
  max="$(cat "${cpu}/cpufreq/scaling_max_freq" 2>/dev/null || echo unknown)"
  governor="$(cat "${cpu}/cpufreq/scaling_governor" 2>/dev/null || echo unknown)"
  echo "[pointcloud-cpu-pressure]   ${core} cur_khz=${cur} max_khz=${max} governor=${governor}"
done

echo "[pointcloud-cpu-pressure] pointcloud and navigation thread placement:"
print_threads_for_pattern "pointcloud_axis_remap" "pointcloud_axis_remap"
print_threads_for_pattern "robot_local_perception" "local_perception_node|robot_local_perception"
print_threads_for_pattern "nav_cloud_preprocessor" "nav_cloud_preprocessor"
print_threads_for_pattern "pointcloud_to_laserscan" "pointcloud_to_laserscan"
print_threads_for_pattern "scan_republisher" "scan_republisher_node"
print_threads_for_pattern "laser_scan_to_flatscan" "laser_scan_to_flatscan"
print_threads_for_pattern "global_localization/localizer" "robot_global_localization|occupancy_grid_localizer|isaac.*localizer"
print_threads_for_pattern "controller/costmap" "controller_server|local_costmap|global_costmap|collision_monitor"

echo "[pointcloud-cpu-pressure] per-core CPU usage:"
mpstat_file="${tmp_dir}/mpstat.txt"
if command -v mpstat >/dev/null 2>&1; then
  mpstat -P ALL 1 "${SAMPLE_SECONDS}" >"${mpstat_file}" 2>/dev/null || true
  sed 's/^/[pointcloud-cpu-pressure]   /' "${mpstat_file}"
else
  top -b -d 1 -n 2 | sed -n '1,30p' | sed 's/^/[pointcloud-cpu-pressure]   /' || true
fi

warnings=0
axis_cpus="$(cpus_for_pattern "pointcloud_axis_remap")"
local_cpus="$(cpus_for_pattern "local_perception_node|robot_local_perception")"
nav_cpus="$(cpus_for_pattern "nav_cloud_preprocessor|pointcloud_to_laserscan|scan_republisher_node|laser_scan_to_flatscan")"
localizer_cpus="$(cpus_for_pattern "robot_global_localization|occupancy_grid_localizer|isaac.*localizer")"
nav2_cpus="$(cpus_for_pattern "controller_server|local_costmap|global_costmap|collision_monitor")"

if [[ -n "${local_cpus}" && -n "${nav_cpus}" ]] && sets_overlap "${local_cpus}" "${nav_cpus}"; then
  echo "[pointcloud-cpu-pressure] WARN robot_local_perception overlaps nav scan chain on CPU(s): local=${local_cpus} nav=${nav_cpus}"
  warnings=$((warnings + 1))
fi

if [[ -n "${local_cpus}" && -n "${localizer_cpus}" ]] && sets_overlap "${local_cpus}" "${localizer_cpus}"; then
  echo "[pointcloud-cpu-pressure] WARN robot_local_perception overlaps localization/localizer on CPU(s): local=${local_cpus} localizer=${localizer_cpus}"
  warnings=$((warnings + 1))
fi

if [[ -n "${axis_cpus}" && -n "${nav2_cpus}" ]] && sets_overlap "${axis_cpus}" "${nav2_cpus}"; then
  echo "[pointcloud-cpu-pressure] WARN pointcloud_axis_remap CPU overlaps Nav2/controller/costmap work: axis=${axis_cpus} nav2=${nav2_cpus}"
  warnings=$((warnings + 1))
fi

if [[ -s "${mpstat_file}" ]]; then
  hot_cpus="$(
    awk '
      /^Average:/ && $2 ~ /^[0-9]+$/ {
        usage = 100.0 - $NF
        if (usage > 85.0) {
          printf "%s(%.1f%%) ", $2, usage
        }
      }
    ' "${mpstat_file}"
  )"
  if [[ -n "${hot_cpus}" ]]; then
    echo "[pointcloud-cpu-pressure] WARN per-core CPU usage above 85%: ${hot_cpus}"
    warnings=$((warnings + 1))
  fi
fi

if [[ -s "${tegrastats_file}" ]] && grep -Eiq 'throt|throttle' "${tegrastats_file}"; then
  echo "[pointcloud-cpu-pressure] WARN tegrastats mentions throttling"
  warnings=$((warnings + 1))
fi

swap_used_mb="$(free -m 2>/dev/null | awk '/^Swap:/ {print $3; exit}')"
if [[ -n "${swap_used_mb}" ]] && awk -v value="${swap_used_mb}" 'BEGIN {exit(value > 256 ? 0 : 1)}'; then
  echo "[pointcloud-cpu-pressure] WARN swap in use: ${swap_used_mb} MiB"
  warnings=$((warnings + 1))
fi

local_status="$(status_once /perception/local_perception_status)"
local_processing_avg="$(field_latest "${local_status}" processing_ms_avg || true)"
local_timer_hz="$(field_latest "${local_status}" timer_tick_hz || field_latest "${local_status}" process_timer_hz || true)"
local_processed_hz="$(field_latest "${local_status}" processed_cloud_hz || true)"
if [[ -n "${local_processing_avg}" && -n "${local_processed_hz}" ]] &&
  float_lt "${local_processing_avg}" 10.0 && float_lt "${local_processed_hz}" 10.0
then
  echo "[pointcloud-cpu-pressure] WARN local perception processing_ms_avg is low but processed_hz is low: processing_ms_avg=${local_processing_avg} timer_hz=${local_timer_hz:-missing} processed_hz=${local_processed_hz}"
  warnings=$((warnings + 1))
fi

echo "[pointcloud-cpu-pressure] WARN_COUNT=${warnings}"
echo "[pointcloud-cpu-pressure] local_cpus=${local_cpus:-missing} nav_cpus=${nav_cpus:-missing} localizer_cpus=${localizer_cpus:-missing} axis_cpus=${axis_cpus:-missing}"

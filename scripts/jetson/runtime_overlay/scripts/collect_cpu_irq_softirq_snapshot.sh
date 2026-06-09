#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-/workspaces/njrh-v3/workspace1}"
REPORT_DIR="${PROJECT_ROOT}/reports"

DURATION_SEC=20
INTERVAL_SEC=1
OUTPUT_FILE=""

usage() {
  cat <<'EOF'
Usage: collect_cpu_irq_softirq_snapshot.sh [--duration-sec N] [--interval-sec N] [--output PATH]

Read-only Jetson CPU/IRQ/softirq snapshot. This script does not change system
settings and does not subscribe to PointCloud2 topics.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --interval-sec)
      INTERVAL_SEC="${2:-}"
      shift 2
      ;;
    --output)
      OUTPUT_FILE="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[cpu-irq-snapshot] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*) echo "[cpu-irq-snapshot] --duration-sec must be an integer" >&2; exit 2 ;;
esac
case "${INTERVAL_SEC}" in
  ''|*[!0-9]*) echo "[cpu-irq-snapshot] --interval-sec must be an integer" >&2; exit 2 ;;
esac
[[ "${DURATION_SEC}" -gt 0 && "${INTERVAL_SEC}" -gt 0 ]] || {
  echo "[cpu-irq-snapshot] duration and interval must be positive" >&2
  exit 2
}

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "${REPORT_DIR}"
if [[ -z "${OUTPUT_FILE}" ]]; then
  OUTPUT_FILE="${REPORT_DIR}/cpu_irq_softirq_snapshot_${timestamp}.md"
fi

tmp_dir="$(mktemp -d /tmp/njrh_cpu_irq_snapshot_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

interval_ms=$((INTERVAL_SEC * 1000))
tegrastats_file="${tmp_dir}/tegrastats.txt"
threads_start="${tmp_dir}/threads_start.txt"
threads_end="${tmp_dir}/threads_end.txt"
interrupts_start="${tmp_dir}/interrupts_start.txt"
interrupts_end="${tmp_dir}/interrupts_end.txt"
softirqs_start="${tmp_dir}/softirqs_start.txt"
softirqs_end="${tmp_dir}/softirqs_end.txt"

safe_run() {
  "$@" 2>&1 || true
}

capture_threads() {
  ps -eLo pid,tid,psr,pcpu,pri,ni,cls,rtprio,comm,args --sort=psr,-pcpu 2>/dev/null || true
}

cat /proc/interrupts >"${interrupts_start}" 2>/dev/null || true
cat /proc/softirqs >"${softirqs_start}" 2>/dev/null || true
capture_threads >"${threads_start}"

if command -v tegrastats >/dev/null 2>&1; then
  timeout "$((DURATION_SEC + INTERVAL_SEC + 2))" tegrastats --interval "${interval_ms}" >"${tegrastats_file}" 2>&1 || true
else
  : >"${tegrastats_file}"
fi

cat /proc/interrupts >"${interrupts_end}" 2>/dev/null || true
cat /proc/softirqs >"${softirqs_end}" 2>/dev/null || true
capture_threads >"${threads_end}"

project_thread_summary() {
  awk '
    BEGIN {
      pat = "/workspaces/njrh-v3/workspace1|/workspaces/isaac_ros-dev/install/hesai_ros_driver|/workspaces/isaac_ros-dev/ros2_ws/install/ranger_base|/opt/ros/humble/lib/robot_localization|/opt/ros/humble/lib/pointcloud_to_laserscan|/opt/ros/humble/lib/nav2_|component_container_mt|occupancy_grid_localizer|standard_navigation.launch.py|occupancy_localization_stack.launch.py"
      printf "| CPU | PID | Threads | Sum %%CPU | Process |\n"
      printf "|---|---:|---:|---:|---|\n"
    }
    NR > 1 && $0 ~ pat {
      pid=$1; cpu=$3; pcpu=$4; comm=$9
      key=cpu SUBSEP pid SUBSEP comm
      count[key] += 1
      sum[key] += pcpu
    }
    END {
      for (key in count) {
        split(key, k, SUBSEP)
        printf "| CPU%s | %s | %d | %.1f | %s |\n", k[1], k[2], count[key], sum[key], k[3]
      }
    }
  ' "${threads_end}"
}

tegrastats_summary() {
  awk '
    /CPU \[/ {
      line=$0
      sub(/^.*CPU \[/, "", line)
      sub(/\].*$/, "", line)
      n=split(line, parts, ",")
      for (i=1; i<=n; i++) {
        gsub(/[[:space:]]/, "", parts[i])
        if (parts[i] ~ /^[0-9]+%@[-0-9]+/) {
          split(parts[i], m, "%@")
          cpu=i-1
          count[cpu]++
          usage_sum[cpu]+=m[1]
          mhz_sum[cpu]+=m[2]
          if (!(cpu in usage_min) || m[1] < usage_min[cpu]) usage_min[cpu]=m[1]
          if (!(cpu in usage_max) || m[1] > usage_max[cpu]) usage_max[cpu]=m[1]
        }
      }
    }
    END {
      printf "| CPU | Samples | Avg usage | Min | Max | Avg MHz |\n"
      printf "|---|---:|---:|---:|---:|---:|\n"
      for (cpu=0; cpu<8; cpu++) {
        if (count[cpu] > 0) {
          printf "| CPU%d | %d | %.1f%% | %.0f%% | %.0f%% | %.0f |\n", cpu, count[cpu], usage_sum[cpu]/count[cpu], usage_min[cpu], usage_max[cpu], mhz_sum[cpu]/count[cpu]
        }
      }
    }
  ' "${tegrastats_file}"
}

softirq_delta() {
  local name="$1"
  awk -v target="${name}:" '
    NR == FNR {
      if ($1 == target) {
        for (i=2; i<=NF; i++) before[i-2]=$i
      }
      next
    }
    $1 == target {
      printf "| CPU | Delta |\n|---|---:|\n"
      for (i=2; i<=NF; i++) printf "| CPU%d | %d |\n", i-2, $i - before[i-2]
    }
  ' "${softirqs_start}" "${softirqs_end}"
}

net_interrupt_candidates() {
  awk 'tolower($0) ~ /(eth|wlan|enp|eno|end|eqos|ether|net|r8169|igb|e1000|mlx|nvethernet|rtl)/ {print}' "${interrupts_end}" || true
}

irq_affinity_snapshot() {
  while IFS= read -r line; do
    irq="$(printf '%s\n' "${line}" | awk -F: '{gsub(/[[:space:]]/, "", $1); print $1}')"
    [[ -n "${irq}" && -r "/proc/irq/${irq}/smp_affinity_list" ]] || continue
    printf 'IRQ %s affinity=%s line=%s\n' "${irq}" "$(cat "/proc/irq/${irq}/smp_affinity_list" 2>/dev/null || true)" "${line}"
  done < <(net_interrupt_candidates)
}

interrupt_delta_table() {
  awk '
    NR == FNR {
      if ($1 ~ /^[0-9]+:/) {
        irq=$1; gsub(":", "", irq)
        for (i=2; i<=NF && $i ~ /^[0-9]+$/; i++) before[irq, i-2]=$i
        label[irq]=$0
      }
      next
    }
    $1 ~ /^[0-9]+:/ {
      irq=$1; gsub(":", "", irq)
      lower=tolower($0)
      if (lower !~ /(eth|wlan|enp|eno|end|eqos|ether|net|r8169|igb|e1000|mlx|nvethernet|rtl)/) next
      total=0
      detail=""
      for (i=2; i<=NF && $i ~ /^[0-9]+$/; i++) {
        d=$i - before[irq, i-2]
        total += d
        detail=detail sprintf(" CPU%d=%d", i-2, d)
      }
      printf "| %s | %d |%s | %s |\n", irq, total, detail, $0
    }
  ' "${interrupts_start}" "${interrupts_end}"
}

cpu0_project_sum="$(
  awk '
    NR > 1 && $3 == 0 && $0 ~ /\/workspaces\/njrh-v3\/workspace1|\/opt\/ros\/humble\/lib\/nav2_|\/opt\/ros\/humble\/lib\/robot_localization|\/workspaces\/isaac_ros-dev/ {
      sum += $4
    }
    END {printf "%.1f", sum + 0.0}
  ' "${threads_end}"
)"
cpu0_avg="$(
  awk '
    /CPU \[/ {
      line=$0; sub(/^.*CPU \[/, "", line); sub(/\].*$/, "", line); split(line, parts, ",")
      gsub(/[[:space:]]/, "", parts[1])
      if (parts[1] ~ /^[0-9]+%@/) {split(parts[1], m, "%@"); sum += m[1]; count++}
    }
    END {if (count) printf "%.1f", sum/count; else printf "nan"}
  ' "${tegrastats_file}"
)"
ksoftirqd0_pcpu="$(
  awk '$9 ~ /^ksoftirqd\/0$/ {sum += $4} END {printf "%.1f", sum + 0.0}' "${threads_end}"
)"
net_rx_cpu0_delta="$(
  awk '
    NR == FNR {if ($1 == "NET_RX:") before=$2; next}
    $1 == "NET_RX:" {print $2 - before}
  ' "${softirqs_start}" "${softirqs_end}" 2>/dev/null || true
)"
cpu6_pointcloud_count="$(
  awk '
    NR > 1 && $3 == 6 && $0 ~ /local_perception|nav_cloud_preprocessor|pointcloud_to_laserscan|scan_republisher|laser_scan_to_flatscan|occupancy_grid_localizer|global_localization/ {
      seen[$1 ":" $9]=1
    }
    END {for (k in seen) n++; print n+0}
  ' "${threads_end}"
)"
cpu7_project_sum="$(
  awk '
    NR > 1 && $3 == 7 && $0 ~ /\/workspaces\/njrh-v3\/workspace1|\/opt\/ros\/humble\/lib|\/workspaces\/isaac_ros-dev/ {sum += $4}
    END {printf "%.1f", sum + 0.0}
  ' "${threads_end}"
)"

{
  cat <<EOF
# CPU / IRQ / SoftIRQ Snapshot

- timestamp_utc: ${timestamp}
- duration_sec: ${DURATION_SEC}
- interval_sec: ${INTERVAL_SEC}
- host: $(hostname 2>/dev/null || echo unknown)
- script: collect_cpu_irq_softirq_snapshot.sh
- mode: read-only

## Conclusions

- CPU0 tegrastats average: ${cpu0_avg}%.
- CPU0 project-visible thread sum from ps snapshot: ${cpu0_project_sum}%.
- CPU0 high load explained by project threads: $(awk -v total="${cpu0_avg}" -v proj="${cpu0_project_sum}" 'BEGIN {if (total != total) print "unknown"; else if (proj + 20 >= total) print "mostly"; else print "not fully"}').
- ksoftirqd/0 pcpu from ps snapshot: ${ksoftirqd0_pcpu}%.
- ksoftirqd/0 significant: $(awk -v metric="${ksoftirqd0_pcpu}" 'BEGIN {if (metric >= 5.0) print "yes"; else print "no"}').
- NET_RX CPU0 delta over sample: ${net_rx_cpu0_delta:-unknown}.
- CPU6 pointcloud/localization process groups present: ${cpu6_pointcloud_count}.
- CPU6 concentrated pointcloud/localizer chain: $(awk -v metric="${cpu6_pointcloud_count}" 'BEGIN {if (metric >= 4) print "yes"; else print "no"}').
- CPU7 visible project thread sum: ${cpu7_project_sum}%.
- CPU7 available for migration: $(awk -v metric="${cpu7_project_sum}" 'BEGIN {print metric < 20.0 ? "likely" : "busy"}').

Use identify_lidar_network_irq.sh for interface-specific LiDAR IRQ attribution before applying any IRQ/RPS/XPS profile.

## Per-Core tegrastats Summary

$(tegrastats_summary)

## Raw tegrastats

\`\`\`text
$(cat "${tegrastats_file}")
\`\`\`

## Memory

\`\`\`text
$(safe_run free -h)
\`\`\`

## RMW / FastDDS Environment

\`\`\`text
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}
FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-unset}
FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}
FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-unset}
NJRH_FASTDDS_PROFILE_ENABLED=${NJRH_FASTDDS_PROFILE_ENABLED:-unset}
\`\`\`

## NJRH_CPUSET Configuration

\`\`\`text
$(env | awk -F= '/^NJRH_CPUSET_/ {print $1 "=" $2}' | sort)
$(if [[ -f "${OVERLAY_ROOT}/config/cpu_affinity.env" ]]; then grep '^export NJRH_CPUSET_' "${OVERLAY_ROOT}/config/cpu_affinity.env" || true; fi)
$(if [[ -f "${OVERLAY_ROOT}/config/cpu_affinity_runtime_override.env" ]]; then echo "# runtime override"; cat "${OVERLAY_ROOT}/config/cpu_affinity_runtime_override.env"; fi)
\`\`\`

## Project Thread Summary By CPU

$(project_thread_summary)

## Thread Snapshot End

\`\`\`text
$(cat "${threads_end}")
\`\`\`

## ksoftirqd Threads

\`\`\`text
$(grep -E 'ksoftirqd/[0-9]+' "${threads_end}" || true)
\`\`\`

## irq Threads

\`\`\`text
$(grep -E '[[:space:]]irq/[0-9]+' "${threads_end}" || true)
\`\`\`

## NET_RX SoftIRQ Delta

$(softirq_delta NET_RX)

## TIMER SoftIRQ Delta

$(softirq_delta TIMER)

## Network Interrupt Candidates

\`\`\`text
$(net_interrupt_candidates)
\`\`\`

## Network Interrupt Delta Candidates

| IRQ | Total delta | Per-core delta | Line |
|---|---:|---|---|
$(interrupt_delta_table)

## Candidate IRQ Affinity

\`\`\`text
$(irq_affinity_snapshot)
\`\`\`

## RPS / XPS

\`\`\`text
$(for file in /sys/class/net/*/queues/rx-*/rps_cpus /sys/class/net/*/queues/tx-*/xps_cpus; do [[ -e "${file}" ]] && printf '%s=%s\n' "${file}" "$(cat "${file}" 2>/dev/null || true)"; done)
\`\`\`

## Network Interfaces

\`\`\`text
$(safe_run ip -br addr)
\`\`\`

## Routes

\`\`\`text
$(safe_run ip route)
\`\`\`

## ethtool

\`\`\`text
$(if command -v ethtool >/dev/null 2>&1; then for iface in /sys/class/net/*; do iface="${iface##*/}"; echo "### ${iface}"; ethtool -i "${iface}" 2>/dev/null || true; ethtool -S "${iface}" 2>/dev/null | sed -n '1,80p' || true; done; else echo "ethtool unavailable"; fi)
\`\`\`

## /proc/interrupts Start

\`\`\`text
$(cat "${interrupts_start}")
\`\`\`

## /proc/interrupts End

\`\`\`text
$(cat "${interrupts_end}")
\`\`\`

## /proc/softirqs Start

\`\`\`text
$(cat "${softirqs_start}")
\`\`\`

## /proc/softirqs End

\`\`\`text
$(cat "${softirqs_end}")
\`\`\`

## Scheduling Class Snapshot

\`\`\`text
$(ps -eLo cls,rtprio,pri,ni,psr,pcpu,comm,args --sort=psr,-pcpu 2>/dev/null | sed -n '1,120p' || true)
\`\`\`
EOF
} >"${OUTPUT_FILE}"

echo "[cpu-irq-snapshot] report=${OUTPUT_FILE}"

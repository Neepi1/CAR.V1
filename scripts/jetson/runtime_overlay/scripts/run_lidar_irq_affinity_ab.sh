#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-/workspaces/njrh-v3/workspace1}"
STATE_DIR="${NJRH_LIDAR_IRQ_AB_STATE_DIR:-/tmp/njrh_lidar_irq_affinity_ab}"

PROFILE="irq_keep_default"
INTERFACE=""
DURATION_SEC=10
APPLY=false
RESTORE=false
PRINT=false
ALLOW_SSH_INTERFACE_RISK=false
RUN_DIAGNOSTICS=true

usage() {
  cat <<'EOF'
Usage: run_lidar_irq_affinity_ab.sh [--profile irq_keep_default|lidar_irq_cpu5|lidar_irq_cpu7|lidar_irq_split_5_7|rps_xps_cpu5|rps_xps_5_7] [--interface IFACE] [--duration-sec N] [--print] [--apply] [--restore] [--allow-ssh-interface-risk] [--no-diagnostics]

Default mode is dry-run. --apply modifies only LiDAR interface IRQ affinity and,
for rps_xps_* profiles, that interface's RPS/XPS masks. RPS-only profiles do
not write IRQ affinity. --restore restores the backup captured before the last
--apply. This script does not change ROS QoS, DDS, timestamps, Nav2, EKF,
FAST-LIO2, or ROS processes.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --interface)
      INTERFACE="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --print)
      PRINT=true
      shift
      ;;
    --apply)
      APPLY=true
      shift
      ;;
    --restore)
      RESTORE=true
      shift
      ;;
    --allow-ssh-interface-risk)
      ALLOW_SSH_INTERFACE_RISK=true
      shift
      ;;
    --no-diagnostics)
      RUN_DIAGNOSTICS=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[lidar-irq-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*) echo "[lidar-irq-ab] --duration-sec must be an integer" >&2; exit 2 ;;
esac
[[ "${DURATION_SEC}" -ge 0 ]] || {
  echo "[lidar-irq-ab] --duration-sec must be non-negative" >&2
  exit 2
}

case "${PROFILE}" in
  irq_keep_default|lidar_irq_cpu5|lidar_irq_cpu7|lidar_irq_split_5_7|rps_xps_cpu5|rps_xps_5_7)
    ;;
  *)
    echo "[lidar-irq-ab] unsupported profile: ${PROFILE}" >&2
    exit 2
    ;;
esac

safe_run() {
  "$@" 2>&1 || true
}

driver_config_path() {
  pgrep -af "hesai_ros_driver_node" 2>/dev/null |
    sed -n 's/.*config_path:=\([^ ]*\).*/\1/p' |
    tail -n 1
}

host_ipv4_set() {
  ip -o -4 addr show 2>/dev/null | awk '{split($4, a, "/"); print a[1]}'
}

candidate_lidar_ips() {
  local cfg="$1"
  [[ -n "${cfg}" && -r "${cfg}" ]] || return 0
  local host_ips
  host_ips="$(host_ipv4_set | tr '\n' ' ')"
  grep -Eo '([0-9]{1,3}\.){3}[0-9]{1,3}' "${cfg}" 2>/dev/null |
    awk -v hosts="${host_ips}" '
      $0 != "0.0.0.0" && $0 != "127.0.0.1" && hosts !~ "(^| )" $0 "( |$)" {print}
    ' |
    awk '!seen[$0]++'
}

infer_interface_from_config() {
  local cfg="$1"
  local ip_addr route_line iface
  while IFS= read -r ip_addr; do
    [[ -n "${ip_addr}" ]] || continue
    route_line="$(ip route get "${ip_addr}" 2>/dev/null || true)"
    iface="$(printf '%s\n' "${route_line}" | awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}')"
    if [[ -n "${iface}" ]]; then
      printf '%s\n' "${iface}"
      return 0
    fi
  done < <(candidate_lidar_ips "${cfg}")
  return 1
}

ssh_interface() {
  local remote_ip=""
  if [[ -n "${SSH_CLIENT:-}" ]]; then
    remote_ip="$(printf '%s\n' "${SSH_CLIENT}" | awk '{print $1}')"
  elif [[ -n "${SSH_CONNECTION:-}" ]]; then
    remote_ip="$(printf '%s\n' "${SSH_CONNECTION}" | awk '{print $1}')"
  fi
  if [[ -n "${remote_ip}" ]]; then
    ip route get "${remote_ip}" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}'
    return 0
  fi
  ip route show default 0.0.0.0/0 2>/dev/null | awk '{print $5; exit}'
}

iface_driver() {
  local iface="$1"
  if command -v ethtool >/dev/null 2>&1; then
    ethtool -i "${iface}" 2>/dev/null | awk -F': ' '$1 == "driver" {print $2; exit}'
  fi
}

matching_interrupt_lines() {
  local iface="$1"
  local driver="$2"
  local pattern="${iface}"
  [[ -n "${driver}" ]] && pattern="${pattern}|${driver}"
  grep -Ei "${pattern}" /proc/interrupts 2>/dev/null || true
}

matching_irqs() {
  matching_interrupt_lines "$1" "$2" |
    awk -F: '$1 ~ /^[[:space:]]*[0-9]+$/ {gsub(/[[:space:]]/, "", $1); print $1}' |
    awk '!seen[$0]++'
}

cpulist_to_hex_mask() {
  local cpulist="$1"
  local mask=0
  local part start end cpu
  IFS=',' read -ra parts <<<"${cpulist}"
  for part in "${parts[@]}"; do
    if [[ "${part}" == *-* ]]; then
      start="${part%-*}"
      end="${part#*-}"
      for ((cpu=start; cpu<=end; cpu++)); do
        mask=$((mask | (1 << cpu)))
      done
    else
      cpu="${part}"
      mask=$((mask | (1 << cpu)))
    fi
  done
  printf '%x\n' "${mask}"
}

profile_irq_cpulist() {
  local profile="$1"
  local index="$2"
  case "${profile}" in
    irq_keep_default)
      echo "unchanged"
      ;;
    lidar_irq_cpu5)
      echo "5"
      ;;
    lidar_irq_cpu7)
      echo "7"
      ;;
    lidar_irq_split_5_7)
      if (( index % 2 == 0 )); then
        echo "5"
      else
        echo "7"
      fi
      ;;
    rps_xps_cpu5|rps_xps_5_7)
      echo "unchanged"
      ;;
  esac
}

profile_rps_xps_cpulist() {
  case "$1" in
    rps_xps_cpu5)
      echo "5"
      ;;
    rps_xps_5_7)
      echo "5,7"
      ;;
    *)
      echo "unchanged"
      ;;
  esac
}

profile_changes_irq() {
  [[ "$(profile_irq_cpulist "$1" 0)" != "unchanged" ]]
}

if [[ "${RESTORE}" == "true" && -z "${INTERFACE}" && -f "${STATE_DIR}/manifest.env" ]]; then
  # shellcheck source=/tmp/njrh_lidar_irq_affinity_ab/manifest.env
  source "${STATE_DIR}/manifest.env"
  INTERFACE="${interface:-}"
fi

cfg="$(driver_config_path)"
if [[ -z "${INTERFACE}" ]]; then
  INTERFACE="$(infer_interface_from_config "${cfg}" 2>/dev/null || true)"
fi

if [[ "${RESTORE}" == "false" ]]; then
  if [[ -z "${INTERFACE}" ]]; then
    echo "[lidar-irq-ab] unable to infer LiDAR interface; pass --interface ethX." >&2
    echo "[lidar-irq-ab] driver_config=${cfg:-missing}" >&2
    exit 2
  fi
  if [[ ! -d "/sys/class/net/${INTERFACE}" ]]; then
    echo "[lidar-irq-ab] interface does not exist: ${INTERFACE}" >&2
    exit 2
  fi
fi

driver=""
ssh_iface=""
ssh_risk="false"
mapfile -t irq_list < <(true)
if [[ -n "${INTERFACE}" ]]; then
  driver="$(iface_driver "${INTERFACE}" || true)"
  ssh_iface="$(ssh_interface || true)"
  [[ -n "${ssh_iface}" && "${ssh_iface}" == "${INTERFACE}" ]] && ssh_risk="true"
  mapfile -t irq_list < <(matching_irqs "${INTERFACE}" "${driver:-}")
fi

backup_current() {
  mkdir -p "${STATE_DIR}"
  if [[ -f "${STATE_DIR}/manifest.env" ]]; then
    echo "[lidar-irq-ab] existing backup kept: ${STATE_DIR}"
    return 0
  fi
  {
    printf 'created_utc=%q\n' "$(date -u +%Y%m%dT%H%M%SZ)"
    printf 'profile=%q\n' "${PROFILE}"
    printf 'interface=%q\n' "${INTERFACE}"
    printf 'driver=%q\n' "${driver:-}"
    printf 'ssh_interface=%q\n' "${ssh_iface:-}"
  } >"${STATE_DIR}/manifest.env"

  local irq affinity
  if profile_changes_irq "${PROFILE}"; then
    : >"${STATE_DIR}/irq_smp_affinity_list.tsv"
    for irq in "${irq_list[@]}"; do
      [[ -r "/proc/irq/${irq}/smp_affinity_list" ]] || continue
      affinity="$(cat "/proc/irq/${irq}/smp_affinity_list" 2>/dev/null || true)"
      printf '%s\t%s\n' "${irq}" "${affinity}" >>"${STATE_DIR}/irq_smp_affinity_list.tsv"
    done
  fi

  : >"${STATE_DIR}/rps_xps.tsv"
  local file value
  for file in /sys/class/net/"${INTERFACE}"/queues/rx-*/rps_cpus /sys/class/net/"${INTERFACE}"/queues/tx-*/xps_cpus; do
    [[ -e "${file}" && -r "${file}" ]] || continue
    value="$(cat "${file}" 2>/dev/null || true)"
    printf '%s\t%s\n' "${file}" "${value}" >>"${STATE_DIR}/rps_xps.tsv"
  done
  echo "[lidar-irq-ab] backup captured: ${STATE_DIR}"
}

restore_previous() {
  if [[ ! -d "${STATE_DIR}" ]]; then
    echo "[lidar-irq-ab] no restore state found: ${STATE_DIR}" >&2
    exit 1
  fi
  local irq value file failed=0
  if [[ -f "${STATE_DIR}/irq_smp_affinity_list.tsv" ]]; then
    while IFS=$'\t' read -r irq value; do
      [[ -n "${irq}" && -w "/proc/irq/${irq}/smp_affinity_list" ]] || continue
      if printf '%s\n' "${value}" >"/proc/irq/${irq}/smp_affinity_list" 2>/dev/null; then
        echo "[lidar-irq-ab] restored IRQ ${irq} smp_affinity_list=${value}"
      else
        failed=$((failed + 1))
        echo "[lidar-irq-ab] warning: failed to restore IRQ ${irq} smp_affinity_list=${value}" >&2
      fi
    done <"${STATE_DIR}/irq_smp_affinity_list.tsv"
  fi
  if [[ -f "${STATE_DIR}/rps_xps.tsv" ]]; then
    while IFS=$'\t' read -r file value; do
      [[ -n "${file}" && -w "${file}" ]] || continue
      if printf '%s\n' "${value}" >"${file}" 2>/dev/null; then
        echo "[lidar-irq-ab] restored ${file}=${value}"
      else
        failed=$((failed + 1))
        echo "[lidar-irq-ab] warning: failed to restore ${file}=${value}" >&2
      fi
    done <"${STATE_DIR}/rps_xps.tsv"
  fi
  if [[ "${failed}" -eq 0 ]]; then
    rm -rf "${STATE_DIR}"
    echo "[lidar-irq-ab] restore complete"
  else
    echo "[lidar-irq-ab] restore finished with ${failed} write failure(s); backup kept: ${STATE_DIR}" >&2
  fi
}

apply_profile() {
  if [[ "${PROFILE}" == "irq_keep_default" ]]; then
    echo "[lidar-irq-ab] irq_keep_default selected; no IRQ/RPS/XPS changes"
    return 0
  fi
  local changes_irq="false"
  profile_changes_irq "${PROFILE}" && changes_irq="true"
  if [[ "${changes_irq}" == "true" && "${#irq_list[@]}" -eq 0 ]]; then
    echo "[lidar-irq-ab] no IRQs matched interface=${INTERFACE} driver=${driver:-unknown}; refusing --apply" >&2
    exit 1
  fi
  if [[ "${changes_irq}" == "true" && "${ssh_risk}" == "true" && "${ALLOW_SSH_INTERFACE_RISK}" != "true" ]]; then
    echo "[lidar-irq-ab] interface ${INTERFACE} appears to carry SSH/default route; refusing --apply without --allow-ssh-interface-risk" >&2
    exit 3
  fi
  backup_current

  local index=0 irq cpulist rps_cpulist rps_mask file
  for irq in "${irq_list[@]}"; do
    cpulist="$(profile_irq_cpulist "${PROFILE}" "${index}")"
    index=$((index + 1))
    [[ "${cpulist}" != "unchanged" ]] || continue
    [[ -w "/proc/irq/${irq}/smp_affinity_list" ]] || {
      echo "[lidar-irq-ab] cannot write /proc/irq/${irq}/smp_affinity_list" >&2
      continue
    }
    if printf '%s\n' "${cpulist}" >"/proc/irq/${irq}/smp_affinity_list" 2>/dev/null; then
      echo "[lidar-irq-ab] applied IRQ ${irq} smp_affinity_list=${cpulist}"
    else
      echo "[lidar-irq-ab] warning: failed to apply IRQ ${irq} smp_affinity_list=${cpulist}" >&2
    fi
  done

  rps_cpulist="$(profile_rps_xps_cpulist "${PROFILE}")"
  if [[ "${rps_cpulist}" != "unchanged" ]]; then
    rps_mask="$(cpulist_to_hex_mask "${rps_cpulist}")"
    for file in /sys/class/net/"${INTERFACE}"/queues/rx-*/rps_cpus /sys/class/net/"${INTERFACE}"/queues/tx-*/xps_cpus; do
      [[ -e "${file}" ]] || continue
      [[ -w "${file}" ]] || {
        echo "[lidar-irq-ab] cannot write ${file}" >&2
        continue
      }
      if printf '%s\n' "${rps_mask}" >"${file}" 2>/dev/null; then
        echo "[lidar-irq-ab] applied ${file}=${rps_mask} cpulist=${rps_cpulist}"
      else
        echo "[lidar-irq-ab] warning: failed to apply ${file}=${rps_mask} cpulist=${rps_cpulist}" >&2
      fi
    done
  fi
}

print_current() {
  echo "[lidar-irq-ab] profile=${PROFILE}"
  echo "[lidar-irq-ab] default_mode=dry-run"
  echo "[lidar-irq-ab] interface=${INTERFACE:-unknown}"
  echo "[lidar-irq-ab] driver=${driver:-unknown}"
  echo "[lidar-irq-ab] driver_config=${cfg:-missing}"
  echo "[lidar-irq-ab] candidate_lidar_ips=$(candidate_lidar_ips "${cfg:-}" | paste -sd ',' -)"
  echo "[lidar-irq-ab] ssh_default_interface=${ssh_iface:-unknown}"
  echo "[lidar-irq-ab] ssh_interface_risk=${ssh_risk}"
  echo "[lidar-irq-ab] irq_list=${irq_list[*]:-none}"
  echo "[lidar-irq-ab] proposed IRQ/RPS/XPS:"
  if [[ "${PROFILE}" == "irq_keep_default" ]]; then
    echo "[lidar-irq-ab]   no changes"
  else
    local index=0 irq cpulist rps_cpulist
    for irq in "${irq_list[@]}"; do
      cpulist="$(profile_irq_cpulist "${PROFILE}" "${index}")"
      echo "[lidar-irq-ab]   IRQ ${irq} -> ${cpulist}"
      index=$((index + 1))
    done
    rps_cpulist="$(profile_rps_xps_cpulist "${PROFILE}")"
    if [[ "${rps_cpulist}" != "unchanged" ]]; then
      echo "[lidar-irq-ab]   RPS/XPS ${INTERFACE} queues -> cpulist=${rps_cpulist} mask=$(cpulist_to_hex_mask "${rps_cpulist}")"
    else
      echo "[lidar-irq-ab]   RPS/XPS unchanged"
    fi
  fi
  echo "[lidar-irq-ab] matching_interrupt_lines:"
  if [[ -n "${INTERFACE}" ]]; then
    matching_interrupt_lines "${INTERFACE}" "${driver:-}" | sed 's/^/[lidar-irq-ab]   /' || true
  else
    echo "[lidar-irq-ab]   unavailable; interface not resolved"
  fi
  echo "[lidar-irq-ab] current affinity:"
  local irq
  for irq in "${irq_list[@]}"; do
    echo "[lidar-irq-ab]   IRQ ${irq} smp_affinity_list=$(cat "/proc/irq/${irq}/smp_affinity_list" 2>/dev/null || echo missing)"
  done
  echo "[lidar-irq-ab] current RPS/XPS:"
  if [[ -n "${INTERFACE}" ]]; then
    for file in /sys/class/net/"${INTERFACE}"/queues/rx-*/rps_cpus /sys/class/net/"${INTERFACE}"/queues/tx-*/xps_cpus; do
      [[ -e "${file}" ]] || continue
      echo "[lidar-irq-ab]   ${file}=$(cat "${file}" 2>/dev/null || true)"
    done
  else
    echo "[lidar-irq-ab]   unavailable; interface not resolved"
  fi
  echo "[lidar-irq-ab] will_not_change=QoS,DDS,timestamps,Nav2_planner_controller,EKF,FAST-LIO2,ROS_processes"
}

run_diagnostics() {
  local label="$1"
  [[ "${RUN_DIAGNOSTICS}" == "true" ]] || return 0
  echo "[lidar-irq-ab] diagnostics=${label}"
  if [[ "${DURATION_SEC}" -gt 0 && -n "${INTERFACE}" ]]; then
    bash "${SCRIPT_DIR}/identify_lidar_network_irq.sh" --interface "${INTERFACE}" --duration-sec "${DURATION_SEC}" || true
  elif [[ -z "${INTERFACE}" ]]; then
    echo "[lidar-irq-ab] identify skipped; interface not resolved"
  else
    print_current
  fi
  bash "${SCRIPT_DIR}/collect_cpu_irq_softirq_snapshot.sh" --duration-sec 20 || true
}

print_current

if [[ "${RESTORE}" == "true" ]]; then
  run_diagnostics "pre-restore"
  restore_previous
elif [[ "${APPLY}" == "true" ]]; then
  run_diagnostics "pre-apply"
  apply_profile
elif [[ "${PRINT}" == "false" ]]; then
  echo "[lidar-irq-ab] no --apply or --restore requested; printed plan only"
fi

if [[ "${APPLY}" == "true" || "${RESTORE}" == "true" ]]; then
  run_diagnostics "post-change"
fi

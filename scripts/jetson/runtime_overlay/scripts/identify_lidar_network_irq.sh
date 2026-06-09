#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC=10
INTERFACE=""

usage() {
  cat <<'EOF'
Usage: identify_lidar_network_irq.sh [--interface IFACE] [--duration-sec N]

Read-only LiDAR network interface and IRQ detector. It does not modify IRQ,
RPS, XPS, DDS, QoS, timestamps, or ROS processes.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interface)
      INTERFACE="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[lidar-irq-id] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*) echo "[lidar-irq-id] --duration-sec must be an integer" >&2; exit 2 ;;
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
  local ip_addr
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

iface_ipv4() {
  local iface="$1"
  ip -o -4 addr show dev "${iface}" 2>/dev/null | awk '{split($4, a, "/"); print a[1]}' | paste -sd ',' -
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
  [[ -n "${driver}" && "${driver}" != "unknown" ]] && pattern="${pattern}|${driver}"
  grep -Ei "${pattern}" /proc/interrupts 2>/dev/null || true
}

matching_irqs() {
  matching_interrupt_lines "$1" "$2" |
    awk -F: '$1 ~ /^[[:space:]]*[0-9]+$/ {gsub(/[[:space:]]/, "", $1); print $1}' |
    awk '!seen[$0]++'
}

interrupt_delta_for_irqs() {
  local start_file="$1"
  local end_file="$2"
  shift 2
  local irqs=("$@")
  [[ "${#irqs[@]}" -gt 0 ]] || return 0
  awk -v irq_list="${irqs[*]}" '
    BEGIN {
      split(irq_list, a, " ")
      for (i in a) wanted[a[i]]=1
      printf "| IRQ | Total delta | Per-core delta | End line |\n"
      printf "|---|---:|---|---|\n"
    }
    NR == FNR {
      if ($1 ~ /^[0-9]+:/) {
        irq=$1; gsub(":", "", irq)
        if (irq in wanted) {
          for (i=2; i<=NF && $i ~ /^[0-9]+$/; i++) before[irq, i-2]=$i
        }
      }
      next
    }
    $1 ~ /^[0-9]+:/ {
      irq=$1; gsub(":", "", irq)
      if (!(irq in wanted)) next
      total=0
      detail=""
      for (i=2; i<=NF && $i ~ /^[0-9]+$/; i++) {
        d=$i - before[irq, i-2]
        total += d
        detail=detail sprintf(" CPU%d=%d", i-2, d)
      }
      printf "| %s | %d |%s | %s |\n", irq, total, detail, $0
    }
  ' "${start_file}" "${end_file}"
}

softirq_net_rx_delta() {
  awk '
    NR == FNR {if ($1 == "NET_RX:") for (i=2; i<=NF; i++) before[i-2]=$i; next}
    $1 == "NET_RX:" {
      printf "| CPU | NET_RX delta |\n|---|---:|\n"
      for (i=2; i<=NF; i++) printf "| CPU%d | %d |\n", i-2, $i - before[i-2]
    }
  ' "$1" "$2"
}

cfg="$(driver_config_path)"
if [[ -z "${INTERFACE}" ]]; then
  INTERFACE="$(infer_interface_from_config "${cfg}" 2>/dev/null || true)"
fi

if [[ -z "${INTERFACE}" ]]; then
  echo "[lidar-irq-id] unable to infer LiDAR interface from current driver config." >&2
  echo "[lidar-irq-id] pass --interface ethX to continue." >&2
  echo "[lidar-irq-id] driver_config=${cfg:-missing}" >&2
  exit 2
fi

if [[ ! -d "/sys/class/net/${INTERFACE}" ]]; then
  echo "[lidar-irq-id] interface does not exist: ${INTERFACE}" >&2
  exit 2
fi

driver="$(iface_driver "${INTERFACE}")"
ssh_iface="$(ssh_interface || true)"
ssh_risk="false"
[[ -n "${ssh_iface}" && "${ssh_iface}" == "${INTERFACE}" ]] && ssh_risk="true"
rx_queues="$(find "/sys/class/net/${INTERFACE}/queues" -maxdepth 1 -type d -name 'rx-*' 2>/dev/null | wc -l | tr -d ' ')"
tx_queues="$(find "/sys/class/net/${INTERFACE}/queues" -maxdepth 1 -type d -name 'tx-*' 2>/dev/null | wc -l | tr -d ' ')"
mapfile -t irq_list < <(matching_irqs "${INTERFACE}" "${driver:-}")

tmp_dir="$(mktemp -d /tmp/njrh_lidar_irq_id_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT
interrupts_start="${tmp_dir}/interrupts_start.txt"
interrupts_end="${tmp_dir}/interrupts_end.txt"
softirqs_start="${tmp_dir}/softirqs_start.txt"
softirqs_end="${tmp_dir}/softirqs_end.txt"

cat /proc/interrupts >"${interrupts_start}" 2>/dev/null || true
cat /proc/softirqs >"${softirqs_start}" 2>/dev/null || true
sleep "${DURATION_SEC}"
cat /proc/interrupts >"${interrupts_end}" 2>/dev/null || true
cat /proc/softirqs >"${softirqs_end}" 2>/dev/null || true

echo "[lidar-irq-id] interface=${INTERFACE}"
echo "[lidar-irq-id] interface_ip=$(iface_ipv4 "${INTERFACE}")"
echo "[lidar-irq-id] driver=${driver:-unknown}"
echo "[lidar-irq-id] rx_queues=${rx_queues} tx_queues=${tx_queues}"
echo "[lidar-irq-id] driver_config=${cfg:-missing}"
echo "[lidar-irq-id] candidate_lidar_ips=$(candidate_lidar_ips "${cfg:-}" | paste -sd ',' -)"
echo "[lidar-irq-id] ssh_default_interface=${ssh_iface:-unknown}"
echo "[lidar-irq-id] ssh_interface_risk=${ssh_risk}"
if [[ "${ssh_risk}" == "true" ]]; then
  echo "[lidar-irq-id] WARN interface ${INTERFACE} appears to carry the SSH/default route; IRQ/RPS apply must use --allow-ssh-interface-risk."
fi

echo "[lidar-irq-id] irq_list=${irq_list[*]:-none}"
for irq in "${irq_list[@]}"; do
  affinity="missing"
  [[ -r "/proc/irq/${irq}/smp_affinity_list" ]] && affinity="$(cat "/proc/irq/${irq}/smp_affinity_list" 2>/dev/null || true)"
  echo "[lidar-irq-id] irq=${irq} smp_affinity_list=${affinity}"
done

echo "[lidar-irq-id] matching_interrupt_lines:"
matching_interrupt_lines "${INTERFACE}" "${driver:-}" | sed 's/^/[lidar-irq-id]   /'

echo "[lidar-irq-id] interrupt_delta:"
interrupt_delta_for_irqs "${interrupts_start}" "${interrupts_end}" "${irq_list[@]}" | sed 's/^/[lidar-irq-id]   /'

echo "[lidar-irq-id] net_rx_softirq_delta:"
softirq_net_rx_delta "${softirqs_start}" "${softirqs_end}" | sed 's/^/[lidar-irq-id]   /'

echo "[lidar-irq-id] rps_xps:"
for file in /sys/class/net/"${INTERFACE}"/queues/rx-*/rps_cpus /sys/class/net/"${INTERFACE}"/queues/tx-*/xps_cpus; do
  [[ -e "${file}" ]] || continue
  echo "[lidar-irq-id]   ${file}=$(cat "${file}" 2>/dev/null || true)"
done

echo "[lidar-irq-id] ip_br_addr:"
safe_run ip -br addr | sed 's/^/[lidar-irq-id]   /'
echo "[lidar-irq-id] ip_route:"
safe_run ip route | sed 's/^/[lidar-irq-id]   /'
if command -v ethtool >/dev/null 2>&1; then
  echo "[lidar-irq-id] ethtool_i:"
  safe_run ethtool -i "${INTERFACE}" | sed 's/^/[lidar-irq-id]   /'
  echo "[lidar-irq-id] ethtool_S_sample:"
  safe_run ethtool -S "${INTERFACE}" | sed -n '1,120p' | sed 's/^/[lidar-irq-id]   /'
else
  echo "[lidar-irq-id] ethtool=unavailable"
fi

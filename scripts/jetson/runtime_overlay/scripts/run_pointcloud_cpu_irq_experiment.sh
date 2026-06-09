#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-/workspaces/njrh-v3/workspace1}"
REPORT_DIR="${PROJECT_ROOT}/reports"

CPU_PROFILE="split_local_nav_v1"
IRQ_PROFILE="irq_keep_default"
INTERFACE=""
DURATION_SEC=120
APPLY=false
RESTORE=false
PRINT=false
KEEP_APPLIED=false
ALLOW_SSH_INTERFACE_RISK=false
REPORT_FILE=""

usage() {
  cat <<'EOF'
Usage: run_pointcloud_cpu_irq_experiment.sh [--cpu-profile PROFILE] [--irq-profile PROFILE] [--interface IFACE] [--duration-sec N] [--print] [--apply] [--restore] [--keep-applied] [--allow-ssh-interface-risk] [--report PATH]

Default mode prints the experiment plan only. --apply runs a reversible A/B:
baseline snapshot, CPU/IRQ profile apply, profile snapshot and pointcloud
diagnostics, then automatic restore unless --keep-applied is set. --restore
only restores CPU and IRQ profile state.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cpu-profile)
      CPU_PROFILE="${2:-}"
      shift 2
      ;;
    --irq-profile)
      IRQ_PROFILE="${2:-}"
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
    --keep-applied)
      KEEP_APPLIED=true
      shift
      ;;
    --allow-ssh-interface-risk)
      ALLOW_SSH_INTERFACE_RISK=true
      shift
      ;;
    --report)
      REPORT_FILE="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[cpu-irq-experiment] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*) echo "[cpu-irq-experiment] --duration-sec must be an integer" >&2; exit 2 ;;
esac
[[ "${DURATION_SEC}" -gt 0 ]] || {
  echo "[cpu-irq-experiment] --duration-sec must be positive" >&2
  exit 2
}

mkdir -p "${REPORT_DIR}"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${REPORT_FILE}" ]]; then
  REPORT_FILE="${REPORT_DIR}/pointcloud_cpu_irq_experiment_${timestamp}.md"
fi

irq_common_args=("--profile" "${IRQ_PROFILE}" "--duration-sec" "5" "--no-diagnostics")
[[ -n "${INTERFACE}" ]] && irq_common_args+=("--interface" "${INTERFACE}")
[[ "${ALLOW_SSH_INTERFACE_RISK}" == "true" ]] && irq_common_args+=("--allow-ssh-interface-risk")

log_section() {
  local title="$1"
  {
    echo
    echo "## ${title}"
    echo
  } >>"${REPORT_FILE}"
}

run_logged() {
  local label="$1"
  shift
  log_section "${label}"
  (
    echo '```text'
    echo "$ $*"
    set +e
    "$@" 2>&1
    local status=$?
    set -e
    echo "[exit_status] ${status}"
    echo '```'
    exit "${status}"
  ) | tee -a "${REPORT_FILE}"
  local status="${PIPESTATUS[0]}"
  return "${status}"
}

run_logged_allow_fail() {
  local label="$1"
  shift
  run_logged "${label}" "$@" || true
}

print_plan() {
  cat <<EOF
[cpu-irq-experiment] cpu_profile=${CPU_PROFILE}
[cpu-irq-experiment] irq_profile=${IRQ_PROFILE}
[cpu-irq-experiment] interface=${INTERFACE:-auto}
[cpu-irq-experiment] duration_sec=${DURATION_SEC}
[cpu-irq-experiment] report=${REPORT_FILE}
[cpu-irq-experiment] default_mode=dry-run
[cpu-irq-experiment] will_not_change=QoS,DDS,timestamps,Nav2_planner_controller,EKF,FAST-LIO2,App_API,mapping_cleanup
[cpu-irq-experiment] apply_sequence:
[cpu-irq-experiment]   1. baseline CPU/IRQ/softirq snapshot
[cpu-irq-experiment]   2. apply CPU profile live through taskset helper without killing processes
[cpu-irq-experiment]   3. apply LiDAR IRQ/RPS/XPS profile only when --apply is explicit
[cpu-irq-experiment]   4. collect profile snapshot and pointcloud/local/nav diagnostics
[cpu-irq-experiment]   5. restore CPU/IRQ state unless --keep-applied is explicit
EOF
  echo "[cpu-irq-experiment] CPU profile plan:"
  bash "${SCRIPT_DIR}/run_cpu_core_allocation_ab.sh" --profile "${CPU_PROFILE}" --print --no-diagnostics | sed 's/^/[cpu-irq-experiment]   /' || true
  echo "[cpu-irq-experiment] IRQ profile plan:"
  bash "${SCRIPT_DIR}/run_lidar_irq_affinity_ab.sh" "${irq_common_args[@]}" --print | sed 's/^/[cpu-irq-experiment]   /' || true
}

write_header() {
  {
    echo "# Pointcloud CPU / IRQ Experiment"
    echo
    echo "- timestamp_utc: ${timestamp}"
    echo "- cpu_profile: ${CPU_PROFILE}"
    echo "- irq_profile: ${IRQ_PROFILE}"
    echo "- interface: ${INTERFACE:-auto}"
    echo "- duration_sec: ${DURATION_SEC}"
    echo "- keep_applied: ${KEEP_APPLIED}"
    echo "- mode: $([[ "${APPLY}" == "true" ]] && echo apply || echo dry-run)"
    echo
    echo "This experiment does not change ROS QoS, DDS middleware/default transport, timestamps, Nav2 controller/planner settings, EKF, FAST-LIO2 logic, App API, or mapping cleanup ownership. It does not subscribe to full-density PointCloud2 topics."
  } >"${REPORT_FILE}"
}

restore_all() {
  run_logged_allow_fail "Restore IRQ/RPS/XPS profile" \
    bash "${SCRIPT_DIR}/run_lidar_irq_affinity_ab.sh" "${irq_common_args[@]}" --restore --no-diagnostics
  run_logged_allow_fail "Restore CPU profile" \
    bash "${SCRIPT_DIR}/run_cpu_core_allocation_ab.sh" --restore --restart --no-diagnostics
}

print_plan
write_header

if [[ "${RESTORE}" == "true" ]]; then
  restore_all
  echo "[cpu-irq-experiment] restore report=${REPORT_FILE}"
  exit 0
fi

if [[ "${APPLY}" != "true" ]]; then
  {
    echo
    echo "## Dry-Run Plan"
    echo
    echo '```text'
    print_plan
    echo '```'
  } >>"${REPORT_FILE}"
  echo "[cpu-irq-experiment] no --apply requested; wrote dry-run report=${REPORT_FILE}"
  exit 0
fi

cleanup_restore() {
  if [[ "${KEEP_APPLIED}" == "true" ]]; then
    echo "[cpu-irq-experiment] keep-applied requested; leaving profile active" | tee -a "${REPORT_FILE}"
    return 0
  fi
  echo "[cpu-irq-experiment] restoring CPU/IRQ state" | tee -a "${REPORT_FILE}"
  restore_all
}
trap cleanup_restore EXIT

run_logged_allow_fail "Baseline CPU/IRQ/softirq snapshot" \
  bash "${SCRIPT_DIR}/collect_cpu_irq_softirq_snapshot.sh" --duration-sec "${DURATION_SEC}"

run_logged_allow_fail "Baseline local perception diagnostics" \
  bash "${SCRIPT_DIR}/diagnose_local_perception_pipeline.sh"
run_logged_allow_fail "Baseline nav scan diagnostics" \
  bash "${SCRIPT_DIR}/diagnose_nav_scan_pipeline.sh"
run_logged_allow_fail "Baseline pointcloud delivery matrix" \
  bash "${SCRIPT_DIR}/verify_pointcloud_delivery_matrix.sh"

run_logged "Apply CPU profile" \
  bash "${SCRIPT_DIR}/run_cpu_core_allocation_ab.sh" --profile "${CPU_PROFILE}" --apply --restart --no-diagnostics
run_logged "Apply LiDAR IRQ profile" \
  bash "${SCRIPT_DIR}/run_lidar_irq_affinity_ab.sh" "${irq_common_args[@]}" --apply --no-diagnostics

run_logged_allow_fail "Profile CPU/IRQ/softirq snapshot" \
  bash "${SCRIPT_DIR}/collect_cpu_irq_softirq_snapshot.sh" --duration-sec "${DURATION_SEC}"

run_logged_allow_fail "Profile local perception diagnostics" \
  bash "${SCRIPT_DIR}/diagnose_local_perception_pipeline.sh"
run_logged_allow_fail "Profile nav scan diagnostics" \
  bash "${SCRIPT_DIR}/diagnose_nav_scan_pipeline.sh"
run_logged_allow_fail "Profile pointcloud delivery matrix" \
  bash "${SCRIPT_DIR}/verify_pointcloud_delivery_matrix.sh"
run_logged_allow_fail "Commercial runtime constraints" \
  bash "${SCRIPT_DIR}/check_commercial_runtime_ready.sh"
run_logged_allow_fail "FAST-LIO residency check" \
  bash -lc 'pgrep -af "fast_lio|fastlio|laser_mapping" || true'

trap - EXIT
cleanup_restore

echo "[cpu-irq-experiment] report=${REPORT_FILE}"

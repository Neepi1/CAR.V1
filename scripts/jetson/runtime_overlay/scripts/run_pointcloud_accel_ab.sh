#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PROFILE=""
DURATION_SEC=120
DO_APPLY=false
DO_RESTART=false
DO_RESTORE=false

usage() {
  cat <<'EOF'
Usage: run_pointcloud_accel_ab.sh --profile legacy|ipc_worker|nitros [--duration-sec SEC] [--apply] [--restart] [--restore]

Without --apply this script only records the current runtime. With --restore it
switches back to legacy and exits after verification.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --profile)
      [[ "$#" -ge 2 ]] || { echo "[pointcloud-accel-ab] --profile requires a value" >&2; exit 2; }
      PROFILE="$2"
      shift 2
      ;;
    --duration-sec)
      [[ "$#" -ge 2 ]] || { echo "[pointcloud-accel-ab] --duration-sec requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --apply)
      DO_APPLY=true
      shift
      ;;
    --restart)
      DO_RESTART=true
      shift
      ;;
    --restore)
      DO_RESTORE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[pointcloud-accel-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${DO_RESTORE}" == "true" ]]; then
  PROFILE="legacy"
  DO_APPLY=true
  DO_RESTART=true
fi

case "${PROFILE}" in
  legacy|ipc_worker|nitros) ;;
  *)
    echo "[pointcloud-accel-ab] valid --profile is required" >&2
    usage >&2
    exit 2
    ;;
esac

[[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || { echo "[pointcloud-accel-ab] invalid duration: ${DURATION_SEC}" >&2; exit 2; }

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${NJRH_PROJECT_ROOT}/reports"
mkdir -p "${report_dir}"
report="${report_dir}/pointcloud_accel_ab_${timestamp}.md"

if [[ "${DO_APPLY}" == "true" ]]; then
  args=(--profile "${PROFILE}")
  [[ "${DO_RESTART}" == "true" ]] && args+=(--restart)
  bash "${SCRIPT_DIR}/set_pointcloud_accel_profile.sh" "${args[@]}"
  if [[ "${DO_RESTART}" == "true" ]]; then
    sleep 12
  fi
else
  echo "[pointcloud-accel-ab] --apply not requested; observing current runtime without profile switch"
fi

tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_accel_ab_XXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

sample_out="${tmp_dir}/samples.txt"
echo "[pointcloud-accel-ab] sampling lightweight status topics for ${DURATION_SEC}s"
sample_start="${SECONDS}"
while (( SECONDS - sample_start < DURATION_SEC )); do
  {
    echo "sample_t=$((SECONDS - sample_start))"
    timeout 3 ros2 topic echo /lidar/axis_remap_status --field data --once 2>/dev/null || true
    timeout 3 ros2 topic echo /lidar/pointcloud_accel_status --field data --once 2>/dev/null || true
    top -b -n1 | awk '/^%Cpu|^Cpu/ {print}' || true
    echo
  } >>"${sample_out}"
  sleep "${NJRH_POINTCLOUD_ACCEL_AB_SAMPLE_PERIOD_SEC:-5}"
done

status_out="${tmp_dir}/verify.txt"
if ! timeout "$((DURATION_SEC + 40))" bash "${SCRIPT_DIR}/verify_pointcloud_accel_profile.sh" >"${status_out}" 2>&1; then
  verify_result="FAIL"
else
  verify_result="PASS"
fi

axis_status="$(timeout 8 ros2 topic echo /lidar/axis_remap_status --field data --once 2>/dev/null || true)"
accel_status="$(timeout 8 ros2 topic echo /lidar/pointcloud_accel_status --field data --once 2>/dev/null || true)"
lidar_info="$(timeout 8 ros2 topic info -v /lidar_points 2>&1 || true)"
thermal="$( { tegrastats --interval 1000 --count 1 2>/dev/null || true; } )"
cpu_snapshot="$(top -b -n1 | awk '/^%Cpu|^Cpu/ {print}' || true)"
fastlio_residual="false"
pgrep -f "fast_lio|fastlio|laser_mapping" >/dev/null 2>&1 && fastlio_residual="true"

{
  echo "# PointCloud Accel A/B ${timestamp}"
  echo
  echo "- profile: ${PROFILE}"
  echo "- apply: ${DO_APPLY}"
  echo "- restart: ${DO_RESTART}"
  echo "- duration_sec: ${DURATION_SEC}"
  echo "- result: ${verify_result}"
  echo "- topology:"
  case "${PROFILE}" in
    legacy)
      echo "  - /lidar_points full trunk"
      echo "  - /_internal/lidar_points_local -> robot_local_perception -> /perception/*"
      echo "  - /lidar_points_nav -> /points_nav -> /scan -> /flatscan"
      ;;
    ipc_worker|nitros)
      echo "  - /lidar_points full trunk"
      echo "  - pointcloud_accel_axis_node workers -> /perception/* and /scan"
      echo "  - /_internal/lidar_points_local and /lidar_points_nav compact debug/compat only"
      echo "  - /points_nav is not production"
      ;;
  esac
  echo
  echo "## Verify Output"
  echo '```text'
  cat "${status_out}"
  echo '```'
  echo
  echo "## Duration Samples"
  echo '```text'
  cat "${sample_out}"
  echo '```'
  echo
  echo "## Axis Status"
  echo '```text'
  echo "${axis_status}"
  echo '```'
  echo
  echo "## Accel Status"
  echo '```text'
  echo "${accel_status}"
  echo '```'
  echo
  echo "## /lidar_points Graph"
  echo '```text'
  echo "${lidar_info}"
  echo '```'
  echo
  echo "## CPU/Thermal"
  echo '```text'
  echo "${cpu_snapshot}"
  echo "${thermal}"
  echo '```'
  echo
  echo "- FAST-LIO2 residual: ${fastlio_residual}"
  if [[ "${verify_result}" == "PASS" && "${fastlio_residual}" == "false" ]]; then
    echo "- recommendation: profile is acceptable for the next loaded field test."
  else
    echo "- recommendation: keep legacy or repeat A/B after fixing reported FAIL/WARN items."
  fi
} >"${report}"

echo "[pointcloud-accel-ab] report=${report}"
cat "${status_out}"

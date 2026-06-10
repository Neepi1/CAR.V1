#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"

PROFILE=""
DURATION_SEC=120
DO_APPLY=false
DO_RESTART=false
DO_RESTORE=false

usage() {
  cat <<'EOF'
Usage: run_pointcloud_accel_ab.sh --profile legacy|ipc_worker|nitros [--duration-sec SEC] [--apply] [--restart] [--restore]

Without --apply this script only records the current runtime. With --restore it
switches back to the profile that was active before this A/B run.
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

case "${PROFILE}" in
  legacy|ipc_worker|nitros) ;;
  *)
    echo "[pointcloud-accel-ab] valid --profile is required" >&2
    usage >&2
    exit 2
    ;;
esac

[[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || { echo "[pointcloud-accel-ab] invalid duration: ${DURATION_SEC}" >&2; exit 2; }

unset NJRH_POINTCLOUD_ACCEL_PROFILE
njrh_load_pointcloud_accel_profile
prior_profile="${NJRH_POINTCLOUD_ACCEL_PROFILE}"
requested_profile="${PROFILE}"

topic_info_file() {
  local topic="$1"
  local label="$2"
  local output_file="$3"
  timeout 8 ros2 topic info -v "${topic}" >"${output_file}/${label}.info" 2>&1 || true
}

topic_count_from_file() {
  local file="$1"
  local key="$2"
  awk -F: -v k="${key}" '$1 ~ k {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}' "${file}"
}

publisher_nodes_from_file() {
  local file="$1"
  awk '
    /^[[:space:]]*Node name:/ {
      node=$0
      sub(/^[[:space:]]*Node name:[[:space:]]*/, "", node)
    }
    /^[[:space:]]*Endpoint type:/ {
      endpoint=$0
      if (endpoint ~ /PUBLISHER/ && node != "") {
        print node
      }
    }
  ' "${file}" | sort -u | awk 'NF {if (out != "") out = out ", " $0; else out = $0} END {print out}'
}

process_snapshot() {
  pgrep -af "pointcloud_axis_remap|pointcloud_accel_axis|robot_local_perception|nav_cloud_preprocessor|pointcloud_to_laserscan|scan_republisher|laser_scan_to_flatscan" 2>/dev/null || true
}

status_once() {
  local topic="$1"
  timeout 8 ros2 topic echo "${topic}" --field data --once 2>/dev/null || true
}

light_hz() {
  local topic="$1"
  local output
  output="$(timeout 14 ros2 topic hz "${topic}" 2>/dev/null || true)"
  awk '/average rate:/ {value=$3} END {if (value != "") print value}' <<<"${output}"
}

cpu_subset_usage() {
  python3 - <<'PY'
import time

cores = {"cpu0", "cpu4", "cpu5", "cpu6", "cpu7"}

def read_stats():
    values = {}
    with open("/proc/stat", encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if parts and parts[0] in cores:
                nums = [int(x) for x in parts[1:]]
                idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
                total = sum(nums)
                values[parts[0]] = (idle, total)
    return values

first = read_stats()
time.sleep(1.0)
second = read_stats()
for core in sorted(cores):
    if core not in first or core not in second:
        continue
    idle_delta = second[core][0] - first[core][0]
    total_delta = second[core][1] - first[core][1]
    used = 0.0 if total_delta <= 0 else 100.0 * (1.0 - idle_delta / total_delta)
    print(f"{core}={used:.1f}%")
PY
}

fastlio_residual() {
  if pgrep -f "fast[_]lio|fast[l]io|laser[_]mapping" >/dev/null 2>&1; then
    echo "true"
  else
    echo "false"
  fi
}

flatscan_helper_status_file() {
  printf '%s\n' "${NJRH_FLATSCAN_HELPER_STATUS_FILE:-${NJRH_RUNTIME_LOG_DIR}/flatscan_helper_status.env}"
}

nav2_state() {
  {
    timeout 5 ros2 lifecycle get /controller_server 2>/dev/null || true
    timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null || true
  } | paste -sd "; " -
}

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${NJRH_PROJECT_ROOT}/reports"
mkdir -p "${report_dir}"
report="${report_dir}/pointcloud_accel_ab_${timestamp}.md"

if [[ "${DO_APPLY}" == "true" ]]; then
  args=(--profile "${PROFILE}")
  [[ "${DO_RESTART}" == "true" ]] && args+=(--restart)
  bash "${SCRIPT_DIR}/set_pointcloud_accel_profile.sh" "${args[@]}"
  if [[ "${DO_RESTART}" == "true" ]]; then
    sleep "${NJRH_POINTCLOUD_ACCEL_AB_RESTART_SETTLE_SEC:-12}"
  fi
else
  echo "[pointcloud-accel-ab] --apply not requested; observing current runtime without profile switch"
fi

unset NJRH_POINTCLOUD_ACCEL_PROFILE
njrh_load_pointcloud_accel_profile
actual_profile="${NJRH_POINTCLOUD_ACCEL_PROFILE}"

tmp_dir="$(mktemp -d /tmp/njrh_pointcloud_accel_ab_XXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

sample_out="${tmp_dir}/samples.txt"
echo "[pointcloud-accel-ab] sampling lightweight status topics for ${DURATION_SEC}s"
sample_start="${SECONDS}"
while (( SECONDS - sample_start < DURATION_SEC )); do
  {
    echo "sample_t=$((SECONDS - sample_start))"
    status_once /lidar/axis_remap_status
    status_once /lidar/pointcloud_accel_status
    status_once /perception/local_perception_status
    cpu_subset_usage
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

topic_info_file /lidar_points lidar "${tmp_dir}"
topic_info_file /perception/obstacle_points obstacle "${tmp_dir}"
topic_info_file /perception/clearing_points clearing "${tmp_dir}"
topic_info_file /scan scan "${tmp_dir}"
topic_info_file /flatscan flatscan "${tmp_dir}"
topic_info_file /points_nav points_nav "${tmp_dir}"
topic_info_file /lidar_points_nav lidar_points_nav "${tmp_dir}"

axis_status="$(status_once /lidar/axis_remap_status)"
accel_status="$(status_once /lidar/pointcloud_accel_status)"
local_status=""
if [[ "${actual_profile}" == "legacy" ]]; then
  local_status="$(status_once /perception/local_perception_status)"
fi
status_field() {
  local text="$1"
  local key="$2"
  awk -v k="${key}" '{
    for (i = 1; i <= NF; ++i) {
      split($i, kv, "=")
      if (kv[1] == k) {
        print substr($i, length(k) + 2)
        exit
      }
    }
  }' <<<"${text}"
}
internal_zero_copy_profile="$(status_field "${accel_status}" internal_zero_copy_profile)"
latest_internal_buffer_points="$(status_field "${accel_status}" latest_internal_buffer_points)"
local_worker_full_cloud_copy_count="$(status_field "${accel_status}" local_worker_full_cloud_copy_count)"
scan_worker_full_cloud_copy_count="$(status_field "${accel_status}" scan_worker_full_cloud_copy_count)"
local_worker_intermediate_pointcloud_build_count="$(status_field "${accel_status}" local_worker_intermediate_pointcloud_build_count)"
scan_worker_intermediate_pointcloud_build_count="$(status_field "${accel_status}" scan_worker_intermediate_pointcloud_build_count)"
lidar_publishers="$(topic_count_from_file "${tmp_dir}/lidar.info" "Publisher count")"
lidar_subscribers="$(topic_count_from_file "${tmp_dir}/lidar.info" "Subscription count")"
points_nav_publishers="$(topic_count_from_file "${tmp_dir}/points_nav.info" "Publisher count")"
points_nav_subscribers="$(topic_count_from_file "${tmp_dir}/points_nav.info" "Subscription count")"
lidar_points_nav_publishers="$(topic_count_from_file "${tmp_dir}/lidar_points_nav.info" "Publisher count")"
scan_publishers="$(topic_count_from_file "${tmp_dir}/scan.info" "Publisher count")"
scan_subscribers="$(topic_count_from_file "${tmp_dir}/scan.info" "Subscription count")"
flatscan_publishers="$(topic_count_from_file "${tmp_dir}/flatscan.info" "Publisher count")"
flatscan_subscribers="$(topic_count_from_file "${tmp_dir}/flatscan.info" "Subscription count")"
trunk_owner="$(publisher_nodes_from_file "${tmp_dir}/lidar.info")"
obstacle_owner="$(publisher_nodes_from_file "${tmp_dir}/obstacle.info")"
clearing_owner="$(publisher_nodes_from_file "${tmp_dir}/clearing.info")"
points_nav_owner="$(publisher_nodes_from_file "${tmp_dir}/points_nav.info")"
scan_owner="$(publisher_nodes_from_file "${tmp_dir}/scan.info")"
flatscan_owner="$(publisher_nodes_from_file "${tmp_dir}/flatscan.info")"
obstacle_hz="$(light_hz /perception/obstacle_points)"
scan_hz="$(light_hz /scan)"
flatscan_hz="$(light_hz /flatscan)"
flatscan_helper_pid="missing"
flatscan_helper_restart_count="missing"
flatscan_helper_mode="missing"
helper_status_file="$(flatscan_helper_status_file)"
if [[ -f "${helper_status_file}" ]]; then
  # shellcheck disable=SC1090
  source "${helper_status_file}"
  flatscan_helper_pid="${FLATSCAN_HELPER_PID:-missing}"
  flatscan_helper_restart_count="${FLATSCAN_HELPER_RESTART_COUNT:-missing}"
  flatscan_helper_mode="${FLATSCAN_HELPER_MODE:-missing}"
fi
cpu_usage="$(cpu_subset_usage)"
thermal="$( { tegrastats --interval 1000 --count 1 2>/dev/null || true; } )"
nav2_lifecycle="$(nav2_state)"
fastlio="$(fastlio_residual)"
binary_snapshot="$(process_snapshot)"
binary_running="$(awk '
  /pointcloud_accel_axis/ {accel=1}
  /pointcloud_axis_remap/ {legacy=1}
  END {
    if (legacy && accel) print "legacy_and_ipc";
    else if (accel) print "pointcloud_accel_axis_node";
    else if (legacy) print "pointcloud_axis_remap_node";
    else print "missing";
  }
' <<<"${binary_snapshot}")"

overall_result="PASS"
if [[ "${verify_result}" != "PASS" || "${fastlio}" != "false" ]]; then
  overall_result="FAIL"
elif [[ -z "${obstacle_hz}" || -z "${scan_hz}" || -z "${flatscan_hz}" ]]; then
  overall_result="WARN"
fi
flatscan_case="OK"
if [[ "${scan_publishers:-0}" != "0" && "${flatscan_publishers:-0}" == "0" ]]; then
  flatscan_case="CASE_FLATSCAN_HELPER_DEAD"
  overall_result="FAIL"
elif [[ -z "${flatscan_hz}" ]]; then
  flatscan_case="CASE_FLATSCAN_HZ_MISSING"
fi

legacy_scan_chain_recovered="n/a"
ipc_no_points_nav_production="n/a"
if [[ "${actual_profile}" == "legacy" ]]; then
  legacy_scan_chain_recovered="false"
  if grep -Eq '(^|, )nav_cloud_preprocessor($|, )' <<<"${points_nav_owner}" \
    && grep -Eq '(^|, )scan_republisher($|, )' <<<"${scan_owner}" \
    && grep -Eq '(^|, )laser_scan_to_flatscan($|, )' <<<"${flatscan_owner}"; then
    legacy_scan_chain_recovered="true"
  fi
elif [[ "${actual_profile}" == "ipc_worker" || "${actual_profile}" == "nitros" ]]; then
  ipc_no_points_nav_production="false"
  if [[ "${points_nav_publishers:-0}" == "0" ]]; then
    ipc_no_points_nav_production="true"
  fi
fi

{
  echo "# PointCloud Accel A/B ${timestamp}"
  echo
  echo "- profile requested: ${requested_profile}"
  echo "- profile before run: ${prior_profile}"
  echo "- profile actually running: ${actual_profile}"
  echo "- binary actually running: ${binary_running}"
  echo "- apply: ${DO_APPLY}"
  echo "- restart: ${DO_RESTART}"
  echo "- restore_requested: ${DO_RESTORE}"
  echo "- duration_sec: ${DURATION_SEC}"
  echo "- verify_result: ${verify_result}"
  echo "- PASS/WARN/FAIL: ${overall_result}"
  echo "- /lidar_points publisher_count: ${lidar_publishers:-missing}"
  echo "- /lidar_points subscriber_count: ${lidar_subscribers:-missing}"
  echo "- trunk owner: ${trunk_owner:-missing}"
  echo "- obstacle owner: ${obstacle_owner:-missing}"
  echo "- clearing owner: ${clearing_owner:-missing}"
  echo "- points_nav owner: ${points_nav_owner:-missing}"
  echo "- scan owner: ${scan_owner:-missing}"
  echo "- flatscan owner: ${flatscan_owner:-missing}"
  echo "- flatscan helper mode: ${flatscan_helper_mode}"
  echo "- flatscan helper pid: ${flatscan_helper_pid}"
  echo "- flatscan helper restart count: ${flatscan_helper_restart_count}"
  echo "- flatscan case: ${flatscan_case}"
  echo "- /points_nav publishers: ${points_nav_publishers:-0}"
  echo "- /points_nav subscribers: ${points_nav_subscribers:-0}"
  echo "- /lidar_points_nav publishers: ${lidar_points_nav_publishers:-0}"
  echo "- /scan publishers: ${scan_publishers:-0}"
  echo "- /scan subscribers: ${scan_subscribers:-0}"
  echo "- /flatscan publishers: ${flatscan_publishers:-0}"
  echo "- /flatscan subscribers: ${flatscan_subscribers:-0}"
  echo "- obstacle_hz: ${obstacle_hz:-missing}"
  echo "- scan_hz: ${scan_hz:-missing}"
  echo "- flatscan_hz: ${flatscan_hz:-missing}"
  echo "- helper_status_file: ${helper_status_file}"
  echo "- internal_zero_copy_profile: ${internal_zero_copy_profile:-missing}"
  echo "- latest_internal_buffer_points: ${latest_internal_buffer_points:-missing}"
  echo "- local_worker_full_cloud_copy_count: ${local_worker_full_cloud_copy_count:-missing}"
  echo "- scan_worker_full_cloud_copy_count: ${scan_worker_full_cloud_copy_count:-missing}"
  echo "- local_worker_intermediate_pointcloud_build_count: ${local_worker_intermediate_pointcloud_build_count:-missing}"
  echo "- scan_worker_intermediate_pointcloud_build_count: ${scan_worker_intermediate_pointcloud_build_count:-missing}"
  echo "- legacy scan chain recovered: ${legacy_scan_chain_recovered}"
  echo "- ipc_worker no production /points_nav hop: ${ipc_no_points_nav_production}"
  echo "- Nav2 lifecycle: ${nav2_lifecycle:-unavailable}"
  echo "- FAST-LIO2 residual: ${fastlio}"
  echo
  echo "## Topology"
  case "${actual_profile}" in
    legacy)
      echo "- /lidar_points full trunk"
      echo "- /_internal/lidar_points_local -> robot_local_perception -> /perception/*"
      echo "- /lidar_points_nav -> /points_nav -> /scan -> /flatscan"
      ;;
    ipc_worker|nitros)
      echo "- /lidar_points full trunk"
      echo "- pointcloud_accel_axis_node workers -> /perception/* and /scan"
      echo "- /_internal/lidar_points_local and /lidar_points_nav compact debug/compat only"
      echo "- /points_nav is not production"
      ;;
  esac
  echo
  echo "## Verify Output"
  echo '```text'
  cat "${status_out}"
  echo '```'
  echo
  echo "## Status Samples"
  echo '```text'
  cat "${sample_out}"
  echo '```'
  echo
  echo "## Latest Status"
  echo '```text'
  echo "axis_status=${axis_status}"
  echo "accel_status=${accel_status}"
  echo "local_status=${local_status}"
  echo '```'
  echo
  echo "## Binary Snapshot"
  echo '```text'
  echo "${binary_snapshot}"
  echo '```'
  echo
  echo "## Topic Graphs"
  echo '```text'
  cat "${tmp_dir}/lidar.info"
  cat "${tmp_dir}/obstacle.info"
  cat "${tmp_dir}/clearing.info"
  cat "${tmp_dir}/scan.info"
  cat "${tmp_dir}/flatscan.info"
  cat "${tmp_dir}/points_nav.info"
  cat "${tmp_dir}/lidar_points_nav.info"
  echo '```'
  echo
  echo "## CPU/Thermal"
  echo '```text'
  echo "${cpu_usage}"
  echo "${thermal}"
  echo '```'
  echo
  if [[ "${verify_result}" == "PASS" && "${fastlio}" == "false" ]]; then
    echo "- recommendation: profile is acceptable for the next loaded field test."
  else
    echo "- recommendation: keep legacy or repeat A/B after fixing reported FAIL/WARN items."
  fi
} >"${report}"

echo "[pointcloud-accel-ab] report=${report}"
cat "${status_out}"

if [[ "${DO_RESTORE}" == "true" ]]; then
  echo "[pointcloud-accel-ab] restoring prior profile=${prior_profile}"
  bash "${SCRIPT_DIR}/set_pointcloud_accel_profile.sh" --profile "${prior_profile}" --restart
  sleep "${NJRH_POINTCLOUD_ACCEL_AB_RESTART_SETTLE_SEC:-12}"
  {
    echo
    echo "## Restore"
    echo
    echo "- restored_profile: ${prior_profile}"
    echo "- restore_command: set_pointcloud_accel_profile.sh --profile ${prior_profile} --restart"
  } >>"${report}"
fi

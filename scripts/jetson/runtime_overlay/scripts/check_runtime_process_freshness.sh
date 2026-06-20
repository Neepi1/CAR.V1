#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

tmp_dir="$(mktemp -d /tmp/njrh_process_freshness_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

STATUS_SECONDS="${NJRH_PROCESS_FRESHNESS_STATUS_SECONDS:-6}"
STRICT="${NJRH_PROCESS_FRESHNESS_STRICT:-true}"
CHECK_ALL="${NJRH_PROCESS_FRESHNESS_CHECK_ALL:-false}"

float_epoch() {
  local value="$1"
  [[ "${value}" =~ ^[0-9]+$ ]] || value=0
  printf '%s\n' "${value}"
}

process_start_epoch() {
  local pid="$1"
  local lstart
  lstart="$(ps -p "${pid}" -o lstart= 2>/dev/null | xargs || true)"
  if [[ -n "${lstart}" ]]; then
    date -d "${lstart}" +%s 2>/dev/null || printf '0\n'
  else
    printf '0\n'
  fi
}

pid_list_for_pattern() {
  local pattern="$1"
  pgrep -f "${pattern}" 2>/dev/null | sort -n | paste -sd ' ' -
}

status_once() {
  local topic="$1"
  local raw_file="${tmp_dir}/status.raw"
  timeout "${STATUS_SECONDS}" ros2 topic echo "${topic}" --field data >"${raw_file}" 2>&1 || true
  awk 'NF && $0 != "---" {line=$0} END {print line}' "${raw_file}"
}

field_value() {
  local text="$1"
  local key="$2"
  printf '%s\n' "${text}" | tr ' ' '\n' | awk -F= -v key="${key}" '$1 == key {print substr($0, length(key) + 2); exit}'
}

print_row() {
  local item="$1"
  local expected="$2"
  local actual="$3"
  local status="$4"
  printf '[process-freshness] AUDIT item=%s expected="%s" actual="%s" status=%s\n' \
    "${item}" "${expected}" "${actual}" "${status}"
}

check_binary_against_source() {
  local label="$1"
  local binary="$2"
  local source="$3"
  if [[ ! -e "${binary}" ]]; then
    print_row "${label}_binary" "installed binary exists" "${binary}" "FAIL"
    return 1
  fi
  if [[ ! -e "${source}" ]]; then
    print_row "${label}_source" "source exists" "${source}" "WARN"
    return 0
  fi
  local binary_mtime
  local source_mtime
  binary_mtime="$(float_epoch "$(stat -Lc %Y "${binary}" 2>/dev/null || echo 0)")"
  source_mtime="$(float_epoch "$(stat -Lc %Y "${source}" 2>/dev/null || echo 0)")"
  if (( binary_mtime >= source_mtime )); then
    print_row "${label}_build" "binary mtime >= source mtime" \
      "binary=${binary_mtime} source=${source_mtime}" "PASS"
    return 0
  fi
  print_row "${label}_build" "binary mtime >= source mtime" \
    "binary=${binary_mtime} source=${source_mtime}" "FAIL"
  return 1
}

check_processes_for_binary() {
  local label="$1"
  local pattern="$2"
  local binary="$3"
  local pids
  local status=0
  pids="$(pid_list_for_pattern "${pattern}")"
  if [[ -z "${pids}" ]]; then
    print_row "${label}_process" "running when service is expected" "not running" "WARN"
    return 0
  fi

  local binary_mtime=0
  if [[ -e "${binary}" ]]; then
    binary_mtime="$(float_epoch "$(stat -Lc %Y "${binary}" 2>/dev/null || echo 0)")"
  fi

  local pid
  for pid in ${pids}; do
    local exe
    local cmd
    local started
    exe="$(readlink -f "/proc/${pid}/exe" 2>/dev/null || true)"
    cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
    started="$(float_epoch "$(process_start_epoch "${pid}")")"
    if [[ "${binary_mtime}" -gt 0 && "${started}" -gt 0 && "${started}" -lt "${binary_mtime}" ]]; then
      print_row "${label}_pid_${pid}" "process start >= binary mtime" \
        "start=${started} binary=${binary_mtime} exe=${exe} cmd=${cmd}" "FAIL"
      status=1
    else
      print_row "${label}_pid_${pid}" "process start >= binary mtime" \
        "start=${started} binary=${binary_mtime} exe=${exe} cmd=${cmd}" "PASS"
    fi
  done
  return "${status}"
}

status=0

POINTCLOUD_AXIS_BIN="${NJRH_POINTCLOUD_REMAP_CPP_BIN:-${NJRH_PROJECT_ROOT}/install/robot_hesai_jt128/lib/robot_hesai_jt128/pointcloud_axis_remap_node}"
POINTCLOUD_AXIS_SRC="${NJRH_PROJECT_ROOT}/src/robot_hesai_jt128/src/pointcloud_axis_remap_node.cpp"
API_BIN="${NJRH_ROBOT_API_SERVER_BIN:-${NJRH_PROJECT_ROOT}/install/robot_api_server/lib/robot_api_server/robot_api_server_node}"
API_SRC="${NJRH_PROJECT_ROOT}/src/robot_api_server/src/robot_api_server_node.cpp"

echo "[process-freshness] project_root=${NJRH_PROJECT_ROOT}"
echo "[process-freshness] runtime_log_dir=${NJRH_RUNTIME_LOG_DIR}"
echo "[process-freshness] check_all=${CHECK_ALL}"

check_binary_against_source pointcloud_axis_remap "${POINTCLOUD_AXIS_BIN}" "${POINTCLOUD_AXIS_SRC}" || status=1
check_binary_against_source robot_api_server "${API_BIN}" "${API_SRC}" ||
  { [[ "${CHECK_ALL}" == "true" ]] && status=1 || true; }

check_processes_for_binary pointcloud_axis_remap "pointcloud_axis_remap" "${POINTCLOUD_AXIS_BIN}" || status=1
check_processes_for_binary robot_api_server "robot_api_server_node|robot_api_server/robot_api_server_node" "${API_BIN}" ||
  { [[ "${CHECK_ALL}" == "true" ]] && status=1 || true; }

axis_status="$(status_once /lidar/axis_remap_status)"
if [[ -z "${axis_status}" ]]; then
  print_row "axis_status" "status sample exists" "missing" "WARN"
else
  echo "[process-freshness] axis_status=${axis_status}"
  for required_field in \
    raw_interarrival_ms_avg \
    raw_interarrival_ms_max \
    lidar_points_publish_interval_ms_avg \
    lidar_points_publish_interval_ms_max \
    trunk_publish_gap_over_100ms_count \
    last_raw_callback_duration_ms \
    last_publish_outputs_start_to_end_ms \
    nav_branch_attempt_hz \
    local_branch_attempt_hz
  do
    if [[ -n "$(field_value "${axis_status}" "${required_field}" || true)" ]]; then
      print_row "axis_status_${required_field}" "field present" "present" "PASS"
    else
      print_row "axis_status_${required_field}" "field present" "missing" "FAIL"
      status=1
    fi
  done
fi

if [[ "${status}" -ne 0 && "${STRICT}" == "true" ]]; then
  echo "[process-freshness] FAIL stale process or old binary suspected"
  exit 1
fi

if [[ "${status}" -ne 0 ]]; then
  echo "[process-freshness] WARN stale process or old binary suspected"
else
  echo "[process-freshness] PASS"
fi

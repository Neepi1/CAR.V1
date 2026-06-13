#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

fail_count=0
warn_count=0
pass_count=0

expand_cpuset() {
  local spec="$1"
  python3 - "$spec" <<'PY'
import sys

spec = sys.argv[1].strip()
values = []
for part in spec.split(","):
    part = part.strip()
    if not part:
        continue
    if "-" in part:
        start, end = part.split("-", 1)
        values.extend(range(int(start), int(end) + 1))
    else:
        values.append(int(part))
print(",".join(str(v) for v in sorted(set(values))))
PY
}

cpuset_equal() {
  [[ "$(expand_cpuset "$1")" == "$(expand_cpuset "$2")" ]]
}

cpuset_allows_any() {
  local actual="$1"
  local forbidden="$2"
  python3 - "$actual" "$forbidden" <<'PY'
import sys

def expand(spec):
    out = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            out.update(range(int(start), int(end) + 1))
        else:
            out.add(int(part))
    return out

raise SystemExit(0 if expand(sys.argv[1]) & expand(sys.argv[2]) else 1)
PY
}

status_of_pid() {
  local pid="$1"
  awk '/^Cpus_allowed_list:/ {print $2; exit}' "/proc/${pid}/status" 2>/dev/null || true
}

psr_of_pid() {
  local pid="$1"
  ps -p "${pid}" -o psr= 2>/dev/null | awk 'NF {print $1; exit}' || true
}

pcpu_of_pid() {
  local pid="$1"
  ps -p "${pid}" -o pcpu= 2>/dev/null | awk 'NF {print $1; exit}' || true
}

find_pids() {
  local pattern="$1"
  ps -eo pid=,args= | awk -v pat="${pattern}" 'index($0, pat) > 0 && index($0, "inspect_runtime_cpu_affinity.sh") == 0 && index($0, "awk -v pat") == 0 {print $1}'
}

emit_row() {
  local name="$1"
  local pid="$2"
  local expected="$3"
  local severity="$4"
  local forbidden="${5:-}"
  local allowed psr pcpu status detail
  allowed="$(status_of_pid "${pid}")"
  psr="$(psr_of_pid "${pid}")"
  pcpu="$(pcpu_of_pid "${pid}")"
  status="PASS"
  detail="ok"

  if [[ -z "${allowed}" ]]; then
    status="FAIL"
    detail="missing_Cpus_allowed_list"
  elif [[ -n "${expected}" ]] && ! cpuset_equal "${allowed}" "${expected}"; then
    if [[ "${severity}" == "warn" ]]; then
      status="WARN"
    else
      status="FAIL"
    fi
    detail="expected_${expected}"
  fi
  if [[ -n "${forbidden}" ]] && [[ -n "${allowed}" ]] && cpuset_allows_any "${allowed}" "${forbidden}"; then
    status="FAIL"
    detail="allows_forbidden_${forbidden}"
  fi

  case "${status}" in
    PASS) pass_count=$((pass_count + 1)) ;;
    WARN) warn_count=$((warn_count + 1)) ;;
    FAIL) fail_count=$((fail_count + 1)) ;;
  esac
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${status}" "${name}" "${pid}" "${allowed:-missing}" "${psr:-?}" "${pcpu:-?}" "${expected:-none}" "${detail}"
}

check_process() {
  local name="$1"
  local pattern="$2"
  local expected="$3"
  local missing_status="$4"
  local mismatch_severity="$5"
  local forbidden="${6:-}"
  local pids
  pids="$(find_pids "${pattern}" | sort -n | uniq || true)"
  if [[ -z "${pids}" ]]; then
    case "${missing_status}" in
      pass)
        pass_count=$((pass_count + 1))
        printf 'PASS\t%s\tmissing\tmissing\t?\t?\t%s\tnot_running\n' "${name}" "${expected:-none}"
        ;;
      fail)
        fail_count=$((fail_count + 1))
        printf 'FAIL\t%s\tmissing\tmissing\t?\t?\t%s\tmissing\n' "${name}" "${expected:-none}"
        ;;
      *)
        warn_count=$((warn_count + 1))
        printf 'WARN\t%s\tmissing\tmissing\t?\t?\t%s\tmissing_or_mode_disabled\n' "${name}" "${expected:-none}"
        ;;
    esac
    return 0
  fi

  local pid
  for pid in ${pids}; do
    emit_row "${name}" "${pid}" "${expected}" "${mismatch_severity}" "${forbidden}"
  done
}

expected_amcl_scan="${NJRH_CPUSET_AMCL_SCAN_ADMISSION:-${NJRH_CPUSET_LOCALIZATION:-6}}"
expected_amcl_scan_impl="${NJRH_AMCL_SCAN_ADMISSION_IMPL:-cpp}"
expected_amcl="${NJRH_CPUSET_AMCL:-${NJRH_CPUSET_LOCALIZATION:-6}}"
expected_bridge="${NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE:-7}"
expected_local_state="${NJRH_CPUSET_ROBOT_LOCAL_STATE:-${NJRH_CPUSET_TF_STATE:-2}}"
expected_local_preprocessor="${NJRH_CPUSET_ROBOT_LOCAL_STATE_ODOM_PREPROCESSOR:-${NJRH_CPUSET_TF_STATE:-2}}"
expected_controller="${NJRH_CPUSET_CONTROLLER_SERVER:-${NJRH_CPUSET_NAV_CONTROL:-3}}"
expected_hesai="${NJRH_CPUSET_HESAI_ROS_DRIVER:-${NJRH_CPUSET_LIDAR_DRIVER:-4}}"
expected_safety="${NJRH_CPUSET_ROBOT_SAFETY:-${NJRH_CPUSET_BASE_CONTROL:-1}}"
expected_ranger="${NJRH_CPUSET_RANGER_BASE_NODE:-${NJRH_CPUSET_BASE_CONTROL:-1}}"

printf 'status\tprocess\tpid\tCpus_allowed_list\tPSR\t%%CPU\texpected\tdetail\n'
case "${expected_amcl_scan_impl}" in
  cpp)
    check_process "amcl_scan_admission_node" "amcl_scan_admission_node" "${expected_amcl_scan}" fail fail "2,3,7"
    check_process "amcl_scan_admission_relay.py" "amcl_scan_admission_relay.py" "${expected_amcl_scan}" pass fail "2,3,7"
    python_fallback_pids="$(find_pids "amcl_scan_admission_relay.py" | sort -n | uniq || true)"
    if [[ -n "${python_fallback_pids}" ]]; then
      fail_count=$((fail_count + 1))
      printf 'FAIL\tamcl_scan_admission_impl\t%s\tmixed\t?\t?\tcpp\tpython_fallback_running_by_default\n' "${python_fallback_pids}"
    fi
    ;;
  python)
    check_process "amcl_scan_admission_node" "amcl_scan_admission_node" "${expected_amcl_scan}" pass fail "2,3,7"
    check_process "amcl_scan_admission_relay.py" "amcl_scan_admission_relay.py" "${expected_amcl_scan}" warn fail "2,3,7"
    cpp_pids="$(find_pids "amcl_scan_admission_node" | sort -n | uniq || true)"
    if [[ -n "${cpp_pids}" ]]; then
      fail_count=$((fail_count + 1))
      printf 'FAIL\tamcl_scan_admission_impl\t%s\tmixed\t?\t?\tpython\tcpp_relay_running_while_python_expected\n' "${cpp_pids}"
    fi
    ;;
  *)
    fail_count=$((fail_count + 1))
    printf 'FAIL\tamcl_scan_admission_impl\tmissing\tmissing\t?\t?\tcpp|python\tinvalid_%s\n' "${expected_amcl_scan_impl}"
    ;;
esac
check_process "amcl" "nav2_amcl" "${expected_amcl}" warn warn ""
check_process "robot_localization_bridge" "localization_bridge_node" "${expected_bridge}" fail fail ""
check_process "robot_local_state_ekf" "ekf_node --ros-args" "${expected_local_state}" fail fail ""
check_process "wheel_odom_ekf_input" "__node:=wheel_odom_ekf_input" "${expected_local_preprocessor}" fail fail ""
check_process "controller_server" "controller_server" "${expected_controller}" fail fail ""
check_process "hesai_accel_driver_node" "hesai_accel_driver_node" "${expected_hesai}" fail fail ""
check_process "robot_safety" "robot_safety_node" "${expected_safety}" fail fail ""
check_process "ranger_base_node" "ranger_base_node" "${expected_ranger}" fail fail ""

echo "summary=PASS:${pass_count} WARN:${warn_count} FAIL:${fail_count}"
if [[ "${fail_count}" -gt 0 ]]; then
  exit 1
fi
exit 0

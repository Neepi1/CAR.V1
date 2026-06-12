#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
REPORT_DIR="${PROJECT_ROOT}/reports"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
REPORT_FILE="${REPORT_DIR}/local_state_input_rates_${TIMESTAMP}.md"
HZ_TIMEOUT_SEC="${NJRH_LOCAL_STATE_RATE_HZ_TIMEOUT_SEC:-5}"
UDP_WINDOW_SEC="${NJRH_LOCAL_STATE_RATE_UDP_WINDOW_SEC:-2}"
EKF_PROFILE="${LOCAL_STATE_EKF_PROFILE:-${NJRH_LOCAL_STATE_EKF_PROFILE:-wheel_imu}}"

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
else
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${PROJECT_ROOT}/install/setup.bash"
  fi
fi

mkdir -p "${REPORT_DIR}"

pass_count=0
warn_count=0
fail_count=0
findings=()

add_pass() {
  pass_count=$((pass_count + 1))
  findings+=("PASS|$1")
  echo "PASS $1"
}

add_warn() {
  warn_count=$((warn_count + 1))
  findings+=("WARN|$1")
  echo "WARN $1"
}

add_fail() {
  fail_count=$((fail_count + 1))
  findings+=("FAIL|$1")
  echo "FAIL $1"
}

float_cmp() {
  local left="$1"
  local op="$2"
  local right="$3"
  awk -v a="${left}" -v b="${right}" -v op="${op}" 'BEGIN {
    if (op == "<=") exit !(a <= b);
    if (op == "<") exit !(a < b);
    if (op == ">=") exit !(a >= b);
    if (op == ">") exit !(a > b);
    exit 1;
  }'
}

udp_rcvbuf_errors() {
  awk '
    /^Udp:/ && !header_seen {for (i = 1; i <= NF; ++i) idx[$i] = i; header_seen = 1; next}
    /^Udp:/ && header_seen {print $idx["RcvbufErrors"]; exit}
  ' /proc/net/snmp 2>/dev/null || echo ""
}

topic_rate() {
  local topic="$1"
  local window="$2"
  local output
  output="$(timeout "${HZ_TIMEOUT_SEC}" ros2 topic hz "${topic}" --window "${window}" 2>&1 || true)"
  printf '%s\n' "${output}" >"/tmp/njrh_local_state_hz_${topic//\//_}.txt"
  printf '%s\n' "${output}" |
    awk '/average rate:/ {rate=$3} END {if (rate != "") print rate}'
}

topic_info_text() {
  local topic="$1"
  ros2 topic info -v "${topic}" 2>&1 || true
}

publisher_count() {
  local topic="$1"
  ros2 topic info "${topic}" 2>/dev/null | awk '/Publisher count:/ {print $3; exit}'
}

contains_node_name() {
  local text="$1"
  local node="$2"
  printf '%s\n' "${text}" | grep -Eq "Node name: ${node}([[:space:]]|$)"
}

rate_rows=()

record_rate() {
  local topic="$1"
  local window="$2"
  local rate
  rate="$(topic_rate "${topic}" "${window}")"
  rate_rows+=("${topic}|${rate:-missing}")
  echo "RATE ${topic} ${rate:-missing}"
}

classify_rates() {
  local topic rate
  for row in "${rate_rows[@]}"; do
    topic="${row%%|*}"
    rate="${row#*|}"
    if [[ -z "${rate}" || "${rate}" == "missing" ]]; then
      add_fail "${topic} rate missing"
      continue
    fi
    case "${topic}" in
      /lidar_imu)
        if float_cmp "${rate}" ">=" 300 && float_cmp "${rate}" "<=" 500; then
          add_pass "${topic} remains high-rate at ${rate}Hz"
        else
          add_warn "${topic} expected roughly 300-500Hz, observed ${rate}Hz"
        fi
        ;;
      /lidar_imu_bias_corrected)
        if float_cmp "${rate}" "<=" 110; then
          add_pass "${topic} is rate-limited at ${rate}Hz"
        elif float_cmp "${rate}" "<=" 130; then
          add_warn "${topic} is above target but below 130Hz: ${rate}Hz"
        elif float_cmp "${rate}" ">=" 300; then
          add_fail "${topic} still near raw IMU rate: ${rate}Hz"
        else
          add_fail "${topic} exceeds rate limit target: ${rate}Hz"
        fi
        ;;
      /local_state/imu_bias)
        if float_cmp "${rate}" "<=" 15; then
          add_pass "${topic} diagnostic rate is bounded at ${rate}Hz"
        else
          add_fail "${topic} diagnostic rate exceeds 15Hz: ${rate}Hz"
        fi
        ;;
      /wheel/odom_ekf)
        if float_cmp "${rate}" ">=" 45 && float_cmp "${rate}" "<=" 55; then
          add_pass "${topic} is stable near 50Hz: ${rate}Hz"
        elif float_cmp "${rate}" ">" 55 && float_cmp "${rate}" "<=" 65; then
          add_warn "${topic} is above target but below 65Hz: ${rate}Hz"
        elif float_cmp "${rate}" ">=" 80; then
          add_fail "${topic} still suggests callback+timer double publish: ${rate}Hz"
        else
          add_warn "${topic} outside 45-55Hz target: ${rate}Hz"
        fi
        ;;
      /local_state/odometry)
        if float_cmp "${rate}" ">=" 45 && float_cmp "${rate}" "<=" 55; then
          add_pass "${topic} EKF output is near 50Hz: ${rate}Hz"
        elif float_cmp "${rate}" ">=" 40 && float_cmp "${rate}" "<" 45; then
          add_warn "${topic} is slightly below target: ${rate}Hz"
        else
          add_fail "${topic} outside acceptable EKF output range: ${rate}Hz"
        fi
        ;;
    esac
  done
}

udp_before="$(udp_rcvbuf_errors)"

if [[ "${EKF_PROFILE}" == "wheel_only" ]]; then
  add_pass "LOCAL_STATE_EKF_PROFILE=wheel_only skips IMU rate checks"
else
  record_rate /lidar_imu 50
  record_rate /lidar_imu_bias_corrected 30
  record_rate /local_state/imu_bias 20
fi
record_rate /wheel/odom 20
record_rate /wheel/odom_ekf 20
record_rate /local_state/odometry 20

sleep "${UDP_WINDOW_SEC}"
udp_after="$(udp_rcvbuf_errors)"
udp_delta="missing"
if [[ -n "${udp_before}" && -n "${udp_after}" ]]; then
  udp_delta=$((udp_after - udp_before))
fi

classify_rates

node_list="$(ros2 node list 2>&1 || true)"
if printf '%s\n' "${node_list}" | grep -qx "/robot_local_state"; then
  add_pass "/robot_local_state is visible in ROS graph"
else
  if pgrep -f "ekf_node --ros-args.*__node:=robot_local_state|robot_localization/ekf_node" >/dev/null 2>&1; then
    add_fail "robot_local_state EKF process is alive but /robot_local_state is missing from ROS graph"
  else
    add_fail "/robot_local_state is missing from ROS graph"
  fi
fi

local_odom_pubs="$(publisher_count /local_state/odometry || true)"
if [[ "${local_odom_pubs:-0}" -gt 0 ]]; then
  add_pass "/local_state/odometry publisher count is ${local_odom_pubs}"
else
  add_fail "/local_state/odometry publisher count is 0"
fi

tf_pubs="$(publisher_count /tf || true)"
tf_info="$(topic_info_text /tf)"
if [[ "${tf_pubs:-0}" -gt 0 ]]; then
  add_pass "/tf publisher count is ${tf_pubs}"
else
  add_fail "/tf publisher count is 0"
fi
if contains_node_name "${tf_info}" "robot_local_state"; then
  add_pass "/tf has robot_local_state publisher"
else
  add_fail "/tf is missing robot_local_state publisher"
fi
if contains_node_name "${tf_info}" "robot_localization_bridge"; then
  add_pass "/tf has robot_localization_bridge publisher"
else
  add_warn "/tf is missing robot_localization_bridge publisher"
fi

wheel_info="$(topic_info_text /wheel/odom_ekf)"
if contains_node_name "${wheel_info}" "robot_local_state"; then
  add_pass "/wheel/odom_ekf has EKF subscriber"
else
  add_fail "/wheel/odom_ekf is missing EKF subscriber"
fi

imu_info="$(topic_info_text /lidar_imu_bias_corrected)"
if [[ "${EKF_PROFILE}" == "wheel_only" ]]; then
  if contains_node_name "${imu_info}" "robot_local_state"; then
    add_fail "wheel_only profile must not have EKF subscriber on /lidar_imu_bias_corrected"
  else
    add_pass "wheel_only profile has no EKF subscriber on /lidar_imu_bias_corrected"
  fi
else
  if contains_node_name "${imu_info}" "robot_local_state"; then
    add_pass "/lidar_imu_bias_corrected has EKF subscriber"
  else
    add_fail "/lidar_imu_bias_corrected is missing EKF subscriber"
  fi
fi

if [[ "${udp_delta}" == "missing" ]]; then
  add_warn "Udp RcvbufErrors delta unavailable"
elif [[ "${udp_delta}" -eq 0 ]]; then
  add_pass "Udp RcvbufErrors delta is 0"
elif [[ "${udp_delta}" -le 100 ]]; then
  add_warn "Udp RcvbufErrors delta is ${udp_delta}"
else
  add_fail "Udp RcvbufErrors growing rapidly: delta=${udp_delta}"
fi

ss_snapshot="$(ss -u -a -n -p 2>/dev/null | grep -E "ekf|robot_local_state|localization_bridge|localization_br|fastdds|dds" || true)"

{
  echo "# Local State Input Rate Verification"
  echo
  echo "- timestamp_utc: ${TIMESTAMP}"
  echo "- hz_timeout_sec: ${HZ_TIMEOUT_SEC}"
  echo "- udp_window_sec: ${UDP_WINDOW_SEC}"
  echo "- local_state_ekf_profile: ${EKF_PROFILE}"
  echo "- udp_rcvbuf_errors_before: ${udp_before:-missing}"
  echo "- udp_rcvbuf_errors_after: ${udp_after:-missing}"
  echo "- udp_rcvbuf_errors_delta: ${udp_delta}"
  echo
  echo "## Rates"
  echo
  echo "| topic | observed_hz |"
  echo "| --- | ---: |"
  for row in "${rate_rows[@]}"; do
    echo "| ${row%%|*} | ${row#*|} |"
  done
  echo
  echo "## Findings"
  echo
  echo "| status | detail |"
  echo "| --- | --- |"
  for finding in "${findings[@]}"; do
    echo "| ${finding%%|*} | ${finding#*|} |"
  done
  echo
  echo "## ROS Graph"
  echo
  echo '```text'
  printf '%s\n' "${node_list}"
  echo '```'
  echo
  echo "## Topic Info"
  echo
  echo "### /local_state/odometry"
  echo '```text'
  topic_info_text /local_state/odometry
  echo '```'
  echo
  echo "### /tf"
  echo '```text'
  printf '%s\n' "${tf_info}"
  echo '```'
  echo
  echo "### /wheel/odom_ekf"
  echo '```text'
  printf '%s\n' "${wheel_info}"
  echo '```'
  echo
  echo "### /lidar_imu_bias_corrected"
  echo '```text'
  printf '%s\n' "${imu_info}"
  echo '```'
  echo
  echo "## DDS/UDP Socket Snapshot"
  echo
  echo '```text'
  printf '%s\n' "${ss_snapshot:-no matching sockets}"
  echo '```'
} >"${REPORT_FILE}"

echo "report=${REPORT_FILE}"
echo "summary=PASS:${pass_count} WARN:${warn_count} FAIL:${fail_count}"

if [[ "${fail_count}" -gt 0 ]]; then
  exit 1
fi
if [[ "${warn_count}" -gt 0 ]]; then
  exit 2
fi
exit 0

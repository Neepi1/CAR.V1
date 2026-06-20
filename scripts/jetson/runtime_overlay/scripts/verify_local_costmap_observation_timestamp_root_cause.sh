#!/usr/bin/env bash
set -euo pipefail

PREFIX="[local-costmap-timestamp-audit]"
REPORT_DIR="${REPORT_DIR:-reports}"
mkdir -p "${REPORT_DIR}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
REPORT="${REPORT_DIR}/local_costmap_timestamp_audit_${STAMP}.md"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck source=/dev/null
  set +u
  source install/setup.bash
  set -u
fi

pass_count=0
warn_count=0
fail_count=0

pass() { echo "${PREFIX} PASS $*"; pass_count=$((pass_count + 1)); }
warn() { echo "${PREFIX} WARN $*"; warn_count=$((warn_count + 1)); }
fail() { echo "${PREFIX} FAIL $*"; fail_count=$((fail_count + 1)); }

plain_status_data() {
  sed -e 's/^data: //' -e 's/^"//' -e 's/"$//' -e 's/\\"/"/g'
}

status_value() {
  local data="$1"
  local key="$2"
  awk -v key="${key}" '
    {
      for (i = 1; i <= NF; ++i) {
        split($i, kv, "=")
        if (kv[1] == key) {
          print kv[2]
          exit
        }
      }
    }' <<<"${data}"
}

param_value() {
  local node="$1"
  local name="$2"
  timeout 5 ros2 param get "${node}" "${name}" 2>/dev/null |
    sed -e 's/^String value is: //' -e 's/^Double value is: //' -e 's/^Integer value is: //' -e 's/^Bool value is: //' || true
}

float_gt() {
  awk -v a="${1:-nan}" -v b="${2:-nan}" 'BEGIN { exit !((a + 0) > (b + 0)) }'
}

float_le() {
  awk -v a="${1:-nan}" -v b="${2:-nan}" 'BEGIN { exit !((a + 0) <= (b + 0)) }'
}

float_abs_diff_le() {
  awk -v a="${1:-nan}" -v b="${2:-nan}" -v limit="${3:-nan}" '
    BEGIN {
      d = (a + 0) - (b + 0)
      if (d < 0) d = -d
      exit !(d <= (limit + 0))
    }'
}

header_once() {
  local topic="$1"
  timeout 8 ros2 topic echo "${topic}" --once --field header 2>/dev/null || true
}

header_frame() {
  awk -F': ' '/frame_id:/ {gsub(/"/, "", $2); print $2; exit}' <<<"${1:-}"
}

header_sec() {
  awk -F': ' '/sec:/ {gsub(/"/, "", $2); print $2; exit}' <<<"${1:-}"
}

header_nanosec() {
  awk -F': ' '/nanosec:/ {gsub(/"/, "", $2); print $2; exit}' <<<"${1:-}"
}

header_age_ms() {
  local sec="$1"
  local nanosec="$2"
  if [[ -z "${sec}" || -z "${nanosec}" ]]; then
    echo "-1"
    return 0
  fi
  python3 - "${sec}" "${nanosec}" <<'PY'
import sys
import time

sec = int(sys.argv[1])
nsec = int(sys.argv[2])
stamp = sec + nsec * 1.0e-9
if stamp <= 0:
    print("-1")
else:
    print(f"{(time.time() - stamp) * 1000.0:.3f}")
PY
}

latest_value() {
  local primary="$1"
  local fallback="$2"
  if [[ -n "${primary}" ]]; then
    echo "${primary}"
  else
    echo "${fallback}"
  fi
}

echo "${PREFIX} collecting read-only timestamp diagnostics"

accel_status_raw="$(timeout 12 ros2 topic echo /lidar/pointcloud_accel_status --once --field data 2>/dev/null | plain_status_data || true)"
scan_header="$(header_once /scan)"

scan_frame="$(header_frame "${scan_header}")"
scan_sec="$(header_sec "${scan_header}")"
scan_nsec="$(header_nanosec "${scan_header}")"
scan_header_age_once_ms="$(header_age_ms "${scan_sec}" "${scan_nsec}")"

global_frame="$(param_value /local_costmap/local_costmap global_frame)"
robot_base_frame="$(param_value /local_costmap/local_costmap robot_base_frame)"
scan_sensor_frame="$(param_value /local_costmap/local_costmap obstacle_layer.scan.sensor_frame)"
tf_filter_tolerance="$(param_value /local_costmap/local_costmap obstacle_layer.tf_filter_tolerance)"
controller_state="$(timeout 5 ros2 lifecycle get /controller_server 2>/dev/null || true)"

tf_echo_output="$(timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -60 || true)"
tf_monitor_output="$(timeout 8 ros2 run tf2_ros tf2_monitor odom base_link 2>&1 | head -80 || true)"

drop_logs=""
if command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx 'NJRH-car'; then
  drop_logs="$(docker logs NJRH-car 2>&1 | grep -i 'Message Filter dropping message' | tail -30 || true)"
elif command -v journalctl >/dev/null 2>&1; then
  drop_logs="$(journalctl --no-pager -n 1000 2>/dev/null | grep -i 'Message Filter dropping message' | tail -30 || true)"
fi
drop_count="$(grep -ci 'Message Filter dropping message' <<<"${drop_logs}" || true)"

raw_header_age_ms="$(status_value "${accel_status_raw}" raw_header_age_ms)"
raw_header_age_ms="$(latest_value "${raw_header_age_ms}" "$(status_value "${accel_status_raw}" last_raw_stamp_age_ms)")"
latest_stamp_age_ms="$(status_value "${accel_status_raw}" latest_internal_buffer_stamp_age_ms)"
latest_update_age_ms="$(status_value "${accel_status_raw}" latest_internal_buffer_update_age_ms)"
latest_seq="$(status_value "${accel_status_raw}" latest_internal_buffer_seq)"
obstacle_output_header_age_ms="$(status_value "${accel_status_raw}" obstacle_output_header_age_ms)"
obstacle_output_source_age_ms="$(status_value "${accel_status_raw}" obstacle_output_source_age_ms)"
obstacle_output_frame_id="$(status_value "${accel_status_raw}" obstacle_output_frame_id)"
clearing_output_header_age_ms="$(status_value "${accel_status_raw}" clearing_output_header_age_ms)"
clearing_output_source_age_ms="$(status_value "${accel_status_raw}" clearing_output_source_age_ms)"
scan_output_header_age_ms="$(status_value "${accel_status_raw}" scan_output_header_age_ms)"
scan_output_source_age_ms="$(status_value "${accel_status_raw}" scan_output_source_age_ms)"
scan_output_frame_id="$(status_value "${accel_status_raw}" scan_output_frame_id)"
suspect_over_100="$(status_value "${accel_status_raw}" tf_drop_suspect_obstacle_header_age_over_100ms_count)"
suspect_over_200="$(status_value "${accel_status_raw}" tf_drop_suspect_obstacle_header_age_over_200ms_count)"

if [[ -z "${scan_output_header_age_ms}" ]]; then
  scan_output_header_age_ms="${scan_header_age_once_ms}"
fi
if [[ -z "${scan_output_frame_id}" ]]; then
  scan_output_frame_id="${scan_frame}"
fi

case_id="CASE_G_UNKNOWN_NEEDS_BAG"
case_detail="status fields are unavailable or contradictory; record a short bag if the drop persists"
result="WARN"

if [[ "${controller_state}" != active* ]]; then
  case_id="CASE_G_UNKNOWN_NEEDS_BAG"
  case_detail="controller_server is not active, timestamp diagnosis cannot be trusted"
  result="FAIL"
elif [[ "${global_frame}" != "odom" || "${robot_base_frame}" != "base_link" ]]; then
  case_id="CASE_F_FRAME_MISMATCH"
  case_detail="local_costmap frame contract is not odom/base_link"
  result="FAIL"
elif [[ "${scan_frame}" != "lidar_level_link" && "${scan_output_frame_id}" != "lidar_level_link" ]]; then
  case_id="CASE_F_FRAME_MISMATCH"
  case_detail="scan output frame is not lidar_level_link"
  result="FAIL"
elif [[ "${scan_sensor_frame}" != "lidar_level_link" ]]; then
  case_id="CASE_F_FRAME_MISMATCH"
  case_detail="local costmap scan sensor_frame contract is not lidar_level_link"
  result="FAIL"
elif [[ -n "${raw_header_age_ms}" ]] && float_gt "${raw_header_age_ms}" 150; then
  case_id="CASE_A_RAW_STAMP_ALREADY_OLD"
  case_detail="raw cloud header is already older than 150ms before accel/local obstacle output"
  result="WARN"
elif [[ -n "${raw_header_age_ms}" && -n "${latest_stamp_age_ms}" && -n "${latest_update_age_ms}" ]] &&
  float_le "${raw_header_age_ms}" 150 &&
  { float_gt "${latest_stamp_age_ms}" 150 || float_gt "${latest_update_age_ms}" 150; }; then
  case_id="CASE_B_INTERNAL_BUFFER_STALE"
  case_detail="raw header age is normal but latest internal buffer stamp/update age is stale"
  result="WARN"
elif [[ -n "${latest_update_age_ms}" && -n "${scan_output_header_age_ms}" && -n "${scan_output_source_age_ms}" ]] &&
  float_le "${latest_update_age_ms}" 150 &&
  float_gt "${scan_output_header_age_ms}" 150 &&
  float_abs_diff_le "${scan_output_header_age_ms}" "${scan_output_source_age_ms}" 50; then
  case_id="CASE_C_OUTPUT_REUSES_OLD_SOURCE_STAMP"
  case_detail="scan output reuses a source stamp that is older than the TF cache window seen by local costmap"
  result="WARN"
elif [[ "${drop_count}" -gt 0 && -n "${scan_output_header_age_ms}" ]] &&
  float_le "${scan_output_header_age_ms}" 100; then
  case_id="CASE_D_TF_CACHE_TIME_AHEAD"
  case_detail="scan header age is fresh but MessageFilter still reports earlier-than-cache"
  result="WARN"
elif [[ "${drop_count}" -eq 0 && -n "${scan_output_header_age_ms}" ]] &&
  float_le "${scan_output_header_age_ms}" 150; then
  case_id="CASE_E_STARTUP_TF_CACHE_WARMUP"
  case_detail="no current MessageFilter drop was found and current scan stamp is within 150ms; prior drops were likely startup TF cache warm-up if they only appeared during activation"
  result="PASS"
fi

case "${result}" in
  PASS) pass "${case_id}: ${case_detail}" ;;
  FAIL) fail "${case_id}: ${case_detail}" ;;
  *) warn "${case_id}: ${case_detail}" ;;
esac

if [[ "${controller_state}" == active* ]]; then
  pass "controller_server active"
else
  fail "controller_server state=${controller_state:-missing}"
fi

if [[ "${global_frame}" == "odom" && "${robot_base_frame}" == "base_link" ]]; then
  pass "local_costmap frames global_frame=odom robot_base_frame=base_link"
else
  fail "local_costmap frames global_frame=${global_frame:-missing} robot_base_frame=${robot_base_frame:-missing}"
fi

if [[ "${scan_sensor_frame}" == "lidar_level_link" ]]; then
  pass "local_costmap scan sensor_frame=lidar_level_link"
else
  fail "sensor_frame mismatch scan=${scan_sensor_frame:-missing}"
fi

if grep -q 'Translation:' <<<"${tf_echo_output}"; then
  pass "odom -> base_link tf2_echo available"
else
  warn "odom -> base_link tf2_echo did not produce a transform in the short sample"
fi

{
  echo "# Local Costmap Timestamp Root-Cause Audit"
  echo
  echo "- generated_at: ${STAMP}"
  echo "- result: ${result}"
  echo "- case: ${case_id}"
  echo "- detail: ${case_detail}"
  echo
  echo "## Runtime Params"
  echo
  echo "| key | value |"
  echo "| --- | --- |"
  echo "| local_costmap.global_frame | ${global_frame:-missing} |"
  echo "| local_costmap.robot_base_frame | ${robot_base_frame:-missing} |"
  echo "| scan.sensor_frame | ${scan_sensor_frame:-missing} |"
  echo "| obstacle_layer.tf_filter_tolerance | ${tf_filter_tolerance:-missing_or_unsupported} |"
  echo "| controller_server | ${controller_state:-missing} |"
  echo
  echo "## Timestamp Metrics"
  echo
  echo "| metric | value_ms |"
  echo "| --- | ---: |"
  echo "| raw_header_age_ms | ${raw_header_age_ms:-missing} |"
  echo "| latest_internal_buffer_stamp_age_ms | ${latest_stamp_age_ms:-missing} |"
  echo "| latest_internal_buffer_update_age_ms | ${latest_update_age_ms:-missing} |"
  echo "| scan_output_header_age_ms | ${scan_output_header_age_ms:-missing} |"
  echo "| scan_output_source_age_ms | ${scan_output_source_age_ms:-missing} |"
  echo "| scan_header_once_age_ms | ${scan_header_age_once_ms:-missing} |"
  echo
  echo "## Frames And Counters"
  echo
  echo "| key | value |"
  echo "| --- | --- |"
  echo "| scan_header.frame_id | ${scan_frame:-missing} |"
  echo "| scan_output_frame_id | ${scan_output_frame_id:-missing} |"
  echo "| latest_internal_buffer_seq | ${latest_seq:-missing} |"
  echo "| tf_drop_suspect_obstacle_header_age_over_100ms_count | ${suspect_over_100:-missing} |"
  echo "| tf_drop_suspect_obstacle_header_age_over_200ms_count | ${suspect_over_200:-missing} |"
  echo "| MessageFilter drop tail count | ${drop_count} |"
  echo
  echo "## Accel Status"
  echo
  echo '```text'
  echo "${accel_status_raw:-missing}"
  echo '```'
  echo
  echo "## TF Echo odom -> base_link"
  echo
  echo '```text'
  echo "${tf_echo_output:-missing}"
  echo '```'
  echo
  echo "## TF Monitor odom -> base_link"
  echo
  echo '```text'
  echo "${tf_monitor_output:-missing}"
  echo '```'
  echo
  echo "## MessageFilter Drop Log Tail"
  echo
  echo '```text'
  echo "${drop_logs:-no drop logs found by this read-only script}"
  echo '```'
  echo
  echo "## Recommendation"
  case "${case_id}" in
    CASE_A_RAW_STAMP_ALREADY_OLD)
      echo "Do not change the canonical cloud trunk first. Audit driver/use_timestamp_type/clock policy."
      ;;
    CASE_B_INTERNAL_BUFFER_STALE)
      echo "Audit pointcloud_accel_axis_node worker scheduling and latest internal buffer update latency."
      ;;
    CASE_C_OUTPUT_REUSES_OLD_SOURCE_STAMP)
      echo "Consider a later phase with publish_time plus max_source_age gate; do not apply it in this audit phase."
      ;;
    CASE_D_TF_CACHE_TIME_AHEAD)
      echo "Audit odom->base_link TF publisher stamps, tf2_monitor, and system clock/cache behavior."
      ;;
    CASE_E_STARTUP_TF_CACHE_WARMUP)
      echo "If drops only occur during activation, add a later TF cache warm-up before costmap/localization admission."
      ;;
    CASE_F_FRAME_MISMATCH)
      echo "Fix frame_id/sensor_frame contract."
      ;;
    *)
      echo "Record a short bag with /tf, /local_state/odometry, /scan, and local costmap logs."
      ;;
  esac
} >"${REPORT}"

echo "${PREFIX} report=${REPORT}"
echo "${PREFIX} raw_header_age_ms=${raw_header_age_ms:-missing} latest_internal_buffer_stamp_age_ms=${latest_stamp_age_ms:-missing} latest_internal_buffer_update_age_ms=${latest_update_age_ms:-missing}"
echo "${PREFIX} scan_output_header_age_ms=${scan_output_header_age_ms:-missing} scan_output_source_age_ms=${scan_output_source_age_ms:-missing}"
echo "${PREFIX} tf_monitor=see_report drop_count=${drop_count}"

if [[ "${fail_count}" -gt 0 ]]; then
  exit 2
fi
if [[ "${warn_count}" -gt 0 ]]; then
  exit 1
fi
exit 0

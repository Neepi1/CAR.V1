#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
PREFIX="[dock-fw-verify]"

fail() {
  echo "${PREFIX} FAIL $*" >&2
  exit 1
}

pass() {
  echo "${PREFIX} PASS $*"
}

require_file() {
  local path="$1"
  [[ -f "${path}" ]] || fail "missing file: ${path}"
}

require_text() {
  local path="$1"
  local needle="$2"
  grep -Fq -- "${needle}" "${path}" || fail "${path} missing: ${needle}"
}

for path in \
  "${WORKSPACE_ROOT}/src/robot_api_server/src/robot_api_server_node.cpp" \
  "${WORKSPACE_ROOT}/src/robot_api_server/include/robot_api_server/docking_job_model.hpp" \
  "${WORKSPACE_ROOT}/src/robot_api_server/src/docking_job_model.cpp" \
  "${WORKSPACE_ROOT}/src/robot_api_server/config/robot_api_server.yaml" \
  "${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/robot_api_server.yaml" \
  "${WORKSPACE_ROOT}/src/robot_nav_config/config/docking.yaml" \
  "${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/docking.yaml" \
  "${WORKSPACE_ROOT}/src/robot_localization_bridge/src/localization_bridge_node.cpp" \
  "${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/localization_bridge.yaml"; do
  require_file "${path}"
done

api_cpp="${WORKSPACE_ROOT}/src/robot_api_server/src/robot_api_server_node.cpp"
job_hpp="${WORKSPACE_ROOT}/src/robot_api_server/include/robot_api_server/docking_job_model.hpp"
api_cfg="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/robot_api_server.yaml"
docking_cfg="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/docking.yaml"
bridge_cpp="${WORKSPACE_ROOT}/src/robot_localization_bridge/src/localization_bridge_node.cpp"

for phase in \
  DOCK_REQUESTED RESOLVE_DOCK_PROFILE BEFORE_PREDOCK_RELOCALIZE BEFORE_PREDOCK_SETTLE \
  NAV_TO_STAGING STAGING_NAV_SUCCEEDED PREDOCK_POSE_VERIFY PREDOCK_YAW_ALIGN \
  PREDOCK_YAW_ALIGN_SETTLE AFTER_PREDOCK_RELOCALIZE AFTER_PREDOCK_SETTLE \
  GS2_DOCK_DETECT FINE_DOCKING_ENTRY_CHECK FINE_ALIGN RESTAGE_RETRY; do
  require_text "${api_cpp}" "${phase}"
done

for symbol in \
  computeExpectedStagingYaw computePredockYawError computeContactYawError normalizeYawError \
  run_predock_yaw_align evaluate_fine_docking_entry classify_fine_docking_failure_code; do
  require_text "${api_cpp}" "${symbol}"
done

for code in \
  DOCK_FAILED_PREDOCK_NAV DOCK_FAILED_PREDOCK_RELOCALIZATION DOCK_FAILED_PREDOCK_SETTLE \
  PREDOCK_YAW_NOT_ALIGNED PREDOCK_YAW_HARD_FAIL PREDOCK_YAW_ALIGN_TIMEOUT \
  PREDOCK_YAW_ALIGN_MODE_SWITCHING_TIMEOUT PREDOCK_YAW_ALIGN_NO_YAW_MOTION \
  GS2_DOCK_DETECT_TIMEOUT FINE_DOCKING_ENTRY_CONDITION_FAILED \
  FINE_DOCKING_REJECTED_YAW_TOO_LARGE FINE_DOCKING_REJECTED_LATERAL_TOO_LARGE \
  FINE_DOCKING_TIMEOUT FINAL_INSERTION_NO_CONTACT DOCK_FAILED_SAFETY_BLOCKED; do
  require_text "${api_cpp}" "${code}"
done

require_text "${api_cpp}" 'predock_yaw_align_cmd_topic_ != "/cmd_vel_docking"'
require_text "${api_cpp}" "create_publisher<geometry_msgs::msg::Twist>(predock_yaw_align_cmd_topic_"
require_text "${api_cpp}" "mode_controller_status_topic_"
require_text "${api_cpp}" "actual_motion_mode_code == 2"
require_text "${api_cpp}" "docking_gs2_scan_topic_"
require_text "${api_cpp}" "set_global_correction_paused_for_docking(job_id, true, \"docking_fine\""
require_text "${api_cpp}" "set_global_correction_paused_for_docking("
require_text "${bridge_cpp}" "correction_pause_service"
require_text "${bridge_cpp}" "GLOBAL_CORRECTION_PAUSED"
require_text "${bridge_cpp}" "global_correction_paused"

for field in \
  dock_profile_id approach_direction contact_frame sensor_frame max_retries retry_count \
  predock_yaw_aligned predock_yaw_align_failure_code fine_entry_checked fine_entry_failure_code \
  global_correction_paused pause_reason display_pose_source; do
  require_text "${job_hpp}" "${field}"
done

for key in \
  docking_framework_state_machine_enabled predock_yaw_align_enabled predock_yaw_align_cmd_topic \
  predock_yaw_align_require_actual_spin fine_docking_entry_require_gs2_fresh \
  fine_docking_entry_require_predock_yaw_aligned docking_pause_global_correction_during_fine \
  localization_bridge_correction_pause_service mode_controller_status_topic; do
  require_text "${api_cfg}" "${key}"
done

require_text "${docking_cfg}" "dock_types:"
require_text "${docking_cfg}" "gs2_rear_charging_dock:"
require_text "${docking_cfg}" "staging_offset_m: 0.60"
require_text "${docking_cfg}" "approach_direction: reverse"

if grep -R "opennav_docking" "${WORKSPACE_ROOT}/src/robot_api_server" "${WORKSPACE_ROOT}/src/robot_docking_manager" >/dev/null 2>&1; then
  fail "opennav_docking dependency found in API/docking manager"
fi
if grep -R "ros2 topic pub" "${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/scripts/observe_docking_predock_yaw_align.sh" \
  "${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/scripts/run_docking_framework_ab.sh" >/dev/null 2>&1; then
  fail "diagnostic scripts must not publish velocity"
fi

pass "D3 docking framework state machine contracts are present"

#!/usr/bin/env bash
# verify_goal_completion_semantics.sh
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"

API_CPP="${WORKSPACE_ROOT}/src/robot_api_server/src/robot_api_server_node.cpp"
DOCKING_JOB_HPP="${WORKSPACE_ROOT}/src/robot_api_server/include/robot_api_server/docking_job_model.hpp"
DOCKING_JOB_CPP="${WORKSPACE_ROOT}/src/robot_api_server/src/docking_job_model.cpp"
ROBOT_API_CONFIG="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/robot_api_server.yaml"

PASS_COUNT=0
FAIL_COUNT=0

pass() { echo "PASS: $*"; PASS_COUNT=$((PASS_COUNT + 1)); }
fail() { echo "FAIL: $*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }

require_file() {
  local file="$1"
  [[ -f "${file}" ]] && pass "file exists: ${file}" || fail "missing file: ${file}"
}

require_text() {
  local file="$1"
  local text="$2"
  if grep -Fq -- "${text}" "${file}" 2>/dev/null; then
    pass "${text}"
  else
    fail "missing '${text}' in ${file}"
  fi
}

for file in "${API_CPP}" "${DOCKING_JOB_HPP}" "${DOCKING_JOB_CPP}" "${ROBOT_API_CONFIG}"; do
  require_file "${file}"
done

require_text "${API_CPP}" "goal_completion_policy"
require_text "${API_CPP}" "position_only"
require_text "${API_CPP}" "pose_required"
require_text "${API_CPP}" "dock_staging"
require_text "${API_CPP}" "goal_completion_policy=dock_staging is reserved for /api/v1/docking/start"
require_text "${API_CPP}" "goal_completion_policy=position_only; final yaw alignment not required"
require_text "${API_CPP}" "position_reached_yaw_aligning"
require_text "${API_CPP}" "task_complete"
require_text "${API_CPP}" "yaw_align_active"
require_text "${API_CPP}" "yaw_align_failed"
require_text "${API_CPP}" "final_pose_verified"
require_text "${API_CPP}" "REPOSITION_AFTER_YAW_DRIFT"
require_text "${API_CPP}" "navigation_max_reposition_after_yaw_retry"
require_text "${API_CPP}" "run_reposition_after_yaw_drift"
require_text "${API_CPP}" "ordinary_final_yaw_align_active_"
require_text "${API_CPP}" "predock_yaw_align_active_"
require_text "${API_CPP}" "cmd_owner_conflict_detected"
require_text "${API_CPP}" "final_yaw_align_blocked_by_docking"
require_text "${API_CPP}" "docking_blocked_by_final_yaw_align"
require_text "${API_CPP}" "PREDOCK_YAW_ALIGN_OWNER_CONFLICT"
require_text "${API_CPP}" "predock_yaw_align_cmd_topic_ != \"/cmd_vel_docking\""
require_text "${API_CPP}" "navigation_final_yaw_align_cmd_topic_ != \"/cmd_vel_nav\""
require_text "${API_CPP}" "navigation_final_yaw_align_cmd_topic_ != \"/cmd_vel_collision_checked\""
require_text "${API_CPP}" "fine_docking_entry_require_predock_yaw_aligned_ && !predock_yaw_aligned"
require_text "${API_CPP}" "goal_completion_policy == \"pose_required\""
require_text "${API_CPP}" "docking_job_.goal_completion_policy"
require_text "${API_CPP}" "dock_staging_handoff_ready"
require_text "${API_CPP}" "post_predock_settle_complete"
require_text "${API_CPP}" "global_correction_pause_applied"

require_text "${DOCKING_JOB_HPP}" "std::string goal_completion_policy"
require_text "${DOCKING_JOB_HPP}" "bool dock_staging_handoff_ready"
require_text "${DOCKING_JOB_HPP}" "bool predock_yaw_align_active"
require_text "${DOCKING_JOB_CPP}" "goal_completion_policy"
require_text "${DOCKING_JOB_CPP}" "dock_staging_handoff_ready"
require_text "${DOCKING_JOB_CPP}" "predock_yaw_align_active"

require_text "${ROBOT_API_CONFIG}" "navigation_default_goal_completion_policy: \"pose_required\""
require_text "${ROBOT_API_CONFIG}" "navigation_max_reposition_after_yaw_retry: 1"
require_text "${ROBOT_API_CONFIG}" "navigation_reposition_after_yaw_drift_timeout_sec: 30.0"
require_text "${ROBOT_API_CONFIG}" "predock_yaw_align_cmd_topic: \"/cmd_vel_docking\""

if grep -Fq 'navigation_final_yaw_align_cmd_topic: "/cmd_vel_docking"' "${ROBOT_API_CONFIG}"; then
  fail "ordinary final_yaw_align must not use /cmd_vel_docking"
else
  pass "ordinary final_yaw_align does not use /cmd_vel_docking"
fi

if grep -R "ros2 topic pub" "${SCRIPT_DIR}/observe_navigation_final_yaw_align.sh" >/dev/null 2>&1; then
  fail "observe_navigation_final_yaw_align.sh must be read-only"
else
  pass "observe_navigation_final_yaw_align.sh is read-only"
fi

echo "summary: pass=${PASS_COUNT} fail=${FAIL_COUNT}"
[[ "${FAIL_COUNT}" -eq 0 ]]

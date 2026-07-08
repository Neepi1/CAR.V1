#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
API_PARAMS_FILE="${API_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/robot_api_server.yaml}"
API_CPP="${WORKSPACE_ROOT}/src/robot_api_server/src/robot_api_server_node.cpp"
DOCKING_VERIFY="${SCRIPT_DIR}/verify_goal_completion_semantics.sh"
PREFIX="[nav2-native-goal]"
FAILURES=0
WARNINGS=0

pass() { echo "${PREFIX} PASS $*"; }
warn() { echo "${PREFIX} WARN $*"; WARNINGS=$((WARNINGS + 1)); }
fail() { echo "${PREFIX} FAIL $*"; FAILURES=$((FAILURES + 1)); }

read_yaml_value() {
  local file="$1"
  local dotted_key="$2"
  python3 - "$file" "$dotted_key" <<'PY'
import sys
path, dotted_key = sys.argv[1], sys.argv[2]
target = dotted_key.split(".")
stack = []
with open(path, encoding="utf-8") as f:
    for raw in f:
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or ":" not in line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        name, value = line.strip().split(":", 1)
        while stack and indent <= stack[-1][0]:
            stack.pop()
        current = [item[1] for item in stack] + [name]
        if current == target:
            print(value.strip().strip('"').strip("'"))
            raise SystemExit(0)
        if value.strip() == "":
            stack.append((indent, name))
raise SystemExit(1)
PY
}

require_file_text() {
  local label="$1"
  local file="$2"
  local text="$3"
  if grep -Fq "$text" "$file"; then
    pass "$label"
  else
    fail "$label missing: $text"
  fi
}

reject_file_text() {
  local label="$1"
  local file="$2"
  local text="$3"
  if grep -Fq "$text" "$file"; then
    fail "$label unexpectedly present: $text"
  else
    pass "$label"
  fi
}

check_static_nav2() {
  [[ -f "${NAV2_PARAMS_FILE}" ]] || { fail "missing NAV2 params: ${NAV2_PARAMS_FILE}"; return; }

  local follow_plugin primary rotate goal_plugins goal_stateful goal_xy goal_yaw planner local_frame controller_plugins fallback transform_tol
  follow_plugin="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.FollowPath.plugin" || true)"
  primary="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.FollowPath.primary_controller" || true)"
  rotate="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.FollowPath.rotate_to_goal_heading" || true)"
  goal_plugins="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.goal_checker_plugins" || true)"
  goal_stateful="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.goal_checker.stateful" || true)"
  goal_xy="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.goal_checker.xy_goal_tolerance" || true)"
  goal_yaw="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.goal_checker.yaw_goal_tolerance" || true)"
  planner="$(read_yaml_value "${NAV2_PARAMS_FILE}" "planner_server.ros__parameters.GridBased.plugin" || true)"
  local_frame="$(read_yaml_value "${NAV2_PARAMS_FILE}" "local_costmap.local_costmap.ros__parameters.global_frame" || true)"
  controller_plugins="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.controller_plugins" || true)"
  fallback="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.FollowPathFallback.plugin" || true)"
  transform_tol="$(read_yaml_value "${NAV2_PARAMS_FILE}" "controller_server.ros__parameters.FollowPath.transform_tolerance" || true)"

  [[ "${follow_plugin}" == "nav2_rotation_shim_controller::RotationShimController" ]] \
    && pass "FollowPath uses RotationShimController" \
    || fail "FollowPath.plugin=${follow_plugin:-missing}"
  [[ "${primary}" == "nav2_mppi_controller::MPPIController" ]] \
    && pass "RotationShim primary_controller preserves MPPI" \
    || fail "FollowPath.primary_controller=${primary:-missing}"
  [[ "${rotate}" == "true" ]] \
    && pass "rotate_to_goal_heading=true" \
    || fail "rotate_to_goal_heading=${rotate:-missing}"
  [[ "${goal_plugins}" == *"goal_checker"* ]] \
    && pass "goal_checker_plugins include goal_checker" \
    || fail "goal_checker_plugins=${goal_plugins:-missing}, expected goal_checker"
  [[ "${goal_stateful}" == "false" ]] \
    && pass "goal_checker.stateful=false" \
    || fail "goal_checker.stateful=${goal_stateful:-missing}"
  [[ "${goal_xy}" == "0.06" ]] \
    && pass "goal_checker.xy_goal_tolerance=${goal_xy}" \
    || fail "goal_checker.xy_goal_tolerance=${goal_xy:-missing}, expected 0.06"
  [[ "${goal_yaw}" == "0.05" ]] \
    && pass "goal_checker.yaw_goal_tolerance=${goal_yaw}" \
    || fail "goal_checker.yaw_goal_tolerance=${goal_yaw:-missing}, expected 0.05"
  [[ "${planner}" == "nav2_smac_planner/SmacPlanner2D" ]] \
    && pass "planner unchanged: ${planner}" \
    || fail "planner changed or missing: ${planner}"
  [[ "${local_frame}" == "odom" ]] \
    && pass "local_costmap.global_frame=odom" \
    || fail "local_costmap.global_frame=${local_frame:-missing}"
  [[ "${controller_plugins}" == *"FollowPath"* ]] \
    && pass "controller_plugins include FollowPath" \
    || fail "controller_plugins=${controller_plugins:-missing}, expected FollowPath"
  [[ "${fallback}" == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController" ]] \
    && pass "RPP fallback preserved" \
    || warn "FollowPathFallback.plugin=${fallback:-missing}"
  [[ "${transform_tol}" == "0.10" || "${transform_tol}" == "0.1" ]] \
    && pass "FollowPath.transform_tolerance unchanged (${transform_tol})" \
    || fail "FollowPath.transform_tolerance changed or missing: ${transform_tol}"
}

check_static_api() {
  [[ -f "${API_PARAMS_FILE}" ]] || { fail "missing API params: ${API_PARAMS_FILE}"; return; }
  require_file_text "pose_required remains default" "${API_PARAMS_FILE}" 'navigation_default_goal_completion_policy: "pose_required"'
  require_file_text "native goal completion enabled" "${API_PARAMS_FILE}" "nav2_native_goal_completion_enabled: true"
  require_file_text "rotation shim flag enabled" "${API_PARAMS_FILE}" "nav2_rotation_shim_enabled: true"
  require_file_text "ordinary API final_yaw bounded fallback enabled" "${API_PARAMS_FILE}" "api_final_yaw_align_fallback_enabled: true"
  require_file_text "API final_yaw enabled only as bounded fallback" "${API_PARAMS_FILE}" "navigation_final_yaw_align_enable: true"
  require_file_text "API Nav2 failed near-goal retry enabled" "${API_PARAMS_FILE}" "navigation_nav2_failed_near_goal_retry_enabled: true"
  require_file_text "API Nav2 failed near-goal retry count" "${API_PARAMS_FILE}" "navigation_nav2_failed_near_goal_retry_max_count: 1"
  require_file_text "API final_yaw waits for bridge smoothing" "${API_PARAMS_FILE}" "navigation_final_yaw_align_wait_bridge_smoothing: true"
  require_file_text "API final_yaw bridge wait timeout configured" "${API_PARAMS_FILE}" "navigation_final_yaw_align_bridge_wait_timeout_ms: 2000"
  require_file_text "API final_yaw pauses global correction" "${API_PARAMS_FILE}" "navigation_pause_global_correction_during_final_yaw: true"
  require_file_text "dock staging predock yaw still uses docking topic" "${API_PARAMS_FILE}" 'predock_yaw_align_cmd_topic: "/cmd_vel_docking"'
  require_file_text "API exposes native_nav2_goal_completion" "${API_CPP}" "native_nav2_goal_completion"
  require_file_text "API exposes api_final_yaw_align_enabled" "${API_CPP}" "api_final_yaw_align_enabled"
  require_file_text "API records commercial completion owner audit" "${API_CPP}" "commercial_final_verify=true"
  require_file_text "API success detail uses commercial final verification" "${API_CPP}" "navigation goal reached by commercial final verification"
  reject_file_text "Nav2 action failure no longer bypasses commercial final verification" "${API_CPP}" '"nav2_failed"'
  require_file_text "Nav2 failure can enter yaw fallback" "${API_CPP}" "nav2_failed_yaw_aligning"
  require_file_text "Nav2 near-goal failure aligns yaw first" "${API_CPP}" "nav2_failed_near_goal_yaw_aligning"
  require_file_text "Nav2 near-goal yaw-first documents retry order" "${API_CPP}" "aligning yaw before same-goal retry"
  require_file_text "Nav2 failure can retry near-goal pose" "${API_CPP}" "retry_nav2_after_nav2_failed_near_goal"
  require_file_text "Nav2 near-goal retry is audited" "${API_CPP}" "near_goal_nav2_retry_attempted=true"
  require_file_text "commercial final verification can accept recovered final pose" "${API_CPP}" "commercial_final_verify=true"
  require_file_text "yaw fallback waits for map smoothing first" "${API_CPP}" "waiting for bridge map->odom smoothing before final yaw alignment"
  require_file_text "yaw fallback refuses active map smoothing" "${API_CPP}" "bridge_smoothing_active_before_final_yaw"
  require_file_text "yaw fallback pauses AMCL/Isaac correction" "${API_CPP}" "global correction paused for final_yaw_align"
  require_file_text "yaw fallback releases AMCL/Isaac correction pause" "${API_CPP}" "global correction resumed after final_yaw_align"
  require_file_text "dock_staging remains reserved" "${API_CPP}" "goal_completion_policy=dock_staging is reserved for /api/v1/docking/start"
  require_file_text "predock yaw owner conflict remains guarded" "${API_CPP}" "PREDOCK_YAW_ALIGN_OWNER_CONFLICT"
  require_file_text "predock yaw still uses /cmd_vel_docking" "${API_CPP}" 'predock_yaw_align_cmd_topic_ != "/cmd_vel_docking"'
}

check_rotation_shim_available() {
  local prefix plugin_file
  prefix="$(timeout 8 ros2 pkg prefix nav2_rotation_shim_controller 2>/dev/null || true)"
  if [[ -z "${prefix}" ]]; then
    warn "runtime nav2_rotation_shim_controller package not visible; static checks still apply"
    return
  fi
  plugin_file="$(find "${prefix}" -path '*nav2_rotation_shim_controller.xml' -print -quit 2>/dev/null || true)"
  if [[ -n "${plugin_file}" ]] && grep -Fq "nav2_rotation_shim_controller::RotationShimController" "${plugin_file}"; then
    pass "runtime RotationShimController plugin XML present"
  else
    fail "runtime RotationShimController plugin XML missing"
  fi
  local runtime_rotate
  runtime_rotate="$(timeout 6 ros2 param get /controller_server FollowPath.rotate_to_goal_heading 2>&1 || true)"
  if [[ "${runtime_rotate}" == *"True"* || "${runtime_rotate}" == *"true"* ]]; then
    pass "runtime controller rotate_to_goal_heading=true"
  else
    warn "runtime controller not yet restarted with N3 config: ${runtime_rotate}"
  fi
}

check_static_nav2
check_static_api
check_rotation_shim_available

if grep -R "max_odom_tf_age_ms" "${WORKSPACE_ROOT}/src" "${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay" >/dev/null 2>&1; then
  pass "max_odom_tf_age_ms references unchanged by this script"
fi
AMCL_CONFIG="${NJRH_AMCL_CONFIG:-${NJRH_OVERLAY_ROOT}/config/amcl_shadow.yaml}"
if [[ -f "${AMCL_CONFIG}" ]] && grep -Eq "^[[:space:]]*tf_broadcast:[[:space:]]*true[[:space:]]*$" "${AMCL_CONFIG}"; then
  fail "active AMCL config has tf_broadcast=true: ${AMCL_CONFIG}"
else
  pass "active AMCL tf_broadcast=true not present"
fi
KILL_PATTERN='p[k]ill -9|kill[a]ll -9'
if grep -E "${KILL_PATTERN}" "$0" "${SCRIPT_DIR}/observe_nav2_native_pose_required_goal.sh" >/dev/null 2>&1; then
  fail "N3 verification/observation scripts contain broad kill"
else
  pass "N3 verification/observation scripts contain no broad kill"
fi

echo "${PREFIX} SUMMARY failures=${FAILURES} warnings=${WARNINGS}"
[[ "${FAILURES}" -eq 0 ]] || exit 1

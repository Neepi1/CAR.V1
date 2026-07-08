#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
REPORT_DIR="${WORKSPACE_ROOT}/reports/post_nav2_final_verify_recovery_${TIMESTAMP}"

MOCK_NAV2_SUCCEEDED=false
MOCK_FINAL_DISTANCE=""
MOCK_YAW_ERROR=""
MOCK_TOLERANCE="0.20"
EXPECT_RETRY_XY=false
EXPECT_FINAL_FAIL=false
EXPECT_TASK_COMPLETE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mock-nav2-succeeded)
      MOCK_NAV2_SUCCEEDED=true
      shift
      ;;
    --mock-final-distance)
      MOCK_FINAL_DISTANCE="${2:?}"
      shift 2
      ;;
    --mock-yaw-error)
      MOCK_YAW_ERROR="${2:?}"
      shift 2
      ;;
    --mock-tolerance)
      MOCK_TOLERANCE="${2:?}"
      shift 2
      ;;
    --expect-retry-xy)
      EXPECT_RETRY_XY=true
      shift
      ;;
    --expect-final-fail)
      EXPECT_FINAL_FAIL=true
      shift
      ;;
    --expect-task-complete)
      EXPECT_TASK_COMPLETE=true
      shift
      ;;
    --output-dir)
      REPORT_DIR="${2:?}"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage:
  verify_post_nav2_final_verify_recovery.sh [mock options]

Static/read-only verifier for the N5 single Nav2 completion owner contract. It
does not send Nav2 goals, relocalization requests, or velocity commands.

Mock examples:
  verify_post_nav2_final_verify_recovery.sh \
    --mock-nav2-succeeded --mock-final-distance 0.269 --mock-yaw-error 0.018 \
    --mock-tolerance 0.2 --expect-task-complete

  verify_post_nav2_final_verify_recovery.sh \
    --mock-nav2-succeeded --mock-final-distance 0.7 --mock-yaw-error 0.018 \
    --expect-task-complete
EOF
      exit 0
      ;;
    *)
      echo "[post-nav2-final-verify] unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

mkdir -p "${REPORT_DIR}"

API_CPP="${WORKSPACE_ROOT}/src/robot_api_server/src/robot_api_server_node.cpp"
API_CFG="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/robot_api_server.yaml"
NAV2_CFG="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/nav2.yaml"
BRIDGE_CFG="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/localization_bridge.yaml"
SUMMARY="${REPORT_DIR}/summary.md"

python3 - "$API_CPP" "$API_CFG" "$NAV2_CFG" "$BRIDGE_CFG" "$MOCK_NAV2_SUCCEEDED" \
  "$MOCK_FINAL_DISTANCE" "$MOCK_YAW_ERROR" "$MOCK_TOLERANCE" \
  "$EXPECT_RETRY_XY" "$EXPECT_FINAL_FAIL" "$EXPECT_TASK_COMPLETE" "$SUMMARY" <<'PY'
import pathlib
import sys

api_cpp = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
api_cfg = pathlib.Path(sys.argv[2]).read_text(encoding="utf-8")
nav2_cfg = pathlib.Path(sys.argv[3]).read_text(encoding="utf-8")
bridge_cfg = pathlib.Path(sys.argv[4]).read_text(encoding="utf-8")
mock_nav2_succeeded = sys.argv[5] == "true"
mock_final_distance = sys.argv[6]
mock_yaw_error = sys.argv[7]
mock_tolerance = float(sys.argv[8])
expect_retry_xy = sys.argv[9] == "true"
expect_final_fail = sys.argv[10] == "true"
expect_task_complete = sys.argv[11] == "true"
summary = pathlib.Path(sys.argv[12])

checks = []

def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))

run_goal_start = api_cpp.index("void run_navigation_goal_job(")
run_goal_end = api_cpp.index("HttpResponse handle_navigation_state()")
run_goal_block = api_cpp[run_goal_start:run_goal_end]

check("commercial final verification marker exists", "commercial_final_verify=true" in api_cpp)
check("commercial final verification success detail exists", "navigation goal reached by commercial final verification" in api_cpp)
check("run goal can retry same Nav2 goal after final audit", "run_post_nav2_final_verify_retry(job_id, target, retry_reason, retry_phase)" in run_goal_block)
check("run goal can use bounded API final yaw fallback through robot_safety", "run_final_yaw_align(job_id, target, pose_check)" in run_goal_block)
check("run goal can use bounded terminal lateral correction", "run_post_nav2_terminal_lateral_correction(" in api_cpp)
check("final verify retry can permit bounded reverse through mode controller", "publish_post_nav2_final_verify_reverse_permit(true)" in api_cpp and "/ranger_mini3/allow_reverse" in api_cpp)
check("run goal does not reposition after yaw drift", "run_reposition_after_yaw_drift(job_id, target)" not in run_goal_block)
check("run goal degrades instead of false-completing final audit overrun", '"degraded_final_pose_verify"' in run_goal_block)
check("run goal audits final pose", "final_pose_auditing" in run_goal_block)
check("failed final pose terminal flag still exists for non-success legacy states", "final_verify_failure_is_terminal" in api_cpp)
check("API final_yaw_align fallback enabled", "api_final_yaw_align_fallback_enabled: true" in api_cfg)
check("API velocity correction enabled only for bounded terminal correction", "post_nav2_final_verify_api_velocity_correction_enabled: true" in api_cfg)
check("terminal lateral correction enabled", "post_nav2_final_verify_terminal_lateral_correction_enabled: true" in api_cfg)
check("final verify retry enabled", "post_nav2_final_verify_enabled: true" in api_cfg)
check("max retry count enabled", "post_nav2_final_verify_max_retry_count: 3" in api_cfg)
check("post-Nav2 acceptance slack allows 8cm after retries", "post_nav2_final_verify_acceptance_slack_m: 0.02" in api_cfg)
check("bridge wait enabled for ordinary completion", "post_nav2_final_verify_wait_bridge_smoothing: true" in api_cfg)
check("same Nav2 goal retry enabled", "post_nav2_final_verify_retry_uses_same_nav2_goal: true" in api_cfg)
check("no cmd_vel_docking for normal final verify", "/cmd_vel_docking" not in run_goal_block)
check("Nav2 controller still MPPI", "nav2_mppi_controller::MPPIController" in nav2_cfg)
check("Nav2 planner still SmacPlanner2D", "nav2_smac_planner/SmacPlanner2D" in nav2_cfg)
check("SimpleGoalChecker rechecks XY every tick", "goal_checker:" in nav2_cfg and "stateful: false" in nav2_cfg)
check("ordinary XY goal tolerance", "xy_goal_tolerance: 0.06" in nav2_cfg)
check("tight yaw goal tolerance", "yaw_goal_tolerance: 0.05" in nav2_cfg)
check("position-only goal checker is not in default Nav2 config", "nav2_controller::PositionGoalChecker" not in nav2_cfg)
check("TF tolerance unchanged", "transform_tolerance: 0.10" in nav2_cfg)
check("bridge max odom age unchanged", "max_odom_tf_age_ms: 100.0" in bridge_cfg)
check("no broad force kill", ("pkill " + "-9") not in api_cpp)

mock_result = "not_run"
if mock_nav2_succeeded and mock_final_distance:
    distance = float(mock_final_distance)
    _ = float(mock_yaw_error or "0.0")
    if distance <= mock_tolerance:
        mock_result = "task_complete"
    elif distance <= 0.25:
        mock_result = "retry_or_degraded"
    else:
        mock_result = "degraded"
    if expect_retry_xy:
        check("mock expects retry xy", mock_result in ("retry_or_degraded",), mock_result)
    if expect_final_fail:
        check("mock expects final fail", mock_result == "degraded", mock_result)
    if expect_task_complete:
        check("mock expects task complete", mock_result == "task_complete", mock_result)

failed = [c for c in checks if not c[1]]
lines = [
    "# Post-Nav2 Final Pose Audit Verification",
    "",
    f"- mock_result: `{mock_result}`",
    f"- mock_final_distance: `{mock_final_distance}`",
    f"- mock_yaw_error: `{mock_yaw_error}`",
    "",
    "## Checks",
]
for name, ok, detail in checks:
    suffix = f" - {detail}" if detail else ""
    lines.append(f"- {'PASS' if ok else 'FAIL'}: {name}{suffix}")
lines.append("")
lines.append(f"overall: {'FAIL' if failed else 'PASS'}")
summary.write_text("\n".join(lines) + "\n", encoding="utf-8")
if failed:
    print(f"[post-nav2-final-verify] FAIL summary={summary}", file=sys.stderr)
    sys.exit(1)
print(f"[post-nav2-final-verify] PASS summary={summary}")
PY

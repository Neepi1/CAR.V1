#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/nav2.yaml}"
STATE_DIR="${NJRH_ROTATION_SHIM_AB_STATE_DIR:-${NJRH_PROJECT_ROOT}/.runtime/nav2_rotation_shim_ab}"
BACKUP_FILE="${STATE_DIR}/nav2.yaml.backup"
PROFILE="baseline"
DO_APPLY=false
DO_PRINT=false
DO_RESTART=false
DO_RESTORE=false
PREFIX="[rotation-shim-ab]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/run_nav2_rotation_shim_ab.sh --profile pose_progress_only --print
  bash scripts/jetson/runtime_overlay/scripts/run_nav2_rotation_shim_ab.sh --profile pose_progress_only --apply --restart
  bash scripts/jetson/runtime_overlay/scripts/run_nav2_rotation_shim_ab.sh --restore --restart

Profiles:
  baseline                    Current production file, print only by default.
  pose_progress_only          PoseProgressChecker, rotate_to_goal_heading=false, threshold=1.20.
  relaxed_shim_1p8            Same, but angular_dist_threshold=1.80.
  relaxed_shim_2p2            Same, but angular_dist_threshold=2.20.
  no_start_shim_diagnostic    Same, but angular_dist_threshold=3.20. Diagnostic only.

This script only changes the runtime Nav2 YAML when --apply is passed. It only
restarts the runtime when --restart is also passed. It does not change
pointcloud, DDS/RMW, EKF, FAST-LIO2, App API, or the speed chain.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --print)
      DO_PRINT=true
      shift
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
      echo "${PREFIX} FAIL unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

profile_values() {
  local profile="$1"
  case "${profile}" in
    baseline)
      echo "baseline baseline baseline"
      ;;
    pose_progress_only)
      echo "nav2_controller::PoseProgressChecker false 1.20"
      ;;
    relaxed_shim_1p8)
      echo "nav2_controller::PoseProgressChecker false 1.80"
      ;;
    relaxed_shim_2p2)
      echo "nav2_controller::PoseProgressChecker false 2.20"
      ;;
    no_start_shim_diagnostic)
      echo "nav2_controller::PoseProgressChecker false 3.20"
      ;;
    *)
      return 1
      ;;
  esac
}

if [[ "${DO_RESTORE}" != "true" ]]; then
  if ! values="$(profile_values "${PROFILE}")"; then
    echo "${PREFIX} FAIL unsupported profile: ${PROFILE}" >&2
    usage >&2
    exit 2
  fi
  read -r TARGET_PLUGIN TARGET_ROTATE TARGET_THRESHOLD <<<"${values}"
fi

print_plan() {
  echo "${PREFIX} nav2_yaml=${NAV2_PARAMS_FILE}"
  if [[ "${DO_RESTORE}" == "true" ]]; then
    echo "${PREFIX} plan=restore backup=${BACKUP_FILE}"
    return
  fi
  echo "${PREFIX} profile=${PROFILE}"
  if [[ "${PROFILE}" == "baseline" ]]; then
    echo "${PREFIX} baseline is observation-only; --apply is ignored for baseline"
  else
    echo "${PREFIX} set progress_checker.plugin=${TARGET_PLUGIN}"
    echo "${PREFIX} set required_movement_radius=0.10"
    echo "${PREFIX} set required_movement_angle=0.10"
    echo "${PREFIX} keep movement_time_allowance=12.0"
    echo "${PREFIX} set FollowPath.rotate_to_goal_heading=${TARGET_ROTATE}"
    echo "${PREFIX} set FollowPath.angular_dist_threshold=${TARGET_THRESHOLD}"
  fi
  echo "${PREFIX} after each profile check:"
  echo "${PREFIX}   ros2 lifecycle get /controller_server"
  echo "${PREFIX}   diagnose max/mean/nonzero linear.x for /cmd_vel_nav_raw, /cmd_vel_nav, /cmd_vel_collision_checked, /cmd_vel_safe, /cmd_vel"
  echo "${PREFIX}   diagnose wheel/local_state odom movement and progress failure"
}

apply_profile() {
  if [[ "${PROFILE}" == "baseline" ]]; then
    echo "${PREFIX} baseline selected; no file changes"
    return 0
  fi
  if [[ ! -f "${NAV2_PARAMS_FILE}" ]]; then
    echo "${PREFIX} FAIL missing nav2 yaml: ${NAV2_PARAMS_FILE}" >&2
    return 1
  fi
  mkdir -p "${STATE_DIR}"
  if [[ ! -f "${BACKUP_FILE}" ]]; then
    cp "${NAV2_PARAMS_FILE}" "${BACKUP_FILE}"
    echo "${PREFIX} backup=${BACKUP_FILE}"
  fi
  python3 - "${NAV2_PARAMS_FILE}" "${TARGET_PLUGIN}" "${TARGET_ROTATE}" "${TARGET_THRESHOLD}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
target_plugin = sys.argv[2]
target_rotate = sys.argv[3]
target_threshold = sys.argv[4]
lines = path.read_text(encoding="utf-8").splitlines()
out = []
in_progress = False
in_follow = False
progress_indent = None
follow_indent = None
angle_seen = False

def indent_of(line: str) -> int:
    return len(line) - len(line.lstrip(" "))

for line in lines:
    stripped = line.strip()
    indent = indent_of(line)
    if stripped == "progress_checker:":
        in_progress = True
        progress_indent = indent
        angle_seen = False
        out.append(line)
        continue
    if in_progress and stripped and indent <= progress_indent:
        if not angle_seen:
            out.append("      required_movement_angle: 0.10")
        in_progress = False
    if in_progress:
        if stripped.startswith("plugin:"):
            out.append("      plugin: \"" + target_plugin + "\"")
            continue
        if stripped.startswith("required_movement_radius:"):
            out.append("      required_movement_radius: 0.10")
            continue
        if stripped.startswith("required_movement_angle:"):
            out.append("      required_movement_angle: 0.10")
            angle_seen = True
            continue
        if stripped.startswith("movement_time_allowance:"):
            if not angle_seen:
                out.append("      required_movement_angle: 0.10")
                angle_seen = True
            out.append("      movement_time_allowance: 12.0")
            continue

    if stripped == "FollowPath:":
        in_follow = True
        follow_indent = indent
        out.append(line)
        continue
    if in_follow and stripped and indent <= follow_indent:
        in_follow = False
    if in_follow:
        if stripped.startswith("angular_dist_threshold:"):
            out.append("      angular_dist_threshold: " + target_threshold)
            continue
        if stripped.startswith("rotate_to_goal_heading:"):
            out.append("      rotate_to_goal_heading: " + target_rotate)
            continue

    out.append(line)

if in_progress and not angle_seen:
    out.append("      required_movement_angle: 0.10")

path.write_text("\n".join(out) + "\n", encoding="utf-8")
PY
}

restore_profile() {
  if [[ ! -f "${BACKUP_FILE}" ]]; then
    echo "${PREFIX} FAIL no backup to restore: ${BACKUP_FILE}" >&2
    return 1
  fi
  cp "${BACKUP_FILE}" "${NAV2_PARAMS_FILE}"
  echo "${PREFIX} restored ${NAV2_PARAMS_FILE} from ${BACKUP_FILE}"
}

restart_runtime() {
  if [[ -n "${NJRH_NAV2_RESTART_CMD:-}" ]]; then
    echo "${PREFIX} restart command: ${NJRH_NAV2_RESTART_CMD}"
    bash -lc "${NJRH_NAV2_RESTART_CMD}"
    return $?
  fi

  if [[ -z "${NJRH_BUILDING_ID:-}" || -z "${NJRH_FLOOR_ID:-}" ]]; then
    echo "${PREFIX} FAIL --restart requires NJRH_NAV2_RESTART_CMD or NJRH_BUILDING_ID/NJRH_FLOOR_ID" >&2
    return 2
  fi

  bash "${SCRIPT_DIR}/stop_floor_navigation.sh"
  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  nohup bash "${SCRIPT_DIR}/run_floor_navigation.sh" "${NJRH_BUILDING_ID}" "${NJRH_FLOOR_ID}" \
    >"${NJRH_RUNTIME_LOG_DIR}/nav2_rotation_shim_ab_restart.log" 2>&1 &
  echo "${PREFIX} restarted resident navigation runtime pid=$! log=${NJRH_RUNTIME_LOG_DIR}/nav2_rotation_shim_ab_restart.log"
}

print_plan

if [[ "${DO_RESTORE}" == "true" ]]; then
  restore_profile || exit 1
elif [[ "${DO_APPLY}" == "true" ]]; then
  apply_profile || exit 1
elif [[ "${DO_PRINT}" != "true" ]]; then
  echo "${PREFIX} no --apply requested; printed plan only"
fi

if [[ "${DO_RESTART}" == "true" ]]; then
  restart_runtime || exit $?
fi

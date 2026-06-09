#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

PROFILE="current"
APPLY=false
RESTORE=false
RESTART=false
PRINT_ONLY=false

usage() {
  cat <<'EOF'
Usage: run_pointcloud_cpu_affinity_ab.sh [--print] [--profile current|split_local_nav|local_priority] [--apply] [--restart] [--restore]

Default mode prints the selected profile and validation commands only.
--apply writes a reversible override block into config/cpu_affinity.env.
--restart reapplies affinity to matching live PIDs through cpu_affinity.sh helpers; it does not kill processes.
--restore restores the backup created by --apply.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --apply)
      APPLY=true
      shift
      ;;
    --restart)
      RESTART=true
      shift
      ;;
    --restore)
      RESTORE=true
      shift
      ;;
    --print)
      PRINT_ONLY=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[cpu-affinity-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

CONFIG_FILE="${NJRH_CPU_AFFINITY_CONFIG:-${NJRH_OVERLAY_ROOT}/config/cpu_affinity.env}"
BACKUP_FILE="${CONFIG_FILE}.phase111.bak"
MARK_BEGIN="# BEGIN Phase 1.11 pointcloud CPU affinity A/B"
MARK_END="# END Phase 1.11 pointcloud CPU affinity A/B"

profile_overrides() {
  local profile="$1"
  case "${profile}" in
    current)
      return 0
      ;;
    split_local_nav)
      cat <<'EOF'
export NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION="6"
export NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR="7"
export NJRH_CPUSET_POINTCLOUD_TO_LASERSCAN="7"
export NJRH_CPUSET_SCAN_REPUBLISHER="7"
export NJRH_CPUSET_LASER_SCAN_TO_FLATSCAN="7"
export NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER="7"
export NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION="7"
EOF
      ;;
    local_priority)
      cat <<'EOF'
export NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION="6"
export NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR="${NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR:-7}"
export NJRH_CPUSET_POINTCLOUD_TO_LASERSCAN="${NJRH_CPUSET_POINTCLOUD_TO_LASERSCAN:-7}"
EOF
      ;;
    *)
      echo "[cpu-affinity-ab] unsupported profile: ${profile}" >&2
      exit 2
      ;;
  esac
}

print_plan() {
  echo "[cpu-affinity-ab] selected_profile=${PROFILE}"
  echo "[cpu-affinity-ab] config_file=${CONFIG_FILE}"
  echo "[cpu-affinity-ab] default mode does not modify files or restart services"
  echo "[cpu-affinity-ab] current pointcloud-related CPU sets:"
  env | awk -F= '/^NJRH_CPUSET_(POINTCLOUD_AXIS_REMAP|ROBOT_LOCAL_PERCEPTION|NAV_CLOUD_PREPROCESSOR|POINTCLOUD_TO_LASERSCAN|SCAN_REPUBLISHER|LASER_SCAN_TO_FLATSCAN|OCCUPANCY_GRID_LOCALIZER|ROBOT_GLOBAL_LOCALIZATION)=/ {print "[cpu-affinity-ab]   " $1 "=" $2}' | sort
  if [[ "${PROFILE}" == "current" ]]; then
    echo "[cpu-affinity-ab] current profile keeps existing config unchanged"
  else
    echo "[cpu-affinity-ab] proposed override block:"
    profile_overrides "${PROFILE}" | sed 's/^/[cpu-affinity-ab]   /'
  fi
  cat <<'EOF'
[cpu-affinity-ab] validation commands:
[cpu-affinity-ab]   bash scripts/jetson/runtime_overlay/scripts/diagnose_local_perception_pipeline.sh
[cpu-affinity-ab]   bash scripts/jetson/runtime_overlay/scripts/diagnose_nav_scan_pipeline.sh
[cpu-affinity-ab]   bash scripts/jetson/runtime_overlay/scripts/diagnose_pointcloud_cpu_pressure.sh
[cpu-affinity-ab]   bash scripts/jetson/runtime_overlay/scripts/verify_pointcloud_delivery_matrix.sh
EOF
}

remove_existing_block() {
  local file="$1"
  local tmp_file="${file}.tmp.$$"
  awk -v begin="${MARK_BEGIN}" -v end="${MARK_END}" '
    $0 == begin {skip = 1; next}
    $0 == end {skip = 0; next}
    !skip {print}
  ' "${file}" >"${tmp_file}"
  mv "${tmp_file}" "${file}"
}

apply_profile() {
  [[ -f "${CONFIG_FILE}" ]] || {
    echo "[cpu-affinity-ab] missing config file: ${CONFIG_FILE}" >&2
    exit 1
  }
  if [[ ! -f "${BACKUP_FILE}" ]]; then
    cp -p "${CONFIG_FILE}" "${BACKUP_FILE}"
    echo "[cpu-affinity-ab] backup created: ${BACKUP_FILE}"
  else
    echo "[cpu-affinity-ab] backup already exists: ${BACKUP_FILE}"
  fi
  remove_existing_block "${CONFIG_FILE}"
  if [[ "${PROFILE}" != "current" ]]; then
    {
      echo "${MARK_BEGIN}"
      echo "# profile=${PROFILE}"
      profile_overrides "${PROFILE}"
      echo "${MARK_END}"
    } >>"${CONFIG_FILE}"
    echo "[cpu-affinity-ab] applied profile=${PROFILE} to ${CONFIG_FILE}"
  else
    echo "[cpu-affinity-ab] current profile selected; override block removed"
  fi
}

restore_profile() {
  [[ -f "${BACKUP_FILE}" ]] || {
    echo "[cpu-affinity-ab] no backup to restore: ${BACKUP_FILE}" >&2
    exit 1
  }
  cp -p "${BACKUP_FILE}" "${CONFIG_FILE}"
  echo "[cpu-affinity-ab] restored ${CONFIG_FILE} from ${BACKUP_FILE}"
}

pid_list_for_pattern() {
  local pattern="$1"
  pgrep -f "${pattern}" 2>/dev/null | sort -n
}

apply_live_affinity() {
  # Re-source the possibly modified config before applying live task affinity.
  # shellcheck source=../config/cpu_affinity.env
  source "${CONFIG_FILE}"

  local pids
  pids="$(pid_list_for_pattern "local_perception_node|robot_local_perception" | paste -sd ' ' -)"
  [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids robot_local_perception ${pids}

  pids="$(pid_list_for_pattern "nav_cloud_preprocessor" | paste -sd ' ' -)"
  [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids nav_cloud_preprocessor ${pids}

  pids="$(pid_list_for_pattern "pointcloud_to_laserscan" | paste -sd ' ' -)"
  [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids pointcloud_to_laserscan ${pids}

  pids="$(pid_list_for_pattern "scan_republisher_node" | paste -sd ' ' -)"
  [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids scan_republisher ${pids}

  pids="$(pid_list_for_pattern "laser_scan_to_flatscan" | paste -sd ' ' -)"
  [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids laser_scan_to_flatscan ${pids}

  pids="$(pid_list_for_pattern "occupancy_grid_localizer|isaac.*localizer" | paste -sd ' ' -)"
  [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids occupancy_grid_localizer ${pids}

  pids="$(pid_list_for_pattern "robot_global_localization" | paste -sd ' ' -)"
  [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids robot_global_localization ${pids}

  echo "[cpu-affinity-ab] live affinity reapplied to matching running PIDs; no process was killed"
}

print_plan

if [[ "${RESTORE}" == "true" ]]; then
  restore_profile
elif [[ "${APPLY}" == "true" ]]; then
  apply_profile
elif [[ "${PRINT_ONLY}" == "false" ]]; then
  echo "[cpu-affinity-ab] no --apply or --restore requested; printed plan only"
fi

if [[ "${RESTART}" == "true" ]]; then
  apply_live_affinity
fi

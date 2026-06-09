#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

PROFILE="baseline"
APPLY=false
RESTORE=false
RESTART=false
PRINT=false
RUN_DIAGNOSTICS=true
OVERRIDE_FILE="${NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE:-${NJRH_OVERLAY_ROOT}/config/cpu_affinity_runtime_override.env}"
BACKUP_FILE="${OVERRIDE_FILE}.bak"

usage() {
  cat <<'EOF'
Usage: run_cpu_core_allocation_ab.sh [--profile baseline|split_local_nav_v1|split_local_nav_v2|local_priority|scan_priority] [--print] [--apply] [--restart] [--restore] [--no-diagnostics]

Default mode is a dry-run plan. --apply writes config/cpu_affinity_runtime_override.env.
--restart reapplies affinity to matching live PIDs with taskset -pc; it does not kill processes.
--restore restores/removes the runtime override file from the backup.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --print)
      PRINT=true
      shift
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
    --no-diagnostics)
      RUN_DIAGNOSTICS=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[cpu-core-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

profile_overrides() {
  local profile="$1"
  case "${profile}" in
    baseline)
      return 0
      ;;
    split_local_nav_v1)
      cat <<'EOF'
export NJRH_CPUSET_HESAI_ROS_DRIVER="4"
export NJRH_CPUSET_POINTCLOUD_AXIS_REMAP="5"
export NJRH_CPUSET_IMU_AXIS_REMAP="2"
export NJRH_CPUSET_ROBOT_LOCAL_STATE="2"
export NJRH_CPUSET_ROBOT_LOCAL_STATE_ODOM_PREPROCESSOR="2"
export NJRH_CPUSET_ROBOT_LOCAL_STATE_IMU_BIAS_FILTER="2"
export NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION="6"
export NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR="7"
export NJRH_CPUSET_POINTCLOUD_TO_LASERSCAN="7"
export NJRH_CPUSET_SCAN_REPUBLISHER="7"
export NJRH_CPUSET_LASER_SCAN_TO_FLATSCAN="7"
export NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER="7"
export NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION="7"
export NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE="7"
export NJRH_CPUSET_CONTROLLER_SERVER="3"
export NJRH_CPUSET_RANGER_BASE_NODE="1"
export NJRH_CPUSET_ROBOT_SAFETY="1"
export NJRH_CPUSET_COLLISION_MONITOR="1"
export NJRH_CPUSET_VELOCITY_SMOOTHER="1"
export NJRH_CPUSET_RANGER_MINI3_MODE_CONTROLLER="1"
export NJRH_CPUSET_BT_NAVIGATOR="0"
export NJRH_CPUSET_PLANNER_SERVER="0"
export NJRH_CPUSET_BEHAVIOR_SERVER="0"
export NJRH_CPUSET_SMOOTHER_SERVER="0"
export NJRH_CPUSET_NAV2_MAP_SERVER="0"
EOF
      ;;
    split_local_nav_v2)
      profile_overrides split_local_nav_v1
      cat <<'EOF'
export NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION="7"
export NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR="6"
export NJRH_CPUSET_POINTCLOUD_TO_LASERSCAN="6"
export NJRH_CPUSET_SCAN_REPUBLISHER="6"
export NJRH_CPUSET_LASER_SCAN_TO_FLATSCAN="6"
export NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER="6"
export NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION="6"
EOF
      ;;
    local_priority)
      profile_overrides split_local_nav_v1
      cat <<'EOF'
export NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION="6"
export NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR="7"
export NJRH_CPUSET_POINTCLOUD_TO_LASERSCAN="7"
export NJRH_CPUSET_SCAN_REPUBLISHER="7"
export NJRH_CPUSET_LASER_SCAN_TO_FLATSCAN="7"
export NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER="7"
export NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION="7"
export NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE="7"
EOF
      ;;
    scan_priority)
      profile_overrides split_local_nav_v1
      cat <<'EOF'
export NJRH_CPUSET_NAV_CLOUD_PREPROCESSOR="6"
export NJRH_CPUSET_POINTCLOUD_TO_LASERSCAN="6"
export NJRH_CPUSET_SCAN_REPUBLISHER="6"
export NJRH_CPUSET_LASER_SCAN_TO_FLATSCAN="6"
export NJRH_CPUSET_ROBOT_LOCAL_PERCEPTION="7"
export NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER="7"
export NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION="7"
EOF
      ;;
    *)
      echo "[cpu-core-ab] unsupported profile: ${profile}" >&2
      exit 2
      ;;
  esac
}

resolved_profile_overrides() {
  profile_overrides "$1" |
    awk -F= '
      /^export [A-Za-z_][A-Za-z0-9_]*=/ {
        key=$1
        if (!(key in seen)) {
          order[++n]=key
          seen[key]=1
        }
        line[key]=$0
        next
      }
      {print}
      END {
        for (i=1; i<=n; i++) {
          print line[order[i]]
        }
      }
    '
}

profile_expectations() {
  case "$1" in
    baseline)
      echo "obstacle_points: no change expected; records current baseline"
      echo "points_nav/scan: no change expected"
      echo "map->odom age: no additional risk"
      echo "migrated nodes: none"
      ;;
    split_local_nav_v1)
      echo "obstacle_points: expected improvement by keeping local_perception on CPU6"
      echo "points_nav/scan: expected improvement by moving nav scan/localizer chain to CPU7"
      echo "map->odom age: possible mild risk because localization_bridge/localizer share CPU7"
      echo "migrated nodes: nav_cloud_preprocessor, pointcloud_to_laserscan, scan_republisher, laser_scan_to_flatscan, occupancy_grid_localizer, global_localization, localization_bridge"
      ;;
    split_local_nav_v2)
      echo "obstacle_points: expected improvement by moving local_perception to CPU7"
      echo "points_nav/scan: keeps scan chain on CPU6; useful if CPU7 localizer causes map->odom jitter"
      echo "map->odom age: lower risk than v1 for bridge if CPU7 only hosts local perception plus bridge defaults"
      echo "migrated nodes: robot_local_perception to CPU7; nav scan/localizer to CPU6"
      ;;
    local_priority)
      echo "obstacle_points: highest expected priority"
      echo "points_nav/scan: may improve if CPU7 has enough headroom; monitor /scan"
      echo "map->odom age: monitor because localizer remains on CPU7"
      echo "migrated nodes: nav scan/localizer away from CPU6, local_perception isolated on CPU6"
      ;;
    scan_priority)
      echo "obstacle_points: may improve by moving local_perception to CPU7, but localizer contention must be watched"
      echo "points_nav/scan: highest expected priority"
      echo "map->odom age: possible risk if CPU7 local perception affects bridge/localizer"
      echo "migrated nodes: scan chain on CPU6, local_perception on CPU7"
      ;;
  esac
}

print_plan() {
  echo "[cpu-core-ab] profile=${PROFILE}"
  echo "[cpu-core-ab] override_file=${OVERRIDE_FILE}"
  echo "[cpu-core-ab] default_mode=dry-run"
  echo "[cpu-core-ab] will_not_change=QoS,DDS,timestamps,Nav2_planner_controller,EKF,FAST-LIO2"
  echo "[cpu-core-ab] expectations:"
  profile_expectations "${PROFILE}" | sed 's/^/[cpu-core-ab]   /'
  if [[ "${PROFILE}" != "baseline" ]]; then
    echo "[cpu-core-ab] proposed overrides:"
    resolved_profile_overrides "${PROFILE}" | sed 's/^/[cpu-core-ab]   /'
  fi
}

run_diagnostics() {
  local label="$1"
  [[ "${RUN_DIAGNOSTICS}" == "true" ]] || return 0
  echo "[cpu-core-ab] diagnostics=${label}"
  bash "${SCRIPT_DIR}/collect_cpu_irq_softirq_snapshot.sh" --duration-sec 20 || true
  bash "${SCRIPT_DIR}/diagnose_local_perception_pipeline.sh" || true
  bash "${SCRIPT_DIR}/diagnose_nav_scan_pipeline.sh" || true
  bash "${SCRIPT_DIR}/verify_pointcloud_delivery_matrix.sh" || true
}

write_override() {
  mkdir -p "$(dirname "${OVERRIDE_FILE}")"
  if [[ ! -e "${BACKUP_FILE}" ]]; then
    if [[ -e "${OVERRIDE_FILE}" ]]; then
      cp -p "${OVERRIDE_FILE}" "${BACKUP_FILE}"
    else
      : >"${BACKUP_FILE}.empty"
    fi
  fi
  {
    echo "# shellcheck shell=bash"
    echo "# Runtime CPU affinity override generated by run_cpu_core_allocation_ab.sh"
    echo "# profile=${PROFILE}"
    echo "# restore with: bash scripts/jetson/runtime_overlay/scripts/run_cpu_core_allocation_ab.sh --restore --restart"
    if [[ "${PROFILE}" != "baseline" ]]; then
      resolved_profile_overrides "${PROFILE}"
    fi
  } >"${OVERRIDE_FILE}"
  echo "[cpu-core-ab] wrote ${OVERRIDE_FILE}"
}

restore_override() {
  if [[ -e "${BACKUP_FILE}" ]]; then
    cp -p "${BACKUP_FILE}" "${OVERRIDE_FILE}"
    echo "[cpu-core-ab] restored ${OVERRIDE_FILE} from ${BACKUP_FILE}"
  elif [[ -e "${BACKUP_FILE}.empty" ]]; then
    rm -f "${OVERRIDE_FILE}" "${BACKUP_FILE}.empty"
    echo "[cpu-core-ab] removed ${OVERRIDE_FILE}; previous state was absent"
  else
    rm -f "${OVERRIDE_FILE}"
    echo "[cpu-core-ab] no backup found; removed ${OVERRIDE_FILE}"
  fi
}

pid_list_for_pattern() {
  local matches
  matches="$(pgrep -f "$1" 2>/dev/null || true)"
  [[ -n "${matches}" ]] || return 0
  printf '%s\n' "${matches}" | sort -n | paste -sd ' ' -
}

apply_live() {
  # shellcheck source=../config/cpu_affinity_runtime_override.env
  [[ -f "${OVERRIDE_FILE}" ]] && source "${OVERRIDE_FILE}"

  local pids
  pids="$(pid_list_for_pattern "hesai_ros_driver_node")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids hesai_ros_driver ${pids}
  pids="$(pid_list_for_pattern "pointcloud_axis_remap")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids pointcloud_axis_remap ${pids}
  pids="$(pid_list_for_pattern "imu_axis_remap")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids imu_axis_remap ${pids}
  pids="$(pid_list_for_pattern "local_perception_node|robot_local_perception")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids robot_local_perception ${pids}
  pids="$(pid_list_for_pattern "nav_cloud_preprocessor")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids nav_cloud_preprocessor ${pids}
  pids="$(pid_list_for_pattern "pointcloud_to_laserscan")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids pointcloud_to_laserscan ${pids}
  pids="$(pid_list_for_pattern "scan_republisher_node")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids scan_republisher ${pids}
  pids="$(pid_list_for_pattern "laser_scan_to_flatscan")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids laser_scan_to_flatscan ${pids}
  pids="$(pid_list_for_pattern "occupancy_grid_localizer|isaac.*localizer|occupancy_grid_localizer_container")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids occupancy_grid_localizer ${pids}
  pids="$(pid_list_for_pattern "robot_global_localization|global_localization_node")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids robot_global_localization ${pids}
  pids="$(pid_list_for_pattern "localization_bridge_node|robot_localization_bridge")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids robot_localization_bridge ${pids}
  pids="$(pid_list_for_pattern "ekf_node --ros-args.*__node:=robot_local_state|local_state_node|imu_gyro_bias_filter_node")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids robot_local_state ${pids}
  pids="$(pid_list_for_pattern "controller_server")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids controller_server ${pids}
  pids="$(pid_list_for_pattern "collision_monitor")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids collision_monitor ${pids}
  pids="$(pid_list_for_pattern "velocity_smoother")"; [[ -n "${pids}" ]] && njrh_apply_affinity_to_pids velocity_smoother ${pids}
  echo "[cpu-core-ab] live affinity reapplied; no process was killed"
}

print_plan

if [[ "${RESTORE}" == "true" ]]; then
  run_diagnostics "pre-restore"
  restore_override
elif [[ "${APPLY}" == "true" ]]; then
  run_diagnostics "pre-apply"
  write_override
elif [[ "${PRINT}" == "false" ]]; then
  echo "[cpu-core-ab] no --apply or --restore requested; printed plan only"
fi

if [[ "${RESTART}" == "true" ]]; then
  apply_live
fi

if [[ "${APPLY}" == "true" || "${RESTORE}" == "true" ]]; then
  run_diagnostics "post-change"
fi

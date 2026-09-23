#!/usr/bin/env bash
# Five-core robot placement. No taskset, ROS or process side effects.
# Mapping shares the five-core limit, but must not join navigation startup boost.
NJRH_NAVIGATION_CPU_KEYS=(
  SYSTEM BASE_CONTROL TF_STATE NAV_CONTROL NAV_PLANNING NAV_SUPERVISION
  LIDAR_PERCEPTION LIDAR_DRIVER LIDAR_STARTUP LIDAR_PIPELINE LOCALIZATION
  NAVIGATION_RUNTIME_OWNER NAV2_CONTROLLER_CURRENT NAV2_CONTROLLER_WIDE
  ROBOT_API_SERVER RUNTIME_HEALTH_GUARD RANGER_BASE_NODE ROBOT_SAFETY
  VELOCITY_SMOOTHER RANGER_MINI3_MODE_CONTROLLER DOCKING_MANAGER
  DOCKING_CAMERA DOCKING_VISION
  ROBOT_LOCAL_STATE ROBOT_LOCAL_STATE_ODOM_PREPROCESSOR
  ROBOT_LOCAL_STATE_IMU_BIAS_FILTER ROBOT_LOCALIZATION_BRIDGE
  CONTROLLER_SERVER COLLISION_MONITOR LOCAL_COSTMAP
  HESAI_ROS_DRIVER POINTCLOUD_AXIS_REMAP POINTCLOUD_ACCEL_CONTAINER
  POINTCLOUD_ACCEL_LOCAL_WORKER POINTCLOUD_ACCEL_SCAN_WORKER
  NITROS_POINTCLOUD_CONTAINER POINTCLOUD_PERCEPTION_PIPELINE
  POINTCLOUD_DOWNSAMPLE IMU_AXIS_REMAP NAV_CLOUD_PREPROCESSOR ROBOT_LOCAL_PERCEPTION
  OCCUPANCY_GRID_LOCALIZER ROBOT_GLOBAL_LOCALIZATION LASER_SCAN_TO_FLATSCAN
  POINTCLOUD_TO_LASERSCAN SCAN_REPUBLISHER AMCL AMCL_SCAN_ADMISSION
  PLANNER_SERVER BT_NAVIGATOR BEHAVIOR_SERVER SMOOTHER_SERVER WAYPOINT_FOLLOWER
  NAV2_MAP_SERVER NAV2_LIFECYCLE_MANAGER
)

NJRH_MAPPING_CPU_KEYS=(
  MAPPING_FRONTEND MAPPING_BACKEND FASTLIO_MAPPING FASTLIO_DESKEW
  FASTLIO_ODOM_BRIDGE SLAM_TOOLBOX_MAPPING PGO_MAPPING POINTCLOUD_FASTLIO_REMAP
  MAPPING_LIDAR_RPS_XPS
)

# Placement only: the official driver uses CPU3; downstream pointcloud/scan
# work may use CPU1,4. Bootstrap uses CPU3 before worker-specific placement.
# The controller/local costmap shares CPU1-3; other navigation compute/system
# work stays on CPU0,1,4. FAST-LIO shares CPU1-4 during mapping. Kernel/IRQ work
# is not isolated, and controller eligibility does not reserve CPU2/3.
# A multi-CPU mask expresses eligibility, not a primary/overflow preference.
njrh_navigation_five_cpuset() {
  case "$1" in
    FASTLIO_MAPPING|FASTLIO_DESKEW)
      printf '1-4\n' ;;
    MAPPING_FRONTEND|MAPPING_BACKEND|SLAM_TOOLBOX_MAPPING|PGO_MAPPING)
      printf '0-1,4\n' ;;
    FASTLIO_ODOM_BRIDGE)
      printf '2\n' ;;
    POINTCLOUD_FASTLIO_REMAP)
      printf '1,4\n' ;;
    MAPPING_LIDAR_RPS_XPS)
      printf '4\n' ;;
    SYSTEM|NAV_SUPERVISION|ROBOT_API_SERVER|RUNTIME_HEALTH_GUARD|DOCKING_CAMERA|NAV2_MAP_SERVER|NAV2_LIFECYCLE_MANAGER)
      printf '0-1,4\n' ;;
    BASE_CONTROL|RANGER_BASE_NODE|ROBOT_SAFETY|VELOCITY_SMOOTHER|RANGER_MINI3_MODE_CONTROLLER|DOCKING_MANAGER|COLLISION_MONITOR)
      printf '1\n' ;;
    TF_STATE|ROBOT_LOCAL_STATE|ROBOT_LOCAL_STATE_IMU_BIAS_FILTER|ROBOT_LOCALIZATION_BRIDGE|IMU_AXIS_REMAP|ROBOT_LOCAL_STATE_ODOM_PREPROCESSOR)
      printf '2\n' ;;
    LIDAR_DRIVER|HESAI_ROS_DRIVER)
      printf '3\n' ;;
    LIDAR_STARTUP)
      printf '3\n' ;;
    NAVIGATION_RUNTIME_OWNER)
      printf '0-1,4\n' ;;
    LIDAR_PERCEPTION|LIDAR_PIPELINE|POINTCLOUD_AXIS_REMAP|POINTCLOUD_ACCEL_CONTAINER|POINTCLOUD_ACCEL_LOCAL_WORKER|POINTCLOUD_ACCEL_SCAN_WORKER|NITROS_POINTCLOUD_CONTAINER|POINTCLOUD_PERCEPTION_PIPELINE|POINTCLOUD_DOWNSAMPLE|NAV_CLOUD_PREPROCESSOR|ROBOT_LOCAL_PERCEPTION|LASER_SCAN_TO_FLATSCAN|POINTCLOUD_TO_LASERSCAN|SCAN_REPUBLISHER)
      printf '1,4\n' ;;
    NAV2_CONTROLLER_CURRENT|NAV2_CONTROLLER_WIDE|CONTROLLER_SERVER|LOCAL_COSTMAP)
      printf '1-3\n' ;;
    NAV_CONTROL|NAV_PLANNING|LOCALIZATION|DOCKING_VISION|OCCUPANCY_GRID_LOCALIZER|ROBOT_GLOBAL_LOCALIZATION|AMCL|AMCL_SCAN_ADMISSION|PLANNER_SERVER|BT_NAVIGATOR|BEHAVIOR_SERVER|SMOOTHER_SERVER|WAYPOINT_FOLLOWER)
      printf '0-1,4\n' ;;
    *) echo "[runtime-overlay] missing five-core placement for $1" >&2; return 2 ;;
  esac
}

njrh_restore_navigation_cpu_baseline() {
  local key var saved value
  for key in "${NJRH_NAVIGATION_CPU_KEYS[@]}" "${NJRH_MAPPING_CPU_KEYS[@]}"; do
    var="NJRH_CPUSET_${key}"
    saved="NJRH_NAV_CPU_BASELINE_${key}"
    if [[ -v "${saved}" ]]; then
      value="${!saved}"
      if [[ "${value}" == set:* ]]; then
        export "${var}=${value#set:}"
      else
        unset "${var}"
      fi
      unset "${saved}"
    fi
  done
}

njrh_apply_navigation_cpu_profile() {
  local key var saved cpuset
  export NJRH_NAVIGATION_CPU_PROFILE="${NJRH_NAVIGATION_CPU_PROFILE:-site_default}"
  case "${NJRH_NAVIGATION_CPU_PROFILE:-site_default}" in
    site_default) return 0 ;;
    navigation_5cpu)
      for key in "${NJRH_NAVIGATION_CPU_KEYS[@]}" "${NJRH_MAPPING_CPU_KEYS[@]}"; do
        cpuset="$(njrh_navigation_five_cpuset "${key}")" || return $?
        var="NJRH_CPUSET_${key}"
        saved="NJRH_NAV_CPU_BASELINE_${key}"
        # Export the pre-profile values so API-created shells can restore them
        # BEFORE resolving defaults, derived mapping values and site overrides.
        if [[ -v "${var}" ]]; then
          export "${saved}=set:${!var}"
        else
          export "${saved}=unset"
        fi
        export "${var}=${cpuset}"
      done
      ;;
    *)
      echo "[runtime-overlay] unsupported NJRH_NAVIGATION_CPU_PROFILE=${NJRH_NAVIGATION_CPU_PROFILE}; expected site_default|navigation_5cpu" >&2
      return 2
      ;;
  esac
}

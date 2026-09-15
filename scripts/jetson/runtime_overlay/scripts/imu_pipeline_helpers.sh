#!/usr/bin/env bash
# IMU-only ownership/compatibility helpers. Sourcing this file has no process,
# affinity or ROS side effects; common_env.sh and cpu_affinity.sh own defaults.

NJRH_IMU_PIPELINE_PROCESS_PATTERN='[/]imu_pipeline_node([[:space:]]|$)'
NJRH_IMU_STANDALONE_REMAP_PATTERN='[/]imu_axis_remap_node([[:space:]]|$)'
NJRH_IMU_STANDALONE_FILTER_PATTERN='[/]imu_gyro_bias_filter_node([[:space:]]|$)'

njrh_resolve_imu_pipeline_mode() {
  local local_state_mode="${1:-${LOCAL_STATE_MODE:-${NJRH_NAV_LOCAL_STATE_MODE:-ekf}}}"
  local requested="${NJRH_IMU_PIPELINE_MODE:-intra_process}"
  local remap_cpus filter_cpus reason
  NJRH_IMU_PIPELINE_EFFECTIVE_MODE=standalone
  case "${requested}" in
    standalone)
      reason=explicit_standalone
      ;;
    intra_process)
      remap_cpus="$(njrh_cpuset_for imu_axis_remap)"
      filter_cpus="$(njrh_cpuset_for robot_local_state_imu_bias_filter)"
      if [[ "${local_state_mode}" != ekf ]]; then
        reason="local_state_mode_${local_state_mode}"
      elif [[ "${LOCAL_STATE_IMU_BIAS_FILTER_ENABLED:-true}" != true ]]; then
        reason=imu_bias_filter_disabled
      elif [[ -z "${remap_cpus}" || -z "${filter_cpus}" || "${remap_cpus}" != "${filter_cpus}" ]]; then
        reason="different_or_unresolved_cpu_masks remap=${remap_cpus:-unset} filter=${filter_cpus:-unset}"
      else
        NJRH_IMU_PIPELINE_EFFECTIVE_MODE=intra_process
        reason="shared_cpu_mask_${remap_cpus}"
      fi
      ;;
    *)
      echo "[runtime-overlay] invalid NJRH_IMU_PIPELINE_MODE=${requested}; expected intra_process or standalone" >&2
      return 2
      ;;
  esac
  echo "[runtime-overlay] IMU pipeline mode=${NJRH_IMU_PIPELINE_EFFECTIVE_MODE} reason=${reason}; CPU policy and sensor parameters unchanged" >&2
}

njrh_imu_pipeline_composed() {
  [[ "${NJRH_IMU_PIPELINE_EFFECTIVE_MODE:-standalone}" == intra_process ]]
}

njrh_imu_pipeline_host_running() {
  pgrep -f "${NJRH_IMU_PIPELINE_PROCESS_PATTERN}" >/dev/null 2>&1
}

njrh_imu_standalone_remap_running() {
  pgrep -f "${NJRH_IMU_STANDALONE_REMAP_PATTERN}" >/dev/null 2>&1
}

njrh_any_imu_ingress_running() {
  njrh_imu_pipeline_host_running || njrh_imu_standalone_remap_running
}

njrh_expected_imu_ingress_running() {
  # A remap-only process must not satisfy a composed request (or vice versa).
  # Do not reuse a mixed old/new topology with two canonical IMU publishers.
  if njrh_imu_pipeline_composed; then
    njrh_imu_pipeline_host_running && ! njrh_imu_standalone_remap_running
  else
    njrh_imu_standalone_remap_running && ! njrh_imu_pipeline_host_running
  fi
}

njrh_apply_imu_pipeline_host_affinity() {
  local remap_cpus filter_cpus
  local -a host_pids=()
  mapfile -t host_pids < <(pgrep -f "${NJRH_IMU_PIPELINE_PROCESS_PATTERN}" 2>/dev/null || true)
  ((${#host_pids[@]} > 0)) || return 0
  remap_cpus="$(njrh_cpuset_for imu_axis_remap)"
  filter_cpus="$(njrh_cpuset_for robot_local_state_imu_bias_filter)"
  if [[ -z "${remap_cpus}" || -z "${filter_cpus}" || "${remap_cpus}" != "${filter_cpus}" ]]; then
    echo "[runtime-overlay] IMU host affinity unchanged: remap=${remap_cpus:-unset} filter=${filter_cpus:-unset} cannot share one existing role; use standalone IMU startup before applying separate CPU masks" >&2
    return 0
  fi
  # Reuse the existing role/placement function; never invent a combined mask or
  # apply both old roles in sequence to every thread of the same process.
  njrh_apply_affinity_to_pids imu_axis_remap "${host_pids[@]}"
}

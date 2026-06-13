#!/usr/bin/env bash

if [[ -z "${NJRH_OVERLAY_ROOT:-}" ]]; then
  echo "[runtime-overlay] cpu_affinity.sh requires common_env.sh to be sourced first" >&2
  return 1 2>/dev/null || exit 1
fi

NJRH_CPU_AFFINITY_CONFIG="${NJRH_CPU_AFFINITY_CONFIG:-${NJRH_OVERLAY_ROOT}/config/cpu_affinity.env}"
if [[ -f "${NJRH_CPU_AFFINITY_CONFIG}" ]]; then
  # shellcheck source=../config/cpu_affinity.env
  source "${NJRH_CPU_AFFINITY_CONFIG}"
fi

NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE="${NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE:-${NJRH_OVERLAY_ROOT}/config/cpu_affinity_runtime_override.env}"
if [[ -f "${NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE}" ]]; then
  # shellcheck source=../config/cpu_affinity_runtime_override.env
  source "${NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE}"
fi
if declare -F njrh_resolve_nav2_controller_cpuset_profile >/dev/null 2>&1; then
  njrh_resolve_nav2_controller_cpuset_profile
fi

njrh_affinity_truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

njrh_affinity_enabled() {
  njrh_affinity_truthy "${NJRH_CPU_AFFINITY_ENABLED:-true}" && command -v taskset >/dev/null 2>&1
}

njrh_affinity_var_name() {
  local service_name="$1"
  local key="${service_name//-/_}"
  key="${key//./_}"
  key="${key//\//_}"
  key="${key^^}"
  printf 'NJRH_CPUSET_%s\n' "${key}"
}

njrh_cpuset_for() {
  local service_name="$1"
  local var_name
  var_name="$(njrh_affinity_var_name "${service_name}")"
  printf '%s\n' "${!var_name:-}"
}

njrh_run_affined() {
  local service_name="$1"
  shift
  local cpuset
  cpuset="$(njrh_cpuset_for "${service_name}")"
  if njrh_affinity_enabled && [[ -n "${cpuset}" ]]; then
    echo "[runtime-overlay] cpu affinity: ${service_name} -> CPU ${cpuset}" >&2
    taskset -c "${cpuset}" "$@"
    return $?
  fi
  "$@"
}

njrh_start_affined_background() {
  local pid_var="$1"
  local service_name="$2"
  shift 2
  local cpuset
  cpuset="$(njrh_cpuset_for "${service_name}")"
  if njrh_affinity_enabled && [[ -n "${cpuset}" ]]; then
    echo "[runtime-overlay] cpu affinity: ${service_name} -> CPU ${cpuset}" >&2
    taskset -c "${cpuset}" "$@" &
  else
    "$@" &
  fi
  printf -v "${pid_var}" '%s' "$!"
}

njrh_exec_affined() {
  local service_name="$1"
  shift
  local cpuset
  cpuset="$(njrh_cpuset_for "${service_name}")"
  if njrh_affinity_enabled && [[ -n "${cpuset}" ]]; then
    echo "[runtime-overlay] cpu affinity: ${service_name} -> CPU ${cpuset}" >&2
    exec taskset -c "${cpuset}" "$@"
  fi
  exec "$@"
}

njrh_apply_affinity_to_pids() {
  local service_name="$1"
  shift
  local cpuset
  cpuset="$(njrh_cpuset_for "${service_name}")"
  if ! njrh_affinity_enabled || [[ -z "${cpuset}" ]]; then
    return 0
  fi
  local pid
  for pid in "$@"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      local task_path
      local task_id
      local task_count=0
      local failed_count=0
      if [[ -d "/proc/${pid}/task" ]]; then
        for task_path in /proc/"${pid}"/task/*; do
          [[ -e "${task_path}" ]] || continue
          task_id="${task_path##*/}"
          if taskset -pc "${cpuset}" "${task_id}" >/dev/null 2>&1; then
            task_count=$((task_count + 1))
          else
            failed_count=$((failed_count + 1))
          fi
        done
      else
        if taskset -pc "${cpuset}" "${pid}" >/dev/null 2>&1; then
          task_count=1
        else
          failed_count=1
        fi
      fi
      echo "[runtime-overlay] cpu affinity applied: ${service_name} pid=${pid} tasks=${task_count} failed=${failed_count} -> CPU ${cpuset}" >&2
    fi
  done
}

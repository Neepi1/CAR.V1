#!/usr/bin/env bash

if [[ -z "${NJRH_OVERLAY_ROOT:-}" ]]; then
  echo "[runtime-overlay] cpu_affinity.sh requires common_env.sh to be sourced first" >&2
  return 1 2>/dev/null || exit 1
fi

source "${NJRH_OVERLAY_ROOT}/scripts/cpu_affinity_profiles.sh"
njrh_restore_navigation_cpu_baseline

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
# Apply after the site override AND the controller resolver, so inherited
# CPU5/6/7 values cannot escape the optional five-core navigation profile.
njrh_apply_navigation_cpu_profile || return $?

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

# The environment retains steady-state masks. Only an explicitly owned cold
# startup session changes launch placement, including processes started later.
njrh_startup_cpu_session_enabled() {
  [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" &&
     "${NJRH_NAVIGATION_CPU_PROFILE:-site_default}" == navigation_5cpu ]] || return 1
  njrh_affinity_enabled || return 1
  local key candidate
  key="$(njrh_affinity_var_name "$1")"
  for candidate in "${NJRH_NAVIGATION_CPU_KEYS[@]}"; do
    [[ "${key}" == "NJRH_CPUSET_${candidate}" ]] && return 0
  done
  return 1
}

njrh_affinity_prefix() {
  local -n result="$1"
  local service_name="$2" cpuset
  result=()
  cpuset="$(njrh_cpuset_for "${service_name}")"
  njrh_affinity_enabled && [[ -n "${cpuset}" ]] || return 0
  if njrh_startup_cpu_session_enabled "${service_name}"; then
    result=(python3 "${NJRH_OVERLAY_ROOT}/scripts/startup_cpu_affinity.py"
      exec --session "${NJRH_STARTUP_CPU_SESSION}" --steady-cpus "${cpuset}"
      --role "${service_name}")
    # API business children (mapping/arm) are not owned by cold navigation.
    [[ "${service_name}" != robot_api_server ]] || result+=(--no-descendants)
    result+=(--)
  else
    result=(taskset -c "${cpuset}")
  fi
}

njrh_effective_cpuset_for() {
  local cpuset
  cpuset="$(njrh_cpuset_for "$1")"
  if njrh_startup_cpu_session_enabled "$1"; then
    python3 "${NJRH_OVERLAY_ROOT}/scripts/startup_cpu_affinity.py" mask \
      --session "${NJRH_STARTUP_CPU_SESSION}" --steady-cpus "${cpuset}"
  else
    printf '%s\n' "${cpuset}"
  fi
}

njrh_begin_startup_cpu_boost() {
  [[ "${NJRH_NAVIGATION_CPU_PROFILE:-site_default}" == navigation_5cpu ]] || return 0
  njrh_affinity_truthy "${NJRH_STARTUP_CPU_BOOST_ENABLED:-true}" || return 0
  njrh_affinity_enabled || return 0
  local owner_pid="${BASHPID:-$$}" session
  if session="$(python3 "${NJRH_OVERLAY_ROOT}/scripts/startup_cpu_affinity.py" begin \
      --owner-pid "${owner_pid}" --session-dir /tmp/njrh_reports/startup_cpu_affinity)"; then
    export NJRH_STARTUP_CPU_SESSION="${session}"
    if ! njrh_apply_affinity_to_current_process navigation_runtime_owner; then
      njrh_finish_startup_cpu_boost placement_failed
    fi
  else
    echo "[runtime-overlay] startup CPU boost unavailable; retaining steady placement" >&2
  fi
}

njrh_finish_startup_cpu_boost() {
  [[ -n "${NJRH_STARTUP_CPU_SESSION:-}" ]] || return 0
  if ! python3 "${NJRH_OVERLAY_ROOT}/scripts/startup_cpu_affinity.py" finish \
      --session "${NJRH_STARTUP_CPU_SESSION}" --reason "$1"; then
    echo "[runtime-overlay] startup CPU restore incomplete; inspect startup CPU session report" >&2
  fi
  # Placement errors are diagnostics, not a new navigation readiness gate.
  return 0
}

njrh_apply_affinity_to_current_process() {
  local service_name="$1"
  local cpuset
  local pid
  local actual
  cpuset="$(njrh_cpuset_for "${service_name}")"
  if ! njrh_affinity_truthy "${NJRH_CPU_AFFINITY_ENABLED:-true}" || [[ -z "${cpuset}" ]]; then
    return 0
  fi
  if ! command -v taskset >/dev/null 2>&1; then
    echo "[runtime-overlay] ERROR: taskset is required to apply ${service_name} CPU affinity" >&2
    return 1
  fi
  pid="${BASHPID:-$$}"
  if njrh_startup_cpu_session_enabled "${service_name}"; then
    local extra=()
    [[ "${service_name}" != robot_api_server ]] || extra=(--no-descendants)
    python3 "${NJRH_OVERLAY_ROOT}/scripts/startup_cpu_affinity.py" register \
      --session "${NJRH_STARTUP_CPU_SESSION}" --pid "${pid}" \
      --steady-cpus "${cpuset}" --role "${service_name}" "${extra[@]}"
    return $?
  fi
  if ! taskset -pc "${cpuset}" "${pid}" >/dev/null 2>&1; then
    echo "[runtime-overlay] ERROR: failed to apply ${service_name} pid=${pid} -> CPU ${cpuset}" >&2
    return 1
  fi
  actual="$(awk '/^Cpus_allowed_list:/ {print $2}' "/proc/${pid}/status" 2>/dev/null || true)"
  if [[ -z "${actual}" ]]; then
    echo "[runtime-overlay] ERROR: failed to verify ${service_name} pid=${pid} CPU affinity" >&2
    return 1
  fi
  echo "[runtime-overlay] cpu affinity applied: ${service_name} pid=${pid} -> CPU ${actual}" >&2
}

njrh_run_affined() {
  local service_name="$1"
  shift
  local affinity=()
  njrh_affinity_prefix affinity "${service_name}"
  "${affinity[@]}" "$@"
}

njrh_start_affined_background() {
  local pid_var="$1"
  local service_name="$2"
  shift 2
  local affinity=()
  njrh_affinity_prefix affinity "${service_name}"
  "${affinity[@]}" "$@" &
  printf -v "${pid_var}" '%s' "$!"
}

njrh_exec_affined() {
  local service_name="$1"
  shift
  local affinity=()
  njrh_affinity_prefix affinity "${service_name}"
  exec "${affinity[@]}" "$@"
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
      if njrh_startup_cpu_session_enabled "${service_name}"; then
        python3 "${NJRH_OVERLAY_ROOT}/scripts/startup_cpu_affinity.py" register \
          --session "${NJRH_STARTUP_CPU_SESSION}" --pid "${pid}" \
          --steady-cpus "${cpuset}" --role "${service_name}"
        continue
      fi
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

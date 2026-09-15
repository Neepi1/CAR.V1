#!/usr/bin/env bash
# Internal AMCL startup bookkeeping. This file never starts or stops a ROS node.

amcl_monotonic_ms() {
  awk '{printf "%.0f\n", $1 * 1000}' /proc/uptime
}

amcl_budget_begin() {
  local seconds="${1:-45}" now duration
  [[ "${seconds}" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]] || return 2
  now="$(amcl_monotonic_ms)" || return 1
  if [[ ! "${NJRH_AMCL_STARTUP_DEADLINE_MS:-}" =~ ^[0-9]+$ ]]; then
    duration="$(awk -v s="${seconds}" 'BEGIN {printf "%.0f", s * 1000}')"
    NJRH_AMCL_STARTUP_DEADLINE_MS=$((now + duration))
    export NJRH_AMCL_STARTUP_DEADLINE_MS
  fi
  (( NJRH_AMCL_STARTUP_DEADLINE_MS > now )) || return 124
}

amcl_budget_timeout() {
  local requested="${1:?client timeout required}" now remaining
  [[ "${requested}" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]] || return 2
  if [[ "${NJRH_AMCL_STARTUP_DEADLINE_MS:-}" =~ ^[0-9]+$ ]]; then
    now="$(amcl_monotonic_ms)" || return 1
    # Reserve room for the short client's TERM -> KILL grace and shell return.
    remaining=$((NJRH_AMCL_STARTUP_DEADLINE_MS - now - 250))
    (( remaining > 0 )) || return 124
    awk -v s="${requested}" -v ms="${remaining}" \
      'BEGIN {if (s < .001) exit 124; printf "%.3f\n", (s * 1000 < ms ? s : ms / 1000)}'
  else
    awk -v s="${requested}" 'BEGIN {if (s < .001) exit 124; printf "%.3f\n", s}'
  fi
}

amcl_client_timeout() {
  local requested="${1:?client timeout required}" limit started finished rc=0
  shift
  [[ $# -gt 0 ]] || return 2
  limit="$(amcl_budget_timeout "${requested}")" || return $?
  started="$(amcl_monotonic_ms)" || return 1
  # Callers pass short-lived clients only, never a resident ROS process.
  timeout --kill-after=0.2s "${limit}s" "$@" || rc=$?
  finished="$(amcl_monotonic_ms)" || finished="${started}"
  printf '[runtime-overlay] AMCL_CLIENT step=%s elapsed_ms=%s exit=%s budget_sec=%s\n' \
    "${AMCL_CLIENT_STEP:-${1##*/}}" "$((finished - started))" "${rc}" "${limit}" >&2
  return "${rc}"
}

# A side-effecting request must not start with the tail of another phase's
# budget. Keep the same deadline; the existing supervisor resumes next round.
amcl_require_client_budget() {
  local requested="${1:?client timeout required}" available
  available="$(amcl_budget_timeout "${requested}")" || return $?
  if ! awk -v available="${available}" -v requested="${requested}" \
    'BEGIN {exit !(available >= requested)}'; then
    printf '[runtime-overlay] AMCL_CLIENT_DEFER step=seed available_sec=%s required_sec=%s at=%s\n' \
      "${available}" "${requested}" "$(date -u +%FT%TZ)" >&2
    return 124
  fi
}

amcl_budget_sleep() {
  local seconds="${1:?sleep duration required}" required now
  [[ "${seconds}" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]] || return 2
  if [[ "${NJRH_AMCL_STARTUP_DEADLINE_MS:-}" =~ ^[0-9]+$ ]]; then
    now="$(amcl_monotonic_ms)" || return 1
    required="$(awk -v s="${seconds}" 'BEGIN {printf "%.0f", s * 1000}')"
    (( NJRH_AMCL_STARTUP_DEADLINE_MS - now >= required + 250 )) || return 124
  fi
  sleep "${seconds}" || return $?
  if [[ "${NJRH_AMCL_STARTUP_DEADLINE_MS:-}" =~ ^[0-9]+$ ]]; then
    now="$(amcl_monotonic_ms)" || return 1
    (( now < NJRH_AMCL_STARTUP_DEADLINE_MS )) || return 124
  fi
  return 0
}

amcl_process_starttime() {
  local pid="${1:-}" stat tail
  [[ "${pid}" =~ ^[1-9][0-9]*$ && -r "/proc/${pid}/stat" ]] || return 1
  IFS= read -r stat <"/proc/${pid}/stat" || return 1
  # comm may contain spaces or ')'; the final ') ' precedes field 3.
  tail="${stat##*) }"
  set -- ${tail}
  [[ $# -ge 20 && "${1}" != Z && "${20}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${20}"
}

amcl_progress_identity() {
  local pid started owner owner_started map params manifest image_name image_path
  local map_hash params_hash asset_hash="" boot_id
  pid="$(amcl_pid_from_file 2>/dev/null)" || return 1
  started="$(amcl_process_starttime "${pid}")" || return 1
  owner="${NJRH_STARTUP_OWNER_PID:-}"
  owner_started="$(amcl_process_starttime "${owner}")" || return 1
  map="$(readlink -f -- "${NAV2_MAP_YAML:-}")" || return 1
  params="$(readlink -f -- "${PARAMS_FILE:-}")" || return 1
  [[ -r "${map}" && -r "${params}" ]] || return 1
  map_hash="$(sha256sum -- "${map}")" || return 1
  params_hash="$(sha256sum -- "${params}")" || return 1
  for manifest in "$(dirname "${map}")/../manifest.json" "$(dirname "${map}")/../../manifest.json"; do
    if [[ -f "${manifest}" ]]; then
      asset_hash="$(sha256sum -- "${manifest}")" || return 1
      break
    fi
  done
  if [[ -z "${asset_hash}" ]]; then
    # Standard Nav2 assets use a scalar image path; an unfamiliar YAML shape
    # disables cache reuse rather than guessing a same-map identity.
    image_name="$(sed -n 's/^[[:space:]]*image:[[:space:]]*//p' "${map}" | head -n 1)"
    image_name="${image_name%$'\r'}"
    if [[ "${image_name}" == \"*\" || "${image_name}" == \'*\' ]]; then
      image_name="${image_name:1:${#image_name}-2}"
    fi
    [[ -n "${image_name}" ]] || return 1
    image_path="${image_name}"
    [[ "${image_path}" == /* ]] || image_path="$(dirname "${map}")/${image_path}"
    [[ -r "${image_path}" ]] || return 1
    asset_hash="$(sha256sum -- "${image_path}")" || return 1
  fi
  IFS= read -r boot_id </proc/sys/kernel/random/boot_id || return 1
  # Hash only explicit identity fields: never dump or source process environ.
  printf '%s\0' amcl-progress-v1 "${boot_id}" "${pid}" "${started}" "${owner}" \
    "${owner_started}" "${NJRH_FLOOR_STARTUP_HANDOFF_NONCE:-}" "${map}" "${map_hash}" \
    "${params}" "${params_hash}" "${asset_hash}" "${MODE:-}" \
    "${NJRH_BUILDING_ID:-}" "${NJRH_FLOOR_ID:-}" "${NJRH_NAV_MAP_ID:-}" "${NJRH_MAP_ID:-}" |
    sha256sum | awk '{print $1}'
}

amcl_progress_reset() {
  local phase
  AMCL_PROGRESS_KEY=""
  AMCL_PROGRESS_SCAN_FRAME=""
  for phase in LIFECYCLE PARAM MAP SCAN ODOM_TF SENSOR_TF MAP_TF WARMUP; do
    printf -v "AMCL_PROGRESS_${phase}" '%s' false
  done
}

amcl_progress_load() {
  local file="${NJRH_AMCL_STARTUP_PROGRESS_FILE:-${STATUS_FILE}.startup-progress}"
  local expected saved_key="" saved_frame="" key value
  local -a proven=()
  amcl_progress_reset
  expected="$(amcl_progress_identity)" || return 0
  [[ "${expected}" =~ ^[a-f0-9]{64}$ ]] || return 0
  AMCL_PROGRESS_KEY="${expected}"
  [[ -r "${file}" ]] || return 0
  while IFS='=' read -r key value; do
    case "${key}" in
      AMCL_PROGRESS_KEY) saved_key="${value}" ;;
      AMCL_PROGRESS_SCAN_FRAME) saved_frame="${value}" ;;
      AMCL_PROGRESS_LIFECYCLE|AMCL_PROGRESS_PARAM|AMCL_PROGRESS_MAP|AMCL_PROGRESS_SCAN|AMCL_PROGRESS_ODOM_TF|AMCL_PROGRESS_SENSOR_TF|AMCL_PROGRESS_MAP_TF|AMCL_PROGRESS_WARMUP)
        [[ "${value}" == true ]] && proven+=("${key}") ;;
    esac
  done <"${file}"
  [[ "${saved_key}" == "${expected}" ]] || return 0
  AMCL_PROGRESS_SCAN_FRAME="${saved_frame}"
  for key in "${proven[@]}"; do printf -v "${key}" '%s' true; done
}

amcl_progress_save() {
  local file="${NJRH_AMCL_STARTUP_PROGRESS_FILE:-${STATUS_FILE}.startup-progress}"
  local identity temp phase variable rc=0
  identity="$(amcl_progress_identity)" || return 0
  [[ -n "${AMCL_PROGRESS_KEY:-}" && "${identity}" == "${AMCL_PROGRESS_KEY}" ]] || return 0
  temp="$(mktemp "${file}.XXXXXX")" || {
    printf '[runtime-overlay] warning: AMCL startup progress could not be saved: %s\n' "${file}" >&2
    return 0
  }
  {
    printf 'AMCL_PROGRESS_KEY=%s\n' "${identity}"
    printf 'AMCL_PROGRESS_SCAN_FRAME=%s\n' "${AMCL_PROGRESS_SCAN_FRAME:-}"
    for phase in LIFECYCLE PARAM MAP SCAN ODOM_TF SENSOR_TF MAP_TF WARMUP; do
      variable="AMCL_PROGRESS_${phase}"
      if [[ "${!variable:-false}" == true ]]; then
        printf '%s=true\n' "${variable}"
      else
        printf '%s=false\n' "${variable}"
      fi
    done
  } >"${temp}" || rc=$?
  if (( rc == 0 )) && mv -f -- "${temp}" "${file}"; then return 0; fi
  rm -f -- "${temp}"
  printf '[runtime-overlay] warning: AMCL startup progress could not be saved: %s\n' "${file}" >&2
  return 0
}

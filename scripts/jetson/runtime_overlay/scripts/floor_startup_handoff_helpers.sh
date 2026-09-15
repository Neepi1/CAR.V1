#!/usr/bin/env bash
# Sourced after commercial_runtime_helpers.sh. Only the cold-start owner uses
# this handshake; neither a missing action graph nor a timeout grants motion.

export NJRH_FLOOR_STARTUP_HANDOFF_FILE="${NJRH_FLOOR_STARTUP_HANDOFF_FILE:-/tmp/njrh_floor_startup_handoff.json}"
export NJRH_FLOOR_STARTUP_HANDOFF_ACK_FILE="${NJRH_FLOOR_STARTUP_HANDOFF_ACK_FILE:-/tmp/njrh_floor_startup_handoff_ack.json}"
export NJRH_FLOOR_STARTUP_HANDOFF_EVIDENCE_FILE="${NJRH_FLOOR_STARTUP_HANDOFF_EVIDENCE_FILE:-/tmp/njrh_floor_startup_target_evidence_$$.json}"
export NJRH_STARTUP_TRIGGER_OUTCOME_FILE="${NJRH_STARTUP_TRIGGER_OUTCOME_FILE:-/tmp/njrh_startup_trigger_outcome_$$.json}"
export NJRH_STARTUP_OWNER_PID="$$"
export NJRH_STARTUP_INSTANCE="startup-$$-$(date +%s%N)"
floor_startup_handoff_active=0

# Keep the original writer, but fence every caller including stage logging,
# degraded AMCL, failure cleanup, and ready commit. The request lives separately
# so a writer already in flight cannot overwrite the handoff request itself.
startup_context_writer_definition="$(declare -f write_runtime_map_context)"
eval "${startup_context_writer_definition/write_runtime_map_context/write_startup_runtime_map_context_unfenced}"
unset startup_context_writer_definition

floor_handoff_requested() {
  [[ -e "${NJRH_FLOOR_STARTUP_HANDOFF_FILE}" ]] || return 1
  floor_handoff_cli is-requested
}

write_runtime_map_context() {
  if [[ "${floor_startup_handoff_active:-0}" -eq 1 ]] || floor_handoff_requested; then
    echo "[runtime-overlay] floor-manager owns runtime context; suppressing startup write state=$1" >&2
    return 0
  fi
  write_startup_runtime_map_context_unfenced "$@"
}

floor_handoff_cli() {
  python3 "${SCRIPT_DIR}/floor_startup_handoff.py" "$@"
}

floor_handoff_guard() {
  if [[ "${floor_startup_handoff_active:-0}" -eq 1 ]]; then
    floor_handoff_cli assert-current
  else
    ! floor_handoff_requested
  fi
}

join_startup_side_effect_worker() {
  local variable="$1"
  local pid="${!variable}"
  [[ -n "${pid}" ]] || return 0
  # Never cancel a lifecycle/trigger RPC client and then call that quiescence.
  # A normal unsuccessful completion is not enough to prove an unknown RPC.
  local rc=0
  wait "${pid}" || rc=$?
  printf -v "${variable}" '%s' ''
  if [[ "${rc}" -ne 0 ]]; then
    echo "[runtime-overlay] handoff refused: prior ${variable} completed with rc=${rc}" >&2
    return 1
  fi
}

adopt_startup_floor_handoff() {
  [[ "${floor_startup_handoff_active:-0}" -eq 0 ]] || return 0
  local assignments
  assignments="$(floor_handoff_cli export-env)" || return 1
  # Quoted assignments come only from validated immutable request fields.
  eval "${assignments}"
  floor_startup_handoff_active=1
  local variable
  for variable in nav2_lifecycle_bringup_pid amcl_resident_pid amcl_readiness_pid; do
    if ! join_startup_side_effect_worker "${variable}"; then
      floor_handoff_cli ack --state failed --failure STARTUP_WORKER_UNPROVEN \
        --detail "old startup side-effect worker did not complete successfully" || true
      return 1
    fi
  done
  if ! floor_handoff_cli check-trigger-outcome; then
    floor_handoff_cli ack --state failed --failure STARTUP_TRIGGER_OUTCOME_UNKNOWN \
      --detail "old trigger RPC has not proven a terminal response" || true
    return 1
  fi
  floor_handoff_cli ack --state adopted \
    --detail "old startup workers joined; exact immutable target adopted" || return 1
  echo "[runtime-overlay] floor startup handoff adopted transaction=${NJRH_RUNTIME_TRANSACTION_ID} map=${NJRH_MAP_ID}" >&2
  # This is a read-only typed observer. floor-manager now performs BEGIN/load/
  # trigger under its hold; pending target TF need not authorize normal goals.
  floor_handoff_cli wait-target || return 1
  assignments="$(floor_handoff_cli export-evidence)" || return 1
  eval "${assignments}"
  export NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK=true
}

complete_startup_floor_handoff() {
  floor_handoff_guard || return 1
  if [[ "${NJRH_AMCL_LOCALIZATION_MODE:-disabled}" == "disabled" ]]; then
    floor_handoff_cli ack --state failed --failure AMCL_DISABLED \
      --detail "strict floor switching requires target AMCL readiness" || true
    return 1
  fi
  wait_for_amcl_readiness_background_if_running || true
  load_amcl_runtime_status
  if [[ "${AMCL_READY:-false}" != "true" ]]; then
    complete_amcl_readiness_with_retries_for_navigation || return 1
  fi
  # Re-observe after lifecycle/AMCL work; do not acknowledge using old TF data.
  floor_handoff_cli wait-target || return 1
  floor_handoff_cli ack --state runtime_ready \
    --detail "target Nav2/costmap and AMCL prepared under floor-manager hold; awaiting COMMIT" || return 1
  floor_handoff_cli wait-commit || return 1
  runtime_ready=1
  echo "[runtime-overlay] floor-manager COMMIT observed; startup never wrote ready" >&2
}

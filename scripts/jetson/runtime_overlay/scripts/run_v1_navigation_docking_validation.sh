#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"

OBSERVE_ONLY=true
INCLUDE_MANUAL_RELOCALIZATION=false
INCLUDE_PREDOCK_YAW_PROBE=false
APPLY_SMALL_YAW_TEST=false
DURATION_SEC=120
OUTPUT_DIR=""
PREFIX="[v1-validation]"

usage() {
  cat <<'EOF'
Usage:
  run_v1_navigation_docking_validation.sh --observe-only --duration-sec 120
  run_v1_navigation_docking_validation.sh --observe-only --include-manual-relocalization
  run_v1_navigation_docking_validation.sh --include-predock-yaw-probe --apply-small-yaw-test

Default is observe-only and does not send motion commands. Manual relocalization
is only run with --include-manual-relocalization. The predock yaw motion probe
only publishes /cmd_vel_docking when both --include-predock-yaw-probe and
--apply-small-yaw-test are provided.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --observe-only)
      OBSERVE_ONLY=true
      shift
      ;;
    --include-manual-relocalization)
      INCLUDE_MANUAL_RELOCALIZATION=true
      shift
      ;;
    --include-predock-yaw-probe)
      INCLUDE_PREDOCK_YAW_PROBE=true
      shift
      ;;
    --apply-small-yaw-test)
      APPLY_SMALL_YAW_TEST=true
      shift
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 10 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 10" >&2
  exit 2
fi
if [[ "${APPLY_SMALL_YAW_TEST}" == "true" && "${INCLUDE_PREDOCK_YAW_PROBE}" != "true" ]]; then
  echo "${PREFIX} FAIL --apply-small-yaw-test requires --include-predock-yaw-probe" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/v1_validation_${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

run_step() {
  local name="$1"
  shift
  echo "${PREFIX} running ${name}"
  if "$@" >"${OUTPUT_DIR}/${name}.log" 2>&1; then
    echo "PASS" >"${OUTPUT_DIR}/${name}.result"
  else
    echo "FAIL" >"${OUTPUT_DIR}/${name}.result"
  fi
}

latest_summary_result() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "UNKNOWN"
  elif grep -Eq 'result: `?FAIL`?|result: FAIL' "${path}"; then
    echo "FAIL"
  elif grep -Eq 'result: `?PASS`?|result: PASS' "${path}"; then
    echo "PASS"
  else
    echo "UNKNOWN"
  fi
}

run_step runtime_config_audit bash "${SCRIPT_DIR}/verify_goal_completion_semantics.sh"
run_step predock_yaw_contract bash "${SCRIPT_DIR}/verify_docking_framework_state_machine.sh"
run_step fine_docking_entry_gate bash "${SCRIPT_DIR}/verify_fine_docking_entry_gate.sh" \
  --output-dir "${OUTPUT_DIR}/fine_docking_entry_gate"

POSE_DIR="${OUTPUT_DIR}/pose_required_navigation"
PREDOCK_TRACE_DIR="${OUTPUT_DIR}/predock_yaw_alignment"
bash "${SCRIPT_DIR}/observe_pose_required_navigation.sh" \
  --duration-sec "${DURATION_SEC}" \
  --output-dir "${POSE_DIR}" \
  >"${OUTPUT_DIR}/pose_required_navigation.log" 2>&1 &
pose_pid=$!
bash "${SCRIPT_DIR}/observe_predock_yaw_alignment_trace.sh" \
  --duration-sec "${DURATION_SEC}" \
  --output-dir "${PREDOCK_TRACE_DIR}" \
  >"${OUTPUT_DIR}/predock_yaw_alignment.log" 2>&1 &
predock_pid=$!
wait "${pose_pid}" || true
wait "${predock_pid}" || true

MANUAL_DIR="${OUTPUT_DIR}/manual_relocalization_api"
manual_result="UNKNOWN"
if [[ "${INCLUDE_MANUAL_RELOCALIZATION}" == "true" ]]; then
  if bash "${SCRIPT_DIR}/verify_manual_relocalization_api.sh" \
    --output-dir "${MANUAL_DIR}" \
    >"${OUTPUT_DIR}/manual_relocalization_api.log" 2>&1; then
    manual_result="PASS"
  else
    manual_result="FAIL"
  fi
fi

PROBE_DIR="${OUTPUT_DIR}/predock_yaw_probe"
probe_result="UNKNOWN"
if [[ "${INCLUDE_PREDOCK_YAW_PROBE}" == "true" ]]; then
  probe_args=(--duration-sec "${DURATION_SEC}" --output-dir "${PROBE_DIR}")
  if [[ "${APPLY_SMALL_YAW_TEST}" == "true" ]]; then
    probe_args+=(--apply-small-yaw-test)
  fi
  if bash "${SCRIPT_DIR}/run_predock_yaw_alignment_probe.sh" "${probe_args[@]}" \
    >"${OUTPUT_DIR}/predock_yaw_probe.log" 2>&1; then
    probe_result="PASS"
  else
    probe_result="FAIL"
  fi
fi

normal_result="$(latest_summary_result "${POSE_DIR}/summary.md")"
if [[ -f "${OUTPUT_DIR}/runtime_config_audit.result" && "$(cat "${OUTPUT_DIR}/runtime_config_audit.result")" == "FAIL" ]]; then
  normal_result="FAIL"
fi
fine_result="$(latest_summary_result "${OUTPUT_DIR}/fine_docking_entry_gate/summary.md")"
predock_contract_result="$(cat "${OUTPUT_DIR}/predock_yaw_contract.result" 2>/dev/null || echo UNKNOWN)"
predock_ready="${probe_result}"
if [[ "${probe_result}" == "UNKNOWN" && "${predock_contract_result}" == "PASS" ]]; then
  predock_ready="UNKNOWN"
elif [[ "${predock_contract_result}" == "FAIL" ]]; then
  predock_ready="FAIL"
fi

no_cmd_owner_conflict=true
robot_safety_chain_intact=true
runtime_report_dirs=("${POSE_DIR}" "${PREDOCK_TRACE_DIR}")
if [[ -d "${PROBE_DIR}" ]]; then
  runtime_report_dirs+=("${PROBE_DIR}")
fi
if grep -R \
  "cmd_owner_conflict_detected[\"': ]*true\\|PREDOCK_YAW_ALIGN_OWNER_CONFLICT" \
  "${runtime_report_dirs[@]}" >/dev/null 2>&1; then
  no_cmd_owner_conflict=false
fi
if grep -R "robot_safety bypass\\|direct_cmd_vel_publish: \`true\`" \
  "${runtime_report_dirs[@]}" >/dev/null 2>&1; then
  robot_safety_chain_intact=false
fi

allowed=false
if [[ "${manual_result}" == "PASS" && "${predock_ready}" == "PASS" && "${fine_result}" == "PASS" &&
      "${no_cmd_owner_conflict}" == "true" && "${robot_safety_chain_intact}" == "true" ]]; then
  allowed=true
fi

{
  echo "# Phase V1 Navigation Docking Validation"
  echo
  echo "- report_dir: \`${OUTPUT_DIR}\`"
  echo "- observe_only: \`${OBSERVE_ONLY}\`"
  echo "- include_manual_relocalization: \`${INCLUDE_MANUAL_RELOCALIZATION}\`"
  echo "- include_predock_yaw_probe: \`${INCLUDE_PREDOCK_YAW_PROBE}\`"
  echo "- apply_small_yaw_test: \`${APPLY_SMALL_YAW_TEST}\`"
  echo
  echo "## Final Verdict"
  echo "- normal_pose_required_ready: ${normal_result}"
  echo "- manual_relocalization_ready: ${manual_result}"
  echo "- predock_yaw_alignment_ready: ${predock_ready}"
  echo "- fine_docking_entry_gate_ready: ${fine_result}"
  echo "- allowed_to_run_full_docking_test: ${allowed}"
  echo
  echo "## Inputs"
  echo "- runtime_config_audit: \`$(cat "${OUTPUT_DIR}/runtime_config_audit.result" 2>/dev/null || echo UNKNOWN)\`"
  echo "- goal_completion_semantics_log: \`runtime_config_audit.log\`"
  echo "- pose_required_summary: \`pose_required_navigation/summary.md\`"
  echo "- predock_yaw_trace_summary: \`predock_yaw_alignment/summary.md\`"
  echo "- manual_relocalization_summary: \`manual_relocalization_api/summary.md\`"
  echo "- fine_docking_entry_gate_summary: \`fine_docking_entry_gate/summary.md\`"
  echo "- predock_yaw_probe_summary: \`predock_yaw_probe/summary.md\`"
} >"${OUTPUT_DIR}/summary.md"

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"

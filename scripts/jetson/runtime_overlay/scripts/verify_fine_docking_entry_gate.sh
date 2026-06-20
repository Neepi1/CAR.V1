#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
OUTPUT_DIR=""
PREFIX="[fine-docking-gate]"

usage() {
  cat <<'EOF'
Usage:
  verify_fine_docking_entry_gate.sh [--output-dir DIR]

Static/read-only contract verifier for the Phase V1 fine docking entry gate.
It never calls /docking/start and never sends velocity commands.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/fine_docking_entry_gate_${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

API_CPP="${WORKSPACE_ROOT}/src/robot_api_server/src/robot_api_server_node.cpp"
API_CFG="${WORKSPACE_ROOT}/scripts/jetson/runtime_overlay/config/robot_api_server.yaml"
JOB_HPP="${WORKSPACE_ROOT}/src/robot_api_server/include/robot_api_server/docking_job_model.hpp"
JOB_CPP="${WORKSPACE_ROOT}/src/robot_api_server/src/docking_job_model.cpp"

pass=0
fail=0
rows=()

check_contains() {
  local label="$1"
  local file="$2"
  local needle="$3"
  if grep -Fq -- "${needle}" "${file}"; then
    rows+=("| ${label} | PASS | \`${file#${WORKSPACE_ROOT}/}\` contains \`${needle}\` |")
    pass=$((pass + 1))
  else
    rows+=("| ${label} | FAIL | missing \`${needle}\` in \`${file#${WORKSPACE_ROOT}/}\` |")
    fail=$((fail + 1))
  fi
}

check_contains "predock_yaw_aligned false blocks fine entry" "${API_CPP}" "fine_docking_entry_require_predock_yaw_aligned_ && !predock_yaw_aligned"
check_contains "PREDOCK_YAW_NOT_ALIGNED failure code exists" "${API_CPP}" "PREDOCK_YAW_NOT_ALIGNED"
check_contains "GS2 freshness required by config" "${API_CFG}" "fine_docking_entry_require_gs2_fresh: true"
check_contains "global correction pause is checked" "${API_CPP}" "global_correction_pause_applied"
check_contains "post predock settle is checked" "${API_CPP}" "post_predock_settle_complete"
check_contains "dock staging handoff gate exists" "${API_CPP}" "dock_staging_handoff_ready"
check_contains "predock pose verify gate exists" "${API_CPP}" "predock_pose_verified"
check_contains "fine bridge settle phase exists" "${API_CPP}" "FINE_DOCKING_BRIDGE_SETTLE"
check_contains "fine bridge smoothing wait exists" "${API_CPP}" "wait_for_bridge_smoothing_before_fine_docking"
check_contains "fine bridge transition timeout code exists" "${API_CPP}" "DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT"
check_contains "fine bridge wait config exists" "${API_CFG}" "docking_fine_wait_for_bridge_smoothing_enabled: true"
check_contains "fine entry result is separated from fine docking result" "${API_CPP}" "fine_entry_failure_code"
check_contains "fine entry fields exported in job model" "${JOB_HPP}" "bool fine_entry_ok"
check_contains "fine entry fields serialized" "${JOB_CPP}" "fine_entry_failure_code"
check_contains "fine docking start service remains behind API state machine" "${API_CPP}" "docking_start_client_"

{
  echo "# Fine Docking Entry Gate Verification"
  echo
  echo "- report_dir: \`${OUTPUT_DIR}\`"
  echo "- calls_docking_start: \`false\`"
  echo "- sends_velocity: \`false\`"
  echo "- pass: \`${pass}\`"
  echo "- fail: \`${fail}\`"
  echo
  echo "| Check | Result | Evidence |"
  echo "| --- | --- | --- |"
  printf '%s\n' "${rows[@]}"
  echo
  if [[ "${fail}" -eq 0 ]]; then
    echo "result: PASS"
  else
    echo "result: FAIL"
  fi
} >"${OUTPUT_DIR}/summary.md"

echo "${PREFIX} wrote ${OUTPUT_DIR}"
[[ "${fail}" -eq 0 ]]

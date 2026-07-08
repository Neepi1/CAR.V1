#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
PROFILE="d3"
DURATION_SEC=180
LABEL="docking_framework_ab"
APPLY=false
DOCK_JSON=""
OUTPUT_DIR=""
PREFIX="[dock-fw-ab]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/run_docking_framework_ab.sh --profile d3 --duration-sec 180
  bash scripts/jetson/runtime_overlay/scripts/run_docking_framework_ab.sh --profile d3 --apply --dock-json '{"building_id":"building_1","floor_id":"floor_1","dock_id":"dock_1"}'

Default mode verifies contracts and records an observation window only. With
--apply and --dock-json it calls POST /api/v1/docking/start after recorders
start. It never publishes velocity and never changes Nav2 planner/controller,
TF tolerances, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, or EKF.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    --apply)
      APPLY=true
      shift
      ;;
    --dock-json)
      DOCK_JSON="${2:-}"
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

if [[ "${PROFILE}" != "baseline" && "${PROFILE}" != "d3" ]]; then
  echo "${PREFIX} FAIL --profile must be baseline|d3" >&2
  exit 2
fi
if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 20 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 20" >&2
  exit 2
fi
if [[ "${APPLY}" == "true" && -z "${DOCK_JSON}" ]]; then
  echo "${PREFIX} FAIL --apply requires --dock-json" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/docking_framework_ab/${TIMESTAMP}_${PROFILE}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

summary="${OUTPUT_DIR}/summary.md"
{
  echo "# Docking Framework A/B"
  echo
  echo "- profile: ${PROFILE}"
  echo "- duration_sec: ${DURATION_SEC}"
  echo "- apply: ${APPLY}"
  echo "- changed_nav2_controller_or_planner_params: no"
  echo "- changed_tf_gate: no"
  echo "- changed_pointcloud_qos_or_dds: no"
  echo "- changed_fastlio: no"
  echo "- changed_ranger_odom_or_ekf: no"
  echo "- command_path: /cmd_vel_docking -> robot_safety -> /cmd_vel"
  echo "- cmd_vel_safe: robot_safety diagnostic mirror"
} >"${summary}"

bash "${SCRIPT_DIR}/verify_docking_framework_state_machine.sh" >"${OUTPUT_DIR}/verify.log" 2>&1
verify_rc=$?
if [[ "${verify_rc}" -ne 0 ]]; then
  echo "${PREFIX} FAIL contract verify failed: ${OUTPUT_DIR}/verify.log" >&2
  exit "${verify_rc}"
fi

observer_out="${OUTPUT_DIR}/observe"
mkdir -p "${observer_out}"
bash "${SCRIPT_DIR}/observe_docking_predock_yaw_align.sh" \
  --duration-sec "${DURATION_SEC}" \
  --label "${PROFILE}_${LABEL}" \
  --api-url "${API_URL}" \
  --output-dir "${observer_out}" &
observer_pid=$!

if [[ "${APPLY}" == "true" ]]; then
  sleep 3
  {
    echo "POST ${API_URL}/api/v1/docking/start"
    curl -fsS -X POST "${API_URL}/api/v1/docking/start" \
      -H 'Content-Type: application/json' \
      --data "${DOCK_JSON}" || true
    echo
  } >"${OUTPUT_DIR}/docking_start_response.json" 2>"${OUTPUT_DIR}/docking_start_response.err"
fi

wait "${observer_pid}"
observer_rc=$?
{
  echo "- observer_rc: ${observer_rc}"
  echo "- verify_log: ${OUTPUT_DIR}/verify.log"
  echo "- observer_summary: ${observer_out}/summary.md"
  echo "- rollback: set predock_yaw_align_enabled=false and docking_pause_global_correction_during_fine=false in robot_api_server.yaml, then restart robot_api_server"
} >>"${summary}"

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${summary}"
exit "${observer_rc}"

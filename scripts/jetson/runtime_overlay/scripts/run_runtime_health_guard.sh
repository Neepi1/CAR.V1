#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

guard_binary="${PROJECT_ROOT}/install/robot_bringup/lib/robot_bringup/runtime_health_guard"
[[ -x "${guard_binary}" ]] || {
  echo "[runtime-overlay] missing C++ runtime health guard; build robot_bringup: ${guard_binary}" >&2
  exit 1
}

export NJRH_RUNTIME_MANAGEMENT_ENABLED="${NJRH_RUNTIME_MANAGEMENT_ENABLED:-true}"
njrh_exec_affined runtime_health_guard "${guard_binary}" "$@"

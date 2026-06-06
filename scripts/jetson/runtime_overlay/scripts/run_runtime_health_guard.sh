#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

guard_script="${SCRIPT_DIR}/runtime_health_guard.py"
[[ -f "${guard_script}" ]] || {
  echo "[runtime-overlay] missing runtime health guard: ${guard_script}" >&2
  exit 1
}

njrh_exec_affined runtime_health_guard python3 "${guard_script}"

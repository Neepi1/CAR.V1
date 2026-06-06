#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

export CONFIG_FILE="${NJRH_PGO_CONFIG_FILE:-${UPSTREAM_WS}/src/fastlio_pgo/config/pgo.yaml}"

njrh_exec_affined pgo_mapping bash "$(require_upstream_script run_pgo.sh)"

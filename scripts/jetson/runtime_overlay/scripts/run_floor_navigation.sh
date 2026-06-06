#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[runtime-overlay] run_floor_navigation.sh is a compatibility wrapper; delegating to resident navigation runtime" >&2
exec bash "${SCRIPT_DIR}/run_navigation_runtime_services.sh" "$@"

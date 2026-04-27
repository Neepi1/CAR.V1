#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

map_name="${1:-}"
building_id="${2:-${NJRH_BUILDING_ID:-building_1}}"
floor_id="${3:-${NJRH_FLOOR_ID:-}}"

[[ -n "${map_name}" && -n "${floor_id}" ]] || {
  echo "usage: promote_map_to_floor.sh <map_name> <building_id> <floor_id>" >&2
  exit 2
}

TOOLKIT="${NJRH_PROJECT_ROOT}/src/robot_map_toolkit/scripts/map_toolkit_cli.py"
[[ -f "${TOOLKIT}" ]] || {
  echo "[runtime-overlay] robot_map_toolkit CLI missing: ${TOOLKIT}" >&2
  exit 1
}

python3 "${TOOLKIT}" \
  --maps-root "${NJRH_RELEASE_ASSETS_DIR}" \
  --building-id "${building_id}" \
  --floor-id "${floor_id}" \
  --flat-maps-dir "${NJRH_MAPS_DIR}" \
  --flat-map-name "${map_name}"

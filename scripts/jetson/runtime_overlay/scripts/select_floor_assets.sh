#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/floor_asset_helpers.sh"

building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
floor_id="${2:-${NJRH_FLOOR_ID:-}}"

resolve_floor_assets "${building_id}" "${floor_id}"

cat <<EOF
export NJRH_BUILDING_ID='${NJRH_BUILDING_ID}'
export NJRH_FLOOR_ID='${NJRH_FLOOR_ID}'
export NJRH_CURRENT_FLOOR_ROOT='${NJRH_CURRENT_FLOOR_ROOT}'
export NAV2_MAP_YAML='${NAV2_MAP_YAML}'
export NAV2_LOCALIZER_MAP_YAML='${NAV2_LOCALIZER_MAP_YAML}'
export NAV2_LOCALIZER_MAP_PNG='${NAV2_LOCALIZER_MAP_PNG}'
export NAV2_KEEP_OUT_MASK_YAML='${NAV2_KEEP_OUT_MASK_YAML}'
export NAV2_SPEED_MASK_YAML='${NAV2_SPEED_MASK_YAML}'
export NAV2_BINARY_MASK_YAML='${NAV2_BINARY_MASK_YAML}'
export NJRH_FLOOR_POSES_YAML='${NJRH_FLOOR_POSES_YAML}'
EOF

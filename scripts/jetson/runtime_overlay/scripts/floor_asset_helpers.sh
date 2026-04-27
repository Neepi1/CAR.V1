#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

validate_floor_assets() {
  local floor_root="$1"
  local missing=()
  local required=(
    "nav/nav_map.yaml"
    "nav/nav_map.pgm"
    "localizer/localizer_map.png"
    "localizer/localizer_params.yaml"
    "filters/keepout_mask.yaml"
    "filters/keepout_mask.pgm"
    "filters/speed_mask.yaml"
    "filters/speed_mask.pgm"
    "filters/binary_mask.yaml"
    "filters/binary_mask.pgm"
    "reports/asset_report.json"
    "poses.yaml"
  )

  local rel
  for rel in "${required[@]}"; do
    if [[ ! -f "${floor_root}/${rel}" ]]; then
      missing+=("${floor_root}/${rel}")
    fi
  done

  if [[ "${#missing[@]}" -gt 0 ]]; then
    printf '[runtime-overlay] floor asset validation failed under %s\n' "${floor_root}" >&2
    printf '  missing: %s\n' "${missing[@]}" >&2
    return 1
  fi
}

resolve_floor_assets() {
  local building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
  local floor_id="${2:-${NJRH_FLOOR_ID:-}}"

  [[ -n "${floor_id}" ]] || {
    echo "[runtime-overlay] floor_id is required" >&2
    return 1
  }

  local floor_root="${NJRH_RELEASE_ASSETS_DIR}/${building_id}/${floor_id}"
  validate_floor_assets "${floor_root}"

  export NJRH_BUILDING_ID="${building_id}"
  export NJRH_FLOOR_ID="${floor_id}"
  export NJRH_CURRENT_FLOOR_ROOT="${floor_root}"
  export NAV2_MAP_YAML="${floor_root}/nav/nav_map.yaml"
  export NAV2_LOCALIZER_MAP_YAML="${floor_root}/localizer/localizer_params.yaml"
  export NAV2_LOCALIZER_MAP_PNG="${floor_root}/localizer/localizer_map.png"
  export NAV2_KEEP_OUT_MASK_YAML="${floor_root}/filters/keepout_mask.yaml"
  export NAV2_SPEED_MASK_YAML="${floor_root}/filters/speed_mask.yaml"
  export NAV2_BINARY_MASK_YAML="${floor_root}/filters/binary_mask.yaml"
  export NJRH_FLOOR_POSES_YAML="${floor_root}/poses.yaml"

  printf '[runtime-overlay] selected floor %s/%s\n' "${building_id}" "${floor_id}" >&2
  printf '[runtime-overlay] NAV2_MAP_YAML=%s\n' "${NAV2_MAP_YAML}" >&2
  printf '[runtime-overlay] NAV2_LOCALIZER_MAP_YAML=%s\n' "${NAV2_LOCALIZER_MAP_YAML}" >&2
}

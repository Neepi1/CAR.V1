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

floor_asset_context_ready_for() {
  local building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
  local floor_id="${2:-${NJRH_FLOOR_ID:-}}"

  [[ "${NJRH_FLOOR_ASSET_CONTEXT_READY:-}" == "1" ]] || return 1
  [[ -n "${floor_id}" ]] || return 1
  [[ "${NJRH_BUILDING_ID:-}" == "${building_id}" ]] || return 1
  [[ "${NJRH_FLOOR_ID:-}" == "${floor_id}" ]] || return 1
  [[ "${NJRH_MAP_CONTEXT_BUILDING_ID:-}" == "${building_id}" ]] || return 1
  [[ "${NJRH_MAP_CONTEXT_FLOOR_ID:-}" == "${floor_id}" ]] || return 1
  [[ -n "${NJRH_CURRENT_FLOOR_ROOT:-}" && -d "${NJRH_CURRENT_FLOOR_ROOT}" ]] || return 1
  [[ -n "${NAV2_MAP_YAML:-}" && -f "${NAV2_MAP_YAML}" ]] || return 1
  [[ -n "${NAV2_LOCALIZER_MAP_YAML:-}" && -f "${NAV2_LOCALIZER_MAP_YAML}" ]] || return 1
  [[ -n "${NAV2_LOCALIZER_MAP_PNG:-}" && -f "${NAV2_LOCALIZER_MAP_PNG}" ]] || return 1
  [[ -n "${NJRH_FLOOR_POSES_YAML:-}" && -f "${NJRH_FLOOR_POSES_YAML}" ]] || return 1
}

resolve_floor_assets_if_needed() {
  local building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
  local floor_id="${2:-${NJRH_FLOOR_ID:-}}"
  if floor_asset_context_ready_for "${building_id}" "${floor_id}"; then
    echo "[runtime-overlay] reusing resolved floor asset context ${building_id}/${floor_id} root=${NJRH_CURRENT_FLOOR_ROOT}" >&2
    return 0
  fi
  resolve_floor_assets "${building_id}" "${floor_id}"
}

resolve_floor_assets() {
  local building_id="${1:-${NJRH_BUILDING_ID:-building_1}}"
  local floor_id="${2:-${NJRH_FLOOR_ID:-}}"

  [[ -n "${floor_id}" ]] || {
    echo "[runtime-overlay] floor_id is required" >&2
    return 1
  }

  local floor_root="${NJRH_RELEASE_ASSETS_DIR}/${building_id}/${floor_id}"
  local runtime_root="${floor_root}/current"
  if [[ -f "${runtime_root}/nav/nav_map.yaml" ]]; then
    validate_floor_assets "${runtime_root}"
  else
    runtime_root="${floor_root}"
    validate_floor_assets "${runtime_root}"
  fi

  export NJRH_BUILDING_ID="${building_id}"
  export NJRH_FLOOR_ID="${floor_id}"
  export NJRH_CURRENT_FLOOR_ROOT="${runtime_root}"
  export NAV2_MAP_YAML="${runtime_root}/nav/nav_map.yaml"
  export NAV2_LOCALIZER_MAP_YAML="${runtime_root}/localizer/localizer_params.yaml"
  export NAV2_LOCALIZER_MAP_PNG="${runtime_root}/localizer/localizer_map.png"
  export NAV2_KEEP_OUT_MASK_YAML="${runtime_root}/filters/keepout_mask.yaml"
  export NAV2_SPEED_MASK_YAML="${runtime_root}/filters/speed_mask.yaml"
  export NAV2_BINARY_MASK_YAML="${runtime_root}/filters/binary_mask.yaml"
  export NJRH_FLOOR_POSES_YAML="${runtime_root}/poses.yaml"

  local asset_report="${runtime_root}/reports/asset_report.json"
  local nav_map_name=""
  local nav_map_id=""
  if [[ -f "${asset_report}" ]]; then
    IFS=$'\t' read -r nav_map_name nav_map_id < <(python3 - "${asset_report}" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], "r", encoding="utf-8") as stream:
        data = json.load(stream)
    name = str(data.get("map_name") or data.get("display_name") or "").replace("\t", " ")
    map_id = str(data.get("map_id") or "").replace("\t", " ")
    print(f"{name}\t{map_id}")
except Exception:
    print("\t")
PY
)
  fi
  export NJRH_NAV_MAP_NAME="${nav_map_name}"
  export NJRH_NAV_MAP_ID="${nav_map_id}"
  export NJRH_MAP_ID="${nav_map_id:-${NJRH_MAP_ID:-}}"
  export NJRH_MAP_DISPLAY_NAME="${nav_map_name:-${NJRH_MAP_DISPLAY_NAME:-}}"
  export NJRH_MAP_CONTEXT_BUILDING_ID="${building_id}"
  export NJRH_MAP_CONTEXT_FLOOR_ID="${floor_id}"
  export NJRH_FLOOR_ASSET_CONTEXT_READY=1

  printf '[runtime-overlay] selected floor %s/%s\n' "${building_id}" "${floor_id}" >&2
  printf '[runtime-overlay] selected floor asset root=%s\n' "${runtime_root}" >&2
  if [[ -n "${NJRH_NAV_MAP_NAME}" ]]; then
    printf '[runtime-overlay] selected navigation map=%s map_id=%s\n' "${NJRH_NAV_MAP_NAME}" "${NJRH_NAV_MAP_ID}" >&2
  fi
  printf '[runtime-overlay] NAV2_MAP_YAML=%s\n' "${NAV2_MAP_YAML}" >&2
  printf '[runtime-overlay] NAV2_LOCALIZER_MAP_YAML=%s\n' "${NAV2_LOCALIZER_MAP_YAML}" >&2
}

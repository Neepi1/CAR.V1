#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXTRA_ARGS=()

if [[ -n "${CAR_PROJECT_ROOT:-}" ]]; then
  EXTRA_ARGS+=(--extra-root "${CAR_PROJECT_ROOT}")
fi

python3 "${ROOT_DIR}/scripts/scan_car_project.py" --root "${ROOT_DIR}" "${EXTRA_ARGS[@]}"
python3 "${ROOT_DIR}/scripts/tf_audit.py" --root "${ROOT_DIR}" "${EXTRA_ARGS[@]}"
python3 "${ROOT_DIR}/scripts/resolve_third_party.py" --root "${ROOT_DIR}" "${EXTRA_ARGS[@]}"

echo "Bootstrap baseline generated under ${ROOT_DIR}/reports"
echo "Local-first reuse policy preserved; network fetch remains explicit and opt-in."

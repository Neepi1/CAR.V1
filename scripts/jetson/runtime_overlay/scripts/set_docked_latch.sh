#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERIFY_SCRIPT="${SCRIPT_DIR}/verify_docked_navigation_undock_gate.sh"

usage() {
  cat <<'USAGE'
Usage: set_docked_latch.sh --confirm|--clear|--print [verify options]

Maintenance helper for the persistent docked latch. It never publishes velocity.
It calls the protected robot API through verify_docked_navigation_undock_gate.sh.

Common options passed through:
  --api-url URL
  --building-id ID
  --floor-id ID
  --map-id ID
  --pose-id ID

Set ROBOT_API_TOKEN when api_token is configured.
USAGE
}

ACTION=""
PASSTHROUGH=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm)
      ACTION="--confirm-latch"
      shift
      ;;
    --clear)
      ACTION="--clear-latch"
      shift
      ;;
    --print)
      ACTION="--print-latch"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      PASSTHROUGH+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${ACTION}" ]]; then
  echo "ERROR: one of --confirm, --clear, or --print is required" >&2
  usage >&2
  exit 2
fi

exec bash "${VERIFY_SCRIPT}" "${ACTION}" "${PASSTHROUGH[@]}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[verify-ranger-pass] deprecated: ranger_base now owns CAN output and motion-mode switching" >&2
echo "[verify-ranger-pass] forwarding to verify_ranger_chassis_core.sh (read-only)" >&2

# Legacy arguments selected a controller profile that no longer exists.  The
# chassis-core verifier is intentionally read-only and accepts no arguments.
exec bash "${SCRIPT_DIR}/verify_ranger_chassis_core.sh"

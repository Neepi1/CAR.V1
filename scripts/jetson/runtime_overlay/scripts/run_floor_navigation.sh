#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "${NJRH_ALLOW_TRANSIENT_NAVIGATION_OWNER:-${NJRH_ALLOW_MANUAL_NAV_OWNER:-}}" in
  1|true|TRUE|yes|YES|on|ON)
    ;;
  *)
    cat >&2 <<'EOF'
[runtime-overlay] ERROR: run_floor_navigation.sh is compatibility-only and is blocked by default.
[runtime-overlay] Production restart is owned by host systemd. Use:
[runtime-overlay]   sudo systemctl restart njrh-runtime.service
[runtime-overlay]
[runtime-overlay] Do not start a second navigation owner with docker exec/nohup. If the
[runtime-overlay] transient owner exits before runtime_ready=1, its cleanup trap can tear
[runtime-overlay] down Nav2/localization while readiness probes keep waiting.
[runtime-overlay]
[runtime-overlay] Debug-only override:
[runtime-overlay]   NJRH_ALLOW_TRANSIENT_NAVIGATION_OWNER=1 bash run_floor_navigation.sh <building_id> <floor_id>
EOF
    exit 2
    ;;
esac

echo "[runtime-overlay] run_floor_navigation.sh transient debug override accepted; delegating to resident navigation runtime" >&2
exec bash "${SCRIPT_DIR}/run_navigation_runtime_services.sh" "$@"

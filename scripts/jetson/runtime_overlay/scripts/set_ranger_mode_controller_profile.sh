#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PROFILE=""
PRINT_ONLY=0
DO_RESTART=0
CONFIG_FILE="${RANGER_MINI3_MODE_CONTROLLER_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/ranger_mini3_mode_controller.yaml}"

usage() {
  cat <<'EOF'
Usage:
  set_ranger_mode_controller_profile.sh --print
  set_ranger_mode_controller_profile.sh --profile official_passthrough [--print]

Legacy custom Ackermann profile switching has been removed. The Ranger Mini 3
mode controller is official-passthrough-only. Runtime restarts must use the full
systemd owner: sudo systemctl restart njrh-runtime.service
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --restart)
      DO_RESTART=1
      shift
      ;;
    --print)
      PRINT_ONLY=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-profile] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${DO_RESTART}" -eq 1 ]]; then
  echo "[ranger-profile] single-node restart is disabled; restart the full njrh-runtime.service owner" >&2
  exit 2
fi

if [[ -n "${PROFILE}" && "${PROFILE}" != "official_passthrough" ]]; then
  echo "[ranger-profile] invalid --profile '${PROFILE}'; legacy custom Ackermann profile was removed" >&2
  exit 2
fi

print_runtime_links() {
  echo "[ranger-profile] fixed_profile=official_passthrough"
  echo "[ranger-profile] legacy_custom_ackermann_removed=true"
  echo "[ranger-profile] config=${CONFIG_FILE}"
  echo "[ranger-profile] /cmd_vel_safe:"
  timeout 3 ros2 topic info -v /cmd_vel_safe 2>/dev/null || true
  echo "[ranger-profile] /cmd_vel:"
  timeout 3 ros2 topic info -v /cmd_vel 2>/dev/null || true
  echo "[ranger-profile] /ranger_mini3/mode_controller_shadow_cmd_vel:"
  timeout 3 ros2 topic info -v /ranger_mini3/mode_controller_shadow_cmd_vel 2>/dev/null || true
}

if [[ "${PRINT_ONLY}" -eq 1 || -n "${PROFILE}" ]]; then
  print_runtime_links
fi

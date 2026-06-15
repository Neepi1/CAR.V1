#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PROFILE=""
DO_RESTART=0
PRINT_ONLY=0
CONFIG_FILE="${RANGER_MINI3_MODE_CONTROLLER_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/ranger_mini3_mode_controller.yaml}"

usage() {
  cat <<'EOF'
Usage:
  set_ranger_mode_controller_profile.sh --profile official_passthrough|custom [--restart] [--print]
  set_ranger_mode_controller_profile.sh --print

Only ranger_mini3_mode_controller is restarted. Nav2, robot_safety, and ranger_base_node are not restarted.
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

current_profile() {
  python3 - "$CONFIG_FILE" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text(encoding="utf-8") if path.exists() else ""
match = re.search(r"^\s*mode_controller_profile:\s*([A-Za-z0-9_]+)\s*$", text, re.M)
print(match.group(1) if match else "missing")
PY
}

write_profile() {
  local profile="$1"
  python3 - "$CONFIG_FILE" "$profile" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
profile = sys.argv[2]
text = path.read_text(encoding="utf-8")
line = f"    mode_controller_profile: {profile}"
if re.search(r"^\s*mode_controller_profile:\s*[A-Za-z0-9_]+\s*$", text, re.M):
    text = re.sub(r"^\s*mode_controller_profile:\s*[A-Za-z0-9_]+\s*$", line, text, count=1, flags=re.M)
else:
    text = re.sub(r"(\n\s*cmd_vel_out_topic:\s*/cmd_vel\s*\n)", r"\1\n" + line + "\n", text, count=1)
path.write_text(text, encoding="utf-8")
PY
}

mode_controller_pids() {
  pgrep -f "ranger_mini3_mode_controller.*mode_controller_node|mode_controller_node.*ranger_mini3_mode_controller" || true
}

restart_mode_controller() {
  mapfile -t pids < <(mode_controller_pids)
  if [[ "${#pids[@]}" -gt 0 ]]; then
    echo "[ranger-profile] stopping ranger_mini3_mode_controller pids: ${pids[*]}"
    local pid
    for pid in "${pids[@]}"; do
      [[ "${pid}" == "$$" ]] && continue
      kill -TERM "${pid}" 2>/dev/null || true
    done
    for _ in {1..30}; do
      mapfile -t pids < <(mode_controller_pids)
      [[ "${#pids[@]}" -eq 0 ]] && break
      sleep 0.1
    done
  fi

  echo "[ranger-profile] starting ranger_mini3_mode_controller only"
  nohup bash "${SCRIPT_DIR}/run_ranger_mini3_mode_controller.sh" \
    >/tmp/njrh_ranger_mini3_mode_controller.log 2>&1 &
  sleep 1.0
}

print_runtime_links() {
  echo "[ranger-profile] current_profile=$(current_profile)"
  echo "[ranger-profile] config=${CONFIG_FILE}"
  echo "[ranger-profile] /cmd_vel_safe:"
  timeout 3 ros2 topic info -v /cmd_vel_safe 2>/dev/null || true
  echo "[ranger-profile] /cmd_vel:"
  timeout 3 ros2 topic info -v /cmd_vel 2>/dev/null || true
}

if [[ -n "${PROFILE}" ]]; then
  case "${PROFILE}" in
    official_passthrough|custom) ;;
    *)
      echo "[ranger-profile] invalid --profile '${PROFILE}'" >&2
      exit 2
      ;;
  esac
  [[ -f "${CONFIG_FILE}" ]] || {
    echo "[ranger-profile] missing config: ${CONFIG_FILE}" >&2
    exit 1
  }
  write_profile "${PROFILE}"
fi

if [[ "${DO_RESTART}" -eq 1 ]]; then
  restart_mode_controller
fi

if [[ "${PRINT_ONLY}" -eq 1 || -n "${PROFILE}" || "${DO_RESTART}" -eq 1 ]]; then
  print_runtime_links
fi

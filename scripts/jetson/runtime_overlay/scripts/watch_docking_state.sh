#!/usr/bin/env bash
set -euo pipefail

API_URL="${API_URL:-http://127.0.0.1:8080}"
INTERVAL_SEC="${INTERVAL_SEC:-1.0}"
PRINT_UNCHANGED_EVERY_SEC="${PRINT_UNCHANGED_EVERY_SEC:-5.0}"
LOG_FILE="${LOG_FILE:-}"
JSONL_FILE="${JSONL_FILE:-}"
NO_LOG=false
NO_JSONL=false

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/watch_docking_state.sh [options]

Options:
  --api-url URL          robot_api_server URL. Default: http://127.0.0.1:8080
  --interval-sec SEC    Poll interval. Default: 1.0
  --print-every-sec SEC Reprint unchanged state this often. Default: 5.0
  --log-file FILE       Human-readable line log. Default: reports/docking_state_watch/*.log
  --jsonl-file FILE     Full /api/v1/docking/state snapshots. Default: same basename .jsonl
  --no-log              Do not write the human-readable line log.
  --no-jsonl            Do not write raw JSONL snapshots.
  -h, --help            Show this help.

Examples:
  bash scripts/jetson/runtime_overlay/scripts/watch_docking_state.sh
  API_URL=http://127.0.0.1:8080 INTERVAL_SEC=0.5 bash scripts/jetson/runtime_overlay/scripts/watch_docking_state.sh
  bash scripts/jetson/runtime_overlay/scripts/watch_docking_state.sh --log-file /tmp/dock_watch.log
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --api-url)
      API_URL="$2"
      shift 2
      ;;
    --interval-sec)
      INTERVAL_SEC="$2"
      shift 2
      ;;
    --print-every-sec)
      PRINT_UNCHANGED_EVERY_SEC="$2"
      shift 2
      ;;
    --log-file)
      LOG_FILE="$2"
      shift 2
      ;;
    --jsonl-file)
      JSONL_FILE="$2"
      shift 2
      ;;
    --no-log)
      NO_LOG=true
      shift
      ;;
    --no-jsonl)
      NO_JSONL=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

if [[ "${NO_LOG}" != "true" || "${NO_JSONL}" != "true" ]]; then
  if [[ -z "${LOG_FILE}" ]]; then
    DEFAULT_LOG_DIR="${WORKSPACE_ROOT}/reports/docking_state_watch"
    if ! mkdir -p "${DEFAULT_LOG_DIR}" 2>/dev/null; then
      DEFAULT_LOG_DIR="/tmp/docking_state_watch"
      mkdir -p "${DEFAULT_LOG_DIR}"
    fi
    LOG_FILE="${DEFAULT_LOG_DIR}/${STAMP}_docking_state_watch.log"
  fi
  if [[ "${NO_LOG}" != "true" ]]; then
    LOG_DIR="$(dirname -- "${LOG_FILE}")"
    if ! mkdir -p "${LOG_DIR}" 2>/dev/null || ! touch "${LOG_FILE}" 2>/dev/null; then
      LOG_FILE="/tmp/docking_state_watch/${STAMP}_docking_state_watch.log"
      mkdir -p "$(dirname -- "${LOG_FILE}")"
      touch "${LOG_FILE}"
    fi
  fi
  if [[ -z "${JSONL_FILE}" ]]; then
    JSONL_FILE="${LOG_FILE%.log}.jsonl"
  fi
  if [[ "${NO_JSONL}" != "true" ]]; then
    JSONL_DIR="$(dirname -- "${JSONL_FILE}")"
    if ! mkdir -p "${JSONL_DIR}" 2>/dev/null || ! touch "${JSONL_FILE}" 2>/dev/null; then
      JSONL_FILE="/tmp/docking_state_watch/${STAMP}_docking_state_watch.jsonl"
      mkdir -p "$(dirname -- "${JSONL_FILE}")"
      touch "${JSONL_FILE}"
    fi
  fi
fi

if [[ "${NO_LOG}" == "true" ]]; then
  LOG_FILE=""
else
  echo "[watch-docking-state] line log: ${LOG_FILE}" >&2
fi
if [[ "${NO_JSONL}" == "true" ]]; then
  JSONL_FILE=""
else
  echo "[watch-docking-state] jsonl: ${JSONL_FILE}" >&2
fi

python3 - "$API_URL" "$INTERVAL_SEC" "$PRINT_UNCHANGED_EVERY_SEC" "$LOG_FILE" "$JSONL_FILE" <<'PY'
import json
import math
import sys
import time
import urllib.error
import urllib.request

api_url = sys.argv[1].rstrip("/")
interval_sec = float(sys.argv[2])
print_unchanged_every_sec = float(sys.argv[3])
log_file = sys.argv[4]
jsonl_file = sys.argv[5]

last_line = None
last_print = 0.0
line_log = open(log_file, "a", buffering=1) if log_file else None
jsonl_log = open(jsonl_file, "a", buffering=1) if jsonl_file else None

def write_line_log(line):
    if line_log:
        line_log.write(line + "\n")

def write_jsonl(snapshot):
    if jsonl_log:
        jsonl_log.write(json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")) + "\n")

def val(data, key, default=""):
    value = data.get(key, default)
    if value is None:
        return default
    return value

def fmt_num(value, digits=3):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "na"
    if not math.isfinite(number):
        return "na"
    return f"{number:.{digits}f}"

def trim(text, width=80):
    text = str(text or "").replace("\n", " ")
    return text if len(text) <= width else text[: width - 3] + "..."

def fetch_json(path):
    with urllib.request.urlopen(f"{api_url}{path}", timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))

while True:
    now = time.time()
    stamp = time.strftime("%H:%M:%S")
    observed_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now))
    try:
        state = fetch_json("/api/v1/docking/state")
        write_jsonl({"observed_at": observed_at, "api_url": api_url, "docking_state": state})
        docking = state.get("docking") or {}
        status = trim(
            docking.get("docking_status_after_request")
            or docking.get("last_status")
            or "",
            64,
        )
        detail = trim(docking.get("detail") or docking.get("last_error_detail") or "", 88)
        line = (
            f"{stamp} "
            f"id={val(docking, 'id')} "
            f"state={val(docking, 'state')} "
            f"phase={val(docking, 'phase')} "
            f"code={val(docking, 'failure_code')} "
            f"nav={val(docking, 'nav_goal_sent')}/{val(docking, 'nav_goal_succeeded')} "
            f"handoff={val(docking, 'dock_staging_handoff_ready')} "
            f"fwd={fmt_num(val(docking, 'predock_forward_m'))} "
            f"lat={fmt_num(val(docking, 'predock_lateral_m'))} "
            f"|lat|={fmt_num(val(docking, 'predock_lateral_abs_m'))} "
            f"yaw={fmt_num(val(docking, 'base_yaw_error'))} "
            f"yawOK={val(docking, 'predock_yaw_aligned')} "
            f"latOK={val(docking, 'predock_lateral_aligned')} "
            f"fine={val(docking, 'fine_entry_checked')}/{val(docking, 'fine_entry_ok')} "
            f"fineLat={fmt_num(val(docking, 'fine_entry_lateral_m'))} "
            f"fineYaw={fmt_num(val(docking, 'fine_entry_base_yaw_error_rad'))} "
            f"gs2=\"{status}\" "
            f"detail=\"{detail}\""
        )
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
        line = f"{stamp} monitor_error={type(exc).__name__}: {exc}"
        write_jsonl({"observed_at": observed_at, "api_url": api_url, "error": line})

    if line != last_line or now - last_print >= print_unchanged_every_sec:
        print(line, flush=True)
        write_line_log(line)
        last_line = line
        last_print = now
    time.sleep(interval_sec)
PY

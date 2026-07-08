#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

API_URL="${API_URL:-http://127.0.0.1:8080}"
CYCLES="1"
TARGET_A="675235"
TARGET_B="512355"
START_TARGET=""
TIMEOUT_SEC="180"
INTER_LEG_READY_TIMEOUT_SEC="60"
MAX_ONLINE_XY_M="0.20"
MAX_ONLINE_YAW_RAD="0.08"
MAX_MAP_ODOM_TRANSLATION_M="0.30"
MAX_MAP_BASE_TRANSLATION_M="0.50"
MAX_YAW_DEG="2.0"
GOAL_COMPLETION_POLICY="pose_required"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/navigation_pingpong_guarded"

usage() {
  cat <<'EOF'
Usage: run_navigation_delivery_pingpong_guarded.sh [options]

Runs guarded back-and-forth navigation between two saved delivery poses. Each
leg uses the normal robot_api_server/Nav2 path, then runs post-goal
relocalization. If the correction exceeds configured thresholds, the script
stops and does not send the next target.

Options:
  --target-a ID                    First saved delivery suffix/id. Default: 675235.
  --target-b ID                    Second saved delivery suffix/id. Default: 512355.
  --cycles N                       Number of target-a->target-b pairs. Default: 1.
  --start-target ID                First target. Default: target-a.
  --timeout-sec SEC                Per-leg navigation timeout. Default: 180.
  --inter-leg-ready-timeout-sec SEC Wait for API/Nav2 goal readiness before
                                    each leg. Default: 60.
  --max-online-xy-m M              Stop if API final XY error exceeds this. Default: 0.20.
  --max-online-yaw-rad RAD         Stop if API final yaw error exceeds this. Default: 0.08.
  --max-map-odom-translation-m M   Stop if map->odom correction exceeds this. Default: 0.30.
  --max-map-base-translation-m M   Stop if map->base_link jump exceeds this. Default: 0.50.
  --max-yaw-deg DEG                Stop if correction yaw exceeds this. Default: 2.0.
  --goal-completion-policy POLICY  API goal policy: pose_required or position_only.
                                    Default: pose_required.
  --output-root DIR                Report root. Default: reports/navigation_pingpong_guarded.
  --api-url URL                    robot_api_server URL. Default: http://127.0.0.1:8080.

This script does not relocalize before each navigation leg. It only relocalizes
after each leg to measure and reset map alignment before deciding whether it is
safe to continue.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --target-a)
      TARGET_A="${2:-}"
      shift 2
      ;;
    --target-b)
      TARGET_B="${2:-}"
      shift 2
      ;;
    --cycles)
      CYCLES="${2:-}"
      shift 2
      ;;
    --start-target)
      START_TARGET="${2:-}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --inter-leg-ready-timeout-sec)
      INTER_LEG_READY_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --max-online-xy-m)
      MAX_ONLINE_XY_M="${2:-}"
      shift 2
      ;;
    --max-online-yaw-rad)
      MAX_ONLINE_YAW_RAD="${2:-}"
      shift 2
      ;;
    --max-map-odom-translation-m)
      MAX_MAP_ODOM_TRANSLATION_M="${2:-}"
      shift 2
      ;;
    --max-map-base-translation-m)
      MAX_MAP_BASE_TRANSLATION_M="${2:-}"
      shift 2
      ;;
    --max-yaw-deg)
      MAX_YAW_DEG="${2:-}"
      shift 2
      ;;
    --goal-completion-policy)
      GOAL_COMPLETION_POLICY="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[nav-pingpong-guarded] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

normalize_delivery_id() {
  local value="$1"
  value="${value#delivery_}"
  [[ -n "${value}" ]] || return 1
  [[ "${value}" =~ ^[A-Za-z0-9_.-]+$ ]] || return 1
  printf '%s\n' "${value}"
}

pose_id_for_target() {
  printf 'delivery_%s\n' "$1"
}

TARGET_A="$(normalize_delivery_id "${TARGET_A}")" || {
  echo "[nav-pingpong-guarded] --target-a must be a non-empty safe delivery id" >&2
  exit 2
}
TARGET_B="$(normalize_delivery_id "${TARGET_B}")" || {
  echo "[nav-pingpong-guarded] --target-b must be a non-empty safe delivery id" >&2
  exit 2
}
[[ "${TARGET_A}" != "${TARGET_B}" ]] || {
  echo "[nav-pingpong-guarded] --target-a and --target-b must differ" >&2
  exit 2
}
if [[ -z "${START_TARGET}" ]]; then
  START_TARGET="${TARGET_A}"
fi
START_TARGET="$(normalize_delivery_id "${START_TARGET}")" || {
  echo "[nav-pingpong-guarded] --start-target must be a non-empty safe delivery id" >&2
  exit 2
}
if [[ "${START_TARGET}" != "${TARGET_A}" && "${START_TARGET}" != "${TARGET_B}" ]]; then
  echo "[nav-pingpong-guarded] --start-target must match --target-a or --target-b" >&2
  exit 2
fi

case "${GOAL_COMPLETION_POLICY}" in
  pose_required|position_only) ;;
  *)
    echo "[nav-pingpong-guarded] --goal-completion-policy must be pose_required or position_only" >&2
    exit 2
    ;;
esac

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_delivery_pingpong_guarded"
mkdir -p "${OUT_DIR}"

echo "[nav-pingpong-guarded] report: ${OUT_DIR}"

python3 - \
  "${CYCLES}" \
  "${TIMEOUT_SEC}" \
  "${INTER_LEG_READY_TIMEOUT_SEC}" \
  "${MAX_ONLINE_XY_M}" \
  "${MAX_ONLINE_YAW_RAD}" \
  "${MAX_MAP_ODOM_TRANSLATION_M}" \
  "${MAX_MAP_BASE_TRANSLATION_M}" \
  "${MAX_YAW_DEG}" <<'PY'
import math
import sys

cycles = int(sys.argv[1])
timeout = float(sys.argv[2])
inter_leg_ready_timeout = float(sys.argv[3])
max_online_xy = float(sys.argv[4])
max_online_yaw = float(sys.argv[5])
max_map_odom = float(sys.argv[6])
max_map_base = float(sys.argv[7])
max_yaw_deg = float(sys.argv[8])
if cycles < 1:
    raise SystemExit("cycles must be >= 1")
if timeout <= 0 or inter_leg_ready_timeout <= 0:
    raise SystemExit("timeout must be > 0")
if max_online_xy <= 0 or max_online_yaw <= 0 or max_map_odom <= 0 or max_map_base <= 0 or max_yaw_deg <= 0:
    raise SystemExit("guard thresholds must be > 0")
if not all(math.isfinite(v) for v in (timeout, inter_leg_ready_timeout, max_online_xy, max_online_yaw, max_map_odom, max_map_base, max_yaw_deg)):
    raise SystemExit("numeric options must be finite")
PY

VALIDATOR="${SCRIPT_DIR}/validate_ekf_ab_report.py"
[[ -f "${VALIDATOR}" ]] || {
  echo "[nav-pingpong-guarded] missing EKF A/B validator: ${VALIDATOR}" >&2
  exit 2
}

{
  echo "# Navigation Delivery Ping-Pong Guarded"
  echo "- timestamp_utc: ${timestamp}"
  echo "- target_a: delivery_${TARGET_A}"
  echo "- target_b: delivery_${TARGET_B}"
  echo "- cycles: ${CYCLES}"
  echo "- start_target: delivery_${START_TARGET}"
  echo "- timeout_sec: ${TIMEOUT_SEC}"
  echo "- inter_leg_ready_timeout_sec: ${INTER_LEG_READY_TIMEOUT_SEC}"
  echo "- max_online_xy_m: ${MAX_ONLINE_XY_M}"
  echo "- max_online_yaw_rad: ${MAX_ONLINE_YAW_RAD}"
  echo "- max_map_odom_translation_m: ${MAX_MAP_ODOM_TRANSLATION_M}"
  echo "- max_map_base_translation_m: ${MAX_MAP_BASE_TRANSLATION_M}"
  echo "- max_yaw_deg: ${MAX_YAW_DEG}"
  echo "- goal_completion_policy: ${GOAL_COMPLETION_POLICY}"
  echo
  echo "| leg | target | report_dir | nav_rc | validator_rc | online_xy_m | online_yaw_rad | map_odom_translation_m | map_base_translation_m | map_odom_yaw_deg | map_base_yaw_deg | decision | reasons |"
  echo "|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|"
} >"${OUT_DIR}/summary.md"

target_for_leg() {
  local idx="$1"
  if [[ "${START_TARGET}" == "${TARGET_A}" ]]; then
    if (( idx % 2 == 1 )); then
      printf '%s\n' "${TARGET_A}"
    else
      printf '%s\n' "${TARGET_B}"
    fi
  else
    if (( idx % 2 == 1 )); then
      printf '%s\n' "${TARGET_B}"
    else
      printf '%s\n' "${TARGET_A}"
    fi
  fi
}

wait_for_api_navigation_ready() {
  local label="$1"
  local ready_json="${OUT_DIR}/api_ready_${label}.json"
  local deadline=$((SECONDS + INTER_LEG_READY_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if python3 - "${API_URL}" >"${ready_json}.tmp" <<'PY'
import json
import sys
import urllib.request

api_url = sys.argv[1].rstrip("/")

def api_get(path):
    with urllib.request.urlopen(api_url + path, timeout=3.0) as response:
        return json.loads(response.read().decode("utf-8", errors="replace"))

status = api_get("/api/v1/status")
navigation_state = api_get("/api/v1/navigation/state")
navigation = status.get("navigation") if isinstance(status, dict) else {}
localization = status.get("localization") if isinstance(status, dict) else {}
post_settle = navigation.get("post_relocalization_settle") if isinstance(navigation, dict) else {}
goal = navigation.get("goal") if isinstance(navigation, dict) else {}
active_goal = isinstance(goal, dict) and str(goal.get("state") or "").lower() == "running"
payload = {
    "ok": bool(status.get("ok")) if isinstance(status, dict) else False,
    "healthy": bool(status.get("healthy")) if isinstance(status, dict) else False,
    "top_state": status.get("state") if isinstance(status, dict) else None,
    "navigation_active": navigation.get("active") if isinstance(navigation, dict) else None,
    "navigation_state": navigation.get("state") if isinstance(navigation, dict) else None,
    "navigation_endpoint_state": navigation_state.get("state") if isinstance(navigation_state, dict) else None,
    "safe_for_goal_start": localization.get("safe_for_goal_start") if isinstance(localization, dict) else None,
    "post_relocalization_settle_complete": post_settle.get("complete") if isinstance(post_settle, dict) else None,
    "active_navigation_goal": active_goal,
}
ready = (
    payload["ok"] is True
    and payload["healthy"] is True
    and payload["top_state"] == "running"
    and payload["navigation_active"] is True
    and payload["navigation_state"] == "running"
    and payload["navigation_endpoint_state"] == "running"
    and payload["safe_for_goal_start"] is True
    and payload["post_relocalization_settle_complete"] is True
    and not active_goal
)
payload["ready"] = ready
print(json.dumps(payload, indent=2, sort_keys=True))
raise SystemExit(0 if ready else 1)
PY
    then
      mv "${ready_json}.tmp" "${ready_json}"
      return 0
    fi
    sleep 1
  done
  mv "${ready_json}.tmp" "${ready_json}" 2>/dev/null || true
  return 1
}

parse_validator_result() {
  local result_json="$1"
  local result_stderr="$2"
  local validator_rc="$3"
  python3 - \
    "${result_json}" \
    "${result_stderr}" \
    "${validator_rc}" <<'PY'
import json
import sys
from pathlib import Path

result_path = Path(sys.argv[1])
stderr_path = Path(sys.argv[2])
validator_rc = int(sys.argv[3])

def metric(metrics, key):
    value = metrics.get(key)
    return "nan" if value is None else str(value)

try:
    result = json.loads(result_path.read_text(encoding="utf-8"))
except Exception:
    stderr_text = stderr_path.read_text(encoding="utf-8").strip() if stderr_path.exists() else ""
    reason = stderr_text or "validator_json_missing_or_invalid"
    print(f"nan\tnan\tnan\tnan\tnan\tnan\tstop_validator_error\t{reason}")
    raise SystemExit(0)

metrics = result.get("metrics") or {}
accepted = result.get("accepted") is True and validator_rc == 0
if accepted:
    decision = "continue"
elif validator_rc == 2:
    decision = "stop_validator_input_error"
else:
    decision = "stop_ekf_ab_rejected"
reasons = ";".join(str(r) for r in (result.get("reasons") or []))
reasons = reasons.replace("|", "/").replace("\n", " ")[:500]
print(
    "\t".join(
        [
            metric(metrics, "final_distance_m"),
            metric(metrics, "final_yaw_error_rad"),
            metric(metrics, "map_odom_translation_m"),
            metric(metrics, "map_base_link_translation_m"),
            metric(metrics, "map_odom_dyaw_deg"),
            metric(metrics, "map_base_link_dyaw_deg"),
            decision,
            reasons,
        ]
    )
)
PY
}

legs=$((CYCLES * 2))
for ((leg=1; leg<=legs; leg++)); do
  target="$(target_for_leg "${leg}")"
  label="pingpong_leg${leg}_delivery_${target}"
  echo "[nav-pingpong-guarded] leg=${leg} target=delivery_${target}"

  if ! wait_for_api_navigation_ready "leg${leg}_before_delivery_${target}"; then
    echo "| ${leg} | delivery_${target} | api_not_ready | 21 |  |  |  |  |  |  |  | stop_api_not_ready | api_navigation_ready_timeout |" >>"${OUT_DIR}/summary.md"
    echo "[nav-pingpong-guarded] stop: API navigation was not ready before delivery_${target}" >&2
    exit 21
  fi

  set +e
  bash "${SCRIPT_DIR}/run_navigation_pose_error_test.sh" \
    --pose-id "$(pose_id_for_target "${target}")" \
    --timeout-sec "${TIMEOUT_SEC}" \
    --goal-completion-policy "${GOAL_COMPLETION_POLICY}" \
    --label "${label}"
  nav_rc=$?
  set -e

  report_dir="$(find "${PROJECT_ROOT}/reports/navigation_pose_error_test" -mindepth 1 -maxdepth 1 -type d -name "*_${label}" | sort | tail -n 1 || true)"
  if [[ -z "${report_dir}" ]]; then
    echo "| ${leg} | delivery_${target} | missing | ${nav_rc} |  |  |  |  |  |  |  | stop_missing_report | report_not_found |" >>"${OUT_DIR}/summary.md"
    echo "[nav-pingpong-guarded] stop: missing report for ${label}" >&2
    exit 20
  fi

  validator_json="${OUT_DIR}/leg${leg}_delivery_${target}_ekf_ab_validation.json"
  validator_stderr="${OUT_DIR}/leg${leg}_delivery_${target}_ekf_ab_validation.stderr"
  set +e
  python3 "${VALIDATOR}" \
    --pose-report "${report_dir}" \
    --max-online-xy-m "${MAX_ONLINE_XY_M}" \
    --max-online-yaw-rad "${MAX_ONLINE_YAW_RAD}" \
    --max-map-odom-translation-m "${MAX_MAP_ODOM_TRANSLATION_M}" \
    --max-map-base-translation-m "${MAX_MAP_BASE_TRANSLATION_M}" \
    --max-correction-yaw-deg "${MAX_YAW_DEG}" \
    --json >"${validator_json}" 2>"${validator_stderr}"
  guard_rc=$?
  guard_line="$(parse_validator_result "${validator_json}" "${validator_stderr}" "${guard_rc}")"
  set -e
  IFS=$'\t' read -r online_xy online_yaw map_odom_t map_base_t map_odom_yaw map_base_yaw decision reasons <<<"${guard_line}"
  echo "| ${leg} | delivery_${target} | ${report_dir#${PROJECT_ROOT}/} | ${nav_rc} | ${guard_rc} | ${online_xy} | ${online_yaw} | ${map_odom_t} | ${map_base_t} | ${map_odom_yaw} | ${map_base_yaw} | ${decision} | ${reasons} |" >>"${OUT_DIR}/summary.md"

  if [[ "${nav_rc}" != "0" ]]; then
    echo "[nav-pingpong-guarded] stop: navigation rc=${nav_rc}" >&2
    exit "${nav_rc}"
  fi
  if [[ "${guard_rc}" != "0" ]]; then
    echo "[nav-pingpong-guarded] stop: guard exceeded after delivery_${target}" >&2
    echo "[nav-pingpong-guarded] summary: ${OUT_DIR}/summary.md"
    exit 10
  fi
done

echo "[nav-pingpong-guarded] completed all legs"
echo "[nav-pingpong-guarded] summary: ${OUT_DIR}/summary.md"

#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

CYCLES="1"
START_TARGET="675235"
TIMEOUT_SEC="180"
MAX_MAP_ODOM_TRANSLATION_M="0.30"
MAX_MAP_BASE_TRANSLATION_M="0.50"
MAX_YAW_DEG="2.0"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/navigation_pingpong_guarded"

usage() {
  cat <<'EOF'
Usage: run_navigation_delivery_pingpong_guarded.sh [options]

Runs guarded back-and-forth navigation between delivery_675235 and
delivery_512355. Each leg uses the normal robot_api_server/Nav2 path, then runs
post-goal relocalization. If the correction exceeds configured thresholds, the
script stops and does not send the next target.

Options:
  --cycles N                       Number of 675235->512355 pairs. Default: 1.
  --start-target 675235|512355     First target. Default: 675235.
  --timeout-sec SEC                Per-leg navigation timeout. Default: 180.
  --max-map-odom-translation-m M   Stop if map->odom correction exceeds this. Default: 0.30.
  --max-map-base-translation-m M   Stop if map->base_link jump exceeds this. Default: 0.50.
  --max-yaw-deg DEG                Stop if correction yaw exceeds this. Default: 2.0.
  --output-root DIR                Report root. Default: reports/navigation_pingpong_guarded.

This script does not relocalize before each navigation leg. It only relocalizes
after each leg to measure and reset map alignment before deciding whether it is
safe to continue.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
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
    --output-root)
      OUTPUT_ROOT="${2:-}"
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

case "${START_TARGET}" in
  675235|delivery_675235) START_TARGET="675235" ;;
  512355|delivery_512355) START_TARGET="512355" ;;
  *)
    echo "[nav-pingpong-guarded] --start-target must be 675235 or 512355" >&2
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
  "${MAX_MAP_ODOM_TRANSLATION_M}" \
  "${MAX_MAP_BASE_TRANSLATION_M}" \
  "${MAX_YAW_DEG}" <<'PY'
import math
import sys

cycles = int(sys.argv[1])
timeout = float(sys.argv[2])
max_map_odom = float(sys.argv[3])
max_map_base = float(sys.argv[4])
max_yaw_deg = float(sys.argv[5])
if cycles < 1:
    raise SystemExit("cycles must be >= 1")
if timeout <= 0:
    raise SystemExit("timeout must be > 0")
if max_map_odom <= 0 or max_map_base <= 0 or max_yaw_deg <= 0:
    raise SystemExit("guard thresholds must be > 0")
if not all(math.isfinite(v) for v in (timeout, max_map_odom, max_map_base, max_yaw_deg)):
    raise SystemExit("numeric options must be finite")
PY

{
  echo "# Navigation Delivery Ping-Pong Guarded"
  echo "- timestamp_utc: ${timestamp}"
  echo "- cycles: ${CYCLES}"
  echo "- start_target: ${START_TARGET}"
  echo "- timeout_sec: ${TIMEOUT_SEC}"
  echo "- max_map_odom_translation_m: ${MAX_MAP_ODOM_TRANSLATION_M}"
  echo "- max_map_base_translation_m: ${MAX_MAP_BASE_TRANSLATION_M}"
  echo "- max_yaw_deg: ${MAX_YAW_DEG}"
  echo
  echo "| leg | target | report_dir | nav_rc | map_odom_translation_m | map_base_translation_m | map_odom_yaw_deg | map_base_yaw_deg | decision |"
  echo "|---:|---|---|---:|---:|---:|---:|---:|---|"
} >"${OUT_DIR}/summary.md"

target_for_leg() {
  local idx="$1"
  if [[ "${START_TARGET}" == "675235" ]]; then
    if (( idx % 2 == 1 )); then
      printf '675235\n'
    else
      printf '512355\n'
    fi
  else
    if (( idx % 2 == 1 )); then
      printf '512355\n'
    else
      printf '675235\n'
    fi
  fi
}

parse_guard() {
  local report_dir="$1"
  python3 - \
    "${report_dir}" \
    "${MAX_MAP_ODOM_TRANSLATION_M}" \
    "${MAX_MAP_BASE_TRANSLATION_M}" \
    "${MAX_YAW_DEG}" <<'PY'
import json
import math
import sys
from pathlib import Path

report = Path(sys.argv[1])
max_map_odom = float(sys.argv[2])
max_map_base = float(sys.argv[3])
max_yaw_deg = float(sys.argv[4])
metrics_path = report / "post_relocalize_compare" / "correction_metrics.json"
if not metrics_path.exists():
    print("nan\tnan\tnan\tnan\tstop_missing_metrics")
    raise SystemExit(10)
metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
map_odom = metrics.get("map_odom_delta") or {}
map_base = metrics.get("map_base_link_delta") or {}
map_odom_translation = float(map_odom.get("translation_m", float("nan")))
map_base_translation = float(map_base.get("translation_m", float("nan")))
map_odom_yaw = abs(float(map_odom.get("dyaw_deg", float("nan"))))
map_base_yaw = abs(float(map_base.get("dyaw_deg", float("nan"))))
bad = (
    not all(math.isfinite(v) for v in (map_odom_translation, map_base_translation, map_odom_yaw, map_base_yaw))
    or map_odom_translation > max_map_odom
    or map_base_translation > max_map_base
    or map_odom_yaw > max_yaw_deg
    or map_base_yaw > max_yaw_deg
)
decision = "stop_guard_exceeded" if bad else "continue"
print(f"{map_odom_translation:.6f}\t{map_base_translation:.6f}\t{map_odom_yaw:.6f}\t{map_base_yaw:.6f}\t{decision}")
raise SystemExit(10 if bad else 0)
PY
}

legs=$((CYCLES * 2))
for ((leg=1; leg<=legs; leg++)); do
  target="$(target_for_leg "${leg}")"
  label="pingpong_leg${leg}_delivery_${target}"
  echo "[nav-pingpong-guarded] leg=${leg} target=delivery_${target}"

  set +e
  if [[ "${target}" == "675235" ]]; then
    bash "${SCRIPT_DIR}/run_navigation_delivery_675235_pose_error_test.sh" \
      --timeout-sec "${TIMEOUT_SEC}" \
      --label "${label}"
  else
    bash "${SCRIPT_DIR}/run_navigation_delivery_512355_pose_error_test.sh" \
      --timeout-sec "${TIMEOUT_SEC}" \
      --label "${label}"
  fi
  nav_rc=$?
  set -e

  report_dir="$(find "${PROJECT_ROOT}/reports/navigation_pose_error_test" -mindepth 1 -maxdepth 1 -type d -name "*_${label}" | sort | tail -n 1 || true)"
  if [[ -z "${report_dir}" ]]; then
    echo "| ${leg} | delivery_${target} | missing | ${nav_rc} |  |  |  |  | stop_missing_report |" >>"${OUT_DIR}/summary.md"
    echo "[nav-pingpong-guarded] stop: missing report for ${label}" >&2
    exit 20
  fi

  set +e
  guard_line="$(parse_guard "${report_dir}")"
  guard_rc=$?
  set -e
  IFS=$'\t' read -r map_odom_t map_base_t map_odom_yaw map_base_yaw decision <<<"${guard_line}"
  echo "| ${leg} | delivery_${target} | ${report_dir#${PROJECT_ROOT}/} | ${nav_rc} | ${map_odom_t} | ${map_base_t} | ${map_odom_yaw} | ${map_base_yaw} | ${decision} |" >>"${OUT_DIR}/summary.md"

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

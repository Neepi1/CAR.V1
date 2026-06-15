#!/usr/bin/env bash
set -Eeuo pipefail

# Read-only verifier for the post-relocalization settle barrier. It never sends
# navigation goals or velocity commands.

REASON="manual"
DURATION_SEC=30
WATCH=false
SIMULATE_STATUS_ONLY=false
EXPECT_PASS=false
EXPECT_FAIL=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --reason)
      REASON="${2:-manual}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-30}"
      shift 2
      ;;
    --watch)
      WATCH=true
      shift
      ;;
    --simulate-status-only)
      SIMULATE_STATUS_ONLY=true
      shift
      ;;
    --expect-pass)
      EXPECT_PASS=true
      shift
      ;;
    --expect-fail)
      EXPECT_FAIL=true
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage: verify_post_relocalization_settle_barrier.sh [options]

Options:
  --reason post_undock|after_predock|manual
  --duration-sec N
  --watch
  --simulate-status-only
  --expect-pass
  --expect-fail
EOF
      exit 0
      ;;
    *)
      echo "[post-reloc-settle] unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
cd "${WORKSPACE_ROOT}"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
set -u

passes=()
warns=()
failures=()

pass() { passes+=("$1"); echo "[post-reloc-settle] PASS $1"; }
warn() { warns+=("$1"); echo "[post-reloc-settle] WARN $1" >&2; }
fail() { failures+=("$1"); echo "[post-reloc-settle] FAIL $1" >&2; }

json_field() {
  local text="$1"
  local key="$2"
  JSON_TEXT="${text}" python3 - "$key" <<'PY'
import json
import os
import sys
key = sys.argv[1]
try:
    text = os.environ.get("JSON_TEXT", "")
    text = "\n".join(line for line in text.splitlines() if line.strip() != "---").strip()
    if "{" in text and "}" in text:
        text = text[text.find("{"):text.rfind("}") + 1]
    data = json.loads(text)
except Exception:
    sys.exit(0)
value = data
for part in key.split("."):
    if not isinstance(value, dict) or part not in value:
        sys.exit(0)
    value = value[part]
if isinstance(value, bool):
    print("true" if value else "false")
elif value is None:
    print("null")
else:
    print(value)
PY
}

bridge_status_once() {
  timeout 5 ros2 topic echo --once --field data /localization/bridge_status 2>/dev/null || true
}

api_status_once() {
  timeout 5 curl -fsS http://127.0.0.1:8080/api/v1/status 2>/dev/null || true
}

topic_has_message() {
  local topic="$1"
  timeout 5 ros2 topic echo --once "${topic}" >/dev/null 2>&1
}

tf_echo_ok() {
  local parent="$1"
  local child="$2"
  local output=""
  output="$(timeout 4 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
  grep -Eq 'At time|Translation:|Rotation:' <<<"${output}"
}

recent_message_filter_drop_count() {
  local logs=""
  if command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' | grep -qx 'NJRH-car'; then
    logs="$(docker logs --since "${DURATION_SEC}s" NJRH-car 2>&1 || true)"
  else
    logs="$(journalctl --no-pager --since "-${DURATION_SEC} seconds" 2>/dev/null || true)"
  fi
  grep -Eci 'Message Filter dropping|earlier than all (the )?data|tf future|future extrapolation' <<<"${logs}" || true
}

status="$(bridge_status_once)"
if [[ -z "${status}" ]]; then
  fail "/localization/bridge_status unavailable"
else
  pass "/localization/bridge_status available"
  for field in \
    last_explicit_relocalization_accept_time \
    last_explicit_relocalization_source \
    last_explicit_relocalization_sequence \
    last_accepted_correction_translation_m \
    last_accepted_correction_yaw_rad \
    map_to_odom_age_ms \
    map_odom_publish_loop_hz \
    map_odom_publish_gap_ms \
    map_odom_publish_gap_max_ms \
    map_odom_publish_callback_duration_us \
    map_odom_latest_accepted_sequence \
    map_odom_last_published_sequence \
    map_odom_latest_source \
    map_odom_state_valid \
    map_odom_correction_paused \
    map_odom_frozen_due_to_pause \
    map_odom_publish_missed_count \
    publisher_decoupled_from_correction \
    map_to_odom_publisher_owner \
    localization_settle_required \
    localization_settle_in_progress \
    localization_settle_start_time \
    localization_settle_reason \
    localization_settle_min_ms \
    localization_settle_complete \
    localization_settle_failure_reason; do
    if [[ -n "$(json_field "${status}" "${field}")" ]]; then
      pass "bridge_status has ${field}"
    else
      fail "bridge_status missing ${field}"
    fi
  done

  owner="$(json_field "${status}" map_to_odom_publisher_owner)"
  [[ "${owner}" == "robot_localization_bridge" ]] && pass "map->odom owner is robot_localization_bridge" || fail "map->odom owner is ${owner:-missing}"
  has_map="$(json_field "${status}" has_map_to_odom)"
  [[ "${has_map}" == "true" ]] && pass "bridge has_map_to_odom=true" || fail "bridge has_map_to_odom=${has_map:-missing}"
  decoupled="$(json_field "${status}" publisher_decoupled_from_correction)"
  [[ "${decoupled}" == "true" ]] && pass "bridge publisher is decoupled from correction callbacks" || fail "bridge publisher_decoupled_from_correction=${decoupled:-missing}"
  state_valid="$(json_field "${status}" map_odom_state_valid)"
  [[ "${state_valid}" == "true" ]] && pass "bridge map_odom_state_valid=true" || fail "bridge map_odom_state_valid=${state_valid:-missing}"
fi

api_status="$(api_status_once)"
if [[ -n "${api_status}" ]]; then
  if [[ -n "$(json_field "${api_status}" localization.post_relocalization_settle.in_progress)" ]]; then
    pass "API exposes localization.post_relocalization_settle"
  else
    warn "API status does not expose localization.post_relocalization_settle"
  fi
else
  warn "API status unavailable on 127.0.0.1:8080"
fi

if [[ "${SIMULATE_STATUS_ONLY}" == "false" ]]; then
  tf_echo_ok map odom && pass "TF map->odom available" || fail "TF map->odom unavailable"
  tf_echo_ok odom base_link && pass "TF odom->base_link available" || fail "TF odom->base_link unavailable"
  tf_echo_ok base_link lidar_level_link && pass "TF base_link->lidar_level_link available" || fail "TF base_link->lidar_level_link unavailable"
  topic_has_message /local_costmap/costmap && pass "/local_costmap/costmap heartbeat observed" || fail "/local_costmap/costmap heartbeat missing"

  drop_count="$(recent_message_filter_drop_count)"
  if [[ "${drop_count}" == "0" ]]; then
    pass "no recent MessageFilter/TF drop text observed"
  else
    warn "recent MessageFilter/TF drop text count=${drop_count}"
  fi
fi

if [[ "${WATCH}" == "true" ]]; then
  end=$((SECONDS + DURATION_SEC))
  while [[ ${SECONDS} -lt ${end} ]]; do
    status="$(bridge_status_once)"
    api_status="$(api_status_once)"
    echo "[post-reloc-settle] watch reason=${REASON} bridge_seq=$(json_field "${status}" last_explicit_relocalization_sequence) bridge_owner=$(json_field "${status}" map_to_odom_publisher_owner) api_settle=$(json_field "${api_status}" localization.post_relocalization_settle.in_progress) failure=$(json_field "${api_status}" localization.post_relocalization_settle.failure_reason)"
    sleep 1
  done
fi

echo "[post-reloc-settle] summary passes=${#passes[@]} warns=${#warns[@]} failures=${#failures[@]}"
if [[ "${EXPECT_FAIL}" == "true" ]]; then
  [[ ${#failures[@]} -gt 0 ]] && exit 0
  echo "[post-reloc-settle] expected failure but checks passed" >&2
  exit 1
fi
if [[ "${EXPECT_PASS}" == "true" && ${#failures[@]} -gt 0 ]]; then
  exit 1
fi
[[ ${#failures[@]} -eq 0 ]]

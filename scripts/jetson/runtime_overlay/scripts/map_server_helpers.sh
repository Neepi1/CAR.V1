#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

wait_for_ros_node() {
  local node_name="$1"
  local timeout_sec="${2:-15}"
  runtime_readiness_probe node "${node_name}" "${timeout_sec}"
}

map_server_param_matches_yaml() {
  local map_yaml="$1"
  [[ -n "${map_yaml}" ]] || return 1

  local current_yaml
  current_yaml="$(timeout 3 ros2 param get /map_server yaml_filename 2>/dev/null || true)"
  [[ "${current_yaml}" == *"${map_yaml}"* ]]
}

map_topic_matches_yaml() {
  local map_yaml="$1"
  [[ -n "${map_yaml}" && -f "${map_yaml}" ]] || return 1

  runtime_readiness_probe map-topic-matches-yaml "${map_yaml}" "${NJRH_MAP_TOPIC_MATCH_TIMEOUT_SEC:-8.0}"
}

map_server_publishing_requested_map() {
  local map_yaml="$1"
  [[ -n "${map_yaml}" ]] || return 1

  map_topic_matches_yaml "${map_yaml}" >/dev/null 2>&1
}

localization_map_external_lifecycle_bringup_enabled() {
  [[ "${NJRH_LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP:-false}" == "true" ]]
}

ensure_map_server_active() {
  local map_yaml="${1:-}"
  local timeout_sec="${2:-30}"
  local deadline=$((SECONDS + timeout_sec))
  local external_wait_logged=0

  if [[ -n "${map_yaml}" ]] && map_server_publishing_requested_map "${map_yaml}"; then
    echo "[runtime-overlay] requested map is already published on /map; continuing without waiting for /map_server discovery" >&2
    return 0
  fi

  wait_for_ros_node "/map_server" "${timeout_sec}" || {
    if [[ -n "${map_yaml}" ]] && map_server_publishing_requested_map "${map_yaml}"; then
      echo "[runtime-overlay] /map_server node discovery unavailable, but requested map is being published; continuing" >&2
      return 0
    fi
    echo "[runtime-overlay] /map_server did not appear within ${timeout_sec}s" >&2
    return 1
  }
  deadline=$((SECONDS + timeout_sec))

  if [[ -n "${map_yaml}" ]] && map_server_publishing_requested_map "${map_yaml}"; then
    echo "[runtime-overlay] requested map is already published after /map_server discovery; continuing without lifecycle state probe" >&2
    return 0
  fi

  if [[ -n "${map_yaml}" ]]; then
    echo "[runtime-overlay] expecting map_server asset: ${map_yaml}" >&2
  else
    local current_yaml
    current_yaml="$(timeout 3 ros2 param get /map_server yaml_filename 2>/dev/null || true)"
    echo "[runtime-overlay] using current map_server asset: ${current_yaml}" >&2
  fi

  while (( SECONDS < deadline )); do
    local state
    state="$(timeout 3 ros2 lifecycle get /map_server 2>/dev/null || true)"
    case "${state}" in
      active*)
        if [[ -z "${map_yaml}" ]] || map_server_publishing_requested_map "${map_yaml}"; then
          return 0
        fi
        echo "[runtime-overlay] /map_server is active but has not published requested map yet" >&2
        ;;
      unconfigured*)
        if localization_map_external_lifecycle_bringup_enabled; then
          if [[ "${external_wait_logged}" -eq 0 ]]; then
            echo "[runtime-overlay] external lifecycle_bringup is managing /map_server; waiting for active/map publication" >&2
            external_wait_logged=1
          fi
        else
          timeout 5 ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true
        fi
        ;;
      inactive*)
        if localization_map_external_lifecycle_bringup_enabled; then
          if [[ "${external_wait_logged}" -eq 0 ]]; then
            echo "[runtime-overlay] external lifecycle_bringup is managing /map_server; waiting for active/map publication" >&2
            external_wait_logged=1
          fi
        else
          timeout 5 ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true
        fi
        ;;
      *)
        if [[ -n "${map_yaml}" ]] && map_server_publishing_requested_map "${map_yaml}"; then
          echo "[runtime-overlay] /map_server lifecycle state unavailable, but requested map is selected; continuing" >&2
          return 0
        fi
        sleep 0.5
        ;;
    esac
    sleep 0.5
  done

  if [[ -n "${map_yaml}" ]] && map_server_publishing_requested_map "${map_yaml}"; then
    echo "[runtime-overlay] /map_server lifecycle state unavailable, but requested map is selected; continuing" >&2
    return 0
  fi

  echo "[runtime-overlay] /map_server did not reach active state within ${timeout_sec}s" >&2
  timeout 3 ros2 lifecycle get /map_server 2>/dev/null || true
  return 1
}

wait_for_occupancy_grid() {
  local topic_name="$1"
  local timeout_sec="${2:-20}"

  runtime_readiness_probe occupancy-grid "${topic_name}" "${timeout_sec}" 1 1
}

wait_for_global_costmap_static() {
  local timeout_sec="${1:-35}"
  local min_cells="${2:-101}"
  if [[ "${NJRH_GLOBAL_COSTMAP_FULL_MESSAGE_GATE:-false}" != "true" ]]; then
    runtime_readiness_probe global-costmap \
      "${timeout_sec}" "${NJRH_GLOBAL_COSTMAP_PUBLISHER_READY_TIMEOUT_SEC:-15}" || {
      echo "[runtime-overlay] global costmap lifecycle or publisher was not ready in time" >&2
      return 1
    }
    echo "[runtime-overlay] global costmap lifecycle and costmap publisher are ready; full OccupancyGrid message gate is deferred" >&2
    return 0
  fi

  runtime_readiness_probe occupancy-grid "/global_costmap/costmap" "${timeout_sec}" "${min_cells}" "${min_cells}" || {
    echo "[runtime-overlay] global costmap did not resize from static map in time" >&2
    return 1
  }
}

#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-status}"

WORKSPACE_HOST="${NJRH_WORKSPACE_HOST:-/home/nvidia/workspaces/njrh-v3/workspace1}"
WORKSPACE_CONTAINER="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
UPSTREAM_WORKSPACE_HOST="${NJRH_UPSTREAM_WORKSPACE_HOST:-/home/nvidia/workspaces/isaac_ros-dev}"
UPSTREAM_WORKSPACE_CONTAINER="${NJRH_UPSTREAM_WORKSPACE_CONTAINER:-/workspaces/isaac_ros-dev}"
UPSTREAM_WORKSPACE_ALIAS_CONTAINER="${NJRH_UPSTREAM_WORKSPACE_ALIAS_CONTAINER:-/workspaces/isaac_ros-dev-upstream}"
CONTAINER_NAME="${NJRH_CONTAINER_NAME:-NJRH-car}"
IMAGE_NAME="${NJRH_IMAGE_NAME:-njrh-car:latest}"
BASE_IMAGE="${NJRH_BASE_IMAGE:-isaac_ros_dev-aarch64:latest}"
RUNTIME_USER="${NJRH_RUNTIME_USER:-root}"
RUNTIME_GROUP="${NJRH_RUNTIME_GROUP:-${RUNTIME_USER}}"
RUNTIME_HOME="${NJRH_RUNTIME_HOME:-}"
if [[ -z "${RUNTIME_HOME}" ]]; then
  if [[ "${RUNTIME_USER}" == "root" ]]; then
    RUNTIME_HOME="/root"
  else
    RUNTIME_HOME="/home/${RUNTIME_USER}"
  fi
fi
ALLOW_BASE_IMAGE_FALLBACK="${NJRH_ALLOW_BASE_IMAGE_FALLBACK:-true}"
DOCKER_BUILD_NETWORK="${NJRH_DOCKER_BUILD_NETWORK:-host}"
DOCKERFILE_PATH="${NJRH_DOCKERFILE_PATH:-${WORKSPACE_HOST}/Dockerfile.car}"
DASHBOARD_PORT="${NJRH_DASHBOARD_PORT:-2048}"
DASHBOARD_HOST="${NJRH_DASHBOARD_HOST:-}"
ROBOT_API_SERVER_PORT="${NJRH_ROBOT_API_SERVER_PORT:-8080}"
DASHBOARD_RUNTIME_ROOT="${NJRH_DASHBOARD_RUNTIME_ROOT:-${WORKSPACE_CONTAINER}/scripts/jetson/runtime_overlay}"
DASHBOARD_LOG_RELATIVE="${NJRH_DASHBOARD_LOG_RELATIVE:-web_dashboard/runtime_logs/njrh_dashboard.out.log}"
DASHBOARD_TRACE_RELATIVE="${NJRH_DASHBOARD_TRACE_RELATIVE:-web_dashboard/runtime_logs/dashboard_trace.log}"
RUNTIME_IMAGE_NAME="$IMAGE_NAME"

die() {
  echo "[njrh-container] $*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "missing required command: $1"
}

workspace_ip() {
  hostname -I 2>/dev/null | awk '{print $1}'
}

dashboard_url() {
  local ip
  ip="$DASHBOARD_HOST"
  if [[ -z "$ip" ]]; then
    ip="$(workspace_ip)"
  fi
  if [[ -z "$ip" ]]; then
    ip="127.0.0.1"
  fi
  printf 'http://%s:%s\n' "$ip" "$DASHBOARD_PORT"
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -Fx "$CONTAINER_NAME" >/dev/null 2>&1
}

container_running() {
  docker ps --format '{{.Names}}' | grep -Fx "$CONTAINER_NAME" >/dev/null 2>&1
}

ensure_base_image() {
  docker image inspect "$BASE_IMAGE" >/dev/null 2>&1 || die \
    "base image not found: $BASE_IMAGE. Build the Isaac ROS dev base first or override NJRH_BASE_IMAGE."
}

ensure_image() {
  if docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    RUNTIME_IMAGE_NAME="$IMAGE_NAME"
    return
  fi

  ensure_base_image
  [[ -d "$WORKSPACE_HOST" ]] || die "workspace host path not found: $WORKSPACE_HOST"
  [[ -f "$DOCKERFILE_PATH" ]] || die "dockerfile not found: $DOCKERFILE_PATH"

  echo "[njrh-container] building image $IMAGE_NAME from $DOCKERFILE_PATH"
  if docker build \
    --network "$DOCKER_BUILD_NETWORK" \
    --build-arg "BASE_IMAGE=$BASE_IMAGE" \
    -f "$DOCKERFILE_PATH" \
    -t "$IMAGE_NAME" \
    "$WORKSPACE_HOST"; then
    RUNTIME_IMAGE_NAME="$IMAGE_NAME"
    return
  fi

  if [[ "$ALLOW_BASE_IMAGE_FALLBACK" == "true" ]]; then
    echo "[njrh-container] image build failed, falling back to base image $BASE_IMAGE"
    RUNTIME_IMAGE_NAME="$BASE_IMAGE"
    return
  fi

  die "failed to build runtime image $IMAGE_NAME"
}

remove_stopped_container() {
  if container_exists && ! container_running; then
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  fi
}

run_container() {
  local docker_args=()

  docker_args+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix")
  if [[ -f "$HOME/.Xauthority" ]]; then
    docker_args+=("-v" "$HOME/.Xauthority:${RUNTIME_HOME}/.Xauthority:rw")
  fi

  docker_args+=("-e" "DISPLAY=${DISPLAY:-:0}")
  docker_args+=("-e" "NVIDIA_VISIBLE_DEVICES=all")
  docker_args+=("-e" "NVIDIA_DRIVER_CAPABILITIES=all")
  docker_args+=("-e" "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
  docker_args+=("-e" "USER=${RUNTIME_USER}")
  docker_args+=("-e" "HOME=${RUNTIME_HOME}")
  docker_args+=("-e" "ISAAC_ROS_WS=$WORKSPACE_CONTAINER")
  docker_args+=("-e" "NJRH_UPSTREAM_ROOT=$UPSTREAM_WORKSPACE_CONTAINER")
  docker_args+=("-e" "NJRH_UPSTREAM_HOST_ROOT=$UPSTREAM_WORKSPACE_HOST")
  docker_args+=("-e" "NJRH_UPSTREAM_ALIAS_ROOT=$UPSTREAM_WORKSPACE_ALIAS_CONTAINER")
  docker_args+=("-e" "HOST_USER_UID=$(id -u)")
  docker_args+=("-e" "HOST_USER_GID=$(id -g)")

  if [[ -n "${SSH_AUTH_SOCK:-}" && -S "${SSH_AUTH_SOCK:-}" ]]; then
    docker_args+=("-v" "$SSH_AUTH_SOCK:/ssh-agent")
    docker_args+=("-e" "SSH_AUTH_SOCK=/ssh-agent")
  fi

  if [[ -x "/usr/bin/tegrastats" ]]; then
    docker_args+=("-v" "/usr/bin/tegrastats:/usr/bin/tegrastats")
  fi
  if [[ -d "/usr/lib/aarch64-linux-gnu/tegra" ]]; then
    docker_args+=("-v" "/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
  fi
  if [[ -d "/usr/src/jetson_multimedia_api" ]]; then
    docker_args+=("-v" "/usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
  fi
  if [[ -d "/usr/share/vpi3" ]]; then
    docker_args+=("-v" "/usr/share/vpi3:/usr/share/vpi3")
  fi
  if [[ -d "/dev/input" ]]; then
    docker_args+=("-v" "/dev/input:/dev/input")
  fi
  if [[ -S "/run/jtop.sock" ]]; then
    docker_args+=("-v" "/run/jtop.sock:/run/jtop.sock:ro")
  fi

  mkdir -p \
    "${WORKSPACE_HOST}/web_dashboard/runtime_logs" \
    "${WORKSPACE_HOST}/maps" \
    "${WORKSPACE_HOST}/maps3d" \
    "${WORKSPACE_HOST}/maps_release"

  if [[ -d "${UPSTREAM_WORKSPACE_HOST}" && "${UPSTREAM_WORKSPACE_HOST}" != "${WORKSPACE_HOST}" ]]; then
    docker_args+=("-v" "${UPSTREAM_WORKSPACE_HOST}:${UPSTREAM_WORKSPACE_CONTAINER}:ro")
    if [[ "${UPSTREAM_WORKSPACE_ALIAS_CONTAINER}" != "${UPSTREAM_WORKSPACE_CONTAINER}" ]]; then
      docker_args+=("-v" "${UPSTREAM_WORKSPACE_HOST}:${UPSTREAM_WORKSPACE_ALIAS_CONTAINER}:ro")
    fi
  fi

  docker run -d \
    --privileged \
    --network host \
    --ipc host \
    --pid host \
    --user "${RUNTIME_USER}" \
    --restart unless-stopped \
    "${docker_args[@]}" \
    -v "${WORKSPACE_HOST}:${WORKSPACE_CONTAINER}" \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp:/tmp \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir "$WORKSPACE_CONTAINER" \
    "$RUNTIME_IMAGE_NAME" \
    /bin/bash -lc "trap 'exit 0' TERM INT; while sleep 3600; do :; done"
}

start_container() {
  if container_running; then
    RUNTIME_IMAGE_NAME="$(docker inspect --format '{{.Config.Image}}' "$CONTAINER_NAME")"
    wait_for_container_ready
    prepare_nitros_tmp
    ensure_gs2_device_in_container
    echo "[njrh-container] container already running: $CONTAINER_NAME"
    return
  fi

  ensure_image
  remove_stopped_container
  if container_exists; then
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  fi
  run_container
  wait_for_container_ready
  prepare_nitros_tmp
  ensure_gs2_device_in_container
  echo "[njrh-container] container started: $CONTAINER_NAME"
}

stop_container() {
  if container_exists; then
    docker rm -f "$CONTAINER_NAME" >/dev/null
    echo "[njrh-container] container removed: $CONTAINER_NAME"
  else
    echo "[njrh-container] container not found: $CONTAINER_NAME"
  fi
}

exec_runtime() {
  docker exec -u "${RUNTIME_USER}" --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" /bin/bash -lc "$1"
}

dashboard_process_lines() {
  docker exec -u "${RUNTIME_USER}" "$CONTAINER_NAME" ps -eo pid,args | grep dashboard_server.py | grep -v grep || true
}

dashboard_running() {
  container_running || return 1
  [[ -n "$(dashboard_process_lines)" ]]
}

dashboard_running_overlay() {
  container_running || return 1
  dashboard_process_lines | grep -F "$DASHBOARD_RUNTIME_ROOT" >/dev/null 2>&1
}

dashboard_assets_ready() {
  container_running || return 1
  docker exec -u "${RUNTIME_USER}" "$CONTAINER_NAME" test -f "${DASHBOARD_RUNTIME_ROOT}/web_dashboard/dashboard_server.py" &&
  docker exec -u "${RUNTIME_USER}" "$CONTAINER_NAME" test -e "${DASHBOARD_RUNTIME_ROOT}/web_dashboard/index.html"
}

dashboard_http_ready() {
  curl -fsS "http://127.0.0.1:${DASHBOARD_PORT}/api/status" >/dev/null 2>&1 &&
  curl -fsS "http://127.0.0.1:${DASHBOARD_PORT}/" >/dev/null 2>&1
}

robot_api_http_ready() {
  local curl_args=(-fsS)
  if [[ -n "${ROBOT_API_TOKEN:-}" ]]; then
    curl_args+=("-H" "X-Robot-Token: ${ROBOT_API_TOKEN}")
  fi
  curl "${curl_args[@]}" "http://127.0.0.1:${ROBOT_API_SERVER_PORT}/api/v1/status" >/dev/null 2>&1
}

kill_dashboard_processes() {
  docker exec "$CONTAINER_NAME" /bin/bash -lc \
    "ps -eo pid=,args= | grep 'python3 .*dashboard_server.py' | grep -v grep | awk '{print \$1}' | xargs -r kill -INT 2>/dev/null || true"
  sleep 1
  docker exec "$CONTAINER_NAME" /bin/bash -lc \
    "ps -eo pid=,args= | grep 'python3 .*dashboard_server.py' | grep -v grep | awk '{print \$1}' | xargs -r kill -9 2>/dev/null || true"
}

exec_runtime_retry() {
  local cmd="$1"
  local attempt
  for attempt in $(seq 1 12); do
    if docker exec -u "${RUNTIME_USER}" --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" /bin/bash -lc "$cmd"; then
      return 0
    fi
    sleep 1
  done
  die "${RUNTIME_USER} exec did not succeed after retries: $cmd"
}

wait_for_container_ready() {
  local attempt
  for attempt in $(seq 1 20); do
    if docker exec "$CONTAINER_NAME" id "${RUNTIME_USER}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  die "container did not become ready for exec: $CONTAINER_NAME"
}

prepare_nitros_tmp() {
  container_running || return 0
  docker exec "$CONTAINER_NAME" /bin/bash -lc \
    "mkdir -p /tmp/isaac_ros_nitros/graphs \
      && chown root:root /tmp/isaac_ros_nitros /tmp/isaac_ros_nitros/graphs \
      && chmod 1777 /tmp/isaac_ros_nitros /tmp/isaac_ros_nitros/graphs" >/dev/null
}

ensure_gs2_device_in_container() {
  container_running || return 0
  local host_gs2="${NJRH_GS2_HOST_DEVICE:-/dev/gs2}"
  local host_target=""
  if [[ -e "$host_gs2" ]]; then
    host_target="$(readlink -f "$host_gs2")"
  elif compgen -G "/dev/serial/by-id/*CP2102*" >/dev/null; then
    host_target="$(readlink -f /dev/serial/by-id/*CP2102* | head -n 1)"
  else
    return 0
  fi

  [[ -n "$host_target" && -e "$host_target" ]] || return 0
  local target_name major_hex minor_hex major_dec minor_dec
  target_name="$(basename "$host_target")"
  read -r major_hex minor_hex < <(stat -c '%t %T' "$host_target")
  major_dec="$((16#$major_hex))"
  minor_dec="$((16#$minor_hex))"

  docker exec -u root "$CONTAINER_NAME" /bin/bash -lc \
    "set -e
      if [[ ! -e '/dev/${target_name}' ]]; then
        mknod -m 666 '/dev/${target_name}' c ${major_dec} ${minor_dec}
      fi
      chmod 666 '/dev/${target_name}' || true
      ln -sf '/dev/${target_name}' /dev/gs2
      mkdir -p /dev/serial/by-id
      ln -sf '../../${target_name}' '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'" >/dev/null 2>&1 || true
}

prepare_release_asset_permissions() {
  container_running || return 0
  docker exec -u root "$CONTAINER_NAME" /bin/bash -lc \
    "mkdir -p '${WORKSPACE_CONTAINER}/maps_release' \
      && chown -R '${RUNTIME_USER}:${RUNTIME_GROUP}' '${WORKSPACE_CONTAINER}/maps_release' \
      && find '${WORKSPACE_CONTAINER}/maps_release' -type d -exec chmod 2775 {} + \
      && find '${WORKSPACE_CONTAINER}/maps_release' -type f -exec chmod 664 {} +" >/dev/null
}

prepare_runtime_overlay_permissions() {
  container_running || return 0
  docker exec -u root "$CONTAINER_NAME" /bin/bash -lc \
    "mkdir -p \
        '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs' \
        '${DASHBOARD_RUNTIME_ROOT}/maps' \
        '${DASHBOARD_RUNTIME_ROOT}/maps3d' \
        '${DASHBOARD_RUNTIME_ROOT}/waypoints' \
      && chown -R '${RUNTIME_USER}:${RUNTIME_GROUP}' \
        '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs' \
        '${DASHBOARD_RUNTIME_ROOT}/maps' \
        '${DASHBOARD_RUNTIME_ROOT}/maps3d' \
        '${DASHBOARD_RUNTIME_ROOT}/waypoints' \
      && find \
        '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs' \
        '${DASHBOARD_RUNTIME_ROOT}/maps' \
        '${DASHBOARD_RUNTIME_ROOT}/maps3d' \
        '${DASHBOARD_RUNTIME_ROOT}/waypoints' \
        -type d -exec chmod 2775 {} + \
      && find \
        '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs' \
        '${DASHBOARD_RUNTIME_ROOT}/maps' \
        '${DASHBOARD_RUNTIME_ROOT}/maps3d' \
        '${DASHBOARD_RUNTIME_ROOT}/waypoints' \
        -type f -exec chmod 664 {} +" >/dev/null
}

start_dashboard() {
  container_running || start_container
  wait_for_container_ready
  prepare_runtime_overlay_permissions
  prepare_release_asset_permissions
  prepare_nitros_tmp
  if dashboard_running_overlay && dashboard_assets_ready && dashboard_http_ready; then
    echo "[njrh-container] dashboard already running: $(dashboard_url)"
    return
  fi

  if dashboard_running; then
    kill_dashboard_processes
  fi

  if docker exec "$CONTAINER_NAME" test -f "${DASHBOARD_RUNTIME_ROOT}/scripts/run_web_dashboard.sh" >/dev/null 2>&1; then
    docker exec -u "${RUNTIME_USER}" --workdir "${DASHBOARD_RUNTIME_ROOT}" "$CONTAINER_NAME" \
      /bin/bash -lc "mkdir -p '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs'; cd '${DASHBOARD_RUNTIME_ROOT}'; nohup env NJRH_UPSTREAM_ROOT='${UPSTREAM_WORKSPACE_CONTAINER}' NJRH_UPSTREAM_HOST_ROOT='${UPSTREAM_WORKSPACE_HOST}' bash scripts/run_web_dashboard.sh > '${DASHBOARD_RUNTIME_ROOT}/${DASHBOARD_LOG_RELATIVE}' 2>&1 </dev/null &"
  else
    docker exec -u "${RUNTIME_USER}" --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" \
      /bin/bash -lc "mkdir -p '$WORKSPACE_CONTAINER/web_dashboard/runtime_logs'; cd '$WORKSPACE_CONTAINER'; nohup bash scripts/run_web_dashboard.sh > '$WORKSPACE_CONTAINER/$DASHBOARD_LOG_RELATIVE' 2>&1 </dev/null &"
  fi
  wait_for_dashboard
  echo "[njrh-container] dashboard ready: $(dashboard_url)"
}

stop_dashboard() {
  if ! container_running; then
    echo "[njrh-container] container not running: $CONTAINER_NAME"
    return
  fi
  kill_dashboard_processes
  echo "[njrh-container] dashboard stopped"
}

common_services_process_lines() {
  docker exec -u "${RUNTIME_USER}" "$CONTAINER_NAME" ps -eo pid,args | grep run_common_services.sh | grep -v grep || true
}

common_services_running() {
  container_running || return 1
  [[ -n "$(common_services_process_lines)" ]]
}

start_common_services() {
  container_running || start_container
  wait_for_container_ready
  prepare_runtime_overlay_permissions
  prepare_release_asset_permissions
  prepare_nitros_tmp
  ensure_gs2_device_in_container
  if common_services_running; then
    wait_for_robot_api
    echo "[njrh-container] common services already running"
    return
  fi
  docker exec -u "${RUNTIME_USER}" --workdir "${DASHBOARD_RUNTIME_ROOT}" "$CONTAINER_NAME" \
    /bin/bash -lc "mkdir -p '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs'; cd '${DASHBOARD_RUNTIME_ROOT}'; nohup env ROBOT_API_TOKEN='${ROBOT_API_TOKEN:-}' ROBOT_API_SERVER_PORT='${ROBOT_API_SERVER_PORT}' bash scripts/run_common_services.sh > '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs/common_services.out.log' 2>&1 </dev/null &"
  sleep 2
  if common_services_running; then
    wait_for_robot_api
    echo "[njrh-container] common services started"
  else
    die "common services did not stay running"
  fi
}

stop_common_services() {
  if ! container_running; then
    echo "[njrh-container] container not running: $CONTAINER_NAME"
    return
  fi
  docker exec "$CONTAINER_NAME" /bin/bash -lc \
    "ps -eo pid=,args= | grep 'run_common_services.sh' | grep -v grep | awk '{print \$1}' | xargs -r kill -INT 2>/dev/null || true"
  echo "[njrh-container] common services stop requested"
}

wait_for_dashboard() {
  local attempt
  for attempt in $(seq 1 25); do
    if dashboard_assets_ready && dashboard_http_ready; then
      return 0
    fi
    sleep 1
  done
  die "dashboard did not become ready on port ${DASHBOARD_PORT}"
}

wait_for_robot_api() {
  local attempt
  for attempt in $(seq 1 30); do
    if robot_api_http_ready; then
      return 0
    fi
    sleep 1
  done
  die "robot_api_server did not become ready on port ${ROBOT_API_SERVER_PORT}"
}

print_status() {
  echo "container_name: ${CONTAINER_NAME}"
  echo "image_name_requested: ${IMAGE_NAME}"
  if container_exists; then
    echo "image_name_resolved: $(docker inspect --format '{{.Config.Image}}' "$CONTAINER_NAME")"
  else
    echo "image_name_resolved: ${RUNTIME_IMAGE_NAME}"
  fi
  echo "workspace_host: ${WORKSPACE_HOST}"
  echo "workspace_container: ${WORKSPACE_CONTAINER}"
  echo "dashboard_url: $(dashboard_url)"
  if container_running; then
    echo "container_status: running"
    if common_services_running; then
      echo "common_services_status: running"
    else
      echo "common_services_status: stopped"
    fi
    if robot_api_http_ready; then
      echo "robot_api_status: running"
    else
      echo "robot_api_status: stopped"
    fi
    if dashboard_running; then
      echo "dashboard_status: running"
      if dashboard_running_overlay; then
        echo "dashboard_runtime_root: ${DASHBOARD_RUNTIME_ROOT}"
      else
        echo "dashboard_runtime_root: non_overlay_process"
      fi
    else
      echo "dashboard_status: stopped"
    fi
  elif container_exists; then
    echo "container_status: stopped"
    echo "common_services_status: stopped"
    echo "robot_api_status: stopped"
    echo "dashboard_status: stopped"
  else
    echo "container_status: missing"
    echo "common_services_status: stopped"
    echo "robot_api_status: stopped"
    echo "dashboard_status: stopped"
  fi
}

  show_logs() {
  container_running || die "container is not running: $CONTAINER_NAME"
  if docker exec "$CONTAINER_NAME" test -f "${DASHBOARD_RUNTIME_ROOT}/${DASHBOARD_LOG_RELATIVE}" >/dev/null 2>&1; then
    docker exec -u "${RUNTIME_USER}" "$CONTAINER_NAME" /bin/bash -lc "tail -n 120 '${DASHBOARD_RUNTIME_ROOT}/${DASHBOARD_LOG_RELATIVE}'"
  elif docker exec "$CONTAINER_NAME" test -f "${WORKSPACE_CONTAINER}/${DASHBOARD_LOG_RELATIVE}" >/dev/null 2>&1; then
    docker exec -u "${RUNTIME_USER}" "$CONTAINER_NAME" /bin/bash -lc "tail -n 120 '${WORKSPACE_CONTAINER}/${DASHBOARD_LOG_RELATIVE}'"
  elif docker exec "$CONTAINER_NAME" test -f "${WORKSPACE_CONTAINER}/${DASHBOARD_TRACE_RELATIVE}" >/dev/null 2>&1; then
    docker exec -u "${RUNTIME_USER}" "$CONTAINER_NAME" /bin/bash -lc "tail -n 120 '${WORKSPACE_CONTAINER}/${DASHBOARD_TRACE_RELATIVE}'"
  else
    echo "dashboard log file not found"
  fi
}

open_shell() {
  container_running || start_container
  exec docker exec -it -u "${RUNTIME_USER}" --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" /bin/bash
}

case "$ACTION" in
  build-image)
    ensure_image
    echo "[njrh-container] image ready: $IMAGE_NAME"
    ;;
  start)
    start_container
    ;;
  stop)
    stop_container
    ;;
  restart)
    stop_container
    start_container
    ;;
  shell)
    open_shell
    ;;
  status)
    print_status
    ;;
  start-dashboard)
    start_dashboard
    ;;
  stop-dashboard)
    stop_dashboard
    ;;
  start-common)
    start_common_services
    print_status
    ;;
  stop-common)
    stop_common_services
    ;;
  dashboard-status)
    if dashboard_running; then
      echo "[njrh-container] dashboard running: $(dashboard_url)"
    else
      echo "[njrh-container] dashboard stopped"
      exit 1
    fi
    ;;
  dashboard-logs)
    show_logs
    ;;
  start-runtime)
    start_container
    start_common_services
    print_status
    ;;
  start-debug-runtime)
    start_container
    start_common_services
    start_dashboard
    print_status
    ;;
  print-url)
    dashboard_url
    ;;
  *)
    die "unsupported action: $ACTION"
    ;;
esac

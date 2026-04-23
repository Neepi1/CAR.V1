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
ALLOW_BASE_IMAGE_FALLBACK="${NJRH_ALLOW_BASE_IMAGE_FALLBACK:-true}"
DOCKER_BUILD_NETWORK="${NJRH_DOCKER_BUILD_NETWORK:-host}"
DOCKERFILE_PATH="${NJRH_DOCKERFILE_PATH:-${WORKSPACE_HOST}/Dockerfile.car}"
DASHBOARD_PORT="${NJRH_DASHBOARD_PORT:-2048}"
DASHBOARD_HOST="${NJRH_DASHBOARD_HOST:-}"
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
    docker_args+=("-v" "$HOME/.Xauthority:/home/admin/.Xauthority:rw")
  fi

  docker_args+=("-e" "DISPLAY=${DISPLAY:-:0}")
  docker_args+=("-e" "NVIDIA_VISIBLE_DEVICES=all")
  docker_args+=("-e" "NVIDIA_DRIVER_CAPABILITIES=all")
  docker_args+=("-e" "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
  docker_args+=("-e" "USER=${USER:-nvidia}")
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

  mkdir -p "${WORKSPACE_HOST}/web_dashboard/runtime_logs" "${WORKSPACE_HOST}/maps" "${WORKSPACE_HOST}/maps3d"

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
    echo "[njrh-container] container already running: $CONTAINER_NAME"
    return
  fi

  ensure_image
  remove_stopped_container
  if container_exists; then
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  fi
  run_container
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

exec_admin() {
  docker exec -u admin --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" /bin/bash -lc "$1"
}

dashboard_process_lines() {
  docker exec -u admin "$CONTAINER_NAME" ps -eo pid,args | grep dashboard_server.py | grep -v grep || true
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
  docker exec -u admin "$CONTAINER_NAME" test -f "${DASHBOARD_RUNTIME_ROOT}/web_dashboard/dashboard_server.py" &&
  docker exec -u admin "$CONTAINER_NAME" test -e "${DASHBOARD_RUNTIME_ROOT}/web_dashboard/index.html"
}

dashboard_http_ready() {
  curl -fsS "http://127.0.0.1:${DASHBOARD_PORT}/api/status" >/dev/null 2>&1 &&
  curl -fsS "http://127.0.0.1:${DASHBOARD_PORT}/" >/dev/null 2>&1
}

kill_dashboard_processes() {
  docker exec "$CONTAINER_NAME" /bin/bash -lc \
    "ps -eo pid=,args= | grep 'python3 .*dashboard_server.py' | grep -v grep | awk '{print \$1}' | xargs -r kill -INT 2>/dev/null || true"
  sleep 1
  docker exec "$CONTAINER_NAME" /bin/bash -lc \
    "ps -eo pid=,args= | grep 'python3 .*dashboard_server.py' | grep -v grep | awk '{print \$1}' | xargs -r kill -9 2>/dev/null || true"
}

exec_admin_retry() {
  local cmd="$1"
  local attempt
  for attempt in $(seq 1 12); do
    if docker exec -u admin --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" /bin/bash -lc "$cmd"; then
      return 0
    fi
    sleep 1
  done
  die "admin exec did not succeed after retries: $cmd"
}

wait_for_container_ready() {
  local attempt
  for attempt in $(seq 1 20); do
    if docker exec "$CONTAINER_NAME" id admin >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  die "container did not become ready for exec: $CONTAINER_NAME"
}

start_dashboard() {
  container_running || start_container
  wait_for_container_ready
  if dashboard_running_overlay && dashboard_assets_ready && dashboard_http_ready; then
    echo "[njrh-container] dashboard already running: $(dashboard_url)"
    return
  fi

  if dashboard_running; then
    kill_dashboard_processes
  fi

  if docker exec "$CONTAINER_NAME" test -f "${DASHBOARD_RUNTIME_ROOT}/scripts/run_web_dashboard.sh" >/dev/null 2>&1; then
    docker exec -u admin --workdir "${DASHBOARD_RUNTIME_ROOT}" "$CONTAINER_NAME" \
      /bin/bash -lc "mkdir -p '${DASHBOARD_RUNTIME_ROOT}/web_dashboard/runtime_logs'; cd '${DASHBOARD_RUNTIME_ROOT}'; nohup env NJRH_UPSTREAM_ROOT='${UPSTREAM_WORKSPACE_CONTAINER}' NJRH_UPSTREAM_HOST_ROOT='${UPSTREAM_WORKSPACE_HOST}' bash scripts/run_web_dashboard.sh > '${DASHBOARD_RUNTIME_ROOT}/${DASHBOARD_LOG_RELATIVE}' 2>&1 </dev/null &"
  else
    docker exec -u admin --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" \
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
    echo "dashboard_status: stopped"
  else
    echo "container_status: missing"
    echo "dashboard_status: stopped"
  fi
}

show_logs() {
  container_running || die "container is not running: $CONTAINER_NAME"
  if docker exec "$CONTAINER_NAME" test -f "${DASHBOARD_RUNTIME_ROOT}/${DASHBOARD_LOG_RELATIVE}" >/dev/null 2>&1; then
    docker exec -u admin "$CONTAINER_NAME" /bin/bash -lc "tail -n 120 '${DASHBOARD_RUNTIME_ROOT}/${DASHBOARD_LOG_RELATIVE}'"
  elif docker exec "$CONTAINER_NAME" test -f "${WORKSPACE_CONTAINER}/${DASHBOARD_LOG_RELATIVE}" >/dev/null 2>&1; then
    docker exec -u admin "$CONTAINER_NAME" /bin/bash -lc "tail -n 120 '${WORKSPACE_CONTAINER}/${DASHBOARD_LOG_RELATIVE}'"
  elif docker exec "$CONTAINER_NAME" test -f "${WORKSPACE_CONTAINER}/${DASHBOARD_TRACE_RELATIVE}" >/dev/null 2>&1; then
    docker exec -u admin "$CONTAINER_NAME" /bin/bash -lc "tail -n 120 '${WORKSPACE_CONTAINER}/${DASHBOARD_TRACE_RELATIVE}'"
  else
    echo "dashboard log file not found"
  fi
}

open_shell() {
  container_running || start_container
  exec docker exec -it -u admin --workdir "$WORKSPACE_CONTAINER" "$CONTAINER_NAME" /bin/bash
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

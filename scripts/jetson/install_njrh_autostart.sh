#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-install}"

SERVICE_NAME="${NJRH_AUTOSTART_SERVICE_NAME:-njrh-runtime.service}"
CAN_SERVICE_NAME="${NJRH_CAN_SERVICE_NAME:-njrh-can.service}"
WORKSPACE_HOST="${NJRH_WORKSPACE_HOST:-/home/nvidia/workspaces/njrh-v3/workspace1}"
WORKSPACE_CONTAINER="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
UPSTREAM_WORKSPACE_HOST="${NJRH_UPSTREAM_WORKSPACE_HOST:-/home/nvidia/workspaces/isaac_ros-dev}"
UPSTREAM_WORKSPACE_CONTAINER="${NJRH_UPSTREAM_WORKSPACE_CONTAINER:-/workspaces/isaac_ros-dev}"
CONTAINER_NAME="${NJRH_CONTAINER_NAME:-NJRH-car}"
ENV_FILE="${NJRH_AUTOSTART_ENV_FILE:-/etc/njrh/runtime.env}"
UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}"
CAN_UNIT_PATH="/etc/systemd/system/${CAN_SERVICE_NAME}"
RUNNER="${WORKSPACE_HOST}/scripts/jetson/njrh_systemd_runtime.sh"
CAN_RUNNER="${WORKSPACE_HOST}/scripts/jetson/bringup_ranger_can_wait.sh"

require_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    return 0
  fi
  sudo -v
}

write_env_file_if_missing() {
  require_sudo
  sudo mkdir -p "$(dirname "${ENV_FILE}")"
  if [[ -f "${ENV_FILE}" ]]; then
    echo "[njrh-autostart] env file already exists: ${ENV_FILE}"
    return 0
  fi
  sudo tee "${ENV_FILE}" >/dev/null <<EOF
NJRH_WORKSPACE_HOST=${WORKSPACE_HOST}
NJRH_WORKSPACE_CONTAINER=${WORKSPACE_CONTAINER}
NJRH_UPSTREAM_WORKSPACE_HOST=${UPSTREAM_WORKSPACE_HOST}
NJRH_UPSTREAM_WORKSPACE_CONTAINER=${UPSTREAM_WORKSPACE_CONTAINER}
NJRH_CONTAINER_NAME=${CONTAINER_NAME}
NJRH_REUSE_COMMON_SERVICES=true
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ROBOT_API_TOKEN=${ROBOT_API_TOKEN:-}
EOF
  sudo chmod 0644 "${ENV_FILE}"
  echo "[njrh-autostart] created env file: ${ENV_FILE}"
}

install_unit() {
  require_sudo
  [[ -x "${RUNNER}" ]] || {
    echo "[njrh-autostart] missing executable runner: ${RUNNER}" >&2
    exit 1
  }
  [[ -x "${CAN_RUNNER}" ]] || {
    echo "[njrh-autostart] missing executable CAN runner: ${CAN_RUNNER}" >&2
    exit 1
  }
  write_env_file_if_missing
  sudo tee "${CAN_UNIT_PATH}" >/dev/null <<EOF
[Unit]
Description=NJRH Ranger CAN bringup
Wants=sys-subsystem-net-devices-can0.device
After=sys-subsystem-net-devices-can0.device
Before=${SERVICE_NAME}

[Service]
Type=oneshot
RemainAfterExit=yes
EnvironmentFile=-${ENV_FILE}
Environment=CAN_IFACE=can0
Environment=CAN_BITRATE=500000
Environment=CAN_WAIT_TIMEOUT_SEC=120
ExecStart=${CAN_RUNNER}

[Install]
WantedBy=multi-user.target
EOF
  sudo tee "${UNIT_PATH}" >/dev/null <<EOF
[Unit]
Description=NJRH car container and common ROS services
Wants=network-online.target docker.service ${CAN_SERVICE_NAME}
After=network-online.target docker.service ${CAN_SERVICE_NAME}
Requires=docker.service ${CAN_SERVICE_NAME}

[Service]
Type=simple
User=${SUDO_USER:-${USER}}
WorkingDirectory=${WORKSPACE_HOST}
EnvironmentFile=-${ENV_FILE}
ExecStart=${RUNNER} run
ExecStop=${RUNNER} stop
Restart=always
RestartSec=10
TimeoutStartSec=240
TimeoutStopSec=30
KillSignal=SIGINT

[Install]
WantedBy=multi-user.target
EOF
  sudo systemctl daemon-reload
  sudo systemctl enable "${CAN_SERVICE_NAME}"
  sudo systemctl enable "${SERVICE_NAME}"
  echo "[njrh-autostart] enabled ${CAN_SERVICE_NAME} and ${SERVICE_NAME}"
}

case "${ACTION}" in
  install)
    install_unit
    systemctl is-enabled "${SERVICE_NAME}" || true
    ;;
  install-start)
    install_unit
    sudo systemctl restart "${CAN_SERVICE_NAME}"
    sudo systemctl restart "${SERVICE_NAME}"
    sudo systemctl --no-pager --full status "${CAN_SERVICE_NAME}" || true
    sudo systemctl --no-pager --full status "${SERVICE_NAME}" || true
    ;;
  start)
    require_sudo
    sudo systemctl start "${SERVICE_NAME}"
    ;;
  stop)
    require_sudo
    sudo systemctl stop "${SERVICE_NAME}"
    ;;
  restart)
    require_sudo
    sudo systemctl restart "${SERVICE_NAME}"
    ;;
  status)
    systemctl --no-pager --full status "${SERVICE_NAME}" || true
    ;;
  disable)
    require_sudo
    sudo systemctl disable --now "${SERVICE_NAME}" || true
    sudo systemctl disable --now "${CAN_SERVICE_NAME}" || true
    sudo rm -f "${UNIT_PATH}"
    sudo rm -f "${CAN_UNIT_PATH}"
    sudo systemctl daemon-reload
    echo "[njrh-autostart] disabled ${CAN_SERVICE_NAME} and ${SERVICE_NAME}"
    ;;
  *)
    echo "usage: $0 {install|install-start|start|stop|restart|status|disable}" >&2
    exit 1
    ;;
esac

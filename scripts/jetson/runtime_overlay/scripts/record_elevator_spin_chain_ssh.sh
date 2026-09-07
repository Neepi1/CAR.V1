#!/usr/bin/env bash
set -uo pipefail

# Host-side SSH entrypoint.  ROS subscriptions run inside NJRH-car because the
# nvidia host user cannot reliably receive this runtime's Fast DDS SHM payloads.

CONTAINER="${NJRH_RUNTIME_CONTAINER:-NJRH-car}"
CONTAINER_WORKSPACE="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
REPORT_ROOT="${NJRH_REPORT_ROOT:-/tmp/njrh_reports}"
DURATION_SEC=120
SAMPLE_HZ=20
LABEL="elevator_spin"
PREFIX="[elevator-spin-ssh]"
CONTAINER_REPORT=""
HOST_REPORT=""
INTERRUPTED=false

usage() {
  cat <<'EOF'
用法（在 Jetson SSH 终端执行）：
  bash /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/record_elevator_spin_chain_ssh.sh \
    --duration-sec 120 --label elevator_spin_01

看到 READY 后再从 App 开始乘梯测试。出现分段自旋后可按 Ctrl+C；脚本也会
到时自动退出。报告固定复制到宿主机 /tmp/njrh_reports/<本次记录>/。

只读记录：五级速度、底盘模式、wheel/local odom、IMU、map 位姿、电梯状态，
并把 /scan 压成车体/StopZone/SlowZone 点数；不保存 scan 数组，不订阅点云。

选项：
  --duration-sec N  记录秒数，1..600，默认 120
  --sample-hz N     CSV 快照频率，2..50，默认 20
  --label TEXT      本次标签
  -h, --help        显示帮助
EOF
}

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --sample-hz)
      SAMPLE_HZ="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "${PREFIX} 未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} --duration-sec 必须是数字" >&2
  exit 2
fi
if ! [[ "${SAMPLE_HZ}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} --sample-hz 必须是数字" >&2
  exit 2
fi
if ! awk -v value="${DURATION_SEC}" 'BEGIN { exit !(value >= 1 && value <= 600) }'; then
  echo "${PREFIX} --duration-sec 必须在 1..600 之间" >&2
  exit 2
fi
if ! awk -v value="${SAMPLE_HZ}" 'BEGIN { exit !(value >= 2 && value <= 50) }'; then
  echo "${PREFIX} --sample-hz 必须在 2..50 之间" >&2
  exit 2
fi

LABEL="$(sanitize_label "${LABEL}")"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_NAME="${TIMESTAMP}_${LABEL}"
HOST_REPORT="${REPORT_ROOT}/${RUN_NAME}"
CONTAINER_REPORT="/tmp/njrh_reports/${RUN_NAME}"
RECORDER="${CONTAINER_WORKSPACE}/scripts/jetson/runtime_overlay/scripts/record_elevator_spin_diagnostic.py"
COMMON_ENV="${CONTAINER_WORKSPACE}/scripts/jetson/runtime_overlay/scripts/common_env.sh"

if ! command -v docker >/dev/null 2>&1; then
  echo "${PREFIX} 未找到 docker；请在 Jetson 宿主机 SSH 终端运行本脚本" >&2
  exit 1
fi
if ! docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -qx true; then
  echo "${PREFIX} 容器 ${CONTAINER} 未运行" >&2
  exit 1
fi
if ! mkdir -p "${HOST_REPORT}"; then
  echo "${PREFIX} 无法创建 ${HOST_REPORT}" >&2
  exit 1
fi
if ! docker exec -i "${CONTAINER}" test -f "${RECORDER}"; then
  echo "${PREFIX} 容器内缺少记录器：${RECORDER}" >&2
  exit 1
fi

stop_container_recorder() {
  local recorder_pid=""
  INTERRUPTED=true
  recorder_pid="$(docker exec -i "${CONTAINER}" sh -c \
    "test -f '${CONTAINER_REPORT}/recorder.pid' && cat '${CONTAINER_REPORT}/recorder.pid'" \
    2>/dev/null || true)"
  if [[ "${recorder_pid}" =~ ^[0-9]+$ ]]; then
    docker exec -i "${CONTAINER}" kill -INT "${recorder_pid}" >/dev/null 2>&1 || true
  fi
}
trap stop_container_recorder INT TERM HUP

echo "${PREFIX} 本次宿主机报告：${HOST_REPORT}"
echo "${PREFIX} 正在容器内建立一个只读 ROS 参与者……"

set +e
docker exec -i \
  -e PYTHONUNBUFFERED=1 \
  "${CONTAINER}" \
  bash -lc "
    unset NJRH_COMMON_ENV_LOADED NJRH_COMMON_ENV_SETUP_DONE
    export NJRH_COMMON_ENV_PARENT_READY=0
    source '${COMMON_ENV}'
    exec python3 '${RECORDER}' \
      --output-dir '${CONTAINER_REPORT}' \
      --duration-sec '${DURATION_SEC}' \
      --sample-hz '${SAMPLE_HZ}' \
      --label '${LABEL}'
  "
CAPTURE_RC=$?
# Always recover the exact report, even after Ctrl+C or an inner diagnostic
# failure.  No workspace file or runtime parameter is changed.
docker cp "${CONTAINER}:${CONTAINER_REPORT}/." "${HOST_REPORT}/" >/dev/null 2>&1
COPY_RC=$?

if [[ "${COPY_RC}" -ne 0 ]]; then
  echo "${PREFIX} 报告复制失败；容器内仍保留：${CONTAINER_REPORT}" >&2
  exit 1
fi
if [[ "${INTERRUPTED}" == "true" ]]; then
  echo "interrupted=true" >>"${HOST_REPORT}/host_capture.env"
fi
{
  echo "container=${CONTAINER}"
  echo "container_report=${CONTAINER_REPORT}"
  echo "host_report=${HOST_REPORT}"
  echo "capture_rc=${CAPTURE_RC}"
} >>"${HOST_REPORT}/host_capture.env"

RECORDER_PID="$(docker exec -i "${CONTAINER}" sh -c \
  "test -f '${CONTAINER_REPORT}/recorder.pid' && cat '${CONTAINER_REPORT}/recorder.pid'" \
  2>/dev/null || true)"
if [[ "${RECORDER_PID}" =~ ^[0-9]+$ ]] && \
  docker exec -i "${CONTAINER}" sh -c \
    "test -r '/proc/${RECORDER_PID}/cmdline' && tr '\\000' ' ' <'/proc/${RECORDER_PID}/cmdline' | grep -Fq '${RECORDER} --output-dir ${CONTAINER_REPORT}'"; then
  echo "${PREFIX} 本次记录进程仍存在，已请求定向停止；请勿开始下一轮" >&2
  stop_container_recorder
  exit 1
fi

echo "${PREFIX} 已完成：${HOST_REPORT}"
echo "${PREFIX} 快速查看：sed -n '1,220p' '${HOST_REPORT}/summary.md'"
exit "${CAPTURE_RC}"

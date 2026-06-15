#!/usr/bin/env bash
set -euo pipefail

MIN_AGE_SEC=600
EXECUTE=false

usage() {
  cat <<'USAGE'
Usage: cleanup_stale_ros2_cli.sh [--min-age-sec N] [--execute]

Dry-run by default. Finds long-running ros2 CLI inspection commands such as
topic/param/service/action/node/interface/lifecycle/doctor/bag and tf2_echo,
and, with --execute,
sends TERM only to those exact PIDs. It intentionally excludes ros2 run and
ros2 launch so formal runtime nodes are not cleaned.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --min-age-sec)
      MIN_AGE_SEC="${2:-}"
      shift 2
      ;;
    --execute|--apply)
      EXECUTE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[cleanup-stale-ros2-cli] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MIN_AGE_SEC}" in
  ''|*[!0-9]*)
    echo "[cleanup-stale-ros2-cli] --min-age-sec must be an integer" >&2
    exit 2
    ;;
esac

collect_stale_cli_pids() {
  ps -eo pid=,etimes=,pcpu=,args= | awk -v min_age="${MIN_AGE_SEC}" '
    {
      pid=$1
      age=$2
      pcpu=$3
      command=substr($0, index($0, $4))
      if (age < min_age) next
      if (command !~ /(^|[[:space:]])(ros2|\/opt\/ros\/humble\/bin\/ros2)([[:space:]]|$)/ && command !~ /(^|[[:space:]])tf2_echo([[:space:]]|$)/) next
      if (command !~ /(^|[[:space:]])(ros2|\/opt\/ros\/humble\/bin\/ros2)[[:space:]]+(topic|param|service|action|node|interface|lifecycle|doctor|bag)([[:space:]]|$)/ && command !~ /(^|[[:space:]])tf2_echo([[:space:]]|$)/) next
      if (command ~ /(^|[[:space:]])(ros2|\/opt\/ros\/humble\/bin\/ros2)[[:space:]]+(run|launch)([[:space:]]|$)/) next
      if (command ~ /cleanup_stale_ros2_cli|awk/) next
      print pid "|" age "|" pcpu "|" command
    }
  '
}

mapfile -t rows < <(collect_stale_cli_pids)

echo "[cleanup-stale-ros2-cli] mode=$([[ "${EXECUTE}" == "true" ]] && echo execute || echo dry-run) min_age_sec=${MIN_AGE_SEC}"
if [[ "${#rows[@]}" -eq 0 ]]; then
  echo "[cleanup-stale-ros2-cli] no stale ros2 CLI inspection commands found"
  exit 0
fi

printf '%-8s %-10s %-8s %s\n' "PID" "AGE_SEC" "%CPU" "COMMAND"
for row in "${rows[@]}"; do
  IFS='|' read -r pid age pcpu command <<<"${row}"
  printf '%-8s %-10s %-8s %s\n' "${pid}" "${age}" "${pcpu}" "${command}"
done

if [[ "${EXECUTE}" != "true" ]]; then
  echo "[cleanup-stale-ros2-cli] dry-run only; rerun with --execute to send TERM to the PIDs above"
  exit 0
fi

for row in "${rows[@]}"; do
  IFS='|' read -r pid _age _pcpu _command <<<"${row}"
  kill -TERM "${pid}" 2>/dev/null || true
done
echo "[cleanup-stale-ros2-cli] TERM sent to ${#rows[@]} stale ros2 CLI PID(s)"

"""Run the real cleanup selection, signaling flow, and top-level ordering.

Only ps/kill/wait/sleep/rm are replaced. No host processes are enumerated or
signaled, and no runtime status files are removed. The fake process table models
children created while their old runtime owner handles a shutdown signal.
"""

from __future__ import annotations

import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts" / "jetson" / "runtime_overlay" / "scripts"
CLEANUP = SCRIPTS / "stop_runtime_processes.sh"
MAIN_MARKER = 'if [[ "${1:-}" == "--check" ]]; then'


@pytest.fixture(scope="module")
def bash() -> str:
    candidates = []
    if sys.platform == "win32":
        candidates.append(Path("C:/Program Files/Git/bin/bash.exe"))
    discovered = shutil.which("bash")
    if discovered:
        candidates.append(Path(discovered))
    executable = next((path for path in candidates if path.exists()), None)
    if executable is None:
        pytest.skip("Bash is required to exercise the real cleanup script")
    return str(executable)


FAKE_BOUNDARIES = r'''
declare -A fake_command=()
declare -A fake_alive=()
declare -a fake_signals=()
fake_rm_calls=0

add_process() {
  fake_command["$1"]="$2"
  fake_alive["$1"]=1
}

case "$scenario" in
  late_cli)
    add_process 100001 'bash /scripts/run_common_services.sh'
    add_process 200001 'bash /scripts/run_navigation_runtime_services.sh'
    ;;
  late_node|stubborn)
    add_process 200001 '/opt/ros/humble/lib/nav2_controller/controller_server --ros-args -r __node:=controller_server'
    ;;
  tools)
    add_process 400001 'pkill -INT -f __node:=controller_server'
    add_process 400002 'pgrep -f robot_global_localization/global_localization_node'
    add_process 400003 'awk /amcl_scan_admission_node/ && !/awk/ {print $1}'
    add_process 400004 'awk -v node_name=__node:=amcl /nav2_amcl\/amcl/ && index($0,node_name)>0 {print $1}'
    ;;
  real_processes)
    add_process 500001 '/opt/ros/humble/lib/nav2_controller/controller_server --ros-args -r __node:=controller_server'
    add_process 500002 '/usr/bin/python3 /opt/ros/humble/bin/ros2 lifecycle set /amcl shutdown'
    add_process 500003 'bash /scripts/run_common_services.sh'
    # "awk" in a real node argument must not exempt the business process.
    add_process 500004 '/opt/ros/humble/lib/nav2_controller/controller_server --ros-args -r __node:=controller_server -p label:=awk'
    add_process 500005 '/usr/bin/python3 /scripts/observe_startup_context.py --minimum-sequence 0'
    ;;
  cameras)
    add_process 600001 'bash /scripts/run_orbbec_336l_depth.sh'
    add_process 600002 '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera336l serial_number:=CPC8563000LM'
    add_process 600003 '/opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=camera_container -r __ns:=/camera336l'
    add_process 600004 '/bin/bash /usr/local/bin/scripts/workspace-entrypoint.sh /bin/bash -lc source /opt/ros/humble/setup.bash exec ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera serial_number:=CV2T6610007C'
    add_process 600005 '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera serial_number:=CV2T6610007C'
    add_process 600006 '/opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=camera_container -r __ns:=/camera'
    add_process 600007 '/usr/bin/python3 /tools/observer.py --frame camera336l_depth_optical_frame'
    add_process 600008 '/opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=camera_container -r __ns:=/camera336l_other'
    add_process 600009 '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera336l_other'
    ;;
  occupancy_launch)
    add_process 700001 '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/launch/occupancy_localization.launch.py map_yaml:=/maps/map.yaml'
    add_process 700002 '/opt/ros/humble/lib/rclcpp_components/component_container_isolated --ros-args -r __node:=occupancy_grid_localizer_container'
    ;;
  empty) ;;
  *) printf 'unknown fake scenario: %s\n' "$scenario" >&2; exit 90 ;;
esac

ps() {
  if [[ "$#" == 4 && "$1" == -p && "$3 $4" == '-o pid=,ppid=,stat=,args=' ]]; then
    local pid
    for pid in ${2//,/ }; do
      [[ -n "${fake_command[$pid]+known}" ]] || return 91
      if [[ "${fake_alive[$pid]:-0}" == 1 ]]; then
        printf '%s 1 S %s\n' "$pid" "${fake_command[$pid]}"
      fi
    done
    return 0
  fi
  # Fail closed if production changes the schema: never fall through to host ps.
  if [[ "$*" != '-eo pid=,args=' ]]; then
    printf 'unsupported fake ps arguments: %s\n' "$*" >&2
    return 91
  fi
  local pid
  for pid in "${!fake_command[@]}"; do
    if [[ "${fake_alive[$pid]:-0}" == 1 ]]; then
      printf '%s %s\n' "$pid" "${fake_command[$pid]}"
    fi
  done
}

kill() {
  local signal="$1"
  shift
  local pid
  for pid in "$@"; do
    if [[ -z "${fake_command[$pid]+known}" ]]; then
      fake_signals+=("INVALID $signal $pid")
      return 92
    fi
    if [[ "$signal" == -0 ]]; then
      [[ "${fake_alive[$pid]:-0}" == 1 ]] || return 1
      continue
    fi
    fake_signals+=("$signal $pid")
    [[ "$scenario" != stubborn ]] || continue
    [[ "${fake_alive[$pid]:-0}" == 1 ]] || continue
    fake_alive["$pid"]=0
    if [[ "$pid" == 200001 && "$scenario" == late_cli ]]; then
      # The resident's cleanup calls AMCL --stop, which creates these after
      # stopping its runner/heartbeat/relay (run_amcl_shadow_localization.sh).
      add_process 300001 'timeout 2 ros2 lifecycle set /amcl shutdown'
      add_process 300002 '/usr/bin/python3 /opt/ros/humble/bin/ros2 lifecycle set /amcl shutdown'
    elif [[ "$pid" == 200001 && "$scenario" == late_node ]]; then
      add_process 200002 '/opt/ros/humble/lib/nav2_controller/controller_server --ros-args -r __node:=controller_server'
    elif [[ "$pid" == 700002 && "${fake_alive[700001]:-0}" == 1 ]]; then
      # A still-running launch owner may respawn its component during cleanup.
      add_process 700003 '/opt/ros/humble/lib/rclcpp_components/component_container_isolated --ros-args -r __node:=occupancy_grid_localizer_container'
    fi
  done
}

wait_pids_gone() {
  shift  # The timeout is replaced, not the real caller's snapshot selection.
  local pid
  for pid in "$@"; do
    [[ "${fake_alive[$pid]:-0}" != 1 ]] || return 1
  done
  return 0
}

sleep() { :; }
rm() { fake_rm_calls=$((fake_rm_calls + 1)); }

audit_exit() {
  local status=$?
  local record pid
  for record in "${fake_signals[@]}"; do
    printf 'AUDIT_SIGNAL %s\n' "$record"
  done
  for pid in "${!fake_command[@]}"; do
    if [[ "${fake_alive[$pid]:-0}" == 1 ]]; then
      printf 'AUDIT_ALIVE %s\n' "$pid"
    fi
  done
  printf 'AUDIT_RM_CALLS %s\n' "$fake_rm_calls"
  exit "$status"
}
trap audit_exit EXIT
'''


def run_cleanup(bash: str, scenario: str, *arguments: str):
    production = CLEANUP.read_text(encoding="utf-8")
    declarations, marker, main = production.partition(MAIN_MARKER)
    assert marker, "Revisit the fixture seam if the cleanup entry point changes"
    original_directory = 'SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"'
    assert declarations.count(original_directory) == 1
    declarations = declarations.replace(
        original_directory, f"SCRIPT_DIR={shlex.quote(SCRIPTS.as_posix())}", 1
    )
    # Definitions and the complete top-level control flow are production code.
    # Inject fake external boundaries after definitions, without copying the
    # common/node/CLI ordering or the --check/final failure decision into tests.
    harness = (
        declarations
        + f"\nscenario={shlex.quote(scenario)}\n"
        + FAKE_BOUNDARIES
        + "\n"
        + marker
        + main
    )
    result = subprocess.run(
        [bash, "--noprofile", "--norc", "-s", "--", *arguments],
        input=harness,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )
    assert "unsupported fake" not in result.stderr, result.stderr
    assert "INVALID" not in result.stdout, result.stdout
    return result


def audit_values(result, label: str) -> list[str]:
    prefix = f"AUDIT_{label} "
    return [line[len(prefix) :] for line in result.stdout.splitlines() if line.startswith(prefix)]


def test_amcl_stop_created_cli_is_swept_before_cleanup_success(bash):
    result = run_cleanup(bash, "late_cli")
    assert result.returncode == 0, result.stderr
    assert audit_values(result, "ALIVE") == []
    signaled = {record.split()[1] for record in audit_values(result, "SIGNAL")}
    assert {"100001", "200001", "300001", "300002"} <= signaled


def test_new_same_class_child_is_not_lost_when_old_snapshot_exits(bash):
    result = run_cleanup(bash, "late_node")
    assert result.returncode == 0, result.stderr
    assert audit_values(result, "ALIVE") == []
    signaled = {record.split()[1] for record in audit_values(result, "SIGNAL")}
    assert signaled == {"200001", "200002"}


def test_cleanup_and_discovery_tools_are_not_business_processes(bash):
    result = run_cleanup(bash, "tools")
    assert result.returncode == 0, result.stderr
    assert audit_values(result, "SIGNAL") == []
    assert set(audit_values(result, "ALIVE")) == {"400001", "400002", "400003", "400004"}


def test_real_nodes_cli_and_owner_remain_in_the_absence_proof(bash):
    result = run_cleanup(bash, "real_processes", "--check")
    assert result.returncode == 1
    for pid in ("500001", "500002", "500003", "500004", "500005"):
        assert pid in result.stderr
    assert audit_values(result, "SIGNAL") == []


@pytest.mark.parametrize("scenario, expected_status", [("empty", 0), ("real_processes", 1)])
def test_check_is_read_only_even_when_processes_remain(bash, scenario, expected_status):
    result = run_cleanup(bash, scenario, "--check")
    assert result.returncode == expected_status, result.stderr
    assert audit_values(result, "SIGNAL") == []
    assert audit_values(result, "RM_CALLS") == ["0"]


def test_genuinely_unstoppable_business_process_still_fails(bash):
    result = run_cleanup(bash, "stubborn")
    assert result.returncode == 1
    assert audit_values(result, "ALIVE") == ["200001"]
    assert {"-INT 200001", "-TERM 200001", "-KILL 200001"} <= set(
        audit_values(result, "SIGNAL")
    )
    assert "200001" in result.stderr
    assert '200001 1 S /opt/ros/humble/lib/nav2_controller/controller_server' in result.stderr


def test_cleanup_stops_navigation_camera_without_signaling_other_camera(bash):
    result = run_cleanup(bash, "cameras")
    assert result.returncode == 0, result.stderr
    signaled = {record.split()[1] for record in audit_values(result, "SIGNAL")}
    assert signaled == {"600001", "600002", "600003"}
    assert set(audit_values(result, "ALIVE")) == {
        "600004", "600005", "600006", "600007", "600008", "600009"
    }


def test_camera_absence_check_only_reports_navigation_camera(bash):
    result = run_cleanup(bash, "cameras", "--check")
    assert result.returncode == 1
    assert audit_values(result, "SIGNAL") == []
    for pid in ("600001", "600002", "600003"):
        assert pid in result.stderr
    for pid in ("600004", "600005", "600006", "600007", "600008", "600009"):
        assert pid not in result.stderr


def test_actual_occupancy_launch_owner_is_stopped_with_its_component(bash):
    result = run_cleanup(bash, "occupancy_launch")
    assert result.returncode == 0, result.stderr
    assert audit_values(result, "ALIVE") == []
    signaled = {record.split()[1] for record in audit_values(result, "SIGNAL")}
    assert {"700001", "700002"} <= signaled

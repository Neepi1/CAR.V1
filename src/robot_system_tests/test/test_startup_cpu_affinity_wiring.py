"""Exercise real shell wiring with a scheduling-CLI fake; never pin a process."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
OVERLAY = ROOT / "scripts/jetson/runtime_overlay"


def run_wiring(tmp_path, program):
    env = {
        key: value for key, value in os.environ.items()
        if not key.startswith("NJRH_") and key not in ("BASH_ENV", "ENV")
    }
    phase = tmp_path / "effective_phase"
    phase.write_text("starting\n", encoding="utf-8")
    executables = tmp_path / "fake_bin"
    executables.mkdir()
    # `exec` bypasses shell functions, so give it an executable shim as well.
    # The shim enters the exported fake; it never starts a real Python helper.
    for name in ("python3", "taskset"):
        shim = executables / name
        shim.write_text(f'#!/usr/bin/env bash\n{name} "$@"\n', encoding="utf-8")
        shim.chmod(0o755)
    env.update({
        "NJRH_OVERLAY_ROOT": OVERLAY.as_posix(),
        "NJRH_NAVIGATION_CPU_PROFILE": "navigation_5cpu",
        "NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE": (tmp_path / "no_override.env").as_posix(),
        "NJRH_STARTUP_CPU_SESSION": (tmp_path / "session.json").as_posix(),
        "NJRH_NAVIGATION_STARTUP_RECEIPT": (tmp_path / "receipt").as_posix(),
        "TEST_EFFECTIVE_PHASE": phase.as_posix(),
        "TEST_SCHEDULER_BIN": executables.as_posix(),
    })
    bash = "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
    assert bash and Path(bash).is_file()
    # The Python scheduling CLI and taskset are the process/OS seams of this
    # shell test. The CLI's real Linux PID/TID behavior has its own tests.
    harness = r'''
set -euo pipefail
python3() {
  [[ "$1" == */startup_cpu_affinity.py ]] || return 90
  shift
  local command="$1" session="" steady="" reason=""
  shift
  while (( $# )); do
    case "$1" in
      --session) session="$2"; shift 2 ;;
      --steady-cpus) steady="$2"; shift 2 ;;
      --reason) reason="$2"; shift 2 ;;
      --role) shift 2 ;;
      --no-descendants) shift ;;
      --pid) shift 2 ;;
      --) shift; break ;;
      *) return 91 ;;
    esac
  done
  [[ "$session" == "$NJRH_STARTUP_CPU_SESSION" ]] || return 92
  case "$command" in
    finish) printf 'steady\n' > "$TEST_EFFECTIVE_PHASE" ;;
    register) : ;;
    mask)
      if [[ "$(< "$TEST_EFFECTIVE_PHASE")" == starting ]]; then printf '0-7\n';
      else printf '%s\n' "$steady"; fi ;;
    exec)
      if [[ "$(< "$TEST_EFFECTIVE_PHASE")" == starting ]]; then
        TEST_EFFECTIVE_CPUSET=0-7 "$@"
      else TEST_EFFECTIVE_CPUSET="$steady" "$@"; fi ;;
    *) return 93 ;;
  esac
}
taskset() {
  [[ "$1" == -c ]] || return 94
  local mask="$2"
  shift 2
  TEST_EFFECTIVE_CPUSET="$mask" "$@"
}
export -f python3 taskset
if command -v cygpath >/dev/null 2>&1; then
  TEST_SCHEDULER_BIN="$(cygpath -u "$TEST_SCHEDULER_BIN")"
fi
export PATH="$TEST_SCHEDULER_BIN:$PATH"
source "$NJRH_OVERLAY_ROOT/scripts/cpu_affinity.sh"
source "$NJRH_OVERLAY_ROOT/scripts/navigation_startup_receipt.sh"
''' + program
    harness_path = tmp_path / "harness.sh"
    harness_path.write_text(harness, encoding="utf-8")
    result = subprocess.run(
        [bash, "--noprofile", "--norc", harness_path.as_posix()],
        env=env, capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result


def test_ready_receipt_finishes_boost_and_preserves_receipt(tmp_path):
    run_wiring(tmp_path, "report_navigation_startup_finished ready\n")
    assert (tmp_path / "receipt").read_text().strip() == "ready"
    assert (tmp_path / "effective_phase").read_text().strip() == "steady"


def test_initialization_retry_receipt_keeps_boost_and_steady_environment(tmp_path):
    result = run_wiring(tmp_path, r'''
report_navigation_startup_finished waiting_for_initialization
njrh_run_affined controller_server bash -c 'printf "effective=%s\n" "$TEST_EFFECTIVE_CPUSET"'
printf 'configured=%s\n' "$NJRH_CPUSET_CONTROLLER_SERVER"
''')
    assert (tmp_path / "receipt").read_text().strip() == "waiting_for_initialization"
    assert (tmp_path / "effective_phase").read_text().strip() == "starting"
    assert result.stdout.splitlines() == ["effective=0-7", "configured=1-3"]


@pytest.mark.parametrize("phase", ["ready", "reused", "waiting_for_localization"])
def test_late_background_launch_uses_steady_mask_after_terminal_receipt(tmp_path, phase):
    result = run_wiring(tmp_path, f'report_navigation_startup_finished {phase}\n' + r'''
njrh_start_affined_background child_pid controller_server bash -c \
  'printf "effective=%s\n" "$TEST_EFFECTIVE_CPUSET"'
wait "$child_pid"
printf 'configured=%s\n' "$NJRH_CPUSET_CONTROLLER_SERVER"
''')
    assert result.stdout.splitlines() == ["effective=1-3", "configured=1-3"]


@pytest.mark.parametrize("setup, expected", [
    ("unset NJRH_STARTUP_CPU_SESSION", "1-3"),
    ("export NJRH_NAVIGATION_CPU_PROFILE=site_default", "1-3"),
    ("export NJRH_CPU_AFFINITY_ENABLED=false", "unmodified"),
])
def test_no_session_nonfive_and_disabled_paths_do_not_borrow_cpus(tmp_path, setup, expected):
    result = run_wiring(tmp_path, setup + r'''
njrh_exec_affined controller_server bash -c \
  'printf "effective=%s\n" "${TEST_EFFECTIVE_CPUSET:-unmodified}"'
''')
    assert result.stdout.strip() == f"effective={expected}"
    assert (tmp_path / "effective_phase").read_text().strip() == "starting"


@pytest.mark.parametrize("finish_first, expected", [(False, "0-7"), (True, "1-3")])
def test_exec_wrapper_preserves_command_arguments_and_selects_current_phase(
    tmp_path, finish_first, expected
):
    program = "report_navigation_startup_finished ready\n" if finish_first else ""
    result = run_wiring(tmp_path, program + r'''
njrh_exec_affined controller_server bash -c \
  'printf "effective=%s|argument=%s\n" "$TEST_EFFECTIVE_CPUSET" "$1"' fixture 'one argument'
''')
    assert result.stdout.strip() == f"effective={expected}|argument=one argument"


@pytest.mark.parametrize("role, key, configured", [
    ("pgo_mapping", "PGO_MAPPING", "7"),
    ("arm_control", "ARM_CONTROL", "5-7"),
])
def test_nonnavigation_role_keeps_own_mask_even_with_inherited_session(
    tmp_path, role, key, configured
):
    result = run_wiring(tmp_path, f'export NJRH_CPUSET_{key}={configured}\n' + f'''
njrh_run_affined {role} bash -c 'printf "effective=%s\\n" "$TEST_EFFECTIVE_CPUSET"'
''')
    assert result.stdout.strip() == f"effective={configured}"
    assert (tmp_path / "effective_phase").read_text().strip() == "starting"


@pytest.mark.parametrize("receipt_setup", [
    "unset NJRH_NAVIGATION_STARTUP_RECEIPT",
    'NJRH_NAVIGATION_STARTUP_RECEIPT="$NJRH_NAVIGATION_STARTUP_RECEIPT/missing/file"',
])
def test_terminal_event_restores_without_a_usable_receipt_path(tmp_path, receipt_setup):
    run_wiring(tmp_path, receipt_setup + "\nreport_navigation_startup_finished ready\n")
    assert (tmp_path / "effective_phase").read_text().strip() == "steady"


@pytest.mark.parametrize("existing_relay", [False, True])
def test_amcl_rechecks_phase_change_instead_of_stopping_healthy_relay(tmp_path, existing_relay):
    source = (OVERLAY / "scripts/run_amcl_shadow_localization.sh").read_text(encoding="utf-8")
    function = source[source.index("start_scan_admission_relay() {"):
                      source.index("wait_for_scan_admission_status_ready() {")]
    result = run_wiring(tmp_path, f"TEST_REUSE_RELAY={int(existing_relay)}\n" + r'''
NJRH_RUNTIME_LOG_DIR="$(dirname "$TEST_EFFECTIVE_PHASE")"
SCAN_RELAY_LOG_FILE="$NJRH_RUNTIME_LOG_DIR/relay.log"
SCAN_RELAY_PID_FILE="$NJRH_RUNTIME_LOG_DIR/relay.pid"
SCAN_RELAY_IMPL=cpp
SCAN_RELAY_CPP_BIN="$BASH"
# ROS admission and process inventory are controlled fixture inputs; no ROS,
# real relay, process signals, or affinity syscalls are performed here.
amcl_startup_side_effect_guard() { return 0; }
scan_admission_enabled() { return 0; }
scan_relay_pid_from_file() { [[ "$TEST_REUSE_RELAY" == 0 ]] || printf '%s\n' "$$"; }
scan_relay_pid_matches_impl() { return 0; }
pid_alive() { return 0; }
amcl_budget_sleep() { return 0; }
nohup() { return 0; }
stop_pid_softly() { printf 'stopped\n' > "$NJRH_RUNTIME_LOG_DIR/stopped"; }
scan_relay_allowed_cpus() {
  # Deterministically finish AFTER the first expected-mask read but BEFORE
  # the first /proc read: old expected=0-7, new actual=0-1,4.
  printf 'steady\n' > "$TEST_EFFECTIVE_PHASE"
  printf '0-1,4\n'
}
''' + function + "\nstart_scan_admission_relay\nprintf 'relay_healthy\\n'\n")
    assert result.stdout.strip() == "relay_healthy"
    assert not (tmp_path / "stopped").exists()
    assert (tmp_path / "effective_phase").read_text().strip() == "steady"


def test_single_api_boundary_boosts_api_but_preserves_business_child_placement(tmp_path, capsys):
    # Reuse the isolated OS adapter; exercise the real public scheduling CLI.
    from test_startup_cpu_affinity import FakeSystem, load_module, new_session, register_pid

    module, system = load_module(), FakeSystem()
    # 100 common -> 200 unregistered supervisor -> 300 actual API.
    # Business children either apply their own non-nav placement (400), or
    # explicitly request a shared navigation role (500); neither borrows CPUs.
    system.nodes.update({
        400: (40, 300, {400: 40}),
        500: (50, 300, {500: 50}),
        600: (60, 500, {600: 60}),
    })
    system.masks.update({200: set(range(8)), 201: set(range(8)),
                         400: {6, 7}, 500: set(range(8)), 600: {7}})
    session = new_session(module, system, tmp_path, capsys)
    assert register_pid(module, system, session, 100, "0-1,4", "--role", "navigation_runtime_owner") == 0
    assert register_pid(module, system, session, 300, "0-1,4", "--role", "robot_api_server", "--no-descendants") == 0
    assert system.masks[300] == set(range(8))
    assert register_pid(module, system, session, 500, "3", "--role", "hesai_ros_driver") == 0
    assert system.masks[500] == {3}
    assert module.main(["finish", "--session", session, "--reason", "ready"], system=system) == 0
    assert system.masks[100] == system.masks[200] == system.masks[201] == system.masks[300] == {0, 1, 4}
    assert system.masks[400] == {6, 7}
    assert system.masks[500] == {3}
    assert system.masks[600] == {7}

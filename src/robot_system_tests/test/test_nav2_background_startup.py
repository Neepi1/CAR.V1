"""Exercise startup scheduling with fake process/ROS boundaries, never a robot."""

import subprocess

import pytest

from test_nav2_prestart_environment import run_outer_entry
from test_navigation_localization_startup import (
    SCRIPTS, bash_executable, shell_function, startup_harness,
)


BACKGROUND = "NJRH_NAV2_LIFECYCLE_BACKGROUND_START"
AFTER_STACK = "NJRH_NAV2_PRESTART_AFTER_LOCALIZATION_STACK"


def delayed_prestart_harness(tmp_path, setup=""):
    path = startup_harness(tmp_path)
    program = path.read_text(encoding="utf-8")
    program = program.replace(
        'ensure_localization_stack_ready_for_navigation() { sleep 0.15; }',
        'ensure_localization_stack_ready_for_navigation() { echo stack-ready >> events; }')
    marker = 'echo "[runtime-overlay] navigation start source='
    defaults = f'''
export {AFTER_STACK}=true
export NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=true
export NJRH_NAV_LOCAL_STATE_MODE=ekf
export NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
export {BACKGROUND}=true
export NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=true
start_prestarted_nav2_lifecycle_background() {{
  echo background-start >> events
  nav2_lifecycle_background_started=1
}}
'''
    program = program.replace(marker, defaults + setup + "\n" + marker, 1)
    path.write_text(program, encoding="utf-8", newline="\n")
    return path


@pytest.mark.parametrize("lifecycle_after_stack", ["true", "false"])
def test_deferred_cold_prestart_constructs_nav2_after_stack_before_trigger(tmp_path, lifecycle_after_stack):
    delayed_prestart_harness(tmp_path,
        f"export NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK={lifecycle_after_stack}")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.index("stack-ready") < events.index("nav") < events.index("trigger")
    assert events.index("stage:localization_stack_ready") < events.index("stage:nav2_layer_prestarted")
    assert events.index("stage:nav2_layer_started") < events.index("background-start") < events.index("trigger")
    assert events.count("nav") == 1


def test_slow_held_nav2_launch_does_not_serialize_initial_localization(tmp_path):
    # Exercise the real startup main and real background worker. The fake Nav2
    # process can finish construction only after the localization call begins.
    # This catches an accidental foreground receipt wait without timing races.
    real_functions = "\n".join(shell_function(name) for name in (
        "start_prestarted_nav2_lifecycle_background",
        "wait_for_prestarted_nav2_launch_hold_ready",
        "wait_for_prestarted_nav2_lifecycle_background",
        "activate_prestarted_nav2_lifecycle",
    ))
    path = delayed_prestart_harness(tmp_path, real_functions + '''
NJRH_NAV2_PRESTART_HOLD_READY_TIMEOUT_SEC=2
run_nav2_lifecycle_sequence() { echo lifecycle-dispatch >> events; }
nav_lifecycle_nodes_active_quick() { echo fallback >> events; return 1; }
write_nav2_lifecycle_ready_status() { echo READY >> events; }
trigger_global_localization_for_navigation() {
  echo trigger >> events
  echo "background-at-trigger:$nav2_lifecycle_background_started" >> events
  touch release-launch
}
''')
    (tmp_path / "run_nav2_navigation.sh").write_text('''#!/usr/bin/env bash
echo nav >> events
while [[ ! -f release-launch ]]; do sleep 0.01; done
echo held-receipt >> events
printf 'NAV2_HOLD_READY=true\\nNAV2_HOLD_READY_WRAPPER_PID=%s\\nNAV2_HOLD_READY_CONTROLLER_PID=%s\\n' "$$" "$$" > "$NJRH_NAV2_HOLD_READY_FILE"
exec sleep 30
''', encoding="utf-8", newline="\n")
    program = path.read_text(encoding="utf-8").replace(
        "echo continued >> events", "activate_prestarted_nav2_lifecycle\necho continued >> events")
    path.write_text(program, encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert "background-at-trigger:1" in events, events
    assert "lifecycle-dispatch" in events, events
    assert events.index("trigger") < events.index("held-receipt") < events.index("lifecycle-dispatch")
    assert events.index("lifecycle-dispatch") < events.index("READY") < events.index("continued")
    assert events.count("trigger") == events.count("nav") == events.count("lifecycle-dispatch") == 1
    assert "fallback" not in events


@pytest.mark.parametrize("env_text,override_text,expected", [
    ("", "", "false"),
    (f"{AFTER_STACK}=true\n", "", "true"),
    (f"{AFTER_STACK}=false\n", "", "false"),
    (f"{AFTER_STACK}=false\n", f"{AFTER_STACK}=true\n", "true"),
])
def test_outer_entry_forwards_deferred_prestart_with_disabled_default(
        tmp_path, env_text, override_text, expected):
    assert run_outer_entry(tmp_path, env_text, override_text)[AFTER_STACK] == expected


@pytest.mark.parametrize("setup,deferred", [
    (f"unset {AFTER_STACK}", False),
    (f"export {AFTER_STACK}=false", False),
    (f"export {AFTER_STACK}=on", True),
    ("navigation_start_source=api_resume", True),
    ("navigation_start_source=direct", False),
    ("export NJRH_POINTCLOUD_ACCEL_PROFILE=legacy", False),
    ("export NJRH_NAV_LOCAL_STATE_MODE=fastlio", False),
])
def test_deferred_prestart_respects_entrypoint_and_explicit_opt_out(tmp_path, setup, deferred):
    delayed_prestart_harness(tmp_path, setup + "\nensure_common_local_state_ready_for_navigation_start() { :; }")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert (events.index("nav") > events.index("stack-ready")) is deferred
    assert events.index("nav") < events.index("trigger")
    assert events.count("nav") == 1


def test_deferred_flag_does_not_override_explicit_prestart_disabled(tmp_path):
    delayed_prestart_harness(tmp_path, '''
export NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=false
trigger_global_localization_for_navigation() { echo trigger >> events; }
''')
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert "nav" not in events and "background-start" not in events
    assert "trigger" in events and "continued" in events


@pytest.mark.parametrize("handoff_when", ["stack", "launch_guard"])
def test_deferred_prestart_does_not_launch_old_map_during_handoff(tmp_path, handoff_when):
    setup = '''
floor_handoff_requested() { [[ -f handoff ]]; }
trigger_global_localization_for_navigation() {
  floor_handoff_guard || return 20
  echo unexpected-trigger >> events
  return 0
}
adopt_startup_floor_handoff() { echo adopted >> events; floor_startup_handoff_active=1; }
'''
    if handoff_when == "stack":
        setup += '''
ensure_localization_stack_ready_for_navigation() {
  echo stack-ready >> events
  touch handoff
}
'''
    else:
        setup += '''
floor_handoff_guard() { touch handoff; echo handoff-at-launch-guard >> events; return 1; }
'''
    delayed_prestart_harness(tmp_path, setup)
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert "nav" not in events and "background-start" not in events
    assert "unexpected-trigger" not in events
    assert events.count("adopted") == 1 and "continued" in events


def test_deferred_prestart_dead_owned_child_fails_before_trigger(tmp_path):
    delayed_prestart_harness(tmp_path)
    (tmp_path / "run_nav2_navigation.sh").write_text(
        '#!/usr/bin/env bash\necho nav-failed >> events\nexit 9\n', encoding="utf-8")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode != 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert "nav-failed" in events
    assert "trigger" not in events and "continued" not in events and "background-start" not in events
    assert events.count("clean-nav") == events.count("clean-loc") == 1


def test_deferred_prestart_keeps_existing_owned_pid_without_duplicate(tmp_path):
    delayed_prestart_harness(tmp_path, '''
sleep 30 &
navigation_pid=$!
owned_before=$navigation_pid
echo "owned:$owned_before" >> events
trigger_global_localization_for_navigation() {
  [[ "$navigation_pid" == "$owned_before" ]]
  kill -0 "$navigation_pid"
  echo existing-owner-kept >> events
  return 0
}
''')
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert "nav" not in events
    assert "existing-owner-kept" in events
    assert events.count("background-start") == 1


@pytest.mark.parametrize("env_text,override_text,expected", [
    ("", "", "false"),
    (f"{BACKGROUND}=true\n", "", "true"),
    (f"{BACKGROUND}=false\n", "", "false"),
    (f"{BACKGROUND}=false\n", f"{BACKGROUND}=true\n", "true"),
])
def test_outer_entry_forwards_background_policy_with_disabled_default(
        tmp_path, env_text, override_text, expected):
    assert run_outer_entry(tmp_path, env_text, override_text)[BACKGROUND] == expected


@pytest.mark.parametrize("background,after_stack", [
    ("true", "true"), ("true", "1"), ("true", "false"), ("false", "true"),
])
def test_actual_startup_obeys_requested_background_phase(tmp_path, background, after_stack):
    path = startup_harness(tmp_path)
    program = path.read_text(encoding="utf-8")
    program = program.replace(
        'ensure_localization_stack_ready_for_navigation() { sleep 0.15; }',
        'ensure_localization_stack_ready_for_navigation() { echo stack-ready >> events; }')
    marker = 'echo "[runtime-overlay] navigation start source='
    setup = f'''
export {BACKGROUND}={background}
export NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK={after_stack}
start_prestarted_nav2_lifecycle_background() {{
  echo background-start >> events
  nav2_lifecycle_background_started=1
}}
'''
    program = program.replace(marker, setup + marker, 1)
    path.write_text(program, encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.index("nav") < events.index("trigger")
    if background == "false":
        assert "background-start" not in events
    else:
        assert events.count("background-start") == 1
        assert (events.index("background-start") > events.index("stack-ready")) == (
            after_stack in ("true", "1"))
        assert events.index("background-start") < events.index("trigger")


def background_wait_program(worker_rc=0, probe_rc=0, handoff=False, active_handoff=False):
    program = '''set -euo pipefail
nav2_prestarted=1
nav2_lifecycle_background_started=1
navigation_pid=fake-nav
NJRH_NAV2_LIFECYCLE_BACKGROUND_ACTIVE_POLL_SEC=0.02
NJRH_NAV2_LIFECYCLE_BACKGROUND_ACTIVE_WAIT_SEC=1
floor_startup_handoff_active=0
floor_handoff_guard() { return 0; }
write_nav2_lifecycle_ready_status() { echo READY >> events; }
runtime_readiness_probe() { echo "probe:$2" >> events; return "$probe_rc"; }
floor_handoff_requested() { [[ -f handoff ]]; }
'''
    program += f'probe_rc={probe_rc}\nfloor_startup_handoff_active={int(active_handoff)}\n'
    program += "\n".join(shell_function(name) for name in (
        "nav_lifecycle_active_quick", "nav_lifecycle_nodes_active_quick",
        "wait_for_prestarted_nav2_lifecycle_background", "activate_prestarted_nav2_lifecycle",
    ))
    program += f'''
(sleep 0.2; {"touch handoff;" if handoff else ""}
 echo worker-finished >> events; exit {worker_rc}) &
nav2_lifecycle_bringup_pid=$!
echo "owned:$nav2_lifecycle_bringup_pid" >> events
rc=0
activate_prestarted_nav2_lifecycle || rc=$?
echo "result:$rc" >> events
[[ -z "$nav2_lifecycle_bringup_pid" ]]
[[ "$nav2_lifecycle_background_started" == 0 ]]
'''
    return program


def run_program(tmp_path, program, timeout=5):
    path = tmp_path / "case.sh"
    path.write_text(program, encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "case.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=timeout)
    events = (tmp_path / "events").read_text().splitlines()
    return result, events


def held_worker_program():
    program = '''set -euo pipefail
nav2_prestarted=1
nav2_lifecycle_background_started=0
nav2_lifecycle_bringup_pid=''
navigation_pid=''
exit_code=0
floor_startup_handoff_active=0
NJRH_NAV2_PRESTART_HOLD_READY_TIMEOUT_SEC=2
NJRH_NAV2_HOLD_READY_FILE="$PWD/held.env"
NJRH_NAV_RUNTIME_STOP_INT_ATTEMPTS=0
NJRH_NAV_RUNTIME_STOP_TERM_ATTEMPTS=5
floor_handoff_requested() { [[ -f handoff ]]; }
floor_handoff_guard() { ! floor_handoff_requested; }
log_startup_stage() { echo "stage:$1" >> events; }
write_runtime_map_context() { echo unexpected-context-write >> events; }
write_nav2_lifecycle_ready_status() { echo READY >> events; }
runtime_readiness_probe() { echo fallback >> events; return 0; }
run_nav2_lifecycle_sequence() {
  floor_handoff_guard || return 1
  echo lifecycle-dispatch >> events
}
publish_receipt() {
  printf 'NAV2_HOLD_READY=true\\nNAV2_HOLD_READY_WRAPPER_PID=%s\\nNAV2_HOLD_READY_CONTROLLER_PID=%s\\n' "$navigation_pid" "$navigation_pid" > held.env
}
'''
    program += "\n".join(shell_function(name) for name in (
        "start_prestarted_nav2_lifecycle_background",
        "wait_for_prestarted_nav2_launch_hold_ready",
        "wait_for_prestarted_nav2_lifecycle_background",
        "activate_prestarted_nav2_lifecycle",
        "ensure_navigation_layer_alive", "nav_lifecycle_nodes_active_quick",
        "nav_lifecycle_active_quick", "wait_for_child_exit", "terminate_child",
    ))
    program += '''
trap 'terminate_child "$nav2_lifecycle_bringup_pid" worker; terminate_child "$navigation_pid" launch' EXIT
sleep 30 &
navigation_pid=$!
'''
    return program


@pytest.mark.parametrize("failure", ["missing", "wrong_owner", "owner_exit"])
def test_held_receipt_failure_cannot_become_ready_from_an_active_graph(tmp_path, failure):
    program = held_worker_program() + '''
start_prestarted_nav2_lifecycle_background
'''
    if failure == "wrong_owner":
        program += '''
printf 'NAV2_HOLD_READY=true\\nNAV2_HOLD_READY_WRAPPER_PID=99999999\\nNAV2_HOLD_READY_CONTROLLER_PID=%s\\n' "$navigation_pid" > held.env
'''
    elif failure == "owner_exit":
        program += '''
builtin kill -TERM "$navigation_pid"
wait "$navigation_pid" || true
'''
    program += '''
rc=0
activate_prestarted_nav2_lifecycle || rc=$?
echo "result:$rc" >> events
[[ -z "$nav2_lifecycle_bringup_pid" ]]
[[ "$nav2_lifecycle_background_started" == 0 ]]
'''
    result, events = run_program(tmp_path, program)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "READY" not in events and "fallback" not in events, events
    assert "result:0" not in events and "lifecycle-dispatch" not in events
    assert "unexpected-context-write" not in events
    assert "not a child" not in result.stderr


def test_pending_receipt_keeps_one_worker_and_waits_before_ready(tmp_path):
    program = held_worker_program() + '''
start_prestarted_nav2_lifecycle_background
owned_worker=$nav2_lifecycle_bringup_pid
start_prestarted_nav2_lifecycle_background
[[ "$owned_worker" == "$nav2_lifecycle_bringup_pid" ]]
echo parent-continued >> events
[[ ! -f held.env ]]
publish_receipt
activate_prestarted_nav2_lifecycle
'''
    result, events = run_program(tmp_path, program)
    assert result.returncode == 0, result.stdout + result.stderr
    assert events.count("lifecycle-dispatch") == events.count("READY") == 1
    assert events.count("stage:nav2_lifecycle_worker_started") == 1
    assert events.index("parent-continued") < events.index("lifecycle-dispatch") < events.index("READY")
    assert "fallback" not in events and "unexpected-context-write" not in events


def test_term_cleans_actual_worker_while_launch_receipt_is_pending(tmp_path):
    program = held_worker_program() + "\n".join(shell_function(name) for name in (
        "cleanup", "on_signal",
    )) + '''
runtime_ready=0
cleanup_started=0
amcl_runtime_started=0
NJRH_AMCL_LOCALIZATION_MODE=disabled
initial_global_localization_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
localization_pid=''
stop_existing_standard_nav_stack() { echo clean-nav >> events; }
stop_existing_localization_stack() { echo clean-loc >> events; }
kill() { echo "signal:$*" >> events; builtin kill "$@"; }
trap on_signal INT TERM
start_prestarted_nav2_lifecycle_background
echo "owned:$nav2_lifecycle_bringup_pid" >> events
builtin kill -TERM "$$"
echo unexpected-continuation >> events
'''
    result, events = run_program(tmp_path, program)
    assert result.returncode == 130, result.stdout + result.stderr
    owned = next(event.split(":", 1)[1] for event in events if event.startswith("owned:"))
    assert f"signal:-INT {owned}" in events
    assert events.count("clean-nav") == events.count("clean-loc") == 1
    assert "lifecycle-dispatch" not in events and "READY" not in events
    assert "unexpected-continuation" not in events


def test_handoff_during_pending_receipt_does_not_activate_old_map(tmp_path):
    program = held_worker_program() + f'''
SCRIPT_DIR=unused
NJRH_FLOOR_STARTUP_HANDOFF_FILE="$PWD/handoff"
source "{(SCRIPTS / 'floor_startup_handoff_helpers.sh').as_posix()}"
amcl_resident_pid=''
amcl_readiness_pid=''
floor_handoff_cli() {{
  case "$1" in
    export-env) echo 'export NJRH_RUNTIME_TRANSACTION_ID=txn NJRH_MAP_ID=target' ;;
    ack) echo "ack:$3" >> events ;;
    *) return 0 ;;
  esac
}}
timeout() {{ echo unexpected-lifecycle-client >> events; return 0; }}
''' + shell_function("run_nav2_lifecycle_sequence") + '''
start_prestarted_nav2_lifecycle_background
touch handoff
publish_receipt
rc=0
adopt_startup_floor_handoff || rc=$?
echo "result:$rc" >> events
[[ -z "$nav2_lifecycle_bringup_pid" ]]
'''
    result, events = run_program(tmp_path, program)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ack:failed" in events and "result:1" in events
    assert "ack:adopted" not in events
    assert "unexpected-lifecycle-client" not in events and "READY" not in events
    assert "unexpected-context-write" not in events


def test_success_waits_for_owned_worker_without_creating_extra_ros_clients(tmp_path):
    result, events = run_program(tmp_path, background_wait_program())
    assert result.returncode == 0, result.stdout + result.stderr
    assert not any(event.startswith("probe:") for event in events), events
    assert events.index("worker-finished") < events.index("READY")
    assert "result:0" in events


@pytest.mark.parametrize("probe_rc,handoff,active_handoff,expected_probes,ready", [
    (0, False, False, 7, True),
    (1, False, False, 1, False),
    (0, True, False, 0, False),
    (0, False, True, 0, False),
])
def test_failed_worker_has_one_fallback_pass_only_without_handoff(
        tmp_path, probe_rc, handoff, active_handoff, expected_probes, ready):
    result, events = run_program(tmp_path, background_wait_program(
        worker_rc=9, probe_rc=probe_rc, handoff=handoff, active_handoff=active_handoff))
    assert result.returncode == 0, result.stdout + result.stderr
    probes = [event for event in events if event.startswith("probe:")]
    assert len(probes) == expected_probes, events
    assert len(probes) == len(set(probes)), "no node may be polled repeatedly"
    assert ("READY" in events) is ready
    assert ("result:0" in events) is ready
    assert events.index("worker-finished") < events.index("result:0" if ready else "result:1")


def test_term_interrupts_join_and_cleanup_retains_owned_worker_pid(tmp_path):
    # Only these fixture-owned children receive signals. The actual cleanup
    # functions run, but broad process sweeps are replaced at the boundary.
    program = background_wait_program().split("\n(sleep 0.2;", 1)[0]
    program += "\n".join(shell_function(name) for name in (
        "wait_for_child_exit", "terminate_child", "cleanup", "on_signal",
    ))
    program += '''
runtime_ready=0
cleanup_started=0
amcl_runtime_started=0
NJRH_AMCL_LOCALIZATION_MODE=disabled
NJRH_NAV_RUNTIME_STOP_INT_ATTEMPTS=0
NJRH_NAV_RUNTIME_STOP_TERM_ATTEMPTS=20
initial_global_localization_pid=''
navigation_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
localization_pid=''
stop_existing_standard_nav_stack() { echo clean-nav >> events; }
stop_existing_localization_stack() { echo clean-loc >> events; }
kill() { echo "signal:$*" >> events; builtin kill "$@"; }
sleep 30 &
nav2_lifecycle_bringup_pid=$!
echo "owned:$nav2_lifecycle_bringup_pid" >> events
owner_pid=$$
(sleep 0.1; builtin kill -TERM "$owner_pid") &
trap on_signal INT TERM
activate_prestarted_nav2_lifecycle
echo unexpected-continuation >> events
'''
    result, events = run_program(tmp_path, program)
    assert result.returncode == 130, result.stdout + result.stderr
    owned = next(event.split(":", 1)[1] for event in events if event.startswith("owned:"))
    assert f"signal:-INT {owned}" in events
    assert events.count("clean-nav") == events.count("clean-loc") == 1
    assert "READY" not in events and "unexpected-continuation" not in events
    assert not any(event.startswith("probe:") for event in events)


@pytest.mark.parametrize("worker_rc", [0, 9])
def test_handoff_joins_same_owned_worker_before_acknowledging(tmp_path, worker_rc):
    # The real floor handoff owns the join instead of treating an active graph
    # or a killed client as evidence that an old transition has completed.
    program = f'''set -euo pipefail
SCRIPT_DIR=unused
NJRH_FLOOR_STARTUP_HANDOFF_FILE="$PWD/request.json"
write_runtime_map_context() {{ echo stale-write >> events; }}
source "{(SCRIPTS / 'floor_startup_handoff_helpers.sh').as_posix()}"
nav2_lifecycle_bringup_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
(sleep 0.2; echo worker-finished >> events; exit {worker_rc}) &
nav2_lifecycle_bringup_pid=$!
floor_handoff_cli() {{
  case "$1" in
    export-env) echo 'export NJRH_RUNTIME_TRANSACTION_ID=txn NJRH_MAP_ID=target' ;;
    export-evidence) echo 'export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE=4' ;;
    ack) echo "ack:$3" >> events ;;
    *) return 0 ;;
  esac
}}
rc=0
adopt_startup_floor_handoff || rc=$?
echo "result:$rc" >> events
write_runtime_map_context ready true old
[[ -z "$nav2_lifecycle_bringup_pid" ]]
'''
    result, events = run_program(tmp_path, program)
    assert result.returncode == 0, result.stdout + result.stderr
    expected_ack = "ack:adopted" if worker_rc == 0 else "ack:failed"
    assert events.index("worker-finished") < events.index(expected_ack)
    assert ("result:0" in events) == (worker_rc == 0)
    assert "stale-write" not in events

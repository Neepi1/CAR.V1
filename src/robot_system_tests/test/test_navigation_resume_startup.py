"""Navigation resume regressions with inert processes and transport adapters."""

import subprocess

import pytest

from test_navigation_localization_startup import (
    SCRIPTS, bash_executable, shell_function, startup_harness,
)


@pytest.mark.parametrize("pending", ["inputs", "isaac"])
def test_retryable_initialization_never_publishes_terminal_failure(tmp_path, pending):
    path = startup_harness(tmp_path)
    program = path.read_text(encoding="utf-8")
    adapters = f'''
NAV2_MAP_YAML=/fixture/map.yaml
navigation_start_source=api_resume
ensure_common_local_state_ready_for_navigation_start() {{ :; }}
unset NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE
runtime_readiness_probe() {{
  if [[ "{pending}" == inputs && ! -f checked ]]; then touch checked; return 1; fi
  return 0
}}
wait_for_isaac_startup_ready() {{
  if [[ "{pending}" == isaac && ! -f checked ]]; then touch checked; return 1; fi
  return 0
}}
'''
    marker = 'echo "[runtime-overlay] navigation start source='
    program = program.replace(marker,
        adapters + shell_function("set_localization_ready_failure") +
        shell_function("ensure_localization_stack_ready_for_navigation") + marker, 1)
    path.write_text(program, encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert "context:failed:false" not in events, events
    assert events.count("trigger") == 1
    assert "continued" in events


def test_terminal_readiness_failure_is_still_failed(tmp_path):
    program = '''set -euo pipefail
unset NJRH_RUNTIME_NONFATAL_LOCALIZATION_FAILURE
write_runtime_map_context() { printf '%s:%s\n' "$1" "$2"; }
''' + shell_function("set_localization_ready_failure") + '''
set_localization_ready_failure LOCAL_STATE_ENDPOINT_NOT_READY "owner missing"
'''
    result = subprocess.run([bash_executable(), "-c", program], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "failed:false"


@pytest.mark.parametrize("source,boosted", [("api_resume", True), ("systemd_autostart", False)])
def test_api_resume_creates_its_own_cpu_session_before_new_children(tmp_path, source, boosted):
    path = startup_harness(tmp_path)
    program = path.read_text(encoding="utf-8")
    marker = 'echo "[runtime-overlay] navigation start source='
    setup = f'''
navigation_start_source={source}
NJRH_NAV_LOCAL_STATE_MODE=ekf
NJRH_STARTUP_CPU_SESSION=inherited-closed-session
njrh_begin_startup_cpu_boost() {{
  [[ -z "${{NJRH_STARTUP_CPU_SESSION:-}}" ]]
  export NJRH_STARTUP_CPU_SESSION=resume-owned-session
  echo boost >> events
}}
njrh_finish_startup_cpu_boost() {{ echo "restore:$NJRH_STARTUP_CPU_SESSION" >> events; }}
ensure_common_local_state_ready_for_navigation_start() {{ :; }}
'''
    program = program.replace(marker, setup + marker, 1)
    path.write_text(program, encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert ("boost" in events) is boosted, events
    if boosted:
        assert events.index("boost") < events.index("loc")
        assert "restore:resume-owned-session" in events
        assert "restore:inherited-closed-session" not in events


def test_api_resume_defers_nav2_until_inputs_but_not_localization_result(tmp_path):
    path = startup_harness(tmp_path)
    program = path.read_text(encoding="utf-8")
    marker = 'echo "[runtime-overlay] navigation start source='
    setup = '''
navigation_start_source=api_resume
NJRH_NAV_LOCAL_STATE_MODE=ekf
NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
NJRH_NAV2_LIFECYCLE_BACKGROUND_START=true
NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=true
NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=false
ensure_common_local_state_ready_for_navigation_start() { :; }
ensure_localization_stack_ready_for_navigation() {
  # Input readiness is the scheduling boundary, not the localization result.
  [[ -z "$navigation_pid" ]] || { echo nav-before-inputs >> events; exit 43; }
  echo stack-ready >> events
}
start_prestarted_nav2_lifecycle_background() {
  echo "configure-all:${NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST:-false}" >> events
  nav2_lifecycle_background_started=1
}
'''
    program = program.replace(marker, setup + marker, 1)
    path.write_text(program, encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.count("configure-all:true") == 1, events
    assert events.index("stack-ready") < events.index("configure-all:true") < events.index("trigger")
    assert events.index("stack-ready") < events.index("nav") < events.index("trigger")


@pytest.mark.parametrize("wait,expected", [(None, ""), ("0", ""), ("3", "42")])
def test_optional_baseline_never_delays_default_initial_trigger(tmp_path, wait, expected):
    setup = "unset NJRH_INITIAL_LOCALIZATION_SEQUENCE_BASELINE_WAIT_SEC\n" if wait is None else (
        f"NJRH_INITIAL_LOCALIZATION_SEQUENCE_BASELINE_WAIT_SEC={wait}\n")
    program = '''set -euo pipefail
initial_global_localization_baseline_sequence=stale_previous_value
wait_for_bridge_relocalization_sequence_after() {
  echo observation >> events
  echo 42
}
''' + setup + shell_function("capture_initial_global_localization_baseline") + '''
capture_initial_global_localization_baseline
printf 'baseline=%s\n' "$initial_global_localization_baseline_sequence"
'''
    result = subprocess.run([bash_executable(), "-c", program], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == f"baseline={expected}"
    assert (tmp_path / "events").exists() is bool(expected)


def test_real_background_worker_overlaps_configure_and_joins_before_ready(tmp_path):
    path = startup_harness(tmp_path)
    program = path.read_text(encoding="utf-8")
    worker = "\n".join(shell_function(name) for name in (
        "start_prestarted_nav2_lifecycle_background",
        "wait_for_prestarted_nav2_launch_hold_ready",
        "wait_for_prestarted_nav2_lifecycle_background",
        "activate_prestarted_nav2_lifecycle"))
    setup = worker + '''
navigation_start_source=api_resume
NJRH_NAV_LOCAL_STATE_MODE=ekf
NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
NJRH_NAV2_LIFECYCLE_BACKGROUND_START=true
NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=true
NJRH_NAV2_PRESTART_HOLD_READY_TIMEOUT_SEC=2
ensure_common_local_state_ready_for_navigation_start() { :; }
ensure_localization_stack_ready_for_navigation() {
  echo stack-ready >> events
}
run_nav2_lifecycle_sequence() {
  [[ "$NJRH_NAV2_LIFECYCLE_CONFIGURE_ALL_FIRST" == true ]]
  echo configure >> events
  touch configuring
  # Simulate Nav2's existing activation wait for the localization-owned TF.
  while [[ ! -f localized ]]; do sleep 0.01; done
  echo active >> events
}
trigger_global_localization_for_navigation() {
  echo trigger >> events
  # Deterministic overlap: the request cannot finish before configuration.
  # Serializing Nav2 behind the result would deadlock this fixture.
  while [[ ! -f configuring ]]; do sleep 0.01; done
  touch localized
}
write_nav2_lifecycle_ready_status() { echo READY >> events; }
nav_lifecycle_nodes_active_quick() { echo unexpected-fallback >> events; return 1; }
'''
    marker = 'echo "[runtime-overlay] navigation start source='
    program = program.replace(marker, setup + marker, 1).replace(
        'echo continued >> events', 'activate_prestarted_nav2_lifecycle\necho continued >> events')
    path.write_text(program, encoding="utf-8", newline="\n")
    (tmp_path / "run_nav2_navigation.sh").write_text('''#!/usr/bin/env bash
echo nav >> events
printf 'NAV2_HOLD_READY=true\nNAV2_HOLD_READY_WRAPPER_PID=%s\nNAV2_HOLD_READY_CONTROLLER_PID=%s\n' "$$" "$$" > "$NJRH_NAV2_HOLD_READY_FILE"
exec sleep 30
''', encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    events = (tmp_path / "events").read_text().splitlines()
    ordered = ["stack-ready", "active", "READY", "continued"]
    assert [e for e in events if e in ordered] == ordered, events
    for concurrent in ("configure", "trigger"):
        assert events.index("stack-ready") < events.index(concurrent) < events.index("active")
    assert "unexpected-fallback" not in events


def test_resume_cpu_session_cannot_rebind_api_or_common_siblings(tmp_path, capsys):
    from test_startup_cpu_affinity import FakeSystem, load_module, register_pid

    module, system = load_module(), FakeSystem()
    # 100 common -> 200 API -> 300 new navigation -> 400 Nav2.
    # Arm (500) and the resident sensor (600) are not this new subtree.
    system.nodes.update({400: (40, 300, {400: 40}),
                         500: (50, 200, {500: 50}),
                         600: (60, 100, {600: 60})})
    system.masks.update({400: {0, 1, 4}, 500: {5, 6, 7}, 600: {3}})
    before = {pid: set(system.masks[pid]) for pid in (100, 200, 500, 600)}
    assert module.main(["begin", "--owner-pid", "300", "--session-dir", str(tmp_path)], system=system) == 0
    session = capsys.readouterr().out.strip()
    assert register_pid(module, system, session, 300, "0-1,4", "--role", "navigation_runtime_owner") == 0
    assert register_pid(module, system, session, 400, "0-1,4", "--role", "controller_server") == 0
    assert system.masks[300] == system.masks[400] == set(range(8))
    assert module.main(["finish", "--session", session, "--reason", "ready"], system=system) == 0
    assert system.masks[300] == system.masks[400] == {0, 1, 4}
    assert {pid: system.masks[pid] for pid in before} == before


def test_api_resume_terminal_localization_failure_finishes_without_later_wait(tmp_path):
    path = startup_harness(tmp_path, trigger_result=1)
    program = path.read_text(encoding="utf-8")
    setup = '''
navigation_start_source=api_resume
NJRH_NAV_LOCAL_STATE_MODE=ekf
NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
ensure_common_local_state_ready_for_navigation_start() { :; }
trigger_global_localization_for_navigation() {
  echo trigger >> events
  export NJRH_RUNTIME_FAILURE_CODE=BRIDGE_ACCEPT_TIMEOUT
  localization_ready_failure_reason="BRIDGE_ACCEPT_TIMEOUT: fixture result rejected"
  return 1
}
# Transport boundary: expose the old unwanted fallback without hanging a test.
python3() { echo unexpected-later-wait >> events; }
write_runtime_map_context() { printf 'context:%s:%s:%s\n' "$1" "$2" "$3" >> events; }
'''
    marker = 'echo "[runtime-overlay] navigation start source='
    path.write_text(program.replace(marker, setup + marker, 1), encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    events = (tmp_path / "events").read_text().splitlines()
    assert result.returncode == 1, result.stdout + result.stderr
    assert "unexpected-later-wait" not in events, events
    assert "continued" not in events, events
    assert events.count("trigger") == 1
    assert events.count("clean-nav") == events.count("clean-loc") == 1
    assert "context:failed:false:BRIDGE_ACCEPT_TIMEOUT: fixture result rejected" in events


@pytest.mark.parametrize("action", ["handoff", "cancel"])
def test_api_resume_keeps_handoff_and_cancel_out_of_terminal_failure(tmp_path, action):
    path = startup_harness(tmp_path)
    program = path.read_text(encoding="utf-8")
    setup = f'''
navigation_start_source=api_resume
NJRH_NAV_LOCAL_STATE_MODE=ekf
NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
NJRH_NAV2_LIFECYCLE_BACKGROUND_START=true
ensure_common_local_state_ready_for_navigation_start() {{ :; }}
start_prestarted_nav2_lifecycle_background() {{ nav2_lifecycle_background_started=1; }}
floor_handoff_requested() {{ [[ -f handoff ]]; }}
trigger_global_localization_for_navigation() {{
  echo trigger >> events
  if [[ "{action}" == cancel ]]; then kill -TERM "$$"; return 1; fi
  touch handoff
  return 20
}}
adopt_startup_floor_handoff() {{ echo adopted >> events; floor_startup_handoff_active=1; }}
python3() {{ echo unexpected-later-wait >> events; }}
'''
    marker = 'echo "[runtime-overlay] navigation start source='
    path.write_text(program.replace(marker, setup + marker, 1), encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    events = (tmp_path / "events").read_text().splitlines()
    assert result.returncode == (0 if action == "handoff" else 130), result.stdout + result.stderr
    assert ("adopted" in events) == (action == "handoff")
    assert ("continued" in events) == (action == "handoff")
    assert "stage:initial_global_localization_failed" not in events
    assert "unexpected-later-wait" not in events
    assert events.count("trigger") == 1

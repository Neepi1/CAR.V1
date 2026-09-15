"""Early AMCL orchestration uses only isolated process/ROS boundary fixtures."""

import subprocess

import pytest

from test_amcl_startup_sequence import run_shell
from test_navigation_localization_startup import (
    SCRIPTS, bash_executable, shell_function, startup_harness,
)
from test_nav2_prestart_environment import run_outer_entry


AMCL_EARLY_KEYS = (
    "NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION",
    "NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE",
)


# The IPC transport is the only status boundary replaced here. The actual
# producer still assembles owner, map, process and lifecycle/seed receipts.
UNAVAILABLE_STATUS_OBSERVER = r'''
STATUS_FILE="$PWD/status.env"
SCAN_RELAY_IMPL=cpp
SCAN_RELAY_CPP_BIN="$PWD/fake-relay"
SCAN_RELAY_PID_FILE="$PWD/relay.pid"
AMCL_EXIT_READY=0
AMCL_EXIT_FAILED=20
AMCL_EXIT_SCAN_ADMISSION_FAILED=22
AMCL_EXIT_LIFECYCLE_FAILED=23
AMCL_EXIT_PENDING=26
AMCL_PID_STALE_CLEARED=false
SCAN_ADMISSION_PID_STALE_CLEARED=false
AMCL_SEED_SUCCEEDED=false
AMCL_SEED_RESPONSE_OK=false
AMCL_NOMOTION_PROBE_USED=false
AMCL_NOMOTION_POSE_RECEIVED=false
AMCL_NOMOTION_POSE_COUNT=0
AMCL_NOMOTION_POSE_HEADER_AGE_MS=''
AMCL_STATIC_STANDBY_ACCEPTED=false
AMCL_STARTUP_EPOCH_SEC=1
scan_admission_enabled() { return 0; }
amcl_status_cli() {
  [[ "$1" == submit ]] || return 91
  local argument
  for argument in "$@"; do
    case "$argument" in
      AMCL_START_RESULT=*) echo "status-submit:${argument#*=}" >> events ;;
    esac
  done
  echo '[runtime-amcl-status] AMCL status observer unavailable' >&2
  return 1
}
'''


def test_early_resident_starts_without_status_observer(tmp_path):
    body = UNAVAILABLE_STATUS_OBSERVER + r'''
NJRH_AMCL_START_SETTLE_SEC=0.01
amcl_pid_from_file() { [[ -s "$PID_FILE" ]] && cat "$PID_FILE"; }
pid_alive() { kill -0 "$1" 2>/dev/null; }
# A live, isolated process replaces only the AMCL executable launch. It owns
# no ROS context and is always reaped by this test's parent shell.
nohup() {
  [[ "$1" == taskset && "$4" == "$AMCL_BIN" ]] || return 92
  echo resident-launched >> events
  sleep 3
}
trap 'if [[ -s "$PID_FILE" ]]; then fixture_pid="$(cat "$PID_FILE")"; kill "$fixture_pid" 2>/dev/null || true; wait "$fixture_pid" 2>/dev/null || true; fi' EXIT
start_amcl_resident
pid_alive "$(cat "$PID_FILE")"
echo resident-available >> events
'''
    run, events = run_shell(tmp_path, body, functions=(
        "write_amcl_runtime_status", "finish_amcl_status",
        "activate_amcl_lifecycle", "start_amcl_node",
        "wait_for_amcl_tf_warmup", "start_amcl_resident"), timeout=6)
    assert run.returncode == 0, run.stdout + run.stderr
    assert events.count("resident-launched") == events.count("lifecycle") == 1, events
    assert "status-submit:starting" in events and "status-submit:waiting_seed" in events
    assert events[-1] == "resident-available", events
    assert "status-submit:ready" not in events
    assert "AMCL_READY" not in run.stderr


@pytest.mark.parametrize("state", ["ready", "disabled"])
def test_ready_requires_committed_status_evidence(tmp_path, state):
    body = UNAVAILABLE_STATUS_OBSERVER + f"\nfinal_state={state}\n" + r'''
rc=0
finish_amcl_status "$final_state" true false '' 0 || rc=$?
echo "finish-rc:$rc" >> events
'''
    run, events = run_shell(tmp_path, body, functions=(
        "write_amcl_runtime_status", "finish_amcl_status"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == [f"status-submit:{state}", "finish-rc:20"], events
    assert "AMCL_READY" not in run.stderr


@pytest.mark.parametrize("state,code", [("starting", 26), ("waiting_seed", 0)])
def test_progress_retains_result_without_status_observer(tmp_path, state, code):
    body = UNAVAILABLE_STATUS_OBSERVER + f"\nprogress_state={state}\nprogress_code={code}\n" + r'''
rc=0
finish_amcl_status "$progress_state" false false 'initialization in progress' "$progress_code" || rc=$?
echo "finish-rc:$rc" >> events
'''
    run, events = run_shell(tmp_path, body, functions=(
        "write_amcl_runtime_status", "finish_amcl_status"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == [f"status-submit:{state}", f"finish-rc:{code}"], events
    assert "AMCL_READY" not in run.stderr


def test_early_start_keeps_owner_guard_before_launch(tmp_path):
    body = UNAVAILABLE_STATUS_OBSERVER + r'''
amcl_startup_side_effect_guard() { return 1; }
rc=0
start_amcl_node || rc=$?
echo "start-rc:$rc" >> events
'''
    run, events = run_shell(tmp_path, body, functions=(
        "write_amcl_runtime_status", "activate_amcl_lifecycle", "start_amcl_node"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == ["start-rc:1"], events
    assert not (tmp_path / "amcl.pid").exists()


@pytest.mark.parametrize("strict", [False, True])
def test_resident_join_is_nonblocking_except_explicit_strict_mode(tmp_path, strict):
    body = r'''
floor_handoff_guard() { return 0; }
amcl_mode_for_navigation() { printf 'gated\n'; }
log_amcl_runtime_status() { :; }
amcl_readiness_pid=''
bash() {
  [[ "$1" == "$SCRIPT_DIR/run_amcl_shadow_localization.sh" ]] || return 91
  echo complete-runner >> events
  sleep 0.3
}
sleep 0.7 &
amcl_resident_pid=$!
start_amcl_readiness_background_if_enabled_for_navigation
echo "first-resident:${amcl_resident_pid:+retained}" >> events
echo "first-readiness:${amcl_readiness_pid:+started}" >> events
if [[ "$NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY" != true ]]; then
  kill -0 "$amcl_resident_pid"
  echo resident-still-running >> events
  sleep 0.8
  start_amcl_readiness_background_if_enabled_for_navigation
  [[ -z "$amcl_resident_pid" ]]
  [[ -n "$amcl_readiness_pid" ]]
  # The second owner is still running; maintenance cannot start another one.
  start_amcl_readiness_background_if_enabled_for_navigation
fi
wait_for_amcl_readiness_background_if_running
echo all-joined >> events
'''
    body = f'NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY={str(strict).lower()}\n' + body
    run, events = run_shell(tmp_path, body, filename="run_navigation_runtime_services.sh",
                            functions=("run_amcl_localization_step",
                                       "start_amcl_readiness_background_if_enabled_for_navigation",
                                       "wait_for_amcl_resident_background_if_running",
                                       "wait_for_amcl_readiness_background_if_running",
                                       "complete_amcl_readiness_if_enabled_for_navigation",
                                       "complete_amcl_readiness_with_retries_for_navigation"),
                            timeout=6)
    assert run.returncode == 0, run.stdout + run.stderr
    assert events.count("complete-runner") == 1, events
    assert events[-1] == "all-joined", events
    if strict:
        assert "first-resident:" in events and "first-readiness:started" in events
        assert "resident-still-running" not in events
    else:
        assert "first-resident:retained" in events and "first-readiness:" in events
        assert events.index("resident-still-running") < events.index("complete-runner")
    assert run.stderr.count("resident warmup background joined") == 1


@pytest.mark.parametrize("opt_out", [False, True])
def test_early_defaults_overlap_preload_and_complete_only_after_trigger(tmp_path, opt_out):
    path = startup_harness(tmp_path)
    harness = path.read_text(encoding="utf-8")
    marker = 'echo "[runtime-overlay] navigation start source='
    prefix, main = harness.split(marker, 1)
    main = marker + main
    source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    extra_main = source[source.index('log_startup_stage "initial_global_localization_ready"'):
                        source.index('if ! wait_for_nav2_layer_ready; then')]
    functions = ("start_startup_context_observer_if_enabled",
                 "amcl_mode_for_navigation", "run_amcl_localization_step",
                 "start_amcl_resident_if_enabled_for_navigation",
                 "start_amcl_resident_background_if_enabled_for_navigation",
                 "wait_for_amcl_resident_background_if_running",
                 "start_amcl_readiness_background_if_enabled_for_navigation",
                 "wait_for_amcl_readiness_background_if_running",
                 "complete_amcl_readiness_if_enabled_for_navigation",
                 "complete_amcl_readiness_with_retries_for_navigation")
    prefix += "\n" + "\n".join(shell_function(name) for name in functions)
    prefix += r'''
NJRH_AMCL_LOCALIZATION_MODE=gated
NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=false
unset NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION
unset NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE
register_amcl_status_for_navigation() { :; }
log_amcl_runtime_status() { :; }
activate_prestarted_nav2_lifecycle() { echo nav2-activate >> events; }
trigger_global_localization_for_navigation() {
  echo trigger >> events
  grep -qx nav events || return 42
  touch accepted
  echo trigger-accepted >> events
}
'''
    if opt_out:
        prefix += '\nNJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=false\n'
        prefix += 'NJRH_AMCL_READINESS_BEFORE_NAV2_LIFECYCLE=false\n'
    runner = r'''#!/usr/bin/env bash
set -euo pipefail
case "${@: -1}" in
  --start-resident)
    echo resident-start >> events
    while ! grep -qx nav events; do sleep 0.01; done
    sleep 0.05
    echo resident-finished >> events
    ;;
  --complete-readiness)
    [[ -f accepted ]] || exit 93
    echo complete-after-trigger >> events
    ;;
  *) exit 94 ;;
esac
'''
    (tmp_path / "run_amcl_shadow_localization.sh").write_text(runner, encoding="utf-8")
    (tmp_path / "amcl_startup_progress.sh").write_text(
        (SCRIPTS / "amcl_startup_progress.sh").read_text(encoding="utf-8"), encoding="utf-8")
    main = main.replace('\necho continued >> events\n', '\n' + extra_main)
    main += '\nwait_for_amcl_readiness_background_if_running\necho continued >> events\n'
    path.write_text(prefix + main, encoding="utf-8")
    run = subprocess.run([bash_executable(), path.name], cwd=tmp_path,
                         capture_output=True, text=True, timeout=8)
    assert run.returncode == 0, run.stdout + run.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.index("nav") < events.index("trigger-accepted") < events.index("nav2-activate")
    if opt_out:
        assert "resident-start" not in events and "complete-after-trigger" not in events
        assert "stage:amcl_readiness_deferred" in events
    else:
        assert events.count("resident-start") == events.count("complete-after-trigger") == 1, events
        assert events.index("nav") < events.index("resident-finished"), events
        assert events.index("loc") < events.index("resident-finished"), events
        assert events.index("trigger-accepted") < events.index("complete-after-trigger"), events
        assert events.index("stage:amcl_readiness_started") < events.index("nav2-activate"), events


@pytest.mark.parametrize("key", AMCL_EARLY_KEYS)
@pytest.mark.parametrize("selection", ["default", "explicit_false", "override_false"])
def test_outer_entry_preserves_amcl_early_policy(tmp_path, key, selection):
    env_text = f"{key}=false\n" if selection == "explicit_false" else ""
    override_text = ""
    if selection == "override_false":
        env_text, override_text = f"{key}=true\n", f"{key}=false\n"
    values = run_outer_entry(tmp_path, env_text, override_text)
    assert values[key] == ("true" if selection == "default" else "false")
    other = next(candidate for candidate in AMCL_EARLY_KEYS if candidate != key)
    assert values[other] == "true"


@pytest.mark.parametrize("selection", [None, "false"])
def test_common_autostart_passes_amcl_early_policy_to_child(tmp_path, selection):
    child = "#!/usr/bin/env bash\nset -euo pipefail\n"
    for key in AMCL_EARLY_KEYS:
        child += f'printf "{key}=%s\\n" "${{{key}}}" >> events\n'
    (tmp_path / "run_navigation_runtime_services.sh").write_text(child, encoding="utf-8")
    body = "\n".join(
        f"unset {key}" if selection is None else f"{key}={selection}"
        for key in AMCL_EARLY_KEYS)
    body += r'''
RESIDENT_NAVIGATION_AUTOSTART=true
resident_navigation_autostart_started=0
autostart_building_id=B10
autostart_floor_id=F1
autostart_map_id=test-103
autostart_display_name=test-103
NJRH_COMMON_SERVICES_MANAGED=true
NJRH_NAVIGATION_STARTUP_RECEIPT="$PWD/startup.phase"
resolve_resident_navigation_autostart_selection() { :; }
resolve_floor_assets_if_needed() { :; }
prepare_resident_navigation_autostart() { :; }
# Execute the real env/child command; only process-manager ownership is faked.
start_common_process() {
  [[ "$1" == resident_navigation_runtime ]] || return 91
  shift 2
  "$@"
  common_last_started_pid=$$
}
start_resident_navigation_autostart_if_selected
start_resident_navigation_autostart_if_selected
'''
    run, events = run_shell(tmp_path, body, filename="run_common_services.sh",
                            functions=("start_resident_navigation_autostart_if_selected",))
    assert run.returncode == 0, run.stdout + run.stderr
    expected = selection or "true"
    assert events == [f"{key}={expected}" for key in AMCL_EARLY_KEYS], events

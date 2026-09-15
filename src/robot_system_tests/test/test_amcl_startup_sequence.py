"""Run actual AMCL startup Bash functions against offline ROS/process fixtures.

No ROS context, robot command, network call or production /tmp file is used.
NJRH_AMCL_TEST_SCRIPT_DIR can select a preserved pre-fix script tree for red tests.
"""

import os
import re
import shutil
import subprocess
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(os.environ.get(
    "NJRH_AMCL_TEST_SCRIPT_DIR", ROOT / "scripts/jetson/runtime_overlay/scripts"))
RUNNER = "run_amcl_shadow_localization.sh"
RUNTIME = "run_navigation_runtime_services.sh"


def shell_function(name, filename=RUNNER, required=True):
    source = (SCRIPTS / filename).read_text(encoding="utf-8")
    match = re.search(rf"(?ms)^{name}\(\) \{{\n.*?^\}}\n", source)
    if not required and match is None:
        return ""
    assert match is not None, name
    return match.group(0)


def bash_executable():
    return "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")


def run_shell(tmp_path, body, functions=(), filename=RUNNER, timeout=20, python_body=None):
    helper = SCRIPTS / "amcl_startup_progress.sh"
    prefix = """set -euo pipefail
export TMPDIR="$PWD"
export NJRH_AMCL_STARTUP_PROGRESS_FILE="$PWD/progress.env"
export NJRH_AMCL_RUNTIME_STATUS_FILE="$PWD/status.env"
export NJRH_AMCL_TF_WARMUP_SEC=0.01
export NJRH_AMCL_READINESS_COMPLETION_TIMEOUT_SEC=60
SCRIPT_DIR="$PWD"
MODE=gated
PARAMS_FILE="$PWD/params.yaml"
PID_FILE="$PWD/amcl.pid"
LOG_FILE="$PWD/amcl.log"
NJRH_RUNTIME_LOG_DIR="$PWD/logs"
AMCL_NODE_NAME=amcl
AMCL_BIN="$PWD/fake-amcl"
AMCL_LIFECYCLE_HELPER="$PWD/lifecycle.py"
NJRH_AMCL_SCAN_FRAME_REQUIRED=lidar_level_link
NJRH_NAV_RUNTIME_OWNER_PID=$$
NJRH_AMCL_OWNER_PID=$$
NJRH_BUILDING_ID=B10
NJRH_FLOOR_ID=F1
NJRH_MAP_ID=test-103
TEST_IDENTITY=process-a-map-a
amcl_startup_side_effect_guard() { return 0; }
amcl_progress_identity() { printf '%s\\n' "$TEST_IDENTITY"; }
amcl_pid_from_file() { printf '123\\n'; }
amcl_process_pids() { printf '123\\n'; }
pid_alive() { return 0; }
write_amcl_runtime_status() { :; }
njrh_cpuset_for() { printf '0\\n'; }
njrh_apply_affinity_to_pids() { :; }
taskset() { return 0; }
effective_scan_topic() { printf '/scan_amcl\\n'; }
wait_for_amcl_tf_broadcast_false() { echo tf-param >> events; }
wait_for_occupancy_grid() { echo map >> events; }
wait_for_topic_message() { echo scan >> events; }
wait_for_tf_transform() { echo "tf:$1:$2" >> events; }
scan_frame_from_topic() { printf 'lidar_level_link\\n'; }
runtime_readiness_probe() {
  [[ "$1" == amcl-inputs ]] || return 93
  local phase
  local -a requested=()
  IFS=, read -r -a requested <<< "$5"
  for phase in "${requested[@]}"; do
    case "$phase" in
      MAP) echo map >> events ;;
      SCAN)
        echo scan >> events
        printf 'AMCL_INPUT_FRAME=lidar_level_link\\n'
        ;;
      MAP_TF) echo tf:map:odom >> events ;;
      ODOM_TF) echo tf:odom:base_link >> events ;;
      SENSOR_TF) echo tf:base_link:lidar_level_link >> events ;;
      *) return 94 ;;
    esac
    printf 'AMCL_INPUT_READY=%s\\n' "$phase"
  done
}
start_scan_admission_relay() { echo relay >> events; }
wait_for_scan_admission_status_ready() { :; }
wait_for_fresh_amcl_scan_input() { :; }
wait_for_amcl_pose_fresh_or_nomotion_update() { return 0; }
"""
    # Only test fixture files are created; the executable never opens ROS.
    (tmp_path / "params.yaml").write_text("amcl: {}\n", encoding="utf-8")
    (tmp_path / "lifecycle.py").write_text("# offline fixture\n", encoding="utf-8")
    binary = tmp_path / "fake-amcl"
    binary.write_text("#!/usr/bin/env bash\nexit 99\n", encoding="utf-8")
    binary.chmod(0o755)
    fixture_bin = tmp_path / "bin"
    fixture_bin.mkdir(exist_ok=True)
    python = fixture_bin / "python3"
    python.write_text("#!/usr/bin/env bash\n" + (python_body or
                      "echo lifecycle >> events\nexit 0\n"), encoding="utf-8")
    python.chmod(0o755)
    prefix += 'export PATH="$PWD/bin:$PATH"\n'
    if helper.exists():
        # The helper is pure shell. Override its process identity boundary only.
        prefix += f'source "{helper.as_posix()}"\n'
        prefix += "amcl_progress_identity() { printf '%s' \"$TEST_IDENTITY\" | sha256sum | awk '{print $1}'; }\n"
        # Functional identity tests spawn many Git Bash utilities on Windows;
        # they must not accidentally become 6-second wall-clock deadline tests.
        # The two deadline cases below explicitly retain their 0.4-second budget.
        prefix += 'amcl_budget_begin 60\n'
        shutil.copyfile(helper, tmp_path / helper.name)
    prefix += shell_function("amcl_preparation_step", required=False)
    prefix += "\n".join(shell_function(name, filename) for name in functions)
    script = tmp_path / "test.sh"
    script.write_text(prefix + "\n" + body, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=timeout)
    events = ((tmp_path / "events").read_text().splitlines()
              if (tmp_path / "events").exists() else [])
    return run, events


def test_already_active_completion_does_not_repeat_lifecycle_query(tmp_path):
    run, events = run_shell(tmp_path, "start_amcl_node\nstart_amcl_node\n", functions=(
        "activate_amcl_lifecycle", "start_amcl_node"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events.count("lifecycle") == 1, events
    assert events.count("tf-param") == 1, events


def test_seed_retry_does_not_repeat_completed_map_scan_tf_preparation(tmp_path):
    body = """
seed_calls=0
seed_amcl_initial_pose() {
  seed_calls=$((seed_calls + 1))
  echo seed >> events
  [[ "$seed_calls" -ge 2 ]]
}
if complete_amcl_readiness_sequence; then exit 99; fi
complete_amcl_readiness_sequence
"""
    run, events = run_shell(tmp_path, body, functions=(
        "wait_for_amcl_tf_warmup", "amcl_static_standby_fast_seed_enabled",
        "complete_amcl_readiness_sequence"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events.count("seed") == 2, events
    for stage in ["map", "scan", "tf:map:odom", "tf:odom:base_link",
                  "tf:base_link:lidar_level_link"]:
        assert events.count(stage) == 1, events


def test_background_enters_one_complete_sequence_not_two_startup_phases(tmp_path):
    body = """
amcl_readiness_pid=''
amcl_resident_pid=''
floor_handoff_guard() { return 0; }
amcl_mode_for_navigation() { printf 'gated\\n'; }
start_amcl_resident_if_enabled_for_navigation() { echo resident >> events; }
complete_amcl_readiness_with_retries_for_navigation() { echo complete >> events; }
start_amcl_readiness_background_if_enabled_for_navigation
wait "$amcl_readiness_pid"
"""
    run, events = run_shell(tmp_path, body, filename=RUNTIME, functions=(
        "start_amcl_readiness_background_if_enabled_for_navigation",))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == ["complete"]


@pytest.mark.skipif(not (SCRIPTS / "amcl_startup_progress.sh").exists(),
                    reason="pre-fix baseline has no preparation checkpoint helper")
@pytest.mark.parametrize("new_identity", [
    "process-b-map-a", "process-a-map-b", "process-a-map-a-owner-b",
])
def test_new_process_map_or_owner_cannot_reuse_preparation(tmp_path, new_identity):
    body = f"""
start_amcl_node
wait_for_amcl_tf_warmup true
start_amcl_node
wait_for_amcl_tf_warmup true
echo boundary >> events
TEST_IDENTITY={new_identity}
start_amcl_node
wait_for_amcl_tf_warmup true
"""
    run, events = run_shell(tmp_path, body, functions=(
        "activate_amcl_lifecycle", "start_amcl_node", "wait_for_amcl_tf_warmup"))
    assert run.returncode == 0, run.stdout + run.stderr
    for stage in ["lifecycle", "tf-param", "map", "scan", "tf:map:odom",
                  "tf:odom:base_link", "tf:base_link:lidar_level_link"]:
        before = events[:events.index("boundary")]
        after = events[events.index("boundary") + 1:]
        assert before.count(stage) == 1, events
        assert after.count(stage) == 1, events


@pytest.mark.skipif(not (SCRIPTS / "amcl_startup_progress.sh").exists(),
                    reason="pre-fix baseline has no preparation checkpoint helper")
def test_same_identity_restores_checkpoints_in_a_new_shell_process(tmp_path):
    body = """
start_amcl_node
wait_for_amcl_tf_warmup true
if [[ "${1:-}" != child ]]; then
  echo new-shell >> events
  bash "$0" child
fi
"""
    run, events = run_shell(tmp_path, body, functions=(
        "activate_amcl_lifecycle", "start_amcl_node", "wait_for_amcl_tf_warmup"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events.count("new-shell") == 1
    for stage in ["lifecycle", "tf-param", "map", "scan", "tf:map:odom",
                  "tf:odom:base_link", "tf:base_link:lidar_level_link"]:
        assert events.count(stage) == 1, events


@pytest.mark.skipif(not (SCRIPTS / "amcl_startup_progress.sh").exists(),
                    reason="pre-fix baseline has no preparation checkpoint helper")
def test_unknown_process_or_owner_identity_disables_cache_reuse(tmp_path):
    body = """
start_amcl_node
wait_for_amcl_tf_warmup true
amcl_progress_identity() { return 1; }
start_amcl_node
wait_for_amcl_tf_warmup true
"""
    run, events = run_shell(tmp_path, body, functions=(
        "activate_amcl_lifecycle", "start_amcl_node", "wait_for_amcl_tf_warmup"))
    assert run.returncode == 0, run.stdout + run.stderr
    for stage in ["lifecycle", "tf-param", "map", "scan", "tf:map:odom",
                  "tf:odom:base_link", "tf:base_link:lidar_level_link"]:
        assert events.count(stage) == 2, events


def test_real_seed_retry_keeps_preparation_single_and_does_not_cache_seed(tmp_path):
    body = """
SEED_SERVICE=/seed
NJRH_AMCL_SEED_RETRY_PERIOD_MS=1
NJRH_AMCL_SEED_RETRY_COUNT=2
complete_amcl_readiness_sequence
[[ "$AMCL_SEED_SUCCEEDED" == true ]]
complete_amcl_readiness_sequence
"""
    python_body = """
echo seed-client >> events
if [[ ! -f first_seed_failed ]]; then
  touch first_seed_failed
  echo 'success=False message=transient'
  exit 1
fi
echo 'success=True message=seeded'
"""
    run, events = run_shell(tmp_path, body, python_body=python_body, functions=(
        "wait_for_amcl_tf_warmup", "amcl_static_standby_fast_seed_enabled",
        "seed_amcl_initial_pose", "complete_amcl_readiness_sequence"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events.count("seed-client") == 3, events
    assert events.count("map") == events.count("scan") == 1, events


def test_seed_with_partial_budget_defers_without_launching_client_then_succeeds(tmp_path):
    body = """
SEED_SERVICE=/seed
NJRH_AMCL_SEED_SERVICE_WAIT_SEC=8
NJRH_AMCL_SEED_CALL_TIMEOUT_SEC=8
NJRH_AMCL_SEED_RETRY_COUNT=1
AMCL_SEED_SUCCEEDED=false
# Deterministic remaining client budget: 1080ms minus the 250ms return grace.
amcl_monotonic_ms() { printf '100000\\n'; }
NJRH_AMCL_STARTUP_DEADLINE_MS=101080
if seed_amcl_initial_pose; then first_rc=0; else first_rc=$?; fi
echo "first_rc:$first_rc" >> events
echo "first_seeded:$AMCL_SEED_SUCCEEDED" >> events
unset NJRH_AMCL_STARTUP_DEADLINE_MS
amcl_budget_begin 60
seed_amcl_initial_pose
echo "final_seeded:$AMCL_SEED_SUCCEEDED" >> events
"""
    python_body = """
echo seed-client >> events
echo 'success=True message=seeded'
"""
    run, events = run_shell(tmp_path, body, python_body=python_body,
                            functions=("seed_amcl_initial_pose",))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == ["first_rc:124", "first_seeded:false", "seed-client",
                      "final_seeded:true"], events


@pytest.mark.skipif(not (SCRIPTS / "amcl_startup_progress.sh").exists(),
                    reason="pre-fix baseline has no bounded-client helper")
def test_client_hard_timeout_retains_unrelated_resident_and_stops_only_client(tmp_path):
    body = """
sleep 5 &
resident=$!
trap 'kill "$resident" 2>/dev/null || true; wait "$resident" 2>/dev/null || true' EXIT
unset NJRH_AMCL_STARTUP_DEADLINE_MS
amcl_budget_begin 0.4
set +e
amcl_client_timeout 10 bash -c 'trap "" TERM; echo "$BASHPID" > client.pid; exec sleep 3'
client_rc=$?
set -e
echo "client_rc:$client_rc" >> events
kill -0 "$resident"
echo resident-alive >> events
if kill -0 "$(<client.pid)" 2>/dev/null; then
  echo client-survived >> events
  exit 91
fi
"""
    started = time.monotonic()
    run, events = run_shell(tmp_path, body, timeout=5)
    elapsed = time.monotonic() - started
    assert run.returncode == 0, run.stdout + run.stderr
    assert "resident-alive" in events and "client-survived" not in events, events
    assert any(event in events for event in ["client_rc:124", "client_rc:137"]), events
    assert elapsed < 2.0, elapsed


@pytest.mark.skipif(not (SCRIPTS / "amcl_startup_progress.sh").exists(),
                    reason="pre-fix baseline has no propagated deadline")
def test_readiness_budget_bounds_blocked_client_and_retains_resident(tmp_path):
    body = """
sleep 5 &
resident=$!
trap 'kill "$resident" 2>/dev/null || true; wait "$resident" 2>/dev/null || true' EXIT
NJRH_AMCL_READINESS_COMPLETION_TIMEOUT_SEC=0.4
NJRH_AMCL_READINESS_COMPLETION_RETRY_SEC=0.1
floor_handoff_guard() { return 0; }
amcl_mode_for_navigation() { printf 'gated\\n'; }
complete_amcl_readiness_if_enabled_for_navigation() {
  echo complete >> events
  amcl_client_timeout 10 bash -c 'sleep 3'
}
if complete_amcl_readiness_with_retries_for_navigation; then exit 90; else rc=$?; fi
echo "completion_rc:$rc" >> events
kill -0 "$resident"
echo resident-alive >> events
"""
    started = time.monotonic()
    run, events = run_shell(tmp_path, body, filename=RUNTIME, functions=(
        "complete_amcl_readiness_with_retries_for_navigation",), timeout=5)
    elapsed = time.monotonic() - started
    assert run.returncode == 0, run.stdout + run.stderr
    assert events.count("complete") == 1, events
    assert "completion_rc:26" in events and "resident-alive" in events, events
    assert elapsed < 2.0, elapsed


def test_pending_completion_returns_to_supervisor_without_same_budget_retries(tmp_path):
    body = """
sleep 5 &
resident=$!
trap 'kill "$resident" 2>/dev/null || true; wait "$resident" 2>/dev/null || true' EXIT
NJRH_AMCL_READINESS_COMPLETION_TIMEOUT_SEC=1.2
NJRH_AMCL_READINESS_COMPLETION_RETRY_SEC=0.1
floor_handoff_guard() { return 0; }
amcl_mode_for_navigation() { printf 'gated\\n'; }
log_amcl_runtime_status() { :; }
# Only the external AMCL runner is replaced; all three production wrappers run.
bash() {
  [[ "$1" == "$SCRIPT_DIR/run_amcl_shadow_localization.sh" ]] || return 92
  [[ "${@: -1}" == --complete-readiness ]] || return 93
  echo runner-pending >> events
  return 26
}
if complete_amcl_readiness_with_retries_for_navigation; then exit 90; else rc=$?; fi
echo "completion_rc:$rc" >> events
kill -0 "$resident"
echo resident-alive >> events
"""
    started = time.monotonic()
    run, events = run_shell(tmp_path, body, filename=RUNTIME, functions=(
        "run_amcl_localization_step",
        "complete_amcl_readiness_if_enabled_for_navigation",
        "complete_amcl_readiness_with_retries_for_navigation"), timeout=5)
    elapsed = time.monotonic() - started
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == ["runner-pending", "completion_rc:26", "resident-alive"], events
    assert elapsed < 1.0, elapsed

"""Offline shell/process tests. No ROS, Docker, services, or robot commands."""

import os
from pathlib import Path
import subprocess

import pytest

from test_production_runtime_shell import _bash_executable, _bash_path


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts/jetson/runtime_overlay/scripts"


def run_startup(tmp_path: Path, body: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment.update(
        NJRH_COMMON_ENV_LOADED="1",
        NJRH_REUSE_COMMON_SERVICES="false",
        NJRH_RUNTIME_LOG_DIR=_bash_path(tmp_path),
        NJRH_OVERLAY_ROOT=_bash_path(tmp_path),
        NJRH_TEST_RELEASE=_bash_path(tmp_path / "release"),
    )
    program = r'''
set -euo pipefail
source "$1/canonical_tf_helpers.sh"
source "$1/common_startup_helpers.sh"
trap cleanup_common_startup_helpers EXIT
runtime_readiness_probe() {
  while [[ ! -f "${NJRH_TEST_RELEASE}" ]]; do sleep 0.02; done
}
''' + body
    harness = tmp_path / "harness.sh"
    harness.write_text(program, encoding="utf-8", newline="\n")
    return subprocess.run(
        [_bash_executable(), _bash_path(harness), _bash_path(SCRIPTS)],
        env=environment, capture_output=True, text=True, timeout=12,
        encoding="utf-8", errors="replace",
    )


def test_slow_chassis_readiness_does_not_prevent_other_helper_start(tmp_path: Path):
    result = run_startup(tmp_path, r'''
start_common_canonical_helper_background ranger_chassis_common sleep 30
start_common_canonical_helper_background robot_description_static_tf_common sleep 30
touch "${NJRH_TEST_RELEASE}"
wait_for_common_startup_job ranger_chassis_common
wait_for_common_startup_job robot_description_static_tf_common
echo parallel_start_verified
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "parallel_start_verified" in result.stdout


def test_successful_readiness_keeps_producer_owned_until_common_cleanup(tmp_path: Path):
    result = run_startup(tmp_path, r'''
start_common_canonical_helper_background robot_description_static_tf_common sleep 30
producer="${canonical_helper_launched_pid}"
wait_for_common_startup_job robot_description_static_tf_common
kill -0 "${producer}"
cleanup_common_startup_helpers
if kill -0 "${producer}" 2>/dev/null; then exit 91; fi
echo owned_producer_stopped
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "owned_producer_stopped" in result.stdout


def test_failed_readiness_cannot_be_reported_as_ready(tmp_path: Path):
    result = run_startup(tmp_path, r'''
runtime_readiness_probe() { return 17; }
start_common_canonical_helper_background ranger_chassis_common sleep 30
if wait_for_common_startup_job ranger_chassis_common; then exit 92; fi
echo readiness_failure_propagated
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "readiness_failure_propagated" in result.stdout


def test_slow_docking_check_overlaps_other_work_without_losing_its_result(tmp_path: Path):
    result = run_startup(tmp_path, r'''
start_common_startup_check docking_sensor runtime_readiness_probe
echo independent_work_completed
touch "${NJRH_TEST_RELEASE}"
wait_for_common_startup_job docking_sensor
echo docking_ready
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.splitlines() == ["independent_work_completed", "docking_ready"]


def test_cleanup_cancels_pending_readiness_and_producer(tmp_path: Path):
    result = run_startup(tmp_path, r'''
start_common_canonical_helper_background ranger_chassis_common sleep 30
producer="${canonical_helper_launched_pid}"
waiter="${common_startup_waiters[ranger_chassis_common]}"
cleanup_common_startup_helpers
if kill -0 "${producer}" 2>/dev/null; then exit 93; fi
if kill -0 "${waiter}" 2>/dev/null; then exit 94; fi
echo pending_startup_cleaned
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "pending_startup_cleaned" in result.stdout


def test_repeated_start_after_ready_does_not_replace_the_owned_producer(tmp_path: Path):
    result = run_startup(tmp_path, r'''
start_common_canonical_helper_background robot_description_static_tf_common sleep 30
producer="${canonical_helper_launched_pid}"
wait_for_common_startup_job robot_description_static_tf_common
start_common_canonical_helper_background robot_description_static_tf_common false
wait_for_common_startup_job robot_description_static_tf_common
[[ "${common_startup_producers[robot_description_static_tf_common]}" == "${producer}" ]]
''')
    assert result.returncode == 0, result.stdout + result.stderr


def test_reused_helper_is_not_adopted_or_stopped(tmp_path: Path):
    result = run_startup(tmp_path, r'''
NJRH_REUSE_COMMON_SERVICES=true
# Simulate the external process-inventory result. Static TF reuse has no ROS call.
canonical_process_running() { return 0; }
start_common_canonical_helper_background robot_description_static_tf_common false
wait_for_common_startup_job robot_description_static_tf_common
[[ ${#common_startup_producers[@]} == 0 ]]
[[ ${#common_startup_waiters[@]} == 0 ]]
cleanup_common_startup_helpers
echo reused_not_owned
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "reused_not_owned" in result.stdout


def test_cleanup_does_not_signal_a_pid_that_is_not_its_live_child(tmp_path: Path):
    result = run_startup(tmp_path, r'''
common_startup_producers[stale_record]="${BASHPID}"
cleanup_common_startup_helpers
echo unowned_pid_untouched
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "unowned_pid_untouched" in result.stdout


def test_synchronous_caller_preserves_common_owned_helper_after_readiness(tmp_path: Path):
    result = run_startup(tmp_path, r'''
start_canonical_helper robot_description_static_tf_common sleep 30
producer="${canonical_helper_launched_pid}"
cleanup_canonical_helpers
kill -0 "${producer}"
kill -TERM "${producer}"
wait "${producer}" 2>/dev/null || true
echo synchronous_behavior_preserved
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "synchronous_behavior_preserved" in result.stdout


@pytest.mark.skipif(os.name == "nt", reason="Requires Linux pgrep descendant inventory")
@pytest.mark.parametrize("stubborn_producer", [False, True])
def test_linux_cleanup_stops_nested_owned_processes(tmp_path: Path, stubborn_producer: bool):
    if stubborn_producer:
        launch = r'''
start_common_canonical_helper_background robot_description_static_tf_common \
  bash -c 'trap "" INT TERM; sleep 30 & echo $! > "${NJRH_TEST_RELEASE}"; wait'
'''
    else:
        launch = r'''
blocked_check() { sleep 30 & echo $! > "${NJRH_TEST_RELEASE}"; wait; }
start_common_startup_check blocking_probe blocked_check
'''
    result = run_startup(tmp_path, launch + r'''
while [[ ! -s "${NJRH_TEST_RELEASE}" ]]; do sleep 0.02; done
descendant="$(<"${NJRH_TEST_RELEASE}")"
cleanup_common_startup_helpers
state="$(ps -o stat= -p "${descendant}" 2>/dev/null || true)"
[[ -z "${state}" || "${state}" == Z* ]]
echo no_live_descendant
''')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "no_live_descendant" in result.stdout


def test_actual_common_flow_overlaps_base_helpers_before_main_services(tmp_path: Path):
    """Execute the real main sequence with fake driver and ROS boundaries."""
    fake_scripts = tmp_path / "scripts"
    fake_scripts.mkdir()
    for name in (
        "run_robot_description.sh", "run_ranger_chassis.sh", "run_local_state.sh",
        "run_pointcloud_accel_pipeline.sh", "run_orbbec_336l_depth.sh",
        "run_orbbec_docking_perception.sh",
    ):
        (fake_scripts / name).write_text("#!/usr/bin/env bash\nexec sleep 30\n", encoding="utf-8")
    sensors = tmp_path / "sensors.yaml"
    sensors.write_text("\n".join(
        "docking_camera_" + field + ": 0.0"
        for field in ("x", "y", "z", "roll", "pitch", "yaw")
    ), encoding="utf-8")
    common = (SCRIPTS / "run_common_services.sh").read_text(encoding="utf-8")
    process_launcher = common[common.index("start_common_process() {"):common.index("canonical_jt128_ingress_running() {")]
    local_state_start = common[common.index("start_robot_local_state_common() {"):common.index("resident_navigation_context_status() {")]
    main = common[common.index("require_can_interface_up\n"):]
    main = main[:main.index('echo "[runtime-overlay] local_perception_common disabled;')]
    body = f'SCRIPT_DIR="{_bash_path(fake_scripts)}"\nROBOT_DESCRIPTION_CONFIG_FILE="{_bash_path(sensors)}"\n' + r'''
common_pids=()
NAV_LOCAL_STATE_MODE=ekf
FASTLIO_AUTOSTART=false
DOCKING_SENSOR_BACKEND=orbbec_336l
NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
NJRH_LOCAL_STATE_START_READY_MODE=endpoint
NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART=true
NJRH_RESIDENT_NAVIGATION_EARLY_AUTOSTART=true
NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false
NJRH_REUSE_COMMON_SERVICES=false
require_can_interface_up() { :; }
stop_stale_pointcloud_accel_pipeline_processes() { :; }
stop_non_mapping_fastlio_runtime_processes() { :; }
rotate_runtime_log() { :; }
# External graph/process inventory is simulated, not read from the test host.
pgrep() { return 1; }
local_state_required_processes_running() { return 0; }
runtime_health_available() { return 1; }
runtime_health_check() { return 0; }
runtime_readiness_probe() { return 0; }
wait_for_runtime_health_local_state_endpoint_ready() { :; }
start_runtime_health_guard_common() { echo observer_started; }
log_common_startup_stage() { echo "stage=$1"; }
wait_for_fresh_header_topic_message() {
  while [[ ! -f "${NJRH_TEST_RELEASE}" ]]; do sleep 0.02; done
}
start_resident_navigation_autostart_if_selected() {
  echo navigation_started
  touch "${NJRH_TEST_RELEASE}"
}
test_cleanup() {
  for pid in "${common_pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  cleanup_common_startup_helpers
  for pid in "${common_pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
}
trap test_cleanup EXIT
''' + process_launcher + local_state_start + main
    result = run_startup(tmp_path, body)
    assert result.returncode == 0, result.stdout + result.stderr
    events = result.stdout.splitlines()
    assert events.index("stage=pointcloud_started") < events.index("stage=ranger_chassis_ready")
    assert events.index("observer_started") < events.index("stage=static_tf_ready")
    assert "stage=docking_sensor_started" not in events

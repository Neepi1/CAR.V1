"""Real common-start branch + real flock, with a fake camera; no ROS/hardware."""
import os
from pathlib import Path
import re

import pytest

from test_common_startup_parallel import SCRIPTS, run_startup
from test_production_runtime_shell import _bash_path

pytestmark = pytest.mark.skipif(os.name == "nt", reason="Real flock owner tests require Linux")


def _function(source, name):
    return re.search(rf"(?ms)^{name}\(\) \{{\n.*?^\}}\n", source).group(0)


def _run_camera_branch(tmp_path, *, fake_processes="", repeated=False, held=False, failure=False):
    source = Path(os.environ.get("NJRH_TEST_COMMON_SCRIPT", str(SCRIPTS / "run_common_services.sh"))).read_text(
        encoding="utf-8"
    )
    driver = (SCRIPTS / "run_orbbec_336l_depth.sh").read_text(encoding="utf-8")
    # Execute the unchanged real owner-lock block, never ROS/SDK initialization.
    lock = driver[driver.index("OWNER_LOCK_FILE=") : driver.index("set +u")]
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    (scripts / "run_orbbec_336l_depth.sh").write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\n" + lock +
        ("exit 7\n" if failure else 'echo launched >> "$NJRH_TEST_CAMERA_LAUNCHES"\nexec sleep 30\n'),
        encoding="utf-8", newline="\n",
    )
    sensors = tmp_path / "sensors.yaml"
    sensors.write_text("\n".join("docking_camera_" + axis + ": 0" for axis in
                                 ("x", "y", "z", "roll", "pitch", "yaw")), encoding="utf-8")
    definitions = source[source.index("start_common_process() {") :
                         source.index("canonical_jt128_ingress_running() {")]
    definitions += _function(source, "start_docking_common_last")
    body = f'''SCRIPT_DIR="{_bash_path(scripts)}"
ROBOT_DESCRIPTION_CONFIG_FILE="{_bash_path(sensors)}"
export NJRH_ORBBEC_CAMERA_OWNER_LOCK_FILE="${{NJRH_TEST_RELEASE}}.camera.lock"
export NJRH_TEST_CAMERA_LAUNCHES="${{NJRH_TEST_RELEASE}}.camera.launches"
FAKE_PROCESSES='{fake_processes}'
''' + r'''
common_pids=()
DOCKING_SENSOR_BACKEND=orbbec_336l
NJRH_DOCKING_MANAGER_AUTOSTART=false
NJRH_COMMON_PROCESS_START_SETTLE_SEC=0.2
reuse_common_services_enabled() { return 0; }
rotate_runtime_log() { :; }
pgrep() {
  [[ "$2" == camera336l ]] && [[ "$FAKE_PROCESSES" == *camera336l* ]]
}
log_common_startup_stage() { :; }
wait_for_fresh_header_topic_message() { return 0; }
cleanup_camera_test() {
  for pid in "${common_pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${common_pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup_camera_test EXIT
''' + definitions + r'''
# Perception is out of scope: stub only its launcher, retain the real camera call.
eval "$(declare -f start_common_process | sed '1s/start_common_process/original_common_start/')"
start_common_process() {
  [[ "$1" == orbbec_docking_perception ]] && return 0
  original_common_start "$@"
}
'''
    if held:
        body += 'exec 8>"$NJRH_ORBBEC_CAMERA_OWNER_LOCK_FILE"\nflock -n 8\n'
    body += 'start_docking_common_last\n'
    if repeated:
        body += 'start_docking_common_last\n'
    result = run_startup(tmp_path, body)
    launched = tmp_path / "release.camera.launches"
    count = len(launched.read_text().splitlines()) if launched.exists() else 0
    return result, count


@pytest.mark.parametrize("processes", [
    "",
    "robot_api_server_node docking_default_sensor_frame:=camera336l_depth_optical_frame",
    "ros2 launch orbbec_camera camera_name:=camera serial_number:=CV2T6610007C",
    "robot_api_server_node camera336l_depth_optical_frame; orbbec_camera camera_name:=camera",
])
def test_absent_camera_is_started_regardless_of_other_process_names(tmp_path, processes):
    result, count = _run_camera_branch(tmp_path, fake_processes=processes)
    assert result.returncode == 0, result.stdout + result.stderr
    assert count == 1, result.stdout + result.stderr


def test_existing_owner_lock_is_success_without_duplicate_driver(tmp_path):
    result, count = _run_camera_branch(tmp_path, held=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert count == 0


def test_repeated_start_keeps_exactly_one_driver(tmp_path):
    result, count = _run_camera_branch(tmp_path, repeated=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert count == 1


def test_real_start_failure_is_not_disguised_as_existing_camera(tmp_path):
    result, count = _run_camera_branch(tmp_path, failure=True)
    assert result.returncode != 0
    assert count == 0

"""Run the real observation checkpoint without ROS or hardware processes.

NJRH_TEST_COMMON_SCRIPT can point at a captured deployed script to replay its
exact failure policy before applying a narrowly scoped deployment patch.
"""

import os
from pathlib import Path
import subprocess

import pytest

from test_production_runtime_shell import _bash_executable, _bash_path


ROOT = Path(__file__).resolve().parents[3]


def observation_checkpoint(source):
    if "start_docking_common_last() {" in source:
        start = source.index('  log_common_startup_stage "docking_sensor_started"',
                             source.index("start_docking_common_last() {"))
        end = source.index('  if [[ "${NJRH_DOCKING_MANAGER_AUTOSTART', start)
    else:
        # The production parallel-start version joins the camera observation
        # before it proceeds to the other common services.
        start = source.index('wait_for_common_startup_job "docking_sensor"')
        # Include the whole conditional for the fixed version.
        if source[start - 3:start] == "if ":
            start -= 3
        end = source.index('echo "[runtime-overlay] local_perception_common disabled;', start)
    return source[start:end]


@pytest.mark.parametrize("observation_rc", [0, 1, 17, 143])
def test_missing_docking_observation_cannot_terminate_common(tmp_path, observation_rc):
    path = Path(os.environ.get("NJRH_TEST_COMMON_SCRIPT", str(
        ROOT / "scripts/jetson/runtime_overlay/scripts/run_common_services.sh")))
    checkpoint = observation_checkpoint(path.read_text(encoding="utf-8"))
    harness = tmp_path / "observation_scope.sh"
    harness.write_text(
        "set -euo pipefail\nDOCKING_SENSOR_BACKEND=orbbec_336l\n"
        'log_common_startup_stage() { echo "stage=$1"; }\n'
        f"wait_for_common_startup_job() {{ return {observation_rc}; }}\n"
        f"wait_for_fresh_header_topic_message() {{ return {observation_rc}; }}\n"
        + checkpoint + '\necho common_continues_to_other_services\n',
        encoding="utf-8", newline="\n")
    result = subprocess.run([_bash_executable(), _bash_path(harness)],
                            capture_output=True, text=True, timeout=5,
                            encoding="utf-8", errors="replace")
    assert result.returncode == 0, result.stdout + result.stderr
    lines = result.stdout.splitlines()
    assert "common_continues_to_other_services" in lines
    assert ("stage=docking_sensor_ready" in lines) == (observation_rc == 0)
    assert ("stage=docking_sensor_degraded" in lines) == (observation_rc != 0)

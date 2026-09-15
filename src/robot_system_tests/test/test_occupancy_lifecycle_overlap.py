"""Exercise occupancy startup ordering with isolated fake ROS processes."""

import re
import subprocess
import time

import pytest

from test_navigation_localization_startup import SCRIPTS, bash_executable


def occupancy_harness(tmp_path, *, map_exit=0, mode="ekf", profile="ipc_worker",
                      interrupt="", external_lifecycle=True):
    source = (SCRIPTS / "run_occupancy_grid_localization.sh").read_text(encoding="utf-8")
    definitions = []
    for name in (
        "wait_for_child_exit", "terminate_child",
        "localization_map_external_lifecycle_bringup_enabled",
        "start_map_server_lifecycle_with_nav2_util",
        "wait_for_map_server_lifecycle_with_nav2_util", "cleanup", "on_signal",
    ):
        match = re.search(rf"(?ms)^{name}\(\) \{{\n.*?^\}}\n", source)
        if match:
            definitions.append(match.group(0))
    main = source[source.index("# MapServer and Isaac can initialize") :]
    fixture_bin = tmp_path / "bin"
    fixture_bin.mkdir()
    client = fixture_bin / "python3"
    client.write_text(
        "#!/usr/bin/env bash\n"
        "echo map-client >> events\n"
        "echo $$ > map-client.pid\n"
        "while [[ ! -e map.release ]]; do sleep 0.02; done\n"
        "echo map-completed >> events\n"
        ": > map.done\n"
        f"exit {map_exit}\n", encoding="utf-8", newline="\n")
    client.chmod(0o755)
    prefix = r'''set -euo pipefail
SCRIPT_DIR="$PWD"
export PATH="$PWD/bin:$PATH"
NJRH_COMMON_SERVICES_MANAGED=true
LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP=true
LOCALIZATION_MAP_LIFECYCLE_BRINGUP_TIMEOUT_SEC=10
NJRH_LOCALIZATION_STOP_INT_ATTEMPTS=0
NJRH_LOCALIZATION_STOP_TERM_ATTEMPTS=3
LAUNCH_FILE=fixture.launch.py
launch_args=()
localization_pid=''
map_lifecycle_bringup_pid=''
localization_exit_code=0
helper_pids=()
ros2() {
  echo occupancy >> events
  while [[ ! -e map.done ]]; do sleep 0.02; done
}
start_canonical_helper() {
  echo "$1" >> events
  sleep 10 &
  helper_pids+=("$!")
}
start_overlay_helper() {
  start_canonical_helper "$@"
  [[ -z "$map_lifecycle_bringup_pid" ]] || echo "$map_lifecycle_bringup_pid" > map-owner.pid
  if [[ "$INTERRUPT" == cancel ]]; then
    while [[ ! -e map-client.pid ]]; do sleep 0.02; done
    kill -TERM "${BASHPID}"
  elif [[ "$INTERRUPT" == helper-failed ]]; then
    while [[ ! -e map-client.pid ]]; do sleep 0.02; done
    return 9
  fi
}
cleanup_localization_stack_patterns() { :; }
cleanup_overlay_helpers() {
  for child in "${helper_pids[@]}"; do terminate_child "$child" fixture-helper; done
  [[ -z "$map_lifecycle_bringup_pid" ]] || exit 94
  for record in map-client.pid map-owner.pid; do
    [[ ! -s "$record" ]] || ! kill -0 "$(<"$record")" 2>/dev/null || exit 95
  done
  echo cleanup-complete >> events
}
require_common_ranger_chassis_for_localization() { echo ranger-check >> events; }
require_common_static_tf_for_localization() { echo static-check >> events; }
require_common_pointcloud_for_localization() { echo pointcloud-check >> events; }
ensure_localization_pointcloud_ready() { echo points-check >> events; }
ensure_resident_fastlio_for_local_state() { echo fastlio-check >> events; }
ensure_resident_local_state_for_localization() { echo local-state-check >> events; }
'''
    prefix += (f"NAV_LOCAL_STATE_MODE={mode}\nNJRH_POINTCLOUD_ACCEL_PROFILE={profile}\n"
               f"INTERRUPT={interrupt}\n"
               f"LOCALIZATION_MAP_EXTERNAL_LIFECYCLE_BRINGUP={str(external_lifecycle).lower()}\n")
    program = "\n".join([prefix, *definitions, "trap cleanup EXIT", "trap on_signal INT TERM", main])
    (tmp_path / "startup.sh").write_text(program, encoding="utf-8", newline="\n")
    return subprocess.Popen([bash_executable(), "startup.sh"], cwd=tmp_path,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def events_at(tmp_path):
    path = tmp_path / "events"
    return path.read_text().splitlines() if path.exists() else []


def wait_event(tmp_path, expected, process, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if expected in events_at(tmp_path):
            return
        if process.poll() is not None:
            break
        time.sleep(0.02)
    raise AssertionError(f"missing {expected}: {events_at(tmp_path)}")


def release_and_join(tmp_path, process):
    (tmp_path / "map.release").touch()
    return process.communicate(timeout=8)


def test_slow_map_does_not_delay_bridge_or_wrapper_but_remains_unfinished(tmp_path):
    process = occupancy_harness(tmp_path)
    try:
        wait_event(tmp_path, "map-client", process)
        wait_event(tmp_path, "global_localization_localization", process, timeout=1)
        events = events_at(tmp_path)
        assert "robot_localization_bridge" in events
        assert "map-completed" not in events
        assert process.poll() is None
        assert "trigger" not in events and "ready" not in events
    finally:
        output, error = release_and_join(tmp_path, process)
    assert process.returncode == 0, output + error


def test_map_lifecycle_failure_is_not_reported_as_ready_and_cleans_helpers(tmp_path):
    process = occupancy_harness(tmp_path, map_exit=7)
    try:
        wait_event(tmp_path, "global_localization_localization", process)
    finally:
        output, error = release_and_join(tmp_path, process)
    assert process.returncode == 1, output + error
    assert "map_server active" not in error
    assert "failed or timed out" in error
    assert events_at(tmp_path)[-1] == "cleanup-complete"


@pytest.mark.parametrize("interrupt,expected", [("cancel", 130), ("helper-failed", 9)])
def test_interruption_before_map_join_cleans_lifecycle_child(tmp_path, interrupt, expected):
    process = occupancy_harness(tmp_path, interrupt=interrupt)
    try:
        output, error = process.communicate(timeout=8)
    finally:
        if process.poll() is None:
            release_and_join(tmp_path, process)
    assert process.returncode == expected, output + error
    events = events_at(tmp_path)
    assert "map-completed" not in events
    assert events[-1] == "cleanup-complete"


@pytest.mark.parametrize("mode,profile", [("fastlio", "ipc_worker"), ("ekf", "legacy")])
def test_diagnostic_launch_keeps_dependency_then_helper_then_occupancy_order(tmp_path, mode, profile):
    process = occupancy_harness(tmp_path, mode=mode, profile=profile)
    try:
        wait_event(tmp_path, "occupancy", process)
    finally:
        # Legacy's external map lifecycle is intentionally disabled; its ROS
        # launch owns that lifecycle. Release the isolated launch fixture too.
        (tmp_path / "map.done").touch()
        output, error = release_and_join(tmp_path, process)
    assert process.returncode == 0, output + error
    events = events_at(tmp_path)
    assert events.index("ranger-check") < events.index("robot_localization_bridge")
    assert events.index("global_localization_localization") < events.index("occupancy")
    if mode == "fastlio":
        assert events.index("fastlio-check") < events.index("robot_localization_bridge")
    if profile == "legacy":
        assert "map-client" not in events


def test_external_lifecycle_disabled_has_no_map_child_to_join(tmp_path):
    process = occupancy_harness(tmp_path, external_lifecycle=False)
    try:
        wait_event(tmp_path, "global_localization_localization", process)
    finally:
        (tmp_path / "map.done").touch()
        output, error = release_and_join(tmp_path, process)
    assert process.returncode == 0, output + error
    assert "map-client" not in events_at(tmp_path)

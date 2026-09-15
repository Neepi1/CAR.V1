"""Cold-start regression tests. Fake boundaries only, never use the live robot."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import pytest

from test_navigation_localization_startup import SCRIPTS, bash_executable, shell_function


def test_service_presence_cannot_admit_uninitialized_isaac(tmp_path):
    program = '''set -euo pipefail
NAV2_MAP_YAML=/test/map.yaml
NJRH_ISAAC_STARTUP_STATE_FILE="$PWD/isaac.json"
runtime_readiness_probe() { return 0; }
env_flag_true() { [[ "$1" == "true" ]]; }
wait_for_isaac_startup_ready() { return 1; }
set_localization_ready_failure() { echo "$1"; }
'''+shell_function("ensure_localization_stack_ready_for_navigation")+'''
if ensure_localization_stack_ready_for_navigation; then exit 91; fi
'''
    path = tmp_path / "test.sh"
    path.write_text(program, encoding="utf-8", newline="\n")
    result = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr


def test_map_and_isaac_launch_precede_sensor_readiness_wait():
    source = (SCRIPTS / "run_occupancy_grid_localization.sh").read_text()
    assert source.index('ros2 launch "${LAUNCH_FILE}"') < source.index(
        'require_common_ranger_chassis_for_localization || exit 1')


def test_common_launches_selected_map_branch_before_joining_sensor_readiness():
    source = (SCRIPTS / "run_common_services.sh").read_text()
    main = source[source.index("require_can_interface_up\n"):]
    assert main.index("start_resident_navigation_autostart_if_selected") < main.index(
        'wait_for_common_startup_job "robot_description_static_tf_common"')


def load_state_module():
    spec = importlib.util.spec_from_file_location("isaac_startup_state_test", SCRIPTS / "isaac_startup_state.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_marker_is_post_graph_only_and_handles_split_output(tmp_path, monkeypatch):
    module = load_state_module()
    monkeypatch.setattr(module, "process_start_ticks", lambda _pid: "123")
    monkeypatch.setattr(module.os, "kill", lambda *_args: None)
    path = tmp_path / "isaac.json"
    state = module.IsaacStartupState(path, "/map.yaml", "/localizer.yaml")
    state.started(100)
    for line in (b"[occupancy_grid_localizer] [NitrosNode] Initializing and running GXF graph\n",
                 b"[other_node] [NitrosNode] Node was started\n"):
        state.output(100, "stderr", line)
        assert not module.startup_ready(path, "/map.yaml")
    marker = b"[INFO] [1788970000] [occupancy_grid_localizer]: [NitrosNode] Node was started\n"
    state.output(99, "stderr", marker)
    assert not module.startup_ready(path, "/map.yaml")
    state.output(100, "stderr", marker[:42])
    assert not module.startup_ready(path, "/map.yaml")
    state.output(100, "stderr", marker[42:])
    assert module.startup_ready(path, "/map.yaml")
    assert not module.startup_ready(path, "/other-map.yaml")
    state.started(101)
    assert not module.startup_ready(path, "/map.yaml")
    state.output(100, "stderr", marker)
    assert not module.startup_ready(path, "/map.yaml")
    state.output(101, "stderr", marker)
    state.exited(100)
    assert module.startup_ready(path, "/map.yaml")
    state.exited(101)
    assert not module.startup_ready(path, "/map.yaml")


@pytest.mark.parametrize("case", ["missing", "malformed", "recycled", "dead"])
def test_stale_or_missing_evidence_is_not_ready(tmp_path, monkeypatch, case):
    module = load_state_module()
    path = tmp_path / "state.json"
    payload = dict(state="ready", pid=123, process_start_ticks="old", map_yaml="/map.yaml")
    if case != "missing":
        path.write_text("broken" if case == "malformed" else json.dumps(payload))
    monkeypatch.setattr(module, "process_start_ticks", lambda _pid: "new" if case == "recycled" else "old")
    def dead(*_args):
        raise ProcessLookupError()
    monkeypatch.setattr(module.os, "kill", dead)
    assert not module.startup_ready(path, "/map.yaml")


@pytest.mark.parametrize("delayed", ["map", "scan", "isaac"])
def test_actual_startup_waits_without_trigger_then_dispatches_once(tmp_path, delayed):
    from test_navigation_localization_startup import startup_harness
    startup_harness(tmp_path)
    path = tmp_path / "startup.sh"
    text = path.read_text()
    # Replace only the readiness adapter; run the real orchestration and cleanup.
    text = text.replace('ensure_localization_stack_ready_for_navigation() { sleep 0.15; }', '''
ensure_localization_stack_ready_for_navigation() {
  grep -q "waiting:" events 2>/dev/null && return 0
  echo "waiting:DELAYED" >> events
  ! grep -qx trigger events
  return 1
}'''.replace("DELAYED", delayed))
    path.write_text(text, encoding="utf-8", newline="\n")
    run = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=8)
    assert run.returncode == 0, run.stdout + run.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.index("waiting:" + delayed) < events.index("trigger")
    assert events.count("trigger") == events.count("nav") == events.count("loc") == 1
    assert events.index("trigger") < events.index("clean-nav")


def test_test_harness_does_not_touch_inherited_production_receipt(tmp_path, monkeypatch):
    from test_navigation_localization_startup import startup_harness
    sentinel = tmp_path / "not-test-owned.env"
    sentinel.write_text("production sentinel")
    monkeypatch.setenv("NJRH_NAV2_HOLD_READY_FILE", str(sentinel))
    startup_harness(tmp_path)
    run = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=8)
    assert run.returncode == 0, run.stdout + run.stderr
    assert sentinel.read_text() == "production sentinel"


def test_humble_launch_event_wiring_without_starting_any_node(tmp_path, monkeypatch):
    launch = pytest.importorskip("launch")
    from launch.actions import OpaqueFunction
    from launch.events.process import ProcessStarted, ProcessIO, ProcessExited
    from launch_ros.actions import ComposableNodeContainer

    path = tmp_path / "current.json"
    monkeypatch.setenv("NJRH_ISAAC_STARTUP_STATE_FILE", str(path))
    spec = importlib.util.spec_from_file_location(
        "test_occupancy_launch", SCRIPTS.parent / "launch/occupancy_localization.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = launch.LaunchContext()
    context.launch_configurations.update(map_yaml="/target.yaml", localizer_map_yaml="/localizer.yaml")
    opaque = next(x for x in description.entities if isinstance(x, OpaqueFunction))
    handlers = opaque.execute(context)
    container = next(x for x in description.entities if isinstance(x, ComposableNodeContainer))
    fields = dict(action=container, name="fake_isaac", cmd=[], cwd=None, env={}, pid=os.getpid())

    def deliver(event):
        for registration in handlers:
            handler = registration.event_handler
            if handler.matches(event):
                context.extend_locals({"event": event})
                for action in handler.handle(event, context) or []:
                    action.execute(context)

    deliver(ProcessStarted(**fields))
    assert json.loads(path.read_text())["state"] == "initializing"
    deliver(ProcessIO(fd=2, text=b"[occupancy_grid_localizer] [NitrosNode] Node was started\n", **fields))
    assert load_state_module().startup_ready(path, "/target.yaml")
    deliver(ProcessExited(returncode=1, **fields))
    assert not load_state_module().startup_ready(path, "/target.yaml")

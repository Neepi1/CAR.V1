"""Stationary startup orchestration tests; no ROS graph or robot is used."""

import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import time
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts/jetson/runtime_overlay/scripts"


def shell_function(name):
    source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    match = re.search(rf"(?ms)^{name}\(\) \{{\n.*?^\}}\n", source)
    assert match is not None, name
    return match.group(0)


def bash_executable():
    return "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")


def startup_harness(tmp_path, trigger_result=0):
    source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    main = source[source.index('echo "[runtime-overlay] navigation start source='):
                  source.index('log_startup_stage "initial_global_localization_ready"')]
    functions = ["env_flag_true", "start_resident_navigation_layer",
                 "wait_for_initial_global_localization", "terminate_child",
                 "wait_for_child_exit", "cleanup", "on_exit", "on_signal",
                 "ensure_navigation_layer_alive", "ensure_localization_layer_alive"]
    if "wait_for_later_initial_localization()" in source:
        functions.append("wait_for_later_initial_localization")
    for filename, marker in [("run_nav2_navigation.sh", "nav"),
                             ("run_occupancy_grid_localization.sh", "loc")]:
        (tmp_path / filename).write_text(
            f'#!/usr/bin/env bash\necho {marker} >> events\nexec sleep 30\n', encoding="utf-8")
    harness = """#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$PWD"
export NJRH_NAV2_HOLD_READY_FILE="$PWD/held.env"
export NJRH_NAV2_LIFECYCLE_READY_STATUS_FILE="$PWD/lifecycle.env"
export NJRH_AMCL_RUNTIME_STATUS_FILE="$PWD/amcl.env"
export NJRH_RUNTIME_MAP_CONTEXT_FILE="$PWD/context.json"
export NJRH_ISAAC_STARTUP_STATE_FILE="$PWD/isaac.json"
export TMPDIR="$PWD"
navigation_start_source=systemd_autostart
NJRH_BUILDING_ID=B11
NJRH_FLOOR_ID=F3
NJRH_AMCL_LOCALIZATION_MODE=disabled
NJRH_NAV_RUNTIME_STOP_INT_ATTEMPTS=0
NJRH_NAV_RUNTIME_STOP_TERM_ATTEMPTS=2
localization_pid=''
navigation_pid=''
initial_global_localization_pid=''
initial_global_localization_baseline_sequence=0
nav2_lifecycle_bringup_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
amcl_status_heartbeat_pid=''
nav2_prestarted=0
nav2_lifecycle_background_started=0
amcl_runtime_started=0
runtime_ready=0
cleanup_started=0
common_local_state_ready=1
localization_ready_failure_reason='injected no result'
exit_code=0
"""
    harness += "\n".join(shell_function(name) for name in functions)
    harness += f'\nsource "{(SCRIPTS / "scan_ownership_helpers.sh").as_posix()}"\n'
    harness += f"""
log_startup_stage() {{ echo "stage:$1" >> events; }}
write_runtime_map_context() {{ echo "context:$1:$2" >> events; }}
clear_nav2_lifecycle_ready_status() {{ :; }}
stop_existing_standard_nav_stack() {{ echo clean-nav >> events; }}
stop_existing_localization_stack() {{ echo clean-loc >> events; }}
njrh_begin_startup_cpu_boost() {{ :; }}
ensure_helper_process_no_probe() {{ :; }}
ensure_localization_stack_ready_for_navigation() {{ sleep 0.15; }}
capture_initial_global_localization_baseline() {{ :; }}
restore_navigation_scan_owner() {{ :; }}
runtime_readiness_probe() {{ return 0; }}
trigger_global_localization_for_navigation() {{
  echo trigger >> events
  grep -qx nav events || return 42
  return {trigger_result}
}}
wait_for_bridge_explicit_relocalization_sequence() {{ echo 2; }}
wait_for_bridge_has_map_to_odom() {{ return 0; }}
python3() {{
  echo waiting-client >> events
  while [[ ! -f resume ]]; do
    if [[ -f cancel ]]; then kill -TERM "$$"; break; fi
    sleep 0.05
  done
}}
trap on_exit EXIT
trap on_signal INT TERM
"""
    harness += f'\nexport NJRH_FLOOR_STARTUP_HANDOFF_FILE="{tmp_path.as_posix()}/handoff.json"\n'
    harness += 'export NJRH_FLOOR_STARTUP_HANDOFF_ACK_FILE="$PWD/ack.json"\n'
    harness += 'export NJRH_FLOOR_STARTUP_HANDOFF_EVIDENCE_FILE="$PWD/evidence.json"\n'
    harness += 'export NJRH_STARTUP_TRIGGER_OUTCOME_FILE="$PWD/outcome.json"\n'
    harness += f'source "{(SCRIPTS / "floor_startup_handoff_helpers.sh").as_posix()}"\n'
    harness += f'source "{(SCRIPTS / "navigation_startup_receipt.sh").as_posix()}"\n'
    harness += 'export NJRH_NAVIGATION_STARTUP_RECEIPT="$PWD/startup.phase"\n'
    harness += main
    harness += '\necho continued >> events\n'
    path = tmp_path / "startup.sh"
    path.write_text(harness, encoding="utf-8")
    return path


def test_nav2_process_starts_before_initial_localization(tmp_path):
    startup_harness(tmp_path)
    run = subprocess.run([bash_executable(), "startup.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=8)
    assert run.returncode == 0, run.stdout + run.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.index("nav") < events.index("trigger") < events.index("continued")


@pytest.mark.parametrize("action", ["resume", "cancel"])
def test_initial_timeout_keeps_processes_and_later_result_continues(tmp_path, action):
    startup_harness(tmp_path, trigger_result=1)
    run = subprocess.Popen([bash_executable(), "startup.sh"], cwd=tmp_path,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 4
        events = []
        while time.monotonic() < deadline:
            if (tmp_path / "events").exists():
                events = (tmp_path / "events").read_text().splitlines()
            if "waiting-client" in events or run.poll() is not None:
                break
            time.sleep(0.05)
        assert run.poll() is None, run.communicate(timeout=2)
        assert "waiting-client" in events
        assert (tmp_path / "startup.phase").read_text().strip() == "waiting_for_localization"
        assert "clean-nav" not in events and "clean-loc" not in events
        assert events.count("trigger") == events.count("nav") == events.count("loc") == 1
        (tmp_path / action).touch()
        stdout, stderr = run.communicate(timeout=6)
        assert run.returncode == (0 if action == "resume" else 130), stdout + stderr
        events = (tmp_path / "events").read_text().splitlines()
        assert ("continued" in events) == (action == "resume")
        assert events.count("trigger") == 1
        assert events.count("clean-nav") == events.count("clean-loc") == 1
    finally:
        (tmp_path / "resume").touch()
        if run.poll() is None:
            run.communicate(timeout=6)


@pytest.mark.parametrize("matching_owner", [True, False])
def test_held_nav2_record_does_not_expire_while_owner_and_controller_are_alive(tmp_path, matching_owner):
    harness = '''#!/usr/bin/env bash
set -euo pipefail
navigation_pid=$$
NJRH_NAV2_PRESTART_HOLD_READY_TIMEOUT_SEC=1
NJRH_NAV2_HOLD_READY_FILE="$PWD/held.env"
ensure_navigation_layer_alive() { return 0; }
printf 'NAV2_HOLD_READY=true\nNAV2_HOLD_READY_STAMP_SEC=1\nNAV2_HOLD_READY_WRAPPER_PID=%s\nNAV2_HOLD_READY_CONTROLLER_PID=%s\n' "$$" "$$" > "$NJRH_NAV2_HOLD_READY_FILE"
'''
    harness += shell_function("wait_for_prestarted_nav2_launch_hold_ready")
    if not matching_owner:
        harness += '\nnavigation_pid=99999999\n'
    harness += '\nwait_for_prestarted_nav2_launch_hold_ready\n'
    (tmp_path / "held.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "held.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=4)
    assert run.returncode == (0 if matching_owner else 1), run.stdout + run.stderr


@pytest.mark.parametrize("baseline,preceding,override,expected", [
    (1, [], {}, 0),
    (1, ["malformed", [], {}, {"last_explicit_relocalization_sequence": 1}], {}, 0),
    (-1, [{"last_explicit_relocalization_sequence": 1}], {}, 0),
    (2, [], {}, 130),
    (-1, [], {}, 130),
    (1, [], {"map_to_odom_publisher_owner": "other"}, 130),
    # Relocalization completion no longer owns navigation admission.
    (1, [], {"safe_for_goal_start": False}, 0),
    (1, [], {"correction_active": True}, 130),
    (1, [], {"has_map_to_odom": False}, 130),
    (1, [], {"current_sequence": 1}, 130),
])
def test_waiting_client_accepts_later_bridge_result_without_triggering(
        monkeypatch, baseline, preceding, override, expected):
    subscriptions = []
    messages = preceding + [dict(
        last_explicit_relocalization_sequence=2,
        has_map_to_odom=True,
        map_to_odom_publisher_owner="robot_localization_bridge",
        safe_for_goal_start=True,
        correction_active=False,
        current_sequence=2,
        target_sequence=2,
    )]
    messages[-1].update(override)

    class Node:
        def create_subscription(self, _type, topic, callback, _qos):
            assert topic == "/localization/bridge_status"
            subscriptions.append(callback)
            return object()

        def create_client(self, *_args):
            raise AssertionError("waiting must not create or call a trigger service")

    ros = types.ModuleType("rclpy")
    ros.init = lambda **_kwargs: None

    def create_node(name, **options):
        assert name == "startup_global_localization_trigger_client"
        assert options == {"enable_rosout": False, "start_parameter_services": False}
        return Node()

    ros.create_node = create_node
    ros.ok = lambda: bool(messages)

    def spin_once(_node, **_kwargs):
        message = types.SimpleNamespace(data=json.dumps(messages.pop(0)))
        for callback in subscriptions:
            callback(message)

    ros.spin_once = spin_once
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = object
    srv = types.ModuleType("robot_interfaces.srv")
    srv.TriggerLocalization = object
    for name, module in {
        "rclpy": ros, "rclpy.qos": qos,
        "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std_msg,
        "robot_interfaces": types.ModuleType("robot_interfaces"),
        "robot_interfaces.srv": srv,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        "startup_trigger_test", SCRIPTS / "call_global_localization_trigger.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "floor_handoff_requested", lambda: False)
    monkeypatch.setattr(sys, "argv", [
        str(SCRIPTS / "call_global_localization_trigger.py"),
        "--reason", "startup_wait", "--wait-for-bridge-after", str(baseline),
    ])
    assert module.main() == expected
    assert len(subscriptions) == 1

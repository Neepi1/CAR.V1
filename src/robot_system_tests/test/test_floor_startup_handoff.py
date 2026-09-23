"""Cold-start handoff contract tests; never connect to ROS or a robot."""

import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import types

import pytest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts/jetson/runtime_overlay/scripts"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "floor_handoff_test", SCRIPTS / "floor_startup_handoff.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fixture_request(tmp_path):
    root = tmp_path / "building-a/floor-2/maps/map-b"
    root.mkdir(parents=True)
    request = dict(schema="njrh.floor_startup_handoff.v1", version=1,
                   state="requested", transaction_id="switch-1", request_nonce="nonce-1",
                   building_id="building-a", floor_id="floor-2", map_id="map-b",
                   asset_epoch=2, asset_digest="sha256:" + "a" * 64,
                   explicit_sequence_baseline=3, speed_filter_enabled=False,
                   asset_root=str(root))
    roles = dict(nav_map_yaml="nav/map.yaml", localizer_map_png="localizer/map.png",
                 localizer_params_yaml="localizer/map.yaml",
                 keepout_mask_yaml="filters/keepout_mask.yaml",
                 speed_mask_yaml="filters/speed_mask.yaml")
    for role, relative in roles.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(role, encoding="utf-8")
        request[role] = str(path)
    for relative in ("poses.yaml", "filters/binary_mask.yaml"):
        (root / relative).write_text("{}", encoding="utf-8")
    # Match ExactMapFixture's real minimal manifest, not the handoff schema.
    manifest = {key: request[key] for key in (
        "building_id", "floor_id", "map_id", "asset_epoch", "asset_digest")}
    manifest.update(schema="njrh.map_manifest.v2", asset_digest_algorithm="sha256",
                    asset_digest_contract="njrh-map-asset-bundle-v1", safe_map_name="map")
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    path = tmp_path / "request.json"
    path.write_text(json.dumps(request), encoding="utf-8")
    return path, request


def test_pending_target_tf_can_wake_startup_without_navigation_permission(tmp_path):
    module = load_module()
    path, request = fixture_request(tmp_path)
    request = module.load_request(path)
    context = dict(request, state="floor_switch_pending", confirmed=False)
    health = dict(request, transition_active=True, runtime_context_valid=False,
                  localizer_ready=True, bridge_ready=True, tf_unique=True,
                  localizer_generation=2, explicit_relocalization_sequence=4,
                  safe_for_goal_start=False, amcl_ready=False)
    assert module.target_health_ready(request, health, context)
    for field, value in (("map_id", "old-map"), ("asset_epoch", 1),
                         ("explicit_relocalization_sequence", 3),
                         ("bridge_ready", False), ("tf_unique", False)):
        assert not module.target_health_ready(request, dict(health, **{field: value}), context)
    assert not module.target_health_ready(request, health, dict(context, transaction_id="old"))


@pytest.mark.parametrize("field,value", [
    ("asset_epoch", True), ("asset_digest", "not-a-digest"), ("asset_digest", "a" * 64),
    ("explicit_sequence_baseline", -1), ("speed_filter_enabled", "false"),
    ("request_nonce", ""), ("map_id", "old-map"),
])
def test_adoption_rejects_invalid_or_wrong_bundle_identity(tmp_path, field, value):
    module = load_module()
    path, request = fixture_request(tmp_path)
    request[field] = value
    path.write_text(json.dumps(request), encoding="utf-8")
    with pytest.raises(ValueError):
        module.load_request(path)


def test_runtime_ack_cannot_reuse_old_nonce_or_localization_evidence(tmp_path):
    module = load_module()
    _, request = fixture_request(tmp_path)
    evidence = dict(request, explicit_relocalization_sequence=4, localizer_generation=2)
    ack = module.acknowledgement(request, "runtime_ready", evidence=evidence)
    assert ack["request_nonce"] == request["request_nonce"]
    assert ack["updated_at"] > 0
    assert ack["explicit_relocalization_sequence"] == 4
    for change in (dict(request_nonce="old"), dict(explicit_relocalization_sequence=3),
                   dict(map_id="old-map"), dict(localizer_generation=0)):
        with pytest.raises(ValueError):
            module.acknowledgement(request, "runtime_ready", evidence=dict(evidence, **change))


def bash_executable():
    return "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")


def test_startup_joins_old_worker_and_never_writes_context_after_request(tmp_path):
    helper = (SCRIPTS / "floor_startup_handoff_helpers.sh").as_posix()
    harness = f'''#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=unused
export NJRH_FLOOR_STARTUP_HANDOFF_FILE="$PWD/request.json"
write_runtime_map_context() {{ echo "write:$1" >> events; }}
source "{helper}"
touch "$NJRH_FLOOR_STARTUP_HANDOFF_FILE"
nav2_lifecycle_bringup_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
(sleep 0.2; echo worker-finished >> events) &
nav2_lifecycle_bringup_pid=$!
floor_handoff_cli() {{
  case "$1" in
    export-env) echo 'export NJRH_RUNTIME_TRANSACTION_ID=txn NJRH_MAP_ID=target' ;;
    export-evidence) echo 'export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE=4' ;;
    ack) grep -qx worker-finished events; echo "ack:$3" >> events ;;
    *) echo "$1" >> events ;;
  esac
}}
write_runtime_map_context starting false old
adopt_startup_floor_handoff
write_runtime_map_context failed false old
write_runtime_map_context degraded true old
write_runtime_map_context ready true old
'''
    (tmp_path / "test.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.index("worker-finished") < events.index("ack:adopted")
    assert "wait-target" in events
    assert not any(event.startswith("write:") for event in events)


def test_runtime_ready_ack_waits_for_amcl_and_floor_manager_commit(tmp_path):
    helper = (SCRIPTS / "floor_startup_handoff_helpers.sh").as_posix()
    harness = f'''#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=unused
write_runtime_map_context() {{ echo unexpected-write >> events; }}
source "{helper}"
floor_startup_handoff_active=1
NJRH_AMCL_LOCALIZATION_MODE=gated
runtime_ready=0
wait_for_amcl_readiness_background_if_running() {{ echo joined-amcl >> events; }}
load_amcl_runtime_status() {{ AMCL_READY=false; }}
complete_amcl_readiness_with_retries_for_navigation() {{ echo amcl-ready >> events; }}
floor_handoff_cli() {{
  if [[ "$1" == wait-commit ]]; then
    [[ "$runtime_ready" == 0 ]]
    grep -qx 'ack runtime_ready' events
    echo manager-committed >> events
  elif [[ "$1" == ack ]]; then echo "ack $3" >> events
  else echo "$1" >> events; fi
}}
complete_startup_floor_handoff
[[ "$runtime_ready" == 1 ]]
'''
    (tmp_path / "test.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events.index("amcl-ready") < events.index("wait-target")
    assert events.index("wait-target") < events.index("ack runtime_ready")
    assert events.index("ack runtime_ready") < events.index("manager-committed")
    assert "unexpected-write" not in events


def test_typed_observer_rejects_stale_wrong_target_then_accepts_pending_tf(tmp_path, monkeypatch):
    module = load_module()
    path, request = fixture_request(tmp_path)
    context_path = tmp_path / "context.json"
    context_path.write_text(json.dumps(dict(request, state="floor_switch_pending", confirmed=False)))
    evidence_path = tmp_path / "evidence.json"
    fields = dict(request, transition_active=True, runtime_context_valid=False,
                  localizer_ready=True, bridge_ready=True, tf_unique=True,
                  localizer_generation=2, explicit_relocalization_sequence=4)
    messages = [dict(fields, map_id="old-map"), dict(fields), dict(fields)]
    stamps = [10, 8, 10]
    subscriptions = []

    class Node:
        def create_subscription(self, _type, topic, callback, _qos):
            assert topic == "/localization/floor_health"
            subscriptions.append(callback)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10200000000))

    ros = types.ModuleType("rclpy")
    ros.init = lambda **_kwargs: None
    ros.create_node = lambda _name: Node()
    ros.ok = lambda: bool(messages)

    def spin_once(_node, **_kwargs):
        values = messages.pop(0)
        values["stamp"] = types.SimpleNamespace(sec=stamps.pop(0), nanosec=0)
        for callback in subscriptions:
            callback(types.SimpleNamespace(**values))
        if messages:
            assert not evidence_path.exists()

    ros.spin_once = spin_once
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    msg = types.ModuleType("robot_interfaces.msg")
    msg.LocalizationHealth = object
    for name, value in {"rclpy": ros, "rclpy.qos": qos,
                        "robot_interfaces": types.ModuleType("robot_interfaces"),
                        "robot_interfaces.msg": msg}.items():
        monkeypatch.setitem(sys.modules, name, value)
    assert module.wait_target(path, context_path, evidence_path) == 0
    result = module.read_json(evidence_path)
    assert result["request_nonce"] == request["request_nonce"]
    assert result["explicit_relocalization_sequence"] == 4


def test_unknown_old_trigger_cannot_be_acknowledged_as_adopted(tmp_path):
    helper = (SCRIPTS / "floor_startup_handoff_helpers.sh").as_posix()
    harness = f'''#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=unused
write_runtime_map_context() {{ :; }}
source "{helper}"
nav2_lifecycle_bringup_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
floor_handoff_cli() {{
  case "$1" in
    export-env) echo 'export NJRH_RUNTIME_TRANSACTION_ID=txn NJRH_MAP_ID=target' ;;
    check-trigger-outcome) return 1 ;;
    ack) echo "$3" >> events ;;
    *) echo unexpected >> events ;;
  esac
}}
if adopt_startup_floor_handoff; then exit 42; fi
[[ "$floor_startup_handoff_active" == 1 ]]
'''
    (tmp_path / "test.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    assert (tmp_path / "events").read_text().splitlines() == ["failed"]


@pytest.mark.parametrize("state", ["committed", "failed"])
def test_new_startup_ignores_terminal_history_but_adopted_owner_remains_fenced(tmp_path, state):
    module = load_module()
    path, request = fixture_request(tmp_path)
    assert module.request_pending(path)
    request["state"] = state
    path.write_text(json.dumps(request), encoding="utf-8")
    assert not module.request_pending(path)
    helper = (SCRIPTS / "floor_startup_handoff_helpers.sh").as_posix()
    harness = f'''#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=unused
write_runtime_map_context() {{ echo "write:$1" >> events; }}
source "{helper}"
floor_handoff_requested() {{ return 1; }}
# New instance sees only a terminal historical request.
write_runtime_map_context starting false new-instance
# The same owner, after adoption, cannot recover its old write authority even
# when floor-manager changes the request to a terminal state.
floor_startup_handoff_active=1
write_runtime_map_context ready true old-owner
write_runtime_map_context failed false old-owner
'''
    (tmp_path / "test.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    assert (tmp_path / "events").read_text().splitlines() == ["write:starting"]


def test_real_cpp_writer_request_is_accepted_by_python_reader(tmp_path):
    """Cross-language contract: compile the actual production serializer.

    Run this on the candidate Linux ROS build host (yaml-cpp dev is required).
    The Windows script-only test host has no ROS/C++ toolchain and skips it.
    """
    compiler = shutil.which("g++") or shutil.which("clang++")
    if not compiler:
        pytest.skip("real C++ writer requires candidate-host C++/yaml-cpp toolchain")
    module = load_module()
    path, request = fixture_request(tmp_path)
    source = tmp_path / "serialize_handoff.cpp"
    source.write_text(r'''
#include "robot_floor_manager/runtime_map_context_writer.hpp"
#include <iostream>
int main(int argc, char ** argv) {
  if (argc != 3) return 2;
  const std::filesystem::path root(argv[1]);
  robot_floor_manager::RuntimeMapContextRecord r;
  r.startup_handoff = true; r.state = "requested"; r.confirmed = false;
  r.transaction_id = "switch-1"; r.request_nonce = "nonce-1";
  r.building_id = "building-a"; r.floor_id = "floor-2"; r.map_id = "map-b";
  r.asset_epoch = 2; r.asset_digest = "sha256:" + std::string(64, 'a');
  r.explicit_sequence_baseline = 3; r.speed_filter_enabled = false;
  r.updated_at_sec = 42.0; r.asset_root = root.string();
  r.nav_map_yaml = (root / "nav/map.yaml").string();
  r.localizer_map_png = (root / "localizer/map.png").string();
  r.localizer_params_yaml = (root / "localizer/map.yaml").string();
  r.keepout_mask_yaml = (root / "filters/keepout_mask.yaml").string();
  r.speed_mask_yaml = (root / "filters/speed_mask.yaml").string();
  std::string error;
  if (!robot_floor_manager::AtomicRuntimeMapContextWriter{}.write(argv[2], r, error)) {
    std::cerr << error; return 1;
  }
  return 0;
}
''', encoding="utf-8")
    root = SCRIPTS.parents[3]
    package = root / "src/robot_floor_manager"
    executable = tmp_path / ("serialize_handoff.exe" if os.name == "nt" else "serialize_handoff")
    build = subprocess.run([compiler, "-std=c++17", "-I", str(package / "include"),
                            str(source), str(package / "src/runtime_map_context_writer.cpp"),
                            "-lyaml-cpp", "-o", str(executable)],
                           capture_output=True, text=True, timeout=60)
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(executable), request["asset_root"], str(path)],
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    actual = module.load_request(path)
    assert module.same_request(request, actual)


def test_committed_startup_loses_side_effect_permission_after_later_hot_switch(tmp_path):
    module = load_module()
    path, request = fixture_request(tmp_path)
    request["state"] = "committed"
    path.write_text(json.dumps(request), encoding="utf-8")
    context_path = tmp_path / "context.json"
    context = dict(request, state="ready", confirmed=True)
    context_path.write_text(json.dumps(context), encoding="utf-8")
    module.require_current_side_effect(path, context_path)
    for replacement in (dict(context, transaction_id="hot-2", map_id="map-c"),
                        dict(context, state="floor_switch_pending", confirmed=False),
                        dict(context, asset_epoch=3)):
        context_path.write_text(json.dumps(replacement), encoding="utf-8")
        with pytest.raises(ValueError, match="no longer owns"):
            module.require_current_side_effect(path, context_path)


def test_failed_request_stops_typed_wait_before_connecting_to_ros(tmp_path):
    module = load_module()
    path, request = fixture_request(tmp_path)
    request["state"] = "failed"
    path.write_text(json.dumps(request), encoding="utf-8")
    with pytest.raises(ValueError, match="marked this handoff failed"):
        module.wait_target(path, tmp_path / "context.json", tmp_path / "evidence.json")


@pytest.mark.parametrize("handler,exit_code", [("on_exit", 1), ("on_signal", 130)])
@pytest.mark.parametrize("receipt_writable", [True, False])
def test_failed_startup_exit_ack_follows_real_cleanup_without_waiting_for_target(
        tmp_path, handler, exit_code, receipt_writable):
    """Run the real exit/cleanup functions with process-control edges replaced."""
    import re
    source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    functions = []
    for name in ("cleanup", handler):
        match = re.search(rf"(?ms)^{name}\(\) \{{\n.*?^\}}\n", source)
        assert match
        functions.append(match.group(0))
    helper = (SCRIPTS / "floor_startup_handoff_helpers.sh").as_posix()
    harness = f'''#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=unused
write_runtime_map_context() {{ echo forbidden-context-write >> events; }}
source "{helper}"
floor_startup_handoff_active=1
floor_startup_old_effects_settled=1
floor_startup_target_work_started=0
cleanup_started=0
runtime_ready=0
amcl_runtime_started=0
navigation_start_source=api_resume
NJRH_AMCL_LOCALIZATION_MODE=disabled
initial_global_localization_pid=''
nav2_lifecycle_bringup_pid=''
navigation_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
localization_pid=''
terminate_child() {{ echo "terminate:$2" >> events; }}
stop_existing_standard_nav_stack() {{ echo nav-cleanup-finished >> events; }}
stop_existing_localization_stack() {{ echo localization-cleanup-finished >> events; }}
floor_handoff_cli() {{
  [[ "$1" == ack-exit ]]
  grep -qx localization-cleanup-finished events
  if [[ {int(receipt_writable)} -eq 0 ]]; then echo ack-write-failed >> events; return 1; fi
  echo "$*" >> events
}}
'''
    harness += "\n".join(functions)
    harness += '\n(false) || ' + handler + '\n'
    (tmp_path / "test.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == exit_code, run.stdout + run.stderr
    events = (tmp_path / "events").read_text().splitlines()
    assert events[-1] == ("ack-exit --effects-settled true" if receipt_writable else "ack-write-failed"), events
    assert events.index("localization-cleanup-finished") < len(events) - 1
    assert "forbidden-context-write" not in events
    if not receipt_writable:
        assert "startup exit receipt unavailable; do not infer effect settlement" in run.stderr


@pytest.mark.parametrize("settled", [True, False])
def test_exit_cli_acknowledges_failed_exact_request_not_ready(tmp_path, settled):
    path, request = fixture_request(tmp_path)
    request["state"] = "failed"
    path.write_text(json.dumps(request), encoding="utf-8")
    ack = tmp_path / "ack.json"
    env = dict(os.environ, NJRH_FLOOR_STARTUP_HANDOFF_NONCE=request["request_nonce"],
               NJRH_STARTUP_INSTANCE="isolated-startup", NJRH_STARTUP_OWNER_PID="123")
    for field, variable in (("transaction_id", "NJRH_RUNTIME_TRANSACTION_ID"),
                            ("building_id", "NJRH_BUILDING_ID"), ("floor_id", "NJRH_FLOOR_ID"),
                            ("map_id", "NJRH_MAP_ID"), ("asset_epoch", "NJRH_MAP_ASSET_EPOCH"),
                            ("asset_digest", "NJRH_MAP_ASSET_DIGEST")):
        env[variable] = str(request[field])
    run = subprocess.run([sys.executable, str(SCRIPTS / "floor_startup_handoff.py"),
                          "ack-exit", "--request", str(path), "--ack", str(ack),
                          "--effects-settled", str(settled).lower()],
                         env=env, capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    result = json.loads(ack.read_text(encoding="utf-8"))
    assert result["state"] == "failed"
    assert result["cleanup_completed"] is True
    assert result["effects_settled"] is settled
    assert result["owner_available"] is False
    assert result["request_nonce"] == request["request_nonce"]
    assert result["failure"] == ("STARTUP_OWNER_EXITED" if settled else "STARTUP_EXIT_EFFECTS_UNPROVEN")
    assert "localizer_generation" not in result


@pytest.mark.parametrize("scenario,expected", [
    ("waiting_target_failed", "true"), ("unknown_trigger", "false"),
    ("old_worker_failed", "false"), ("target_work_started", "false"),
])
def test_real_adoption_exit_does_not_infer_rpc_settlement_from_shutdown(tmp_path, scenario, expected):
    helper = (SCRIPTS / "floor_startup_handoff_helpers.sh").as_posix()
    harness = f'''#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=unused
write_runtime_map_context() {{ :; }}
source "{helper}"
nav2_lifecycle_bringup_pid=''
amcl_resident_pid=''
amcl_readiness_pid=''
scenario={scenario}
if [[ "$scenario" == old_worker_failed ]]; then (exit 7) & nav2_lifecycle_bringup_pid=$!; fi
floor_handoff_cli() {{
  case "$1" in
    export-env) echo 'export NJRH_RUNTIME_TRANSACTION_ID=txn NJRH_MAP_ID=target' ;;
    check-trigger-outcome) [[ "$scenario" != unknown_trigger ]] ;;
    wait-target) [[ "$scenario" == target_work_started ]] ;;
    export-evidence) echo 'export NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE=4' ;;
    ack-exit) echo "$*" >> events ;;
    ack) echo "ack:$3" >> events ;;
    *) exit 91 ;;
  esac
}}
adopt_startup_floor_handoff || true
# Model cleanup returning; none of its child kills are settlement evidence.
acknowledge_startup_floor_handoff_exit
acknowledge_startup_floor_handoff_exit
'''
    (tmp_path / "test.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    exits = [event for event in (tmp_path / "events").read_text().splitlines()
             if event.startswith("ack-exit")]
    assert exits == ["ack-exit --effects-settled " + expected]


@pytest.mark.parametrize("change", ["nonce", "identity", "committed", "no_owner"])
def test_exit_ack_cannot_overwrite_another_or_committed_request(tmp_path, monkeypatch, change):
    module = load_module()
    path, request = fixture_request(tmp_path)
    for variable, value in {
        "NJRH_FLOOR_STARTUP_HANDOFF_NONCE": request["request_nonce"],
        "NJRH_STARTUP_OWNER_PID": "123", "NJRH_STARTUP_INSTANCE": "startup-test",
        "NJRH_RUNTIME_TRANSACTION_ID": request["transaction_id"],
        "NJRH_BUILDING_ID": request["building_id"], "NJRH_FLOOR_ID": request["floor_id"],
        "NJRH_MAP_ID": request["map_id"], "NJRH_MAP_ASSET_EPOCH": str(request["asset_epoch"]),
        "NJRH_MAP_ASSET_DIGEST": request["asset_digest"],
    }.items():
        monkeypatch.setenv(variable, value)
    request["state"] = "failed"
    if change == "nonce":
        request["request_nonce"] = "replacement"
    elif change == "identity":
        request["transaction_id"] = "replacement"
    elif change == "committed":
        request["state"] = "committed"
    elif change == "no_owner":
        monkeypatch.delenv("NJRH_FLOOR_STARTUP_HANDOFF_NONCE")
    path.write_text(json.dumps(request), encoding="utf-8")
    with pytest.raises(ValueError):
        module.exit_acknowledgement(module.current_request(path, allow_failed=True), True)


def test_retired_owner_keeps_supervision_without_starting_amcl_work(tmp_path):
    source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    import re
    match = re.search(r"(?ms)^maintain_amcl_readiness_background_for_navigation\(\) \{\n.*?^\}\n", source)
    assert match
    harness = '''#!/usr/bin/env bash
set -euo pipefail
floor_handoff_guard() { return 1; }
start_amcl_readiness_background_if_enabled_for_navigation() { echo forbidden >> events; }
'''
    harness += match.group(0)
    harness += '\nmaintain_amcl_readiness_background_for_navigation\necho supervision-continues >> events\n'
    (tmp_path / "test.sh").write_text(harness, encoding="utf-8")
    run = subprocess.run([bash_executable(), "test.sh"], cwd=tmp_path,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout + run.stderr
    assert (tmp_path / "events").read_text().splitlines() == ["supervision-continues"]


def test_default_amcl_background_starts_after_live_ready_context_commit():
    source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
    tail = source[source.index('if ! wait_for_nav2_layer_ready; then'):]
    commit = tail.index('if ! commit_runtime_ready_context "${ready_context_message}" observe_wrapper_service; then')
    amcl = tail.index('maintain_amcl_readiness_background_for_navigation')
    assert commit < amcl
    assert tail.index('report_navigation_startup_finished ready') < amcl
    assert 'complete_amcl_readiness_with_retries_for_navigation' in tail[:commit]
    assert 'wait_for_fresh_tf_transform "map" "odom"' in source
    assert '${NJRH_RUNTIME_READY_MAP_ODOM_MAX_AGE_SEC:-0.5}' in source

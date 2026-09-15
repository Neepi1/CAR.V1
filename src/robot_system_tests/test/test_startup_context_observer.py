"""Final startup context uses real shell/helper with only transport boundaries faked."""

import importlib.util
import json
import sys
import types

import pytest

from test_amcl_startup_sequence import SCRIPTS, run_shell


def run_final_context_case(tmp_path, mode="ready"):
    body = r'''
floor_startup_handoff_active=0
[[ "$MODE" != handoff_active ]] || floor_startup_handoff_active=1
complete_startup_floor_handoff() { echo handoff-complete >> events; return 0; }
floor_handoff_guard() { [[ "$MODE" != handoff || ! -f observed ]]; }
NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE=7
NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK=true
[[ "$MODE" != not_accepted ]] || NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK=false
runtime_ready=0
helper_process_pattern() { printf wrapper; }
helper_process_running() { echo owner-lookup >> events; return 0; }
wait_for_ros_service() { echo OLD-SERVICE >> events; return 0; }
wait_for_bridge_relocalization_sequence_after() { echo OLD-BRIDGE >> events; printf '7\n'; }
timeout() {
  echo combined-client >> events
  [[ "$*" == *'--service-wait-sec 10 --bridge-wait-sec 15 --minimum-sequence 6'* ]] || return 91
  touch observed
  [[ "$MODE" != service_missing ]] || { printf 'service_ready: false\n'; return 2; }
  case "$MODE" in
    bridge_missing|not_accepted|owner_lost|tf_missing)
      printf 'service_ready: true\n'; return 3 ;;
    newer_sequence) printf 'service_ready: true\nexplicit_sequence: 8\n'; return 0 ;;
    handoff_rc) printf 'service_ready: true\n'; return 20 ;;
  esac
  printf 'service_ready: true\nexplicit_sequence: 7\n'
}
set_localization_ready_failure() { echo "failure:$1" >> events; }
ensure_localization_layer_alive() { echo owner-live >> events; [[ "$MODE" != owner_lost ]]; }
wait_for_fresh_tf_transform() { echo fresh-tf >> events; [[ "$MODE" != tf_missing ]]; }
write_runtime_map_context() { echo persist >> events; [[ "$MODE" != write_failure ]]; }
runtime_map_context_matches_current_floor() { echo verify >> events; [[ "$MODE" != verify_failure ]]; }
rc=0
ensure_global_localization_wrapper_resident defer_service_observation || rc=$?
commit_runtime_ready_context fixture observe_wrapper_service || rc=$?
echo "rc:$rc:ready:$runtime_ready" >> events
'''
    return run_shell(tmp_path, "MODE=" + mode + "\n" + body,
        filename="run_navigation_runtime_services.sh", functions=(
        "ensure_global_localization_wrapper_resident", "commit_runtime_ready_context"))


def test_final_context_uses_one_client_and_keeps_durable_commit(tmp_path):
    result, events = run_final_context_case(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert events == ["owner-lookup", "combined-client", "persist", "verify", "rc:0:ready:1"]


def test_adopted_floor_handoff_keeps_original_service_check(tmp_path):
    result, events = run_final_context_case(tmp_path, "handoff_active")
    assert result.returncode == 0, result.stdout + result.stderr
    assert events == ["owner-lookup", "OLD-SERVICE", "handoff-complete", "rc:0:ready:0"]


@pytest.mark.parametrize("mode,expected", [
    ("bridge_missing", ["owner-live", "fresh-tf", "persist", "verify", "rc:0:ready:1"]),
    ("service_missing", ["failure:GLOBAL_LOCALIZATION_RESIDENT_SERVICE_MISSING", "rc:1:ready:0"]),
    ("newer_sequence", ["rc:1:ready:0"]),
    ("not_accepted", ["rc:1:ready:0"]),
    ("owner_lost", ["owner-live", "rc:1:ready:0"]),
    ("tf_missing", ["owner-live", "fresh-tf", "rc:1:ready:0"]),
    ("write_failure", ["persist", "rc:1:ready:0"]),
    ("verify_failure", ["persist", "verify", "rc:1:ready:0"]),
    ("handoff", ["rc:1:ready:0"]), ("handoff_rc", ["rc:1:ready:0"]),
])
def test_final_context_preserves_failure_fallback_and_write_boundaries(tmp_path, mode, expected):
    result, events = run_final_context_case(tmp_path, mode)
    assert result.returncode == 0, result.stdout + result.stderr
    assert events == ["owner-lookup", "combined-client", *expected]


def context_observer_fixture(monkeypatch, mode="ready"):
    events = []
    clock = [0.0]
    callbacks = []

    class Node:
        def get_service_names_and_types(self):
            if mode == "service_error":
                raise RuntimeError("graph lookup failed")
            if clock[0] >= 0.1 and mode != "no_service":
                if "service-discovered" not in events:
                    events.append("service-discovered")
                return [("/global_localization/trigger", ["robot_interfaces/srv/TriggerLocalization"])]
            return []

        def create_subscription(self, message_type, topic, callback, qos):
            assert topic == "/localization/bridge_status"
            assert qos.depth == 5 and qos.history == "keep_last"
            assert qos.reliability == "reliable" and qos.durability == "volatile"
            assert "service-discovered" in events, "do not retain a pre-service reader queue"
            if mode == "subscription_error":
                raise RuntimeError("subscription creation failed")
            callbacks.append(callback)
            events.append("subscribe")
            return object()

        def destroy_node(self):
            events.append("destroy")
            if mode == "cleanup_error":
                raise RuntimeError("cleanup failed")

    node = Node()
    ros = types.ModuleType("rclpy")
    ros.init = lambda **kwargs: events.append("init")
    ros.ok = lambda: True
    ros.shutdown = lambda: events.append("shutdown")

    def create_node(name, **kwargs):
        assert kwargs == {"enable_rosout": False, "start_parameter_services": False}
        events.append("node")
        return node

    def spin_once(owner, *, timeout_sec):
        assert owner is node and 0 < timeout_sec <= 0.1
        clock[0] += timeout_sec
        if callbacks and mode not in ("no_bridge", "handoff"):
            if mode == "malformed":
                payload = "not JSON"
            else:
                payload = json.dumps({
                    "last_explicit_relocalization_sequence": 6 if mode == "old_sequence" else (
                        True if mode == "boolean_sequence" else "7" if mode == "string_sequence"
                        else 8 if mode == "newer_sequence" else 7),
                    "has_map_to_odom": "true" if mode == "string_sequence" else mode != "no_map_odom",
                    "map_to_odom_publisher_owner": "wrong_owner" if mode == "wrong_owner" else "robot_localization_bridge",
                })
            callbacks[0](types.SimpleNamespace(data=payload))

    ros.create_node = create_node
    ros.spin_once = spin_once
    qos = types.ModuleType("rclpy.qos")
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
    qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    qos.qos_profile_sensor_data = types.SimpleNamespace(depth=5, history="keep_last")
    messages = types.ModuleType("std_msgs.msg")
    messages.String = type("String", (), {})
    for name, module in {"rclpy": ros, "rclpy.qos": qos, "std_msgs.msg": messages}.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("context_observer_fixture", SCRIPTS / "observe_startup_context.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "floor_handoff_requested", lambda: mode == "handoff" and clock[0] >= 0.15)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(sys, "argv", [str(SCRIPTS / "observe_startup_context.py"),
        "--service-wait-sec", "0.3", "--bridge-wait-sec", "0.4", "--minimum-sequence", "6"])
    return module, events, clock


def test_observer_uses_live_post_service_bridge_on_one_node(monkeypatch, capsys):
    module, events, clock = context_observer_fixture(monkeypatch)
    assert module.main() == 0
    assert capsys.readouterr().out.splitlines() == ["service_ready: true", "explicit_sequence: 7"]
    assert events.count("node") == 1
    assert events.index("service-discovered") < events.index("subscribe")
    assert events[-2:] == ["destroy", "shutdown"]
    assert clock[0] < 0.3


@pytest.mark.parametrize("mode,expected,proof", [
    ("no_service", 2, ["service_ready: false"]),
    ("service_error", 1, []),
    ("no_bridge", 3, ["service_ready: true"]),
    ("old_sequence", 3, ["service_ready: true"]),
    ("boolean_sequence", 3, ["service_ready: true"]),
    ("wrong_owner", 3, ["service_ready: true"]),
    ("no_map_odom", 3, ["service_ready: true"]),
    ("malformed", 3, ["service_ready: true"]),
    ("subscription_error", 1, ["service_ready: true"]),
    ("handoff", 20, ["service_ready: true"]),
    ("newer_sequence", 0, ["service_ready: true", "explicit_sequence: 8"]),
    ("string_sequence", 0, ["service_ready: true", "explicit_sequence: 7"]),
    ("cleanup_error", 0, ["service_ready: true", "explicit_sequence: 7"]),
])
def test_observer_keeps_live_evidence_and_cleanup_boundaries(monkeypatch, capsys, mode, expected, proof):
    module, events, clock = context_observer_fixture(monkeypatch, mode)
    assert module.main() == expected
    assert capsys.readouterr().out.splitlines() == proof
    assert events.count("node") == 1 and events[-2:] == ["destroy", "shutdown"]
    if mode in ("no_service", "service_error"):
        assert "subscribe" not in events
    if expected == 3:
        assert 0.5 <= clock[0] <= 0.55
    if mode == "no_service":
        assert 0.3 <= clock[0] <= 0.35
    if mode == "handoff":
        assert clock[0] <= 0.2


def run_warm_context_case(tmp_path, mode="ready", through_main=False):
    # The fixture executable replaces only the ROS transport process. Startup,
    # child ownership, final commit and compatibility fallback are real shell.
    (tmp_path / "observer").write_text(r'''#!/usr/bin/env bash
set -eu
echo warm-start >> events
while [[ $# -gt 0 ]]; do
  case "$1" in
    --commit-file) commit="$2"; shift 2 ;;
    *) shift ;;
  esac
done
[[ "$MODE" != early_exit ]] || exit 4
while [[ ! -f "$commit" ]]; do sleep 0.01; done
echo commit-request >> events
[[ "$MODE" != handoff ]] || exit 20
[[ "$MODE" != warm_error ]] || exit 4
echo 'service_ready: true'
[[ "$MODE" != missing_bridge ]] || exit 3
if [[ "$MODE" == newer_sequence ]]; then
  echo 'explicit_sequence: 8'
else
  echo 'explicit_sequence: 7'
fi
''', encoding="utf-8", newline="\n")
    body = r'''
export MODE
chmod +x observer
export TMPDIR="$PWD"
export NJRH_STARTUP_CONTEXT_OBSERVER_BIN="$PWD/observer"
navigation_start_source=systemd_autostart
nav2_lifecycle_background_started=1
floor_startup_handoff_active=0
NJRH_RUNTIME_EXPLICIT_RELOCALIZATION_SEQUENCE=7
NJRH_RUNTIME_LAST_TRIGGERED_RELOCALIZATION_OK=true
runtime_ready=0
floor_handoff_requested() { return 1; }
floor_handoff_guard() { return 0; }
complete_startup_floor_handoff() { echo adopted-floor >> events; }
set_localization_ready_failure() { echo "failure:$1" >> events; }
ensure_localization_layer_alive() { echo owner-live >> events; }
wait_for_fresh_tf_transform() { echo fresh-tf >> events; }
write_runtime_map_context() { echo persist >> events; }
runtime_map_context_matches_current_floor() { echo verify >> events; }
NJRH_NAV_RUNTIME_STOP_INT_ATTEMPTS=0
NJRH_NAV_RUNTIME_STOP_TERM_ATTEMPTS=1
timeout() {
  if [[ "$*" == *observe_startup_context.py* ]]; then
    echo cold-client >> events
    printf 'service_ready: true\nexplicit_sequence: 7\n'
  else
    shift 3
    exec "$@"
  fi
}
start_startup_context_observer_if_enabled
first_pid="${startup_context_observer_pid:-}"
start_startup_context_observer_if_enabled
[[ -n "$first_pid" && "$first_pid" == "$startup_context_observer_pid" ]] || exit 91
for unused in {1..100}; do grep -qx warm-start events 2>/dev/null && break; sleep 0.01; done
if [[ "$MODE" == early_exit ]]; then sleep 0.05; fi
echo nav2-still-starting >> events
[[ "$runtime_ready" == 0 ]] || exit 92
[[ ! -e "$startup_context_observer_dir/commit" ]] || exit 93
if [[ "$MODE" == adopted ]]; then floor_startup_handoff_active=1; fi
if [[ "$MODE" == term ]]; then
  localization_pid=''; navigation_pid=''; initial_global_localization_pid=''
  nav2_lifecycle_bringup_pid=''; amcl_resident_pid=''; amcl_readiness_pid=''
  amcl_runtime_started=0; NJRH_AMCL_LOCALIZATION_MODE=disabled; cleanup_started=0
  stop_existing_standard_nav_stack() { :; }
  stop_existing_localization_stack() { :; }
  trap on_signal TERM
  kill -TERM "$$"
  exit 94
fi
rc=0
commit_runtime_ready_context fixture observe_wrapper_service || rc=$?
echo "rc:$rc:ready:$runtime_ready:pid:${startup_context_observer_pid:-}" >> events
stop_startup_context_observer
'''
    if through_main:
        source = (SCRIPTS / "run_navigation_runtime_services.sh").read_text(encoding="utf-8")
        main = source[source.index('\nstart_startup_context_observer_if_enabled\nif ! activate_'):
                      source.index('\nif ! wait_for_nav2_layer_ready; then')]
        body = body.replace("start_startup_context_observer_if_enabled\nfirst_pid=", r'''
activate_prestarted_nav2_lifecycle() {
  for unused in {1..100}; do
    if grep -qx warm-start events 2>/dev/null; then echo nav2-joined >> events; return 0; fi
    sleep 0.01
  done
  return 1
}
''' + main + "\nfirst_pid=", 1)
    return run_shell(tmp_path, "MODE=" + mode + "\n" + body,
        filename="run_navigation_runtime_services.sh", functions=(
        "start_startup_context_observer_if_enabled", "collect_startup_context_observer",
        "stop_startup_context_observer", "wait_for_child_exit", "terminate_child",
        "cleanup", "on_signal", "commit_runtime_ready_context"))


def test_prewarmed_context_waits_for_final_commit_and_reuses_one_owned_worker(tmp_path):
    result, events = run_warm_context_case(tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert events == ["warm-start", "nav2-still-starting", "commit-request",
                      "persist", "verify", "rc:0:ready:1:pid:"]
    assert not list(tmp_path.glob("njrh-context-observer.*"))


@pytest.mark.parametrize("mode,tail", [
    ("newer_sequence", ["rc:1:ready:0:pid:"]),
    ("handoff", ["rc:1:ready:0:pid:"]),
    ("warm_error", ["cold-client", "persist", "verify", "rc:0:ready:1:pid:"]),
    ("early_exit", ["cold-client", "persist", "verify", "rc:0:ready:1:pid:"]),
    ("missing_bridge", ["owner-live", "fresh-tf", "persist", "verify", "rc:0:ready:1:pid:"]),
])
def test_prewarmed_context_keeps_original_failure_and_handoff_semantics(tmp_path, mode, tail):
    result, events = run_warm_context_case(tmp_path, mode)
    assert result.returncode == 0, result.stdout + result.stderr
    expected = ["warm-start", "nav2-still-starting"]
    if mode != "early_exit":
        expected.append("commit-request")
    assert events == expected + tail
    assert not list(tmp_path.glob("njrh-context-observer.*"))


def test_startup_main_warms_observer_before_joining_nav2_and_keeps_commit_late(tmp_path):
    result, events = run_warm_context_case(tmp_path, through_main=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert events == ["warm-start", "nav2-joined", "nav2-still-starting", "commit-request",
                      "persist", "verify", "rc:0:ready:1:pid:"]


@pytest.mark.parametrize("mode", ["term", "adopted"])
def test_warm_context_worker_is_reaped_on_parent_term_or_adopted_floor(tmp_path, mode):
    result, events = run_warm_context_case(tmp_path, mode)
    assert result.returncode == (130 if mode == "term" else 0), result.stdout + result.stderr
    assert "commit-request" not in events and "persist" not in events and "cold-client" not in events
    assert ("adopted-floor" in events) == (mode == "adopted")
    assert not list(tmp_path.glob("njrh-context-observer.*"))

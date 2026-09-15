"""Isolated shell/metadata contract tests. Never source production startup code."""
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
PIPELINE = ROOT / "scripts/jetson/runtime_overlay/scripts/run_pointcloud_accel_pipeline.sh"


def bash_binary():
    if os.name == "nt":
        candidate = Path("C:/Program Files/Git/bin/bash.exe")
        if candidate.exists():
            return str(candidate)
    return shutil.which("bash") or pytest.skip("bash unavailable")


def function(name):
    text = PIPELINE.read_text(encoding="utf-8")
    match = re.search(rf"^{name}\(\) \{{\n.*?^\}}", text, re.M | re.S)
    assert match, name
    return match.group(0)


def run_shell(body):
    functions = "\n".join(function(name) for name in (
        "observe_flatscan_helper", "note_flatscan_observer_unavailable",
        "note_flatscan_graph_miss", "supervise_flatscan_helper", "truthy"))
    harness = r'''
set -euo pipefail
FLATSCAN_MONITOR_CHECK_BIN=/bin/true
FLATSCAN_MONITOR_FILE=unused
FLATSCAN_MONITOR_MAX_AGE_SEC=2
FLATSCAN_MESSAGE_CONFIRM_TIMEOUT_SEC=10
FLATSCAN_HELPER_MISSING_CONFIRMATIONS=3
FLATSCAN_HELPER_REQUIRED=true
flatscan_pid=fixture-not-a-pid
flatscan_helper_graph_miss_count=0
flatscan_helper_healthy_since_epoch=0
flatscan_helper_restart_count=0
flatscan_helper_health_state=starting
flatscan_observer_sequence=0
flatscan_observer_source=""
flatscan_observer_kind=""
flatscan_observer_state=starting
calls=0
healthy=0
probes=0
probe_result=1
upstream=0
mock_sequence=1
mock_source=guard-1:localizer-1
mock_kind=input
checker_rc=50
write_flatscan_helper_status() { :; }
run_flatscan_supervision_probe() {
  printf 'status=fixture source_id=%s evidence_kind=%s evidence_sequence=%s\n' "$mock_source" "$mock_kind" "$mock_sequence"
  return "$checker_rc"
}
note_flatscan_healthy() {
  healthy=$((healthy + 1)); flatscan_helper_graph_miss_count=0; flatscan_helper_health_state=healthy;
}
confirm_flatscan_stream_after_graph_misses() { probes=$((probes + 1)); return "$probe_result"; }
scan_publisher_exists() { return "$upstream"; }
restart_flatscan_helper_if_allowed() { calls=$((calls + 1)); flatscan_helper_graph_miss_count=0; }
ros2() { echo "unexpected ROS CLI" >&2; exit 99; }
'''
    result = subprocess.run([bash_binary()], input=functions + "\n" + harness + "\n" + body,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_normal_operation_does_not_probe_ros():
    run_shell("checker_rc=0; for mock_sequence in 1 2 3 4; do observe_flatscan_helper; done\n"
              "[[ $healthy == 4 && $calls == 0 && $probes == 0 ]]")


@pytest.mark.parametrize("code", [1, 40, 41, 42, 43, 44, 46, 127])
def test_observer_failure_never_means_producer_fault(code):
    run_shell(f"checker_rc={code}; flatscan_helper_graph_miss_count=2\n"
              "observe_flatscan_helper\n"
              "[[ $calls == 0 && $probes == 0 && $flatscan_helper_graph_miss_count == 0 && "
              "$flatscan_helper_health_state == observer_unavailable ]]")


def test_repeated_evidence_cannot_accumulate_failures():
    run_shell("for i in 1 2 3 4 5; do observe_flatscan_helper; done\n"
              "[[ $calls == 0 && $probes == 0 && $flatscan_helper_graph_miss_count == 1 ]]")


def test_actual_three_candidates_still_require_independent_confirmation():
    run_shell("for mock_sequence in 1 2 3; do observe_flatscan_helper; done\n"
              "[[ $calls == 1 && $probes == 1 ]]")


@pytest.mark.parametrize("setting", ["probe_result=0", "upstream=1"])
def test_flowing_or_unconfirmed_upstream_does_not_restart(setting):
    run_shell(setting + "\nfor mock_sequence in 1 2 3; do observe_flatscan_helper; done\n"
              "[[ $calls == 0 && $probes == 1 ]]")


def test_generation_changes_reset_failure_evidence():
    run_shell("for mock_sequence in 1 2; do observe_flatscan_helper; done\n"
              "mock_source=guard-2:localizer-1; mock_sequence=3; observe_flatscan_helper\n"
              "[[ $calls == 0 && $probes == 0 && $flatscan_helper_graph_miss_count == 1 ]]")


def test_out_of_order_evidence_does_not_accumulate_failures():
    run_shell("mock_sequence=5; observe_flatscan_helper\n"
              "mock_sequence=4; observe_flatscan_helper\n"
              "[[ $calls == 0 && $probes == 0 && $flatscan_observer_sequence == 5 && "
              "$flatscan_helper_graph_miss_count == 1 ]]")


def test_missing_observer_breaks_consecutive_failure_streak():
    run_shell("for mock_sequence in 1 2; do observe_flatscan_helper; done\n"
              "checker_rc=40; observe_flatscan_helper\n"
              "checker_rc=50; mock_sequence=3; observe_flatscan_helper\n"
              "[[ $calls == 0 && $probes == 0 && $flatscan_helper_graph_miss_count == 1 ]]")


@pytest.mark.parametrize("setting", ["mock_sequence=0", "mock_sequence=bad", "mock_source=bad", "mock_kind=bad"])
def test_malformed_evidence_cannot_trigger_confirmation(setting):
    run_shell(setting + "\nfor i in 1 2 3; do observe_flatscan_helper; done\n"
              "[[ $calls == 0 && $probes == 0 && $flatscan_helper_graph_miss_count == 0 ]]")


def test_repeated_healthy_evidence_does_not_reset_restart_budget():
    run_shell("checker_rc=0; flatscan_helper_restart_count=5\n"
              "for i in 1 2 3; do observe_flatscan_helper; done\n"
              "[[ $healthy == 1 && $flatscan_helper_restart_count == 5 && "
              "$flatscan_helper_healthy_since_epoch == 0 ]]")


def test_missing_checker_does_not_fall_back_to_regular_cli():
    run_shell("FLATSCAN_MONITOR_CHECK_BIN=/fixture-does-not-exist\nobserve_flatscan_helper\n"
              "[[ $calls == 0 && $probes == 0 && $flatscan_observer_state == checker_missing ]]")


def test_confirmed_child_exit_keeps_existing_recovery():
    run_shell("driver_pid=fixture; flatscan_helper_mode=standalone; FLATSCAN_SUPERVISE_PERIOD_SEC=0\n"
              "checks=0\nkill() { checks=$((checks + 1)); [[ $checks == 1 ]]; }\n"
              "flatscan_helper_running() { return 1; }\nsleep() { :; }\nwait() { return 0; }\n"
              "supervise_flatscan_helper\n[[ $calls == 1 && $probes == 0 ]]")


def test_shell_syntax_and_atomic_legacy_status(tmp_path):
    result = subprocess.run([bash_binary(), "-n", str(PIPELINE)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    script = function("write_flatscan_helper_status") + "\n" + r'''
set -euo pipefail
FLATSCAN_STATUS_FILE="$1/status.env"
flatscan_status_directory_ready=false
flatscan_observer_state=observer_unavailable
flatscan_observer_source=guard:localizer
flatscan_observer_sequence=1
flatscan_helper_mode=standalone
flatscan_pid=123
flatscan_helper_restart_count=0
flatscan_helper_graph_miss_count=0
flatscan_helper_health_state=observer_unavailable
flatscan_helper_healthy_since_epoch=0
flatscan_helper_restart_cooldown_until_epoch=0
FLATSCAN_HELPER_MISSING_CONFIRMATIONS=3
FLATSCAN_HELPER_REQUIRED=true
FLATSCAN_HELPER_RESTART=true
write_flatscan_helper_status
source "$FLATSCAN_STATUS_FILE"
[[ $FLATSCAN_HELPER_PID == 123 && $FLATSCAN_HELPER_HEALTH_STATE == observer_unavailable ]]
[[ $FLATSCAN_HELPER_UPDATED_AT == *Z ]]
'''
    result = subprocess.run([bash_binary(), "-s", "--", str(tmp_path).replace("\\", "/")],
                            input=script, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert not list(tmp_path.glob("*.tmp.*"))


def test_metadata_is_additive_and_uses_existing_receive_callback():
    text = (ROOT / "src/robot_global_localization/src/global_localization_node.cpp").read_text(encoding="utf-8")
    assert text.count("create_subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>") == 1
    assert "localizer_input_.received_monotonic_sec = steady_now_seconds();" in text
    monitor = (ROOT / "src/robot_bringup/src/runtime_flatscan_monitor.cpp").read_text(encoding="utf-8")
    assert "create_subscription<std_msgs::msg::String>" in monitor
    assert "create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false)" in monitor
    assert "count_publishers(" not in monitor
    assert "create_timer" not in monitor
    assert "FlatScan>" not in monitor and "LaserScan>" not in monitor


@pytest.mark.parametrize("probe_rc", [0, 1])
def test_unknown_startup_preserves_bounded_oneoff_publisher_probe(tmp_path, probe_rc):
    path = ROOT / "scripts/jetson/runtime_overlay/scripts/run_occupancy_grid_localization.sh"
    text = path.read_text(encoding="utf-8")
    definitions = []
    for name, opening, closing in (
        ("flatscan_publisher_ready_for_localization", r"\{", r"\}"),
        ("flatscan_supervisor_status_ready_for_localization", r"\(", r"\)"),
        ("require_common_pointcloud_for_localization", r"\{", r"\}"),
    ):
        match = re.search(rf"^{name}\(\) {opening}\n.*?^{closing}", text, re.M | re.S)
        assert match, name
        definitions.append(match.group(0))
    harness = r'''
set -euo pipefail
NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
NJRH_FLATSCAN_HELPER_STATUS_FILE="$1/status.env"
LOCALIZER_FLATSCAN_TOPIC=/flatscan
unset NJRH_LOCALIZATION_COMMON_FLATSCAN_WAIT_SEC
printf 'FLATSCAN_HELPER_HEALTH_STATE=observer_unavailable\nFLATSCAN_HELPER_PID=unused\n' \
  > "$NJRH_FLATSCAN_HELPER_STATUS_FILE"
pointcloud_accel_pipeline_runtime_running() { return 0; }
calls=0
wait_for_topic_publisher_from_node() {
  calls=$((calls + 1))
  [[ $1 == /flatscan && $2 == laser_scan_to_flatscan && $3 == 20 ]] || exit 97
  return "$probe_rc"
}
restart_flatscan_helper_if_allowed() { exit 98; }
ros2() { exit 99; }
if flatscan_supervisor_status_ready_for_localization; then exit 96; fi
rc=0
require_common_pointcloud_for_localization || rc=$?
[[ $calls == 1 && $rc == "$probe_rc" ]]
source "$NJRH_FLATSCAN_HELPER_STATUS_FILE"
[[ $FLATSCAN_HELPER_HEALTH_STATE == observer_unavailable ]]
'''
    script = "\n".join(definitions) + f"\nprobe_rc={probe_rc}\n" + harness
    result = subprocess.run([bash_binary(), "-s", "--", str(tmp_path).replace("\\", "/")],
                            input=script, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_native_snapshot_policy(tmp_path):
    command = shlex.split(os.environ.get("CXX", ""))
    if not command:
        found = next((p for c in ("c++", "g++", "clang++") if (p := shutil.which(c))), None)
        if not found:
            pytest.skip("No local C++ compiler; parent must run native policy/ROS build in isolation")
        command = [found]
    command += ["-std=c++17", "-O2", "-I", str(ROOT / "src/robot_bringup/include")]
    if os.environ.get("RAPIDJSON_INCLUDE_DIR"):
        command += ["-I", os.environ["RAPIDJSON_INCLUDE_DIR"]]
    binary = tmp_path / ("flatscan_policy.exe" if os.name == "nt" else "flatscan_policy")
    command += [str(ROOT / "src/robot_bringup/test/test_runtime_flatscan_snapshot.cpp"), "-o", str(binary)]
    build = subprocess.run(command, capture_output=True, text=True, timeout=90)
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=15)
    assert run.returncode == 0, run.stdout + run.stderr

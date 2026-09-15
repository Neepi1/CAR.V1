"""Run the actual ROS-free health reader against legacy and C++ snapshots.

Set NJRH_RUNTIME_HEALTH_CHECK_BIN to test an uninstalled candidate. Without it,
use the installed executable or compile just the CLI in pytest's temp directory.
No ROS node, service owner, systemctl command, or robot connection is started.
"""

import copy
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import time
import unittest

import pytest


ROOT = Path(__file__).resolve().parents[3]
CPP_METADATA = (
    "implementation", "odom_watch", "generation", "sequence", "updated_monotonic_sec",
    "boot_id", "clock_valid", "sampling_delayed", "sample_period_sec", "message_count_semantics",
)
TOPIC_HEALTH_CHECKS = (
    ("docking_observation_fresh", "/dock/target_observation"),
    ("docking_sensor_healthy", "/dock/target_observation"),
    ("safety_status_fresh", "/safety/status"),
    ("local_scan_fresh", "/scan"),
    ("local_costmap_fresh", "/local_costmap/costmap"),
    ("global_costmap_fresh", "/global_costmap/costmap"),
)


def resolve_runtime_health_check(build_dir):
    """Resolve a real binary; build failures must fail, never silently skip."""
    configured = os.environ.get("NJRH_RUNTIME_HEALTH_CHECK_BIN")
    if configured:
        binary = Path(configured).expanduser().resolve()
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise AssertionError(f"Configured health checker is not executable: {binary}")
        return binary

    suffix = ".exe" if os.name == "nt" else ""
    for prefix in (ROOT / "install" / "robot_bringup", ROOT / "install"):
        binary = prefix / "lib" / "robot_bringup" / ("runtime_health_check" + suffix)
        if binary.is_file() and os.access(binary, os.X_OK):
            return binary

    compiler = shlex.split(os.environ["CXX"]) if os.environ.get("CXX") else []
    if not compiler:
        compiler = next(([found] for name in ("c++", "g++", "clang++")
                         if (found := shutil.which(name))), [])
    if not compiler or not shutil.which(compiler[0]):
        raise unittest.SkipTest(
            "No health-check binary or C++ compiler; set NJRH_RUNTIME_HEALTH_CHECK_BIN"
        )

    build_dir = Path(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)
    binary = build_dir / ("runtime_health_check" + suffix)
    command = compiler + [
        "-std=c++17", "-O2", "-I", str(ROOT / "src/robot_bringup/include"),
    ]
    if os.environ.get("RAPIDJSON_INCLUDE_DIR"):
        command += ["-I", os.environ["RAPIDJSON_INCLUDE_DIR"]]
    command += [str(ROOT / "src/robot_bringup/src/runtime_health_check.cpp"),
                "-o", str(binary)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, (
        f"Standalone health-check build failed (no ROS/DDS libraries linked):\n"
        f"{result.stdout}\n{result.stderr}"
    )
    return binary


def legacy_snapshot():
    now = time.time()
    return {
        "updated_at": now,
        "summary": {
            "local_state_endpoint_ready": True,
            "local_state_fastlio_endpoint_ready": True,
            "localization_bridge_endpoint_ready": True,
            "local_state_topic_ready": True,
            "local_state_ready": True,
            "local_odom_fresh": True,
            "odom_base_tf_fresh": True,
            "map_odom_tf_ready": True,
            "docking_sensor_healthy": True,
        },
        "topics": {
            "/local_state/odometry": {
                "publishers": 1, "last_received_at": now,
                "last_stamp_sec": now - 0.03, "last_age_sec": 0.03,
                "message_count": 100,
            },
            "/scan": {"publishers": 1, "last_received_at": now},
        },
        "tf": {
            "odom->base_link": {"last_age_sec": 0.03},
            "map->odom": {"last_age_sec": 0.05},
        },
    }


def cpp_snapshot(no_update_age_sec=0.0, seen_valid=True, sequence=1, generation="test-gen"):
    snapshot = legacy_snapshot()
    snapshot.update({
        "schema": "njrh.runtime_health.v1", "implementation": "cpp",
        "generation": generation, "sequence": sequence, "clock_valid": True,
        "updated_monotonic_sec": time.monotonic(),
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8"),
        "sampling_delayed": False,
        "sample_period_sec": 1.0,
        "message_count_semantics": "sampled_messages_not_publisher_rate",
        "odom_watch": {
            "no_update_age_sec": no_update_age_sec, "timeout_sec": 3.0,
            "seen_valid": seen_valid,
        },
    })
    now = snapshot["updated_at"]
    for topic, message_type in {
        "/dock/target_observation": "robot_interfaces/msg/DockTargetObservation",
        "/safety/status": "std_msgs/msg/String",
        "/scan": "sensor_msgs/msg/LaserScan",
        "/local_costmap/costmap": "nav_msgs/msg/OccupancyGrid",
        "/global_costmap/costmap": "nav_msgs/msg/OccupancyGrid",
    }.items():
        snapshot["topics"][topic] = {
            "type": message_type, "publishers": 1, "subscriptions": 1,
            "last_received_at": now, "last_age_sec": 0.03, "message_count": 5,
            "last_stamp_sec": None if topic == "/safety/status" else now - 0.03,
        }
    snapshot["topics"]["/dock/target_observation"].update(
        sensor_healthy=True, valid=True, source="orbbec_336l_depth", reason="test_fixture",
    )
    snapshot["summary"].update({key: True for key, _ in TOPIC_HEALTH_CHECKS})
    return snapshot


@pytest.fixture(scope="module")
def health_check_bin(tmp_path_factory):
    return resolve_runtime_health_check(tmp_path_factory.mktemp("health_check_cli"))


@pytest.fixture
def query(health_check_bin, tmp_path):
    def run(snapshot, *args, max_age="2.0"):
        path = tmp_path / "health.json"
        if snapshot is not None:
            path.write_text(
                snapshot if isinstance(snapshot, str) else json.dumps(snapshot, allow_nan=False),
                encoding="utf-8",
            )
        elif path.exists():
            path.unlink()
        return subprocess.run(
            [str(health_check_bin), str(path), max_age, *args],
            capture_output=True, text=True, timeout=5,
            env={**os.environ, "NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC": "0.75",
                 "NJRH_RUNTIME_HEALTH_TF_FRESH_SEC": "0.25",
                 "NJRH_RUNTIME_HEALTH_MAP_TF_FRESH_SEC": "1.0",
                 "NJRH_RUNTIME_HEALTH_DOCKING_FRESH_SEC": "1.5",
                 "NJRH_RUNTIME_HEALTH_SCAN_FRESH_SEC": "1.0",
                 "NJRH_RUNTIME_HEALTH_TOPIC_FRESH_SEC": "1.5"},
        )
    return run


def assert_diagnostic(result, code, status, evidence=None):
    assert result.returncode == code, result.stdout + result.stderr
    tokens = dict(item.split("=", 1) for item in result.stdout.split() if "=" in item)
    assert tokens["status"] == status
    if evidence is not None:
        assert tokens["evidence_id"] == evidence
    return tokens


def test_legacy_snapshot_available_and_ready(query):
    snapshot = legacy_snapshot()
    assert query(snapshot, "available").returncode == 0
    tokens = assert_diagnostic(query(snapshot, "diagnostic"), 0, "ready")
    assert tokens["evidence_id"] == f"legacy:{int(snapshot['updated_at'] * 1e6)}"


def test_legacy_schema_without_cpp_metadata_remains_compatible(query):
    snapshot = legacy_snapshot()
    snapshot["schema"] = "njrh.runtime_health.v1"
    assert_diagnostic(query(snapshot, "diagnostic"), 0, "ready")


@pytest.mark.parametrize("age", [0.0, 0.75, 1.0, 2.8])
def test_runtime_grace_is_not_the_startup_freshness_threshold(query, age):
    snapshot = cpp_snapshot(age)
    snapshot["topics"]["/local_state/odometry"]["last_age_sec"] = age
    snapshot["summary"].update(local_odom_fresh=False, local_state_topic_ready=False)
    assert_diagnostic(query(snapshot, "diagnostic", "0.75"), 0, "ready", "test-gen:1")
    assert query(snapshot, "check", "local_state_topic_ready").returncode == 1


@pytest.mark.parametrize("age", [3.0, 3.001, 10.0])
@pytest.mark.parametrize("seen_valid", [False, True])
def test_three_seconds_without_fresh_update_is_fault_56(query, age, seen_valid):
    snapshot = cpp_snapshot(age, seen_valid, sequence=17)
    # A stale summary cannot override the monotonic no-update evidence.
    snapshot["odom_watch"]["fault_candidate"] = False
    assert_diagnostic(query(snapshot, "diagnostic"), 56, "odom_no_fresh_update", "test-gen:17")


@pytest.mark.parametrize("age", [0.0, 2.8])
def test_initial_sampling_warmup_is_observer_status_not_odom_fault(query, age):
    snapshot = cpp_snapshot(age, seen_valid=False)
    snapshot["summary"]["local_state_endpoint_ready"] = False
    snapshot["topics"]["/local_state/odometry"]["publishers"] = 0
    assert_diagnostic(query(snapshot, "diagnostic"), 45, "observer_sampling_warmup")


def test_fresh_sample_with_missing_graph_is_observer_inconsistency(query):
    for snapshot in (legacy_snapshot(), cpp_snapshot(0.2)):
        snapshot["summary"]["local_state_endpoint_ready"] = False
        snapshot["topics"]["/local_state/odometry"]["publishers"] = 0
        assert_diagnostic(query(snapshot, "diagnostic"), 44, "observer_graph_inconsistent")


@pytest.mark.parametrize("case,code,status", [
    ("endpoint", 50, "endpoint_missing"),
    ("publisher", 51, "publisher_missing"),
    ("unseen", 52, "odom_unseen"),
    ("stamp", 53, "odom_stamp_stale"),
    ("receive", 53, "odom_stamp_stale"),
])
def test_legacy_fault_classification_remains_compatible(query, case, code, status):
    snapshot = legacy_snapshot()
    topic = snapshot["topics"]["/local_state/odometry"]
    topic["last_age_sec"] = 10.0
    if case == "endpoint":
        snapshot["summary"]["local_state_endpoint_ready"] = False
    elif case == "publisher":
        topic["publishers"] = 0
    elif case == "unseen":
        topic["last_received_at"] = None
    elif case == "receive":
        topic["last_age_sec"] = 0.03
        topic["last_received_at"] = 1.0
    assert_diagnostic(query(snapshot, "diagnostic"), code, status)


@pytest.mark.parametrize("field,value", [
    ("no_update_age_sec", None), ("no_update_age_sec", -1.0),
    ("no_update_age_sec", "3.0"), ("timeout_sec", 0),
    ("timeout_sec", -3.0), ("timeout_sec", None),
    ("seen_valid", None), ("seen_valid", "true"),
])
def test_invalid_watch_cannot_be_reported_as_real_odom_fault(query, field, value):
    snapshot = cpp_snapshot(5.0)
    snapshot["odom_watch"][field] = value
    assert_diagnostic(query(snapshot, "diagnostic"), 41, "observer_invalid")


def test_snapshot_observer_errors_take_priority_over_odom_fault(query):
    snapshot = cpp_snapshot(10.0)
    stale = copy.deepcopy(snapshot)
    stale["updated_at"] = 1.0
    stale["updated_monotonic_sec"] = time.monotonic() - 60
    future = copy.deepcopy(snapshot)
    future["updated_at"] = time.time() + 60
    future["updated_monotonic_sec"] = time.monotonic() + 60
    clock_invalid = copy.deepcopy(snapshot)
    clock_invalid["clock_valid"] = False
    delayed = copy.deepcopy(snapshot)
    delayed["sampling_delayed"] = True
    for data, code, status in (
        (None, 40, "observer_unavailable"),
        ("{broken", 41, "observer_invalid"),
        ("[]", 41, "observer_invalid"),
        (stale, 42, "observer_stale"),
        (future, 43, "observer_clock_invalid"),
        (clock_invalid, 43, "observer_clock_invalid"),
        (delayed, 46, "observer_sampling_delayed"),
    ):
        assert_diagnostic(query(data, "diagnostic"), code, status)
        assert query(data, "available").returncode == code


@pytest.mark.parametrize("field", [
    "schema", "implementation", "odom_watch", "sequence", "generation",
    "updated_monotonic_sec", "clock_valid", "boot_id",
])
def test_cpp_snapshot_requires_its_complete_envelope(query, field):
    snapshot = cpp_snapshot(3.0)
    del snapshot[field]
    assert_diagnostic(query(snapshot, "diagnostic"), 41, "observer_invalid")


@pytest.mark.parametrize("sequence", [0, -1, 0.5, "1", None, 2**64])
def test_cpp_sequence_must_be_positive_uint64(query, sequence):
    snapshot = cpp_snapshot(3.0, sequence=sequence)
    assert_diagnostic(query(snapshot, "diagnostic"), 41, "observer_invalid")


def test_cpp_boot_identity_must_match_including_newline(query):
    snapshot = cpp_snapshot(3.0)
    for boot_id in ("another-boot\n", snapshot["boot_id"].rstrip("\n")):
        snapshot["boot_id"] = boot_id
        assert_diagnostic(query(snapshot, "diagnostic"), 42, "observer_stale")


@pytest.mark.parametrize("marker", CPP_METADATA)
@pytest.mark.parametrize("null_value", [False, True], ids=["value", "null"])
def test_partial_cpp_snapshot_cannot_fall_back_to_legacy(query, marker, null_value):
    snapshot = legacy_snapshot()
    snapshot[marker] = None if null_value else cpp_snapshot()[marker]
    assert_diagnostic(query(snapshot, "diagnostic"), 41, "observer_invalid")
    assert query(snapshot, "available").returncode == 41
    assert query(snapshot, "check", "docking_sensor_healthy").returncode == 41


@pytest.mark.parametrize("field,value", [
    ("schema", "njrh.runtime_health.v0"), ("implementation", "python"),
    ("generation", ""), ("generation", "bad id"), ("updated_monotonic_sec", "NaN"),
    ("clock_valid", "true"), ("boot_id", ""),
])
def test_cpp_invalid_envelope_fields_are_observer_invalid(query, field, value):
    snapshot = cpp_snapshot(4.0)
    snapshot[field] = value
    assert_diagnostic(query(snapshot, "diagnostic"), 41, "observer_invalid")


def test_cpp_wall_clock_jump_is_observer_error_not_odom_fault(query):
    for wall_jump in (-3600.0, 3600.0):
        snapshot = cpp_snapshot(10.0)
        snapshot["updated_at"] += wall_jump
        assert_diagnostic(query(snapshot, "diagnostic"), 43, "observer_clock_invalid")
        assert query(snapshot, "available").returncode == 43


def test_evidence_identity_distinguishes_generation_and_sequence(query):
    for generation, sequence in (("g1", 7), ("g1", 8), ("g2", 7)):
        snapshot = cpp_snapshot(3.0, generation=generation, sequence=sequence)
        assert_diagnostic(query(snapshot, "diagnostic"), 56, "odom_no_fresh_update",
                          f"{generation}:{sequence}")


@pytest.mark.parametrize("key", [
    "local_state_endpoint", "local_state_fastlio_endpoint", "localization_bridge_endpoint",
    "local_state_ready", "local_state_topic_ready", "local_odom_fresh", "docking_sensor_healthy",
    "odom_base_tf_fresh", "map_odom_tf_ready",
    "scan_topic_seen", "odom_base_tf_seen", "map_odom_tf_seen",
])
def test_check_interface_preserves_existing_helper_keys(query, key):
    snapshot = legacy_snapshot()
    assert query(snapshot, "check", key).returncode == 0
    snapshot.update(summary={}, topics={}, tf={})
    assert query(snapshot, "check", key).returncode == 1


def test_check_requires_boolean_true_not_truthy_string(query):
    snapshot = cpp_snapshot()
    snapshot["summary"]["docking_sensor_healthy"] = "false"
    assert query(snapshot, "check", "docking_sensor_healthy").returncode == 1
    assert query(snapshot, "check", "unknown_key").returncode == 1


def test_topic_requires_a_publisher_and_observed_message(query):
    snapshot = cpp_snapshot()
    assert query(snapshot, "topic", "/scan").returncode == 0
    snapshot["topics"]["/scan"]["publishers"] = 0
    assert query(snapshot, "topic", "/scan").returncode == 1
    snapshot["topics"]["/scan"].update(publishers=1, last_received_at=None)
    assert query(snapshot, "topic", "/scan").returncode == 1
    assert query(snapshot, "topic", "/not_observed").returncode == 1


@pytest.mark.parametrize("age,expected", [(0.03, 0), (0.20, 0), (0.30, 1), (-0.5, 1), (None, 1)])
def test_tf_age_and_slash_normalization(query, age, expected):
    snapshot = cpp_snapshot()
    snapshot["tf"]["odom->base_link"]["last_age_sec"] = age
    assert query(snapshot, "tf", "/odom", "/base_link", "0.25").returncode == expected
    assert query(snapshot, "tf", "missing", "base_link", "0.25").returncode == 1


@pytest.mark.parametrize("format", ["legacy", "cpp"])
@pytest.mark.parametrize("key", [
    "local_odom_fresh", "local_state_topic_ready", "local_state_ready",
    "odom_base_tf_fresh", "map_odom_tf_ready",
])
def test_freshness_checks_age_cached_samples_at_query_time(query, format, key):
    snapshot = cpp_snapshot() if format == "cpp" else legacy_snapshot()
    snapshot["updated_at"] -= 1.2
    if format == "cpp":
        snapshot["updated_monotonic_sec"] -= 1.2
    assert query(snapshot, "available").returncode == 0
    assert query(snapshot, "check", key).returncode == 1
    assert query(snapshot, "check", "local_state_endpoint").returncode == 0
    assert query(snapshot, "tf", "odom", "base_link", "0.25").returncode == 1


def test_runtime_watch_also_accounts_for_elapsed_snapshot_age(query):
    snapshot = cpp_snapshot(2.5)
    snapshot["updated_at"] -= 0.75
    snapshot["updated_monotonic_sec"] -= 0.75
    assert_diagnostic(query(snapshot, "diagnostic"), 56, "odom_no_fresh_update")


@pytest.mark.parametrize("format", ["legacy", "cpp"])
@pytest.mark.parametrize("key,topic", TOPIC_HEALTH_CHECKS)
def test_topic_health_adds_snapshot_age_to_cached_sample_age(query, format, key, topic):
    full = cpp_snapshot()
    snapshot = full if format == "cpp" else legacy_snapshot()
    snapshot["topics"][topic] = copy.deepcopy(full["topics"][topic])
    snapshot["summary"][key] = True
    sample = snapshot["topics"][topic]
    sample["last_age_sec"] = 0.4
    if sample["last_stamp_sec"] is not None:
        sample["last_stamp_sec"] = snapshot["updated_at"] - 0.4
    else:
        sample["last_received_at"] = snapshot["updated_at"] - 0.4
    assert query(snapshot, "check", key).returncode == 0
    # For the 1.5s limits, neither age alone is stale. Their 1.7s sum is stale,
    # also exceeding the scan's 1.0s limit, while the 2.0s snapshot remains usable.
    snapshot["updated_at"] -= 1.3
    if format == "cpp":
        snapshot["updated_monotonic_sec"] -= 1.3
    sample["last_received_at"] -= 1.3
    if sample["last_stamp_sec"] is not None:
        sample["last_stamp_sec"] -= 1.3
    assert query(snapshot, "available").returncode == 0
    assert query(snapshot, "check", key).returncode == 1


@pytest.mark.parametrize("key,topic", TOPIC_HEALTH_CHECKS)
def test_cpp_topic_health_requires_a_sample_even_when_summary_says_healthy(query, key, topic):
    snapshot = cpp_snapshot()
    del snapshot["topics"][topic]
    assert query(snapshot, "available").returncode == 0
    assert query(snapshot, "check", key).returncode == 1


@pytest.mark.parametrize("key,topic", TOPIC_HEALTH_CHECKS)
def test_legacy_missing_optional_sample_preserves_summary_compatibility(query, key, topic):
    snapshot = legacy_snapshot()
    snapshot["topics"].pop(topic, None)
    snapshot["summary"][key] = True
    assert query(snapshot, "check", key).returncode == 0
    snapshot["summary"][key] = False
    assert query(snapshot, "check", key).returncode == 1


@pytest.mark.parametrize("age", [None, "0.03", -0.5, 2.0])
@pytest.mark.parametrize("key,topic", TOPIC_HEALTH_CHECKS)
def test_cpp_topic_health_rejects_invalid_or_stale_sample_age(query, key, topic, age):
    snapshot = cpp_snapshot()
    snapshot["topics"][topic]["last_age_sec"] = age
    assert query(snapshot, "check", key).returncode == 1


@pytest.mark.parametrize("key,topic", TOPIC_HEALTH_CHECKS)
def test_cpp_fresh_sample_cannot_override_unhealthy_summary(query, key, topic):
    snapshot = cpp_snapshot()
    assert query(snapshot, "check", key).returncode == 0
    snapshot["summary"][key] = False
    assert query(snapshot, "check", key).returncode == 1


@pytest.mark.parametrize("args", [("unknown",), ("check",), ("topic",), ("tf", "odom", "base_link")])
def test_invalid_cli_command_or_missing_arguments_is_not_odom_fault(query, args):
    assert query(cpp_snapshot(), *args).returncode == 41


@pytest.mark.parametrize("max_age", ["0", "-1", "nan", "inf"])
def test_invalid_snapshot_age_limit_is_not_odom_fault(query, max_age):
    assert query(cpp_snapshot(), "available", max_age=max_age).returncode == 41

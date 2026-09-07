from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
RECORDER_PATH = (
    WORKSPACE_ROOT
    / "scripts"
    / "jetson"
    / "runtime_overlay"
    / "scripts"
    / "record_docking_rotation_trace.py"
)
WRAPPER_PATH = RECORDER_PATH.with_suffix(".sh")


def load_recorder_module():
    spec = importlib.util.spec_from_file_location(
        "record_docking_rotation_trace", RECORDER_PATH
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def wheel_segment(start: float, end: float, yaw_start: float, yaw_end: float):
    result = []
    steps = int(round((end - start) / 0.05))
    for index in range(steps + 1):
        ratio = index / max(1, steps)
        result.append(
            {
                "elapsed_s": start + ratio * (end - start),
                "angular_z": 0.10,
                "yaw_rad": yaw_start + ratio * (yaw_end - yaw_start),
            }
        )
    return result


def command_segment(topic: str, start: float, end: float, angular_z: float):
    result = []
    steps = int(round((end - start) / 0.05))
    for index in range(steps + 1):
        ratio = index / max(1, steps)
        result.append(
            {
                "elapsed_s": start + ratio * (end - start),
                "topic": topic,
                "linear_x": 0.0,
                "linear_y": 0.0,
                "angular_z": angular_z,
            }
        )
    return result


def test_detects_nav2_then_api_predock_rotate_stop_rotate():
    recorder = load_recorder_module()
    wheel = [
        *wheel_segment(1.0, 2.0, 0.0, 0.10),
        *wheel_segment(5.0, 6.0, 0.10, 0.18),
    ]
    commands = [
        *command_segment(recorder.COMMAND_TOPICS["nav_raw"], 0.9, 2.1, 0.12),
        *command_segment(recorder.COMMAND_TOPICS["nav_smoothed"], 0.9, 2.1, 0.12),
        *command_segment(
            recorder.COMMAND_TOPICS["collision_checked"], 0.9, 2.1, 0.12
        ),
        *command_segment(recorder.COMMAND_TOPICS["final"], 0.9, 2.1, 0.12),
        *command_segment(recorder.COMMAND_TOPICS["docking"], 4.9, 6.1, 0.08),
        *command_segment(recorder.COMMAND_TOPICS["final"], 4.9, 6.1, 0.08),
    ]
    api = [
        {
            "elapsed_s": 1.5,
            "docking_phase": "PREDOCK_NAVIGATION",
            "navigation_phase": "running",
            "predock_yaw_align_active": False,
        },
        {
            "elapsed_s": 5.5,
            "docking_phase": "PREDOCK_YAW_ALIGN_RECOVERY",
            "navigation_phase": "succeeded",
            "predock_yaw_align_active": True,
        },
    ]

    result = recorder.analyze_capture(commands, wheel, api, [])

    assert result["verdict"] == "RED_ROTATE_STOP_ROTATE_CAPTURED"
    assert result["symptom_detected"] is True
    assert [row["source"] for row in result["physical_rotation_episodes"]] == [
        "NAV2_RAW",
        "API_PREDOCK_YAW",
    ]
    assert "NAV2_TO_DOCKING_HANDOFF" in result["pause_windows"][0]["reasons"]


def test_pause_classifies_collision_monitor_suppression():
    recorder = load_recorder_module()
    previous = {"end_s": 2.0, "source": "NAV2_RAW"}
    following = {"start_s": 3.0, "source": "NAV2_RAW"}
    commands = [
        *command_segment(recorder.COMMAND_TOPICS["nav_raw"], 2.1, 2.9, 0.10),
        *command_segment(recorder.COMMAND_TOPICS["nav_smoothed"], 2.1, 2.9, 0.10),
    ]

    pause = recorder.classify_pause(previous, following, commands, [], [])

    assert "COLLISION_MONITOR_SUPPRESSION" in pause["reasons"]


def test_idle_trace_is_green_capable_without_false_two_stage_result():
    recorder = load_recorder_module()
    wheel = [
        {"elapsed_s": index * 0.05, "angular_z": 0.0, "yaw_rad": 0.0}
        for index in range(20)
    ]

    result = recorder.analyze_capture([], wheel, [], [])

    assert result["verdict"] == "NO_TWO_STAGE_ROTATION_CAPTURED"
    assert result["symptom_detected"] is False
    assert result["pause_windows"] == []


def test_api_token_can_be_resolved_from_unique_runtime_process(tmp_path, monkeypatch):
    recorder = load_recorder_module()
    monkeypatch.delenv("ROBOT_API_TOKEN", raising=False)
    process_dir = tmp_path / "123"
    process_dir.mkdir()
    (process_dir / "cmdline").write_bytes(
        b"/workspace/install/robot_api_server/lib/robot_api_server/"
        b"robot_api_server_node\0--ros-args\0"
    )
    (process_dir / "environ").write_bytes(
        b"RMW_IMPLEMENTATION=rmw_fastrtps_cpp\0"
        b"ROBOT_API_TOKEN=field-test-token\0"
    )

    token, source = recorder.resolve_api_token(tmp_path)

    assert token == "field-test-token"
    assert source == "robot_api_server_process"


def test_recorder_contract_is_read_only_and_excludes_heavy_topics():
    python_source = RECORDER_PATH.read_text(encoding="utf-8")
    wrapper_source = WRAPPER_PATH.read_text(encoding="utf-8")

    assert "create_publisher(" not in python_source
    assert "create_client(" not in python_source
    assert "PointCloud2" not in python_source
    assert "LaserScan" not in python_source
    assert '"/scan"' not in python_source
    assert "ROBOT_API_TOKEN" in python_source
    assert "/tmp/njrh_reports/" in python_source
    assert "/tmp/njrh_reports/" in wrapper_source
    for topic in (
        "/cmd_vel_nav_raw",
        "/cmd_vel_docking",
        "/cmd_vel_collision_checked",
        "/cmd_vel",
    ):
        assert topic in python_source

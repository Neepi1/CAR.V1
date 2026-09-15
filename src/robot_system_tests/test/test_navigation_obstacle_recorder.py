"""Exercise the recorder's actual callbacks without ROS or robot motion."""
import ast
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import ModuleType, SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/jetson/runtime_overlay/scripts/observe_navigation_failure_minimal.sh"


def script_text():
    if os.environ.get("NAV_OBSERVER_BASELINE") == "1":
        return subprocess.check_output(
            ["git", "show", "HEAD:" + SCRIPT.relative_to(ROOT).as_posix()], cwd=ROOT
        ).decode("utf-8")
    return SCRIPT.read_text(encoding="utf-8")


@pytest.fixture
def recorder():
    script = script_text()
    payload = script.split("<<'PY'\n", 1)[1].rsplit("\nPY\n", 1)[0]
    tree = ast.parse(payload)
    compile(tree, str(SCRIPT), "exec")
    definitions = ast.Module(body=[n for n in tree.body if isinstance(
        n, (ast.FunctionDef, ast.ClassDef))], type_ignores=[])
    ns = {"Node": object, "time": time, "math": math, "json": json,
          "hashlib": hashlib, "ackermann_min_turning_radius_m": 0.81,
          "store_costmap_snapshots": True}
    exec(compile(definitions, str(SCRIPT), "exec"), ns)
    return ns


def twist(x=0.0, y=0.0, yaw=0.0):
    return NS(linear=NS(x=x, y=y, z=0.0), angular=NS(x=0.0, y=0.0, z=yaw))


def observer(ns):
    node = ns["MinimalNavigationObserver"].__new__(ns["MinimalNavigationObserver"])
    node.started_monotonic = time.monotonic()
    node.scalar_status = {}
    node.event_counts = {}
    node.events_file = io.StringIO()
    return node


def test_lateral_is_not_misreported_as_zero(recorder):
    shape = recorder["command_shape"]
    assert shape(twist(y=0.05))["shape"] == "lateral"
    assert shape(twist(x=0.1, y=0.02))["shape"] == "mixed_lateral"
    assert shape(twist())["shape"] == "zero"
    assert shape(twist(x=-0.08, yaw=-0.09))["shape"] == "ackermann_feasible"


def test_permit_refresh_keeps_age_without_duplicate_transition(recorder):
    node = observer(recorder)
    assert node.scalar_snapshot() == {}  # unobserved is never false
    topic = "/ranger_mini3/nav_terminal_reverse_enable"
    node.on_scalar(topic, NS(data=True))
    node.scalar_status[topic]["received_monotonic"] -= 1.0
    assert node.scalar_snapshot()[topic]["receive_age_sec"] >= 1.0
    node.on_scalar(topic, NS(data=True))
    assert node.scalar_snapshot()[topic]["receive_age_sec"] < 0.5
    node.on_scalar(topic, NS(data=False))
    assert node.event_counts["scalar_state_changed"] == 2
    assert node.scalar_snapshot()[topic]["data"] is False


def test_repaired_path_records_full_geometry_and_detects_same_stamp_change(recorder):
    node = observer(recorder)
    topic = "/ranger_mini3/ordinary_local_repair_path"
    node.path_stats = {topic: {"count": 0, "empty_count": 0}}
    node.latest_path_geometry = {}
    node.path_fingerprints = {}
    node.path_last_saved = {}
    node.path_frames_file = io.StringIO()
    header = NS(frame_id="map", stamp=NS(sec=12, nanosec=34))
    poses = [NS(header=header, pose=NS(position=NS(x=x, y=0.0, z=0.0),
             orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))) for x in (0.0, 0.5, 1.0)]
    msg = NS(header=header, poses=poses)
    node.on_path(topic, msg)
    node.on_path(topic, msg)
    poses[1].pose.position.y = 0.4
    node.on_path(topic, msg)
    rows = [json.loads(line) for line in node.path_frames_file.getvalue().splitlines()]
    assert len(rows) == 2
    assert rows[0]["sha256"] != rows[1]["sha256"]
    assert rows[1]["poses_xyyaw"][1] == [0.5, 0.4, 0.0]
    assert rows[1]["stamp_sec"] == 12
    assert rows[1]["frame_id"] == "map"


def test_actual_footprint_is_saved_not_reconstructed_from_window(recorder):
    node = observer(recorder)
    node.last_footprint_monotonic = -math.inf
    header = NS(frame_id="odom", stamp=NS(sec=3, nanosec=9))
    points = [NS(x=x, y=y, z=0.0) for x, y in ((0.39, 0.28), (0.39, -0.28),
                                                        (-0.39, -0.28), (-0.39, 0.28))]
    node.on_footprint(NS(header=header, polygon=NS(points=points)))
    assert node.latest_footprint["frame_id"] == "odom"
    assert node.latest_footprint["points_xyz"] == [[p.x, p.y, 0.0] for p in points]


def test_signal_ownership_read_only_and_non_overwriting_contract():
    script = script_text()
    assert "SignalHandlerOptions.NO" in script
    assert "signal.signal(signal.SIGINT, request_stop)" in script
    assert "signal.signal(signal.SIGTERM, request_stop)" in script
    assert "if not ready_printed" in script
    assert 'if ! mkdir "${OUTPUT_DIR}"' in script
    assert "mktemp -d" in script
    assert "ThreadPoolExecutor(max_workers=1)" in script
    assert '"/ranger_base/status"' in script
    assert '"/ranger_mini3_mode_controller/status"' not in script
    for forbidden in ("create_publisher(", "create_client(", "ActionClient(",
                      "ros2 param set", "ros2 service call", "from sensor_msgs.msg import"):
        assert forbidden not in script


@pytest.mark.parametrize("ending", ["operator_interrupt", "decoder_error"])
def test_full_observer_wiring_and_partial_report_offline(monkeypatch, tmp_path, ending):
    """Run the real Python entrypoint against fake DDS, never initialize ROS."""
    class FakeNode:
        def __init__(self, *args, **kwargs):
            assert kwargs == {"enable_rosout": False, "start_parameter_services": False}
            self.subscriptions = {}
            self.timers = []

        def create_subscription(self, msgtype, topic, callback, qos):
            self.subscriptions[topic] = callback
            return callback

        def create_timer(self, period, callback):
            self.timers.append((period, callback))

        def get_topic_names_and_types(self):
            return [("/z_collision/state", ["nav2_msgs/msg/CollisionMonitorState"])] + [
                (f"/critic_{n}", ["std_msgs/msg/String"]) for n in range(100)]

        def count_publishers(self, topic):
            return int(topic == "/z_collision/state")

        def destroy_node(self):
            self.destroyed = True

    def fake_module(name, **values):
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    qos = NS(KEEP_LAST=1, BEST_EFFORT=2, RELIABLE=3, TRANSIENT_LOCAL=4)
    fake_module("rclpy.node", Node=FakeNode)
    fake_module("rclpy.qos", DurabilityPolicy=qos, HistoryPolicy=qos,
                ReliabilityPolicy=qos, QoSProfile=lambda **kwargs: NS(**kwargs))
    fake_module("rclpy.signals", SignalHandlerOptions=NS(NO=0))
    for package, names in {
        "action_msgs.msg": ["GoalStatusArray"],
        "geometry_msgs.msg": ["PolygonStamped", "Twist"],
        "rcl_interfaces.msg": ["Log"],
        "nav2_msgs.msg": ["SpeedLimit", "CollisionMonitorState"],
        "nav_msgs.msg": ["OccupancyGrid", "Odometry", "Path"],
        "std_msgs.msg": ["Bool", "String", "UInt8"],
    }.items():
        fake_module(package, **{name: type(name, (), {}) for name in names})
    action_type = NS(Impl=NS(FeedbackMessage=object))
    fake_module("nav2_msgs.action", FollowPath=action_type, NavigateToPose=action_type)
    ns = {"__name__": "__main__"}
    state = {"ok": True, "spins": 0}

    def spin(node, **kwargs):
        state["spins"] += 1
        if state["spins"] == 1:
            node.started_monotonic -= 7.0  # reach READY without sleeping
            sub = node.subscriptions
            pose = NS(position=NS(x=1., y=2., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
            header = NS(frame_id="odom", stamp=NS(sec=12, nanosec=0))
            sub["/local_state/odometry"](NS(header=header, child_frame_id="base_link",
                pose=NS(pose=pose), twist=NS(twist=twist(x=.3))))
            for name in ("navigate_to_pose", "follow_path"):
                feedback = NS(current_pose=NS(header=NS(frame_id="map", stamp=header.stamp), pose=pose),
                              distance_remaining=2., number_of_recoveries=0, distance_to_goal=2., speed=.3)
                sub[f"/{name}/_action/feedback"](NS(goal_id=NS(uuid=[2]*16), feedback=feedback))
            sub["/cmd_vel_nav_raw"](twist(x=.3))
            sub["/rosout"](NS(name="controller_server", msg="No valid control", level=30,
                stamp=NS(sec=12, nanosec=30), file="optimizer.cpp", function="compute", line=123))
            node.on_sample_timer()
        else:
            node.subscriptions["/z_collision/state"](NS(action_type=2, polygon_name="SlowZone"))
            if ending == "decoder_error":
                raise RuntimeError("synthetic callback decode error")
            ns["request_stop"](signal.SIGINT, None)

    fake_module("rclpy", init=lambda **kw: None, ok=lambda: state["ok"], spin_once=spin,
                shutdown=lambda: state.update(ok=False))
    monkeypatch.setattr(sys, "argv", ["recorder", "60", ".5", "http://unused", str(tmp_path),
                                     "true", "true", "false", "true", ".81", "true", ".5"])
    monkeypatch.setenv("NJRH_NAV_OBSERVER_MODULE_DIR", str(SCRIPT.parent))
    monkeypatch.setenv("ROBOT_API_TOKEN", "")
    script = script_text()
    payload = script.split("<<'PY'\n", 1)[1].rsplit("\nPY\n", 1)[0]
    tree = ast.parse(payload)
    # The process snapshot is separately tested; do not enumerate the host here.
    tree.body = [n for n in tree.body if not (isinstance(n, ast.Expr) and
        isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name) and
        n.value.func.id == "save_runtime_snapshot")]
    saved_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    saved_path = sys.path[:]
    try:
        if ending == "decoder_error":
            with pytest.raises(SystemExit) as exc:
                exec(compile(tree, str(SCRIPT), "exec"), ns)
            assert exc.value.code == 1
        else:
            exec(compile(tree, str(SCRIPT), "exec"), ns)
    finally:
        sys.path[:] = saved_path
        for signum, handler in saved_handlers.items():
            signal.signal(signum, handler)
    summary = json.loads((tmp_path/"summary.json").read_text())
    assert summary["recorder_version"] == 3
    assert summary["stop_reason"].startswith("observer_error" if ending == "decoder_error" else ending)
    assert summary["evidence_availability"]["collision_state"]["status"] == "messages_received"
    assert not summary["evidence_availability"]["full_optimizer_replay_available"]
    assert ns["node"].destroyed and not state["ok"]
    assert all(h.closed for h in ns["node"].evidence.handles.values())
    graph = json.loads((tmp_path/"diagnostic_graph.jsonl").read_text())
    assert len(graph["topics"]) <= 24
    assert not any("critic" in topic for topic in ns["node"].subscriptions)
    feedback = [json.loads(line) for line in (tmp_path/"action_feedback.jsonl").read_text().splitlines()]
    assert len(feedback) == 2
    assert feedback[0]["odometry_match"]["usable_for_approximate_alignment"]
    events = [json.loads(line) for line in (tmp_path/"diagnostic_events.jsonl").read_text().splitlines()]
    assert events[0]["source_stamp"] == {"sec": 12, "nanosec": 30}
    assert events[0]["context"]["commands"]["/cmd_vel_nav_raw"]["last"]["linear"]["x"] == .3
    assert "NOT COLLECTED" in (tmp_path/"summary.md").read_text()

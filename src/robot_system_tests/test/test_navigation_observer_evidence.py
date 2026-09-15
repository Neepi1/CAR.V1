"""Recorder-v3 evidence only: no ROS context, network or robot side effects."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]
MODULE = ROOT / 'scripts/jetson/runtime_overlay/scripts/navigation_observer_evidence.py'


@pytest.fixture
def module():
    spec = importlib.util.spec_from_file_location('navigation_observer_evidence', MODULE)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


@pytest.fixture
def evidence(module, tmp_path):
    obj = module.NavigationEvidence(tmp_path)
    yield obj
    obj.close()


def test_collision_reason_preserved_and_unknown_not_called_clear(evidence):
    assert evidence.collision_snapshot() is None
    evidence.collision('/collision_monitor_state', NS(action_type=1, polygon_name='StopZone'))
    state = evidence.collision_snapshot()
    assert state['action_type'] == 1
    assert state['polygon_name'] == 'StopZone'
    evidence.collision('/collision_monitor_state', NS(action_type=87, polygon_name='future_zone'))
    assert evidence.collision_snapshot()['action_name'] == 'UNKNOWN_87'


def test_duplicate_collision_state_is_counted_not_spammed(evidence):
    msg = NS(action_type=2, polygon_name='SlowZone')
    evidence.collision('/collision_monitor_state', msg)
    evidence.collision('/collision_monitor_state', msg)
    assert evidence.collision_snapshot()['message_count'] == 2
    evidence.flush()
    rows = [json.loads(x) for x in (evidence.directory/'diagnostic_events.jsonl').read_text().splitlines()]
    assert len(rows) == 1


def test_navigation_feedback_keeps_frame_goal_and_matched_odom(evidence):
    evidence.remember_odom({'stamp_sec': 12, 'stamp_nanosec': 0, 'frame_id': 'odom',
                            'child_frame_id': 'base_link', 'x': 1., 'y': 2., 'yaw_rad': .3})
    header = NS(frame_id='map', stamp=NS(sec=12, nanosec=20000000))
    pose = NS(header=header, pose=NS(position=NS(x=3., y=4., z=0.),
                                      orientation=NS(x=0., y=0., z=0., w=1.)))
    feedback = NS(current_pose=pose, distance_remaining=2., number_of_recoveries=0)
    evidence.feedback('navigate_to_pose', NS(goal_id=NS(uuid=[1]*16), feedback=feedback))
    row = evidence.latest_feedback['navigate_to_pose']
    assert row['goal_id'] == '01'*16
    assert row['map_pose']['frame_id'] == 'map'
    assert row['odometry_match']['stamp_delta_sec'] == pytest.approx(-.02)
    assert row['odometry_match']['usable_for_approximate_alignment']
    assert row['map_pose']['stamp_sec'] == 12


def test_stale_or_wrong_frame_odom_is_not_claimed_aligned(evidence):
    evidence.remember_odom({'stamp_sec': 1, 'stamp_nanosec': 0, 'frame_id': 'map',
                            'child_frame_id': 'wrong', 'x': 0, 'y': 0, 'yaw_rad': 0})
    assert not evidence.match_odom(2.)['usable_for_approximate_alignment']


def test_capture_coverage_distinguishes_publisher_from_messages(evidence):
    assert evidence.coverage()['collision_state']['status'] == 'graph_not_sampled'
    evidence.graph({'/collision_monitor_state': {'types': ['nav2_msgs/msg/CollisionMonitorState'],
                                                'publisher_count': 1}}, [])
    coverage = evidence.coverage()
    assert coverage['collision_state']['status'] == 'publisher_seen_no_message'
    assert coverage['mppi_candidate_rejection_reason']['status'] == 'not_collected'
    assert not coverage['full_optimizer_replay_available']


def test_nonfinite_feedback_is_rejected_as_evidence_not_valid_pose(evidence):
    evidence.feedback('follow_path', NS(goal_id=NS(uuid=[3]*16),
        feedback=NS(distance_to_goal=float('nan'), speed=.4)))
    assert 'follow_path' not in evidence.latest_feedback
    assert (evidence.directory/'action_feedback.jsonl').read_text() == ''
    event = json.loads((evidence.directory/'diagnostic_events.jsonl').read_text())
    assert event['kind'] == 'invalid_feedback'


def test_feedback_rate_limit_does_not_drop_new_goal(evidence):
    for goal in (1, 1, 2):
        evidence.feedback('follow_path', NS(goal_id=NS(uuid=[goal]*16),
                                           feedback=NS(distance_to_goal=3., speed=.4)))
    evidence.flush()
    rows = (evidence.directory/'action_feedback.jsonl').read_text().splitlines()
    assert len(rows) == 2


def test_log_source_stamp_and_context_are_preserved(evidence):
    msg = NS(name='controller_server', msg='Ordinary MPPI has no valid control', level=30,
             stamp=NS(sec=3, nanosec=4), file='optimizer.cpp', function='compute', line=9)
    evidence.log(msg, {'raw_command': {'linear_x': 0.0}})
    evidence.flush()
    row = json.loads((evidence.directory/'diagnostic_events.jsonl').read_text())
    assert row['source_stamp'] == {'sec': 3, 'nanosec': 4}
    assert row['source_file'] == 'optimizer.cpp'
    assert row['context']['raw_command']['linear_x'] == 0.0


def test_collision_info_transition_is_preserved_without_state_publisher(evidence):
    evidence.log(NS(name='collision_monitor', msg='Robot will slowdown', level=20,
        stamp=NS(sec=20, nanosec=4), file='collision_monitor_node.cpp', function='process', line=9), {})
    row = json.loads((evidence.directory/'diagnostic_events.jsonl').read_text())
    assert row['source_stamp'] == {'sec': 20, 'nanosec': 4}
    assert row['level'] == 20
    assert evidence.collision_snapshot() is None  # a log is not a state-topic sample


def test_process_counter_reader_handles_missing_process(module, tmp_path):
    row = module.controller_process_counters(123, proc_root=tmp_path)
    assert row['available'] is False
    assert row['pid'] == 123


def test_proc_counters_are_not_reported_as_compute_duration(module, tmp_path):
    p = tmp_path/'123'
    p.mkdir()
    # Fields 3..24; utime/stime are offsets 11/12 in this suffix.
    fields = ['0']*22
    fields[0] = 'S'
    fields[11:13] = ['17', '3']
    fields[19] = '4567'
    (p/'stat').write_text('123 (controller server) '+' '.join(fields))
    (p/'schedstat').write_text('100000000 3000000 19')
    row = module.controller_process_counters(123, proc_root=tmp_path)
    assert row['available']
    assert row['utime_ticks'] == 17
    assert row['main_thread_runqueue_wait_ns'] == 3000000
    assert row['mppi_compute_duration_ms'] is None

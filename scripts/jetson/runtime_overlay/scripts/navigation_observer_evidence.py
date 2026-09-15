"""Bounded recorder evidence. No ROS initialization, publishers or service clients."""
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def receipt():
    return {'captured_at': datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z'),
            'received_monotonic': time.monotonic()}


def controller_process_counters(pid, proc_root=Path('/proc')):
    """Process CPU and MAIN-thread runqueue counters, not MPPI compute latency."""
    row = {**receipt(), 'pid': pid, 'available': False, 'mppi_compute_duration_ms': None}
    try:
        proc = Path(proc_root)/str(pid)
        fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
        row.update(available=True, process_state=fields[0], utime_ticks=int(fields[11]),
                   stime_ticks=int(fields[12]), process_start_ticks=int(fields[19]),
                   clock_ticks_per_sec=os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else None)
        try:
            run, wait, slices = map(int, (proc/'schedstat').read_text().split()[:3])
            row.update(main_thread_runtime_ns=run, main_thread_runqueue_wait_ns=wait,
                       main_thread_timeslices=slices)
        except (OSError, ValueError) as exc:
            row['schedstat_error'] = str(exc)
    except (OSError, ValueError, IndexError) as exc:
        row['error'] = str(exc)
    return row


class NavigationEvidence:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.handles = {name: (self.directory/name).open('x', encoding='utf-8') for name in (
            'diagnostic_events.jsonl', 'action_feedback.jsonl', 'process_samples.jsonl',
            'diagnostic_graph.jsonl')}
        self.odom = deque(maxlen=400)
        self.latest_feedback = {}
        self.feedback_saved = {}
        self.feedback_counts = {}
        self.collision_states = {}
        self.graph_topics = {}
        self.graph_errors = []
        self.graph_sampled = False
        self.last_process_sample = -math.inf
        self.controller_pids = []
        snapshot = self.directory/'runtime_snapshot.json'
        if snapshot.exists():
            self.controller_pids = [p['pid'] for p in json.loads(snapshot.read_text()).get(
                'processes', {}).get('controller_server', [])]
        versions = {}
        for package in ('nav2_mppi_controller', 'nav2_collision_monitor', 'nav2_msgs'):
            try:
                versions[package] = ET.parse('/opt/ros/humble/share/'+package+'/package.xml').findtext('version')
            except (OSError, ET.ParseError):
                versions[package] = None
        self.versions = versions

    def write(self, name, value):
        self.handles[name].write(json.dumps(value, ensure_ascii=True, allow_nan=False)+'\n')
        self.handles[name].flush()

    def flush(self):
        for handle in self.handles.values():
            handle.flush()

    def close(self):
        for handle in self.handles.values():
            handle.close()

    def remember_odom(self, row):
        self.odom.append(dict(row))

    def match_odom(self, stamp):
        if not self.odom or stamp <= 0:
            return {'usable_for_approximate_alignment': False, 'reason': 'missing_pose_stamp_or_odometry'}
        value = min(self.odom, key=lambda o: abs(o['stamp_sec']+o['stamp_nanosec']/1e9-stamp))
        delta = value['stamp_sec']+value['stamp_nanosec']/1e9-stamp
        valid = abs(delta) <= .10 and value['frame_id'] == 'odom' and value['child_frame_id'] == 'base_link'
        return {'usable_for_approximate_alignment': valid, 'stamp_delta_sec': delta,
                'method': 'nearest_buffered_odom_not_exact_tf', 'odometry': value}

    def feedback(self, action, msg):
        goal = ''.join(f'{int(b):02x}' for b in msg.goal_id.uuid)
        now = time.monotonic()
        previous = self.latest_feedback.get(action)
        self.feedback_counts[action] = self.feedback_counts.get(action, 0)+1
        if previous and previous['goal_id'] == goal and now-self.feedback_saved[action] < .20:
            return  # At most 5 Hz per action; a different goal is always retained.
        row = {**receipt(), 'action': action, 'goal_id': goal}
        f = msg.feedback
        if action == 'navigate_to_pose':
            p, header = f.current_pose.pose, f.current_pose.header
            q = p.orientation
            row['map_pose'] = {'frame_id': header.frame_id, 'stamp_sec': int(header.stamp.sec),
                               'stamp_nanosec': int(header.stamp.nanosec), 'x': float(p.position.x),
                               'y': float(p.position.y), 'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
            row.update(distance_remaining=float(f.distance_remaining), number_of_recoveries=int(f.number_of_recoveries))
            row['odometry_match'] = self.match_odom(header.stamp.sec+header.stamp.nanosec/1e9)
            if header.frame_id != 'map':
                row['odometry_match']['usable_for_approximate_alignment'] = False
                row['odometry_match']['reason'] = 'navigation_pose_not_map_frame'
        else:
            row.update(distance_to_goal=float(f.distance_to_goal), speed=float(f.speed))
        # Non-finite feedback is evidence of invalid input, never silently a valid pose.
        try:
            self.write('action_feedback.jsonl', row)
        except ValueError:
            self.write('diagnostic_events.jsonl', {**receipt(), 'kind': 'invalid_feedback', 'action': action, 'goal_id': goal})
            return
        self.latest_feedback[action] = row
        self.feedback_saved[action] = now

    def collision(self, topic, msg):
        old = self.collision_states.get(topic)
        code = int(msg.action_type)
        state = {**receipt(), 'topic': topic, 'action_type': code,
                 'action_name': {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH'}.get(code, f'UNKNOWN_{code}'),
                 'polygon_name': str(msg.polygon_name), 'message_count': (old or {}).get('message_count', 0)+1}
        self.collision_states[topic] = state
        if old is None or (old['action_type'], old['polygon_name']) != (code, state['polygon_name']):
            self.write('diagnostic_events.jsonl', {'kind': 'collision_state_changed', **state})

    def collision_snapshot(self):
        if not self.collision_states:
            return None
        latest = max(self.collision_states.values(), key=lambda s: s['received_monotonic'])
        return {**latest, 'receive_age_sec': time.monotonic()-latest['received_monotonic'],
                'age_semantics': 'last_event_age_not_heartbeat_validity',
                'observed_state_topics': len(self.collision_states)}

    def graph(self, topics, errors):
        self.graph_sampled = True
        self.graph_topics = topics
        self.graph_errors = errors
        self.write('diagnostic_graph.jsonl', {**receipt(), 'topics': topics, 'errors': errors})
        self.save_coverage()

    def coverage(self):
        collision_pubs = sum(v.get('publisher_count', 0) or 0 for v in self.graph_topics.values()
                             if 'nav2_msgs/msg/CollisionMonitorState' in v.get('types', []))
        status = ('messages_received' if self.collision_states else
                  'publisher_seen_no_message' if collision_pubs else
                  'graph_not_sampled' if not self.graph_sampled else
                  'graph_query_inconclusive' if self.graph_errors else 'no_publisher_observed')
        return {'recorder_version': 3, 'package_versions': self.versions,
                'collision_state': {'status': status, 'publisher_count_observed': collision_pubs,
                                    'received_topics': list(self.collision_states)},
                'action_feedback_counts': self.feedback_counts,
                'map_pose_feedback_received': 'navigate_to_pose' in self.latest_feedback,
                'odometry_messages_buffered': len(self.odom),
                'mppi_candidate_rejection_reason': {'status': 'not_collected',
                    'reason': 'No enabled and validated critic/rejection interface is consumed; script never enables visualization or changes controller logging.'},
                'mppi_compute_duration': {'status': 'not_collected',
                    'reason': 'Command receive gaps and /proc counters are not optimizer computation timings.'},
                'full_optimizer_replay_available': False,
                'diagnostic_gaps_are_not_navigation_failures': True}

    def save_coverage(self):
        (self.directory/'evidence_availability.json').write_text(json.dumps(self.coverage(), indent=2)+'\n')

    def process_sample(self):
        now = time.monotonic()
        if now-self.last_process_sample < 1.:
            return
        self.last_process_sample = now
        for pid in self.controller_pids[:2]:
            self.write('process_samples.jsonl', controller_process_counters(pid))

    def log(self, msg, context):
        if not any(name in msg.name for name in ('controller_server', 'collision_monitor', 'bt_navigator')):
            return
        # Humble reports collision action transitions at INFO even when it has
        # no CollisionMonitorState publisher. Preserve those source stamps too.
        if int(msg.level) < 30 and 'collision_monitor' not in msg.name and not any(
                s in msg.msg.lower() for s in ('recovered', 'repair', 'wait_clear')):
            return
        self.write('diagnostic_events.jsonl', {**receipt(), 'kind': 'navigation_log',
            'node': msg.name, 'level': int(msg.level), 'message': msg.msg,
            'source_stamp': {'sec': msg.stamp.sec, 'nanosec': msg.stamp.nanosec},
            'source_file': msg.file, 'source_function': msg.function, 'source_line': int(msg.line),
            'context': context})

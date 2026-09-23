"""Offline public-interface regressions; no production ROS/domain required."""
import importlib.util
import json
from pathlib import Path
import unittest
import tempfile
import os
from types import SimpleNamespace


def module():
    spec = importlib.util.spec_from_file_location('capture', HERE / 'nav_jerk_capture.py')
    obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj


def sample(topic, seq, seconds, vx, vy=0., wz=0.):
    return {'topic': topic, 'seq': seq, 'monotonic_ns': int(seconds*1e9),
            'wall_ns': int(seconds*1e9), 'ros_ns': int(seconds*1e9), 'source_headers': [],
            'type': 'geometry_msgs/msg/Twist',
            'data': {'linear': {'x': vx, 'y': vy, 'z': 0.}, 'angular': {'x': 0., 'y': 0., 'z': wz}}}

HERE = Path(__file__).resolve().parent


class DiscoveryTests(unittest.TestCase):
    def test_graph_drives_remapped_chain_and_deduplicates_shared_roles(self):
        spec = importlib.util.spec_from_file_location('capture', HERE / 'nav_jerk_capture.py')
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        cfg = m.load_config(HERE / 'nav_jerk_capture.yaml')
        def endpoint(node):
            return {'node': '/' + node, 'gid': node, 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}
        graph = {'/changed/raw': {'types': ['geometry_msgs/msg/TwistStamped'],
                 'pubs': [endpoint('controller_server')], 'subs': [endpoint('velocity_smoother')]},
                 '/changed/smoothed': {'types': ['geometry_msgs/msg/Twist'],
                 'pubs': [endpoint('velocity_smoother'), endpoint('behavior_server')],
                 'subs': [endpoint('collision_monitor')]}}
        result = m.build_topic_map(graph, {}, cfg)
        self.assertEqual(result['roles']['controller_output'], ['/changed/raw'])
        self.assertEqual(result['roles']['smoother_input'], ['/changed/raw'])
        self.assertEqual(len(result['topics']), 2)
        self.assertTrue(result['topics']['/changed/smoothed']['multi_publisher'])
        self.assertTrue(result['missing_roles'])
        self.assertFalse(result['internal_mppi_output_visible'])


class StorageTests(unittest.TestCase):
    def test_bag_qos_keeps_observed_unknown_depth_and_durations(self):
        m = module()
        profile = m.bag_qos({'history': 'QoSHistoryPolicy.KEEP_ALL', 'depth': 0,
            'reliability': 'QoSReliabilityPolicy.BEST_EFFORT', 'durability': 'QoSDurabilityPolicy.VOLATILE',
            'deadline_ns': 1234567890, 'lifespan_ns': 987654321,
            'liveliness': 'QoSLivelinessPolicy.MANUAL_BY_TOPIC', 'liveliness_lease_duration_ns': 2000000001})
        self.assertEqual(profile['depth'], 0)
        self.assertEqual(profile['history'], 2)
        self.assertEqual(profile['deadline'], {'sec': 1, 'nsec': 234567890})
        self.assertEqual(profile['liveliness'], 3)

    def test_queue_has_byte_and_count_bounds_and_records_loss(self):
        m = module()
        q = m.BoundedInbox(2, 8)
        self.assertTrue(q.put({'seq': 1, 'topic': '/a', 'raw': b'1234'}))
        self.assertTrue(q.put({'seq': 2, 'topic': '/a', 'raw': b'5678'}))
        self.assertFalse(q.put({'seq': 3, 'topic': '/a', 'raw': b'9'}))
        self.assertEqual(q.get()['seq'], 1)
        self.assertEqual(q.stats()['dropped']['/a']['first_seq'], 3)
        self.assertEqual(q.stats()['max_bytes'], 8)

    @unittest.skipIf(os.name == 'nt', 'POSIX inode rotation, run in Linux isolated tests')
    def test_log_rename_rotation_keeps_old_tail_and_new_file(self):
        m = module()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'nav.log'
            source.write_bytes(b'initial\n')
            rows = []
            tail = m.RotatingLog(source, 1024)
            rows.extend(tail.read())
            with source.open('ab') as f: f.write(b'old_tail\n')
            source.rename(root / 'nav.log.1')
            source.write_bytes(b'new_file\n')
            rows.extend(tail.read())
            tail.close()
            text = ''.join(row.get('text', '') for row in rows)
            self.assertIn('old_tail', text)
            self.assertIn('new_file', text)
            self.assertTrue(any(row.get('event') == 'rotated' for row in rows))

    @unittest.skipIf(os.name == 'nt', 'POSIX inode rotation, run in Linux isolated tests')
    def test_new_rotated_file_is_not_tail_truncated(self):
        m = module()
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'nav.log'
            p.write_bytes(b'old')
            tail = m.RotatingLog(p, 3)
            tail.read()
            p.rename(p.with_suffix('.old'))
            p.write_bytes(b'new-first-line\nnew-last-line\n')
            text = ''.join(r.get('text', '') for r in tail.read())
            tail.close()
            self.assertIn('new-first-line', text)


class ReportTests(unittest.TestCase):
    def test_event_reports_observed_motion_change_without_claiming_causality(self):
        m = module()
        cfg = m.load_config(HERE / 'nav_jerk_capture.yaml')
        rows = [sample('/odom', 1, .9, .3), sample('/odom', 2, 1.1, 0.)]
        for r in rows: r['type'] = 'nav_msgs/msg/Odometry'
        event = {'kind': 'manual_mark', 'monotonic_ns': 1000000000}
        result = m.event_window(event, {}, rows, {'topics': {}, 'edges': [], 'observability_gaps': []}, cfg)
        motion = result['actual_motion_response']['by_topic']['/odom']
        self.assertEqual(motion['status'], 'observed_velocity_change')
        self.assertEqual(motion['baseline_ref'], 1)
        self.assertEqual(motion['first_changed_ref'], 2)

    def test_graph_does_not_verify_disconnected_ports(self):
        m = module()
        cfg = m.load_config(HERE / 'nav_jerk_capture.yaml')
        def endpoint(name):
            return {'node': '/' + name, 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}
        graph = {
            '/raw': {'types': ['geometry_msgs/msg/Twist'], 'pubs': [endpoint('controller_server')], 'subs': []},
            '/wrong_input': {'types': ['geometry_msgs/msg/Twist'], 'pubs': [], 'subs': [endpoint('velocity_smoother')]}}
        result = m.build_topic_map(graph, {}, cfg)
        edge = result['edges'][0]
        self.assertFalse(edge['verified'])
        self.assertEqual(edge['missing_reason'], 'ports_do_not_share_topic')

    def test_pending_mark_before_ctrl_c_survives_offline_report(self):
        m = module()
        cfg = m.load_config(HERE / 'nav_jerk_capture.yaml')
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            mark_host = os.uname().nodename if hasattr(os, 'uname') else os.environ.get('COMPUTERNAME')
            m.save(root/'session.json', {'cfg': cfg, 'host': mark_host, 'started_monotonic_ns': 0})
            m.save(root/'topic_map.json', {'topics': {}, 'roles': {}, 'edges': [], 'observability_gaps': []})
            mark = m.create_mark(root, 'last instant')
            out = root/'report'
            m.report(SimpleNamespace(directory=str(root), output_dir=str(out), config_explicit=False), cfg)
            events = list(m.json_rows(out/'events.jsonl'))
            self.assertEqual(len(events), 1)
            self.assertEqual(events[0]['id'], mark['id'])
            self.assertTrue(events[0]['not_ingested_before_stop'])

    def test_zero_is_a_message_and_silence_is_not_a_zero(self):
        m = module()
        rows = [sample('/a', i+1, t, x) for i, (t, x) in enumerate([(1, .3), (1.1, 0), (1.2, .3), (2, .3)])]
        events = m.command_candidates(rows, m.load_config(HERE / 'nav_jerk_capture.yaml')['thresholds'], active_windows=[(0, 3)])
        kinds = [e['kind'] for e in events]
        self.assertIn('zero_command', kinds)
        self.assertIn('short_stop_restart', kinds)
        self.assertIn('command_gap', kinds)
        gap = next(e for e in events if e['kind'] == 'command_gap')
        self.assertEqual(gap['previous_ref'], 3)
        self.assertNotEqual(gap['kind'], 'zero_command')

    def test_smooth_speed_noise_and_idle_silence_not_faults(self):
        m = module()
        rows = [sample('/a', i+1, i*.1, .3+i*.001, wz=(-1)**i*.001) for i in range(30)]
        thresholds = m.load_config(HERE / 'nav_jerk_capture.yaml')['thresholds']
        self.assertEqual(m.command_candidates(rows, thresholds), [])
        rows.append(sample('/a', 31, 10, .33))
        self.assertFalse(any(e['kind'] == 'command_gap' for e in m.command_candidates(rows, thresholds)))

    def test_duplicate_source_stamp_has_no_derivative(self):
        m = module()
        a = sample('/a', 1, 1, .1)
        b = sample('/a', 2, 1.1, .4)
        a['source_headers'] = b['source_headers'] = [{'stamp_ns': 100, 'frame': 'base_link'}]
        self.assertIsNone(m.derivative(a, b, .35)['acceleration'])

    def test_twist_and_stamped_twist_keep_signed_axes(self):
        m = module()
        a = {'linear': {'x': -.2, 'y': .1}, 'angular': {'z': -.3}}
        self.assertEqual(m.velocity(a), (-.2, .1, -.3))
        self.assertEqual(m.velocity({'header': {}, 'twist': a}), (-.2, .1, -.3))

    def test_qos_mixed_publishers_uses_compatible_subscription_not_producer_mutation(self):
        m = module()
        pubs = [{'reliability': 'RELIABLE', 'durability': 'TRANSIENT_LOCAL'},
                {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE'}]
        choice = m.qos_choice(pubs)
        self.assertEqual(choice['reliability'], 'BEST_EFFORT')
        self.assertEqual(choice['durability'], 'VOLATILE')
        self.assertTrue(choice['mixed_durability_retained_gap'])

    def test_disk_reserve_stops_before_exhaustion(self):
        m = module()
        cfg = m.load_config(HERE / 'nav_jerk_capture.yaml')
        self.assertEqual(m.budget_reason(0, 1, cfg), 'disk_free')
        self.assertEqual(m.budget_reason(cfg['disk_budget_mb']*1024**2, 10**12, cfg), 'disk_budget')

    def test_report_clock_jump_missing_full_grid_and_midchain_boundary(self):
        m = module()
        cfg = m.load_config(HERE/'nav_jerk_capture.yaml')
        cfg['window_sec'] = .3
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            topic_map = {'topics': {t: {'roles': [role], 'multi_publisher': False, 'cadence': 'command'}
                         for t, role in [('/a', 'smoother_output'), ('/b', 'collision_output')]},
                         'roles': {'smoother_output': ['/a'], 'collision_output': ['/b']},
                         'edges': [{'from_topics': ['/a'], 'to_topics': ['/b']}], 'observability_gaps': []}
            m.save(root/'session.json', {'cfg': cfg, 'started_monotonic_ns': 0})
            m.save(root/'topic_map.json', topic_map)
            rows = [sample('/a', 1, 1, .3), sample('/b', 2, 1.01, .3), sample('/a', 3, 1.1, .3), sample('/b', 4, 1.11, 0)]
            rows[-1]['wall_ns'] += 1000000000
            rows[-1]['ros_ns'] += 1000000000
            with (root/'index.jsonl').open('w') as f:
                for row in rows: f.write(m.dumps(row)+'\n')
            out = root/'report'
            m.report(SimpleNamespace(directory=str(root), output_dir=str(out), config_explicit=False), cfg)
            events = list(m.json_rows(out/'events.jsonl'))
            self.assertTrue(any(e['kind'] == 'wall_clock_jump' for e in events))
            self.assertTrue(any(e['kind'] == 'ros_clock_jump' for e in events))
            zero = next(e for e in events if e['kind'] == 'zero_command')
            self.assertEqual(zero['first_observed_boundary']['observed_topic'], '/b')
            self.assertEqual(zero['first_observed_boundary']['adjacent_received_command_comparisons'][0]['upstream_ref'], 3)
            quality = json.loads((out/'quality.json').read_text())
            self.assertFalse(quality['costmap']['complete_reconstruction_proven'])
            self.assertIsNone(quality['topics']['/a']['comparable_source_age_sec'])

    def test_mark_writes_file_without_importing_ros(self):
        m = module()
        with tempfile.TemporaryDirectory() as d:
            m.save(Path(d)/'session.json', {'pid': 123})
            mark = m.create_mark(d, 'felt jerk')
            self.assertEqual(json.loads((Path(d)/'marks'/(mark['id']+'.json')).read_text())['label'], 'felt jerk')

    def test_svg_breaks_at_gap_without_interpolation(self):
        m = module()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'plot.svg'
            m.velocity_svg(path, {'/a': [sample('/a', 1, 1, .3), sample('/a', 2, 2, .4)]}, [], [], .35)
            self.assertEqual(path.read_text(encoding='utf-8').count('<polyline'), 6)  # two disjoint segments per axis


if __name__ == '__main__':
    unittest.main()

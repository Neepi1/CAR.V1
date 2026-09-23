"""No ROS, HTTP or production process access: temporary fixtures only."""
import json
import tempfile
from pathlib import Path
import unittest

from nav_event_evidence import (resolve_manifest, DiagnosticRequest, evidence_quality,
                                parse_process, bounded_command, EvidenceSession)


class EvidenceTests(unittest.TestCase):
    def test_remaps_and_live_parameters_resolve_real_chain_and_deduplicate(self):
        processes = [
            parse_process(1, ['/opt/controller_server', '--ros-args', '-r',
                             'cmd_vel:=/different/raw']),
            parse_process(2, ['/opt/velocity_smoother', '--ros-args', '-r',
                             'cmd_vel:=/different/raw', '-r',
                             'cmd_vel_smoothed:=/different/smoothed']),
            parse_process(3, ['/opt/collision_monitor']),
            parse_process(4, ['/opt/robot_safety_node']),
            parse_process(5, ['/opt/ranger_base_node', '-p',
                             'odom_topic_name:=/different/wheel'])]
        params = {
            'controller_server': {'odom_topic': '/different/odom'},
            'collision_monitor': {'cmd_vel_in_topic': '/different/smoothed',
                'cmd_vel_out_topic': '/different/checked', 'observation_sources': ['scan'],
                'scan': {'topic': '/different/scan'}},
            'robot_safety': {'cmd_vel_in_topic': '/different/checked',
                            'cmd_vel_out_topic': '/cmd_vel'}}
        manifest = resolve_manifest(processes, params)
        names = [x['name'] for x in manifest['topics']]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn('/different/wheel', names)
        self.assertNotIn('/scan', names)
        self.assertIn('/different/scan', names)
        self.assertEqual(manifest['chain']['smoother_input'], '/different/raw')
        self.assertEqual(manifest['chain']['collision_output'], '/different/checked')
        self.assertFalse(manifest['chain_gaps'])

    def test_missing_process_not_silently_assumed_current_mapping(self):
        result = resolve_manifest([], {})
        self.assertIn('controller_server: process missing or nonunique', result['chain_gaps'])
        self.assertIsNone(result['chain']['controller_output'])

    def test_node_and_namespace_remapping(self):
        proc = parse_process(1, ['/opt/controller_server', '--ros-args',
                              '-r', '__ns:=/robot', '-r', '__node:=control',
                              '-r', 'cmd_vel:=raw'])
        result = resolve_manifest([proc], {})
        self.assertEqual(result['chain']['controller_output'], '/robot/raw')

    def test_request_does_not_overwrite_another_session_or_remove_it(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            request = root/'request'
            first = DiagnosticRequest(request, root, 'one', 10, 2)
            first.start()
            self.assertEqual(json.loads(request.read_text())['session'], 'one')
            with self.assertRaises(FileExistsError):
                DiagnosticRequest(request, root, 'two', 10, 2).start()
            request.write_text(json.dumps({'session': 'another'}))
            first.close()
            self.assertTrue(request.exists())

    def test_readiness_requires_messages_not_only_files_and_no_fake_motion(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            (root/'motion').mkdir()
            (root/'motion'/'quality.json').write_text(json.dumps({'ready': True,
                'topics': {'/wheel/odom': {'messages': 0}}}))
            manifest = {'topics': [{'name': '/wheel/odom', 'critical': True,
                                   'cadence': 'continuous'}], 'chain_gaps': []}
            q = evidence_quality(root, manifest, require_mppi=True)
            self.assertFalse(q['ready'])
            self.assertIn('/wheel/odom: no recorded message', q['gaps'])
            self.assertIn('MPPI: no internal frame yet (idle or diagnostics unavailable)', q['gaps'])

    def test_bounded_subprocess_timeout_does_not_escape(self):
        import sys
        value = bounded_command([sys.executable, '-c', 'import time; time.sleep(3)'], .02)
        self.assertEqual(value['status'], 'timeout')

    def test_diagnostic_request_is_published_complete_and_removed_by_owner(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'request'
            request = DiagnosticRequest(path, Path(root), 'ascii_1', 10, 2)
            request.start()
            self.assertEqual(json.loads(path.read_text())['max_bytes'], 2*1024*1024)
            self.assertEqual(list(Path(root).glob('.navlite_request_*')), [])
            request.close()
            request.close()
            self.assertFalse(path.exists())

    def test_frame_presence_does_not_hide_producer_write_failure(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            (root/'motion').mkdir()
            (root/'motion'/'quality.json').write_text(json.dumps({'ready': True,'topics': {}}))
            snapshot = root/'mppi_1_1'
            snapshot.mkdir()
            (snapshot/'frames.jsonl').write_text('{}\n')
            (snapshot/'status.json').write_text(json.dumps({'complete':False,
                'reason':'disk_budget','frames':1,'write_errors':1}))
            quality = evidence_quality(root, {'topics': [],'chain_gaps': []})
            self.assertFalse(quality['ready'])
            self.assertIn('MPPI: dropped frame or write failure', quality['gaps'])

    def test_private_native_child_stopped_without_group_or_robot_signal(self):
        from unittest.mock import Mock, patch
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as root:
            session = EvidenceSession(Path(root), SimpleNamespace())
            child = Mock()
            child.poll.return_value = None
            session.process = child
            session.manifest = {'topics': [],'chain_gaps': []}
            with patch.object(session, 'poll', return_value={'ready':False}):
                session.close()
            import signal
            child.send_signal.assert_called_once_with(signal.SIGINT)
            child.wait.assert_called_once_with(timeout=8)
            child.kill.assert_not_called()

    def test_tf_remap_is_applied_without_duplicate_subscription(self):
        proc = parse_process(1, ['/opt/controller_server','-r','/tf:=/custom_tf'])
        manifest = resolve_manifest([proc], {})
        topics = [row['name'] for row in manifest['topics']]
        self.assertIn('/custom_tf',topics)
        self.assertNotIn('/tf',topics)


if __name__ == '__main__':
    unittest.main()

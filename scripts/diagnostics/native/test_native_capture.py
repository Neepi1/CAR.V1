#!/usr/bin/env python3
"""CLI integration tests. Refuses non-isolated networking/IPC; no robot programs."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest

ARGS = None


def require_isolation():
    devices = [line.split(':')[0].strip() for line in Path('/proc/net/dev').read_text().splitlines()[2:]]
    if os.environ.get('NAVLITE_ISOLATED') != '1' or os.environ.get('ROS_DOMAIN_ID') != '181' or devices != ['lo']:
        raise RuntimeError('test requires private net/IPC/mount, private /dev/shm, domain 181, loopback only')


class CaptureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        require_isolation()
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, PolygonStamped
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
        from tf2_msgs.msg import TFMessage
        cls.rclpy = rclpy
        rclpy.init()
        cls.node = rclpy.create_node('navlite_fixture', enable_rosout=False, start_parameter_services=False)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.started = time.monotonic()
        cls.tick = 0
        cls.second_raw = None
        cls.repeat_source = False
        cls.pause_wheel = False
        cls.pub = {}
        cls.types = {}
        ordinary = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, typ in [('/raw', Twist), ('/smooth', TwistStamped), ('/collision', Twist), ('/final', Twist),
                          ('/odom', Odometry), ('/wheel', Odometry), ('/scan', LaserScan),
                          ('/tf', TFMessage), ('/grid', OccupancyGrid), ('/path', RosPath),
                          ('/footprint', PolygonStamped)]:
            cls.pub[name] = cls.node.create_publisher(typ, name, ordinary)
            cls.types[name] = typ
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.pub['/tf_static'] = cls.node.create_publisher(TFMessage, '/tf_static', retained)
        cls.types['/tf_static'] = TFMessage
        fixed = TransformStamped()
        fixed.header.frame_id = 'base_link'
        fixed.child_frame_id = 'lidar'
        fixed.transform.rotation.w = 1.
        cls.pub['/tf_static'].publish(TFMessage(transforms=[fixed]))
        cls.grid = OccupancyGrid()
        cls.grid.header.frame_id = 'odom'
        cls.grid.info.width = cls.grid.info.height = 200
        cls.grid.info.resolution = .05
        cls.grid.data = [0] * 40000
        cls.scan = LaserScan()
        cls.scan.header.frame_id = 'lidar'
        cls.scan.angle_min, cls.scan.angle_max = -3.14, 3.14
        cls.scan.angle_increment = 6.28 / 1440
        cls.scan.ranges = [2.] * 1440

        def publish():
            cls.tick += 1
            n = cls.tick
            stamp = cls.node.get_clock().now().to_msg()
            if cls.repeat_source:
                stamp.sec, stamp.nanosec = 123, 456
            if n % 2 == 0:
                for name in ('/raw', '/smooth', '/collision', '/final'):
                    msg = cls.types[name]()
                    velocity = msg.twist if name == '/smooth' else msg
                    # A repeated intermediate-layer zero, not a recorder inference.
                    velocity.linear.x = 0. if name in ('/collision', '/final') and n % 500 < 100 else .3
                    velocity.angular.z = -.2
                    if name == '/smooth':
                        msg.header.stamp = stamp
                        msg.header.frame_id = 'base_link'
                    cls.pub[name].publish(msg)
                    if name == '/raw' and cls.second_raw is not None:
                        cls.second_raw.publish(msg)
                for name in ('/odom', '/wheel'):
                    if name == '/wheel' and cls.pause_wheel:
                        continue
                    msg = Odometry()
                    msg.header.stamp = stamp
                    msg.header.frame_id = 'odom'
                    msg.child_frame_id = 'base_link'
                    msg.twist.twist.linear.x = .29
                    cls.pub[name].publish(msg)
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = 'odom'
                transform.child_frame_id = 'base_link'
                transform.transform.rotation.w = 1.
                cls.pub['/tf'].publish(TFMessage(transforms=[transform]))
            if n % 7 == 0:
                cls.scan.header.stamp = stamp
                cls.pub['/scan'].publish(cls.scan)
            if n % 12 == 0:
                cls.grid.header.stamp = stamp
                cls.pub['/grid'].publish(cls.grid)
                path = RosPath()
                path.header.stamp = stamp
                path.header.frame_id = 'map'
                cls.pub['/path'].publish(path)
                footprint = PolygonStamped()
                footprint.header.stamp = stamp
                footprint.header.frame_id = 'odom'
                cls.pub['/footprint'].publish(footprint)
        cls.timer = cls.node.create_timer(.01, publish)
        cls.worker = threading.Thread(target=cls.executor.spin)
        cls.worker.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.worker.join()
        cls.node.destroy_node()
        cls.rclpy.shutdown()

    def capture(self, name, duration=4, extra=(), alter=None, interrupt=False):
        directory = Path(ARGS.output) / name
        directory.mkdir()
        topics = []
        for topic, typ in self.types.items():
            package = typ.__module__.split('.')[0]
            typename = package + '/msg/' + typ.__name__
            topics.append({'name': topic, 'role': topic.strip('/'), 'expected_type': typename,
                           'critical': True, 'cadence': 'event' if topic in ('/tf_static', '/path', '/footprint') else 'continuous'})
        manifest = {'schema': 1, 'topics': topics}
        if alter:
            alter(manifest)
        config = directory / 'manifest.json'
        config.write_text(json.dumps(manifest))
        output = directory / 'capture'
        log = (directory / 'process.log').open('w')
        cmd = [ARGS.binary, '--manifest', str(config), '--output', str(output), '--duration', str(duration),
               '--discovery-sec', '1', '--min-free-mb', '0', '--disk-budget-mb', '128', '--queue-mb', '8', *extra]
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        if interrupt:
            # Drive SIGINT during real recording, not during nondeterministic DDS cold discovery.
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                quality_path = output/'quality.json'
                if quality_path.exists() and json.loads(quality_path.read_text()).get('written', 0) > 20:
                    break
                time.sleep(.05)
            proc.send_signal(signal.SIGINT)
        code = proc.wait(timeout=duration+20)
        log.close()
        self.assertIn(code, (0, 3), (code, (directory/'process.log').read_text()))
        quality = json.loads((output/'quality.json').read_text())
        return output, quality

    def test_01_original_messages_and_correlated_receive_index(self):
        output, quality = self.capture('normal', duration=ARGS.perf_seconds)
        self.assertFalse(quality['incomplete'], quality)
        rows = list(csv.DictReader((output/'receive_index.csv').open()))
        self.assertGreater(len(rows), 300)
        self.assertEqual(len({r['bag_timestamp_ns'] for r in rows}), len(rows))
        for r in rows:
            self.assertTrue(r['publisher_gid'])
            self.assertGreater(int(r['recv_monotonic_ns']), 0)
        # Every middle-recording marker +/-1s must include scan + both odoms + TF.
        start = min(int(r['recv_monotonic_ns']) for r in rows)
        for center in (start+3_000_000_000, start+5_000_000_000):
            for topic in ('/scan', '/odom', '/wheel', '/tf', '/raw', '/smooth', '/collision', '/final', '/grid'):
                window = [r for r in rows if r['topic'] == topic and abs(int(r['recv_monotonic_ns'])-center) < 1_000_000_000]
                self.assertGreater(len(window), 3, topic)
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(output/'bag'), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        by_key = {(r['topic'], int(r['bag_timestamp_ns'])): r for r in rows}
        seen = set()
        signs = set()
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            self.assertIn((topic, stamp), by_key)
            self.assertEqual(len(raw), int(by_key[(topic, stamp)]['serialized_bytes']))
            seen.add((topic, stamp))
            if topic in ('/raw', '/smooth', '/collision'):
                msg = deserialize_message(raw, self.types[topic])
                velocity = msg.twist if topic == '/smooth' else msg
                self.assertEqual(velocity.angular.z, -.2)
                if topic == '/collision': signs.add(velocity.linear.x)
                if topic == '/smooth': self.assertEqual(msg.header.frame_id, 'base_link')
            elif topic == '/scan':
                msg = deserialize_message(raw, self.types[topic])
                self.assertEqual(len(msg.ranges), 1440)
        self.assertEqual(len(seen), len(rows))
        self.assertEqual(signs, {0., .3})
        self.assertGreater(quality['topics']['/tf_static']['written'], 0)
        print('NATIVE_PERF', json.dumps(quality['performance']), flush=True)

    def test_02_missing_topic_is_visible_during_and_after_capture(self):
        def alter(manifest):
            manifest['topics'].append({'name': '/not_published', 'role': 'missing_wheel',
                                       'expected_type': 'nav_msgs/msg/Odometry', 'critical': True, 'cadence': 'continuous'})
        output, quality = self.capture('missing', duration=3, alter=alter)
        self.assertFalse(quality['ready'])
        self.assertIn('no_messages:/not_published', quality['incomplete_reasons'])
        self.assertEqual(quality['topics']['/not_published']['messages'], 0)
        self.assertGreater(quality['topics']['/wheel']['messages'], 30)
        self.assertTrue((output/'bag'/'metadata.yaml').exists())

    def test_03_incompatible_qos_not_reported_as_success(self):
        def alter(manifest):
            manifest['topics'] = [t for t in manifest['topics'] if t['name'] == '/scan']
            manifest['topics'][0]['qos'] = {'reliability': 'reliable', 'durability': 'volatile', 'depth': 10}
        _, quality = self.capture('qos', duration=3, alter=alter)
        self.assertTrue(quality['incomplete'])
        self.assertEqual(quality['topics']['/scan']['messages'], 0)
        self.assertGreater(quality['topics']['/scan']['incompatible_qos_events'], 0)

    def test_04_ctrl_c_closes_bag_and_keeps_partial_data(self):
        output, quality = self.capture('interrupt', duration=10, interrupt=True)
        self.assertEqual(quality['reason'], 'signal_2')
        self.assertTrue(quality['incomplete'])
        self.assertGreater(quality['written'], 20)
        self.assertTrue((output/'bag'/'metadata.yaml').exists())

    def test_05_disk_budget_refuses_before_unbounded_capture(self):
        _, quality = self.capture('budget', extra=('--disk-budget-mb', '1'))
        self.assertEqual(quality['reason'], 'disk_budget')
        self.assertTrue(quality['incomplete'])

    def test_06_oversized_message_queue_drop_is_evidence(self):
        _, quality = self.capture('queue', duration=3, extra=('--queue-mb', '.001'))
        self.assertGreater(quality['queue_drops'], 0)
        self.assertGreater(quality['topics']['/grid']['dropped'], 0)
        self.assertTrue(quality['incomplete'])
        self.assertGreater(quality['first_drop_seq'], 0)

    def test_07_multi_publisher_gid_and_repeated_header_stamps(self):
        from geometry_msgs.msg import Twist
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        cls = type(self)
        cls.second_raw = cls.node.create_publisher(Twist, '/raw', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        cls.repeat_source = True
        try:
            output, quality = self.capture('multi', duration=3)
        finally:
            cls.second_raw = None
            cls.repeat_source = False
        rows = list(csv.DictReader((output/'receive_index.csv').open()))
        gids = {r['publisher_gid'] for r in rows if r['topic'] == '/raw'}
        self.assertEqual(len(gids), 2)
        self.assertNotIn('', gids)
        self.assertEqual(len({r['bag_timestamp_ns'] for r in rows}), len(rows))
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(output/'bag'), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        stamps = []
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == '/smooth':
                msg = deserialize_message(raw, self.types[topic])
                stamps.append((msg.header.stamp.sec, msg.header.stamp.nanosec))
        self.assertGreater(len(stamps), 20)
        self.assertEqual(set(stamps), {(123, 456)})
        self.assertEqual(quality['dropped'], 0)

    def test_08_gap_after_readiness_remains_in_quality_after_recovery(self):
        def interruption():
            time.sleep(2.5)
            type(self).pause_wheel = True
            time.sleep(3.1)
            type(self).pause_wheel = False
        worker = threading.Thread(target=interruption)
        worker.start()
        try:
            _, quality = self.capture('late_gap', duration=7)
        finally:
            worker.join()
            type(self).pause_wheel = False
        self.assertTrue(quality['topics']['/wheel']['ever_stale'])
        self.assertIn('continuous_gap:/wheel', quality['incomplete_reasons'])
        self.assertGreater(quality['topics']['/wheel']['max_gap_ns'], 2_000_000_000)

    def test_09_disk_low_preflight_is_not_reported_complete(self):
        _, quality = self.capture('disk_low', extra=('--min-free-mb', '10000000'))
        self.assertEqual(quality['reason'], 'disk_low')
        self.assertTrue(quality['incomplete'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--perf-seconds', type=float, default=8)
    ARGS, rest = parser.parse_known_args()
    Path(ARGS.output).mkdir(exist_ok=True)
    unittest.main(argv=[__file__, *rest], verbosity=2)

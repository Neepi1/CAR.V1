#!/usr/bin/env python3
"""Integration smoke: requires private net+IPC namespaces, never run in robot domain.

The wrapper checks namespace separation before this test starts ANY ROS context.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

HERE = Path(__file__).resolve().parent


def assert_isolated():
    for kind in ('net', 'ipc'):
        parent = os.getenv('NAV_JERK_PARENT_' + kind.upper())
        if not parent or os.readlink('/proc/self/ns/' + kind) == parent:
            raise RuntimeError('refusing ROS tests without independently isolated ' + kind + ' namespace')
    if os.getenv('ROS_DOMAIN_ID') != '187':
        raise RuntimeError('isolated test requires ROS_DOMAIN_ID=187')


class RosCaptureTest(unittest.TestCase):
    def test_index_conversion_preserves_ros_fields(self):
        from nav_jerk_capture import clean,ros_dict
        from rosidl_runtime_py.convert import message_to_ordereddict
        from geometry_msgs.msg import TwistStamped,PolygonStamped,Point32
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu,BatteryState
        from tf2_msgs.msg import TFMessage
        from action_msgs.msg import GoalStatusArray,GoalStatus
        from rcl_interfaces.msg import ParameterEvent,Parameter,ParameterValue
        odom=Odometry(); odom.twist.twist.linear.x=-.2; odom.pose.covariance[0]=.04
        imu=Imu(); imu.angular_velocity.z=-.3
        battery=BatteryState(current=float('nan'),cell_voltage=[1.,2.],location='robot')
        polygon=PolygonStamped(); polygon.polygon.points=[Point32(x=.47,y=-.36)]
        value=ParameterValue(type=5,byte_array_value=[b'\x01',b'\xff'])
        for msg in (TwistStamped(),odom,imu,battery,polygon,TFMessage(),
                    GoalStatusArray(status_list=[GoalStatus()]),
                    ParameterEvent(changed_parameters=[Parameter(name='bytes',value=value)])):
            self.assertEqual(clean(ros_dict(msg)),clean(message_to_ordereddict(msg)),type(msg))

    def test_busy_front_event_cannot_starve_motion_evidence(self):
        # Actual installed executor iterator logic; substitute ready events only.
        # No production ROS participant is used (the wrapper also isolates this test).
        from rclpy.executors import Executor
        from types import SimpleNamespace
        import collections
        from nav_jerk_capture import RosObserver
        stop = threading.Event()
        class ReadyEvents:
            wait_for_ready_callbacks = Executor.wait_for_ready_callbacks
            _cb_iter = _last_args = _last_kwargs = None
            def __init__(self): self.received = collections.Counter()
            def _wait_for_ready_callbacks(self, *args, **kwargs):
                for name in ('velocity', 'wheel_odom', 'scan', 'tf'):
                    yield name, None, None
            def spin_once(self, timeout_sec):
                name = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)[0]
                self.received[name] += 1
                if sum(self.received.values()) >= 100: stop.set()
        observer = RosObserver.__new__(RosObserver)
        observer.stop, observer.rclpy = stop, SimpleNamespace(ok=lambda: True)
        observer.executor = ReadyEvents()
        observer.spin(.04)
        self.assertGreater(observer.executor.received['wheel_odom'], 0)
        self.assertGreater(observer.executor.received['scan'], 0)
        self.assertGreater(observer.executor.received['tf'], 0)

    @classmethod
    def setUpClass(cls):
        assert_isolated()
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist, TwistStamped
        from nav_msgs.msg import Odometry, OccupancyGrid
        from sensor_msgs.msg import LaserScan, Imu, BatteryState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        cls.rclpy = rclpy
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        cls.done = threading.Event()
        cls.executor = SingleThreadedExecutor()
        cls.nodes, cls.pubs = [], []
        def node(name, parameters={}):
            n = Node(name, enable_rosout=False)
            for k, v in parameters.items(): n.declare_parameter(k, v)
            cls.nodes.append(n)
            cls.executor.add_node(n)
            return n
        cls.controller = node('controller_server', {'odom_topic': '/fixture/odom'})
        smoother = node('velocity_smoother')
        collision = node('collision_monitor', {'cmd_vel_in_topic': '/fixture/smooth', 'cmd_vel_out_topic': '/fixture/checked'})
        safety = node('robot_safety', {'cmd_vel_in_topic': '/fixture/checked', 'cmd_vel_out_topic': '/fixture/final', 'cmd_vel_mirror_topic': '/fixture/mirror', 'api_cmd_vel_in_topic': '/fixture/api', 'docking_cmd_vel_in_topic': '/fixture/dock', 'elevator_entry_cmd_vel_in_topic': '/fixture/smooth'})
        chassis = node('ranger_base_node')
        costmap = node('local_costmap')
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        cls.raw = cls.controller.create_publisher(TwistStamped, '/fixture/raw', be)
        cls.smooth = smoother.create_publisher(Twist, '/fixture/smooth', 10)
        cls.checked = collision.create_publisher(Twist, '/fixture/checked', 10)
        cls.final = safety.create_publisher(Twist, '/fixture/final', 10)
        cls.odom = chassis.create_publisher(Odometry, '/fixture/odom', 10)
        cls.imu = chassis.create_publisher(Imu, '/fixture/imu', be)
        cls.battery = chassis.create_publisher(BatteryState, '/fixture/battery', 10)
        cls.scan = chassis.create_publisher(LaserScan, '/fixture/scan', be)
        for n, topic, typ in [(smoother, '/fixture/raw', TwistStamped), (collision, '/fixture/smooth', Twist), (collision, '/fixture/scan', LaserScan), (safety, '/fixture/checked', Twist), (safety, '/fixture/api', Twist), (safety, '/fixture/dock', Twist), (safety, '/fixture/smooth', Twist), (chassis, '/fixture/final', Twist)]:
            n.create_subscription(typ, topic, lambda msg: None, be)
        # A second publisher is kept separate; no inferred producer identity.
        cls.second = node('behavior_server').create_publisher(Twist, '/fixture/smooth', 10)
        cls.cycle = 0
        def tick():
            cls.cycle += 1
            now = cls.controller.get_clock().now().to_msg()
            raw = TwistStamped(); raw.header.stamp = now; raw.header.frame_id = 'base_link'; raw.twist.linear.x = .3
            cls.raw.publish(raw)
            msg = Twist(); msg.linear.x = .3
            cls.smooth.publish(msg)
            if cls.cycle % 30 < 3: msg.linear.x = 0.0
            cls.checked.publish(msg); cls.final.publish(msg)
            odom = Odometry(); odom.header.stamp = now; odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'; odom.twist.twist.linear.x = .2
            cls.odom.publish(odom)
            imu = Imu(); imu.header.stamp = now; imu.header.frame_id = 'imu_link'; imu.linear_acceleration.z = 9.81
            cls.imu.publish(imu)
            battery = BatteryState(); battery.header.stamp = now; battery.current = -.1; battery.voltage = 50.; battery.present = False
            cls.battery.publish(battery)
            scan = LaserScan(); scan.header.stamp = now; scan.header.frame_id = 'laser'; scan.ranges = [1.]*36
            cls.scan.publish(scan)
        cls.controller.create_timer(.05, tick)
        def spin():
            while not cls.done.is_set(): cls.executor.spin_once(timeout_sec=.05)
        cls.thread = threading.Thread(target=spin)
        cls.thread.start()
        cls.root = Path(tempfile.mkdtemp(prefix='nav_jerk_ros_test_', dir='/tmp/njrh_reports'))
        cls.cfg = json.loads((HERE/'nav_jerk_capture.yaml').read_text())
        # Use the shipped discovery window: 1.5 s can miss cold Fast DDS endpoints.
        # Keep assertions on real received data; do not turn an empty bag into a pass.
        cls.cfg.update(discovery_sec=6, ready_timeout_sec=1, duration_sec=2, param_timeout_sec=1, param_total_sec=4,
                       source_files=[], log_globs=[], http_enabled=False, packages=[], min_free_mb=1,
                       nodes=['controller_server', 'velocity_smoother', 'collision_monitor', 'robot_safety', 'ranger_base_node'])
        cls.config = cls.root/'config.yaml'
        cls.config.write_text(json.dumps(cls.cfg))
        print('ISOLATED_REPORT=' + str(cls.root), flush=True)

    def run_capture(self, name, extra=(), interrupt=False, script=None):
        out = self.root / name
        log = (self.root/(name+'.log')).open('w')
        p = subprocess.Popen([sys.executable, str(script or HERE/'nav_jerk_capture.py'), 'record', '--workspace', str(self.root),
             '--config', str(self.config), '--output-dir', str(out), *extra], stdout=log, stderr=subprocess.STDOUT)
        try:
            if interrupt:
                deadline = time.monotonic()+30
                while not (out/'readiness.json').exists() and time.monotonic()<deadline and p.poll() is None: time.sleep(.1)
                self.assertTrue((out/'readiness.json').exists(), (self.root/(name+'.log')).read_text())
                m = subprocess.run([sys.executable, str(HERE/'nav_jerk_capture.py'), 'mark', '--directory', str(out), '--label', 'test_stop'], timeout=5, capture_output=True)
                self.assertEqual(m.returncode, 0, m.stderr)
                time.sleep(.3)
                p.send_signal(signal.SIGINT)
            p.wait(timeout=45)
        finally:
            if p.poll() is None:
                p.terminate()
                try: p.wait(timeout=5)
                except subprocess.TimeoutExpired: p.kill(); p.wait()
            log.close()
        return out, p.returncode

    def test_six_groups_continue_after_readiness_under_load(self):
        from rclpy.node import Node
        from geometry_msgs.msg import Twist, TwistStamped, PolygonStamped, Point32, TransformStamped, PoseStamped
        from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
        from map_msgs.msg import OccupancyGridUpdate
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import String
        from rcl_interfaces.msg import Log
        from rclpy.qos import QoSProfile, DurabilityPolicy
        import collections
        # Many independently ready topics expose the actual incident's starvation.
        extra = self.controller.create_publisher
        noise = [extra(String, '/ranger_mini3/fixture_'+str(i), 10) for i in range(32)]
        wheel = extra(Odometry, '/wheel/odom', 10)
        tf = extra(TFMessage, '/tf', 10)
        static = extra(TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        path = extra(RosPath, '/received_global_plan', 10)
        log = extra(Log, '/rosout', 10)
        n = Node('local_costmap', namespace='/local_costmap', enable_rosout=False)
        for key,value in [('width',10),('height',10),('resolution',.05)]: n.declare_parameter(key,value)
        self.executor.add_node(n); self.nodes.append(n)
        grid = n.create_publisher(OccupancyGrid, '/local_costmap/costmap', 10)
        update = n.create_publisher(OccupancyGridUpdate, '/local_costmap/costmap_updates', 10)
        footprint = n.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 10)
        cfg = dict(self.cfg); cfg['nodes'] = self.cfg['nodes']+['local_costmap/local_costmap']
        cfg['ready_timeout_sec'] = 5
        self.config.write_text(json.dumps(cfg))
        transform = TransformStamped(); transform.header.frame_id='base_link'; transform.child_frame_id='laser'; transform.transform.rotation.w=1.
        static.publish(TFMessage(transforms=[transform]))
        msg = Twist(); msg.linear.x=.3
        def fast():
            stamped = TwistStamped(); stamped.twist=msg; stamped.header.stamp=self.controller.get_clock().now().to_msg()
            self.raw.publish(stamped); self.smooth.publish(msg); self.checked.publish(msg); self.final.publish(msg)
        def evidence():
            now=self.controller.get_clock().now().to_msg()
            o=Odometry(); o.header.stamp=now; o.header.frame_id='odom'; o.twist.twist.linear.x=.2; wheel.publish(o)
            transform.header.stamp=now; tf.publish(TFMessage(transforms=[transform]))
            for p in noise: p.publish(String(data='fixture'))
        def geometry():
            now=self.controller.get_clock().now().to_msg()
            g=OccupancyGrid(); g.header.stamp=now; g.header.frame_id='odom'; g.info.width=g.info.height=200; g.info.resolution=.05; g.data=[0]*40000; grid.publish(g)
            u=OccupancyGridUpdate(); u.header=g.header; u.width=u.height=2; u.data=[0]*4; update.publish(u)
            f=PolygonStamped(); f.header=g.header; f.polygon.points=[Point32(x=.4,y=.3),Point32(x=-.4,y=-.3)]; footprint.publish(f)
            p=RosPath(); p.header=g.header; p.poses=[PoseStamped(header=g.header)]; path.publish(p)
            l=Log(); l.stamp=now; l.level=20; l.name='collision_monitor'; l.msg='fixture: StopZone stop / release'; log.publish(l)
        timers=[self.controller.create_timer(.005,fast),self.controller.create_timer(.05,evidence),self.controller.create_timer(.2,geometry)]
        measurements={}
        try:
            scripts=[('candidate',Path(os.getenv('NAV_JERK_TEST_SCRIPT',str(HERE/'nav_jerk_capture.py'))))]
            baseline=HERE/'baseline_nav_jerk_capture.py'
            if baseline.exists() and not os.getenv('NAV_JERK_TEST_SCRIPT'): scripts.insert(0,('baseline',baseline))
            for label,script in scripts:
                out,code=self.run_capture('load_'+label,['--duration','18'],script=script)
                self.assertEqual(code,0,(self.root/('load_'+label+'.log')).read_text())
                rows=[json.loads(s) for s in (out/'index.jsonl').read_text().splitlines()]
                first=min(r['monotonic_ns'] for r in rows)
                # Synthetic mark at +11 s: the entire +/-5 s lies after initial readiness.
                window=[r for r in rows if first+6e9<=r['monotonic_ns']<=first+16e9]
                counts=collections.Counter(r['topic'] for r in window)
                aux=[json.loads(s) for s in (out/'auxiliary.jsonl').read_text().splitlines()]
                cpu=[p['cpu_percent_one_core'] for r in aux if r.get('kind')=='performance'
                     for p in r['processes'] if p['pid']==r['recorder_pid'] and p.get('cpu_percent_one_core') is not None]
                measurements[label]={'window_counts':dict(counts),'cpu_mean_one_core':sum(cpu)/len(cpu) if cpu else None,'cpu_max_one_core':max(cpu) if cpu else None}
                if label=='candidate':
                    for topic in ('/wheel/odom','/fixture/odom','/fixture/scan','/tf','/local_costmap/costmap',
                                  '/local_costmap/costmap_updates','/local_costmap/published_footprint','/received_global_plan','/rosout'):
                        self.assertGreater(counts[topic],5,(topic,counts))
                    self.assertTrue(any(r['topic']=='/tf_static' for r in rows))
                    # Full scan and costmap remain in bag; no minima-only substitute.
                    self.assertTrue(any(r.get('grid',{}).get('width')==200 for r in rows))
        finally:
            for timer in timers: self.controller.destroy_timer(timer)
            self.config.write_text(json.dumps(self.cfg))
            (self.root/'six_group_load_comparison.json').write_text(json.dumps(measurements,indent=2))
            print('LOAD_COMPARISON='+json.dumps(measurements),flush=True)

    def test_real_bag_raw_index_report_and_midchain_zero(self):
        import sqlite3
        out, code = self.run_capture('normal', ['--duration', '5'])
        self.assertEqual(code, 0, (self.root/'normal.log').read_text())
        result = json.loads((out/'capture_result.json').read_text())
        self.assertFalse(result['writer']['writer_error'])
        rows = [json.loads(s) for s in (out/'index.jsonl').read_text().splitlines()]
        self.assertTrue(any(r['type'] == 'geometry_msgs/msg/TwistStamped' for r in rows))
        self.assertTrue(all(r['publisher_gid'] is None for r in rows))
        self.assertTrue(all('source_headers' in r for r in rows))
        from rclpy.serialization import deserialize_message
        from geometry_msgs.msg import TwistStamped
        for dbfile in (out/'bag').glob('*.db3'):
            db = sqlite3.connect(dbfile)
            self.assertEqual(db.execute('select count(*) from messages').fetchone()[0], len(rows))
            stamp, payload = db.execute("select m.timestamp,m.data from messages m join topics t on m.topic_id=t.id where t.name='/fixture/raw' limit 1").fetchone()
            raw = deserialize_message(payload, TwistStamped)
            indexed = next(r for r in rows if r['topic']=='/fixture/raw' and r['bag_timestamp_ns']==stamp)
            self.assertEqual(raw.header.frame_id, indexed['source_headers'][0]['frame'])
            self.assertEqual(raw.twist.linear.x, indexed['data']['twist']['linear']['x'])
            db.close()
        auxiliary = [json.loads(s) for s in (out/'auxiliary.jsonl').read_text().splitlines()]
        samples = [p for r in auxiliary if r.get('kind')=='performance'
                   for p in r['processes'] if p['pid']==r['recorder_pid']]
        self.assertTrue(any(p.get('cpu_percent_one_core') is not None and p.get('rss_kib', 0)>0 for p in samples))
        report_dir = self.root/'offline_report'
        p = subprocess.run([sys.executable, str(HERE/'nav_jerk_capture.py'), 'report', '--directory', str(out), '--output-dir', str(report_dir)], timeout=30, capture_output=True)
        self.assertEqual(p.returncode, 0, p.stderr.decode())
        events = (report_dir/'events.csv').read_text()
        self.assertIn('zero_command', events)
        self.assertIn('/fixture/checked', events)
        self.assertTrue((report_dir/'raw_bag.csv').stat().st_size > 100)

    def test_ctrl_c_preserves_bag_and_mark(self):
        out, code = self.run_capture('interrupt', ['--duration', '30'], interrupt=True)
        self.assertEqual(code, 0, (self.root/'interrupt.log').read_text())
        result = json.loads((out/'capture_result.json').read_text())
        self.assertTrue(result['incomplete'])
        self.assertEqual(result['reason'], 'signal_2')
        self.assertTrue((out/'bag/metadata.yaml').exists())
        self.assertIn('manual_mark', (out/'auxiliary.jsonl').read_text())

    def test_disk_budget_preserves_partial_metadata_without_recording(self):
        out, code = self.run_capture('disk', ['--disk-budget-mb', '1'])
        self.assertEqual(code, 1)
        result = json.loads((out/'capture_result.json').read_text())
        self.assertTrue(result['incomplete'])
        self.assertIn('disk_budget', result['error'])

    def test_real_qos_mismatch_is_reported_not_silently_called_matched(self):
        code = '''import threading, json
from nav_jerk_capture import RosObserver
o = RosObserver(threading.Event())
m = {'topics': {'/fixture/raw': {'types':['geometry_msgs/msg/TwistStamped'], 'critical':True,
     'subscription_qos': {'depth':10,'reliability':'RELIABLE','durability':'VOLATILE'}}},
     'missing_roles':{},'controller_odom_topic':'/fixture/odom'}
try:
 o.subscribe(m); o.spin(2)
 assert o.received['/fixture/raw'] == 0
 assert o.qos_events[('/fixture/raw','incompatible_qos')] > 0, dict(o.qos_events)
 assert o.coverage(m)['state'] == 'INCOMPLETE'
finally: o.close()
'''
        r = subprocess.run([sys.executable, '-c', code], cwd=HERE, capture_output=True, timeout=15)
        self.assertEqual(r.returncode, 0, r.stderr.decode())

    def test_write_failure_keeps_original_and_reports_incomplete(self):
        # Injection is strictly test-side, at storage boundary, not a production option.
        code = '''import json,threading,tempfile,time
from pathlib import Path
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Twist
from nav_jerk_capture import BagSink
cfg=json.loads(Path('nav_jerk_capture.yaml').read_text())
out=Path(tempfile.mkdtemp(dir='/tmp/njrh_reports',prefix='nav_jerk_write_failure_'))
m={'topics':{'/test/velocity':{'types':['geometry_msgs/msg/Twist'],'pubs':[]}}}
stop=threading.Event(); sink=BagSink(out,m,cfg,stop)
class FailedDisk:
 def write(self,*a): raise OSError(28,'No space left on device')
 def close(self): pass
 def flush(self): pass
sink.handles['index'].close(); sink.handles['index']=FailedDisk()
sink.inbox.put({'seq':1,'topic':'/test/velocity','raw':serialize_message(Twist()),'monotonic_ns':time.monotonic_ns(),'wall_ns':time.time_ns(),'ros_ns':None})
assert stop.wait(3)
result=sink.close(); assert 'No space' in result['writer_error']; assert (out/'bag/metadata.yaml').exists()
'''
        r = subprocess.run([sys.executable, '-c', code], cwd=HERE, capture_output=True, timeout=15)
        self.assertEqual(r.returncode, 0, r.stderr.decode())

    def test_transient_local_late_subscription_preserves_retained_message(self):
        code = '''import threading
from nav_jerk_capture import RosObserver
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
o=RosObserver(threading.Event())
try:
 p=o.node.create_publisher(String,'/fixture/retained',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 p.publish(String(data='published before observer subscription'))
 o.spin(.2)
 m={'topics':{'/fixture/retained':{'types':['std_msgs/msg/String'],'critical':True,
    'subscription_qos':{'depth':100,'reliability':'RELIABLE','durability':'TRANSIENT_LOCAL'}}}}
 o.subscribe(m); o.spin(1)
 assert o.received['/fixture/retained']==1, dict(o.received)
finally: o.close()
'''
        r = subprocess.run([sys.executable, '-c', code], cwd=HERE, capture_output=True, timeout=10)
        self.assertEqual(r.returncode, 0, r.stderr.decode())

    def test_budget_reached_during_capture_closes_partial_bag(self):
        out = self.root/'budget_during_capture'
        code = '''import sys,json
from pathlib import Path
import nav_jerk_capture as m
checks=[]
def budget(*args):
 checks.append(1)
 index=Path(sys.argv[3])/'index.jsonl'
 return 'disk_budget' if len(checks)>1 and index.exists() and index.stat().st_size>0 else None
m.budget_reason=budget
rc=m.main(['record','--workspace',sys.argv[1],'--config',sys.argv[2],'--output-dir',sys.argv[3],'--duration','15'])
r=json.loads((Path(sys.argv[3])/'capture_result.json').read_text())
assert rc==0 and r['reason']=='disk_budget' and r['incomplete'],r
assert sum(r['writer']['written'].values())>0
assert (Path(sys.argv[3])/'bag/metadata.yaml').exists()
'''
        r = subprocess.run([sys.executable, '-c', code, str(self.root), str(self.config), str(out)],
                           cwd=HERE, capture_output=True, timeout=30)
        self.assertEqual(r.returncode, 0, r.stderr.decode())

    @classmethod
    def tearDownClass(cls):
        cls.done.set(); cls.thread.join()
        cls.executor.shutdown()
        for n in cls.nodes: n.destroy_node()
        cls.rclpy.shutdown()


if __name__ == '__main__':
    assert_isolated()
    unittest.main()

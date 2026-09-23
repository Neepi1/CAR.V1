"""Offline CDR/sqlite fixtures: no ROS domain, network, or robot required."""
import csv
import importlib.util
import json
from pathlib import Path
import sqlite3
import struct
import tempfile
import unittest


class CdrWriter:
    def __init__(self):
        self.data = bytearray(b'\x00\x01\x00\x00')

    def number(self, kind, value):
        size = struct.calcsize(kind)
        self.data.extend(b'\x00' * (-(len(self.data) - 4) % size))
        self.data.extend(struct.pack('<' + kind, value))
        return self

    def string(self, value):
        raw = value.encode() + b'\x00'
        self.number('I', len(raw))
        self.data.extend(raw)
        return self

    def header(self, stamp=123, frame='odom'):
        return self.number('i', stamp // 10**9).number('I', stamp % 10**9).string(frame)

    def doubles(self, values):
        for value in values:
            self.number('d', value)
        return self


def twist(vx=0.2, stamped=False):
    w = CdrWriter()
    if stamped:
        w.header()
    return bytes(w.doubles([vx, 0, 0, 0, 0, -0.1]).data)


def odom():
    return bytes(CdrWriter().header().string('base_link').doubles(
        [1, 2, 0, 0, 0, 0, 1] + [0.] * 36 + [0.2, 0, 0, 0, 0, -.1] + [0.] * 36).data)


class EvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).with_name('nav_event_evidence_report.py')
        spec = importlib.util.spec_from_file_location('evidence', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / 'capture'
        motion = self.root / 'motion'
        (motion / 'bag').mkdir(parents=True)
        self.rows = []
        self.topics = [
            {'name': '/raw', 'role': 'controller', 'type': 'geometry_msgs/msg/Twist', 'critical': True, 'cadence': 'continuous', 'max_gap_sec': .75},
            {'name': '/wheel', 'role': 'wheel_odom', 'type': 'nav_msgs/msg/Odometry', 'critical': True, 'cadence': 'continuous', 'max_gap_sec': .75},
        ]
        self.db = sqlite3.connect(motion / 'bag' / 'data.db3')
        self.addCleanup(self.db.close)
        self.db.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        seq = 0
        for tid, topic in enumerate(self.topics, 1):
            self.db.execute('INSERT INTO topics VALUES(?,?,?,?)', (tid, topic['name'], topic['type'], 'cdr'))
            for ordinal in range(23):
                seq += 1
                mono = 10**12 + ordinal * 500000000
                stamp = 10**18 + seq
                raw = twist() if tid == 1 else odom()
                self.db.execute('INSERT INTO messages VALUES(?,?,?,?)', (seq, tid, stamp, raw))
                self.rows.append(dict(seq=seq, topic=topic['name'], topic_ordinal=ordinal+1,
                    bag_timestamp_ns=stamp, recv_monotonic_ns=mono, recv_wall_ns=mono+10**18,
                    recv_ros_ns=mono+10**18, publisher_gid='abc', rmw_source_timestamp_ns=0,
                    rmw_received_timestamp_ns=0, serialized_bytes=len(raw)))
        self.db.commit()
        (self.root / 'marks.jsonl').write_text(json.dumps({'label': 'jerk', 'read_monotonic_ns': 10**12+5500000000, 'read_wall_ns': 10**18+10**12+5500000000})+'\n')
        (motion / 'quality.json').write_text(json.dumps({'ready': True, 'incomplete': False, 'queue_drops': 0, 'topics': {}}))
        self.flush()

    def flush(self):
        motion = self.root / 'motion'
        (motion / 'topic_map.json').write_text(json.dumps({'schema': 1, 'topics': self.topics}))
        with (motion / 'receive_index.csv').open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            w.writeheader()
            w.writerows(self.rows)

    def report(self):
        self.flush()
        return self.module.build_report(self.root, Path(self.tmp.name)/'report', window=5)

    def codes(self, result):
        return {gap['code'] for gap in result['gaps']}

    def test_valid_real_cdr_and_repeated_source_stamp(self):
        result = self.report()
        self.assertTrue(result['motion_complete'], result['gaps'])
        self.assertEqual(len(result['coverage']), 2)
        with (Path(self.tmp.name)/'report'/'raw_messages.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        wheel = next(row for row in rows if row['topic'] == '/wheel')
        payload = json.loads(wheel['data_json'])
        self.assertEqual(payload['header']['stamp_ns'], 123)
        self.assertEqual(payload['pose']['position']['y'], 2)
        self.assertEqual(len(payload['twist_covariance']), 36)

    def test_missing_wheel_odom_fails(self):
        self.db.execute('DELETE FROM messages WHERE topic_id=2')
        self.db.commit()
        self.rows = [r for r in self.rows if r['topic'] == '/raw']
        result = self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('required_topic_missing', self.codes(result))

    def test_window_gap_and_not_just_files_present(self):
        self.rows = [r for r in self.rows if r['topic'] != '/wheel' or r['topic_ordinal'] < 5 or r['topic_ordinal'] > 18]
        result = self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('marker_window_gap', self.codes(result))

    def test_duplicate_index_is_not_arbitrarily_joined(self):
        self.rows.append(dict(self.rows[0]))
        result = self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('duplicate_receive_index', self.codes(result))

    def test_clock_jump_invalidates_wall_alignment(self):
        for row in self.rows:
            if row['topic_ordinal'] > 10:
                row['recv_wall_ns'] += 10**9
        result = self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('wall_clock_jump', self.codes(result))

    def test_event_topic_static_value_not_stale(self):
        self.topics[1]['cadence'] = 'event'
        self.rows = [r for r in self.rows if r['topic'] != '/wheel' or r['topic_ordinal'] == 1]
        self.db.execute('DELETE FROM messages WHERE topic_id=2 AND id<>24')
        self.db.commit()
        result = self.report()
        self.assertTrue(result['motion_complete'], result['gaps'])
        self.assertEqual(result['coverage'][1]['coverage_kind'], 'latest_observed_state_only')

    def test_task_previous_only_is_partial_not_fault(self):
        self.topics[0]['cadence']='task'
        self.rows=[r for r in self.rows if r['topic']!='/raw' or r['topic_ordinal']==1]
        self.db.execute('DELETE FROM messages WHERE topic_id=1 AND id<>1')
        self.db.commit()
        result=self.report()
        self.assertFalse(result['motion_complete'])
        row=next(r for r in result['coverage'] if r['topic']=='/raw')
        self.assertFalse(row['complete'])
        self.assertIn('not automatically a fault',row['activity_note'])
        self.assertNotEqual(row['coverage_kind'],'latest_observed_state_only')

    def test_wrong_expected_endpoint_blocks_chain_evidence(self):
        self.topics[0]['endpoint_expected']={'publisher':['/controller_server'],'subscriber':['/velocity_smoother']}
        self.topics[0]['publishers']=[{'node':'/unrelated_controller'}]
        self.topics[0]['subscribers']=[{'node':'//velocity_smoother'}]
        result=self.report()
        self.assertFalse(result['motion_complete'])
        missing=[g for g in result['gaps'] if g['code']=='chain_endpoint_binding_unproven']
        self.assertEqual(len(missing),1)
        self.assertEqual(missing[0]['expected_node'],'/controller_server')

    def test_required_manifest_topic_cannot_disappear_from_graph(self):
        (self.root/'topic_manifest.json').write_text(json.dumps({'topics':[
            {'name':'/missing_safety','critical':True,'role':'safety_output'}],
            'chain_gaps':['controller_server: process nonunique']}))
        result=self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('required_topic_not_in_recorded_graph',self.codes(result))
        self.assertIn('chain_discovery_unproven',self.codes(result))

    def test_truncated_cdr_fails(self):
        self.db.execute('UPDATE messages SET data=? WHERE id=1', (b'\0\1\0\0',))
        self.db.commit()
        result = self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('cdr_decode_failed', self.codes(result))

    def test_twist_stamped_and_scan(self):
        decoded = self.module.decode_cdr('geometry_msgs/msg/TwistStamped', twist(-.2, True))
        self.assertEqual(decoded['twist']['linear']['x'], -.2)
        w = CdrWriter().header(456, 'laser')
        for v in [-1, 1, .5, .001, .1, .2, 8]:
            w.number('f', v)
        w.number('I', 3)
        for v in [1, float('inf'), 3]:
            w.number('f', v)
        w.number('I', 0)
        data = self.module.decode_cdr('sensor_msgs/msg/LaserScan', bytes(w.data))
        self.assertEqual(data['ranges'], [1., float('inf'), 3.])
        self.assertEqual(data['header']['frame_id'], 'laser')

    def test_zero_and_gap_are_different_events(self):
        self.db.execute('UPDATE messages SET data=? WHERE id=8', (twist(0),))
        self.db.commit()
        self.rows = [r for r in self.rows if r['topic'] != '/raw' or r['topic_ordinal'] < 13 or r['topic_ordinal'] > 17]
        self.report()
        with (Path(self.tmp.name)/'report'/'velocity_events.csv').open() as stream:
            events = list(csv.DictReader(stream))
        kinds = {r['kind'] for r in events}
        self.assertIn('vx_zero', kinds)
        self.assertIn('observed_receive_gap', kinds)

    def test_mppi_payload_truncation_not_complete(self):
        snap = self.root / 'mppi_1_2'
        snap.mkdir()
        (snap/'schema.json').write_text(json.dumps({'schema': 1, 'byte_order': 'little', 'session': 'a'}))
        (snap/'payload.bin').write_bytes(b'\0')
        frame = {'compute_seq': 1, 'start_monotonic_ns': 10**12+5*10**9,
                 'end_monotonic_ns': 10**12+5*10**9+10, 'session': 'a',
                 'map_frame': 'odom', 'pose_frame': 'odom', 'path_frame': 'odom',
                 'base_frame': 'base_link', 'costmap': {'offset': 0, 'length': 4, 'dtype': 'u1', 'shape': [2,2]},
                 'map_dimensions': [2,2], 'incomplete': False, 'sequence_valid': False}
        (snap/'frames.jsonl').write_text(json.dumps(frame)+'\n')
        (snap/'status.json').write_text(json.dumps({'complete': True, 'dropped': 0, 'write_errors': 0}))
        result = self.report()
        self.assertFalse(result['mppi']['complete'])
        self.assertIn('mppi_payload_invalid', self.codes(result))

    def test_boot_id_mismatch_fails(self):
        (self.root/'summary.json').write_text(json.dumps({'boot_id':'new-boot'}))
        result=self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('marker_boot_id_mismatch',self.codes(result))

    def test_grid_update_dimensions_and_footprint_cdr(self):
        w=CdrWriter().header(4,'odom').number('i',1).number('i',2).number('I',2).number('I',1).number('I',2)
        w.number('b',-1).number('b',100)
        data=self.module.decode_cdr('map_msgs/msg/OccupancyGridUpdate',bytes(w.data))
        self.assertEqual(data['data'],[-1,100])
        w=CdrWriter().header(4,'odom').number('I',3)
        for point in ((0,0,0),(1,0,0),(0,1,0)):
            for v in point: w.number('f',v)
        data=self.module.decode_cdr('geometry_msgs/msg/PolygonStamped',bytes(w.data))
        self.assertEqual(len(data['points']),3)

    def test_partial_json_retains_report_and_marks_incomplete(self):
        with (self.root/'marks.jsonl').open('a') as stream:
            stream.write('{"label":')
        result=self.report()
        self.assertFalse(result['motion_complete'])
        self.assertIn('jsonl_record_invalid',self.codes(result))

    def test_nav2_costmap_cdr_unsigned_cells(self):
        w=CdrWriter().header(4,'odom')
        for _ in range(2): w.number('i',0).number('I',5)
        w.string('master').number('f',.05).number('I',2).number('I',1)
        w.doubles([0,0,0,0,0,0,1]).number('I',2).number('B',254).number('B',255)
        data=self.module.decode_cdr('nav2_msgs/msg/Costmap',bytes(w.data))
        self.assertEqual(data['data'],[254,255])

    def test_mppi_valid_payload_but_single_frame_not_complete_window(self):
        snap=self.root/'mppi_valid'
        snap.mkdir()
        (snap/'schema.json').write_text(json.dumps({'schema':1,'byte_order':'little','session':'a'}))
        path=struct.pack('<7d',1,2,0,0,0,0,1)
        stamp=struct.pack('<q',123)
        sequence=struct.pack('<6d',.2,0,-.1,.3,0,-.1)
        (snap/'payload.bin').write_bytes(bytes([0,0,0,254])+path+stamp+sequence)
        frame={'compute_seq':1,'start_monotonic_ns':10**12+5*10**9,'end_monotonic_ns':10**12+5*10**9+10,
               'map_frame':'odom','pose_frame':'odom','path_frame':'odom','base_frame':'base_link','session':'a',
               'map_dimensions':[2,2],'resolution':.05,'pose_stamp_ns':123,'path_stamp_ns':123,
               'footprint':[[.5,.3,0],[.5,-.3,0],[-.5,-.3,0]],'path_frames_all_equal_header':True,
               'costmap':{'offset':0,'length':4,'dtype':'u1','shape':[2,2]},
               'path':{'offset':4,'length':56,'dtype':'f8','shape':[1,7]},
               'path_stamps':{'offset':60,'length':8,'dtype':'i8','shape':[1]},
               'sequence':{'offset':68,'length':48,'dtype':'f8','shape':[2,3]},
               'command_offset':0,'command_returned':True,'sequence_valid':True,'command':[.2,0,0,0,0,-.1]}
        (snap/'frames.jsonl').write_text(json.dumps(frame)+'\n')
        (snap/'status.json').write_text(json.dumps({'complete':True}))
        result=self.report()
        self.assertNotIn('mppi_payload_invalid',self.codes(result))
        self.assertIn('mppi_marker_window_partial',self.codes(result))
        self.assertFalse(result['mppi']['complete'])
        self.assertTrue(result['mppi']['instances'][0]['valid'])

    def test_pre_map_failure_is_recorded_observation_not_corrupt_payload(self):
        snap=self.root/'mppi_premap'
        snap.mkdir()
        (snap/'schema.json').write_text(json.dumps({'schema':1,'byte_order':'little','session':'a'}))
        (snap/'status.json').write_text(json.dumps({'complete':True}))
        (snap/'payload.bin').write_bytes(b'')
        frame={'compute_seq':1,'start_monotonic_ns':10,'end_monotonic_ns':20,'session':'a',
               'command_returned':False,'sequence_valid':False,'stage':'transformPath',
               'exception':'Received plan with zero length','exception_type':'St13runtime_error',
               'pose_frame':'map','pose_stamp_ns':123,'path_stamp_ns':0,'map_copy_monotonic_ns':0,
               'map_frame':'','path_frame':'','base_frame':'','footprint':[],'resolution':0,
               'map_dimensions':[0,0],'path_original_count':0,'sequence_original_count':0,
               'path_frames_all_equal_header':True,
               'costmap':{'offset':0,'length':0,'dtype':'u1','shape':[0,0]},
               'path':{'offset':0,'length':0,'dtype':'f8','shape':[0,7]},
               'path_stamps':{'offset':0,'length':0,'dtype':'i8','shape':[0]},
               'sequence':{'offset':0,'length':0,'dtype':'f8','shape':[0,3]}}
        destination=Path(self.tmp.name)/'premap_report'
        destination.mkdir()
        for stage in ('transformPath','idle_reset','parameters_lock','costmap_lock'):
            with self.subTest(stage=stage):
                frame['stage']=stage
                (snap/'frames.jsonl').write_text(json.dumps(frame)+'\n')
                gaps=[]
                result=self.module._mppi_report(self.root,destination,[],5,gaps)
                self.assertTrue(result['complete'],gaps)
                self.assertEqual(result['pre_map_failure_frames'],1)
                self.assertFalse(result['observations'][0]['geometry_available'])
                self.assertEqual(result['observations'][0]['exception'],'Received plan with zero length')
        for change in ({'stage':'optimize'},{'command_returned':True},{'exception':''},
                       {'exception_type':''},{'map_copy_monotonic_ns':1},{'map_dimensions':[1,1]}):
            with self.subTest(change=change):
                bad=dict(frame,stage='transformPath')
                bad.update(change)
                (snap/'frames.jsonl').write_text(json.dumps(bad)+'\n')
                gaps=[]
                result=self.module._mppi_report(self.root,destination,[],5,gaps)
                self.assertFalse(result['complete'])
                self.assertTrue(any(g['code']=='mppi_payload_invalid' for g in gaps))


if __name__ == '__main__':
    unittest.main()

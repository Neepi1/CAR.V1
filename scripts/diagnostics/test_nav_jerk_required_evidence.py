"""Recorder regressions at the existing HTTP/quality interfaces; no robot access."""
import importlib.util
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = Path(__file__).resolve().parent
def module():
    spec = importlib.util.spec_from_file_location('capture', HERE/'nav_jerk_capture.py')
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m

class RequiredEvidenceTests(unittest.TestCase):
    def test_each_marker_exposes_missing_wheel_window_and_retained_static(self):
        m=module()
        rows=m.marker_window_coverage(
            {'/wheel/odom':[(1_000_000_000,1)], '/tf_static':[(1_000_000_000,2)],
             '/scan':[(9_000_000_000,3),(11_000_000_000,4)]},
            {'topics':{'/wheel/odom':{'cadence':'continuous'}, '/scan':{'cadence':'continuous'},
                       '/tf_static':{'cadence':'event'}}},
            [{'id':'operator-1','monotonic_ns':10_000_000_000}],5.)
        by_topic={r['topic']:r for r in rows}
        self.assertEqual(by_topic['/wheel/odom']['window_count'],0)
        self.assertEqual(by_topic['/wheel/odom']['state'],'MISSING_IN_WINDOW')
        self.assertEqual(by_topic['/scan']['before_count'],1)
        self.assertEqual(by_topic['/scan']['after_count'],1)
        self.assertEqual(by_topic['/scan']['first_seq'],3)
        self.assertEqual(by_topic['/tf_static']['state'],'RETAINED_BEFORE_WINDOW')

    def test_initial_messages_do_not_hide_later_motion_data_gap(self):
        m=module()
        mapping={'topics':{
            '/wheel/odom':{'types':['nav_msgs/msg/Odometry'],'cadence':'continuous'},
            '/scan':{'types':['sensor_msgs/msg/LaserScan'],'cadence':'continuous'},
            '/speed_limit':{'types':['nav2_msgs/msg/SpeedLimit'],'cadence':'event'}}}
        health=m.continuous_health(mapping,{'/wheel/odom':208,'/scan':77},
            {'/wheel/odom':5_000_000_000,'/scan':10_000_000_000},11_000_000_000,2.)
        self.assertEqual(health['stale_topics'],['/wheel/odom'])
        self.assertEqual(health['state'],'INCOMPLETE')
        self.assertNotIn('/speed_limit',health['topics'])

    def test_http_uses_existing_robot_token_contract(self):
        m = module()
        stop = threading.Event()
        rows = []
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200 if self.headers.get('X-Robot-Token')=='fixture-token' else 401)
                self.end_headers(); self.wfile.write(b'{"ok":true}')
            def log_message(self, *args): pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        worker = threading.Thread(target=server.serve_forever); worker.start()
        class Sink:
            def auxiliary(self, kind, value):
                rows.append(value); stop.set()
        cfg = m.load_config(HERE/'nav_jerk_capture.yaml')
        cfg['http_base'] = 'http://127.0.0.1:' + str(server.server_port)
        try:
            with patch.dict('os.environ', {'ROBOT_API_TOKEN': 'fixture-token'}):
                m.http_worker(cfg, stop, Sink())
        finally:
            server.shutdown(); worker.join(); server.server_close()
        self.assertEqual(rows[0].get('status'), 200, rows)

if __name__ == '__main__': unittest.main()

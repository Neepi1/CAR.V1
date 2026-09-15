#!/usr/bin/env python3
"""Real API regression. Run ONLY in private PID/mount/network/IPC namespaces.

Publishes synthetic low-rate TF in an isolated ROS domain; no action, service,
motion or map-switch requests. Exercises the pose -> map -> mapping snapshot
path that a status-only cold API smoke test misses.
"""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('binary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=100)
    args = parser.parse_args()
    assert len(subprocess.check_output(['ip', '-o', 'link'], text=True).splitlines()) == 1
    assert len([p for p in Path('/proc').iterdir() if p.name.isdigit()]) < 12, 'private PID namespace required'
    for namespace in ('net', 'ipc', 'mnt', 'pid'):
        # Runner supplies the original namespace identity, before unshare.
        original = os.environ['API_TEST_ORIGINAL_' + namespace.upper()]
        assert os.readlink('/proc/self/ns/' + namespace) != original
    os.environ.update(ROS_DOMAIN_ID='228', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for key in list(os.environ):
        if key.startswith(('NJRH_', 'ROBOT_')) or key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            os.environ.pop(key)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    args.output.mkdir(parents=True, exist_ok=True)
    requests = []
    result = {'ok': False, 'requests': requests, 'motion_requests': 0,
              'synthetic_tf_only': True, 'binary': str(args.binary)}

    def get(path):
        start = time.monotonic()
        try:
            with urllib.request.urlopen('http://127.0.0.1:18549' + path, timeout=2) as response:
                code, body = response.status, json.load(response)
        except urllib.error.HTTPError as error:
            code, body = error.code, json.load(error)
        sample = {'path': path, 'status': code, 'ms': round((time.monotonic()-start)*1000, 3)}
        requests.append(sample)
        assert code == 200 and body.get('ok', True), (sample, body)
        if path == '/api/v1/robot/pose':
            assert abs(body['x'] - 1.25) < 1e-6 and abs(body['y'] - 2.5) < 1e-6, body
        return body

    with tempfile.TemporaryDirectory(prefix='pose_query_', dir=args.output) as temporary:
        root = Path(temporary)
        # Ancestor PID namespaces still see child processes. Never execute the
        # production path: the runtime's exact-executable ownership checker
        # would legitimately count it as a second production API owner.
        fixture_binary = root/'isolated_query_fixture'
        shutil.copy2(args.binary.resolve(), fixture_binary)
        assert fixture_binary.resolve() != args.binary.resolve()
        paths = {'maps_root': 'maps', 'runtime_maps_dir': 'runtime_maps',
                 'runtime_map_context_file': 'context.json', 'last_navigation_map_file': 'last_map.json',
                 'docking_contact_latch_file': 'dock_latch.json', 'amcl_runtime_status_file': 'amcl.env',
                 'mapping_2d_log_file': 'mapping.log', 'mapping_lidar_rps_xps_state_dir': 'rps_state',
                 'docking_manager_log_file': 'docking.log', 'navigation_resume_log_file': 'resume.log',
                 'navigation_stop_log_file': 'stop.log'}
        params = {key: str(root/value) for key, value in paths.items()}
        params.update(host='127.0.0.1', port=18549, api_token='isolated-regression',
                      mapping_2d_start_command='/bin/false', navigation_resume_command='/bin/false',
                      navigation_stop_command='/bin/false', docking_manager_start_command='/bin/false',
                      elevator_runtime_adapter_enabled='false', elevator_arm_button_control_enabled='false',
                      service_timeout_sec=0.5)
        opener = urllib.request.build_opener()
        opener.addheaders = [('X-Robot-Token', 'isolated-regression')]
        urllib.request.install_opener(opener)
        command = [str(fixture_binary), '--ros-args']
        for key, value in params.items():
            command += ['-p', f'{key}:={value}']
        rclpy.init()
        node = rclpy.create_node('isolated_pose_query_fixture')
        publisher = node.create_publisher(TFMessage, '/tf', 10)
        stopped = threading.Event()

        def publish_tf():
            while not stopped.is_set():
                transforms = []
                for parent, child, x, y in (('map', 'odom', 1.25, 2.5), ('odom', 'base_link', 0., 0.)):
                    transform = TransformStamped()
                    transform.header.stamp = node.get_clock().now().to_msg()
                    transform.header.frame_id = parent
                    transform.child_frame_id = child
                    transform.transform.translation.x = x
                    transform.transform.translation.y = y
                    transform.transform.rotation.w = 1.
                    transforms.append(transform)
                publisher.publish(TFMessage(transforms=transforms))
                stopped.wait(0.1)

        thread = threading.Thread(target=publish_tf)
        thread.start()
        with (args.output/'api.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ, ROS_LOG_DIR=str(root/'ros_logs')))
            try:
                deadline = time.monotonic()+35
                while True:
                    assert process.poll() is None, 'API exited during startup'
                    assert time.monotonic() < deadline, 'API startup timeout'
                    try:
                        get('/api/v1/openapi')
                        if publisher.get_subscription_count():
                            break
                    except (OSError, urllib.error.URLError):
                        pass
                    stopped.wait(.1)
                stopped.wait(.3)
                # This must succeed, not merely return fast with NO_FRESH_POSE.
                get('/api/v1/robot/pose')
                paths = ['/api/v1/robot/pose', '/api/v1/navigation/state', '/api/v1/maps', '/api/v1/status'] * 3
                for _ in range(args.rounds):
                    barrier = threading.Barrier(len(paths))
                    def simultaneous_get(path):
                        barrier.wait(timeout=5)
                        return get(path)
                    with concurrent.futures.ThreadPoolExecutor(max_workers=len(paths)) as pool:
                        list(pool.map(simultaneous_get, paths))
                result['ok'] = True
                result['max_ms'] = max(item['ms'] for item in requests)
            except Exception as error:
                result['error'] = type(error).__name__ + ': ' + str(error)
            finally:
                stopped.set()
                thread.join(timeout=2)
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
                print(json.dumps({k: v for k, v in result.items() if k != 'requests'}, indent=2))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

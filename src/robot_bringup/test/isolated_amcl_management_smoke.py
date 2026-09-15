#!/usr/bin/env python3
"""Isolated guard/IPC/lifecycle integration. No robot services or commands."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guard-bin', required=True)
    parser.add_argument('--client-bin', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '223' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Requires isolated ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1')
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from lifecycle_msgs.msg import TransitionEvent
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = Node('amcl', enable_rosout=False)
    guard = None
    with tempfile.TemporaryDirectory(prefix='isolated_amcl_management_') as temp:
        path = Path(temp)
        status = path / 'amcl.env'
        log = path / 'guard.log'
        try:
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                others = [name for name in node.get_node_names() if name != 'amcl']
                if others:
                    raise RuntimeError(f'Isolation domain is occupied: {others}')
            event_pub = node.create_publisher(TransitionEvent, '/amcl/transition_event', 1)
            pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 1)
            scan_pub = node.create_publisher(String, '/scan_amcl', 1)
            admission_pub = node.create_publisher(String, '/amcl_scan_admission/status', 1)
            del scan_pub, admission_pub  # Node retains publisher ownership.
            env = os.environ.copy()
            env.update(NJRH_RUNTIME_MANAGEMENT_ENABLED='true',
                       NJRH_AMCL_RUNTIME_STATUS_FILE=str(status),
                       NJRH_RUNTIME_HEALTH_FILE=str(path / 'health.json'),
                       NJRH_RUNTIME_HEALTH_OBSERVE_TF='false',
                       NJRH_RUNTIME_HEALTH_OBSERVE_TOPIC_MESSAGES='false',
                       NJRH_RUNTIME_HEALTH_OBSERVE_HEAVY_TOPICS='false',
                       NJRH_RUNTIME_HEALTH_SAMPLE_PERIOD_SEC='1',
                       NJRH_RUNTIME_HEALTH_GRAPH_PERIOD_SEC='5')
            with log.open('w') as stream:
                guard = subprocess.Popen([args.guard_bin, '--output', str(path / 'health.json')],
                                         stdout=stream, stderr=subprocess.STDOUT, env=env)
            pidfile = path / 'amcl.pid'
            pidfile.write_text(str(os.getpid()))
            fields = dict(AMCL_MODE='gated', AMCL_START_RESULT='ready', AMCL_READY='true',
                          AMCL_DEGRADED='false', LIFECYCLE_VERIFIED='true',
                          AMCL_SEED_SUCCEEDED='true', AMCL_SEED_RESPONSE_OK='true',
                          AMCL_STATIC_STANDBY_ACCEPTED='true', SCAN_ADMISSION_ENABLED='false',
                          NODE_NAME='amcl', POSE_TOPIC='/amcl_pose', SCAN_TOPIC='/scan_amcl')

            def client(operation, evidence=False):
                command = [args.client_bin, '--status-file', str(status), operation]
                if evidence:
                    command += ['--owner-pid', str(os.getpid()), '--owner-generation', 'test-owner',
                                '--map-generation', 'test-map', '--amcl-pid-file', str(pidfile),
                                '--amcl-exe', sys.executable]
                    for key, value in fields.items():
                        command += ['--set', f'{key}={value}']
                result = subprocess.run(command, capture_output=True, text=True, timeout=3)
                if result.returncode:
                    raise RuntimeError(result.stderr)
                return result.stdout

            def read_status():
                if guard.poll() is not None:
                    raise RuntimeError(f'Guard exited {guard.returncode}: {log.read_text()}')
                if not status.exists():
                    return {}
                return {k: v.strip('"') for k, v in
                        (line.split('=', 1) for line in status.read_text().splitlines())}

            def until(predicate, seconds=12, publish=None):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if publish:
                        publish()
                    rclpy.spin_once(node, timeout_sec=0.05)
                    current = read_status()
                    if predicate(current):
                        return current
                raise AssertionError(f'Condition timed out: {read_status()}')

            deadline = time.monotonic() + 8
            while not Path(str(status) + '.control/socket').exists():
                if time.monotonic() > deadline:
                    raise AssertionError(log.read_text())
                time.sleep(0.05)
            client('ping')
            client('register', True)
            current = until(lambda d: d.get('AMCL_TRACKING_READY') == 'true' and
                            d.get('AMCL_POSE_PUBLISHER_COUNT') == '1')
            assert current['AMCL_STATIC_STANDBY'] == 'true'
            assert current['AMCL_CORRECTION_READY'] == 'false'
            assert current['AMCL_STATUS_WRITER'] == 'runtime_health_guard'
            previous_stamp = current['AMCL_STATUS_STAMP_SEC']
            until(lambda d: d.get('AMCL_STATUS_STAMP_SEC') != previous_stamp)

            def event(state):
                msg = TransitionEvent()
                msg.goal_state.id = state
                event_pub.publish(msg)

            until(lambda d: d.get('AMCL_LIFECYCLE_ACTIVE') == 'false' and
                  d.get('AMCL_TRACKING_READY') == 'false', publish=lambda: event(2))
            until(lambda d: d.get('AMCL_LIFECYCLE_ACTIVE') == 'true' and
                  d.get('AMCL_TRACKING_READY') == 'true', publish=lambda: event(3))

            def pose():
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                pose_pub.publish(msg)

            until(lambda d: d.get('AMCL_LAST_POSE_AGE_MS', '') != '', publish=pose)
            fields.update(AMCL_MODE='disabled', AMCL_START_RESULT='disabled')
            client('submit', True)
            current = until(lambda d: d.get('AMCL_STATE') == 'AMCL_DISABLED')
            assert current['AMCL_TRACKING_READY'] == 'false'
            print(json.dumps({'static_standby': 'passed', 'native_ipc': 'passed',
                              'lifecycle_inactive_active': 'passed', 'pose_take': 'passed',
                              'disabled_transition': 'passed', 'isolated_domain': 223}))
        finally:
            if guard is not None:
                guard.terminate()
                try:
                    guard.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    guard.kill()
                    guard.wait(timeout=5)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

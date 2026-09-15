#!/usr/bin/env python3
"""Small topic/service + reader-teardown smoke; private netns is mandatory."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def assert_isolated():
    assert os.environ.get('NJRH_DDS_ISOLATED_TEST') == '1'
    assert os.readlink('/proc/self/ns/net') != os.readlink('/proc/1/ns/net')
    assert os.environ.get('ROS_DOMAIN_ID') == '183'
    assert os.environ.get('RMW_IMPLEMENTATION') == 'rmw_fastrtps_cpp'


def qos(durable=False):
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=(DurabilityPolicy.TRANSIENT_LOCAL if durable
                                 else DurabilityPolicy.VOLATILE))


def writer():
    rclpy.init()
    node = rclpy.create_node('dds_upgrade_writer')
    loaded = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                     if '/libfastrtps.so.' in line})
    print(json.dumps({'writer_dds_library': loaded,
                      'transport': os.environ['FASTDDS_BUILTIN_TRANSPORTS']}), flush=True)
    pub = node.create_publisher(String, '/dds_upgrade/sequence', qos())
    latched = node.create_publisher(String, '/dds_upgrade/static', qos(True))
    latched.publish(String(data='static-payload'))
    state = {'count': 0}

    def tick():
        state['count'] += 1
        pub.publish(String(data=str(state['count'])))

    def answer(request, response):
        response.success = True
        response.message = str(state['count'])
        return response

    node.create_timer(0.05, tick)
    node.create_service(Trigger, '/dds_upgrade/ping', answer)
    # Fixed lifetime lets the test assert normal writer shutdown, not mask it
    # with SIGKILL on success. An outer process timeout handles real deadlocks.
    deadline = time.monotonic() + 35
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({'writer_published': state['count'], 'shutdown': 'normal'}), flush=True)


def churn():
    rclpy.init()
    previous = 0
    for attempt in range(20):
        node = rclpy.create_node('dds_upgrade_reader_' + str(attempt))
        samples, static = [], []
        node.create_subscription(String, '/dds_upgrade/sequence',
                                 lambda msg: samples.append(int(msg.data)), qos())
        node.create_subscription(String, '/dds_upgrade/static',
                                 lambda msg: static.append(msg.data), qos(True))
        client = node.create_client(Trigger, '/dds_upgrade/ping')
        future = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if future is None and client.service_is_ready():
                future = client.call_async(Trigger.Request())
            rclpy.spin_once(node, timeout_sec=0.05)
            if len(samples) >= 3 and static and future is not None and future.done():
                break
        assert len(samples) >= 3 and max(samples) > previous, (attempt, samples)
        assert static[-1] == 'static-payload', (attempt, static)
        assert future is not None and future.done() and future.result().success, attempt
        previous = max(samples)
        node.destroy_node()
        print(json.dumps({'reader': attempt, 'last_sequence': previous,
                          'topic': 'pass', 'transient_local': 'pass', 'service': 'pass'}), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    assert_isolated()
    if len(sys.argv) > 1 and sys.argv[1] == 'writer':
        writer()
    else:
        child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), 'writer'])
        try:
            churn()
            assert child.wait(timeout=40) == 0
            print('ROS_TRANSPORT_SMOKE_PASS', flush=True)
        finally:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()

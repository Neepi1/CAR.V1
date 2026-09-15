#!/usr/bin/env python3
"""Exercise the real bridge in a private localhost domain; no robot interfaces."""
import argparse
import json
import math
import os
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class Fixture(Node):
    def __init__(self):
        super().__init__('correction_application_fixture')
        self.status = {}
        self.transforms = []
        self.odom = self.create_publisher(Odometry, '/local_state/odometry', 10)
        self.isaac = self.create_publisher(PoseWithCovarianceStamped, '/localization_result', 10)
        self.amcl = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', lambda msg: None, 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.025, self.publish_odom)
        self.create_subscription(String, '/localization/bridge_status', self.on_status, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.force = self.create_client(Trigger, '/robot_localization_bridge/force_accept_next_localization')

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self.transforms.append((tf.transform.translation.x,
                                        2 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)))

    def publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)

    def wait(self, predicate, seconds=6):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.025)
            if predicate():
                return
        raise AssertionError(f'timeout; status={self.status}')

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.025)

    def pose(self, publisher, x, yaw):
        msg = PoseWithCovarianceStamped()
        # Existing odom history, newer than explicit arm; never latest-TF fallback.
        msg.header.stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=0.05)).to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        for index in (0, 7, 35):
            msg.pose.covariance[index] = 0.001
        publisher.publish(msg)

    def explicit(self, x, yaw, label):
        old = self.status.get('last_explicit_relocalization_sequence', 0)
        future = self.force.call_async(Trigger.Request())
        self.wait(future.done)
        assert future.result().success, future.result()
        self.spin_for(0.10)
        start = len(self.transforms)
        self.pose(self.isaac, x, yaw)
        self.wait(lambda: self.status.get('last_explicit_relocalization_sequence', 0) > old
                  and self.status.get('target_sequence', 0) > 0
                  and self.status.get('target_sequence') == self.status.get('last_explicit_map_odom_target_sequence')
                  and self.status.get('current_sequence') == self.status.get('target_sequence')
                  and self.status.get('map_odom_last_published_sequence') == self.status.get('target_sequence'))
        state = self.status
        if label != 'initial_lock':
            assert state['smoothing_policy'] == 'explicit_relocalization_immediate', state
        assert state['smoothing_total_duration_sec'] == 0.0, state
        assert not state['correction_active'], state
        self.spin_for(0.10)
        observed = self.transforms[start:]
        assert any(abs(tx-x) < 1e-5 and abs(tyaw-yaw) < 1e-5 for tx, tyaw in observed), observed
        # Ignore queued old samples; no intermediate transform may be published.
        previous = self.previous
        assert all((abs(tx-x) < 1e-5 and abs(tyaw-yaw) < 1e-5) or
                   (abs(tx-previous[0]) < 1e-5 and abs(tyaw-previous[1]) < 1e-5)
                   for tx, tyaw in observed), observed
        self.previous = (x, yaw)
        print(json.dumps({'case': label, 'published_sequence': state['map_odom_last_published_sequence'],
                          'policy': state['smoothing_policy'], 'duration': state['smoothing_total_duration_sec']}))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bridge', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '219' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('requires ROS_DOMAIN_ID=219 and ROS_LOCALHOST_ONLY=1')
    params = {'amcl_input_enabled': 'true', 'amcl_gate_mode': 'gated',
              'publish_rate_hz': '50.0', 'map_odom_smoothing_publish_rate_hz': '50.0',
              'amcl_runtime_status_file': '/tmp/nonexistent_isolated_amcl_status',
              'amcl_initial_pose_seed_enabled': 'true', 'amcl_initial_pose_publish_repetitions': '1',
              'amcl_post_isaac_refine_enabled': 'false',
              'amcl_accept_when_isaac_recently_triggered_delay_sec': '0.0',
              'map_odom_smoothing_translation_rate_mps': '0.20',
              'map_odom_smoothing_yaw_rate_radps': '0.25',
              'max_odom_tf_age_ms': '1000.0'}
    command = [args.bridge, '--ros-args', '-r', '__node:=isolated_correction_bridge']
    for key, value in params.items():
        command += ['-p', f'{key}:={value}']
    with tempfile.TemporaryFile(mode='w+') as log:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        node = None
        try:
            rclpy.init()
            node = Fixture()
            node.previous = (0.0, 0.0)
            node.wait(lambda: node.force.service_is_ready() and node.isaac.get_subscription_count() > 0)
            node.spin_for(0.3)
            node.explicit(0.0, 0.0, 'initial_lock')
            node.explicit(2.0, 0.5, 'large_explicit')
            node.explicit(2.08, 0.55, 'small_explicit')
            start = len(node.transforms)
            count = node.status.get('amcl_accepted_count', 0)
            node.pose(node.amcl, 2.14, 0.65)
            node.wait(lambda: node.status.get('amcl_accepted_count', 0) > count)
            node.wait(lambda: not node.status.get('correction_active', True))
            node.spin_for(0.1)
            assert node.status['smoothing_policy'] == 'default', node.status
            assert abs(node.status['smoothing_total_duration_sec'] - 0.4) < 0.01, node.status
            assert any(2.081 < x < 2.139 for x, _ in node.transforms[start:]), node.transforms[start:]
            assert node.status['smoothing_translation_rate_mps'] == 0.20, node.status
            assert node.status['smoothing_yaw_rate_radps'] == 0.25, node.status
            print(json.dumps({'case': 'ordinary_amcl', 'policy': 'default', 'intermediate_tf_observed': True,
                              'duration': node.status['smoothing_total_duration_sec']}))
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


if __name__ == '__main__':
    main()

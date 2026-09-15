#!/usr/bin/env python3
"""Exercise the real wrapper with fake Isaac/bridge peers in an isolated ROS domain."""
import argparse
import json
import os
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from robot_interfaces.srv import TriggerLocalization
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import TransformBroadcaster


def run(binary, case, tracked=False):
    if tracked:
        from robot_interfaces.srv import TriggerLocalizationTracked
        from robot_interfaces.msg import LocalizationTriggerStatus
    node = Node('trigger_confirmation_fixture')
    status_pub = node.create_publisher(String, '/localization/bridge_status', 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/localization_result', 10)
    tf = TransformBroadcaster(node)
    dispatched_at = None
    dispatch_count = 0
    outcomes = []
    status = dict(
        accepted_result_count=0, rejected_result_count=0,
        last_explicit_relocalization_sequence=0,
        last_explicit_map_odom_target_sequence=0,
        current_sequence=1, target_sequence=1,
        has_map_to_odom=True, safe_for_goal_start=True, correction_active=False,
        map_to_odom_age_ms=10.0, map_odom_publish_gap_ms=20.0,
        map_to_odom_publisher_owner='robot_localization_bridge',
        current_source='isaac_triggered', target_source='isaac_triggered',
        last_reject_reason='', last_rejected_source='none',
        floor_transition_active=False, floor_runtime_context_valid=True,
    )

    def arm(_request, response):
        response.success = True
        return response

    def dispatch(_request, response):
        nonlocal dispatched_at, dispatch_count
        dispatch_count += 1
        dispatched_at = time.monotonic()
        return response

    services = [
        node.create_service(Trigger, '/robot_localization_bridge/force_accept_next_localization', arm),
        node.create_service(Empty, '/trigger_grid_search_localization', dispatch),
    ]

    def publish():
        if dispatched_at is not None:
            elapsed = time.monotonic() - dispatched_at
            if case != 'no_result' and elapsed > 0.1:
                status.update(
                    accepted_result_count=1, last_explicit_relocalization_sequence=1,
                    last_explicit_map_odom_target_sequence=2,
                    last_explicit_relocalization_source='isaac_triggered',
                    last_accept_reason='EXPLICIT_TRIGGERED_RELOCALIZATION',
                    target_sequence=2, correction_active=True, safe_for_goal_start=False,
                )
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.pose.orientation.w = 1.0
                pose_pub.publish(pose)
                if case in ('amcl_reject', 'amcl_refine', 'empty_reject'):
                    status.update(rejected_result_count=1, last_rejected_source='amcl_gated',
                                  last_reject_reason='AMCL_POST_ISAAC_REFINE_INCONSISTENT')
                if case == 'empty_reject':
                    status['last_reject_reason'] = ''
                if elapsed > (2.8 if case in ('late', 'late_navigation_not_ready') else 0.65) and case != 'never_settles':
                    status.update(current_sequence=2, correction_active=False, safe_for_goal_start=True)
                    if case in ('navigation_not_ready', 'late_navigation_not_ready'):
                        status.update(safe_for_goal_start=False,
                                      floor_transition_active=True,
                                      floor_runtime_context_valid=False)
                    if case == 'amcl_refine':
                        status.update(current_sequence=3, target_sequence=3,
                                      amcl_post_isaac_refined_sequence=1,
                                      current_source='amcl_gated', target_source='amcl_gated',
                                      last_reject_reason='')
                if case == 'foreign_sequence':
                    status['last_explicit_relocalization_sequence'] = 2
                if case == 'wrong_owner':
                    status['map_to_odom_publisher_owner'] = 'foreign_bridge'
                if case == 'stale_tf':
                    status.update(map_to_odom_age_ms=5000.0, map_odom_publish_gap_ms=5000.0)
        msg = String()
        msg.data = json.dumps(status)
        status_pub.publish(msg)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        tf.sendTransform(transform)

    timer = node.create_timer(0.04, publish)
    service_type = TriggerLocalizationTracked if tracked else TriggerLocalization
    client = node.create_client(service_type, '/global_localization/trigger' + ('/tracked' if tracked else ''))
    if tracked:
        subscription = node.create_subscription(LocalizationTriggerStatus,
            '/global_localization/trigger_status', lambda msg: outcomes.append(msg), 128)
    with tempfile.TemporaryFile(mode='w+t') as log:
        process = subprocess.Popen([
            binary, '--ros-args', '-p', 'localizer_input_freshness_enabled:=false',
            '-p', 'runtime_map_context_file:=/nonexistent/isolated-trigger-test.json',
            '-p', 'service_call_timeout_sec:=1.0', '-p', 'result_wait_timeout_sec:=1.2',
            '-p', 'bridge_accept_timeout_sec:=1.2', '-p', 'map_to_odom_wait_timeout_sec:=1.0',
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            assert client.wait_for_service(timeout_sec=8), 'wrapper service did not start'
            warmup = time.monotonic() + 0.8
            while time.monotonic() < warmup:
                rclpy.spin_once(node, timeout_sec=0.02)
            request = service_type.Request()
            if tracked:
                request.request_id = 'test-' + case
            request.reason = 'isolated_confirmation_' + case
            future = client.call_async(request)
            deadline = time.monotonic() + 8
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert future.done(), 'wrapper request timed out'
            response = future.result()
            expected = case not in ('no_result', 'never_settles', 'late', 'late_navigation_not_ready',
                                    'foreign_sequence', 'wrong_owner', 'stale_tf')
            accepted = (response.status.state == LocalizationTriggerStatus.SUCCEEDED
                        if tracked else response.accepted)
            message = response.status.detail if tracked else response.message
            print(json.dumps(dict(case=case, tracked=tracked, accepted=accepted, message=message)), flush=True)
            assert accepted == expected, (case, message)
            if expected:
                assert time.monotonic() - dispatched_at >= 0.65, 'returned before settling'
            if tracked:
                if not expected:
                    assert response.status.state == LocalizationTriggerStatus.UNKNOWN
                    assert not response.status.outcome_known
                duplicate = client.call_async(request)
                until = time.monotonic() + 2
                while not duplicate.done() and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert duplicate.done()
                assert dispatch_count == 1, 'duplicate request dispatched twice'
                if not expected:
                    competing = service_type.Request()
                    competing.request_id = 'different-' + case
                    competing.reason = request.reason
                    competing_future = client.call_async(competing)
                    until = time.monotonic() + 1
                    while not competing_future.done() and time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    assert competing_future.done()
                    rejected = competing_future.result().status
                    assert rejected.outcome_known and rejected.state == LocalizationTriggerStatus.FAILED
                    assert dispatch_count == 1, 'unknown request permitted a new Isaac dispatch'
                if case in ('late', 'late_navigation_not_ready'):
                    until = time.monotonic() + 5
                    while time.monotonic() < until and not any(
                            o.request_id == request.request_id and o.outcome_known and
                            o.state == LocalizationTriggerStatus.SUCCEEDED for o in outcomes):
                        rclpy.spin_once(node, timeout_sec=0.02)
                    assert any(o.outcome_known and o.state == LocalizationTriggerStatus.SUCCEEDED
                               for o in outcomes), 'late result was never reconciled'
                    print('late completion reconciled without another dispatch', flush=True)
        finally:
            process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', required=True)
    parser.add_argument('--tracked', action='store_true')
    parser.add_argument('--case', default='amcl_reject', choices=[
        'amcl_reject', 'empty_reject', 'amcl_refine', 'success', 'navigation_not_ready',
        'no_result', 'never_settles', 'late', 'late_navigation_not_ready',
        'foreign_sequence', 'wrong_owner', 'stale_tf', 'all'])
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '217' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('requires ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1; never use the live domain')
    rclpy.init()
    try:
        cases = ['amcl_reject', 'empty_reject', 'amcl_refine', 'success',
                 'navigation_not_ready', 'no_result', 'never_settles', 'wrong_owner', 'stale_tf']
        if args.tracked:
            cases += ['late', 'late_navigation_not_ready', 'foreign_sequence']
        for selected in cases if args.case == 'all' else [args.case]:
            run(args.node, selected, args.tracked)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

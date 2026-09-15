#!/usr/bin/env python3
"""Real wrapper + fake input/Isaac/bridge; use only a network-isolated container.

Requires ROS_DOMAIN_ID=218 and ROS_LOCALHOST_ONLY=1. No live topics, hardware,
or production services are used. Discovery warmup belongs to this fixture, not
the wrapper. Header age and receipt age deliberately differ in stale cases.
"""
import argparse
import json
import os
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from isaac_ros_pointcloud_interfaces.msg import FlatScan
from rclpy.node import Node
from robot_interfaces.srv import TriggerLocalization
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import TransformBroadcaster


CASES = (
    'never_fresh', 'post_arm_stale', 'stale_then_fresh', 'future', 'healthy',
    'duplicate_stamp_after_arm', 'post_arm_recovers', 'arm_unknown',
    'header_replay_after_arm',
)


def run(binary, case, tracked=False):
    if tracked:
        from robot_interfaces.msg import LocalizationTriggerStatus
        from robot_interfaces.srv import TriggerLocalizationTracked

    node = Node('fresh_input_dispatch_fixture')
    scan_pub = node.create_publisher(FlatScan, '/flatscan', 10)
    status_pub = node.create_publisher(String, '/localization/bridge_status', 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/localization_result', 10)
    tf = TransformBroadcaster(node)
    counts = dict(arm=0, dispatch=0, stale_published=0, fresh_published=0,
                  duplicate_published=0, replay_published=0)
    timing = dict(reader_seen=None, first_fresh=None, first_arm=None,
                  first_dispatch=None, recovered_fresh=None)
    input_mode = 'fresh' if case in (
        'healthy', 'post_arm_stale', 'duplicate_stamp_after_arm',
        'post_arm_recovers', 'arm_unknown', 'header_replay_after_arm') else 'stale'
    if case == 'future':
        input_mode = 'future'
    fail_after_arm = case == 'post_arm_stale'
    frozen_stamp_ns = None
    dispatched_at = None
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
        nonlocal input_mode
        counts['arm'] += 1
        if timing['first_arm'] is None:
            timing['first_arm'] = time.monotonic()
        # Before returning the arm acknowledgement, change all subsequent input
        # headers. The wrapper must not fall through when its post-arm wait fails.
        if fail_after_arm:
            input_mode = 'stale'
        elif case == 'duplicate_stamp_after_arm':
            input_mode = 'duplicate'
        elif case == 'header_replay_after_arm':
            input_mode = 'replay'
        elif case == 'post_arm_recovers':
            input_mode = 'stale'
        elif case == 'arm_unknown':
            # Deliberately exceed the wrapper's configured 1 s arm RPC budget.
            # This delays only the fake peer in the isolated fixture. Its late
            # successful reply does not prove an Isaac computation happened.
            time.sleep(1.5)
        response.success = True
        response.message = 'isolated fake bridge armed'
        return response

    def dispatch(_request, response):
        nonlocal dispatched_at
        counts['dispatch'] += 1
        dispatched_at = time.monotonic()
        if timing['first_dispatch'] is None:
            timing['first_dispatch'] = dispatched_at
        return response

    services = [
        node.create_service(Trigger, '/robot_localization_bridge/force_accept_next_localization', arm),
        node.create_service(Empty, '/trigger_grid_search_localization', dispatch),
    ]

    def publish():
        nonlocal input_mode, frozen_stamp_ns
        now_mono = time.monotonic()
        if scan_pub.get_subscription_count() > 0 and timing['reader_seen'] is None:
            timing['reader_seen'] = now_mono
        if (case == 'stale_then_fresh' and timing['reader_seen'] is not None
                and now_mono - timing['reader_seen'] >= 0.75):
            input_mode = 'fresh'
        if (case == 'post_arm_recovers' and timing['first_arm'] is not None
                and now_mono - timing['first_arm'] >= 0.65):
            input_mode = 'fresh'
            if timing['recovered_fresh'] is None:
                timing['recovered_fresh'] = now_mono
        scan = FlatScan()
        stamp_ns = node.get_clock().now().nanoseconds
        if input_mode == 'stale':
            stamp_ns -= 4_700_000_000
            counts['stale_published'] += 1
        elif input_mode == 'future':
            stamp_ns += 4_700_000_000
        elif input_mode == 'duplicate':
            # New DDS messages / receive sequence, but only ONE new source
            # timestamp. Repeated publication cannot satisfy two good frames.
            if frozen_stamp_ns is None:
                frozen_stamp_ns = stamp_ns
            stamp_ns = frozen_stamp_ns
            counts['duplicate_published'] += 1
        elif input_mode == 'replay':
            # S, S-0.1 s, S, then repeat: a backward sample must never reset
            # the previously accepted timestamp high-water mark. All initial
            # samples are still recent, so age alone cannot reject this replay.
            if frozen_stamp_ns is None:
                frozen_stamp_ns = stamp_ns
            stamp_ns = frozen_stamp_ns
            if counts['replay_published'] % 3 == 1:
                stamp_ns -= 100_000_000
            counts['replay_published'] += 1
        else:
            counts['fresh_published'] += 1
            if timing['reader_seen'] is not None and timing['first_fresh'] is None:
                timing['first_fresh'] = now_mono
        scan.header.stamp.sec = stamp_ns // 1_000_000_000
        scan.header.stamp.nanosec = stamp_ns % 1_000_000_000
        scan.header.frame_id = 'lidar_link'
        scan.range_min = 0.1
        scan.range_max = 20.0
        scan.angles = [-2.0, -1.0, 0.0, 1.0, 2.0]
        scan.ranges = [2.0] * 5
        scan.intensities = [1.0] * 5
        scan_pub.publish(scan)

        if dispatched_at is not None and now_mono - dispatched_at > 0.15:
            status.update(
                accepted_result_count=1, last_explicit_relocalization_sequence=1,
                last_explicit_map_odom_target_sequence=2,
                last_explicit_relocalization_source='isaac_triggered',
                last_accept_reason='EXPLICIT_TRIGGERED_RELOCALIZATION',
                current_sequence=2, target_sequence=2,
                correction_active=False, safe_for_goal_start=True,
            )
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
        message = String()
        message.data = json.dumps(status)
        status_pub.publish(message)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        tf.sendTransform(transform)

    timer = node.create_timer(0.04, publish)
    service_type = TriggerLocalizationTracked if tracked else TriggerLocalization
    client = node.create_client(
        service_type, '/global_localization/trigger' + ('/tracked' if tracked else ''))
    outcomes = []
    if tracked:
        outcome_subscription = node.create_subscription(
            LocalizationTriggerStatus, '/global_localization/trigger_status',
            lambda message: outcomes.append(message), 32)

    def spin_until(predicate, seconds, failure):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate(), failure

    def call(request, seconds=8):
        future = client.call_async(request)
        spin_until(future.done, seconds, 'wrapper request timed out')
        return future.result()

    def request_for(suffix):
        request = service_type.Request()
        request.reason = 'isolated_fresh_input_' + case + '_' + suffix
        if tracked:
            request.request_id = case + '-' + suffix
        return request

    def assert_outcome(response, accepted, reconciled=False):
        if tracked:
            outcome = response.status
            if case == 'arm_unknown' and not accepted:
                if reconciled:
                    assert outcome.outcome_known, outcome
                    assert outcome.state == LocalizationTriggerStatus.FAILED, outcome
                    assert outcome.code == 'LOCALIZATION_NOT_DISPATCHED', outcome
                    return outcome.detail
                assert not outcome.outcome_known, outcome
                assert outcome.state == LocalizationTriggerStatus.UNKNOWN, outcome
                assert outcome.code == 'LOCALIZATION_OUTCOME_UNKNOWN', outcome
                assert 'BRIDGE_FORCE_ACCEPT_TIMEOUT' in outcome.detail, outcome
                return outcome.detail
            assert outcome.outcome_known, ('unexpected UNKNOWN', outcome)
            wanted = (LocalizationTriggerStatus.SUCCEEDED if accepted
                      else LocalizationTriggerStatus.FAILED)
            assert outcome.state == wanted, outcome
            if not accepted:
                assert outcome.code == 'LOCALIZATION_NOT_DISPATCHED', outcome
            detail = outcome.detail
        else:
            assert response.accepted == accepted, response.message
            detail = response.message
        if not accepted:
            assert 'dispatch_state=not_dispatched' in detail, detail
        return detail

    with tempfile.TemporaryFile(mode='w+t') as log:
        process = subprocess.Popen([
            binary, '--ros-args', '-p', 'localizer_input_freshness_enabled:=true',
            '-p', 'localizer_input_topic:=/flatscan',
            '-p', 'localizer_input_wait_timeout_sec:=3.0',
            '-p', 'localizer_input_max_age_sec:=0.5',
            '-p', 'localizer_input_required_consecutive_good:=2',
            '-p', 'runtime_map_context_file:=/nonexistent/isolated-fresh-input-test.json',
            '-p', 'service_call_timeout_sec:=1.0', '-p', 'result_wait_timeout_sec:=1.2',
            '-p', 'bridge_accept_timeout_sec:=1.2', '-p', 'map_to_odom_wait_timeout_sec:=1.0',
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            assert client.wait_for_service(timeout_sec=8), 'wrapper service did not start'
            # Warm only fake bridge/TF discovery. FlatScan reader is created lazily
            # by the real wrapper after the request, so it is not pre-populated.
            warmup = time.monotonic() + 0.6
            while time.monotonic() < warmup:
                rclpy.spin_once(node, timeout_sec=0.02)
            first_request = request_for('first')
            started = time.monotonic()
            response = call(first_request)
            successful = case in ('healthy', 'stale_then_fresh', 'post_arm_recovers')
            wanted_arms = 1 if successful or case in (
                'post_arm_stale', 'duplicate_stamp_after_arm', 'arm_unknown',
                'header_replay_after_arm') else 0
            assert counts['dispatch'] == int(successful), ('unexpected Isaac calls', counts)
            assert counts['arm'] == wanted_arms, ('unexpected bridge arm calls', counts)
            detail = assert_outcome(response, successful)
            if successful:
                assert timing['first_fresh'] is not None
                assert timing['first_arm'] >= timing['first_fresh'], timing
                # The configured input budget is 3 s; healthy input must return
                # early instead of sleeping through that budget.
                assert timing['first_dispatch'] - timing['first_fresh'] < 1.8, timing
                if case == 'post_arm_recovers':
                    assert timing['recovered_fresh'] is not None, timing
                    assert timing['first_dispatch'] >= timing['recovered_fresh'], timing
                    assert timing['recovered_fresh'] - timing['first_arm'] >= 0.65, timing
            elif case == 'arm_unknown':
                assert 'BRIDGE_FORCE_ACCEPT_TIMEOUT' in detail, detail
                assert time.monotonic() - started >= 1.0, 'did not exercise arm timeout'
            else:
                assert time.monotonic() - started >= 2.8, 'did not exercise input timeout'
                if case == 'duplicate_stamp_after_arm':
                    assert counts['duplicate_published'] >= 2, counts
                if case == 'header_replay_after_arm':
                    assert counts['replay_published'] >= 3, counts
            if tracked:
                if case == 'arm_unknown':
                    # The initial RPC timeout is still UNKNOWN. Once the original
                    # arm future returns, the existing 1 Hz reconciler can prove
                    # that no Isaac call was attempted; it must not retain an
                    # admission lock forever or send another arm/Isaac request.
                    spin_until(lambda: any(
                        outcome.request_id == first_request.request_id
                        and outcome.outcome_known
                        and outcome.state == LocalizationTriggerStatus.FAILED
                        and outcome.code == 'LOCALIZATION_NOT_DISPATCHED'
                        for outcome in outcomes), 4,
                        'late successful arm was never reconciled as not dispatched')
                    assert counts['arm'] == 1 and counts['dispatch'] == 0, counts
                duplicate = call(first_request, 2)
                assert_outcome(duplicate, successful, reconciled=case == 'arm_unknown')
                assert counts['arm'] == wanted_arms and counts['dispatch'] == int(successful)
                if case == 'post_arm_stale':
                    # Confirmed arm without any Isaac dispatch is a known failed
                    # request, not permanent UNKNOWN. Only a NEW explicit request
                    # may proceed once input has genuinely recovered.
                    input_mode = 'fresh'
                    fail_after_arm = False
                    next_response = call(request_for('fresh-explicit-request'))
                    assert_outcome(next_response, True)
                    assert counts['arm'] == 2 and counts['dispatch'] == 1, counts
                    assert_outcome(call(first_request, 2), False)
                    assert counts['arm'] == 2 and counts['dispatch'] == 1, counts
            print(json.dumps(dict(case=case, tracked=tracked, passed=True,
                                  elapsed_sec=time.monotonic() - started,
                                  counts=counts, detail=detail)), flush=True)
        except BaseException:
            log.flush()
            log.seek(0)
            print(log.read(), flush=True)
            print(json.dumps(dict(case=case, tracked=tracked, counts=counts,
                                  timing=timing)), flush=True)
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', required=True)
    parser.add_argument('--tracked', action='store_true')
    parser.add_argument('--case', choices=CASES + ('all',), default='all')
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID') != '218'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise SystemExit('requires ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1; never use live domain')
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') != 'rmw_fastrtps_cpp':
        raise SystemExit('requires rmw_fastrtps_cpp')
    rclpy.init()
    try:
        for selected in CASES if args.case == 'all' else [args.case]:
            run(args.node, selected, args.tracked)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

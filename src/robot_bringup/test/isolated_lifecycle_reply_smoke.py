"""Private real GetState regression: no sensors, TF, goals or state changes."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

parser = argparse.ArgumentParser()
parser.add_argument('--binary', required=True)
parser.add_argument('--case', choices=['delayed_active', 'inactive_then_active', 'no_response', 'shutdown'], required=True)
args = parser.parse_args()
assert os.environ['ROS_DOMAIN_ID'] == '229'
assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
assert not list(Path('/dev').glob('nvidia*'))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.srv import GetState

rclpy.init(args=[])
node = rclpy.create_node('private_lifecycle_reply_fixture', enable_rosout=False,
                         start_parameter_services=False)
executor = SingleThreadedExecutor()
executor.add_node(node)
requests = []
received = threading.Event()
thread = None
child = None
details = {'case': args.case, 'verdict': 'running'}
target = '/private_lifecycle_reply_target'

def respond(request, response):
    requests.append(time.monotonic())
    received.set()
    time.sleep(1.2)
    response.current_state.id = 2 if args.case == 'inactive_then_active' and len(requests) == 1 else 3
    response.current_state.label = 'inactive' if response.current_state.id == 2 else 'active'
    return response

server = node.create_service(GetState, target+'/get_state', respond)
try:
    # In no_response the endpoint exists, but no executor handles requests.
    if args.case != 'no_response':
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
    budget = 2 if args.case == 'no_response' else 5
    begin = time.monotonic()
    child = subprocess.Popen([args.binary, 'lifecycle-active', target, str(budget)],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if args.case == 'shutdown':
        assert received.wait(5), 'request was not received before interrupt'
        time.sleep(0.15)
        signaled = time.monotonic()
        child.send_signal(signal.SIGINT)
    output, _ = child.communicate(timeout=10)
    elapsed = time.monotonic()-begin
    details.update(rc=child.returncode, elapsed_sec=elapsed, request_count=len(requests), output=output)
    if requests:
        details['first_request_to_exit_sec'] = time.monotonic()-requests[0]
    if args.case in ('delayed_active', 'inactive_then_active'):
        assert child.returncode == 0, details
        assert len(requests) == (1 if args.case == 'delayed_active' else 2), details
        assert 'lifecycle node active:' in output, details
        if args.case == 'delayed_active':
            assert details['first_request_to_exit_sec'] < 3.5, details
    else:
        assert child.returncode == 1 and 'lifecycle node active:' not in output, details
        if args.case == 'shutdown':
            details['interrupt_to_exit_sec'] = time.monotonic()-signaled
            assert details['interrupt_to_exit_sec'] < 1.5, details
        else:
            # Includes process initialization; does not claim a precise DDS cold-start time.
            assert 1.9 <= elapsed < 6.0, details
    details['verdict'] = 'passed'
except BaseException as exc:
    details.update(verdict='failed', error=repr(exc))
    raise
finally:
    if child is not None and child.poll() is None:
        child.kill()
        child.wait(timeout=2)
    executor.shutdown(timeout_sec=2)
    if thread is not None:
        thread.join(timeout=2)
        assert not thread.is_alive(), 'private executor did not finish'
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(json.dumps(details), flush=True)

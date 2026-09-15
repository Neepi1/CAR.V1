"""Isolated startup regression: fake driver/ROS boundaries, real shell flow."""
import os
import importlib.util
from pathlib import Path
import shutil
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / 'scripts/jetson/runtime_overlay/scripts'
pytestmark = pytest.mark.skipif(os.name != 'posix', reason='Linux /proc startup ownership')


def fixture(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    for name in ('run_local_state.sh', 'canonical_tf_helpers.sh', 'common_startup_helpers.sh',
                 'local_state_startup.py'):
        if (SCRIPTS / name).exists():
            shutil.copyfile(SCRIPTS / name, scripts / name)
    (scripts / 'common_env.sh').write_text('''
runtime_readiness_probe_bin() { echo "$TEST_PROBE"; }
runtime_readiness_probe() { "$TEST_PROBE" "$@"; }
''')
    (scripts / 'runtime_health_helpers.sh').write_text('''
runtime_health_available() { return 1; }
''')
    (scripts / 'imu_pipeline_helpers.sh').write_text('''
njrh_resolve_imu_pipeline_mode() { :; }
njrh_imu_pipeline_composed() { return 1; }
''')
    (scripts / 'cpu_affinity.sh').write_text('''
njrh_start_affined_background() {
  local variable="$1"; shift 2
  "$@" & printf -v "$variable" '%s' "$!"
}
''')
    package = tmp_path / 'install/robot_local_state/lib/robot_local_state'
    package.mkdir(parents=True)
    prefix = tmp_path / 'ros'
    binary = prefix / 'lib/robot_localization/ekf_node'
    binary.parent.mkdir(parents=True)
    # Real ELF executable with inert behavior; no ROS code or input channels.
    code = tmp_path / 'fake.c'
    code.write_text('''#include <signal.h>
#include <unistd.h>
static volatile sig_atomic_t running=1;
void stop(int s){running=0;}
int main(){signal(SIGINT,stop); signal(SIGTERM,stop); while(running) usleep(10000); return 0;}
''')
    subprocess.run(['cc', str(code), '-o', str(binary)], check=True)
    shutil.copyfile(binary, package / 'local_state_node')
    shutil.copyfile(binary, package / 'imu_gyro_bias_filter_node')
    for file in package.iterdir():
        file.chmod(0o755)
    config = tmp_path / 'config'
    config.mkdir()
    for name in ('local_state_ekf_wheel_spin_imu.yaml', 'local_state_wheel_odom_ekf_spin_imu.yaml', 'local_state_imu_bias_filter.yaml'):
        (config / name).write_text('{}')
    modules = tmp_path / 'modules/ament_index_python'
    modules.mkdir(parents=True)
    (modules / '__init__.py').touch()
    (modules / 'packages.py').write_text('import os\ndef get_package_prefix(name): return os.environ["TEST_PREFIX"]\n')
    probe = tmp_path / 'probe.py'
    probe.write_text('''#!/usr/bin/env python3
import os, sys, time
command=sys.argv[1]
if command == 'imu-bias-filter':
    if os.environ.get('TEST_IMU_FAIL') == 'true':
        print('ORIGINAL_IMU_FAILURE', flush=True); sys.exit(4)
    timeout=float(sys.argv[4]); delay=float(os.environ['TEST_IMU_DELAY'])
    time.sleep(min(delay, timeout))
    if delay >= timeout:
        print('IMU gyro bias filter outputs did not become ready', flush=True); sys.exit(1)
    print('[runtime-overlay] IMU bias filter ready:', flush=True)
elif command == 'local-state-endpoint':
    time.sleep(float(os.environ.get('TEST_ENDPOINT_DELAY', '0')))
    if os.environ.get('TEST_ENDPOINT_FAIL') == 'true':
        print('ORIGINAL_ENDPOINT_FAILURE', flush=True); sys.exit(1)
    print('[runtime-overlay] robot_local_state endpoint ready', flush=True)
    time.sleep(float(os.environ.get('TEST_ENDPOINT_EXIT_DELAY', '0')))
elif command == 'fresh-tf':
    print('[runtime-overlay] fresh TF ready:', flush=True)
''')
    probe.chmod(0o755)
    env = {**os.environ, 'NJRH_PROJECT_ROOT': str(tmp_path), 'NJRH_OVERLAY_ROOT': str(tmp_path),
           'NJRH_RUNTIME_LOG_DIR': str(tmp_path / 'logs'), 'NJRH_REUSE_COMMON_SERVICES': 'false',
           'NJRH_LOCAL_STATE_START_READY_MODE': 'endpoint', 'LOCAL_STATE_MODE': 'ekf',
           'LOCAL_STATE_EKF_PROFILE': 'wheel_spin_imu', 'LOCAL_STATE_CLEAN_STALE_EKF_MODE': 'false',
           'LOCAL_STATE_STARTUP_TIMEOUT_SEC': '30', 'TEST_IMU_DELAY': '8.5',
           'NJRH_LOCAL_STATE_STARTUP_REPORT_DIR': str(tmp_path / 'reports'),
           'TEST_PREFIX': str(prefix), 'TEST_PROBE': str(probe),
           'PYTHONPATH': str(modules.parent)}
    return scripts, env


def run_start(tmp_path, scripts, env):
    harness = '''set -euo pipefail
source "$1/canonical_tf_helpers.sh"
source "$1/common_startup_helpers.sh"
trap cleanup_common_startup_helpers EXIT
start_common_canonical_helper_background robot_local_state_common bash "$1/run_local_state.sh"
wait_for_common_startup_job robot_local_state_common
echo STARTUP_READY
'''
    start = time.monotonic()
    result = subprocess.run(['bash', '-s', '--', str(scripts)], input=harness,
                            env=env, text=True, capture_output=True, timeout=45)
    return result, time.monotonic() - start


@pytest.mark.parametrize('delay', ['8.5', '12'])
def test_imu_after_old_eight_second_limit_still_starts_owned_ekf(tmp_path, delay):
    scripts, env = fixture(tmp_path)
    env['TEST_IMU_DELAY'] = delay
    result, elapsed = run_start(tmp_path, scripts, env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'STARTUP_READY' in result.stdout
    log = (tmp_path / 'logs/robot_local_state_common.log').read_text()
    assert 'starting robot_local_state EKF profile=' in log, log
    assert elapsed < 20  # Ready means finish early, not a fixed 30-second sleep.


def test_failed_attempt_log_survives_next_attempt(tmp_path):
    scripts, env = fixture(tmp_path)
    env.update(LOCAL_STATE_STARTUP_TIMEOUT_SEC='3', TEST_IMU_DELAY='60')
    failed, elapsed = run_start(tmp_path, scripts, env)
    assert failed.returncode != 0
    assert elapsed < 8
    log_path = tmp_path / 'logs/robot_local_state_common.log'
    previous = log_path.read_bytes()
    assert b'starting robot_local_state EKF profile=' not in previous
    env.update(LOCAL_STATE_STARTUP_TIMEOUT_SEC='30', TEST_IMU_DELAY='0')
    success, _ = run_start(tmp_path, scripts, env)
    assert success.returncode == 0, success.stderr
    archives = list((tmp_path / 'reports').glob('*.log'))
    assert archives and any(file.read_bytes() == previous for file in archives)


def test_child_failure_is_reported_without_waiting_outside_budget(tmp_path):
    scripts, env = fixture(tmp_path)
    env['TEST_IMU_FAIL'] = 'true'
    result, elapsed = run_start(tmp_path, scripts, env)
    assert result.returncode != 0
    assert elapsed < 8
    assert 'ORIGINAL_IMU_FAILURE' in result.stderr
    assert 'producer_exited' in result.stderr


def test_endpoint_wait_uses_remainder_not_a_new_timeout(tmp_path):
    scripts, env = fixture(tmp_path)
    env.update(LOCAL_STATE_STARTUP_TIMEOUT_SEC='4', TEST_IMU_DELAY='0', TEST_ENDPOINT_DELAY='30')
    result, elapsed = run_start(tmp_path, scripts, env)
    assert result.returncode != 0
    # The 4-second readiness budget excludes the existing INT/TERM cleanup.
    assert 4 <= elapsed < 10
    assert 'WAIT_EKF_ENDPOINT' in result.stderr


@pytest.mark.parametrize('ready_mode', ['endpoint', 'fresh_tf'])
def test_ready_receipt_does_not_wait_for_client_shutdown(tmp_path, ready_mode):
    scripts, env = fixture(tmp_path)
    env.update(TEST_IMU_DELAY='0', TEST_ENDPOINT_EXIT_DELAY='30', NJRH_LOCAL_STATE_START_READY_MODE=ready_mode)
    result, elapsed = run_start(tmp_path, scripts, env)
    assert result.returncode == 0, result.stderr
    assert elapsed < 8


def test_exact_process_check_rejects_shell_text_and_wrong_node(tmp_path):
    scripts, env = fixture(tmp_path)
    helper = scripts / 'local_state_startup.py'
    spec = importlib.util.spec_from_file_location('startup_check', helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    binary = str(Path(env['TEST_PREFIX']) / 'lib/robot_localization/ekf_node')
    with subprocess.Popen(['bash', '-c', 'sleep 2', 'robot_localization/ekf_node']) as shell:
        assert not module.ekf_process(str(shell.pid))
    for node, expected in [('wrong_node', False), ('robot_local_state', True)]:
        child = subprocess.Popen([binary, '--ros-args', '-r', f'__node:={node}', '-r', '/odometry/filtered:=/local_state/odometry'])
        try:
            time.sleep(0.1)  # allow exec/signal-handler installation in fixture
            assert module.ekf_process(str(child.pid)) is expected
            assert str(child.pid) in module.descendants(str(os.getpid()))
        finally:
            child.terminate()
            try:
                child.wait(timeout=1)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=1)
        assert not module.ekf_process(str(child.pid))


def test_foreign_ekf_cannot_satisfy_this_startup_and_duplicates_are_rejected(tmp_path):
    scripts, env = fixture(tmp_path)
    spec = importlib.util.spec_from_file_location('owned_startup', scripts / 'local_state_startup.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    binary = str(Path(env['TEST_PREFIX']) / 'lib/robot_localization/ekf_node')
    args = [binary, '--ros-args', '-r', '__node:=robot_local_state', '-r', '/odometry/filtered:=/local_state/odometry']
    children = [subprocess.Popen(args), subprocess.Popen(args), subprocess.Popen(['sleep', '5'])]
    try:
        time.sleep(0.1)
        assert module.wait_owned(str(children[2].pid), time.monotonic() + 0.2,
                                 env['TEST_PROBE'], 'endpoint', '/nonexistent-fixture-log') == 1
        assert module.wait_owned(str(os.getpid()), time.monotonic() + 1,
                                 env['TEST_PROBE'], 'endpoint', '/nonexistent-fixture-log') == 1
    finally:
        for child in children:
            child.terminate()
            try:
                child.wait(timeout=1)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=1)

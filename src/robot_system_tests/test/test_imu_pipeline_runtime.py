"""IMU ownership shell tests: all process, affinity and ROS seams are fakes."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts/jetson/runtime_overlay/scripts"
HELPER = SCRIPTS / "imu_pipeline_helpers.sh"


def shell_function(source: str, name: str) -> str:
    start = source.index(f"{name}() {{")
    return source[start:source.index("\n}\n", start) + 3]


@pytest.fixture(scope="module")
def bash():
    candidates = [Path("C:/Program Files/Git/bin/bash.exe")] if os.name == "nt" else []
    if discovered := shutil.which("bash"):
        candidates.append(Path(discovered))
    executable = next((p for p in candidates if p.is_file()), None)
    if executable is None:
        pytest.skip("Bash required for IMU startup wiring tests")
    return str(executable)


def run_shell(bash, program, **overrides):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("NJRH_", "LOCAL_STATE_")) and k not in ("BASH_ENV", "ENV")}
    env.update(TEST_HELPER=HELPER.as_posix(), TEST_REMAP_CPUS="2", TEST_FILTER_CPUS="2")
    env.update({k: str(v) for k, v in overrides.items()})
    prelude = r'''
set -euo pipefail
njrh_cpuset_for() {
  case "$1" in
    imu_axis_remap) printf '%s\n' "$TEST_REMAP_CPUS" ;;
    robot_local_state_imu_bias_filter) printf '%s\n' "$TEST_FILTER_CPUS" ;;
    *) return 90 ;;
  esac
}
source "$TEST_HELPER"
'''
    return subprocess.run([bash, "--noprofile", "--norc", "-s"], input=prelude + program,
                          env=env, capture_output=True, text=True, timeout=10)


@pytest.mark.parametrize("environment,expected,reason", [
    ({}, "intra_process", "shared_cpu_mask_2"),
    ({"NJRH_IMU_PIPELINE_MODE": "standalone"}, "standalone", "explicit_standalone"),
    ({"LOCAL_STATE_IMU_BIAS_FILTER_ENABLED": "false"}, "standalone", "imu_bias_filter_disabled"),
    ({"LOCAL_STATE_MODE": "fastlio"}, "standalone", "local_state_mode_fastlio"),
    ({"LOCAL_STATE_MODE": "passthrough"}, "standalone", "local_state_mode_passthrough"),
    ({"LOCAL_STATE_MODE": "legacy"}, "standalone", "local_state_mode_legacy"),
    ({"NJRH_NAV_LOCAL_STATE_MODE": "fastlio"}, "standalone", "local_state_mode_fastlio"),
    ({"LOCAL_STATE_MODE": "ekf", "NJRH_NAV_LOCAL_STATE_MODE": "fastlio"}, "intra_process", "shared_cpu_mask_2"),
    ({"TEST_REMAP_CPUS": "6", "TEST_FILTER_CPUS": "2"}, "standalone", "different_or_unresolved_cpu_masks"),
    ({"TEST_REMAP_CPUS": "", "TEST_FILTER_CPUS": ""}, "standalone", "different_or_unresolved_cpu_masks"),
])
def test_resolve_compatibility_without_process_or_affinity_actions(bash, environment, expected, reason):
    result = run_shell(bash, r'''
pgrep() { echo unexpected-process-read >&2; return 91; }
pkill() { echo unexpected-process-write >&2; return 92; }
taskset() { echo unexpected-affinity-write >&2; return 93; }
njrh_resolve_imu_pipeline_mode
printf '%s\n' "$NJRH_IMU_PIPELINE_EFFECTIVE_MODE"
''', **environment)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == expected
    assert reason in result.stderr
    assert "unexpected-" not in result.stderr


def test_invalid_requested_mode_is_an_actionable_configuration_error(bash):
    result = run_shell(bash, "njrh_resolve_imu_pipeline_mode\n", NJRH_IMU_PIPELINE_MODE="typo")
    assert result.returncode == 2
    assert "expected intra_process or standalone" in result.stderr


@pytest.mark.parametrize("profile,expected", [("site_default", "standalone"), ("navigation_5cpu", "intra_process")])
def test_existing_cpu_profiles_are_resolved_without_changing_their_masks(bash, profile, expected):
    result = run_shell(bash, r'''
source "$NJRH_OVERLAY_ROOT/scripts/cpu_affinity.sh"
njrh_resolve_imu_pipeline_mode
printf '%s\n' "$NJRH_IMU_PIPELINE_EFFECTIVE_MODE"
printf '%s/%s\n' "$(njrh_cpuset_for imu_axis_remap)" "$(njrh_cpuset_for robot_local_state_imu_bias_filter)"
''', NJRH_OVERLAY_ROOT=SCRIPTS.parent.as_posix(), NJRH_NAVIGATION_CPU_PROFILE=profile,
                       NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE="/nonexistent/imu_test_override.env")
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [expected, "6/2" if profile == "site_default" else "2/2"]


@pytest.mark.parametrize("mode,processes,expected", [
    ("intra_process", "/install/robot_bringup/lib/robot_bringup/imu_pipeline_node --remap-params /r --filter-params /f", True),
    ("standalone", "/install/robot_hesai_jt128/lib/robot_hesai_jt128/imu_axis_remap_node --ros-args", True),
    ("intra_process", "/install/robot_hesai_jt128/lib/robot_hesai_jt128/imu_axis_remap_node --ros-args", False),
    ("standalone", "/install/robot_bringup/lib/robot_bringup/imu_pipeline_node --remap-params /r --filter-params /f", False),
    ("intra_process", "/bin/imu_pipeline_node --remap-params /r\n/bin/imu_axis_remap_node --ros-args", False),
    ("standalone", "/bin/imu_pipeline_node --remap-params /r\n/bin/imu_axis_remap_node --ros-args", False),
    ("intra_process", "/bin/imu_pipeline_node_other --ros-args", False),
])
def test_repeated_ingress_checks_require_expected_owner_without_mixed_topology(bash, mode, processes, expected):
    result = run_shell(bash, r'''
pgrep() {
  [[ "$1" == -f ]] || return 94
  local command
  while IFS= read -r command; do
    if [[ "$command" =~ $2 ]]; then return 0; fi
  done <<< "$TEST_PROCESSES"
  return 1
}
njrh_resolve_imu_pipeline_mode
for unused in 1 2; do
  if njrh_expected_imu_ingress_running; then echo complete; else echo incomplete; fi
done
''', NJRH_IMU_PIPELINE_MODE=mode, TEST_PROCESSES=processes)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ["complete" if expected else "incomplete"] * 2


def fake_executable(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/usr/bin/env bash\nexit 99\n", encoding="utf-8", newline="\n")
    path.chmod(0o755)
    return path.as_posix()


@pytest.mark.parametrize("mode,old_filter", [("intra_process", False), ("intra_process", True), ("standalone", True)])
def test_driver_starts_only_selected_owner_with_original_affinity_role(bash, tmp_path, mode, old_filter):
    driver = (SCRIPTS / "run_driver.sh").read_text(encoding="utf-8")
    remap = fake_executable(tmp_path / "imu_axis_remap_node")
    host = fake_executable(tmp_path / "imu_pipeline_node")
    params = tmp_path / "params with spaces.yaml"
    params.write_text("{}\n", encoding="utf-8")
    program = shell_function(driver, "start_canonical_imu_ingress") + r'''
njrh_start_affined_background() { printf 'START'; printf ' <%s>' "$@"; printf '\n'; printf -v "$1" 4242; }
pgrep() { [[ "$TEST_OLD_FILTER" == true && "$2" == "$NJRH_IMU_STANDALONE_FILTER_PATTERN" ]]; }
pkill() { printf 'KILL <%s> <%s> <%s>\n' "$1" "$2" "$3"; }
sleep() { :; }
njrh_resolve_imu_pipeline_mode
imu_remap_pid=""
start_canonical_imu_ingress
printf 'OWNER_PID=%s\n' "$imu_remap_pid"
'''
    result = run_shell(bash, program, NJRH_IMU_PIPELINE_MODE=mode,
                       IMU_REMAP_CONFIG=params.as_posix(), IMU_REMAP_CPP_BIN=remap,
                       NJRH_IMU_PIPELINE_CPP_BIN=host,
                       TEST_OLD_FILTER="true" if old_filter else "false",
                       LOCAL_STATE_IMU_BIAS_FILTER_PARAMS_FILE=params.as_posix())
    assert result.returncode == 0, result.stderr
    assert result.stdout.count("START") == 1
    assert "<imu_remap_pid> <imu_axis_remap>" in result.stdout
    assert "OWNER_PID=4242" in result.stdout
    if mode == "intra_process" and old_filter:
        assert result.stdout.count("KILL") == 3  # Includes the -KILL signal token.
        assert "KILL <-INT> <-f> <[/]imu_gyro_bias_filter_node([[:space:]]|$)>" in result.stdout
        assert "KILL <-KILL> <-f> <[/]imu_gyro_bias_filter_node([[:space:]]|$)>" in result.stdout
    else:
        assert "KILL" not in result.stdout
    if mode == "intra_process":
        assert f"<{host}> <--remap-params> <{params.as_posix()}> <--filter-params> <{params.as_posix()}>" in result.stdout
        assert f"<{remap}>" not in result.stdout
    else:
        assert f"<{remap}> <--ros-args> <--params-file> <{params.as_posix()}>" in result.stdout
        assert f"<{host}>" not in result.stdout


@pytest.mark.parametrize("mode", ["intra_process", "standalone"])
def test_local_state_only_starts_standalone_filter_and_keeps_original_ready_probe(bash, tmp_path, mode):
    source = (SCRIPTS / "run_local_state.sh").read_text(encoding="utf-8")
    start = source.index('if [[ "${LOCAL_STATE_IMU_BIAS_FILTER_ENABLED}" == "true" ]]; then\n  if njrh_imu_pipeline_composed; then')
    end = source.index('\nif [[ "${EKF_USES_IMU}" != "true"', start)
    fake_executable(tmp_path / "install/robot_local_state/lib/robot_local_state/imu_gyro_bias_filter_node")
    program = r'''
njrh_start_affined_background() { echo "START $1 $2"; printf -v "$1" 4242; }
runtime_readiness_probe() { printf 'READY'; printf ' <%s>' "$@"; printf '\n'; }
sleep() { :; }
kill() { [[ "$1" == -0 && "$2" == 4242 ]]; }
njrh_resolve_imu_pipeline_mode
LOCAL_STATE_IMU_BIAS_FILTER_ENABLED=true
IMU_BIAS_FILTER_PARAMS_FILE=/unused_params.yaml
imu_bias_pid=""
''' + source[start:end] + '\nprintf "FILTER_PID=%s\\n" "$imu_bias_pid"\n'
    result = run_shell(bash, program, NJRH_IMU_PIPELINE_MODE=mode, NJRH_PROJECT_ROOT=tmp_path.as_posix())
    assert result.returncode == 0, result.stderr
    assert "READY <imu-bias-filter> </lidar_imu_bias_corrected> </local_state/imu_bias> <8>" in result.stdout
    if mode == "intra_process":
        assert "START" not in result.stdout
        assert "FILTER_PID=\n" in result.stdout
    else:
        assert "START imu_bias_pid robot_local_state_imu_bias_filter" in result.stdout
        assert "FILTER_PID=4242" in result.stdout


def test_local_state_and_canonical_cleanup_do_not_match_composed_host(bash):
    local = (SCRIPTS / "run_local_state.sh").read_text(encoding="utf-8")
    canonical = (SCRIPTS / "canonical_tf_helpers.sh").read_text(encoding="utf-8")
    program = "\n".join([
        shell_function(local, "cleanup_stale_ekf_mode_processes"),
        shell_function(local, "cleanup_ekf_mode"),
        shell_function(canonical, "canonical_helper_process_pattern"),
    ]) + r'''
host_command='/work/install/robot_bringup/lib/robot_bringup/imu_pipeline_node --remap-params /config/jt128_canonical_imu_remap.yaml --filter-params /config/local_state_imu_bias_filter.yaml'
pkill() { [[ ! "$host_command" =~ ${@: -1} ]] || { echo HOST_MATCH; return 96; }; }
sleep() { :; }
terminate_child() { printf 'CHILD <%s> <%s>\n' "$1" "$2"; }
cleanup_stale_ekf_mode_processes
ekf_pid=100; wheel_odom_pid=101; imu_bias_pid=''
cleanup_ekf_mode
pattern="$(canonical_helper_process_pattern robot_local_state_common)"
[[ ! "$host_command" =~ $pattern ]] || { echo HOST_MATCH; exit 97; }
'''
    result = run_shell(bash, program, NJRH_OVERLAY_ROOT="/overlay")
    assert result.returncode == 0, result.stderr
    assert "HOST_MATCH" not in result.stdout
    assert "CHILD <100>" in result.stdout and "CHILD <101>" in result.stdout
    assert "CHILD <> <IMU gyro bias filter>" in result.stdout


def test_whole_runtime_cleanup_matches_host_but_not_other_named_process(bash):
    source = (SCRIPTS / "runtime_process_patterns.sh").read_text(encoding="utf-8")
    result = run_shell(bash, source + r'''
[[ '/install/robot_bringup/lib/robot_bringup/imu_pipeline_node --remap-params /r' =~ $NJRH_RUNTIME_ALL_PATTERN ]]
[[ ! '/install/robot_bringup/lib/robot_bringup/imu_pipeline_node_other --ros-args' =~ $NJRH_RUNTIME_ALL_PATTERN ]]
''')
    assert result.returncode == 0, result.stderr


def test_both_driver_ingress_paths_and_common_share_mode_contract():
    driver = (SCRIPTS / "run_driver.sh").read_text(encoding="utf-8")
    common = (SCRIPTS / "run_common_services.sh").read_text(encoding="utf-8")
    assert driver.count("\n  start_canonical_imu_ingress\n") == 1
    assert driver.count("\nstart_canonical_imu_ingress\n") == 1
    assert "njrh_expected_imu_ingress_running" in shell_function(driver, "jt128_imu_remap_running")
    assert "njrh_any_imu_ingress_running" in shell_function(driver, "any_jt128_ingress_process_running")
    assert "NJRH_IMU_PIPELINE_PROCESS_PATTERN" in shell_function(driver, "stop_jt128_ingress_processes")
    assert "njrh_expected_imu_ingress_running" in shell_function(common, "canonical_jt128_ingress_running")
    assert 'njrh_resolve_imu_pipeline_mode "${NAV_LOCAL_STATE_MODE}"' in common
    assert 'env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_driver.sh"' in common
    assert 'env LOCAL_STATE_MODE="${NAV_LOCAL_STATE_MODE}" bash "${SCRIPT_DIR}/run_pointcloud_accel_pipeline.sh"' in common

"""Actual AMCL shell orchestration, with only the ROS observation boundary faked."""

from test_amcl_startup_sequence import run_shell


def test_cold_preparation_uses_one_observation_and_actual_scan_frame(tmp_path):
    body = r'''
runtime_readiness_probe() {
  echo "observer:$*" >> events
  printf 'AMCL_INPUT_FRAME=observed_lidar\n'
  for phase in MAP SCAN MAP_TF ODOM_TF SENSOR_TF; do
    printf 'AMCL_INPUT_READY=%s\n' "$phase"
  done
}
amcl_progress_load
wait_for_amcl_tf_warmup true
[[ "$AMCL_PROGRESS_SCAN_FRAME" == observed_lidar ]]
wait_for_amcl_tf_warmup true
'''
    run, events = run_shell(tmp_path, body, functions=("wait_for_amcl_tf_warmup",))
    assert run.returncode == 0, run.stdout + run.stderr
    observers = [event for event in events if event.startswith("observer:")]
    assert len(observers) == 1, events
    assert not any(event in {"map", "scan"} or event.startswith("tf:")
                   for event in events), events


def test_partial_timeout_resumes_only_missing_conditions_in_new_shell(tmp_path):
    body = r'''
runtime_readiness_probe() {
  echo "observer:$5:frame=$4" >> events
  if [[ ! -f partial ]]; then
    touch partial
    printf 'AMCL_INPUT_FRAME=actual_lidar\nAMCL_INPUT_READY=MAP\nAMCL_INPUT_READY=SCAN\n'
    return 124
  fi
  [[ "$5" == MAP_TF,ODOM_TF,SENSOR_TF && "$4" == actual_lidar ]] || return 97
  for phase in MAP_TF ODOM_TF SENSOR_TF; do printf 'AMCL_INPUT_READY=%s\n' "$phase"; done
}
amcl_progress_load
if [[ "${1:-}" != child ]]; then
  if wait_for_amcl_tf_warmup true; then exit 98; fi
  [[ "${AMCL_PROGRESS_WARMUP:-false}" == false ]]
  bash "$0" child
else
  wait_for_amcl_tf_warmup true
fi
'''
    run, events = run_shell(tmp_path, body, functions=("wait_for_amcl_tf_warmup",))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == [
        "observer:MAP,SCAN,MAP_TF,ODOM_TF,SENSOR_TF:frame=-",
        "observer:MAP_TF,ODOM_TF,SENSOR_TF:frame=actual_lidar",
    ]


def test_success_exit_without_all_required_evidence_cannot_complete(tmp_path):
    body = r'''
runtime_readiness_probe() {
  printf 'AMCL_INPUT_READY=MAP\nAMCL_INPUT_READY=SCAN\nAMCL_INPUT_READY=MAP_TF\n'
  printf 'AMCL_INPUT_READY=WARMUP\nAMCL_INPUT_READY=SEED\n'
}
amcl_progress_load
if wait_for_amcl_tf_warmup true; then exit 99; fi
[[ "${AMCL_PROGRESS_WARMUP:-false}" == false ]]
[[ "${AMCL_PROGRESS_SEED:-false}" == false ]]
'''
    run, _ = run_shell(tmp_path, body, functions=("wait_for_amcl_tf_warmup",))
    assert run.returncode == 0, run.stdout + run.stderr


def test_foreground_tracking_mode_keeps_its_bounded_pending_retry(tmp_path):
    body = r'''
NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true
NJRH_AMCL_READINESS_COMPLETION_TIMEOUT_SEC=3
NJRH_AMCL_READINESS_COMPLETION_RETRY_SEC=0.01
floor_handoff_guard() { return 0; }
amcl_mode_for_navigation() { printf 'gated\n'; }
log_amcl_runtime_status() { :; }
bash() {
  echo runner >> events
  if [[ ! -f first_pending ]]; then touch first_pending; return 26; fi
  return 0
}
complete_amcl_readiness_with_retries_for_navigation
'''
    run, events = run_shell(tmp_path, body, filename="run_navigation_runtime_services.sh",
                           functions=("run_amcl_localization_step",
                                      "complete_amcl_readiness_if_enabled_for_navigation",
                                      "complete_amcl_readiness_with_retries_for_navigation"))
    assert run.returncode == 0, run.stdout + run.stderr
    assert events == ["runner", "runner"]

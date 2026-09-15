"""Startup scheduling regression tests; fake processes, no ROS or robot access."""

import os
from pathlib import Path
import re
import subprocess
import http.server
import sys
import threading

import pytest

from test_common_startup_parallel import SCRIPTS, run_startup
from test_production_runtime_shell import _bash_executable, _bash_path


def function(source, name):
    return re.search(rf"(?ms)^{name}\(\) \{{\n.*?^\}}\n", source).group(0)


@pytest.mark.parametrize("managed,expected", [("true", False), ("false", True)])
def test_nav2_cannot_compete_with_common_for_helper_creation(tmp_path, managed, expected):
    source = (SCRIPTS / "run_nav2_navigation.sh").read_text(encoding="utf-8")
    body = f'NJRH_COMMON_SERVICES_MANAGED={managed}\n' + r'''
helper_pids=()
helper_process_pattern() { echo robot_safety_node; }
helper_process_running() { return 1; }
''' + function(source, "ensure_resident_overlay_helper_process") + r'''
ensure_resident_overlay_helper_process robot_safety robot_safety \
  bash -c 'touch "${NJRH_TEST_RELEASE}.spawned"'
wait
'''
    result = run_startup(tmp_path, body)
    assert result.returncode == 0, result.stdout + result.stderr
    assert (tmp_path / "release.spawned").exists() == expected


@pytest.mark.parametrize("observation_rc", [0, 17])
def test_real_common_sequence_defers_docking_and_survives_missing_observation(tmp_path, observation_rc):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    for name in ("robot_description", "ranger_chassis", "local_state",
                 "pointcloud_accel_pipeline", "orbbec_336l_depth",
                 "orbbec_docking_perception", "docking_manager",
                 "robot_api_server_supervised"):
        (scripts / f"run_{name}.sh").write_text("#!/usr/bin/env bash\nexec sleep 30\n", encoding="utf-8")
    sensors = tmp_path / "sensors.yaml"
    sensors.write_text("\n".join("docking_camera_" + x + ": 0.0"
                                for x in ("x", "y", "z", "roll", "pitch", "yaw")), encoding="utf-8")
    source = (SCRIPTS / "run_common_services.sh").read_text(encoding="utf-8")
    main = source[source.index("require_can_interface_up\n"):]
    main = main[:main.index('echo "[runtime-overlay] common services are running;')]
    definitions = source[source.index("start_common_process() {"):source.index("canonical_jt128_ingress_running() {")]
    definitions += source[source.index("start_robot_local_state_common() {"):source.index("resident_navigation_context_status() {")]
    for name in ("start_docking_common_last", "update_common_deferred_startup"):
        if f"{name}() {{" in source:
            definitions += function(source, name)
    body = f'SCRIPT_DIR="{_bash_path(scripts)}"\nROBOT_DESCRIPTION_CONFIG_FILE="{_bash_path(sensors)}"\n' + r'''
common_pids=()
NAV_LOCAL_STATE_MODE=ekf
FASTLIO_AUTOSTART=false
DOCKING_SENSOR_BACKEND=orbbec_336l
NJRH_POINTCLOUD_ACCEL_PROFILE=ipc_worker
NJRH_LOCAL_STATE_START_READY_MODE=endpoint
NJRH_RUNTIME_HEALTH_GUARD_AUTOSTART=true
NJRH_RESIDENT_NAVIGATION_EARLY_AUTOSTART=true
NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false
RESIDENT_NAVIGATION_AUTOSTART=auto
resident_navigation_autostart_started=0
resident_navigation_autostart_pid=""
api_http_ready=0
docking_startup_done=0
NJRH_NAVIGATION_STARTUP_RECEIPT="${NJRH_TEST_RELEASE}.phase"
require_can_interface_up() { :; }
stop_stale_pointcloud_accel_pipeline_processes() { :; }
stop_non_mapping_fastlio_runtime_processes() { :; }
rotate_runtime_log() { :; }
pgrep() { return 1; }
local_state_required_processes_running() { return 0; }
runtime_health_available() { return 1; }
runtime_health_check() { return 0; }
runtime_readiness_probe() { return 0; }
wait_for_runtime_health_local_state_endpoint_ready() { :; }
start_runtime_health_guard_common() { echo observer_started; }
log_common_startup_stage() { echo "stage=$1"; }
start_overlay_helper() { echo "helper=$1"; }
wait_for_robot_api_server_common_ready() { echo api_process_present; }
common_api_http_ready() { echo api_http_response; return 0; }
wait_for_resident_navigation_autostart_if_started() { :; }
start_resident_navigation_autostart_if_selected() {
  [[ "${resident_navigation_autostart_started}" == 0 ]] || return 0
  resident_navigation_autostart_started=1
  echo navigation_started
  printf 'waiting_for_localization\n' > "${NJRH_NAVIGATION_STARTUP_RECEIPT}"
}
test_cleanup() {
  for pid in "${common_pids[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  cleanup_common_startup_helpers
  for pid in "${common_pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
}
trap test_cleanup EXIT
''' + f'wait_for_fresh_header_topic_message() {{ return {observation_rc}; }}\n' + definitions + main
    result = run_startup(tmp_path, body)
    assert result.returncode == 0, result.stdout + result.stderr
    events = result.stdout.splitlines()
    assert events.index("observer_started") < events.index("stage=local_state_ready")
    assert events.index("navigation_started") < events.index("stage=local_state_ready")
    assert events.count("helper=robot_safety_common") == 1
    assert events.index("api_http_response") < events.index("stage=docking_sensor_started")
    assert events.index("navigation_started") < events.index("stage=docking_sensor_started")
    assert "stage=docking_manager_ready" in events
    assert ("stage=docking_sensor_ready" in events) == (observation_rc == 0)
    assert ("stage=docking_sensor_degraded" in events) == (observation_rc != 0)


@pytest.mark.parametrize("phase", ["ready", "waiting_for_localization", "reused", "exited", "no_map"])
def test_deferred_start_waits_for_http_and_initialization_then_runs_once(tmp_path, phase):
    source = (SCRIPTS / "run_common_services.sh").read_text(encoding="utf-8")
    body = function(source, "update_common_deferred_startup") + r'''
api_http_ready=0
docking_startup_done=0
resident_navigation_autostart_started=1
sleep 30 &
resident_navigation_autostart_pid=$!
common_startup_producers[navigation]="${resident_navigation_autostart_pid}"
NJRH_NAVIGATION_STARTUP_RECEIPT="${NJRH_TEST_RELEASE}.phase"
log_common_startup_stage() { echo "stage=$1"; }
common_api_http_ready() { [[ -f "${NJRH_TEST_RELEASE}.http" ]]; }
start_docking_common_last() { echo docking_launch; }
# A live PID, no HTTP or receipt: not ready. Malformed receipt: also not ready.
update_common_deferred_startup
printf 'garbage\n' > "${NJRH_NAVIGATION_STARTUP_RECEIPT}"
touch "${NJRH_TEST_RELEASE}.http"
update_common_deferred_startup
echo initialization_pending
'''
    if phase == "exited":
        body += 'kill -TERM "${resident_navigation_autostart_pid}"\nwait "${resident_navigation_autostart_pid}" || true\n'
    elif phase == "no_map":
        body += "resident_navigation_autostart_started=0\n"
    else:
        body += f'printf "{phase}\\n" > "${{NJRH_NAVIGATION_STARTUP_RECEIPT}}"\n'
    body += "update_common_deferred_startup\nupdate_common_deferred_startup\n"
    result = run_startup(tmp_path, body)
    assert result.returncode == 0, result.stdout + result.stderr
    events = result.stdout.splitlines()
    assert events.count("docking_launch") == 1
    assert events.index("initialization_pending") < events.index("docking_launch")
    assert events.count("stage=robot_api_server_ready") == 1


@pytest.mark.parametrize("status,payload,expected", [
    (200, b'{"ok":true,"endpoints":["GET /api/v1/navigation/state"]}', 0),
    (401, b'{"ok":false}', 1),
    (200, b'not json', 1),
    (200, b'{"ok":true}', 1),
])
def test_http_ready_requires_real_api_metadata(tmp_path, status, payload, expected):
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            assert self.path == "/api/v1/openapi"
            assert self.headers["X-Robot-Token"] == "isolated-test-token"
            self.send_response(status)
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        result = run_startup(tmp_path, f'''
export ROBOT_API_SERVER_PORT={server.server_port}
export ROBOT_API_TOKEN=isolated-test-token
python3() {{ "{_bash_path(Path(sys.executable))}" "$@"; }}
common_api_http_ready
''')
        assert result.returncode == expected, result.stdout + result.stderr
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)


@pytest.mark.parametrize("managed,expected", [("true", "docking_manager_start_command:=''"), ("false", "")])
def test_common_api_disables_only_on_demand_manager_creation(tmp_path, managed, expected):
    source = (SCRIPTS / "run_robot_api_server.sh").read_text(encoding="utf-8")
    arguments = source[source.index("docking_backend_args=()"):source.index('case "${DOCKING_SENSOR_BACKEND}" in')]
    result = run_startup(tmp_path, f"NJRH_COMMON_SERVICES_MANAGED={managed}\n" + arguments +
                         '\nprintf "%s\\n" "${docking_backend_args[@]}"\n')
    assert result.returncode == 0, result.stdout + result.stderr
    assert expected in result.stdout if expected else result.stdout.strip() == ""

"""Exercise the production CPU-profile resolver; never start or pin a robot."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
OVERLAY = ROOT / "scripts/jetson/runtime_overlay"


NAV_SERVICES = """
system base_control tf_state nav_control nav_planning nav_supervision
lidar_perception lidar_driver lidar_startup lidar_pipeline localization navigation_runtime_owner
nav2_controller_current nav2_controller_wide robot_api_server runtime_health_guard
ranger_base_node robot_safety velocity_smoother ranger_mini3_mode_controller
docking_manager docking_camera docking_vision robot_local_state
robot_local_state_odom_preprocessor robot_local_state_imu_bias_filter
robot_localization_bridge controller_server collision_monitor local_costmap
hesai_ros_driver pointcloud_axis_remap pointcloud_accel_container
pointcloud_accel_local_worker pointcloud_accel_scan_worker nitros_pointcloud_container
pointcloud_perception_pipeline pointcloud_downsample imu_axis_remap
nav_cloud_preprocessor robot_local_perception occupancy_grid_localizer
robot_global_localization laser_scan_to_flatscan pointcloud_to_laserscan
scan_republisher amcl amcl_scan_admission planner_server bt_navigator
behavior_server smoother_server waypoint_follower nav2_map_server nav2_lifecycle_manager
""".split()
NON_NAV_SERVICES = """
arm_control arm_planning custom_unrelated
""".split()
MAPPING_SERVICES = """
mapping_frontend mapping_backend fastlio_mapping
fastlio_odom_bridge fastlio_deskew slam_toolbox_mapping pgo_mapping
pointcloud_fastlio_remap mapping_lidar_rps_xps
""".split()
EXPECTED_MAPPING_CPU = {
    name: ("1-4" if name in ("fastlio_mapping", "fastlio_deskew") else
           "2" if name == "fastlio_odom_bridge" else
           "1,4" if name == "pointcloud_fastlio_remap" else
           "4" if name == "mapping_lidar_rps_xps" else "0-1,4")
    for name in MAPPING_SERVICES
}
FIVE_CPU_GROUPS = {
    "1": """base_control ranger_base_node robot_safety velocity_smoother
        ranger_mini3_mode_controller docking_manager collision_monitor""".split(),
    "2": """tf_state robot_local_state robot_local_state_imu_bias_filter
        robot_localization_bridge imu_axis_remap robot_local_state_odom_preprocessor""".split(),
    "3": "lidar_driver hesai_ros_driver lidar_startup".split(),
    "1-3": "nav2_controller_current nav2_controller_wide controller_server local_costmap".split(),
    "1,4": """lidar_perception lidar_pipeline pointcloud_axis_remap
        pointcloud_accel_container pointcloud_accel_local_worker pointcloud_accel_scan_worker
        nitros_pointcloud_container pointcloud_perception_pipeline pointcloud_downsample
        nav_cloud_preprocessor robot_local_perception laser_scan_to_flatscan
        pointcloud_to_laserscan scan_republisher""".split(),
    "0-1,4": """system nav_supervision robot_api_server runtime_health_guard
        docking_camera nav2_map_server nav2_lifecycle_manager
        navigation_runtime_owner nav_control nav_planning localization
        docking_vision occupancy_grid_localizer robot_global_localization
        amcl amcl_scan_admission
        planner_server bt_navigator behavior_server smoother_server waypoint_follower""".split(),
}
EXPECTED_FIVE_CPU = {name: cpus for cpus, names in FIVE_CPU_GROUPS.items() for name in names}


def assert_grouped_profile(values):
    assert set(EXPECTED_FIVE_CPU) == set(NAV_SERVICES)
    assert sum(map(len, FIVE_CPU_GROUPS.values())) == len(NAV_SERVICES)
    assert {name: values[name] for name in NAV_SERVICES} == EXPECTED_FIVE_CPU
    assert {name: values[name] for name in MAPPING_SERVICES} == EXPECTED_MAPPING_CPU


SITE_OVERRIDE = '''export NJRH_NAV2_CONTROLLER_CPU_PROFILE="control_wide"
export NJRH_CPUSET_NAV2_CONTROLLER_WIDE="3,5"
export NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER="3"
export NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION="3"
'''


def resolve_sequence(tmp_path, script, *, override=SITE_OVERRIDE, inherited=None):
    override_file = tmp_path / "cpu_affinity_runtime_override.env"
    override_file.write_text(override, encoding="utf-8")
    env = {
        key: value for key, value in os.environ.items()
        if not key.startswith("NJRH_") and key not in ("BASH_ENV", "ENV")
    }
    env.update(inherited or {})
    env.update({
        "NJRH_OVERLAY_ROOT": OVERLAY.as_posix(),
        "NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE": override_file.as_posix(),
        "TEST_AFFINITY_SOURCE": (OVERLAY / "scripts/cpu_affinity.sh").as_posix(),
    })
    bash = "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
    assert bash and Path(bash).is_file()
    emit = '''set -euo pipefail
emit() {
  local stage="$1" service
  shift
  for service in "$@"; do
    printf '%s|%s|%s\\n' "$stage" "$service" "$(njrh_cpuset_for "$service")"
  done
}
'''
    result = subprocess.run(
        [bash, "--noprofile", "--norc", "-c", emit + script, "profile-test",
         *NAV_SERVICES, *MAPPING_SERVICES, *NON_NAV_SERVICES],
        env=env, capture_output=True, text=True, timeout=15,
    )
    return result


def stages(result):
    assert result.returncode == 0, result.stdout + result.stderr
    values = {}
    for line in result.stdout.splitlines():
        stage, service, cpus = line.split("|")
        values.setdefault(stage, {})[service] = cpus
    return values


def test_five_cpu_profile_covers_scan_localization_tf_and_nav2(tmp_path):
    values = stages(resolve_sequence(tmp_path, '''
source "$TEST_AFFINITY_SOURCE"
emit five "$@"
''', inherited={"NJRH_NAVIGATION_CPU_PROFILE": "navigation_5cpu"}))
    assert_grouped_profile(values["five"])


def test_only_controller_group_gains_state_and_driver_cores(tmp_path):
    values = stages(resolve_sequence(tmp_path, '''
source "$TEST_AFFINITY_SOURCE"
emit five "$@"
''', inherited={"NJRH_NAVIGATION_CPU_PROFILE": "navigation_5cpu"}))["five"]
    for name in NAV_SERVICES:
        cpus = set()
        for part in values[name].split(','):
            bounds = list(map(int, part.split('-')))
            cpus.update(range(bounds[0], bounds[-1] + 1))
        if name in FIVE_CPU_GROUPS["3"]:
            assert cpus == {3}, name
        elif name in FIVE_CPU_GROUPS["1,4"]:
            assert cpus == {1, 4}, name
        elif name in FIVE_CPU_GROUPS["2"]:
            assert cpus == {2}, name
        elif name in FIVE_CPU_GROUPS["1-3"]:
            assert cpus == {1, 2, 3}, name
        else:
            assert cpus <= {0, 1, 4}, name


def test_switch_back_to_site_default_restores_current_override(tmp_path):
    result = resolve_sequence(tmp_path, '''
source "$TEST_AFFINITY_SOURCE"
emit baseline "$@"
export NJRH_NAVIGATION_CPU_PROFILE=navigation_5cpu
source "$TEST_AFFINITY_SOURCE"
emit five "$@"
export NJRH_NAVIGATION_CPU_PROFILE=site_default
source "$TEST_AFFINITY_SOURCE"
emit restored "$@"
''', inherited={"NJRH_CPUSET_CUSTOM_UNRELATED": "7"})
    values = stages(result)
    assert values["restored"] == values["baseline"]
    assert_grouped_profile(values["five"])
    for name in NON_NAV_SERVICES:
        assert values["five"][name] == values["baseline"][name], name


@pytest.mark.parametrize("controller_profile", ["current", "control_wide"])
def test_final_profile_wins_over_inherited_values_and_controller_resolver(
    tmp_path, controller_profile
):
    values = stages(resolve_sequence(tmp_path, '''
source "$TEST_AFFINITY_SOURCE"
njrh_resolve_nav2_controller_cpuset_profile
emit final "$@"
''', override=SITE_OVERRIDE + f'export NJRH_NAV2_CONTROLLER_CPU_PROFILE={controller_profile}\n',
        inherited={
            "NJRH_NAVIGATION_CPU_PROFILE": "navigation_5cpu",
            "NJRH_CPUSET_CONTROLLER_SERVER": "7",
            "NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER": "6",
            "NJRH_CPUSET_NAVIGATION_RUNTIME_OWNER": "0-7",
            "NJRH_CPUSET_ARM_CONTROL": "5-7",
            "NJRH_CPUSET_CUSTOM_UNRELATED": "6,7",
            **{f"NJRH_CPUSET_{name.upper()}": "7" for name in MAPPING_SERVICES},
        }))
    assert_grouped_profile(values["final"])
    assert values["final"]["arm_control"] == "5-7"
    assert values["final"]["custom_unrelated"] == "6,7"


def test_child_shells_and_repeated_loading_preserve_profile_and_baseline(tmp_path):
    values = stages(resolve_sequence(tmp_path, '''
source "$TEST_AFFINITY_SOURCE"
emit baseline "$@"
export NJRH_NAVIGATION_CPU_PROFILE=navigation_5cpu
source "$TEST_AFFINITY_SOURCE"
source "$TEST_AFFINITY_SOURCE"
export -f emit
"$BASH" --noprofile --norc -c '
  set -euo pipefail
  source "$TEST_AFFINITY_SOURCE"
  emit child_five "$@"
  export NJRH_NAVIGATION_CPU_PROFILE=site_default
  source "$TEST_AFFINITY_SOURCE"
  emit child_restored "$@"
' child "$@"
export NJRH_NAVIGATION_CPU_PROFILE=site_default
source "$TEST_AFFINITY_SOURCE"
emit parent_restored "$@"
'''))
    assert_grouped_profile(values["child_five"])
    assert values["child_restored"] == values["baseline"]
    assert values["parent_restored"] == values["baseline"]
    for name in NON_NAV_SERVICES:
        assert values["child_five"][name] == values["baseline"][name]


def test_latest_site_override_is_respected_after_restoring_snapshot(tmp_path):
    values = stages(resolve_sequence(tmp_path, '''
export NJRH_NAVIGATION_CPU_PROFILE=navigation_5cpu
source "$TEST_AFFINITY_SOURCE"
printf 'export NJRH_CPUSET_ROBOT_LOCALIZATION_BRIDGE="1"\\n' >> "$NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE"
export NJRH_NAVIGATION_CPU_PROFILE=site_default
source "$TEST_AFFINITY_SOURCE"
emit restored "$@"
'''))
    assert values["restored"]["robot_localization_bridge"] == "1"
    assert values["restored"]["controller_server"] == "3,5"


def test_unknown_profile_is_a_configuration_error(tmp_path):
    result = resolve_sequence(tmp_path, 'source "$TEST_AFFINITY_SOURCE"\n',
                              inherited={"NJRH_NAVIGATION_CPU_PROFILE": "typo"})
    assert result.returncode == 2
    assert "expected site_default|navigation_5cpu" in result.stderr


def test_resolving_profile_never_pins_or_starts_processes(tmp_path):
    result = resolve_sequence(tmp_path, '''
taskset() { echo "unexpected taskset" >&2; return 99; }
ros2() { echo "unexpected ros2" >&2; return 99; }
docker() { echo "unexpected docker" >&2; return 99; }
systemctl() { echo "unexpected systemctl" >&2; return 99; }
export NJRH_NAVIGATION_CPU_PROFILE=navigation_5cpu
source "$TEST_AFFINITY_SOURCE"
emit resolved "$@"
''')
    assert result.returncode == 0, result.stderr
    assert "unexpected" not in result.stderr


@pytest.mark.parametrize("profile, expected_calls", [("site_default", 0), ("navigation_5cpu", 1)])
def test_common_startup_pins_parent_only_for_opt_in_profile(tmp_path, profile, expected_calls):
    # Exercise the actual startup prefix up to the next unrelated profile loader.
    # Other helper sources and OS scheduling are fixture boundaries; no node runs.
    fixture_overlay = tmp_path / "overlay"
    scripts = fixture_overlay / "scripts"
    config = fixture_overlay / "config"
    scripts.mkdir(parents=True)
    config.mkdir()
    for name in ("cpu_affinity.sh", "cpu_affinity_profiles.sh", "imu_pipeline_helpers.sh"):
        shutil.copyfile(OVERLAY / "scripts" / name, scripts / name)
    shutil.copyfile(OVERLAY / "config/cpu_affinity.env", config / "cpu_affinity.env")
    for name in ("canonical_tf_helpers.sh", "common_startup_helpers.sh",
                 "nav_runtime_helpers.sh", "floor_asset_helpers.sh"):
        (scripts / name).write_text("# fixture: no ROS or system changes\n", encoding="utf-8")
    startup = (OVERLAY / "scripts/run_common_services.sh").read_text(encoding="utf-8")
    prefix = startup.split('source "${SCRIPT_DIR}/pointcloud_accel_profile.sh"', 1)[0]
    (scripts / "run_common_services.sh").write_text(prefix, encoding="utf-8")
    bash = "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
    env = {key: value for key, value in os.environ.items()
           if not key.startswith("NJRH_") and key not in ("BASH_ENV", "ENV")}
    env.update({
        "NJRH_OVERLAY_ROOT": fixture_overlay.as_posix(),
        "NJRH_NAVIGATION_CPU_PROFILE": profile,
        "TEST_STARTUP": (scripts / "run_common_services.sh").as_posix(),
        "TEST_CALLS": (tmp_path / "calls.txt").as_posix(),
    })
    result = subprocess.run([bash, "--noprofile", "--norc", "-c", '''
taskset() { printf '%s\\n' "$*" >> "$TEST_CALLS"; }
awk() { printf '0-1,4\\n'; }
source "$TEST_STARTUP"
'''], env=env, capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    calls = (tmp_path / "calls.txt").read_text().splitlines() if (tmp_path / "calls.txt").exists() else []
    assert len(calls) == expected_calls
    if calls:
        assert calls[0].startswith("-pc 0-1,4 ")


@pytest.mark.parametrize("profile", ["site_default", "navigation_5cpu"])
@pytest.mark.parametrize("service", ["jt128_driver", "pointcloud_accel_pipeline", "robot_api_server"])
def test_sensor_wrapper_affinity_applies_before_wrapper_initialization(tmp_path, profile, service):
    startup = (OVERLAY / "scripts/run_common_services.sh").read_text(encoding="utf-8")
    launcher = startup.split("start_common_process() {", 1)[1].split(
        "\nstart_orbbec_336l_depth_common()", 1)[0]
    program = '''
source "$TEST_AFFINITY_SOURCE"
reuse_common_services_enabled() { return 1; }
rotate_runtime_log() { :; }
taskset() { printf 'affinity:%s\\n' "$*"; shift 2; "$@"; }
common_pids=()
NJRH_RUNTIME_LOG_DIR="$(dirname "$NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE")"
start_common_process() {''' + launcher + f'''
start_common_process {service} unused bash -c 'echo wrapper_entered; sleep 0.4'
wait "$common_last_started_pid"
cat "$NJRH_RUNTIME_LOG_DIR/{service}.log"
'''
    result = resolve_sequence(tmp_path, program, inherited={"NJRH_NAVIGATION_CPU_PROFILE": profile})
    assert result.returncode == 0, result.stdout + result.stderr
    expected = profile == "navigation_5cpu" and service != "robot_api_server"
    assert ("affinity:" in result.stdout) is expected
    assert "wrapper_entered" in result.stdout
    if expected:
        # Both launch paths perform driver initialization before child-specific
        # affinity is applied; processing workers retain their separate masks.
        expected_cpu = "3"
        assert f"affinity:-c {expected_cpu} bash" in result.stdout
        assert result.stdout.index("affinity:") < result.stdout.index("wrapper_entered")


@pytest.mark.parametrize("profile, enabled", [
    ("site_default", "true"), ("navigation_5cpu", "true"), ("navigation_5cpu", "false")])
@pytest.mark.parametrize("probe", ["graph", "rate"])
def test_flatscan_checks_do_not_inherit_driver_core(tmp_path, profile, enabled, probe):
    source = (OVERLAY / "scripts/run_pointcloud_accel_pipeline.sh").read_text(encoding="utf-8")
    functions = "topic_publisher_count() {" + source.split("topic_publisher_count() {", 1)[1].split(
        "\nflatscan_startup_rate_confirmed()", 1)[0]
    program = '''
source "$TEST_AFFINITY_SOURCE"
taskset() { printf '%s\\n' "$*" >> "$TEST_AFFINITY_TRACE"; shift 2; "$@"; }
timeout() { [[ "$1" == --kill-after=1 ]] && shift; shift; "$@"; }
ros2() {
  if [[ "$2" == info ]]; then printf 'Publisher count: 1\\n';
  else printf 'average rate: 15.0\\n'; fi
}
FLATSCAN_GRAPH_PROBE_TIMEOUT_SEC=4
FLATSCAN_MESSAGE_CONFIRM_TIMEOUT_SEC=10
FLATSCAN_MIN_HZ=5.0
''' + functions + ("\nprintf 'count=%s\\n' \"$(topic_publisher_count /flatscan)\"\n"
                  if probe == "graph" else "\nflatscan_hz_ok\n")
    trace = tmp_path / "probe_affinity_trace.txt"
    result = resolve_sequence(tmp_path, program, inherited={
        "NJRH_NAVIGATION_CPU_PROFILE": profile, "NJRH_CPU_AFFINITY_ENABLED": enabled,
        "TEST_AFFINITY_TRACE": trace.as_posix()})
    assert result.returncode == 0, result.stdout + result.stderr
    expected = profile == "navigation_5cpu" and enabled == "true"
    calls = trace.read_text() if trace.exists() else ""
    assert ("-c 0-1,4 timeout --kill-after=1" in calls) is expected
    if probe == "graph":
        assert result.stdout.strip() == "count=1"
    else:
        assert "/flatscan ready: hz=15.0 min=5.0" in result.stderr


@pytest.mark.skipif(os.name == "nt", reason="The real Nav2 guard checks Linux /proc PID existence")
@pytest.mark.parametrize("controller_profile", ["current", "control_wide"])
def test_actual_nav2_affinity_guard_accepts_kernel_range_notation(tmp_path, controller_profile):
    source = (OVERLAY / "scripts/run_nav2_navigation.sh").read_text(encoding="utf-8")
    guard = source[source.index("controller_threads_match_cpuset() {"):
                   source.index("nav2_external_lifecycle_bringup_enabled() {")]
    result = resolve_sequence(tmp_path, '''
source "$TEST_AFFINITY_SOURCE"
# Only inventory is mocked. Run the real guard and per-thread comparison.
controller_server_pids() { printf '%s\\n' "$$"; }
read_proc_cpuset() { printf '1-3\\n'; }
awk() { printf '1-3\\n'; }
NJRH_NAV2_CONTROLLER_AFFINITY_CHECK_TIMEOUT_SEC=0
''' + guard + '\nwait_for_controller_server_affinity\n',
        inherited={"NJRH_NAVIGATION_CPU_PROFILE": "navigation_5cpu"},
        override=SITE_OVERRIDE + f'export NJRH_NAV2_CONTROLLER_CPU_PROFILE={controller_profile}\n')
    assert result.returncode == 0, result.stdout + result.stderr
    assert "allowed=1-3" in result.stderr


def test_all_five_core_masks_use_canonical_kernel_range_notation(tmp_path):
    values = stages(resolve_sequence(tmp_path, 'source "$TEST_AFFINITY_SOURCE"\nemit five "$@"\n',
                    inherited={"NJRH_NAVIGATION_CPU_PROFILE": "navigation_5cpu"}))["five"]
    for name in NAV_SERVICES + MAPPING_SERVICES:
        cpus = set()
        for part in values[name].split(','):
            bounds = list(map(int, part.split('-')))
            cpus.update(range(bounds[0], bounds[-1] + 1))
        groups = []
        for cpu in sorted(cpus):
            if groups and cpu == groups[-1][-1] + 1:
                groups[-1].append(cpu)
            else:
                groups.append([cpu])
        canonical = ','.join(str(g[0]) if len(g) == 1 else f'{g[0]}-{g[-1]}' for g in groups)
        assert values[name] == canonical, name


@pytest.mark.parametrize("profile,enabled", [
    ("navigation_5cpu", "true"), ("navigation_5cpu", "false"),
    ("site_default", "true"),
])
def test_projected_mapping_prefix_resolves_old_mode_overrides_before_launch(tmp_path, profile, enabled):
    scripts = tmp_path / "overlay/scripts"
    config = tmp_path / "overlay/config"
    scripts.mkdir(parents=True)
    config.mkdir()
    for name in ("cpu_affinity.sh", "cpu_affinity_profiles.sh"):
        shutil.copyfile(OVERLAY / "scripts" / name, scripts / name)
    shutil.copyfile(OVERLAY / "config/cpu_affinity.env", config / "cpu_affinity.env")
    for name in ("canonical_tf_helpers.sh", "scan_ownership_helpers.sh", "mapping_fastdds_transport.sh"):
        (scripts / name).write_text("# inert test fixture\n", encoding="utf-8")
    prefix = (OVERLAY / "scripts/run_projected_map.sh").read_text(encoding="utf-8").split(
        "log_mapping_startup_stage() {", 1)[0]
    entry = scripts / "mapping_prefix.sh"
    entry.write_text(prefix + '''
printf 'selected|fastlio|%s\\n' "$NJRH_CPUSET_FASTLIO_DESKEW"
printf 'selected|slam|%s\\n' "$NJRH_CPUSET_SLAM_TOOLBOX_MAPPING"
printf 'selected|rps|%s\\n' "$SLAM2D_LIDAR_RPS_XPS_CPUSET"
for role in fastlio_deskew fastlio_mapping fastlio_odom_bridge pgo_mapping mapping_frontend; do
  if njrh_startup_cpu_session_enabled "$role"; then exit 91; fi
done
''', encoding="utf-8")
    trace = tmp_path / "affinity_calls"
    result = resolve_sequence(tmp_path, '''
taskset() { printf '%s\\n' "$*" >> "$TEST_CALLS"; }
awk() { printf '0-1,4\\n'; }
source "$TEST_MAPPING_PREFIX"
''', inherited={
        "NJRH_OVERLAY_ROOT": scripts.parent.as_posix(),
        "NJRH_UPSTREAM_ROOT": tmp_path.as_posix(),
        "NJRH_NAVIGATION_CPU_PROFILE": profile,
        "NJRH_CPU_AFFINITY_ENABLED": enabled,
        "NJRH_STARTUP_CPU_SESSION": "inherited-cold-navigation-session",
        "NJRH_SLAM2D_FASTLIO_CPUSET": "7",
        "NJRH_SLAM2D_SLAM_TOOLBOX_CPUSET": "3,7",
        "NJRH_SLAM2D_LIDAR_RPS_XPS_CPUSET": "5",
        "TEST_MAPPING_PREFIX": entry.as_posix(), "TEST_CALLS": trace.as_posix(),
    })
    values = stages(result)["selected"]
    assert values == ({"fastlio": "1-4", "slam": "0-1,4", "rps": "4"}
                      if profile == "navigation_5cpu" else
                      {"fastlio": "7", "slam": "3,7", "rps": "5"})
    calls = trace.read_text() if trace.exists() else ""
    assert bool(calls) == (profile == "navigation_5cpu" and enabled == "true")
    if calls:
        assert calls.startswith("-pc 0-1,4 ")

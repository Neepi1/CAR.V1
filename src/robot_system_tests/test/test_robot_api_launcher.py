"""Execute the real API launcher with isolated installation/OS adapters.

The real common_env and affinity implementation run unchanged. Only installed
setup files, colcon, ros2, taskset and the API executable are fixture adapters.
No ROS participant, supervisor restart or live process inspection is involved.
"""

import os
from pathlib import Path
import subprocess

import pytest

from test_production_runtime_shell import _bash_executable, _bash_path as to_bash_path


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts/jetson/runtime_overlay/scripts"


def run_launcher(tmp_path, *, entry="inherited", installed=True, build_rc=0):
    root_text, scripts_text = to_bash_path(tmp_path), to_bash_path(SCRIPTS)

    def _bash_path(path):
        # Convert the two roots once, not one cygpath subprocess per fixture
        # file. Every mutable path below is relative to pytest's own root.
        return scripts_text if path == SCRIPTS else root_text + "/" + path.relative_to(tmp_path).as_posix()

    project = tmp_path / "project with spaces"
    install = project / "install"
    binary = install / "robot_api_server/lib/robot_api_server/robot_api_server_node"
    binary.parent.mkdir(parents=True)
    for name in ("setup.bash", "local_setup.bash"):
        (install / name).write_text("# isolated installed setup adapter\n", encoding="utf-8")
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    api_template = tmp_path / "api-template"
    api_template.write_text(r'''#!/usr/bin/env bash
set -euo pipefail
printf 'api-start\n' >> "$TEST_EVENTS"
printf '%s\0' "$@" > "$TEST_ARGV"
[[ "${ROS_DISTRO:-}" == humble && "${ROS_VERSION:-}" == 2 ]] || exit 94
case ":${AMENT_PREFIX_PATH:-}:" in
  *":$TEST_PROJECT/install/robot_api_server:"*) ;;
  *) exit 95 ;;
esac
# The installed API links robot_interfaces typesupport (CMakeLists.txt and
# the captured API ldd report); its dependency path is separate in an
# isolated install. This is an environment adapter, not a ROS success mock.
case ":${LD_LIBRARY_PATH:-}:" in
  *":$TEST_PROJECT/install/robot_interfaces/lib:"*) ;;
  *) exit 96 ;;
esac
exit 33
''', encoding="utf-8")
    api_template.chmod(0o755)
    if installed:
        binary.write_text(api_template.read_text(encoding="utf-8"), encoding="utf-8")
        binary.chmod(0o755)
    programs = {
        "getent": r'''[[ "$1" == passwd ]] || exit 91
printf 'fixture:x:1000:1000::%s:/bin/bash\n' "$TEST_PROJECT"
''',
        "taskset": r'''[[ "$1" == -c && "$2" == 0-1,4 ]] || exit 91
echo "affinity:$2" >> "$TEST_EVENTS"
shift 2
exec "$@"
''',
        "ros2": r'''echo ros2-run >> "$TEST_EVENTS"
[[ "$1 $2 $3" == 'run robot_api_server robot_api_server_node' ]] || exit 92
shift 3
exec "$TEST_API_BINARY" "$@"
''',
        "colcon": r'''printf 'build\n' >> "$TEST_EVENTS"
printf '%s\0' "$@" > "$TEST_BUILD_ARGV"
[[ "$TEST_BUILD_RC" == 0 ]] || exit "$TEST_BUILD_RC"
cp "$TEST_API_TEMPLATE" "$TEST_API_BINARY"
chmod +x "$TEST_API_BINARY"
''',
    }
    for name, body in programs.items():
        path = fake_bin / name
        path.write_text("#!/usr/bin/env bash\nset -euo pipefail\n" + body, encoding="utf-8")
        path.chmod(0o755)
    affinity = tmp_path / "affinity.env"
    affinity.write_text("NJRH_CPU_AFFINITY_ENABLED=true\nNJRH_CPUSET_ROBOT_API_SERVER=0-1,4\n",
                        encoding="utf-8")
    adapter = tmp_path / "setup-adapter.bash"
    adapter.write_text(r'''
source() {
  case "$1" in
    /opt/ros/humble/setup.bash)
      echo "$TEST_PHASE:setup:ros" >> "$TEST_EVENTS"
      export ROS_DISTRO=humble ROS_VERSION=2
      export AMENT_PREFIX_PATH="/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
      export LD_LIBRARY_PATH="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
      ;;
    "$TEST_PROJECT/install/setup.bash"|"$TEST_PROJECT/install/local_setup.bash")
      echo "$TEST_PHASE:setup:${1##*/}" >> "$TEST_EVENTS"
      export AMENT_PREFIX_PATH="$TEST_PROJECT/install/robot_api_server:$TEST_PROJECT/install/robot_interfaces:${AMENT_PREFIX_PATH:-}"
      export LD_LIBRARY_PATH="$TEST_PROJECT/install/robot_api_server/lib:$TEST_PROJECT/install/robot_interfaces/lib:${LD_LIBRARY_PATH:-}"
      ;;
    *) builtin source "$@" ;;
  esac
}
export -f source
''', encoding="utf-8")
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(("NJRH_", "ROS_", "AMENT_", "COLCON_", "ROBOT_API_", "TEST_"))
           and key not in ("BASH_ENV", "ENV", "LD_LIBRARY_PATH")}
    env.update({
        "BASH_ENV": _bash_path(adapter),
        "PATH": _bash_path(fake_bin) + ":/usr/bin:/bin",
        "TEST_PROJECT": _bash_path(project),
        "TEST_API_BINARY": _bash_path(binary),
        "TEST_API_TEMPLATE": _bash_path(api_template),
        "TEST_BUILD_RC": str(build_rc),
        "TEST_EVENTS": _bash_path(tmp_path / "events"),
        "TEST_ARGV": _bash_path(tmp_path / "argv"),
        "TEST_BUILD_ARGV": _bash_path(tmp_path / "build-argv"),
        "TEST_PHASE": "parent",
        "TEST_SCRIPTS": _bash_path(SCRIPTS),
        "NJRH_PROJECT_ROOT": _bash_path(project),
        "NJRH_UPSTREAM_ROOT": _bash_path(tmp_path / "upstream"),
        "NJRH_FASTDDS_PROFILE_ENABLED": "false",
        "NJRH_CPU_AFFINITY_CONFIG": _bash_path(affinity),
        "NJRH_CPU_AFFINITY_RUNTIME_OVERRIDE": _bash_path(tmp_path / "absent-override"),
        "NJRH_COMMON_SERVICES_MANAGED": "true",
        "ROBOT_API_SERVER_CONFIG": _bash_path(tmp_path / "api config.yaml"),
        "ROBOT_API_SERVER_PORT": "8123",
    })
    for name in ("RUNTIME_LOG_DIR", "MAPS_DIR", "MAPS3D_DIR", "RELEASE_ASSETS_DIR", "WAYPOINTS_DIR"):
        env["NJRH_" + name] = _bash_path(tmp_path / name.lower())
    for name in ("LOCAL_STATE_EKF_PROFILE_FILE", "ISAAC_LOCALIZATION_MODE_FILE", "AMCL_LOCALIZATION_PROFILE_FILE"):
        env["NJRH_" + name] = _bash_path(tmp_path / (name.lower() + ".absent"))
    program = 'set -euo pipefail\n'
    if entry in ("inherited", "missing_overlay", "partial_dependencies"):
        # Produce receipts and ROS/overlay exports by actually running the
        # common setup, then cross a real new-shell process boundary.
        program += 'source "$TEST_SCRIPTS/common_env.sh"\n'
        program += '[[ "$NJRH_COMMON_ENV_SETUP_DONE" == 1 ]]\n'
        if entry == "missing_overlay":
            program += 'export AMENT_PREFIX_PATH=/opt/ros/humble\n'
        elif entry == "partial_dependencies":
            # Keep both ready markers, all ament prefixes, Humble libraries,
            # and the API's own library path. Only its runtime dependency is
            # lost; testing API's own prefix alone must not skip repair.
            program += 'export LD_LIBRARY_PATH="$TEST_PROJECT/install/robot_api_server/lib:/opt/ros/humble/lib"\n'
    elif entry == "marker_only":
        program += 'export NJRH_COMMON_ENV_PARENT_READY=1\n'
    elif entry != "cold":
        raise AssertionError(entry)
    program += 'export TEST_PHASE=child\nexec bash "$TEST_SCRIPTS/run_robot_api_server.sh"\n'
    run = subprocess.run([_bash_executable(), "--noprofile", "--norc", "-c", program],
                         env=env, capture_output=True, text=True, timeout=10)
    events = (tmp_path / "events").read_text().splitlines() if (tmp_path / "events").exists() else []

    def arguments(name):
        path = tmp_path / name
        return path.read_bytes().decode().rstrip("\0").split("\0") if path.exists() else []

    return run, events, arguments("argv"), arguments("build-argv"), env


@pytest.mark.parametrize("entry", ["inherited", "cold", "marker_only", "missing_overlay"])
def test_api_direct_launch_preserves_setup_provenance_and_arguments(tmp_path, entry):
    run, events, argv, build_argv, env = run_launcher(tmp_path, entry=entry)
    assert run.returncode == 33, run.stdout + run.stderr
    assert argv == [
        "--ros-args", "--params-file", env["ROBOT_API_SERVER_CONFIG"], "-p", "port:=8123",
        "-p", "docking_manager_start_command:=''",
        "-p", "docking_observation_backend:=target_observation",
        "-p", "docking_target_observation_topic:=/dock/target_observation",
        "-p", "docking_target_observation_source:=orbbec_336l_depth",
        "-p", "docking_default_dock_profile_id:=orbbec_336l_rear_charging_dock",
        "-p", "docking_default_dock_profile_type:=depth_geometry",
        "-p", "docking_default_sensor_frame:=camera336l_depth_optical_frame",
    ]
    assert build_argv == []
    assert events.count("affinity:0-1,4") == events.count("api-start") == 1
    child_setups = [event for event in events if event.startswith("child:setup:")]
    if entry == "inherited":
        assert child_setups == [], events
    else:
        # A cold shell or an incomplete marker must load both setup layers,
        # but each exactly once, before reaching the installed executable.
        assert child_setups.count("child:setup:ros") in ((0, 1) if entry == "missing_overlay" else (1,)), events
        assert sum(event.endswith((":setup.bash", ":local_setup.bash"))
                   for event in child_setups) == 1, events
    assert "ros2-run" not in events, events


def test_api_restores_missing_robot_interfaces_library_path(tmp_path):
    run, events, _, build_argv, _ = run_launcher(tmp_path, entry="partial_dependencies")
    assert run.returncode == 33, run.stdout + run.stderr + repr(events)
    assert build_argv == []
    assert [event for event in events if event.startswith("child:setup:")] == [
        "child:setup:local_setup.bash"], events
    assert events.count("api-start") == 1 and "ros2-run" not in events


@pytest.mark.parametrize("build_rc", [0, 9])
def test_api_direct_launch_retains_build_fallback_and_failure(tmp_path, build_rc):
    run, events, argv, build_argv, _ = run_launcher(tmp_path, installed=False, build_rc=build_rc)
    assert build_argv == ["build", "--packages-select", "robot_map_asset_identity",
                          "robot_interfaces", "robot_elevator_manager", "robot_api_server",
                          "--symlink-install"]
    assert events.count("build") == 1
    if build_rc:
        assert run.returncode == build_rc, run.stdout + run.stderr
        assert "api-start" not in events and argv == []
    else:
        assert run.returncode == 33, run.stdout + run.stderr
        assert events.index("build") < events.index("api-start")
        assert any(event in ("child:setup:setup.bash", "child:setup:local_setup.bash")
                   for event in events[events.index("build") + 1:]), events
        assert "ros2-run" not in events, events

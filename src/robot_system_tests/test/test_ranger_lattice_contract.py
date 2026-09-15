import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
NAV_CONFIG = ROOT / "src" / "robot_nav_config"
LATTICE_STEM = "ranger_mini3_ackermann_0p05m_0p81m_16"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def test_ranger_lattice_artifact_and_manifest_are_valid():
    lattice_dir = NAV_CONFIG / "lattice"
    config_path = lattice_dir / f"{LATTICE_STEM}.config.json"
    lattice_path = lattice_dir / f"{LATTICE_STEM}.json"
    manifest_path = lattice_dir / f"{LATTICE_STEM}.manifest.json"
    validator = NAV_CONFIG / "tools" / "validate_ranger_mini3_lattice.py"

    result = subprocess.run(
        [
            sys.executable,
            str(validator),
            "--config",
            str(config_path),
            "--lattice",
            str(lattice_path),
            "--manifest",
            str(manifest_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr

    lattice = json.loads(lattice_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    metadata = lattice["lattice_metadata"]
    assert metadata["motion_model"] == "ackermann"
    assert metadata["turning_radius"] == 0.81
    assert metadata["grid_resolution"] == 0.05
    assert metadata["num_of_headings"] == 16
    assert metadata["number_of_trajectories"] == 104
    assert manifest["artifact_sha256"] == sha256(lattice_path)
    assert manifest["motion_contract"]["allow_reverse_expansion"] is True
    assert manifest["motion_contract"]["reverse_execution_scope"] == "terminal_goal_window_only_v1"
    assert manifest["motion_contract"]["reverse_admission_distance_m"] == 0.3
    assert manifest["motion_contract"]["contains_spin_primitives"] is True
    assert manifest["motion_contract"]["contains_lateral_primitives"] is False
    assert manifest["motion_contract"]["mid_path_spin_execution"] == "not_admitted_v1"


def test_ranger_lattice_is_opt_in_and_only_overrides_planner_server():
    default_nav2 = (NAV_CONFIG / "config" / "nav2.yaml").read_text(encoding="utf-8")
    profile = (
        NAV_CONFIG / "config" / "planner_profiles" / "ranger_mini3_lattice.yaml"
    ).read_text(encoding="utf-8")
    preserve = (
        NAV_CONFIG / "config" / "planner_profiles" / "preserve_base.yaml"
    ).read_text(encoding="utf-8")
    launch = (
        ROOT / "src" / "robot_bringup" / "launch" / "standard_navigation.launch.py"
    ).read_text(encoding="utf-8")
    runtime = (
        ROOT
        / "scripts"
        / "jetson"
        / "runtime_overlay"
        / "scripts"
        / "run_nav2_navigation.sh"
    ).read_text(encoding="utf-8")
    systemd_runtime = (ROOT / "scripts" / "jetson" / "njrh_systemd_runtime.sh").read_text(
        encoding="utf-8"
    )
    lattice_bt_path = (
        NAV_CONFIG
        / "behavior_trees"
        / "navigate_to_pose_ranger_lattice.xml"
    )
    lattice_bt = lattice_bt_path.read_text(encoding="utf-8")

    assert 'plugin: "nav2_smac_planner/SmacPlanner2D"' in default_nav2
    assert "min_turning_r: 0.81" in default_nav2
    assert "resolution: 0.05" in default_nav2
    assert set(line for line in preserve.splitlines() if line.strip()) == {
        "planner_server:",
        "  ros__parameters: {}",
    }
    assert profile.startswith("planner_server:\n  ros__parameters:\n")
    assert "controller_server:" not in profile
    assert "global_costmap:" not in profile
    assert "local_costmap:" not in profile
    assert 'plugin: "robot_nav_config::RangerMini3LatticePlanner"' in profile
    assert "expected_planner_frequency: 1.0" in profile
    assert 'lattice_filepath: "__RANGER_LATTICE_FILE__"' in profile
    assert "allow_reverse_expansion: true" in profile
    assert "rotation_penalty: 3.0" in profile
    assert "direct_corridor_enabled: true" in profile
    assert "direct_corridor_minimum_length_m: 1.0" in profile
    assert "direct_corridor_goal_yaw_tolerance_rad: 0.20" in profile
    assert "direct_corridor_sample_step_m: 0.025" in profile
    assert "direct_corridor_max_cost: 0" in profile
    assert "smooth_path: true" in profile
    assert "tolerance: 1.0e-10" in profile
    assert "max_iterations: 1000" in profile
    assert "w_data: 0.2" in profile
    assert "w_smooth: 0.3" in profile
    assert "do_refinement: true" in profile
    assert "analytic_expansion_max_length: 4.05" in profile

    assert 'LaunchConfiguration("planner_profile_file")' in launch
    assert 'LaunchConfiguration("ranger_lattice_filepath")' in launch
    assert 'LaunchConfiguration("nav_to_pose_bt_xml")' in launch
    assert '"parameters"] = [configured_params, configured_planner_profile]' in launch
    assert '{"default_nav_to_pose_bt_xml": nav_to_pose_bt_xml}' in launch
    assert 'with_cpu_affinity("bt_navigator", bt_navigator_node_kwargs)' in launch
    assert 'with_cpu_affinity("planner_server", planner_node_kwargs)' in launch

    assert 'planner_profile="${NJRH_NAV2_PLANNER_PROFILE:-smac2d}"' in runtime
    assert "smac2d)" in runtime
    assert "ranger_lattice)" in runtime
    assert "validate_ranger_mini3_lattice.py" in runtime
    assert "navigate_to_pose.xml" in runtime
    assert "navigate_to_pose_ranger_lattice_recovery.xml" in runtime
    assert 'nav_to_pose_bt_xml:="${nav_to_pose_bt_xml}"' in runtime
    assert 'planner_profile_file:="${planner_profile_file}"' in runtime
    assert 'ranger_lattice_filepath:="${ranger_lattice_file}"' in runtime
    assert '"NJRH_NAV2_PLANNER_PROFILE=${NJRH_NAV2_PLANNER_PROFILE:-smac2d}"' in systemd_runtime
    assert '"NJRH_NAV2_PLANNER_PROFILE_FILE=${NJRH_NAV2_PLANNER_PROFILE_FILE:-}"' in systemd_runtime
    assert '"NJRH_RANGER_LATTICE_FILE=${NJRH_RANGER_LATTICE_FILE:-}"' in systemd_runtime
    assert '"NJRH_NAV2_BT_XML=${NJRH_NAV2_BT_XML:-}"' in systemd_runtime

    ET.parse(lattice_bt_path)
    assert '<PipelineSequence name="NavigateRangerLatticeWithValidPath">' in lattice_bt
    assert '<Fallback name="KeepValidRangerLatticePath">' in lattice_bt
    assert '<GlobalUpdatedGoal/>' in lattice_bt
    assert '<IsPathValid path="{path}"/>' in lattice_bt
    assert lattice_bt.index('<IsPathValid path="{path}"/>') < lattice_bt.index(
        '<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>'
    )

    for nav2_config in (
        NAV_CONFIG / "config" / "nav2.yaml",
        ROOT / "scripts" / "jetson" / "runtime_overlay" / "config" / "nav2.yaml",
    ):
        nav2_text = nav2_config.read_text(encoding="utf-8")
        assert "nav2_globally_updated_goal_condition_bt_node" in nav2_text
        assert "nav2_is_path_valid_condition_bt_node" in nav2_text


def test_ranger_lattice_generator_is_pinned_to_jetson_nav2_version():
    generator = (
        NAV_CONFIG / "tools" / "generate_ranger_mini3_lattice.py"
    ).read_text(encoding="utf-8")
    cmake = (NAV_CONFIG / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (NAV_CONFIG / "package.xml").read_text(encoding="utf-8")
    planner_plugins = ET.parse(
        NAV_CONFIG / "robot_nav_config_planner_plugins.xml"
    ).getroot()

    assert 'PINNED_NAV2_TAG = "1.1.20"' in generator
    assert 'PINNED_NAV2_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"' in generator
    assert "PINNED_SOURCE_HASHES" in generator
    assert "verify_generator_source(generator_root)" in generator
    assert "add_ranger_spin_primitives(output)" in generator
    assert "lattice" in cmake
    assert "validate_ranger_mini3_lattice.py" in cmake
    assert "test_ranger_lattice_nav2_compatibility" in cmake
    assert "ranger_mini3_lattice_planner" in cmake
    assert "test_ranger_direct_corridor" in cmake
    assert "<depend>nav2_smac_planner</depend>" in package_xml
    assert planner_plugins.find("class").attrib["type"] == (
        "robot_nav_config::RangerMini3LatticePlanner"
    )

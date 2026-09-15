"""Static deployment/scope contracts; motion behavior is tested in C++ separately."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src/robot_nav_config"
TREES = PACKAGE / "behavior_trees"


def test_persistent_recovery_preserves_goal_and_exact_prepared_path():
    tree = ET.parse(TREES / "navigate_to_pose_ranger_lattice_recovery.xml")
    task = tree.find("./BehaviorTree/OrdinaryRecoveryLoop")
    assert task is not None
    assert len(task) == 2
    normal, recovery = task
    assert normal.tag == "PipelineSequence"
    assert [node.tag for node in recovery] == [
        "ComputePathToPose", "PrepareOrdinaryRecovery", "PipelineSequence"
    ]
    assert recovery[0].get("goal") == "{goal}"
    retry = recovery[2]
    assert recovery[0].get("path") == recovery[1].get("path") == retry.find("OrdinaryFollowPath").get("path")
    assert retry.find("./Fallback/OrdinaryPathTargetsGoal").get("goal") == "{goal}"
    assert len(tree.findall(".//OrdinaryFollowPath")) == 2
    for tag in ("ClearEntireCostmap", "Spin", "BackUp", "RetryUntilSuccessful"):
        assert tree.find(f".//{tag}") is None


def test_docking_elevator_and_baselines_do_not_select_recovery():
    paths = list(TREES.glob("navigate_elevator*.xml")) + [
        TREES / "navigate_to_predock.xml", TREES / "navigate_to_pose.xml",
        TREES / "navigate_to_pose_ranger_lattice.xml", TREES / "navigate_through_poses.xml"
    ]
    for path in paths:
        text = path.read_text(encoding="utf-8")
        assert "OrdinaryFollowPath" not in text
        assert "PrepareOrdinaryRecovery" not in text
        assert "OrdinaryRecoveryLoop" not in text


def test_runtime_has_plugin_but_unchanged_precision_and_progress():
    for file in (PACKAGE / "config/nav2.yaml",
                 ROOT / "scripts/jetson/runtime_overlay/config/nav2.yaml"):
        config = yaml.safe_load(file.read_text(encoding="utf-8"))
        assert "ordinary_navigation_recovery_bt_node" in config["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
        controller = config["controller_server"]["ros__parameters"]
        assert controller["goal_checker"]["xy_goal_tolerance"] == 0.06
        assert controller["goal_checker"]["yaw_goal_tolerance"] == 0.05
        assert controller["progress_checker"]["movement_time_allowance"] == 12.0
    launcher = (ROOT / "scripts/jetson/runtime_overlay/scripts/run_nav2_navigation.sh").read_text(encoding="utf-8")
    assert "${nav_bt_root}/navigate_to_pose_ranger_lattice_recovery.xml" in launcher
    assert "${nav_bt_root}/navigate_to_pose.xml" in launcher


def test_internal_service_does_not_modify_shared_interface_package():
    assert not (ROOT / "src/robot_interfaces/srv/PrepareOrdinaryNavigationRecovery.srv").exists()
    schema = (PACKAGE / "srv/PrepareOrdinaryNavigationRecovery.srv").read_text(encoding="utf-8")
    assert "builtin_interfaces/Time attempt_started" in schema
    assert "nav_msgs/Path path" in schema
    assert "bool prepared" in schema

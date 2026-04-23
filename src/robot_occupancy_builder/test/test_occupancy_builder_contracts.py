from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = ROOT / "src" / "robot_occupancy_builder"


def test_package_layout_exists():
    expected = [
        PACKAGE_ROOT / "package.xml",
        PACKAGE_ROOT / "CMakeLists.txt",
        PACKAGE_ROOT / "README.md",
        PACKAGE_ROOT / "config" / "live_draft.yaml",
        PACKAGE_ROOT / "config" / "release_rebuild.yaml",
        PACKAGE_ROOT / "launch" / "live_draft.launch.py",
        PACKAGE_ROOT / "launch" / "release_rebuild.launch.py",
        PACKAGE_ROOT / "scripts" / "occupancy_builder_live_node.py",
        PACKAGE_ROOT / "scripts" / "occupancy_builder_release_node.py",
        PACKAGE_ROOT / "scripts" / "rebuild_from_bag.py",
        PACKAGE_ROOT / "scripts" / "occupancy_postprocess.py",
    ]
    for path in expected:
        assert path.exists(), path


def test_live_draft_contract_is_fixed():
    text = (PACKAGE_ROOT / "config" / "live_draft.yaml").read_text(encoding="utf-8")
    assert "/mapping/frontend_pose" in text
    assert "/mapping/draft_map" in text
    assert "/sensors/lidar/points_raw" in text


def test_release_rebuild_contract_mentions_raw_bag():
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    helper = (PACKAGE_ROOT / "scripts" / "occupancy_postprocess.py").read_text(encoding="utf-8")
    assert "raw bag" in readme.lower()
    assert "optimized trajectory" in readme.lower()
    assert "run_release_rebuild" in helper
    assert "nav_map.pgm" in helper
    assert "localizer_map.png" in helper


def test_semantic_and_raytrace_requirements_are_present():
    helper = (PACKAGE_ROOT / "scripts" / "occupancy_postprocess.py").read_text(encoding="utf-8")
    assert "ground" in helper
    assert "ramp" in helper
    assert "obstacle" in helper
    assert "_bresenham_line" in helper
    assert "log_odds" in helper

#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
AGGREGATE = PACKAGE_ROOT / "src" / "features" / "docking" / "docking_feature_module.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "docking"
    / "lifecycle"
    / "docking_correction_pause_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "lifecycle"
    / "docking_correction_pause_module.cpp"
)


def main() -> None:
    assert HEADER.exists(), "DockingCorrectionPauseModule public seam is missing"
    assert SOURCE.exists(), "DockingCorrectionPauseModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    aggregate = AGGREGATE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "class DockingCorrectionPauseModule" in header
    assert "struct DockingCorrectionPauseConfig" in header
    assert "struct DockingCorrectionPausePorts" in header
    assert "bool set_paused(" in header
    assert "bool release_stale_if_needed(" in header

    for behavior in (
        "FINE_DOCKING_ENTRY_CHECK",
        "relocalize_after_fine_docking",
        "docking_fine",
        "active docking fine job still owns global correction pause",
        "no stale docking_fine correction pause",
        "stale docking_fine correction pause cleanup",
        "frozen_map_odom_plus_odom_base",
    ):
        assert behavior in source, behavior

    assert 'features/docking/lifecycle/docking_correction_pause_module.hpp' in aggregate
    assert "std::unique_ptr<DockingCorrectionPauseModule> correction_pause_;" in aggregate
    assert "DockingCorrectionPauseModule>" not in node
    for removed_owner in (
        "  void set_docking_global_correction_pause_state(",
        "  bool set_global_correction_paused_for_docking(",
        "  static bool docking_phase_owns_fine_correction_pause(",
        "  static bool bridge_status_has_docking_fine_pause(",
        "  bool release_stale_docking_fine_pause_if_needed(",
        "docking_pause_global_correction_during_fine_",
    ):
        assert removed_owner not in node, removed_owner


if __name__ == "__main__":
    main()

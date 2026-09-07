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
    / "dock_contact_interlock_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "docking"
    / "lifecycle"
    / "dock_contact_interlock_module.cpp"
)


def main() -> None:
    assert HEADER.exists(), "DockContactInterlockModule public seam is missing"
    assert SOURCE.exists(), "DockContactInterlockModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    aggregate = AGGREGATE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "class DockContactInterlockModule" in header
    assert "struct DockContactLatchSnapshot" in header
    assert "struct PreNavigationDockCheck" in header
    assert "DockContactLatchSnapshot read_latch() const" in header
    assert "void update_latch(" in header
    assert "void on_bms_contact_evidence(" in header
    assert "PreNavigationDockCheck snapshot()" in header
    assert "std::string pre_navigation_check_json(" in header

    for responsibility in (
        "njrh.docking_contact_latch.v1",
        "charging_session_latch_from_stable_bms_contact",
        "charging_session_latch_cleared_confirmed_undocked_no_contact",
        "stale_bms_latch_cleared_live_undocked_no_contact",
        "DOCKED_CHARGE_IDLE",
        "UNCERTAIN_ON_DOCK",
        "CONFIRMED_UNDOCKED",
        "final_auto_undock_required",
        "dock_contact_snapshot",
    ):
        assert responsibility in source, responsibility

    assert 'features/docking/lifecycle/dock_contact_interlock_module.hpp' in aggregate
    assert "std::unique_ptr<DockContactInterlockModule> contact_interlock_;" in aggregate
    assert "DockContactInterlockModule>" not in node

    for removed_owner in (
        "  struct DockContactLatchSnapshot",
        "  struct PreNavigationDockCheck",
        "  double dock_contact_latch_age_sec(",
        "  void refresh_dock_contact_latch_derived_fields(",
        "  DockContactLatchSnapshot read_dock_contact_latch(",
        "  void update_dock_contact_latch(",
        "  bool bms_latch_write_allowed_by_runtime(",
        "  void maybe_update_bms_dock_contact_latch(",
        "  PreNavigationDockCheck pre_navigation_dock_check_snapshot(",
        "  std::string bms_charging_contact_snapshot_json(",
        "  std::string dock_contact_latch_snapshot_json(",
        "  std::string pre_navigation_dock_check_json(",
        "have_last_dock_contact_latch_write_",
        "last_dock_contact_latch_docked_",
        "last_dock_contact_latch_source_",
        "last_dock_contact_latch_reason_",
        "last_dock_contact_latch_dock_id_",
        "last_dock_contact_latch_note_",
    ):
        assert removed_owner not in node, removed_owner


if __name__ == "__main__":
    main()

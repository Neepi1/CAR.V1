#!/usr/bin/env python3
"""Static ownership contract for durable charging-dock evidence."""

from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src" / "docking_manager_node.cpp"
text = SOURCE.read_text(encoding="utf-8")

battery_callback = text.split("battery_sub_ =", 1)[1].split("odom_sub_ =", 1)[0]
assert "update_dock_contact_latch" not in battery_callback, (
    "an unscoped BatteryState callback must not create durable on-dock evidence"
)

contact_stop = text.split("void finalize_docked_stop", 1)[1].split(
    "bool dock_contact_latch_is_docked", 1
)[0]
assert 'update_dock_contact_latch(true, "docking_manager"' in contact_stop, (
    "confirmed contact-stop must persist strong on-dock evidence"
)

undock = text.split("void finish_undock", 1)[1].split(
    "bool battery_indicates_charging", 1
)[0]
assert 'update_dock_contact_latch(false, "docking_manager", "undocked"' in undock, (
    "proven controlled undock must clear persistent on-dock evidence"
)

print("dock contact latch ownership contract: PASS")

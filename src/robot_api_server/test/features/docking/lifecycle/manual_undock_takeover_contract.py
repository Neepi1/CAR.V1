#!/usr/bin/env python3
"""Ensure manual undock cannot be deadlocked by an old docking owner."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
HTTP = PACKAGE_ROOT / "src" / "features" / "docking" / "lifecycle" / "docking_http_module.cpp"
AGGREGATE = PACKAGE_ROOT / "src" / "features" / "docking" / "docking_feature_module.cpp"

http = HTTP.read_text(encoding="utf-8")
aggregate = AGGREGATE.read_text(encoding="utf-8")

undock = http.split("HttpResponse DockingHttpModule::Impl::handle_undock", 1)[1]
for token in (
    "takeover_required",
    "ports_.prepare_controlled_undock",
    '\\"already_undocked\\":true',
    '\\"sent_velocity\\":false',
):
    assert token in undock, token

takeover = aggregate.split("bool prepare_controlled_undock", 1)[1].split(
    "void complete", 1
)[0]
for token in (
    "cancel_requested = true",
    "cancel_active_goal",
    "stop_if_available",
    "join_worker",
    '"canceled"',
    "release_stale_if_needed",
):
    assert token in takeover, token

print("manual undock takeover contract: PASS")

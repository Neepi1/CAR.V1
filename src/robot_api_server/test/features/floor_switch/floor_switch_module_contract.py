#!/usr/bin/env python3

import argparse
from pathlib import Path


def require(text: str, needle: str, source: Path) -> None:
    if needle not in text:
        raise AssertionError(f"missing {needle!r} in {source}")


def forbid(text: str, needle: str, source: Path) -> None:
    if needle in text:
        raise AssertionError(f"legacy floor-switch ownership {needle!r} remains in {source}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node-source", type=Path, required=True)
    parser.add_argument("--module-header", type=Path, required=True)
    parser.add_argument("--module-source", type=Path, required=True)
    args = parser.parse_args()

    composition_path = (
        args.node_source.parent
        / "application"
        / "composition"
        / "application_composition_module.cpp"
    )
    node = composition_path.read_text(encoding="utf-8")
    header = args.module_header.read_text(encoding="utf-8")
    module = args.module_source.read_text(encoding="utf-8")
    aggregate_path = (
        args.node_source.parent
        / "features"
        / "floor_switch"
        / "floor_switch_feature_module.cpp"
    )
    aggregate = aggregate_path.read_text(encoding="utf-8")
    router_wiring_path = (
        args.node_source.parent
        / "application"
        / "routing"
        / "application_router_wiring.cpp"
    )
    router_wiring = router_wiring_path.read_text(encoding="utf-8")

    require(
        node,
        '#include "robot_api_server/features/floor_switch/floor_switch_feature_module.hpp"',
        composition_path,
    )
    require(
        node,
        "std::unique_ptr<FloorSwitchFeatureModule> floor_switch_feature_module_",
        composition_path,
    )
    require(aggregate, "std::unique_ptr<FloorSwitchModule> module_", aggregate_path)
    require(
        router_wiring,
        "floor_switch->handle_http(request, motion_admission_epoch)",
        router_wiring_path,
    )

    require(header, "class FloorSwitchModule", args.module_header)
    require(header, "std::optional<HttpResponse> handle_http", args.module_header)
    require(header, "FloorRuntimeInterlockDecision interlock_decision", args.module_header)
    require(header, "void shutdown()", args.module_header)

    # Both runtime callers and HTTP admission must carry the exact operation to
    # the same policy. A generic decision here would reintroduce the asset lock.
    require(module, "return interlock_.decision_for_operation(operation);", args.module_source)
    for signature, following in (
        ("bool operation_blocked(", "std::optional<HttpResponse> interlock_response("),
        ("std::optional<HttpResponse> interlock_response(", "void shutdown()"),
    ):
        body = module.split(signature, 1)[1].split(following, 1)[0]
        require(body, "interlock_decision(operation)", args.module_source)
        forbid(body, "interlock_decision()", args.module_source)

    for route in (
        "/api/v1/floor-switch/start",
        "/api/v1/floor-switch/state",
        "/api/v1/floor-switch/cancel",
        "/api/v1/floors/switch",
    ):
        require(module, route, args.module_source)

    for legacy_owner in (
        "handle_live_floor_switch_start(",
        "handle_live_floor_switch_state(",
        "handle_live_floor_switch_cancel(",
        "run_live_floor_switch_worker(",
        "handle_switch_floor(",
        "live_floor_switch_client_",
        "floor_switch_client_",
        "floor_transition_status_sub_",
        "localization_floor_health_sub_",
        "live_floor_switch_transaction_",
    ):
        forbid(node, legacy_owner, composition_path)


if __name__ == "__main__":
    main()

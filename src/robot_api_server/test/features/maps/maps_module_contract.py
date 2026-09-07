#!/usr/bin/env python3

import argparse
from pathlib import Path


def require(text: str, needle: str, source: Path) -> None:
    if needle not in text:
        raise AssertionError(f"missing {needle!r} in {source}")


def forbid(text: str, needle: str, source: Path) -> None:
    if needle in text:
        raise AssertionError(f"legacy maps ownership {needle!r} remains in {source}")


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
        args.node_source.parent / "features" / "maps" / "maps_feature_module.cpp"
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
        '#include "robot_api_server/features/maps/maps_feature_module.hpp"',
        composition_path,
    )
    require(
        node,
        "std::unique_ptr<MapsFeatureModule> maps_feature_module_",
        composition_path,
    )
    require(aggregate, "std::unique_ptr<MapsModule> module_", aggregate_path)
    require(
        router_wiring,
        "maps->handle_http(request, motion_admission_epoch)",
        router_wiring_path,
    )

    require(header, "class MapsModule", args.module_header)
    require(header, "std::optional<HttpResponse> handle_http", args.module_header)
    require(header, "MapCatalog & catalog()", args.module_header)
    require(header, "bool validate_manifest_assets", args.module_header)
    require(header, "void activate_manifest", args.module_header)

    for route in (
        "/api/v1/maps",
        "/api/v1/maps/semantic_layer",
        "/api/v1/maps/poses",
        "/api/v1/maps/filters/keepout",
    ):
        require(module, route, args.module_source)

    for legacy_owner in (
        "handle_maps(",
        "handle_delete_map(",
        "resolve_map_manifest_from_query(",
        "handle_get_keepout_filter(",
        "handle_get_semantic_layer(",
        "handle_get_poses(",
        "handle_save_pose(",
        "handle_save_current_pose(",
        "handle_delete_pose(",
        "handle_replace_poses_batch(",
        "apply_keepout_runtime(",
        "handle_save_keepout_filter(",
        "recover_pending_map_activation(",
        "ensure_legacy_floor_map_manifest(",
        "keepout_mask_load_client_",
        "keepout_mask_sub_",
    ):
        forbid(node, legacy_owner, composition_path)


if __name__ == "__main__":
    main()

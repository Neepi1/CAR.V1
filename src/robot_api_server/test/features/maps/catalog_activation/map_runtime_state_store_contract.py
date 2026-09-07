#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
ROOT_NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "maps"
    / "catalog_activation"
    / "map_runtime_state_store.hpp"
)


def main() -> None:
    root = ROOT_NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    aggregate = (
        PACKAGE_ROOT / "src" / "features" / "maps" / "maps_feature_module.cpp"
    ).read_text(encoding="utf-8")

    for token in (
        "class MapRuntimeStateStore",
        "void write_runtime_map_context(",
        "std::optional<RuntimeMapContext> read_runtime_map_context() const",
        "void clear_runtime_map_context() const",
        "void write_last_navigation_map_selection(",
    ):
        assert token in header, f"runtime state store contract is missing: {token}"

    assert "std::unique_ptr<MapRuntimeStateStore> runtime_state_store_" in aggregate
    for legacy_owner in (
        "  void write_runtime_map_context(\n",
        "  std::optional<RuntimeMapContext> read_runtime_map_context() const",
        "  void clear_runtime_map_context() const",
        "  void write_last_navigation_map_selection(\n",
    ):
        assert legacy_owner not in root, f"composition root still owns: {legacy_owner!r}"

    print(
        "PASS: map runtime context and last-selection persistence are owned by "
        "MapRuntimeStateStore"
    )


if __name__ == "__main__":
    main()

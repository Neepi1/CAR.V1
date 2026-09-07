#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "localization"
    / "post_relocalization_settle_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "localization"
    / "post_relocalization_settle_module.cpp"
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    require(HEADER.is_file(), "post-relocalization settle module header is missing")
    require(SOURCE.is_file(), "post-relocalization settle module source is missing")

    node = NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    aggregate = (
        PACKAGE_ROOT
        / "src"
        / "features"
        / "localization"
        / "localization_feature_module.cpp"
    ).read_text(encoding="utf-8")
    require(
        "std::unique_ptr<PostRelocalizationSettleModule> settle_" in aggregate,
        "localization aggregate must own the post-relocalization settle module",
    )
    for legacy_root_ownership in (
        "struct PostRelocalizationSettleResult",
        "struct PostRelocalizationSettleState",
        "post_relocalization_settle_mutex_",
        "PostRelocalizationSettleState post_relocalization_settle_state_",
        "update_post_relocalization_settle_state(",
        "while (std::chrono::steady_clock::now() <= deadline)",
    ):
        require(
            legacy_root_ownership not in node,
            f"composition root still owns settle detail: {legacy_root_ownership}",
        )

    require(
        "class PostRelocalizationSettleModule" in header,
        "settle module public facade is missing",
    )
    for responsibility in (
        "wait_for_settle",
        "state_snapshot",
        "state_json",
        "POST_RELOCALIZATION_SEQUENCE_MISMATCH",
        "POST_RELOCALIZATION_LOCAL_COSTMAP_NOT_UPDATED",
        "POST_RELOCALIZATION_MAP_ODOM_PUBLISH_SEQUENCE_LAG",
    ):
        require(
            responsibility in header or responsibility in source,
            f"settle module responsibility is missing: {responsibility}",
        )


if __name__ == "__main__":
    main()

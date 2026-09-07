#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = PACKAGE_ROOT / "src" / "robot_api_server_node.cpp"
COMPOSITION = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "composition"
    / "application_composition_module.cpp"
)
HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "localization"
    / "localization_feature_module.hpp"
)
SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "localization"
    / "localization_feature_module.cpp"
)


def main() -> None:
    root = ROOT_NODE.read_text(encoding="utf-8")
    composition = COMPOSITION.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    for token in (
        "struct LocalizationFeatureModuleDependencies",
        "struct LocalizationFeatureLateDependencies",
        "class LocalizationFeatureModule",
        "void complete(LocalizationFeatureLateDependencies dependencies)",
        "LocalizationModule & module()",
        "PostRelocalizationSettleModule & settle()",
    ):
        assert token in header, token

    for owned_type in (
        "LocalizationModule",
        "PostRelocalizationSettleModule",
    ):
        assert f"unique_ptr<{owned_type}>" in source, owned_type

    assert (
        "std::unique_ptr<LocalizationFeatureModule> localization_feature_module_;"
        in composition
    )
    assert "localization_feature_module_->complete(" in composition
    assert composition.index("localization_feature_module_->complete(") < composition.index(
        "api_gateway_module_->start();"
    )
    assert "LocalizationFeatureModule" not in root

    for leaked_wiring in (
        "LocalizationModulePorts ",
        "PostRelocalizationSettlePorts ",
        "std::unique_ptr<LocalizationModule>",
        "std::unique_ptr<PostRelocalizationSettleModule>",
        "wait_for_post_relocalization_settle_barrier(",
        "post_relocalization_settle_state_json()",
    ):
        assert leaked_wiring not in composition, leaked_wiring

    print("PASS: localization aggregate owns localization and settle wiring")


if __name__ == "__main__":
    main()

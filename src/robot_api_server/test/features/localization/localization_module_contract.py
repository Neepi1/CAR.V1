#!/usr/bin/env python3

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[3]
ROOT_NODE = (
    PACKAGE_ROOT
    / "src"
    / "application"
    / "composition"
    / "application_composition_module.cpp"
)
MODULE_HEADER = (
    PACKAGE_ROOT
    / "include"
    / "robot_api_server"
    / "features"
    / "localization"
    / "localization_module.hpp"
)
MODULE_SOURCE = (
    PACKAGE_ROOT
    / "src"
    / "features"
    / "localization"
    / "localization_module.cpp"
)
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"
ROUTER_WIRING = (
    PACKAGE_ROOT / "src" / "application" / "routing" / "application_router_wiring.cpp"
)


def main() -> None:
    assert MODULE_HEADER.is_file(), "localization top-level module header is missing"
    assert MODULE_SOURCE.is_file(), "localization top-level module source is missing"

    root = ROOT_NODE.read_text(encoding="utf-8")
    module = MODULE_SOURCE.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")
    router_wiring = ROUTER_WIRING.read_text(encoding="utf-8")

    assert "features/localization/localization_module.hpp" in root
    assert "std::unique_ptr<LocalizationFeatureModule> localization_feature_module_" in root
    assert "std::unique_ptr<LocalizationModule> module_" in (
        PACKAGE_ROOT / "src" / "features" / "localization" / "localization_feature_module.cpp"
    ).read_text(encoding="utf-8")
    assert "src/features/localization/localization_module.cpp" in cmake
    assert "localization->handle_http(request, motion_admission_epoch)" in router_wiring

    forbidden_root_ownership = (
        "void handle_localization_result(",
        "void handle_localization_bridge_status(",
        "void handle_tf_message(",
        "void handle_tf_static_message(",
        "localization_result_sub_",
        "localization_bridge_status_sub_",
        "localization_trigger_client_",
        "localization_bridge_correction_pause_client_",
        "amcl_nomotion_update_client_",
        "latest_localization_result_seq_",
        "latest_map_to_odom_x_",
        "latest_odom_to_base_x_",
    )
    for symbol in forbidden_root_ownership:
        assert symbol not in root, f"root node still owns localization detail: {symbol}"

    required_module_ownership = (
        '"/api/v1/localization/trigger"',
        "handle_localization_result",
        "handle_localization_bridge_status",
        "handle_tf_message",
        "trigger_localization_and_wait_for_result",
        "wait_for_manual_relocalization_amcl_refine",
    )
    for symbol in required_module_ownership:
        assert symbol in module, f"localization module does not own: {symbol}"


if __name__ == "__main__":
    main()

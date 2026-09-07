#!/usr/bin/env python3

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NODE = (
    ROOT
    / "src"
    / "application"
    / "composition"
    / "application_composition_module.cpp"
)
HEADER = (
    ROOT
    / "include"
    / "robot_api_server"
    / "application"
    / "subscriptions"
    / "subscription_module.hpp"
)
SOURCE = ROOT / "src" / "application" / "subscriptions" / "subscription_module.cpp"
ROUTER_WIRING = ROOT / "src" / "application" / "routing" / "application_router_wiring.cpp"


def main() -> None:
    assert HEADER.exists(), "SubscriptionModule public boundary is missing"
    assert SOURCE.exists(), "SubscriptionModule implementation is missing"

    node = NODE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    router_wiring = ROUTER_WIRING.read_text(encoding="utf-8")

    assert "class SubscriptionModule" in header
    assert "std::optional<HttpResponse> handle_http" in header
    assert "void acquire(" in header
    assert "void release(" in header
    assert "std::string snapshot_json() const" in header
    assert "SubscriptionScanSnapshot scan_snapshot() const" in header

    for route in (
        "/api/v1/subscriptions/acquire",
        "/api/v1/subscriptions/release",
        "/api/v1/subscriptions/heartbeat",
    ):
        assert route in source
        assert f'request.path == "{route}"' not in node

    for ownership_marker in (
        "handle_subscription_update",
        "set_subscription_resource_active",
        "set_scan_subscription_active",
        "subscription_client_id_from_body",
        "subscription_resources_from_body",
        "subscription_ttl_ms_from_body",
        "resource_list_json",
        "subscription_ttl_timer_",
        "subscription_lifecycle_mutex_",
        "latest_scan_frame_",
        "latest_scan_range_count_",
        "latest_scan_angle_min_",
        "latest_scan_angle_max_",
        "latest_scan_received_at_",
        "have_scan_",
    ):
        assert ownership_marker not in node, ownership_marker
    assert "SharedPtr scan_sub_;" not in node

    assert 'application/subscriptions/subscription_feature_module.hpp' in node
    assert 'application/subscriptions/subscription_api.hpp' not in node
    assert 'application/subscriptions/subscription_manager.hpp' not in node
    assert "subscriptions->handle_http(request)" in router_wiring
    assert (
        "std::unique_ptr<SubscriptionFeatureModule> subscription_feature_module_;"
        in node
    )

    assert "create_wall_timer" in source
    assert "create_subscription<sensor_msgs::msg::LaserScan>" in source
    assert "ports.ensure_status_resident" in source
    assert "ports.set_live_map_page_active" in source
    assert "ports.ensure_tf_resident" in source
    assert "ports.clear_teleop_command" in source
    assert 'resource == "status"' in source
    assert 'resource == "live_map"' in source
    assert 'resource == "scan"' in source
    assert 'resource == "tf"' in source
    assert 'resource == "teleop"' in source


if __name__ == "__main__":
    main()

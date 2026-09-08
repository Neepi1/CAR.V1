from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_dock_interlock_interfaces_are_generated():
    cmake = _read("src/robot_interfaces/CMakeLists.txt")
    state = _read("src/robot_interfaces/msg/DockSafetyInterlockState.msg")
    service = _read("src/robot_interfaces/srv/ReconcileDockInterlock.srv")

    assert '"msg/DockSafetyInterlockState.msg"' in cmake
    assert '"srv/ReconcileDockInterlock.srv"' in cmake
    assert "bool memory_latched" in state
    assert "bool live_bms_contact" in state
    assert "bool outside_dock_zone_proven" in service
    assert "RESULT_BMS_CONTACT_ACTIVE" in service


def test_robot_safety_owns_state_and_constrained_reconciliation():
    node = _read("src/robot_safety/src/robot_safety_node.cpp")
    policy = _read("src/robot_safety/src/dock_contact_policy.cpp")

    assert "create_publisher<robot_interfaces::msg::DockSafetyInterlockState>" in node
    assert "create_service<robot_interfaces::srv::ReconcileDockInterlock>" in node
    assert "evaluate_dock_interlock_reconcile(context)" in node
    assert "OUTSIDE_DOCK_ZONE_NOT_PROVEN" in policy
    assert "BMS_NO_CONTACT_NOT_STABLE" in policy
    assert "DOCKING_COMMAND_ACTIVE" in policy


def test_navigation_preflight_reuses_existing_undock_or_reconciles_without_motion():
    check = _read(
        "src/robot_api_server/src/features/docking/lifecycle/"
        "dock_contact_interlock_module.cpp"
    )
    undock = _read(
        "src/robot_api_server/src/features/docking/lifecycle/"
        "pre_navigation_undock_module.cpp"
    )
    wiring = _read(
        "src/robot_api_server/src/features/docking/docking_feature_module.cpp"
    )

    assert 'pre_navigation_recovery_action = "CONTROLLED_UNDOCK"' in check
    assert 'pre_navigation_recovery_action = "CLEAR_STALE_INTERLOCK"' in check
    assert 'pre_navigation_recovery_action = "BLOCK"' in check
    assert 'request.recovery_action == "CLEAR_STALE_INTERLOCK"' in undock
    assert "ports_.reconcile_stale_interlock(" in undock
    assert "call_undock_with_charging_retry" in undock
    assert "controlled undock requires a resolved dock_id" in undock
    assert "find_floor_catalog_pose(" in wiring
    assert "current_robot_pose_snapshot()" in wiring


def test_source_and_runtime_configs_enable_the_same_fail_closed_contract():
    api_paths = (
        "src/robot_api_server/config/robot_api_server.yaml",
        "scripts/jetson/runtime_overlay/config/robot_api_server.yaml",
    )
    safety_paths = (
        "src/robot_safety/config/robot_safety.yaml",
        "scripts/jetson/runtime_overlay/config/robot_safety.yaml",
    )

    for path in api_paths:
        config = _read(path)
        assert "navigation_require_dock_safety_interlock_state: true" in config
        assert 'dock_safety_interlock_state_topic: "/safety/dock_interlock_state"' in config
        assert (
            'dock_safety_interlock_reconcile_service: '
            '"/safety/reconcile_dock_interlock"'
        ) in config
        assert "dock_interlock_zone_pose_max_age_sec: 0.5" in config
        assert "dock_interlock_zone_near_radius_m: 1.0" in config
        assert "dock_interlock_zone_clear_radius_m: 1.5" in config

    for path in safety_paths:
        config = _read(path)
        assert "bms_docking_interlock_reconcile_no_contact_sec: 3.0" in config
        assert "dock_safety_interlock_state_topic: /safety/dock_interlock_state" in config
        assert (
            "dock_safety_interlock_reconcile_service: "
            "/safety/reconcile_dock_interlock"
        ) in config

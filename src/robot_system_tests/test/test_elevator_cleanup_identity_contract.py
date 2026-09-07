from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
API_ROOT = ROOT / "src" / "robot_api_server"


def _function_body(source: str, start: str, end: str) -> str:
    begin = source.index(start)
    finish = source.index(end, begin)
    return source[begin:finish]


def test_cleanup_does_not_expire_transient_local_asset_snapshot():
    runtime = (API_ROOT / "src" / "elevator_ros_runtime_port.cpp").read_text(
        encoding="utf-8"
    )
    cleanup = _function_body(
        runtime,
        "RuntimeResult validate_live_cleanup_identity(",
        "\n  RuntimeResult validate_transaction(",
    )

    assert "have_localizer_asset_state_" in cleanup
    assert "localizer_asset_state_received_at_sec_" not in cleanup
    assert "assess_elevator_cleanup_runtime_identity(" in cleanup
    for diagnostic in (
        "asset_present=",
        "health_age_sec=",
        "bridge_age_sec=",
        "localizer_exact=",
        "health_exact=",
        "bridge_exact=",
    ):
        assert diagnostic in cleanup


def test_cleanup_keeps_live_health_and_bridge_fresh_and_exact():
    policy = (API_ROOT / "src" / "elevator_runtime_policy.cpp").read_text(
        encoding="utf-8"
    )
    assessment = _function_body(
        policy,
        "assess_elevator_cleanup_runtime_identity(",
        "\nstd::optional<robot_elevator_manager::ElevatorRuntimePose>",
    )

    assert "evidence.localization_health_received_at_sec" in assessment
    assert "evidence.localization_bridge_received_at_sec" in assessment
    assert "assessment.localization_health_fresh" in assessment
    assert "assessment.localization_bridge_fresh" in assessment
    assert "assessment.localization_health_exact" in assessment
    assert "assessment.localization_bridge_exact" in assessment
    assert "assessment.localizer_asset_exact" in assessment
    assert assessment.count("<= live_evidence_max_age_sec") == 2
    proven = assessment[
        assessment.index("assessment.proven =") : assessment.index(
            "return assessment;"
        )
    ]
    for required in (
        "assessment.localizer_asset_present",
        "assessment.localizer_asset_exact",
        "assessment.localization_health_fresh",
        "assessment.localization_health_exact",
        "assessment.localization_bridge_fresh",
        "assessment.localization_bridge_exact",
    ):
        assert required in proven

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
API_ROOT = ROOT / "src" / "robot_api_server"
EXECUTION_ROOT = API_ROOT / "src" / "features" / "elevator" / "execution"


def _function_body(source: str, start: str, end: str) -> str:
    begin = source.index(start)
    finish = source.index(end, begin)
    return source[begin:finish]


def test_cleanup_does_not_expire_transient_local_asset_snapshot():
    runtime = (EXECUTION_ROOT / "elevator_ros_runtime_port.cpp").read_text(
        encoding="utf-8"
    )
    cleanup = _function_body(
        runtime,
        "RuntimeResult validate_strict_cleanup_runtime_identity(",
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
    policy = (EXECUTION_ROOT / "elevator_runtime_policy.cpp").read_text(
        encoding="utf-8"
    )
    assessment = _function_body(
        policy,
        "assess_elevator_cleanup_runtime_identity(",
        "\nstd::optional<ElevatorRuntimeFloorIdentity>",
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


def test_cleanup_only_entry_uses_tested_policy_and_preserves_caller_safety_checks():
    runtime = (EXECUTION_ROOT / "elevator_ros_runtime_port.cpp").read_text(
        encoding="utf-8"
    )
    entry = _function_body(
        runtime,
        "RuntimeResult validate_live_cleanup_identity(",
        "\n  RuntimeResult validate_strict_cleanup_runtime_identity(",
    )
    assert "check_elevator_cleanup_runtime_readiness(" in entry
    assert "options_.persistent_recovery_lock_enabled, context.disposition" in entry
    assert "validate_strict_cleanup_runtime_identity(" in entry
    assert "*runtime_superseded_out = false" in entry
    # Supporting wiring check only. Actual callback policy is executed by C++
    # tests; these markers guard accidental removal of the independent proofs.
    finalization = _function_body(
        runtime, "RuntimeResult finalize_recovery(", "\n  void request_cancel("
    )
    for required in (
        "wait_for_restart_action_idle(", "wait_for_stop()",
        "wait_for_recovery_resource_absence(", "wait_for_no_foreign_elevator_hold(",
        "prove_no_delayed_side_effect_unknown()", "ReleaseMotionHoldIfExecutionIdle",
    ):
        assert required in finalization

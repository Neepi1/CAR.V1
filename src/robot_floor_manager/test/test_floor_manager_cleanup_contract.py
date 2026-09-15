"""Node wiring checks supplement (not replace) the real C++ cleanup tests.

No ROS process, service or robot connection is created by these tests.
"""

from pathlib import Path
import re


NODE = (Path(__file__).resolve().parents[1] / "src/floor_manager_node.cpp").read_text(
    encoding="utf-8"
)


def method(name):
    start = re.search(r"^  (?:bool|void|auto) " + name + r"\(", NODE, re.M)
    assert start, name
    brace = NODE.index("{", start.start())
    depth = 1
    cursor = brace + 1
    while depth:
        depth += (NODE[cursor] == "{") - (NODE[cursor] == "}")
        cursor += 1
    return NODE[brace:cursor]


def test_target_effect_latched_before_every_mutating_request():
    for name in (
        "load_map_with_client",  # NavMap and filter maps share this path.
        "apply_localizer_assets",
        "trigger_localization",
        "clear_costmap",
        "prepare_mask_lifecycle",
    ):
        body = method(name)
        assert "dispatch_target_request(" in body, name
        assert "target_effect_dispatched_ = false;" not in body, name
    dispatch = method("dispatch_target_request")
    assert dispatch.index("target_effect_dispatched_ = true;") < dispatch.index("async_send_request(request,")
    assert "observed.wait_for(std::chrono::milliseconds(0))" in dispatch
    assert "target_response_settled(observed.get())" in dispatch
    # Reset only when the serialized worker begins a newly admitted transaction.
    assert NODE.count("target_effect_dispatched_ = false;") == 1
    assert "target_effect_dispatched_ = false;" in method("execute_live_floor_switch")


def test_cleanup_uses_dispatch_evidence_not_begin_acknowledgement():
    assert re.search(
        r"select_floor_transition_cleanup\(\s*"
        r"begin_established, begin_outcome_unknown, target_effect_dispatched_\)",
        NODE,
    )
    restore = method("abort_bridge_and_prove_source")
    assert "::OP_ABORT_PREMUTATION" in restore
    assert re.search(r"::OP_ABORT\b", restore) is None
    assert "response->runtime_context_valid" in restore
    assert "wait_for_source_runtime_context_after" in restore


def test_pause_release_is_proven_before_motion_hold_release():
    start = NODE.index("const auto release_pre_mutation_resources =")
    cleanup = NODE[start:NODE.index("const auto retain_safety_resources =", start)]
    pause_check = re.search(r"if \(!pause_released\)\s*\{\s*return false;\s*\}", cleanup)
    assert pause_check
    assert cleanup.index("set_correction_pause(") < pause_check.start()
    assert pause_check.end() < cleanup.index("set_motion_hold(")
    assert "effect.transaction_id" in cleanup


def test_restored_source_requires_fresh_unchanged_localization():
    proof = method("wait_for_source_runtime_context_after")
    for required in (
        "localization_health_generation_ > generation_after_response",
        "last_localization_health_received_steady_sec_",
        "evidence_max_age_sec_",
        "same_runtime_identity(",
        "restored_explicit_sequence",
        "last_localization_health_->runtime_context_valid",
        "!last_localization_health_->transition_active",
        "last_localization_health_->tf_unique",
    ):
        assert required in proof
    assert "source_runtime_context_->explicit_relocalization_sequence" not in proof
    assert "source_runtime_context_->localizer_generation" not in proof
    assert "source_runtime_context_ = last_localization_health_;" in proof
    assert "response->explicit_relocalization_sequence" in method("abort_bridge_and_prove_source")


def test_restore_is_persisted_before_releasing_transaction_resources():
    start = NODE.index("if (source_restored) {")
    end = NODE.index("const auto retained = retain_safety_resources();", start)
    restored = NODE[start:end]
    assert restored.index("write_source_runtime_context(") < restored.index(
        "source_context_written && release_pre_mutation_resources()"
    )
    assert restored.index("release_pre_mutation_resources()") < restored.index(
        "runtime_context_confirmed_ = true;"
    )
    for expected in (
        "selected_map_id_ = evidence.active_map_id;",
        "published_asset_epoch_ = evidence.asset_epoch;",
        "published_asset_digest_ = evidence.asset_digest;",
    ):
        assert expected in restored


def test_completed_target_effect_is_not_a_permanent_motion_lock():
    start = NODE.index("case robot_floor_manager::FloorTransitionEffectKind::kHoldAndLock:")
    cleanup = NODE[start:NODE.index("case robot_floor_manager::FloorTransitionEffectKind::kNone:", start)]
    assert "target_requests_settled()" in cleanup
    assert "evidence.failure_resources_released = true;" in cleanup
    assert '"floor_switch_failed"' in cleanup
    assert "deferred_failure_cleanup_.store(true)" in cleanup


def test_wrapper_response_does_not_hide_nested_unknown_side_effects():
    assert 'response->message.find("BRIDGE_FORCE_ACCEPT_TIMEOUT")' in NODE
    assert 'response->message.find("BRIDGE_FORCE_ACCEPT_FAILED")' in NODE
    assert 'response->message.find("dispatch_state=not_dispatched")' in NODE
    assert 'response->rollback_succeeded && response->code.find("TIMEOUT")' in NODE
    assert "outcome.success || localizer_apply_failed_before_mutation(outcome.code)" in NODE
    assert "tracked_floor_transaction_ != request.transaction_id" in method("emergency_lock_after_exception")

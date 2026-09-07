#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "robot_elevator_manager/elevator_fsm.hpp"
#include "robot_elevator_manager/elevator_release_loader.hpp"

namespace robot_elevator_manager
{

// Physical location class captured before FAILURE_CLEANUP overwrites the
// elevator FSM state. Only the two OUTSIDE dispositions may ever release the
// transaction hold after the runtime adapter proves the exact corresponding
// floor context ready. RETAIN_LOCK requires on-site service.
enum class ElevatorCleanupDisposition
{
  kSourceOutside,
  kTargetOutside,
  kRetainLock,
};

ElevatorCleanupDisposition cleanup_disposition_for(
  ElevatorState state) noexcept;
std::string to_string(ElevatorCleanupDisposition disposition);

// Physical occupancy is intentionally independent from the active floor/map
// identity used for runtime cleanup. A robot can still be using the source
// map while it is already inside the cabin.
enum class ElevatorPhysicalZone
{
  kUnknown,
  kSourceOutside,
  kDoorway,
  kCabin,
  kTargetOutside,
};

ElevatorPhysicalZone physical_zone_for(ElevatorState state) noexcept;
std::string to_string(ElevatorPhysicalZone zone);

struct ElevatorRuntimeCapabilities
{
  ElevatorRuntimeCapabilities() = default;
  ElevatorRuntimeCapabilities(
    const bool runtime_capable_value,
    const bool motion_authorized_value,
    const bool floor_switch_capable_value,
    const bool automatic_button_control_value = false)
  : runtime_capable(runtime_capable_value),
    motion_authorized(motion_authorized_value),
    floor_switch_capable(floor_switch_capable_value),
    automatic_button_control(automatic_button_control_value)
  {
  }

  bool runtime_capable{false};
  bool motion_authorized{false};
  bool floor_switch_capable{false};
  // When true, the runtime port executes the two button effects. Door-open
  // and floor-arrival observations always remain explicit operator gates.
  bool automatic_button_control{false};
};

enum class ElevatorRuntimeFailureDisposition
{
  // Unknown failures remain fail-closed: the execution module must attempt
  // safety cleanup before it can expose a terminal result.
  kCleanupRequired,
  // The runtime adapter proved that the transaction was rejected before it
  // bound a release, acquired a hold/lease, changed mode, sent a goal, or
  // mutated a floor asset.
  kRejectedBeforeEffects,
  // The new transaction applied no effect, but pre-existing or stale runtime
  // ownership cannot be reconciled by cleanup that depends on this
  // transaction being prepared. Persist LOCKED directly for maintenance
  // recovery instead of attempting an invalid HoldAndCancel effect.
  kRecoveryRequiredBeforeEffects,
};

struct ElevatorRuntimeResult
{
  ElevatorRuntimeResult() = default;
  ElevatorRuntimeResult(
    const bool success_value,
    std::string code_value,
    std::string detail_value,
    const bool safety_hold_proven_value = false,
    const bool dual_odom_stop_proven_value = false,
    const ElevatorRuntimeFailureDisposition failure_disposition_value =
    ElevatorRuntimeFailureDisposition::kCleanupRequired,
    const bool safety_hold_absence_proven_value = false,
    const bool runtime_resources_reconciled_value = false,
    const bool motion_not_authorized_proven_value = false,
    const bool operator_confirmation_required_value = false)
  : success(success_value),
    code(std::move(code_value)),
    detail(std::move(detail_value)),
    safety_hold_proven(safety_hold_proven_value),
    dual_odom_stop_proven(dual_odom_stop_proven_value),
    failure_disposition(failure_disposition_value),
    safety_hold_absence_proven(safety_hold_absence_proven_value),
    runtime_resources_reconciled(runtime_resources_reconciled_value),
    motion_not_authorized_proven(motion_not_authorized_proven_value),
    operator_confirmation_required(operator_confirmation_required_value)
  {
  }

  bool success{false};
  std::string code{"ELEVATOR_RUNTIME_EFFECT_FAILED"};
  std::string detail;
  // These are authoritative observations, not requested intent. They let
  // failure cleanup report UNKNOWN/unsafe honestly when a hold or stop could
  // not be proven. Exactly one of safety_hold_proven and
  // safety_hold_absence_proven establishes a known hold state; neither (or
  // both) means UNKNOWN and must remain fail-closed.
  bool safety_hold_proven{false};
  bool dual_odom_stop_proven{false};
  ElevatorRuntimeFailureDisposition failure_disposition{
    ElevatorRuntimeFailureDisposition::kCleanupRequired};
  // Used only by explicit recovery finalization. A false value is UNKNOWN,
  // never proof that a hold is absent.
  bool safety_hold_absence_proven{false};
  // True only after the adapter has sequence-fenced and freshly observed
  // absence of every transaction-owned correction, mode, execution, action,
  // and floor-handoff resource. The owner safety hold may still be retained.
  bool runtime_resources_reconciled{false};
  // Authoritative per-effect evidence that robot_safety never admitted motion
  // after the navigation hold was released and before Nav2 reached a known
  // terminal result. This is not inferred from an error string. It is used
  // only to prove that a failed cabin-entry navigation never left the source
  // landing; every unknown or post-authorization failure remains fail-closed.
  bool motion_not_authorized_proven{false};
  // A successful automatic button attempt may explicitly fall back to the
  // existing operator gate only after the runtime has proven the arm stowed.
  // This is used for a deployed signal contract whose physical capability is
  // known unavailable; it is never inferred from an HTTP status alone.
  bool operator_confirmation_required{false};
};

struct ElevatorRuntimeEffect
{
  ElevatorEffect effect;
  std::string building_id;
  std::string elevator_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  std::optional<ElevatorRuntimePose> target_pose;
  ElevatorCleanupDisposition cleanup_disposition{
    ElevatorCleanupDisposition::kRetainLock};
};

struct ElevatorRuntimeFloorIdentity
{
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
};

struct ElevatorRuntimeCleanupContext
{
  std::string transaction_id;
  std::string building_id;
  ElevatorCleanupDisposition disposition{
    ElevatorCleanupDisposition::kRetainLock};
  ElevatorRuntimeFloorIdentity source;
  ElevatorRuntimeFloorIdentity target;
  bool legacy_preflight_orphan{false};
  // Durable operator evidence, reconstructed from the execution journal.
  // This may authorize only the one-way RETAIN_LOCK -> SOURCE_OUTSIDE
  // recovery-context promotion for the same immutable transaction/assets.
  bool source_outside_confirmation_recorded{false};
  // True only when the durable execution journal proves that a floor-switch
  // action may have been submitted. Recovery must not depend on the floor
  // action server before this boundary, while every unknown/positive value
  // remains fail-closed.
  bool floor_switch_action_may_have_been_submitted{true};
};

// Adapter seam for automatic effects. Door and floor observations never cross
// this seam. The two button effects cross it only when the port advertises
// automatic_button_control; otherwise they remain strict operator gates.
class ElevatorRuntimePort
{
public:
  virtual ~ElevatorRuntimePort() = default;

  virtual ElevatorRuntimeCapabilities capabilities() const noexcept = 0;
  // Bind and validate this exact frozen release. Implementations must not
  // reread a mutable "current" release selector.
  virtual ElevatorRuntimeResult prepare(
    const std::string & transaction_id,
    const FrozenElevatorRelease & release) = 0;
  virtual ElevatorRuntimeResult apply(const ElevatorRuntimeEffect & effect) = 0;
  // Reconcile a journaled nonterminal transaction after a process restart.
  // Implementations must retain an exact owner hold, cancel any action goals
  // whose admission/handle state was lost with the process, and prove a
  // dual-odometry settled stop. The recovery path never resumes the mission
  // and never releases the safety hold.
  virtual ElevatorRuntimeResult recover_locked(
    const ElevatorRuntimeCleanupContext & context)
  {
    request_cancel(context.transaction_id);
    return {
      false,
      "ELEVATOR_RESTART_RECOVERY_UNSUPPORTED",
      "runtime port cannot prove restart recovery",
      false,
      false,
    };
  }
  // Complete an explicit maintenance recovery after recover_locked() has
  // retained the owner hold and the module has journaled release intent.
  // Success must include fresh dual-odometry stop evidence and authoritative
  // proof that this exact transaction's owner hold is absent.
  virtual ElevatorRuntimeResult finalize_recovery(
    const ElevatorRuntimeCleanupContext & context)
  {
    (void)context;
    return {
      false,
      "ELEVATOR_RECOVERY_FINALIZE_UNSUPPORTED",
      "runtime port cannot safely release a recovered transaction",
      false,
      false,
    };
  }
  // Non-blocking emergency path used after cancellation intent is journaled.
  // The adapter must immediately request cancellation of the transaction-owned
  // goal and assert/retain its owner-scoped safety hold so a blocked apply()
  // can be interrupted without waiting for its normal timeout.
  virtual void request_cancel(const std::string & transaction_id) noexcept = 0;
  // Called by the module's single worker while a transaction is active,
  // including during long operator-confirmation waits.
  virtual ElevatorRuntimeResult heartbeat(const std::string & transaction_id) = 0;
  virtual ElevatorRuntimeResult poll_health(const std::string & transaction_id) = 0;
};

// Deterministic adapter for pure-C++ acceptance tests. Results are consumed in
// FIFO order per effect kind; an effect succeeds when no result was queued.
class InMemoryElevatorRuntimePort final : public ElevatorRuntimePort
{
public:
  explicit InMemoryElevatorRuntimePort(
    ElevatorRuntimeCapabilities capabilities = {true, true, true});

  ElevatorRuntimeCapabilities capabilities() const noexcept override;
  ElevatorRuntimeResult prepare(
    const std::string & transaction_id,
    const FrozenElevatorRelease & release) override;
  ElevatorRuntimeResult apply(const ElevatorRuntimeEffect & effect) override;
  ElevatorRuntimeResult recover_locked(
    const ElevatorRuntimeCleanupContext & context) override;
  ElevatorRuntimeResult finalize_recovery(
    const ElevatorRuntimeCleanupContext & context) override;
  void request_cancel(const std::string & transaction_id) noexcept override;
  ElevatorRuntimeResult heartbeat(const std::string & transaction_id) override;
  ElevatorRuntimeResult poll_health(const std::string & transaction_id) override;

  void queue_result(ElevatorEffectKind kind, ElevatorRuntimeResult result);
  void queue_prepare_result(ElevatorRuntimeResult result);
  void queue_recovery_result(ElevatorRuntimeResult result);
  void queue_recovery_finalize_result(ElevatorRuntimeResult result);
  void queue_heartbeat_result(ElevatorRuntimeResult result);
  void queue_health_result(ElevatorRuntimeResult result);
  std::vector<ElevatorRuntimeEffect> applied_effects() const;
  std::optional<FrozenElevatorRelease> prepared_release() const;
  std::vector<std::string> cancellation_requests() const;

private:
  ElevatorRuntimeCapabilities capabilities_;
  mutable std::mutex mutex_;
  std::map<ElevatorEffectKind, std::vector<ElevatorRuntimeResult>> queued_results_;
  std::vector<ElevatorRuntimeResult> queued_prepare_results_;
  std::vector<ElevatorRuntimeResult> queued_recovery_results_;
  std::vector<ElevatorRuntimeResult> queued_recovery_finalize_results_;
  std::vector<ElevatorRuntimeResult> queued_heartbeat_results_;
  std::vector<ElevatorRuntimeResult> queued_health_results_;
  std::vector<ElevatorRuntimeEffect> applied_effects_;
  std::optional<FrozenElevatorRelease> prepared_release_;
  std::vector<std::string> cancellation_requests_;
};

struct ElevatorExecutionEventRecord
{
  std::uint64_t sequence{0U};
  std::string code;
  std::string detail;
  std::string timestamp;
};

struct ElevatorExecutionSnapshot
{
  std::string transaction_id;
  std::string state{"IDLE"};
  std::string phase{"IDLE"};
  std::uint64_t effect_sequence{0U};
  bool awaiting_confirmation{false};
  std::string expected_confirmation;
  std::string current_floor_id;
  std::string current_map_id;
  ElevatorPhysicalZone physical_zone{ElevatorPhysicalZone::kUnknown};
  // Durable checkpoint retained when FAILURE_CLEANUP or restart LOCKED
  // replaces the live FSM state exposed to callers.
  std::string interrupted_state;
  std::string interrupted_expected_confirmation;
  // safety_hold_active is meaningful only while this bit is true. UNKNOWN is
  // represented explicitly instead of being collapsed into "not active".
  bool safety_hold_state_known{false};
  bool safety_hold_active{false};
  // Authoritative dual-odometry settled-stop evidence from the runtime port.
  // This must never be inferred merely because an effect returned success.
  bool dual_odom_stop_proven{false};
  bool runtime_capable{false};
  bool runtime_applied{false};
  // Historical runtime_applied is not a recovery interlock. This separate
  // proof bit records whether all transaction-owned runtime resources have
  // been reconciled and freshly observed absent.
  bool runtime_resources_reconciled{false};
  bool motion_authorized{false};
  bool floor_switch_capable{false};
  bool terminal{false};
  std::string pinned_release_id;
  // Exact internal replay identity. It includes every frozen pose binding and
  // coordinate but is intentionally not part of the App JSON projection.
  std::string frozen_release_identity;
  std::uint64_t pinned_release_generation{0U};
  std::string building_id;
  std::string elevator_id;
  std::string source_floor_id;
  std::string source_map_id;
  std::uint64_t source_asset_epoch{0U};
  std::string source_asset_digest;
  std::string target_floor_id;
  std::string target_map_id;
  std::uint64_t target_asset_epoch{0U};
  std::string target_asset_digest;
  std::string operator_id;
  std::string created_at;
  std::string updated_at;
  std::string failure_code;
  std::string detail;
  // Captured durably before state/phase are overwritten by FAILURE_CLEANUP.
  std::string failure_origin_state;
  std::string failure_origin_effect_kind{"NONE"};
  ElevatorCleanupDisposition cleanup_disposition{
    ElevatorCleanupDisposition::kRetainLock};
  std::vector<ElevatorExecutionEventRecord> events;
  std::vector<ElevatorExecutionEventRecord> errors;
};

// RUNTIME_EFFECT_INTENT is persisted before the runtime adapter is called.
// BEGIN_FLOOR_TRANSITION is the effect that submits the floor action; the
// subsequent SWITCH_FLOOR effect only awaits its result. This predicate is the
// durable recovery boundary shared with the production runtime adapter.
bool snapshot_may_have_submitted_floor_switch_action(
  const ElevatorExecutionSnapshot & snapshot) noexcept;

// Server-authoritative recovery affordances. The App renders these values but
// cannot invent an unlock path from local failure-code heuristics.
std::vector<std::string> allowed_recovery_actions(
  const ElevatorExecutionSnapshot & snapshot);

struct ElevatorExecutionStart
{
  std::string transaction_id;
  std::string operator_id;
  FrozenElevatorRelease release;
};

struct ElevatorExecutionConfirmation
{
  std::string transaction_id;
  std::string expected_state;
  std::uint64_t effect_sequence{0U};
  std::string event;
  std::string observed_floor_id;
  std::string operator_id;
};

struct ElevatorExecutionCancellation
{
  std::string transaction_id;
  std::uint64_t effect_sequence{0U};
  std::string reason;
  std::string operator_id;
};

struct ElevatorExecutionRecovery
{
  ElevatorExecutionRecovery() = default;

  ElevatorExecutionRecovery(
    std::string transaction_id_value,
    std::string expected_state_value,
    const std::uint64_t effect_sequence_value,
    std::string operator_id_value,
    std::string reason_value,
    const bool source_outside_confirmed_value = false,
    std::string confirmed_floor_id_value = {},
    std::string action_value = {},
    std::string physical_zone_value = {},
    const bool stationary_confirmed_value = false,
    const bool door_zone_clear_confirmed_value = false)
  : transaction_id(std::move(transaction_id_value)),
    expected_state(std::move(expected_state_value)),
    effect_sequence(effect_sequence_value),
    operator_id(std::move(operator_id_value)),
    reason(std::move(reason_value)),
    source_outside_confirmed(source_outside_confirmed_value),
    confirmed_floor_id(std::move(confirmed_floor_id_value)),
    action(std::move(action_value)),
    physical_zone(std::move(physical_zone_value)),
    stationary_confirmed(stationary_confirmed_value),
    door_zone_clear_confirmed(door_zone_clear_confirmed_value)
  {
  }

  std::string transaction_id;
  std::string expected_state;
  std::uint64_t effect_sequence{0U};
  std::string operator_id;
  std::string reason;
  // This is deliberately narrower than a generic unlock. The operator may
  // confirm only a physically observed source-outside recovery; the runtime
  // adapter must independently prove idle actions, exact source assets, the
  // owner hold, and dual-odometry stop before release.
  bool source_outside_confirmed{false};
  std::string confirmed_floor_id;
  std::string action;
  std::string physical_zone;
  bool stationary_confirmed{false};
  bool door_zone_clear_confirmed{false};
};

enum class ElevatorExecutionReplyKind
{
  kAccepted,
  kInvalid,
  kConflict,
  kStorageFailure,
};

struct ElevatorExecutionReply
{
  ElevatorExecutionReplyKind kind{ElevatorExecutionReplyKind::kStorageFailure};
  std::string code{"ELEVATOR_EXECUTION_INTERNAL_ERROR"};
  std::string detail;
  std::optional<ElevatorExecutionSnapshot> snapshot;

  bool accepted() const noexcept
  {
    return kind == ElevatorExecutionReplyKind::kAccepted &&
           snapshot.has_value();
  }
};

struct ElevatorExecutionOptions
{
  // Compatibility default for isolated legacy tests. Production explicitly
  // disables this policy: failed elevator work is cleaned up automatically
  // and must never become a persistent, operator-cleared global lock.
  bool persistent_recovery_lock_enabled{true};
};

// Thread-safe, single-active-transaction execution module. Every automatic
// effect is journaled before the runtime port sees it. A nonterminal journal
// found after restart becomes a terminal LOCKED snapshot and cannot resume
// implicitly. A dedicated worker recovery path re-establishes the owner hold,
// cancels unknown action state, and proves a dual-odometry stop without
// releasing the lock. start/confirm/cancel never wait for prepare(), apply(),
// or the normal runtime timeout; they journal intent and hand work to one
// worker.
class ElevatorExecutionModule
{
public:
  ElevatorExecutionModule(
    std::filesystem::path journal_path,
    std::shared_ptr<ElevatorRuntimePort> runtime_port,
    ElevatorExecutionOptions options = {});
  ~ElevatorExecutionModule();

  ElevatorExecutionModule(const ElevatorExecutionModule &) = delete;
  ElevatorExecutionModule & operator=(const ElevatorExecutionModule &) = delete;
  ElevatorExecutionModule(ElevatorExecutionModule &&) = delete;
  ElevatorExecutionModule & operator=(ElevatorExecutionModule &&) = delete;

  ElevatorExecutionReply start(const ElevatorExecutionStart & command);
  ElevatorExecutionReply confirm(const ElevatorExecutionConfirmation & command);
  ElevatorExecutionReply cancel(const ElevatorExecutionCancellation & command);
  ElevatorExecutionReply recover(const ElevatorExecutionRecovery & command);
  ElevatorExecutionReply snapshot(const std::string & transaction_id) const;
  std::optional<ElevatorExecutionSnapshot> current_snapshot() const;
  std::optional<std::string> active_transaction() const;
  // True when an on-disk journal exists but cannot be read or durably
  // reconciled. API admission must fail closed even if no snapshot can be
  // rendered.
  bool journal_recovery_required() const;
  bool persistent_recovery_lock_enabled() const noexcept;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace robot_elevator_manager

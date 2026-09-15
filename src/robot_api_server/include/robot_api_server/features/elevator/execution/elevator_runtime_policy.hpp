#pragma once

#include <cstdint>
#include <chrono>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "robot_elevator_manager/elevator_execution_module.hpp"

namespace robot_api_server
{

// Owns one asynchronous cancel response across bounded recovery attempts.
// A wait timeout does not erase the future: the already-submitted request can
// still complete later, and a retry must observe that same response instead
// of submitting a duplicate request with another unknown side effect.
template<typename ResponseT>
class RetainedRecoveryCancelResponse
{
public:
  bool pending() const noexcept
  {
    return future_.valid();
  }

  template<typename Submit>
  bool submit_if_absent(Submit && submit)
  {
    if (pending()) {
      return false;
    }
    auto future = std::forward<Submit>(submit)();
    if (!future.valid()) {
      throw std::runtime_error("cancel request returned an invalid future");
    }
    future_ = std::move(future);
    return true;
  }

  template<typename Rep, typename Period>
  std::future_status wait_for(
    const std::chrono::duration<Rep, Period> & duration) const
  {
    if (!pending()) {
      throw std::logic_error("no cancel response is pending");
    }
    return future_.wait_for(duration);
  }

  ResponseT take_ready()
  {
    if (!pending()) {
      throw std::logic_error("no cancel response is pending");
    }
    auto future = std::move(future_);
    future_ = {};
    return future.get();
  }

  void abandon() noexcept
  {
    future_ = {};
  }

private:
  std::shared_future<ResponseT> future_;
};

struct ElevatorPreparedSourceEvidence
{
  bool runtime_context_confirmed{false};
  std::string runtime_context_state;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  bool navigation_idle{false};
  bool mapping_idle{false};
  bool docking_idle{false};
};

// Cleanup combines one latched identity snapshot with two live readiness
// streams. Asset receipt time is intentionally absent: the publisher is
// transient-local and republishes only when the asset changes. Health and
// bridge status remain time-bounded live evidence.
struct ElevatorCleanupRuntimeIdentityEvidence
{
  bool localizer_asset_present{false};
  bool localizer_asset_exact{false};
  bool localization_health_present{false};
  bool localization_health_exact{false};
  double localization_health_received_at_sec{0.0};
  bool localization_bridge_present{false};
  bool localization_bridge_exact{false};
  double localization_bridge_received_at_sec{0.0};
};

struct ElevatorCleanupRuntimeIdentityAssessment
{
  bool proven{false};
  bool localizer_asset_present{false};
  bool localizer_asset_exact{false};
  bool localization_health_fresh{false};
  bool localization_health_exact{false};
  bool localization_bridge_fresh{false};
  bool localization_bridge_exact{false};
  double localization_health_age_sec{-1.0};
  double localization_bridge_age_sec{-1.0};
};

// Durable elevator recovery can outlive a whole runtime restart.  When a
// transaction failed before any floor-switch Action could have been
// submitted, the restarted runtime may already be bound to either frozen
// endpoint.  This snapshot is used to prove that the current endpoint is one
// of those exact immutable assets; it never permits an unrelated floor.
struct ElevatorCleanupRuntimeFloorEvidence
{
  bool confirmed{false};
  std::string state;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
};

enum class ElevatorExecutionInterlockKind
{
  kClear,
  kActive,
  kRecoveryRequired,
  kDelayedSideEffectUnknown,
};

struct ElevatorExecutionInterlock
{
  ElevatorExecutionInterlockKind kind{ElevatorExecutionInterlockKind::kClear};
  std::string transaction_id;
  std::uint64_t delayed_side_effect_unknown_count{0U};

  bool blocked() const noexcept
  {
    return kind != ElevatorExecutionInterlockKind::kClear;
  }

  bool recovery_required() const noexcept
  {
    return kind == ElevatorExecutionInterlockKind::kRecoveryRequired;
  }

  bool delayed_side_effect_unknown() const noexcept
  {
    return kind == ElevatorExecutionInterlockKind::kDelayedSideEffectUnknown;
  }
};

// Linearizes every delayed motion/runtime admission with the final elevator
// recovery barrier. HTTP requests capture an epoch before doing slow asset or
// runtime checks. Recovery invalidates that epoch while holding the same
// mutex through its final cancel/idle checks and conditional hold release.
// Consequently a request that entered before recovery can never become a new
// motion side effect after recovery has completed.
class ElevatorMotionAdmissionFence
{
public:
  using Epoch = std::uint64_t;

  class AdmissionGuard
  {
  public:
    AdmissionGuard(AdmissionGuard &&) noexcept = default;
    AdmissionGuard & operator=(AdmissionGuard &&) noexcept = default;
    AdmissionGuard(const AdmissionGuard &) = delete;
    AdmissionGuard & operator=(const AdmissionGuard &) = delete;

    bool admitted() const noexcept
    {
      return admitted_;
    }

    bool stale() const noexcept
    {
      return stale_;
    }

    const ElevatorExecutionInterlock & interlock() const noexcept
    {
      return interlock_;
    }

    void unlock()
    {
      if (lock_.owns_lock()) {
        lock_.unlock();
      }
    }

  private:
    friend class ElevatorMotionAdmissionFence;

    AdmissionGuard(
      std::unique_lock<std::mutex> lock,
      const bool admitted,
      const bool stale,
      ElevatorExecutionInterlock interlock)
    : lock_(std::move(lock)),
      admitted_(admitted),
      stale_(stale),
      interlock_(std::move(interlock))
    {
    }

    std::unique_lock<std::mutex> lock_;
    bool admitted_{false};
    bool stale_{false};
    ElevatorExecutionInterlock interlock_;
  };

  Epoch capture_epoch() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return epoch_;
  }

  template<typename InterlockProbe>
  AdmissionGuard acquire_for_submission(
    const Epoch expected_epoch,
    InterlockProbe && probe)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (expected_epoch != epoch_) {
      return AdmissionGuard(
        std::move(lock), false, true,
        ElevatorExecutionInterlock{
          ElevatorExecutionInterlockKind::kRecoveryRequired,
          "stale-admission-epoch"});
    }
    if (closed_) {
      return AdmissionGuard(
        std::move(lock), false, false,
        ElevatorExecutionInterlock{
          ElevatorExecutionInterlockKind::kRecoveryRequired,
          "admission-fence-closed"});
    }
    auto interlock = std::forward<InterlockProbe>(probe)();
    return AdmissionGuard(
      std::move(lock), !interlock.blocked(), false, std::move(interlock));
  }

  std::unique_lock<std::mutex> invalidate_pending_and_lock()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    closed_ = true;
    advance_epoch_locked();
    return lock;
  }

  void close_and_invalidate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    advance_epoch_locked();
  }

  void reopen_and_invalidate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!closed_) {
      return;
    }
    closed_ = false;
    advance_epoch_locked();
  }

private:
  void advance_epoch_locked()
  {
    ++epoch_;
    if (epoch_ == 0U) {
      ++epoch_;
    }
  }

  mutable std::mutex mutex_;
  Epoch epoch_{1U};
  bool closed_{false};
};

inline bool elevator_recovery_maintenance_peer_allowed(
  const bool api_token_configured,
  const bool peer_is_loopback) noexcept
{
  return api_token_configured || peer_is_loopback;
}

inline ElevatorExecutionInterlock evaluate_elevator_execution_interlock(
  const std::optional<
    robot_elevator_manager::ElevatorExecutionSnapshot> & snapshot,
  const bool journal_recovery_required = false,
  const bool persistent_recovery_lock_enabled = true)
{
  if (persistent_recovery_lock_enabled && journal_recovery_required) {
    return {
      ElevatorExecutionInterlockKind::kRecoveryRequired,
      snapshot ? snapshot->transaction_id : "journal-unavailable",
    };
  }
  if (!snapshot) {
    return {};
  }
  if (
    !persistent_recovery_lock_enabled && !snapshot->terminal &&
    snapshot->state == "FAILURE_CLEANUP")
  {
    // Production cleanup is background reconciliation, not a global operator
    // latch. The failed transaction cannot emit another elevator effect, but
    // unrelated API work must not be held hostage by unavailable recovery
    // endpoints (for example while Nav2 is intentionally offline).
    return {};
  }
  const bool terminal_recovery_required =
    persistent_recovery_lock_enabled &&
    snapshot->terminal &&
    (!snapshot->safety_hold_state_known || snapshot->safety_hold_active ||
    (snapshot->runtime_applied &&
    !snapshot->runtime_resources_reconciled));
  if (
    (persistent_recovery_lock_enabled && snapshot->state == "LOCKED") ||
    terminal_recovery_required)
  {
    return {
      ElevatorExecutionInterlockKind::kRecoveryRequired,
      snapshot->transaction_id,
    };
  }
  if (!snapshot->terminal) {
    return {
      ElevatorExecutionInterlockKind::kActive,
      snapshot->transaction_id,
    };
  }
  return {};
}

robot_elevator_manager::ElevatorRuntimeResult validate_elevator_prepared_source(
  const robot_elevator_manager::FrozenElevatorRelease & release,
  const ElevatorPreparedSourceEvidence & evidence);

// Cleanup-only seam. Success here proves neither stopping nor resource release;
// callers must independently prove both before releasing their owner hold.
template<typename StrictReadinessProbe>
robot_elevator_manager::ElevatorRuntimeResult check_elevator_cleanup_runtime_readiness(
  const bool persistent_recovery_lock_enabled,
  const robot_elevator_manager::ElevatorCleanupDisposition disposition,
  StrictReadinessProbe && strict_readiness_probe)
{
  using robot_elevator_manager::ElevatorCleanupDisposition;
  if (disposition != ElevatorCleanupDisposition::kSourceOutside &&
    disposition != ElevatorCleanupDisposition::kTargetOutside)
  {
    return {false, "ELEVATOR_CLEANUP_DISPOSITION_UNSAFE",
      "only a proven outside-floor disposition can validate release identity"};
  }
  if (!persistent_recovery_lock_enabled) {
    return {true, "ELEVATOR_CLEANUP_RUNTIME_READINESS_NOT_REQUIRED",
      "nonpersistent cleanup does not require a navigable map; stopping and "
      "owner-resource release must still be independently proven"};
  }
  return std::forward<StrictReadinessProbe>(strict_readiness_probe)();
}

ElevatorCleanupRuntimeIdentityAssessment
assess_elevator_cleanup_runtime_identity(
  const ElevatorCleanupRuntimeIdentityEvidence & evidence,
  double now_sec,
  double live_evidence_max_age_sec) noexcept;

std::optional<robot_elevator_manager::ElevatorRuntimeFloorIdentity>
resolve_elevator_cleanup_runtime_floor(
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & context,
  const ElevatorCleanupRuntimeFloorEvidence & evidence) noexcept;

// A non-persistent restart cleanup may encounter a completely new, confirmed
// runtime after the old elevator transaction has already been abandoned.  In
// that narrow restart-only case, the current runtime can supersede the stale
// frozen endpoints once its complete identity is available.  Ordinary live
// cleanup continues to use resolve_elevator_cleanup_runtime_floor() and never
// accepts an unrelated map.
bool elevator_nonpersistent_restart_cleanup_superseded_by_ready_runtime(
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & context,
  const ElevatorCleanupRuntimeFloorEvidence & evidence) noexcept;

bool elevator_cleanup_context_equal(
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & left,
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & right) noexcept;

// Allows an already-bound production runtime to consume the durable on-site
// confirmation recorded after a retained cabin-entry failure. All immutable
// transaction, building, floor, map, epoch, and digest fields remain exact;
// no other disposition transition is permitted.
bool elevator_cleanup_context_rebind_allowed(
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & bound,
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & requested)
noexcept;

enum class ElevatorNavigationProfile
{
  kOrdinaryNav2,
  kElevatorScoped,
  kElevatorReverseEntryStaging,
  kElevatorReverseDocking,
  kElevatorCabinDirect,
};

struct ElevatorNavigationRequest
{
  robot_elevator_manager::ElevatorRuntimePose target;
  ElevatorNavigationProfile profile{ElevatorNavigationProfile::kOrdinaryNav2};
  robot_elevator_manager::ElevatorNavigationIntent navigation_intent{
    robot_elevator_manager::ElevatorNavigationIntent::kNone};
};

struct ElevatorMapPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  // ROS measurement stamp and API-cache age. These are evidence fields, not
  // alternative pose coordinates or a commissioned waypoint binding.
  double stamp_sec{0.0};
  double age_sec{-1.0};
};

// Proves that a floor transition produced a new, fresh and stable live
// map-frame robot pose. It deliberately has no cabin/cabin_panel input: those
// poses are navigation targets and must never become cross-map localization
// equality constraints.
class ElevatorTargetMapPoseTracker
{
public:
  void reset(double source_pose_stamp_sec) noexcept;
  bool observe(
    const ElevatorMapPose & pose,
    double observed_at_sec,
    double max_age_sec,
    double settle_sec,
    double translation_stability_m,
    double yaw_stability_rad) noexcept;

  bool ready() const noexcept {return ready_;}
  const std::string & last_reason() const noexcept {return last_reason_;}

private:
  double source_pose_stamp_sec_{0.0};
  double last_pose_stamp_sec_{0.0};
  double stable_since_sec_{-1.0};
  ElevatorMapPose stable_anchor_;
  bool have_stable_anchor_{false};
  bool ready_{false};
  std::string last_reason_{"target map pose has not been observed"};
};

enum class ElevatorMotionAdmissionScope
{
  // The first leg is ordinary navigation to the hall-call pose.  No elevator
  // operating-mode contract is active yet.
  kHallApproach,

  // Every motion leg after hall-call completion runs inside the exact
  // transaction-owned operating-mode contract. The former execution lease
  // is deliberately not part of elevator-test admission.
  kElevatorExecution,
};

enum class ElevatorMotionAdmissionKind
{
  kWaiting,
  kAuthorized,
  kCommandWarmup,
  kHardBlocked,
};

ElevatorMotionAdmissionScope motion_admission_scope_for_navigation(
  robot_elevator_manager::ElevatorNavigationIntent intent) noexcept;

// Every navigation leg after the hall-call pose uses the transaction-scoped
// obstacle-unchecked command route, including the source landing/entry-staging
// move. Only the pre-call hall approach retains the ordinary
// collision-monitor-checked command chain.
bool elevator_navigation_bypasses_collision_monitor(
  robot_elevator_manager::ElevatorNavigationIntent intent) noexcept;

struct ElevatorMotionAdmissionEvidence
{
  bool motion_allowed_present{false};
  bool motion_allowed{false};
  std::uint64_t motion_allowed_generation{0U};
  double motion_allowed_received_at_sec{0.0};

  bool safety_status_present{false};
  std::string safety_status;
  std::uint64_t safety_status_generation{0U};
  double safety_status_received_at_sec{0.0};

  bool interlock_present{false};
  std::uint64_t interlock_generation{0U};
  double interlock_received_at_sec{0.0};
  bool interlock_hold_active{false};
  bool interlock_motion_blocked{false};
  bool interlock_effective_motion_blocked{false};
  // Legacy execution-session evidence remains visible so a stale session can
  // block the ordinary hall approach, but new elevator-test transactions do
  // not create one.
  bool execution_session_engaged{false};
  bool execution_lease_active{false};
  bool exact_elevator_mode_contract{false};
  std::string interlock_block_reason;
};

struct ElevatorMotionAdmissionAssessment
{
  ElevatorMotionAdmissionKind kind{ElevatorMotionAdmissionKind::kWaiting};
  bool ready_to_wait_for_nav2{false};
  std::string block_reason;
  std::string detail;
};

ElevatorMotionAdmissionAssessment assess_elevator_motion_admission(
  ElevatorMotionAdmissionScope scope,
  const ElevatorMotionAdmissionEvidence & evidence,
  std::uint64_t minimum_motion_allowed_generation,
  std::uint64_t minimum_safety_status_generation,
  std::uint64_t minimum_interlock_generation,
  double now_sec,
  double max_age_sec) noexcept;

std::optional<ElevatorNavigationRequest>
resolve_elevator_navigation_request(
  const robot_elevator_manager::FrozenElevatorRelease & release,
  const robot_elevator_manager::ElevatorEffect & effect);

// Hall-call goals normally use ordinary Nav2. When an elevator-owned goal is
// already inside the bounded doorway-motion envelope, select the scoped
// four-wheel planner/controller so a blocked startup spin can be replaced by
// checked forward, reverse, or lateral motion. Missing or invalid live pose
// evidence keeps the ordinary profile.
ElevatorNavigationProfile select_elevator_navigation_profile(
  const ElevatorNavigationRequest & request,
  const std::optional<ElevatorMapPose> & current_pose,
  double scoped_max_distance_m) noexcept;

// Returns the exact FollowPath plugin selected by the profile's behavior
// tree. Ordinary Nav2 has no elevator-controller execution session.
std::optional<std::string> elevator_controller_id_for_profile(
  ElevatorNavigationProfile profile,
  robot_elevator_manager::PoseRole target_role);

// The panel-only speed profile is installed alongside the configured direct
// cabin tree. Other cabin roles retain the configured tree unchanged.
std::string elevator_cabin_behavior_tree_for_role(
  const std::string & direct_behavior_tree,
  robot_elevator_manager::PoseRole target_role);

// One immutable elevator effect is one navigation action attempt. Coordinates
// are intentionally absent so retrying an identical pose still produces a new
// controller session.
std::optional<std::string> make_elevator_controller_session_id(
  const std::string & transaction_id,
  std::uint64_t effect_sequence);

std::optional<robot_elevator_manager::ElevatorRuntimePose>
resolve_elevator_navigation_target(
  const robot_elevator_manager::FrozenElevatorRelease & release,
  const robot_elevator_manager::ElevatorEffect & effect);

class DualOdomStopTracker
{
public:
  void reset(
    double linear_threshold_mps = 0.02,
    double angular_threshold_radps = 0.03);
  void observe_wheel(double vx, double vy, double wz, double received_at_sec);
  void observe_local(double vx, double vy, double wz, double received_at_sec);

  bool stopped(
    double now_sec,
    double max_age_sec,
    double settle_sec) const;

private:
  struct Channel
  {
    bool seen{false};
    double received_at_sec{0.0};
    double linear_speed_mps{0.0};
    double angular_speed_radps{0.0};
    std::optional<double> stable_since_sec;
  };

  void observe(
    Channel & channel,
    double vx,
    double vy,
    double wz,
    double received_at_sec);

  mutable std::mutex mutex_;
  double linear_threshold_mps_{0.02};
  double angular_threshold_radps_{0.03};
  Channel wheel_;
  Channel local_;
};

}  // namespace robot_api_server

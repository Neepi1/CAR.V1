#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace robot_localization_bridge
{

struct FloorTransitionIdentity
{
  std::string transaction_id;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
};

struct FloorTransitionCommitEvidence
{
  bool correction_pause_released{false};
  std::uint64_t explicit_relocalization_sequence{0U};
  bool map_odom_valid{false};
  bool correction_active{false};
  std::uint64_t current_sequence{0U};
  std::uint64_t target_sequence{0U};
  std::uint64_t last_published_sequence{0U};
};

struct FloorTransitionPreMutationAbortEvidence
{
  FloorTransitionIdentity source;
  bool source_assets_unchanged{false};
};

struct FloorTransitionContextSnapshot
{
  bool transition_active{false};
  bool runtime_context_valid{true};
  bool failed_locked{false};
  bool recovery_required{false};
  FloorTransitionIdentity pending;
  FloorTransitionIdentity active;
  std::uint64_t begin_explicit_relocalization_sequence{0U};
  std::uint64_t accepted_explicit_relocalization_sequence{0U};
  std::string detail{"LEGACY_CONTEXT_UNSCOPED"};
};

enum class FloorTransitionDecisionCode
{
  kOk,
  kInvalidRequest,
  kPauseUnproven,
  kConflict,
  kIdentityMismatch,
  kExplicitRelocalizationUnproven,
  kMapOdomUnsettled,
  kFailedLocked,
  kStaleCommand,
  kPreMutationUnproven,
};

struct FloorTransitionContextDecision
{
  bool accepted{false};
  bool idempotent{false};
  FloorTransitionDecisionCode code{FloorTransitionDecisionCode::kInvalidRequest};
  std::string message;
  std::uint64_t applied_sequence{0U};
  FloorTransitionContextSnapshot state;
};

class FloorTransitionContext
{
public:
  FloorTransitionContextDecision seed_active_source(
    const FloorTransitionIdentity & identity);

  FloorTransitionContextDecision begin(
    const FloorTransitionIdentity & identity,
    const FloorTransitionIdentity & source,
    bool floor_pause_owned,
    std::uint64_t current_explicit_relocalization_sequence,
    std::uint64_t command_sequence);

  FloorTransitionContextDecision commit(
    const FloorTransitionIdentity & identity,
    const FloorTransitionCommitEvidence & evidence,
    std::uint64_t command_sequence);

  FloorTransitionContextDecision abort(
    const FloorTransitionIdentity & identity,
    std::uint64_t command_sequence);

  FloorTransitionContextDecision abort_pre_mutation(
    const FloorTransitionIdentity & identity,
    const FloorTransitionPreMutationAbortEvidence & evidence,
    std::uint64_t command_sequence);

  bool candidate_allowed(bool explicit_trigger, bool corrections_paused) const;
  FloorTransitionContextSnapshot snapshot() const;

private:
  FloorTransitionContextDecision decision(
    bool accepted,
    bool idempotent,
    FloorTransitionDecisionCode code,
    const std::string & message,
    std::uint64_t applied_sequence = 0U) const;

  bool command_sequence_valid(
    const FloorTransitionIdentity & identity,
    std::uint64_t command_sequence,
    FloorTransitionContextDecision & rejection);

  FloorTransitionContextSnapshot state_;
  std::unordered_map<std::string, std::uint64_t> last_command_sequences_;
  // An ended transaction can never acquire the context again, even with a
  // higher sequence. A new, explicitly paused transaction is required.
  std::unordered_set<std::string> terminated_transactions_;
};

const char * to_string(FloorTransitionDecisionCode code) noexcept;

}  // namespace robot_localization_bridge

#pragma once

#include <cstdint>
#include <string>

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
};

struct FloorTransitionContextDecision
{
  bool accepted{false};
  bool idempotent{false};
  FloorTransitionDecisionCode code{FloorTransitionDecisionCode::kInvalidRequest};
  std::string message;
  FloorTransitionContextSnapshot state;
};

class FloorTransitionContext
{
public:
  FloorTransitionContextDecision begin(
    const FloorTransitionIdentity & identity,
    bool floor_pause_owned,
    std::uint64_t current_explicit_relocalization_sequence);

  FloorTransitionContextDecision commit(
    const FloorTransitionIdentity & identity,
    const FloorTransitionCommitEvidence & evidence);

  FloorTransitionContextDecision abort(const FloorTransitionIdentity & identity);

  bool candidate_allowed(bool explicit_trigger, bool corrections_paused) const;
  FloorTransitionContextSnapshot snapshot() const;

private:
  FloorTransitionContextDecision decision(
    bool accepted,
    bool idempotent,
    FloorTransitionDecisionCode code,
    const std::string & message) const;

  FloorTransitionContextSnapshot state_;
};

const char * to_string(FloorTransitionDecisionCode code) noexcept;

}  // namespace robot_localization_bridge

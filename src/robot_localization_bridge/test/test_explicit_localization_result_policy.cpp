#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "robot_localization_bridge/explicit_localization_result_policy.hpp"

namespace robot_localization_bridge
{
namespace
{

using explicit_localization::IsaacResultAdmission;

struct TargetIdentity
{
  std::string transaction_id{"floor-tx-2"};
  std::string building_id{"B15"};
  std::string floor_id{"F2"};
  std::string map_id{"map_f2"};
  std::uint64_t asset_epoch{14U};
  std::string asset_digest{
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef"};
};

struct LocalizerEvidence
{
  std::string transaction_id{"floor-tx-2"};
  bool success{true};
  bool applying{false};
  bool reloaded{true};
  bool active_identity_valid{true};
  std::string requested_building_id{"B15"};
  std::string requested_floor_id{"F2"};
  std::string requested_map_id{"map_f2"};
  std::uint64_t requested_asset_epoch{14U};
  std::string requested_asset_digest{
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef"};
  std::string active_building_id{"B15"};
  std::string active_floor_id{"F2"};
  std::string active_map_id{"map_f2"};
  std::uint64_t active_asset_epoch{14U};
  std::string active_asset_digest{
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef"};
  std::uint64_t localizer_generation{3U};
  bool localizer_ready{true};
};

TEST(ExplicitLocalizationResultPolicy, UnarmedIsaacResultIsDiagnosticOnly)
{
  EXPECT_EQ(
    explicit_localization::classify_isaac_result(false, 0.0, 100.0, 1.0),
    IsaacResultAdmission::kIgnoreUnarmed);
}

TEST(ExplicitLocalizationResultPolicy, ResultBeforeImmutableArmWindowIsDrained)
{
  EXPECT_EQ(
    explicit_localization::classify_isaac_result(true, 100.0, 98.999, 1.0),
    IsaacResultAdmission::kIgnoreBeforeArm);
  EXPECT_EQ(
    explicit_localization::classify_isaac_result(true, 100.0, 99.0, 1.0),
    IsaacResultAdmission::kEvaluateExplicit);
}

TEST(ExplicitLocalizationResultPolicy, ExplicitResultUsesTimestampedTfInsteadOfWallAgeOnly)
{
  EXPECT_FALSE(explicit_localization::enforce_wall_age_limit(true));
  EXPECT_TRUE(explicit_localization::enforce_wall_age_limit(false));
}

TEST(ExplicitLocalizationResultPolicy, ExactPendingFloorTargetMayCrossIndependentMapOrigin)
{
  explicit_localization::ForcedTranslationLimitContext context;
  context.explicit_trigger = true;
  context.floor_transition_active = true;
  context.runtime_context_valid = false;
  context.floor_transition_failed_locked = false;
  context.exact_pending_target_localizer = true;

  EXPECT_TRUE(explicit_localization::forced_translation_within_limit(
      29.671, 20.0, context));
}

TEST(ExplicitLocalizationResultPolicy, FloorLimitBypassRequiresCompleteTargetProof)
{
  const TargetIdentity target;
  LocalizerEvidence evidence;
  EXPECT_TRUE(explicit_localization::proves_exact_pending_floor_target(
      target, evidence));

  evidence.transaction_id = "older-floor-tx";
  EXPECT_FALSE(explicit_localization::proves_exact_pending_floor_target(
      target, evidence));
  evidence.transaction_id = target.transaction_id;

  evidence.active_map_id = "map_f1";
  EXPECT_FALSE(explicit_localization::proves_exact_pending_floor_target(
      target, evidence));
  evidence.active_map_id = target.map_id;

  evidence.reloaded = false;
  EXPECT_FALSE(explicit_localization::proves_exact_pending_floor_target(
      target, evidence));
}

TEST(ExplicitLocalizationResultPolicy, OrdinaryExplicitRelocalizationKeepsTwentyMetreLimit)
{
  explicit_localization::ForcedTranslationLimitContext context;
  context.explicit_trigger = true;

  EXPECT_FALSE(explicit_localization::forced_translation_within_limit(
      29.671, 20.0, context));
  EXPECT_TRUE(explicit_localization::forced_translation_within_limit(
      13.603, 20.0, context));
}

TEST(ExplicitLocalizationResultPolicy, InexactOrLockedFloorTargetCannotBypassLimit)
{
  explicit_localization::ForcedTranslationLimitContext context;
  context.explicit_trigger = true;
  context.floor_transition_active = true;
  context.runtime_context_valid = false;

  context.exact_pending_target_localizer = false;
  EXPECT_FALSE(explicit_localization::forced_translation_within_limit(
      29.671, 20.0, context));

  context.exact_pending_target_localizer = true;
  context.floor_transition_failed_locked = true;
  EXPECT_FALSE(explicit_localization::forced_translation_within_limit(
      29.671, 20.0, context));
}

}  // namespace
}  // namespace robot_localization_bridge

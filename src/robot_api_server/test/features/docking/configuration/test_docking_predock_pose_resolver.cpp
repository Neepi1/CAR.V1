#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "robot_api_server/features/docking/configuration/docking_predock_pose_resolver.hpp"

namespace robot_api_server::features::docking::configuration
{
namespace
{

StoredPose pose(
  std::string id,
  std::string type = "dock_predock",
  std::string name = "",
  const double x = 0.0,
  const double y = 0.0,
  const double yaw = 0.0)
{
  StoredPose result;
  result.id = std::move(id);
  result.type = std::move(type);
  result.name = std::move(name);
  result.x = x;
  result.y = y;
  result.yaw = yaw;
  return result;
}

class ResolverHarness
{
public:
  DockingPredockPoseResolverPorts ports()
  {
    DockingPredockPoseResolverPorts result;
    result.find_pose = [this](
      const std::string &,
      const std::string &,
      const std::string & pose_id) -> std::optional<StoredPose>
      {
        const auto match = std::find_if(
          poses.begin(), poses.end(),
          [&pose_id](const StoredPose & item) {return item.id == pose_id;});
        return match == poses.end() ?
               std::nullopt : std::optional<StoredPose>(*match);
      };
    result.read_poses = [this](const std::string &, const std::string &) {
        return poses;
      };
    return result;
  }

  std::vector<StoredPose> poses;
};

std::optional<StoredPose> resolve(
  const DockingPredockPoseResolver & resolver,
  const std::string & requested,
  const StoredPose & dock,
  std::string & source,
  std::string & error,
  int & error_status)
{
  return resolver.resolve(
    "B15", "F1", "dock_a", dock, requested,
    source, error, error_status);
}

TEST(DockingPredockPoseResolverTest, MissingPortsAreRejected)
{
  EXPECT_THROW(
    DockingPredockPoseResolver({}, {}),
    std::invalid_argument);
}

TEST(DockingPredockPoseResolverTest, UnsafeExplicitIdIsBadRequest)
{
  ResolverHarness harness;
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  EXPECT_FALSE(resolve(resolver, "../escape", pose("dock_a"), source, error, error_status));
  EXPECT_EQ(error_status, 400);
  EXPECT_EQ(error, "valid predock_pose_id is required");
}

TEST(DockingPredockPoseResolverTest, MissingExplicitIdIsNotFound)
{
  ResolverHarness harness;
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  EXPECT_FALSE(resolve(resolver, "missing", pose("dock_a"), source, error, error_status));
  EXPECT_EQ(error_status, 404);
  EXPECT_EQ(error, "predock_pose_id not found in poses.yaml: missing");
}

TEST(DockingPredockPoseResolverTest, ExplicitPoseHasHighestPriority)
{
  ResolverHarness harness;
  harness.poses = {pose("chosen"), pose("dock_a_predock")};
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  const auto result = resolve(
    resolver, "chosen", pose("dock_a"), source, error, error_status);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->id, "chosen");
  EXPECT_EQ(source, "manual_predock_explicit");
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(error_status, 0);
}

TEST(DockingPredockPoseResolverTest, ConventionalIdOrderIsPreserved)
{
  ResolverHarness harness;
  harness.poses = {pose("approach_dock_a"), pose("dock_a_predock")};
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  const auto result = resolve(
    resolver, "", pose("dock_a"), source, error, error_status);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->id, "dock_a_predock");
  EXPECT_EQ(source, "manual_predock_auto_id");
}

TEST(DockingPredockPoseResolverTest, UniqueCaseInsensitiveDockNameMatchIsSelected)
{
  ResolverHarness harness;
  harness.poses = {pose("manual_a", "PREDOCK", "Main Charger_Approach")};
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  const auto result = resolve(
    resolver, "", pose("dock_a", "dock", "Main Charger"),
    source, error, error_status);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->id, "manual_a");
  EXPECT_EQ(source, "manual_predock_auto_name");
}

TEST(DockingPredockPoseResolverTest, AmbiguousNamedMatchesRequireExplicitId)
{
  ResolverHarness harness;
  harness.poses = {
    pose("manual_a", "predock", "dock_a_predock"),
    pose("manual_b", "dock_approach", "DOCK_A_PREDOCK")};
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  EXPECT_FALSE(resolve(resolver, "", pose("dock_a"), source, error, error_status));
  EXPECT_EQ(error_status, 409);
  EXPECT_EQ(
    error,
    "multiple matching dock predock poses found; pass predock_pose_id explicitly");
}

TEST(DockingPredockPoseResolverTest, OneTypedPoseIsTheUniqueTypeFallback)
{
  ResolverHarness harness;
  harness.poses = {pose("manual_a", "dock_predock", "unrelated")};
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  const auto result = resolve(
    resolver, "", pose("dock_a"), source, error, error_status);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->id, "manual_a");
  EXPECT_EQ(source, "manual_predock_auto_unique_type");
}

TEST(DockingPredockPoseResolverTest, MultipleTypedFallbacksAreConflict)
{
  ResolverHarness harness;
  harness.poses = {
    pose("manual_a", "dock_predock", "unrelated-a"),
    pose("manual_b", "predock", "unrelated-b")};
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string source;
  std::string error;
  int error_status = 0;

  EXPECT_FALSE(resolve(resolver, "", pose("dock_a"), source, error, error_status));
  EXPECT_EQ(error_status, 409);
  EXPECT_EQ(
    error,
    "multiple dock_predock poses found; pass predock_pose_id or name one dock_a_predock");
}

TEST(DockingPredockPoseResolverTest, DisabledDistanceCheckStillEnforcesYaw)
{
  ResolverHarness harness;
  DockingPredockPoseResolver resolver({}, harness.ports());
  std::string error;

  EXPECT_TRUE(
    resolver.validate(
      pose("dock", "dock", "", 0.0, 0.0, 0.0),
      pose("predock", "predock", "", 5.0, 0.0, 0.7),
      error));
  EXPECT_FALSE(
    resolver.validate(
      pose("dock", "dock", "", 0.0, 0.0, 0.0),
      pose("predock", "predock", "", 1.0, 0.0, 1.0),
      error));
  EXPECT_NE(error.find("distance_check=disabled"), std::string::npos);
  EXPECT_NE(error.find("heading aligned to the charger"), std::string::npos);
}

TEST(DockingPredockPoseResolverTest, EnabledDistanceWindowIsEnforced)
{
  ResolverHarness harness;
  DockingPredockPoseResolverConfig config;
  config.distance_check_enabled = true;
  config.min_distance_m = 0.50;
  config.max_distance_m = 1.20;
  config.max_yaw_error_rad = 0.80;
  DockingPredockPoseResolver resolver(config, harness.ports());
  std::string error;

  EXPECT_FALSE(
    resolver.validate(
      pose("dock", "dock", "", 0.0, 0.0, 0.0),
      pose("predock", "predock", "", 0.25, 0.0, 0.0),
      error));
  EXPECT_NE(error.find("allowed=[0.500,1.200]"), std::string::npos);
}

}  // namespace
}  // namespace robot_api_server::features::docking::configuration

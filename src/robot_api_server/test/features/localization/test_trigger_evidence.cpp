#include <atomic>
#include <thread>
#include "gtest/gtest.h"
#include "robot_api_server/features/localization/trigger_evidence.hpp"

using robot_api_server::features::localization::TriggerEvidence;
using robot_api_server::features::localization::TriggerEvidenceRegistry;
using Status = TriggerEvidence::Status;

TEST(TriggerEvidence, UnknownSurvivesHttpLifetimeAndLateSuccessResolvesOnce)
{
  std::atomic<int> pending{0};
  TriggerEvidenceRegistry registry;
  auto evidence = std::make_shared<TriggerEvidence>("a", [&] {++pending;}, [&] {--pending;});
  registry.insert(evidence);
  Status status;
  status.request_id = "a";
  status.state = Status::UNKNOWN;
  registry.observe(status);
  EXPECT_EQ(pending, 1);
  evidence.reset();
  status.state = Status::SUCCEEDED;
  status.outcome_known = true;
  status.baseline_explicit_sequence = 2;
  status.accepted_explicit_sequence = 3;
  std::thread callback([&] {registry.observe(status);});
  registry.observe(status);
  callback.join();
  EXPECT_EQ(pending, 0);
}

TEST(TriggerEvidence, RejectsForeignIdUnprovenSuccessAndTerminalRegression)
{
  int pending = 0;
  TriggerEvidence evidence("a", [&] {++pending;}, [&] {--pending;});
  Status status;
  status.request_id = "b";
  status.outcome_known = true;
  status.state = Status::FAILED;
  evidence.observe(status);
  EXPECT_EQ(pending, 1);
  status.request_id = "a";
  status.state = Status::SUCCEEDED;
  evidence.observe(status);
  EXPECT_EQ(pending, 1);
  status.state = Status::FAILED;
  evidence.observe(status);
  evidence.observe(status);
  EXPECT_EQ(pending, 0);
  status.state = Status::UNKNOWN;
  status.outcome_known = false;
  evidence.observe(status);
  EXPECT_TRUE(evidence.snapshot().outcome_known);
}

TEST(TriggerEvidence, CompletionNeverClearsOtherRequests)
{
  int pending = 0;
  TriggerEvidenceRegistry registry;
  auto started = [&] {++pending;};
  auto resolved = [&] {--pending;};
  registry.insert(std::make_shared<TriggerEvidence>("a", started, resolved));
  registry.insert(std::make_shared<TriggerEvidence>("b", started, resolved));
  Status status;
  status.request_id = "a";
  status.state = Status::FAILED;
  status.outcome_known = true;
  registry.observe(status);
  EXPECT_EQ(pending, 1);
  registry.observe(status);
  EXPECT_EQ(pending, 1);
}

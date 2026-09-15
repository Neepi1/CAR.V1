#include <gtest/gtest.h>
#include <chrono>
#include <filesystem>
#include <future>
#include <stdexcept>
#include <unistd.h>

#include "robot_api_server/features/mapping/runtime/mapping_save_job.hpp"

using robot_api_server::HttpResponse;
using robot_api_server::json_bool_value;
using robot_api_server::json_string_value;
using robot_api_server::features::mapping::runtime::MappingSaveJob;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

class MappingSaveJobTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/mapping_save_job_XXXXXX";
    const auto directory = mkdtemp(pattern);
    ASSERT_NE(directory, nullptr);
    root = directory;
  }
  void TearDown() override {fs::remove_all(root);}
  fs::path root;
};

TEST_F(MappingSaveJobTest, PollAndDuplicateDoNotWaitForShutdownOrSaveTwice)
{
  MappingSaveJob job(root);
  std::promise<void> reached, finish;
  auto released = finish.get_future().share();
  int writes = 0;
  auto response = job.submit("request_1", "B10:F10:map", [&](const auto & saved) {
      ++writes;
      saved("{\"map_id\":\"one\",\"map_saved\":true}");
      reached.set_value();
      released.wait();
      return HttpResponse{200, "application/json",
        "{\"map_id\":\"one\",\"map_saved\":true,\"mapping_stopped\":true}"};
    });
  const auto reached_status = reached.get_future().wait_for(2s);
  auto poll = std::async(std::launch::async, [&] {return job.find("request_1");});
  const auto poll_status = poll.wait_for(100ms);
  auto duplicate = std::async(std::launch::async, [&] {
      return job.submit("request_1", "B10:F10:map", [&](const auto &) {
          ++writes; return HttpResponse{};
        });
    });
  const auto duplicate_status = duplicate.wait_for(100ms);
  finish.set_value(); // Release before any ASSERT so regressions cannot hang.
  job.join();
  EXPECT_EQ(response.status, 202);
  EXPECT_EQ(reached_status, std::future_status::ready);
  EXPECT_EQ(poll_status, std::future_status::ready);
  EXPECT_EQ(duplicate_status, std::future_status::ready);
  EXPECT_TRUE(json_bool_value(poll.get().body, "map_saved", false));
  EXPECT_EQ(duplicate.get().status, 200);
  EXPECT_EQ(writes, 1);
  EXPECT_TRUE(json_bool_value(job.find("request_1").body, "mapping_stopped", false));
  EXPECT_EQ(job.submit("request_1", "different", {}).status, 409);
}

TEST_F(MappingSaveJobTest, StopFailureRetainsCommittedMapAcrossReconstruction)
{
  {
    MappingSaveJob job(root);
    job.submit("request_2", "key", [](const auto & saved) -> HttpResponse {
        saved("{\"map_id\":\"persisted\",\"map_saved\":true}");
        throw std::runtime_error("shutdown failed");
      });
    job.join();
    EXPECT_EQ(json_string_value(job.find("request_2").body, "state"), "failed");
  }
  MappingSaveJob restored(root);
  const auto result = restored.find("request_2");
  EXPECT_EQ(result.status, 200);
  EXPECT_TRUE(json_bool_value(result.body, "map_saved", false));
  EXPECT_FALSE(json_bool_value(result.body, "mapping_stopped", false));
  EXPECT_NE(result.body.find("persisted"), std::string::npos);
  EXPECT_EQ(restored.submit("request_2", "key", {}).status, 200);
}

TEST_F(MappingSaveJobTest, FailedWriteIsNotSavedAndCannotBeSilentlyRepeated)
{
  MappingSaveJob job(root);
  job.submit("request_3", "key", [](const auto &) {
      return HttpResponse{500, "application/json", "{\"error\":\"disk failed\"}"};
    });
  job.join();
  EXPECT_FALSE(json_bool_value(job.find("request_3").body, "map_saved", false));
  EXPECT_EQ(job.submit("request_3", "key", {}).status, 200);
  EXPECT_EQ(job.find("../escape").status, 400);
}

TEST_F(MappingSaveJobTest, InterruptedPriorProcessIsNotRetriedOrReportedFinished)
{
  robot_api_server::features::maps::durable_write_text_file_atomic(root / "old.json",
    "{\"request_id\":\"old\",\"request_key\":\"key\",\"state\":\"running\","
    "\"map_saved\":true,\"mapping_stopped\":false}");
  MappingSaveJob restored(root);
  const auto result = restored.submit("old", "key", {});
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(json_string_value(result.body, "state"), "recovery_required");
  EXPECT_FALSE(restored.running());
}

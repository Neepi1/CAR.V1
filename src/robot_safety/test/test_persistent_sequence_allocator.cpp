#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "robot_safety/persistent_sequence_allocator.hpp"

namespace robot_safety
{
namespace
{

class TemporarySequenceState
{
public:
  TemporarySequenceState()
  {
    root_ = std::filesystem::temp_directory_path() /
      ("njrh_sequence_allocator_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root_);
  }

  ~TemporarySequenceState()
  {
    std::error_code ignored;
    std::filesystem::remove_all(root_, ignored);
  }

  std::filesystem::path file() const
  {
    return root_ / "sequence.state";
  }

private:
  std::filesystem::path root_;
};

TEST(PersistentSequenceAllocator, ReplacementProcessStartsAboveOldProcessLimit)
{
  TemporarySequenceState state;
  PersistentSequenceAllocator first(state.file(), 32U);
  const auto old_first = first.next();
  ASSERT_TRUE(old_first.has_value());

  PersistentSequenceAllocator replacement(state.file(), 32U);
  const auto new_first = replacement.next();
  ASSERT_TRUE(new_first.has_value());
  EXPECT_GT(*new_first, first.limit());
}

TEST(PersistentSequenceAllocator, NeverIssuesBeyondReservedBlock)
{
  TemporarySequenceState state;
  PersistentSequenceAllocator allocator(state.file(), 4U);
  ASSERT_TRUE(allocator.next().has_value());
  ASSERT_TRUE(allocator.next().has_value());
  ASSERT_TRUE(allocator.next().has_value());
  EXPECT_FALSE(allocator.next().has_value());
}

TEST(PersistentSequenceAllocator, SynchronizeAdvancesWithinBlockAndFailsClosedPastIt)
{
  TemporarySequenceState state;
  PersistentSequenceAllocator allocator(state.file(), 16U);
  allocator.synchronize(allocator.base() + 8U);
  const auto next = allocator.next();
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(*next, allocator.base() + 9U);

  allocator.synchronize(allocator.limit() + 100U);
  EXPECT_FALSE(allocator.next().has_value());
}

TEST(PersistentSequenceAllocator, KeepsCompleteHighWatermarkAcrossPartialTail)
{
  TemporarySequenceState state;
  PersistentSequenceAllocator first(state.file(), 32U);
  const auto first_base = first.base();
  {
    std::ofstream stream(state.file(), std::ios::app | std::ios::binary);
    ASSERT_TRUE(stream.good());
    stream << "123";
  }

  PersistentSequenceAllocator replacement(state.file(), 32U);
  EXPECT_GT(replacement.base(), first.limit());

  std::ifstream stream(state.file(), std::ios::binary);
  ASSERT_TRUE(stream.good());
  const std::string journal{
    std::istreambuf_iterator<char>(stream),
    std::istreambuf_iterator<char>()};
  EXPECT_EQ(
    journal.rfind("R:" + std::to_string(first_base) + "\n", 0U), 0U);
  EXPECT_NE(journal.find("123!discard\n"), std::string::npos);

  PersistentSequenceAllocator third(state.file(), 32U);
  EXPECT_GT(third.base(), replacement.limit());
}

}  // namespace
}  // namespace robot_safety

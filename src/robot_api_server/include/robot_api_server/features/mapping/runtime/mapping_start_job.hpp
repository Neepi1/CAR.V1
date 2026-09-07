#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <sys/types.h>

namespace robot_api_server::features::mapping::runtime
{

struct MappingStartJobSnapshot
{
  std::uint64_t id{0U};
  std::string state{"idle"};
  std::string phase{"idle"};
  std::string detail;
  std::string started_at;
  std::string finished_at;
  bool navigation_was_active{false};
  bool navigation_cancel_ok{false};
  bool navigation_stop_ok{false};
  bool cancel_requested{false};
  pid_t mapping_pid{-1};
};

std::string mapping_start_job_json(const MappingStartJobSnapshot & job);

class MappingStartJobTracker
{
public:
  MappingStartJobSnapshot snapshot() const;
  std::string json() const;
  bool running() const;

  std::optional<std::uint64_t> begin(
    bool navigation_was_active,
    const std::string & started_at);
  bool request_cancel(const std::string & detail);
  bool cancel_requested(std::uint64_t job_id) const;
  bool set_phase(
    std::uint64_t job_id,
    const std::string & phase,
    const std::string & detail = "");
  bool set_navigation_result(
    std::uint64_t job_id,
    bool cancel_ok,
    bool stop_ok);
  bool finish(
    std::uint64_t job_id,
    const std::string & state,
    const std::string & detail,
    pid_t mapping_pid,
    const std::string & finished_at);

private:
  mutable std::mutex mutex_;
  MappingStartJobSnapshot job_;
  std::uint64_t sequence_{0U};
};

}  // namespace robot_api_server::features::mapping::runtime

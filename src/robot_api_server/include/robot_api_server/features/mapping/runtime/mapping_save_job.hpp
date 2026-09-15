#pragma once

#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::mapping::runtime
{

// A mapping save owns its frozen grid until assets and shutdown have completed.
// Only one save worker exists; HTTP polling never joins it or waits for shutdown.
class MappingSaveJob
{
public:
  using Saved = std::function<void(const std::string &)>;
  using Work = std::function<HttpResponse(const Saved &)>;

  explicit MappingSaveJob(std::filesystem::path directory)
  : directory_(std::move(directory)) {}
  ~MappingSaveJob() {join();}

  static bool valid_id(const std::string & id)
  {
    return !id.empty() && id.size() <= 96U &&
           id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") ==
           std::string::npos;
  }

  bool running() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
  }

  void join()
  {
    if (worker_.joinable()) {worker_.join();}
  }

  // Caller serializes submit/start/stop within the mapping module only.
  HttpResponse submit(const std::string & id, const std::string & key, Work work)
  {
    if (!valid_id(id)) {
      return {400, "application/json", error_json("valid request_id is required")};
    }
    if (const auto previous = find(id); previous.status != 404) {
      const auto prior_key = json_string_value(previous.body, "request_key");
      if (!prior_key || *prior_key != key) {
        return {409, "application/json", error_json("request_id belongs to a different save")};
      }
      return previous;
    }
    if (running()) {
      return {409, "application/json", error_json("another mapping save is still running")};
    }
    join();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      id_ = id;
      key_ = key;
      state_ = "running";
      phase_ = "saving";
      result_ = "{}";
      result_status_ = 0;
      map_saved_ = false;
      mapping_stopped_ = false;
      running_ = true;
      try {
        persist_locked();
      } catch (...) {
        running_ = false;
        state_ = "failed";
        throw; // No worker or map write was started.
      }
    }
    try {
      worker_ = std::thread([this, work = std::move(work)]() {
          HttpResponse result;
          try {
            result = work([this](const std::string & saved_result) {
                std::lock_guard<std::mutex> lock(mutex_);
                map_saved_ = true;
                phase_ = "stopping";
                result_ = saved_result;
                persist_locked(); // Commit proof before entering slow shutdown.
              });
          } catch (const std::exception & error) {
            result = {500, "application/json", error_json(error.what())};
          } catch (...) {
            result = {500, "application/json", error_json("mapping save worker failed")};
          }
          std::lock_guard<std::mutex> lock(mutex_);
          result_status_ = result.status;
          if (result.status >= 200 && result.status < 300) {
            result_ = result.body;
            map_saved_ = json_bool_value(result.body, "map_saved", false);
            mapping_stopped_ = json_bool_value(result.body, "mapping_stopped", false);
          } else if (!map_saved_) {
            result_ = result.body;
          }
          state_ = result.status >= 200 && result.status < 300 ? "finished" : "failed";
          phase_ = "finished";
          running_ = false;
          try {persist_locked();} catch (...) {
            // Preserve the committed result in memory. The prior durable record
            // remains queryable after restart as recovery_required, never success.
            state_ = "recovery_required";
          }
        });
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
      state_ = "failed";
      phase_ = "worker_start_failed";
      persist_locked();
      throw;
    }
    auto response = find(id);
    response.status = 202;
    return response;
  }

  HttpResponse find(const std::string & id) const
  {
    if (!valid_id(id)) {
      return {400, "application/json", error_json("valid request_id is required")};
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (id == id_) {return {200, "application/json", json_locked()};}
    }
    const auto path = directory_ / (id + ".json");
    if (!std::filesystem::exists(path)) {
      return {404, "application/json", error_json("mapping save request not found")};
    }
    auto body = maps::read_regular_text_file_checked(path, 256U * 1024U);
    // A prior process's unfinished task must not appear to keep running forever,
    // nor may a retry create a second map or stop a newly started mapping session.
    const std::string running_state = "\"state\":\"running\"";
    if (const auto position = body.find(running_state); position != std::string::npos) {
      body.replace(position, running_state.size(), "\"state\":\"recovery_required\"");
    }
    return {200, "application/json", body};
  }

private:
  std::string json_locked() const
  {
    return "{\"ok\":true,\"accepted\":true,\"request_id\":" + json_string(id_) +
           ",\"request_key\":" + json_string(key_) +
           ",\"state\":" + json_string(state_) +
           ",\"phase\":" + json_string(phase_) +
           ",\"map_saved\":" + (map_saved_ ? "true" : "false") +
           ",\"mapping_stopped\":" + (mapping_stopped_ ? "true" : "false") +
           ",\"result_status\":" + std::to_string(result_status_) +
           ",\"result\":" + result_ + "}";
  }

  void persist_locked() const
  {
    std::filesystem::create_directories(directory_);
    maps::durable_write_text_file_atomic(directory_ / (id_ + ".json"), json_locked());
  }

  const std::filesystem::path directory_;
  mutable std::mutex mutex_;
  std::thread worker_;
  std::string id_, key_, state_, phase_;
  std::string result_{"{}"};
  int result_status_{0};
  bool running_{false}, map_saved_{false}, mapping_stopped_{false};
};

}  // namespace robot_api_server::features::mapping::runtime

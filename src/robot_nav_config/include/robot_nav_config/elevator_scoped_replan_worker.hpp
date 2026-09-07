#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_search.hpp"

namespace robot_nav_config {

struct ElevatorScopedReplanRequest {
  std::size_t plan_generation{0U};
  std::uint64_t request_signature{0U};
  std::string costmap_frame;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  std::vector<geometry_msgs::msg::Point> footprint;
  ElevatorScopedPose start;
  ElevatorScopedPose goal;
  ElevatorScopedSearchParameters parameters;
};

struct ElevatorScopedReplanResult {
  std::size_t plan_generation{0U};
  std::uint64_t request_signature{0U};
  std::string costmap_frame;
  ElevatorScopedPose goal;
  ElevatorScopedSearchResult search;
};

class ElevatorScopedReplanWorker {
public:
  using SearchExecutor = std::function<ElevatorScopedSearchResult(
      const ElevatorScopedReplanRequest &)>;

  explicit ElevatorScopedReplanWorker(SearchExecutor executor = {});
  ~ElevatorScopedReplanWorker();

  ElevatorScopedReplanWorker(const ElevatorScopedReplanWorker &) = delete;
  ElevatorScopedReplanWorker &
  operator=(const ElevatorScopedReplanWorker &) = delete;

  void start();
  void stop();
  bool submit(ElevatorScopedReplanRequest request);
  std::optional<ElevatorScopedReplanResult> take_result();
  bool busy() const;

private:
  void run();

  SearchExecutor executor_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread thread_;
  bool stop_requested_{false};
  bool running_{false};
  std::optional<ElevatorScopedReplanRequest> pending_request_;
  std::optional<ElevatorScopedReplanResult> completed_result_;
};

} // namespace robot_nav_config

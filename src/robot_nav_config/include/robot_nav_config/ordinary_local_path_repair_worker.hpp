#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>

#include "nav2_util/lifecycle_node.hpp"
#include "robot_nav_config/ordinary_local_path_repair.hpp"

namespace robot_nav_config
{

struct OrdinaryLocalPathRepairWork
{
  std::size_t plan_generation{0U};
  OrdinaryLocalPathRepairRequest request;
};

struct OrdinaryLocalPathRepairWorkResult
{
  std::size_t plan_generation{0U};
  OrdinaryLocalPathRepairRequest request;
  OrdinaryLocalPathRepairResult repair;
  std::exception_ptr error;
};

class OrdinaryLocalPathRepairWorker
{
public:
  using RepairExecutor = std::function<OrdinaryLocalPathRepairResult(
      const OrdinaryLocalPathRepairRequest &)>;

  OrdinaryLocalPathRepairWorker(
    nav2_util::LifecycleNode::SharedPtr node,
    RepairExecutor executor = {});
  ~OrdinaryLocalPathRepairWorker();

  OrdinaryLocalPathRepairWorker(const OrdinaryLocalPathRepairWorker &) = delete;
  OrdinaryLocalPathRepairWorker & operator=(
    const OrdinaryLocalPathRepairWorker &) = delete;

  void start();
  void stop();
  bool submit(OrdinaryLocalPathRepairWork work);
  std::optional<OrdinaryLocalPathRepairWorkResult> take_result();
  bool busy() const;

private:
  void run();

  nav2_util::LifecycleNode::SharedPtr node_;
  RepairExecutor executor_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread thread_;
  bool stop_requested_{false};
  bool running_{false};
  std::optional<OrdinaryLocalPathRepairWork> pending_work_;
  std::optional<OrdinaryLocalPathRepairWorkResult> completed_result_;
};

}  // namespace robot_nav_config

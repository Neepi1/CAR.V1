#include <memory>
#include <string>
#include <sstream>
#include "std_msgs/msg/string.hpp"
#include "robot_nav_config/navigation_recovery/recovery_loop.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "robot_nav_config/srv/prepare_ordinary_navigation_recovery.hpp"

namespace robot_nav_config::navigation_recovery
{
class OrdinaryFollowPath : public nav2_behavior_tree::FollowPathAction
{
public:
  OrdinaryFollowPath(const std::string & name, const BT::NodeConfiguration & config)
  : FollowPathAction(name, "follow_path", config) {}

  static BT::PortsList providedPorts()
  {
    auto ports = FollowPathAction::providedPorts();
    ports.insert(BT::OutputPort<bool>("aborted"));
    ports.insert(BT::OutputPort<builtin_interfaces::msg::Time>("attempt_started"));
    return ports;
  }

  void on_tick() override
  {
    setOutput("aborted", false);
    const builtin_interfaces::msg::Time started = node_->now();
    setOutput("attempt_started", started);
    FollowPathAction::on_tick();
  }

  BT::NodeStatus on_aborted() override
  {
    setOutput("aborted", true);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override
  {
    setOutput("aborted", false);
    return BT::NodeStatus::FAILURE;
  }
};

using Prepare = robot_nav_config::srv::PrepareOrdinaryNavigationRecovery;
class RosRecoveryMonitor : public RecoveryMonitor
{
public:
  explicit RosRecoveryMonitor(const BT::NodeConfiguration & config)
  : node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    client_ = node_->create_client<Prepare>(
      "/controller_server/prepare_ordinary_navigation_recovery",
      rmw_qos_profile_services_default, group_);
    publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/navigation/ordinary_recovery_status", rclcpp::QoS(1).reliable());
  }
  ~RosRecoveryMonitor() override {invalidate();}
  double now() const override
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void invalidate() override
  {
    if (pending_) {client_->remove_pending_request(request_id_);}
    pending_ = false;
    cached_.reset();
    next_query_ = 0;
  }
  std::optional<RecoveryState::Observation> sample(
    const nav_msgs::msg::Path & path, const builtin_interfaces::msg::Time & started) override
  {
    if (path.poses.empty() || rclcpp::Time(started).nanoseconds() <= 0) {return std::nullopt;}
    if (started != attempt_ || path.poses.back().pose != endpoint_ || path.header.frame_id != frame_) {
      invalidate();
      attempt_ = started;
      endpoint_ = path.poses.back().pose;
      frame_ = path.header.frame_id;
    }
    executor_.spin_some();
    if (pending_ && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto result = future_.get();
      cached_ = RecoveryState::Observation{
        result->matched, result->recoverable, result->waiting, result->progress_epoch};
      received_ = now();
      pending_ = false;
    }
    if (pending_ && now() - sent_ > 1.0) {invalidate();}
    if (!pending_ && now() >= next_query_ && client_->service_is_ready()) {
      auto request = std::make_shared<Prepare::Request>();
      request->inspect_only = true;
      request->attempt_started = started;
      // Inspection needs only the endpoint; do not send the whole path at 2 Hz.
      request->path.header = path.header;
      request->path.poses.push_back(path.poses.back());
      auto sent = client_->async_send_request(request);
      future_ = sent.future.share();
      request_id_ = sent.request_id;
      pending_ = true;
      sent_ = now();
      next_query_ = sent_ + 0.5;
    }
    return cached_ && now() - received_ <= 1.5 ? cached_ : std::nullopt;
  }
  void report(const geometry_msgs::msg::PoseStamped & goal, const std::string & phase,
    unsigned retries) override
  {
    const auto goal_stamp = rclcpp::Time(goal.header.stamp).nanoseconds();
    if (phase == last_phase_ && goal_stamp == last_goal_ && now() < next_report_) {return;}
    if (phase != last_phase_ || goal_stamp != last_goal_) {
      RCLCPP_INFO(node_->get_logger(), "[ordinary-recovery] task=%ld phase=%s retries=%u",
        static_cast<long>(goal_stamp), phase.c_str(), retries);
    }
    last_phase_ = phase;
    last_goal_ = goal_stamp;
    next_report_ = now() + 0.5;
    std::ostringstream out;
    out << "{\"version\":1,\"goal_stamp_ns\":" << goal_stamp <<
      ",\"stamp_ns\":" << node_->now().nanoseconds() << ",\"phase\":\"" << phase <<
      "\",\"retries\":" << retries << ",\"progress_epoch\":" <<
      (cached_ ? cached_->progress_epoch : 0) << "}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    publisher_->publish(msg);
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Prepare>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Client<Prepare>::SharedFuture future_;
  std::int64_t request_id_{0}, last_goal_{0};
  bool pending_{false};
  double next_query_{0}, sent_{0}, received_{0}, next_report_{0};
  builtin_interfaces::msg::Time attempt_;
  geometry_msgs::msg::Pose endpoint_;
  std::string frame_, last_phase_;
  std::optional<RecoveryState::Observation> cached_;
};

class PrepareOrdinaryRecovery : public nav2_behavior_tree::BtServiceNode<Prepare>
{
public:
  PrepareOrdinaryRecovery(const std::string & name, const BT::NodeConfiguration & config)
  : BtServiceNode(name, config, "/controller_server/prepare_ordinary_navigation_recovery") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<builtin_interfaces::msg::Time>("attempt_started")});
  }

  void on_tick() override
  {
    getInput("path", request_->path);
    getInput("attempt_started", request_->attempt_started);
    request_->task_started = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal").header.stamp;
  }

  BT::NodeStatus on_completion(std::shared_ptr<Prepare::Response> response) override
  {
    RCLCPP_WARN(node_->get_logger(), "[ordinary-recovery] prepared=%s reason=%s poses=%zu",
      response->prepared ? "true" : "false", response->reason.c_str(), request_->path.poses.size());
    if (response->prepared) {
      increment_recovery_count();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace robot_nav_config::navigation_recovery

BT_REGISTER_NODES(factory)
{
  using namespace robot_nav_config::navigation_recovery;
  factory.registerBuilder<OrdinaryRecoveryLoop>("OrdinaryRecoveryLoop",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<OrdinaryRecoveryLoop>(name, config,
        std::make_unique<RosRecoveryMonitor>(config));
    });
  factory.registerNodeType<OrdinaryFollowPath>("OrdinaryFollowPath");
  factory.registerNodeType<PrepareOrdinaryRecovery>("PrepareOrdinaryRecovery");
  // The Ranger planner preserves the exact commanded endpoint. During a retry,
  // retain the prepared path unless the outer goal was explicitly replaced.
  factory.registerSimpleCondition("OrdinaryPathTargetsGoal", [](BT::TreeNode & node) {
      nav_msgs::msg::Path path;
      geometry_msgs::msg::PoseStamped goal;
      if (!node.getInput("path", path) || !node.getInput("goal", goal)) {
        return BT::NodeStatus::FAILURE;
      }
      return !path.poses.empty() && path.header.frame_id == goal.header.frame_id &&
             path.poses.back().pose == goal.pose ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }, {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")});
  factory.registerSimpleCondition("OrdinaryFollowAborted", [](BT::TreeNode & node) {
      bool aborted = false;
      node.getInput("aborted", aborted);
      return aborted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }, {BT::InputPort<bool>("aborted")});
}

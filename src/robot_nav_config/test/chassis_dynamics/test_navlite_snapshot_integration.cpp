// Execute ONLY in private network + IPC + mount namespaces, with private /dev/shm.
#include <cassert>
#include <fstream>
#include <iostream>
#include <xtensor/xrandom.hpp>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#define NAVLITE_SNAPSHOT_REQUEST_PATH "/tmp/njrh_reports/navlite_snapshot_integration/production_request.json"
#include "robot_nav_config/navlite_snapshot_replay.hpp"
#ifndef NAVLITE_MPPI_IMPLEMENTATION
#define NAVLITE_MPPI_IMPLEMENTATION "../../src/chassis_dynamics/mppi_controller.cpp"
#endif
#include NAVLITE_MPPI_IMPLEMENTATION

using namespace robot_nav_config;
namespace ns=robot_nav_config::navlite_snapshot;

struct Trial
{
  std::vector<std::array<double,6>> commands;
  std::string failure;
};

Trial run_trial(bool record, const std::shared_ptr<nav2_util::LifecycleNode> & node,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & map, nav2_core::GoalChecker * checker,
  const nav_msgs::msg::Path & plan)
{
  xt::random::seed(12345);
  auto handler=std::make_unique<mppi::ParametersHandler>(node);
  RangerOptimizer optimizer; optimizer.initialize(node,"FollowPath",map,handler.get());
  chassis_dynamics::Parameters parameters;
  optimizer.install_model(parameters,{0.55,0.2,0.9},{-0.95,-0.3,-1.1});
  geometry_msgs::msg::Twist speed; speed.linear.x=0.45; speed.angular.z=0.1;
  auto frame=std::make_unique<ns::Frame>(); Trial result;
  const auto output=std::filesystem::path("/tmp/njrh_reports/navlite_snapshot_integration");
  std::filesystem::create_directories(output);
  ns::Recorder recorder((output/"request.json").string(),output.string());
  if(record) {
    std::ofstream f(output/"request.json");
    f<<nlohmann::json{{"schema",1},{"session","integration_"+std::to_string(getpid())},
      {"output_dir",output.string()},{"max_bytes",16777216},
      {"deadline_monotonic_ns",ns::Recorder::monotonic_ns()+10000000000LL}}; f.close();
    for(int i=0;i<200&&!recorder.enabled();++i) {std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    assert(recorder.enabled());
  }
  for(unsigned i=0;i<5;++i) {
    ns::Frame * snapshot=record?recorder.begin(i):nullptr;
    if(snapshot) {
      snapshot->model_enabled=true; snapshot->pose=pose_values(plan.poses.front().pose);
      snapshot->odom=twist_values(speed);
      snapshot->dynamics={parameters.linear_delay,parameters.linear_tau,parameters.acceleration,
        parameters.deceleration,parameters.steering_delay,parameters.steering_tau,parameters.wheelbase,parameters.track};
      snapshot->smoother_limits={0.55,0.95,0.9,1.1};
    }
    chassis_dynamics::PredictionInput input;
    input.smoother_valid=true; input.smoother_linear=0.55; input.smoother_angular=0.08;
    input.issued={{-0.4,0.5,0.08},{-0.1,0.45,0.1}};
    optimizer.input(input);
    const auto command=optimizer.evalControl(plan.poses.front(),speed,plan,checker,snapshot);
    result.commands.push_back(twist_values(command.twist));
    if(snapshot) {
      snapshot->command=twist_values(command.twist); snapshot->command_returned=true;
      snapshot->smoother_valid=true; snapshot->smoother_linear=0.55; snapshot->smoother_angular=0.08;
      snapshot->history_count=2; snapshot->issued[0]={-0.4,0.5,0.08}; snapshot->issued[1]={-0.1,0.45,0.1};
      const auto & selected=snapshot->sequence[snapshot->command_offset];
      assert(selected[0]==command.twist.linear.x && selected[1]==command.twist.linear.y && selected[2]==command.twist.angular.z);
      const auto replay=ns::replay(*snapshot);
      auto actual=replay.state;
      chassis_dynamics::RangerMotionModel model(handler.get(),"FollowPath",parameters,snapshot->model_dt,0.55,0.95,0.9,1.1);
      model.set_input(input,snapshot->model_dt,snapshot->constraints[0],snapshot->constraints[1],snapshot->constraints[3]);
      model.predict(actual);
      assert(xt::all(xt::equal(actual.vx,replay.state.vx)));
      assert(xt::all(xt::equal(actual.wz,replay.state.wz)));
      ns::ReplayIntegrator native;
      const auto actual_path=native.integrate(actual,static_cast<float>(snapshot->model_dt));
      assert(xt::all(xt::equal(actual_path.x,replay.trajectory.x)));
      assert(xt::all(xt::equal(actual_path.y,replay.trajectory.y)));
      assert(xt::all(xt::equal(actual_path.yaws,replay.trajectory.yaws)));
      recorder.finish(snapshot);
    }
    speed=command.twist;
  }
  auto * grid=map->getCostmap();
  std::memset(grid->getCharMap(),254,grid->getSizeInCellsX()*grid->getSizeInCellsY());
  ns::Frame * failed=record?recorder.begin(99):nullptr;
  try {optimizer.evalControl(plan.poses.front(),speed,plan,checker,failed); assert(false);}
  catch(const std::runtime_error & error) {
    result.failure=error.what();
    if(failed) {
      ns::text(failed->exception,error.what());
      assert(!failed->sequence_valid && !failed->command_returned); recorder.finish(failed);
    }
  }
  recorder.stop(); if(record) {std::filesystem::remove(output/"request.json");}
  optimizer.shutdown(); grid->resetMap(0,0,grid->getSizeInCellsX(),grid->getSizeInCellsY());
  return result;
}

int main(int argc,char ** argv)
{
  if(!std::getenv("NAVLITE_ISOLATED") || std::string(std::getenv("NAVLITE_ISOLATED"))!="1") {
    std::cerr<<"requires private network/IPC/mount namespace\n";return 2;
  }
  std::ifstream network("/proc/net/dev"); std::string line;
  while(std::getline(network,line)) {
    if(line.find(':')!=std::string::npos && line.substr(0,line.find(':')).find("lo")==std::string::npos) {
      std::cerr<<"refusing non-loopback network namespace\n";return 2;
    }
  }
  rclcpp::init(argc,argv);
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("snapshot_test_map");
  map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  map->set_parameter(rclcpp::Parameter("footprint",std::string("[[0.47,0.36],[0.47,-0.36],[-0.47,-0.36],[-0.47,0.36]]")));
  assert(map->on_configure(rclcpp_lifecycle::State{})==nav2_util::CallbackReturn::SUCCESS);
  map->getCostmap()->resizeMap(200,200,0.05,-5,-5); map->getCostmap()->resetMap(0,0,200,200);
  auto node=std::make_shared<nav2_util::LifecycleNode>("snapshot_test_controller");
  node->declare_parameter("controller_frequency",15.0);
  node->declare_parameter("FollowPath.model_dt",1.0/15.0);
  node->declare_parameter("FollowPath.time_steps",48);
  node->declare_parameter("FollowPath.batch_size",64);
  node->declare_parameter("FollowPath.regenerate_noises",false);
  node->declare_parameter("FollowPath.retry_attempt_limit",3);
  node->declare_parameter("FollowPath.motion_model",std::string("Ackermann"));
  node->declare_parameter("FollowPath.AckermannConstraints.min_turning_r",0.81);
  node->declare_parameter("FollowPath.critics",std::vector<std::string>{"ObstaclesCritic","GoalCritic"});
  nav2_controller::SimpleGoalChecker checker; checker.initialize(node,"goal_checker",map);
  nav_msgs::msg::Path plan; plan.header.frame_id=map->getGlobalFrameID(); plan.header.stamp=node->now();
  for(int i=0;i<=60;++i) {
    geometry_msgs::msg::PoseStamped p; p.header=plan.header; p.pose.position.x=i*0.05; p.pose.orientation.w=1.0;
    plan.poses.push_back(p);
  }
  const auto off=run_trial(false,node,map,&checker,plan);
  const auto on=run_trial(true,node,map,&checker,plan);
  assert(off.commands==on.commands);
  assert(off.failure=="Optimizer fail to compute path" && on.failure==off.failure);
  // Exercise the complete production compute hook (exact transformed path/map/feedback),
  // not just its optimizer seam. Only the test build's request-file pathname differs.
  const std::filesystem::path hook_output="/tmp/njrh_reports/navlite_snapshot_integration/production_hook";
  std::filesystem::create_directories(hook_output);
  auto controller=std::make_unique<RangerMPPIController>();
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  controller->configure(node,"FollowPath",tf,map); controller->activate(); controller->setPlan(plan);
  {
    std::ofstream f(NAVLITE_SNAPSHOT_REQUEST_PATH);
    f<<nlohmann::json{{"schema",1},{"session","production_hook_"+std::to_string(getpid())},
      {"output_dir",hook_output.string()},{"max_bytes",16777216},
      {"deadline_monotonic_ns",ns::Recorder::monotonic_ns()+10000000000LL}};
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(700)); // test-only writer activation wait
  geometry_msgs::msg::Twist velocity; velocity.linear.x=0.45;
  for(int n=0;n<3;++n) {const auto command=controller->computeVelocityCommands(plan.poses.front(),velocity,&checker); velocity=command.twist;}
  std::memset(map->getCostmap()->getCharMap(),254,40000);
  try {controller->computeVelocityCommands(plan.poses.front(),velocity,&checker); assert(false);}
  catch(const std::runtime_error & e) {assert(std::string(e.what())=="Optimizer fail to compute path");}
  controller->setPlan(nav_msgs::msg::Path{});
  std::string unknown_on, unknown_off;
  try {controller->computeVelocityCommands(plan.poses.front(),velocity,&checker); assert(false);}
  catch(const std::exception & e) {unknown_on=std::string(typeid(e).name())+":"+e.what();}
  std::filesystem::remove(NAVLITE_SNAPSHOT_REQUEST_PATH);
  std::this_thread::sleep_for(std::chrono::milliseconds(650));
  try {controller->computeVelocityCommands(plan.poses.front(),velocity,&checker); assert(false);}
  catch(const std::exception & e) {unknown_off=std::string(typeid(e).name())+":"+e.what();}
  assert(!unknown_on.empty() && unknown_on==unknown_off);
  std::cout<<"PASS unknown transformPath exception diag on/off unchanged "<<unknown_on<<'\n';
  controller->deactivate(); controller->cleanup(); controller.reset();
  map->on_cleanup(rclcpp_lifecycle::State{}); rclcpp::shutdown();
  std::cout<<"PASS actual optimizer diag off/on exact commands, original exception, pre-shift selector, replay bitwise state/native trajectory\n";
}

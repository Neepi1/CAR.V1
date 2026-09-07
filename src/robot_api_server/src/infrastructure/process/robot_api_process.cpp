#include "robot_api_server/infrastructure/process/robot_api_process.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/application/composition/application_composition_module.hpp"

using namespace std::chrono_literals;

namespace robot_api_server::infrastructure::process {
namespace {

using application::composition::ApplicationCompositionModule;

bool is_transient_action_client_exception(const std::exception &exc) {
  const std::string message = exc.what();
  return message.find("Taking data from action client but no ready event") !=
         std::string::npos;
}

class RobotApiServerNode final : public rclcpp::Node {
public:
  RobotApiServerNode()
      : Node("robot_api_server"),
        application_(std::make_unique<ApplicationCompositionModule>(*this)) {}

private:
  std::unique_ptr<ApplicationCompositionModule> application_;
};

} // namespace

int run_robot_api_server_process(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<RobotApiServerNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok()) {
      try {
        executor.spin();
        break;
      } catch (const std::runtime_error &exc) {
        if (!is_transient_action_client_exception(exc)) {
          throw;
        }
        RCLCPP_ERROR(
            node->get_logger(),
            "continuing after transient action client executor exception: %s",
            exc.what());
        std::this_thread::sleep_for(100ms);
      }
    }
  } catch (const std::exception &exc) {
    std::cerr << "robot_api_server fatal exception: " << exc.what()
              << std::endl;
    exit_code = 1;
  } catch (...) {
    std::cerr << "robot_api_server unknown fatal exception" << std::endl;
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}

} // namespace robot_api_server::infrastructure::process

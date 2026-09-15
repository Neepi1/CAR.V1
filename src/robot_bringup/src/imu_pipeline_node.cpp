#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_hesai_jt128/imu_node_factory.hpp"
#include "robot_local_state/imu_node_factory.hpp"

// Only composition lives here. Sensor conversion and local-state bias learning
// remain in their owning packages. Neither executor publishes canonical TF.
int main(int argc, char ** argv)
{
  std::string remap_params;
  std::string filter_params;
  bool with_filter = true;
  bool intra_process = true;
  std::vector<std::string> ros_arguments;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if ((arg == "--remap-params" || arg == "--filter-params") && i + 1 < argc) {
        (arg == "--remap-params" ? remap_params : filter_params) = argv[++i];
      } else if (arg == "--without-bias-filter") {
        with_filter = false;
      } else if (arg == "--dds-only") {
        intra_process = false;
      } else if (arg == "--ros-args") {
        for (++i; i < argc; ++i) {
          ros_arguments.emplace_back(argv[i]);
        }
      } else {
        throw std::invalid_argument("unknown or incomplete argument: " + arg);
      }
    }
    if (remap_params.empty() || (with_filter && filter_params.empty())) {
      throw std::invalid_argument(
        "usage: imu_pipeline_node --remap-params FILE "
        "[--filter-params FILE | --without-bias-filter] [--dds-only]");
    }

    rclcpp::init(0, nullptr);
    auto options = [&ros_arguments](const std::string & params) {
        std::vector<std::string> args{"--ros-args", "--params-file", params};
        args.insert(args.end(), ros_arguments.begin(), ros_arguments.end());
        // Do not apply a process-wide node remap or enable IPC on TF/static TF.
        return rclcpp::NodeOptions().use_global_arguments(false)
          .use_intra_process_comms(false).arguments(args);
      };
    auto filter = with_filter ? robot_local_state::make_imu_gyro_bias_filter_node(
      options(filter_params), intra_process) : nullptr;
    auto remap = robot_hesai_jt128::make_imu_axis_remap_node(
      options(remap_params), intra_process && with_filter);

    // A missing TF can block the filter for its existing lookup timeout. Keep
    // the canonical raw IMU executor independent so mapping is not blocked too.
    rclcpp::executors::SingleThreadedExecutor remap_executor;
    rclcpp::executors::SingleThreadedExecutor filter_executor;
    remap_executor.add_node(remap);
    if (filter) {
      filter_executor.add_node(filter);
    }
    RCLCPP_INFO(remap->get_logger(), "IMU pipeline started: filter=%s transport=%s",
      with_filter ? "enabled" : "disabled", intra_process && with_filter ? "intra_process" : "DDS");

    std::exception_ptr worker_error;
    std::thread worker;
    // Do not let a cleanup exception unwind across a still-joinable thread.
    auto stop_executors = [&]() noexcept {
        try {rclcpp::shutdown();} catch (...) {}
        try {remap_executor.cancel();} catch (...) {}
        try {filter_executor.cancel();} catch (...) {}
      };
    if (filter) {
      worker = std::thread([&]() {
          try {
            filter_executor.spin();
          } catch (...) {
            worker_error = std::current_exception();
            stop_executors();
          }
        });
    }
    std::exception_ptr main_error;
    try {
      remap_executor.spin();
    } catch (...) {
      main_error = std::current_exception();
    }
    // Wake and join both executors before destroying nodes/the TF listener.
    // SIGINT and SIGTERM also reach this path through rclcpp's context shutdown.
    stop_executors();
    if (worker.joinable()) {
      worker.join();
    }
    if (main_error) {
      std::rethrow_exception(main_error);
    }
    if (worker_error) {
      std::rethrow_exception(worker_error);
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "imu_pipeline_node: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}

#pragma once

#include <memory>

namespace rclcpp {
class Node;
}

namespace robot_api_server::application::composition {

class ApplicationCompositionModule {
public:
  explicit ApplicationCompositionModule(rclcpp::Node &node);
  ~ApplicationCompositionModule();

  ApplicationCompositionModule(const ApplicationCompositionModule &) = delete;
  ApplicationCompositionModule &
  operator=(const ApplicationCompositionModule &) = delete;
  ApplicationCompositionModule(ApplicationCompositionModule &&) = delete;
  ApplicationCompositionModule &
  operator=(ApplicationCompositionModule &&) = delete;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace robot_api_server::application::composition

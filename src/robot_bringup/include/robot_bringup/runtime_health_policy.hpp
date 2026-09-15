#pragma once

#include <cmath>
#include <optional>

namespace robot_bringup::health {

// Receive time alone cannot prove liveness: a publisher may replay an old stamp.
class OdomWatch {
public:
  explicit OdomWatch(double timeout = 3.0) : timeout_(timeout) {}
  void reset(double now) {
    started_ = now;
    last_good_.reset();
    last_stamp_.reset();
  }
  bool observe(double now, double ros_now, double stamp) {
    if (!std::isfinite(stamp) || stamp <= 0.0 ||
        ros_now - stamp < -0.25 || ros_now - stamp > timeout_ ||
        (last_stamp_ && stamp <= *last_stamp_)) {
      return false;
    }
    last_stamp_ = stamp;
    last_good_ = now;
    return true;
  }
  double no_update_age(double now) const { return now - last_good_.value_or(started_); }
  bool fault(double now) const { return no_update_age(now) >= timeout_; }
  bool seen_valid() const { return last_good_.has_value(); }
private:
  double timeout_;
  double started_{0.0};
  std::optional<double> last_good_;
  std::optional<double> last_stamp_;
};

}  // namespace robot_bringup::health

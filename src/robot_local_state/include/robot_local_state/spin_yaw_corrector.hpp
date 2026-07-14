#ifndef ROBOT_LOCAL_STATE__SPIN_YAW_CORRECTOR_HPP_
#define ROBOT_LOCAL_STATE__SPIN_YAW_CORRECTOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace robot_local_state
{

struct SpinYawCorrectorConfig
{
  std::uint8_t spinning_motion_mode{2};
  double command_start_threshold_radps{0.05};
  double command_zero_linear_threshold_mps{0.02};
  double command_zero_angular_threshold_radps{0.02};
  double imu_motion_threshold_radps{0.03};
  double imu_stop_threshold_radps{0.02};
  double imu_stop_stable_sec{0.30};
  double imu_timeout_sec{0.20};
  double imu_max_integration_dt_sec{0.05};
  double settle_timeout_sec{2.0};
  double freeze_xy_max_command_linear_mps{0.03};
  bool freeze_xy_while_spinning{true};
  bool replace_spin_twist_with_imu{false};
};

struct CorrectedWheelSample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double angular_velocity_z{0.0};
  bool correction_active{false};
  bool imu_fresh{false};
};

struct SpinYawCorrectorStatus
{
  bool initialized{false};
  bool correction_active{false};
  bool spin_command_seen{false};
  bool zero_command_seen{false};
  bool imu_fresh{false};
  bool settle_ready{false};
  std::uint8_t motion_mode{0};
  double raw_yaw_unwrapped{0.0};
  double corrected_yaw_unwrapped{0.0};
  double yaw_offset_rad{0.0};
  double spin_imu_delta_rad{0.0};
  double latest_imu_yaw_rate_radps{0.0};
  std::uint64_t completed_spin_count{0};
  std::uint64_t imu_fallback_count{0};
  std::uint64_t imu_gap_count{0};
  std::uint64_t forced_settle_count{0};
};

class SpinYawCorrector
{
public:
  explicit SpinYawCorrector(const SpinYawCorrectorConfig & config)
  : config_(config)
  {
  }

  void observe_motion_mode(const std::uint8_t motion_mode, const double now_sec)
  {
    const bool entered_spinning =
      (!motion_mode_valid_ || motion_mode_ != config_.spinning_motion_mode) &&
      motion_mode == config_.spinning_motion_mode;
    motion_mode_ = motion_mode;
    motion_mode_valid_ = true;

    if (entered_spinning) {
      begin_spin(now_sec);
    } else if (
      correction_active_ && motion_mode != config_.spinning_motion_mode &&
      !spin_command_seen_)
    {
      abort_requested_ = true;
    }
  }

  void observe_command(
    const double linear_x, const double linear_y, const double angular_z,
    const double now_sec)
  {
    command_linear_mps_ = std::hypot(linear_x, linear_y);
    command_angular_radps_ = angular_z;
    command_valid_ = true;

    const bool spin_command =
      std::abs(angular_z) >= config_.command_start_threshold_radps &&
      command_linear_mps_ <= config_.freeze_xy_max_command_linear_mps;
    if (spin_command && motion_mode_valid_ && motion_mode_ == config_.spinning_motion_mode) {
      if (!correction_active_) {
        begin_spin(now_sec);
      }
      spin_command_seen_ = true;
      zero_command_seen_ = false;
      settle_ready_ = false;
      imu_stable_since_sec_ = invalid_time();
      return;
    }

    const bool zero_command =
      command_linear_mps_ <= config_.command_zero_linear_threshold_mps &&
      std::abs(angular_z) <= config_.command_zero_angular_threshold_radps;
    if (correction_active_ && spin_command_seen_ && zero_command) {
      if (!zero_command_seen_) {
        zero_command_seen_sec_ = now_sec;
      }
      zero_command_seen_ = true;
    }
  }

  void observe_imu(const double yaw_rate_radps, const double stamp_sec)
  {
    if (!std::isfinite(yaw_rate_radps) || !std::isfinite(stamp_sec)) {
      return;
    }

    latest_imu_yaw_rate_radps_ = yaw_rate_radps;
    latest_imu_stamp_sec_ = stamp_sec;
    imu_valid_ = true;

    if (!correction_active_ || !spin_reference_ready_) {
      return;
    }

    if (valid_time(last_integrated_imu_stamp_sec_)) {
      const double dt = stamp_sec - last_integrated_imu_stamp_sec_;
      if (dt > 0.0 && dt <= config_.imu_max_integration_dt_sec) {
        spin_imu_delta_rad_ +=
          0.5 * (last_integrated_imu_yaw_rate_radps_ + yaw_rate_radps) * dt;
      } else if (dt > config_.imu_max_integration_dt_sec) {
        ++imu_gap_count_;
      }
    }
    last_integrated_imu_stamp_sec_ = stamp_sec;
    last_integrated_imu_yaw_rate_radps_ = yaw_rate_radps;

    if (std::abs(yaw_rate_radps) >= config_.imu_motion_threshold_radps) {
      imu_motion_seen_ = true;
    }

    if (zero_command_seen_ && std::abs(yaw_rate_radps) <= config_.imu_stop_threshold_radps) {
      if (!valid_time(imu_stable_since_sec_)) {
        imu_stable_since_sec_ = stamp_sec;
      }
      if ((stamp_sec - imu_stable_since_sec_) >= config_.imu_stop_stable_sec) {
        settle_ready_ = true;
      }
    } else {
      imu_stable_since_sec_ = invalid_time();
      settle_ready_ = false;
    }
  }

  CorrectedWheelSample correct_wheel(
    const double raw_x, const double raw_y, const double raw_yaw,
    const double raw_angular_velocity_z, const double now_sec)
  {
    if (!wheel_initialized_) {
      wheel_initialized_ = true;
      last_raw_yaw_ = raw_yaw;
      raw_yaw_unwrapped_ = raw_yaw;
      latest_raw_x_ = raw_x;
      latest_raw_y_ = raw_y;
      latest_corrected_x_ = raw_x;
      latest_corrected_y_ = raw_y;
      latest_corrected_yaw_ = raw_yaw;
    } else {
      raw_yaw_unwrapped_ += normalize_angle(raw_yaw - last_raw_yaw_);
      last_raw_yaw_ = raw_yaw;
      latest_raw_x_ = raw_x;
      latest_raw_y_ = raw_y;
    }

    if (correction_active_ && !spin_reference_ready_) {
      establish_spin_reference(now_sec);
    }

    if (correction_active_ && spin_reference_ready_) {
      const bool imu_timed_out =
        (now_sec - correction_started_sec_) > config_.imu_timeout_sec &&
        (!imu_valid_ || (now_sec - latest_imu_stamp_sec_) > config_.imu_timeout_sec);
      if (imu_timed_out || abort_requested_) {
        abort_to_raw_wheel();
      }
    }

    if (
      correction_active_ && zero_command_seen_ && valid_time(zero_command_seen_sec_) &&
      (now_sec - zero_command_seen_sec_) >= config_.settle_timeout_sec)
    {
      settle_ready_ = true;
      ++forced_settle_count_;
    }

    if (correction_active_ && spin_reference_ready_) {
      latest_corrected_yaw_ = spin_start_corrected_yaw_ + spin_imu_delta_rad_;

      const bool freeze_xy =
        config_.freeze_xy_while_spinning &&
        (!command_valid_ || command_linear_mps_ <= config_.freeze_xy_max_command_linear_mps);
      if (freeze_xy) {
        latest_corrected_x_ = spin_start_corrected_x_;
        latest_corrected_y_ = spin_start_corrected_y_;
      } else {
        const double active_yaw_offset = latest_corrected_yaw_ - raw_yaw_unwrapped_;
        const double raw_delta_x = raw_x - spin_start_raw_x_;
        const double raw_delta_y = raw_y - spin_start_raw_y_;
        double corrected_delta_x = 0.0;
        double corrected_delta_y = 0.0;
        rotate_position(
          raw_delta_x, raw_delta_y, active_yaw_offset,
          corrected_delta_x, corrected_delta_y);
        latest_corrected_x_ = spin_start_corrected_x_ + corrected_delta_x;
        latest_corrected_y_ = spin_start_corrected_y_ + corrected_delta_y;
      }

      if (settle_ready_) {
        finalize_spin();
      }
    } else {
      transform_raw_position(raw_x, raw_y, latest_corrected_x_, latest_corrected_y_);
      latest_corrected_yaw_ = raw_yaw_unwrapped_ + yaw_offset_rad_;
    }

    const bool imu_fresh = imu_is_fresh(now_sec);
    double corrected_wz = raw_angular_velocity_z;
    if (
      correction_active_ && config_.replace_spin_twist_with_imu && imu_fresh)
    {
      corrected_wz = latest_imu_yaw_rate_radps_;
    }

    return CorrectedWheelSample{
      latest_corrected_x_, latest_corrected_y_, latest_corrected_yaw_, corrected_wz,
      correction_active_, imu_fresh};
  }

  SpinYawCorrectorStatus status(const double now_sec) const
  {
    return SpinYawCorrectorStatus{
      wheel_initialized_, correction_active_, spin_command_seen_, zero_command_seen_,
      imu_is_fresh(now_sec), settle_ready_, motion_mode_, raw_yaw_unwrapped_,
      latest_corrected_yaw_, yaw_offset_rad_, spin_imu_delta_rad_,
      latest_imu_yaw_rate_radps_, completed_spin_count_, imu_fallback_count_,
      imu_gap_count_, forced_settle_count_};
  }

private:
  static double normalize_angle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double invalid_time()
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  static bool valid_time(const double value)
  {
    return std::isfinite(value);
  }

  static void rotate_position(
    const double x, const double y, const double yaw,
    double & rotated_x, double & rotated_y)
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    rotated_x = c * x - s * y;
    rotated_y = s * x + c * y;
  }

  void transform_raw_position(
    const double raw_x, const double raw_y,
    double & corrected_x, double & corrected_y) const
  {
    rotate_position(raw_x, raw_y, yaw_offset_rad_, corrected_x, corrected_y);
    corrected_x += position_transform_x_;
    corrected_y += position_transform_y_;
  }

  void anchor_position_transform()
  {
    double rotated_raw_x = 0.0;
    double rotated_raw_y = 0.0;
    rotate_position(
      latest_raw_x_, latest_raw_y_, yaw_offset_rad_,
      rotated_raw_x, rotated_raw_y);
    position_transform_x_ = latest_corrected_x_ - rotated_raw_x;
    position_transform_y_ = latest_corrected_y_ - rotated_raw_y;
  }

  bool imu_is_fresh(const double now_sec) const
  {
    return imu_valid_ && (now_sec - latest_imu_stamp_sec_) <= config_.imu_timeout_sec;
  }

  void begin_spin(const double now_sec)
  {
    correction_active_ = true;
    spin_reference_ready_ = false;
    spin_command_seen_ =
      command_valid_ &&
      std::abs(command_angular_radps_) >= config_.command_start_threshold_radps;
    zero_command_seen_ = false;
    settle_ready_ = false;
    abort_requested_ = false;
    imu_motion_seen_ = false;
    correction_started_sec_ = now_sec;
    zero_command_seen_sec_ = invalid_time();
    imu_stable_since_sec_ = invalid_time();
    last_integrated_imu_stamp_sec_ = invalid_time();
    last_integrated_imu_yaw_rate_radps_ = 0.0;
    spin_imu_delta_rad_ = 0.0;
    if (wheel_initialized_) {
      establish_spin_reference(now_sec);
    }
  }

  void establish_spin_reference(const double now_sec)
  {
    spin_reference_ready_ = true;
    correction_started_sec_ = now_sec;
    transform_raw_position(
      latest_raw_x_, latest_raw_y_,
      spin_start_corrected_x_, spin_start_corrected_y_);
    spin_start_raw_x_ = latest_raw_x_;
    spin_start_raw_y_ = latest_raw_y_;
    spin_start_corrected_yaw_ = raw_yaw_unwrapped_ + yaw_offset_rad_;
    latest_corrected_x_ = spin_start_corrected_x_;
    latest_corrected_y_ = spin_start_corrected_y_;
    latest_corrected_yaw_ = spin_start_corrected_yaw_;
    if (imu_valid_) {
      last_integrated_imu_stamp_sec_ = latest_imu_stamp_sec_;
      last_integrated_imu_yaw_rate_radps_ = latest_imu_yaw_rate_radps_;
    }
  }

  void finalize_spin()
  {
    yaw_offset_rad_ = latest_corrected_yaw_ - raw_yaw_unwrapped_;
    anchor_position_transform();
    correction_active_ = false;
    spin_reference_ready_ = false;
    settle_ready_ = false;
    abort_requested_ = false;
    ++completed_spin_count_;
  }

  void abort_to_raw_wheel()
  {
    yaw_offset_rad_ = latest_corrected_yaw_ - raw_yaw_unwrapped_;
    anchor_position_transform();
    correction_active_ = false;
    spin_reference_ready_ = false;
    settle_ready_ = false;
    abort_requested_ = false;
    ++imu_fallback_count_;
  }

  SpinYawCorrectorConfig config_;
  bool wheel_initialized_{false};
  double last_raw_yaw_{0.0};
  double raw_yaw_unwrapped_{0.0};
  double latest_raw_x_{0.0};
  double latest_raw_y_{0.0};
  double latest_corrected_x_{0.0};
  double latest_corrected_y_{0.0};
  double latest_corrected_yaw_{0.0};
  double position_transform_x_{0.0};
  double position_transform_y_{0.0};
  double yaw_offset_rad_{0.0};

  bool motion_mode_valid_{false};
  std::uint8_t motion_mode_{0};
  bool command_valid_{false};
  double command_linear_mps_{0.0};
  double command_angular_radps_{0.0};

  bool correction_active_{false};
  bool spin_reference_ready_{false};
  bool spin_command_seen_{false};
  bool zero_command_seen_{false};
  bool settle_ready_{false};
  bool abort_requested_{false};
  bool imu_motion_seen_{false};
  double correction_started_sec_{0.0};
  double zero_command_seen_sec_{invalid_time()};
  double imu_stable_since_sec_{invalid_time()};
  double spin_start_corrected_x_{0.0};
  double spin_start_corrected_y_{0.0};
  double spin_start_corrected_yaw_{0.0};
  double spin_start_raw_x_{0.0};
  double spin_start_raw_y_{0.0};
  double spin_imu_delta_rad_{0.0};

  bool imu_valid_{false};
  double latest_imu_stamp_sec_{0.0};
  double latest_imu_yaw_rate_radps_{0.0};
  double last_integrated_imu_stamp_sec_{invalid_time()};
  double last_integrated_imu_yaw_rate_radps_{0.0};

  std::uint64_t completed_spin_count_{0};
  std::uint64_t imu_fallback_count_{0};
  std::uint64_t imu_gap_count_{0};
  std::uint64_t forced_settle_count_{0};
};

}  // namespace robot_local_state

#endif  // ROBOT_LOCAL_STATE__SPIN_YAW_CORRECTOR_HPP_

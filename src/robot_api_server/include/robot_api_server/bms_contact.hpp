#pragma once

#include <string>

#include "sensor_msgs/msg/battery_state.hpp"

namespace robot_api_server
{

struct BatteryContactEvaluation
{
  bool contact{false};
  std::string reason{"no_contact"};
};

double normalized_soc_percent(float percentage);

BatteryContactEvaluation evaluate_battery_charging_contact(
  const sensor_msgs::msg::BatteryState & msg,
  double current_min_a,
  double voltage_min_v,
  double voltage_max_v,
  bool full_soc_voltage_contact_enable,
  double full_soc_threshold_pct);

}  // namespace robot_api_server

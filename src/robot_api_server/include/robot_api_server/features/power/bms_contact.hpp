#pragma once

#include <string>

#include "sensor_msgs/msg/battery_state.hpp"

namespace robot_api_server
{

struct BmsChargingContactSnapshot
{
  bool have_state{false};
  bool have_soc{false};
  bool fresh{false};
  bool contact{false};
  bool contact_stable{false};
  std::string reason{"no_bms_state"};
  double age_sec{-1.0};
  double contact_stable_duration_sec{0.0};
  double no_contact_duration_sec{0.0};
  double soc{0.0};
  double voltage{0.0};
  double current{0.0};
  double temperature{0.0};
  int power_supply_status{sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN};
  int power_supply_health{sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN};
  int power_supply_technology{sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN};
  bool present{false};
};

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
  double full_soc_threshold_pct,
  bool confirmed_dock_context = false);

}  // namespace robot_api_server

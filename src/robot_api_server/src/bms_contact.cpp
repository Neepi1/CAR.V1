#include "robot_api_server/bms_contact.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace robot_api_server
{
namespace
{

bool voltage_in_contact_range(const float voltage, const double min_v, const double max_v)
{
  const double v = static_cast<double>(voltage);
  return std::isfinite(v) && v >= min_v && v <= max_v;
}

}  // namespace

double normalized_soc_percent(const float percentage)
{
  if (!std::isfinite(percentage)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double raw = static_cast<double>(percentage);
  return std::clamp(raw <= 1.0 ? raw * 100.0 : raw, 0.0, 100.0);
}

BatteryContactEvaluation evaluate_battery_charging_contact(
  const sensor_msgs::msg::BatteryState & msg,
  const double current_min_a,
  const double voltage_min_v,
  const double voltage_max_v,
  const bool full_soc_voltage_contact_enable,
  const double full_soc_threshold_pct)
{
  if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
    return {true, "power_supply_status=CHARGING"};
  }
  if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL) {
    return {true, "power_supply_status=FULL"};
  }
  if (std::isfinite(msg.current) && static_cast<double>(msg.current) > current_min_a) {
    return {true, "current_above_threshold"};
  }
  if (msg.present && voltage_in_contact_range(msg.voltage, voltage_min_v, voltage_max_v)) {
    return {true, "present_voltage_valid"};
  }
  const double soc = normalized_soc_percent(msg.percentage);
  if (full_soc_voltage_contact_enable && msg.present && std::isfinite(soc) && soc >= full_soc_threshold_pct &&
    voltage_in_contact_range(msg.voltage, voltage_min_v, voltage_max_v)) {
    return {true, "full_soc_present_voltage_valid"};
  }

  std::ostringstream reason;
  reason << "no_contact status=" << static_cast<int>(msg.power_supply_status)
         << " present=" << (msg.present ? "true" : "false");
  return {false, reason.str()};
}

}  // namespace robot_api_server

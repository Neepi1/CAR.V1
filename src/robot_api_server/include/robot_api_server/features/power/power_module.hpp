#pragma once

#include <functional>
#include <memory>
#include <string>

#include "robot_api_server/features/power/bms_contact.hpp"

namespace rclcpp
{
class Node;
}

namespace robot_api_server::features::power
{

struct PowerModuleConfig
{
  std::string state_topic{"/battery_state"};
  double state_max_age_sec{3.0};
  double charging_current_min_a{0.10};
  double charging_contact_voltage_min_v{40.0};
  double charging_contact_voltage_max_v{1000.0};
  bool full_soc_voltage_contact_enable{true};
  double full_soc_threshold_pct{99.0};
  double contact_stable_required_sec{2.0};
};

struct PowerModulePorts
{
  // Called after the new snapshot is committed and its mutex is released.
  std::function<void(const BatteryContactEvaluation &, bool, double)>
  on_contact_evidence;
  std::function<void(bool)> on_charging_contact;
  // Existing confirmed occupancy only, not a return-to-dock job/predock phase.
  // Called without the power snapshot mutex; must not derive context from BMS.
  std::function<bool()> confirmed_dock_context;
};

// Owns the complete process-resident BatteryState edge and its derived
// charging-contact evidence. Consumers receive one immutable snapshot instead
// of sharing raw fields or the subscription mutex.
class PowerModule
{
public:
  PowerModule(rclcpp::Node & node, PowerModuleConfig config, PowerModulePorts ports);
  ~PowerModule();

  PowerModule(const PowerModule &) = delete;
  PowerModule & operator=(const PowerModule &) = delete;

  BmsChargingContactSnapshot snapshot() const;
  bool charging_contact_active() const;
  std::string state_topic() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::power

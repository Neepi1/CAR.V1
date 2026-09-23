#include "robot_api_server/features/power/power_module.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace robot_api_server::features::power
{
namespace
{

struct PowerState
{
  mutable std::mutex mutex;
  BmsChargingContactSnapshot snapshot;
  std::chrono::steady_clock::time_point received_at{};
  bool contact_timer_started{false};
  bool no_contact_timer_started{false};
  std::chrono::steady_clock::time_point contact_started_at{};
  std::chrono::steady_clock::time_point no_contact_started_at{};
};

}  // namespace

struct PowerModule::Impl
{
  Impl(rclcpp::Node & node, PowerModuleConfig module_config, PowerModulePorts module_ports)
  : config(std::move(module_config)), ports(std::move(module_ports))
  {
    subscription = node.create_subscription<sensor_msgs::msg::BatteryState>(
      config.state_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr message) {
        handle_state(*message);
      });
  }

  void handle_state(const sensor_msgs::msg::BatteryState & message)
  {
    const bool confirmed_dock_context =
      std::isfinite(message.current) && message.current > config.charging_current_min_a &&
      ports.confirmed_dock_context && ports.confirmed_dock_context();
    const auto contact = evaluate_battery_charging_contact(
      message,
      config.charging_current_min_a,
      config.charging_contact_voltage_min_v,
      config.charging_contact_voltage_max_v,
      config.full_soc_voltage_contact_enable,
      config.full_soc_threshold_pct,
      confirmed_dock_context);
    const auto now = std::chrono::steady_clock::now();
    bool contact_stable = false;
    double contact_stable_duration_sec = 0.0;
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      auto & current = state.snapshot;
      const double soc = normalized_soc_percent(message.percentage);
      if (std::isfinite(soc)) {
        current.soc = soc;
        current.have_soc = true;
      } else {
        current.have_soc = false;
      }
      current.voltage = static_cast<double>(message.voltage);
      current.current = static_cast<double>(message.current);
      current.temperature = static_cast<double>(message.temperature);
      current.power_supply_status = static_cast<int>(message.power_supply_status);
      current.power_supply_health = static_cast<int>(message.power_supply_health);
      current.power_supply_technology = static_cast<int>(message.power_supply_technology);
      current.present = message.present;
      current.contact = contact.contact;
      current.reason = contact.reason;

      if (contact.contact) {
        if (!state.contact_timer_started) {
          state.contact_started_at = now;
          state.contact_timer_started = true;
        }
        state.no_contact_timer_started = false;
        current.contact_stable_duration_sec =
          std::chrono::duration<double>(now - state.contact_started_at).count();
        current.no_contact_duration_sec = 0.0;
      } else {
        if (!state.no_contact_timer_started) {
          state.no_contact_started_at = now;
          state.no_contact_timer_started = true;
        }
        state.contact_timer_started = false;
        current.no_contact_duration_sec =
          std::chrono::duration<double>(now - state.no_contact_started_at).count();
        current.contact_stable_duration_sec = 0.0;
      }

      current.contact_stable =
        current.contact &&
        current.contact_stable_duration_sec >= config.contact_stable_required_sec;
      current.have_state = true;
      current.fresh = true;
      current.age_sec = 0.0;
      state.received_at = now;
      contact_stable = current.contact_stable;
      contact_stable_duration_sec = current.contact_stable_duration_sec;
    }

    if (ports.on_contact_evidence) {
      ports.on_contact_evidence(contact, contact_stable, contact_stable_duration_sec);
    }
    if (ports.on_charging_contact) {
      ports.on_charging_contact(contact.contact);
    }
  }

  BmsChargingContactSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    auto result = state.snapshot;
    if (!result.have_state) {
      result.reason = "no_bms_state";
      return result;
    }
    result.age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - state.received_at).count();
    result.fresh = result.age_sec <= config.state_max_age_sec;
    if (!result.fresh) {
      result.contact = false;
      result.contact_stable = false;
      result.reason = "stale_bms_state";
      return result;
    }
    result.contact_stable =
      result.contact &&
      result.contact_stable_duration_sec >= config.contact_stable_required_sec;
    return result;
  }

  PowerModuleConfig config;
  PowerModulePorts ports;
  mutable PowerState state;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription;
};

PowerModule::PowerModule(
  rclcpp::Node & node,
  PowerModuleConfig config,
  PowerModulePorts ports)
: impl_(std::make_unique<Impl>(node, std::move(config), std::move(ports)))
{
}

PowerModule::~PowerModule() = default;

BmsChargingContactSnapshot PowerModule::snapshot() const
{
  return impl_->snapshot();
}

bool PowerModule::charging_contact_active() const
{
  return snapshot().contact;
}

std::string PowerModule::state_topic() const
{
  return impl_->config.state_topic;
}

}  // namespace robot_api_server::features::power

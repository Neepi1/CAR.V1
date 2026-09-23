#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "robot_api_server/features/power/power_module.hpp"
#include "robot_api_server/features/power/bms_contact.hpp"

namespace power = robot_api_server::features::power;
using namespace std::chrono_literals;

TEST(PowerModule, UnscopedPositiveCurrentIsNotDockContact)
{
  sensor_msgs::msg::BatteryState message;
  message.voltage = 498.0F;
  message.percentage = 85.0F;
  for (const auto current : {0.11F, 0.7F, 1.1F}) {
    message.current = current;
    const auto evaluation = robot_api_server::evaluate_battery_charging_contact(
      message, 0.10, 40.0, 1000.0, true, 99.0);
    EXPECT_FALSE(evaluation.contact) << current;
  }
}

TEST(PowerModule, PositiveCurrentSupportsConfirmedDockButDoesNotReplaceOtherEvidence)
{
  sensor_msgs::msg::BatteryState message;
  message.current = 0.7F;
  message.voltage = 498.0F;
  EXPECT_TRUE(robot_api_server::evaluate_battery_charging_contact(
    message, 0.10, 40.0, 1000.0, true, 99.0, true).contact);
  message.current = -1.0F;
  EXPECT_FALSE(robot_api_server::evaluate_battery_charging_contact(
    message, 0.10, 40.0, 1000.0, true, 99.0, true).contact);
  message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  EXPECT_TRUE(robot_api_server::evaluate_battery_charging_contact(
    message, 0.10, 40.0, 1000.0, true, 99.0).contact);
  message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  message.current = 0.7F;
  message.present = true;
  EXPECT_TRUE(robot_api_server::evaluate_battery_charging_contact(
    message, 0.10, 40.0, 1000.0, true, 99.0).contact);
}

TEST(PowerModule, FullSocStatusAloneIsNotPhysicalDockContact)
{
  sensor_msgs::msg::BatteryState message;
  message.percentage = 1.0F;
  message.voltage = 54.0F;
  message.current = 0.0F;
  message.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  message.present = false;

  const auto evaluation = robot_api_server::evaluate_battery_charging_contact(
    message, 0.10, 40.0, 1000.0, true, 99.0);

  EXPECT_FALSE(evaluation.contact);
  EXPECT_EQ(evaluation.reason, "full_without_physical_contact_evidence");
}

TEST(PowerModule, OwnsCommittedContactEvidenceCallbacksAndFreshness)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("robot_api_power_module_test");

  power::PowerModuleConfig config;
  config.state_topic = "/robot_api_power_module_test/battery_state";
  config.state_max_age_sec = 0.20;
  config.contact_stable_required_sec = 0.0;

  std::vector<std::string> callback_order;
  std::vector<bool> evidence_contacts;
  std::vector<bool> teleop_contacts;
  bool callback_saw_committed_snapshot = false;
  power::PowerModule * module_ptr = nullptr;
  bool confirmed_context = false;
  power::PowerModulePorts ports;
  ports.confirmed_dock_context = [&]() {
      // This read would deadlock if the caller held the PowerState mutex.
      if (module_ptr) { (void)module_ptr->snapshot(); }
      return confirmed_context;
    };
  ports.on_contact_evidence =
    [&](const robot_api_server::BatteryContactEvaluation & evaluation,
    const bool stable,
    const double stable_duration_sec) {
      callback_order.emplace_back("evidence");
      evidence_contacts.push_back(evaluation.contact);
      ASSERT_NE(module_ptr, nullptr);
      const auto committed = module_ptr->snapshot();
      callback_saw_committed_snapshot =
        committed.have_state && committed.contact == evaluation.contact &&
        committed.contact_stable == stable;
      EXPECT_GE(stable_duration_sec, 0.0);
    };
  ports.on_charging_contact = [&](const bool contact) {
      callback_order.emplace_back("teleop");
      teleop_contacts.push_back(contact);
    };

  auto module = std::make_unique<power::PowerModule>(*node, config, std::move(ports));
  module_ptr = module.get();
  EXPECT_EQ(module->state_topic(), config.state_topic);
  EXPECT_EQ(module->snapshot().reason, "no_bms_state");

  auto publisher = node->create_publisher<sensor_msgs::msg::BatteryState>(
    config.state_topic, rclcpp::QoS(10));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < discovery_deadline &&
    publisher->get_subscription_count() == 0U)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);

  sensor_msgs::msg::BatteryState message;
  message.percentage = 0.50F;
  message.voltage = 48.0F;
  message.current = 0.20F;
  message.temperature = 31.0F;
  message.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  message.power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  message.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  message.present = true;
  publisher->publish(message);

  const auto receive_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < receive_deadline &&
    !module->snapshot().have_state)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const auto fresh = module->snapshot();
  ASSERT_TRUE(fresh.have_state);
  EXPECT_TRUE(fresh.have_soc);
  EXPECT_DOUBLE_EQ(fresh.soc, 50.0);
  EXPECT_TRUE(fresh.fresh);
  EXPECT_TRUE(fresh.contact);
  EXPECT_TRUE(fresh.contact_stable);
  EXPECT_EQ(fresh.reason, "power_supply_status=CHARGING");
  EXPECT_TRUE(callback_saw_committed_snapshot);
  ASSERT_EQ(callback_order.size(), 2U);
  EXPECT_EQ(callback_order[0], "evidence");
  EXPECT_EQ(callback_order[1], "teleop");
  ASSERT_EQ(evidence_contacts.size(), 1U);
  ASSERT_EQ(teleop_contacts.size(), 1U);
  EXPECT_TRUE(evidence_contacts.back());
  EXPECT_TRUE(teleop_contacts.back());

  message.current = 0.7F;
  message.present = false;
  message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  for (const auto confirmed : {false, true, false}) {
    confirmed_context = confirmed;
    callback_order.clear();
    publisher->publish(message);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (callback_order.size() < 2U && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_EQ(callback_order.size(), 2U);
    const auto snapshot = module->snapshot();
    EXPECT_EQ(snapshot.contact, confirmed);
    EXPECT_EQ(snapshot.contact_stable, confirmed);
    EXPECT_EQ(evidence_contacts.back(), confirmed);
    EXPECT_EQ(teleop_contacts.back(), confirmed);
    EXPECT_FLOAT_EQ(snapshot.current, 0.7F);  // Raw BMS telemetry is never rewritten.
    if (!confirmed) {
      EXPECT_EQ(snapshot.reason, "current_ignored_without_confirmed_dock_context");
      EXPECT_DOUBLE_EQ(snapshot.contact_stable_duration_sec, 0.0);
    }
  }

  std::this_thread::sleep_for(250ms);
  const auto stale = module->snapshot();
  EXPECT_FALSE(stale.fresh);
  EXPECT_FALSE(stale.contact);
  EXPECT_FALSE(stale.contact_stable);
  EXPECT_EQ(stale.reason, "stale_bms_state");
  EXPECT_FALSE(module->charging_contact_active());

  callback_order.clear();
  message.percentage = 0.40F;
  message.voltage = 0.0F;
  message.current = 0.0F;
  message.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  message.present = false;
  publisher->publish(message);
  const auto no_contact_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < no_contact_deadline &&
    callback_order.size() < 2U)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(callback_order.size(), 2U);
  EXPECT_EQ(callback_order[0], "evidence");
  EXPECT_EQ(callback_order[1], "teleop");
  EXPECT_FALSE(evidence_contacts.back());
  EXPECT_FALSE(teleop_contacts.back());
  EXPECT_FALSE(module->snapshot().contact);
  EXPECT_DOUBLE_EQ(module->snapshot().contact_stable_duration_sec, 0.0);

  std::this_thread::sleep_for(20ms);
  callback_order.clear();
  publisher->publish(message);
  const auto duration_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < duration_deadline &&
    callback_order.size() < 2U)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  const auto no_contact = module->snapshot();
  EXPECT_TRUE(no_contact.fresh);
  EXPECT_FALSE(no_contact.contact);
  EXPECT_FALSE(no_contact.contact_stable);
  EXPECT_GT(no_contact.no_contact_duration_sec, 0.0);

  executor.remove_node(node);
  publisher.reset();
  module.reset();
  node.reset();
  rclcpp::shutdown();
}

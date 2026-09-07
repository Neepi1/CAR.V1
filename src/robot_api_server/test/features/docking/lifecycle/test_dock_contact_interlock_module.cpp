#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/battery_state.hpp"

#include "robot_api_server/features/docking/lifecycle/dock_contact_interlock_module.hpp"

namespace robot_api_server::features::docking
{
namespace
{

namespace fs = std::filesystem;

class TemporaryLatchFile
{
public:
  TemporaryLatchFile()
  {
    static std::atomic<unsigned int> sequence{0U};
    directory_ = fs::temp_directory_path() /
      ("robot_api_dock_contact_interlock_" + std::to_string(++sequence));
    fs::create_directories(directory_);
  }

  ~TemporaryLatchFile()
  {
    std::error_code error;
    fs::remove_all(directory_, error);
  }

  fs::path path() const {return directory_ / "docking_contact_latch.json";}

private:
  fs::path directory_;
};

struct Harness
{
  application::runtime_mode::RuntimeModeSnapshot runtime;
  BmsChargingContactSnapshot bms;
  bool navigation_goal_running{false};
  double wall_time{1000.0};
  std::string timestamp{"1000"};
  std::vector<std::string> warnings;

  DockContactInterlockPorts ports()
  {
    DockContactInterlockPorts result;
    result.runtime_snapshot = [this]() {return runtime;};
    result.bms_snapshot = [this]() {return bms;};
    result.navigation_goal_running = [this]() {return navigation_goal_running;};
    result.wall_time_seconds = [this]() {return wall_time;};
    result.timestamp_now = [this]() {return timestamp;};
    result.warn = [this](const std::string & warning) {warnings.push_back(warning);};
    return result;
  }
};

DockContactInterlockConfig config_for(const fs::path & latch_file)
{
  DockContactInterlockConfig config;
  config.latch_file = latch_file;
  config.bms_ttl_sec = 300.0;
  config.bms_clear_no_contact_sec = 3.0;
  config.allow_bms_stale_auto_undock = false;
  config.clear_when_live_undocked_no_contact = true;
  config.max_age_warn_sec = 600.0;
  config.charging_current_min_a = 0.10;
  config.full_soc_threshold_pct = 99.0;
  config.docking_status_topic = "/docking/status";
  return config;
}

TEST(DockContactInterlockModuleTest, RejectsIncompletePorts)
{
  EXPECT_THROW(
    DockContactInterlockModule(config_for("unused.json"), DockContactInterlockPorts{}),
    std::invalid_argument);
}

TEST(DockContactInterlockModuleTest, ConfirmedUndockedWithoutLatchAllowsNavigation)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "undocked";
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());

  const auto check = module.snapshot();

  EXPECT_EQ(check.dock_occupancy_state, "CONFIRMED_UNDOCKED");
  EXPECT_EQ(check.docked_state_class, "NOT_DOCKED");
  EXPECT_FALSE(check.final_auto_undock_required);
  EXPECT_FALSE(check.can_auto_undock);
  EXPECT_EQ(check.auto_undock_reason, "confirmed_undocked");
}

TEST(DockContactInterlockModuleTest, StableBmsContactCreatesStrongChargingSessionLatch)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "charging";
  harness.runtime.docking_active = true;
  harness.bms.have_state = true;
  harness.bms.fresh = true;
  harness.bms.contact = true;
  harness.bms.contact_stable = true;
  harness.bms.reason = "charging_status";
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());

  module.on_bms_contact_evidence(
    BatteryContactEvaluation{true, "charging_status"}, true, 2.0);
  const auto persisted = module.read_latch();
  const auto check = module.snapshot();

  ASSERT_TRUE(persisted.valid);
  EXPECT_TRUE(persisted.latched_docked);
  EXPECT_EQ(persisted.source, "charging_session");
  EXPECT_TRUE(persisted.source_charging_session);
  EXPECT_EQ(check.dock_occupancy_state, "DOCKED_CHARGING");
  EXPECT_TRUE(check.final_auto_undock_required);
  EXPECT_TRUE(check.can_auto_undock);
}

TEST(DockContactInterlockModuleTest, FullChargeIdlePreservesStrongSessionAndRequiresUndock)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "stopped";
  harness.bms.have_state = true;
  harness.bms.have_soc = true;
  harness.bms.fresh = true;
  harness.bms.contact = false;
  harness.bms.no_contact_duration_sec = 30.0;
  harness.bms.soc = 100.0;
  harness.bms.current = 0.0;
  harness.bms.present = false;
  harness.bms.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());
  module.update_latch(true, "charging_session", "charging_observed", "dock_main");

  const auto check = module.snapshot();

  EXPECT_FALSE(check.dock_contact_latch_auto_cleared);
  EXPECT_TRUE(check.charging_session_latched);
  EXPECT_TRUE(check.full_charge_idle_on_dock);
  EXPECT_EQ(check.dock_occupancy_state, "DOCKED_CHARGE_IDLE");
  EXPECT_TRUE(check.final_auto_undock_required);
  EXPECT_TRUE(module.read_latch().latched_docked);
}

TEST(DockContactInterlockModuleTest, ConfirmedUndockAndStableNoContactClearStrongSession)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "undocked";
  harness.bms.have_state = true;
  harness.bms.fresh = true;
  harness.bms.contact = false;
  harness.bms.no_contact_duration_sec = 3.0;
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());
  module.update_latch(true, "charging_session", "charging_observed", "dock_main");

  harness.timestamp = "1001";
  harness.wall_time = 1001.0;
  const auto check = module.snapshot();

  EXPECT_TRUE(check.dock_contact_latch_auto_cleared);
  EXPECT_EQ(
    check.dock_contact_latch_clear_reason,
    "charging_session_latch_cleared_confirmed_undocked_no_contact");
  EXPECT_EQ(check.dock_occupancy_state, "CONFIRMED_UNDOCKED");
  EXPECT_FALSE(check.final_auto_undock_required);
  EXPECT_FALSE(module.read_latch().latched_docked);
}

TEST(DockContactInterlockModuleTest, StaleWeakBmsLatchDoesNotRequireAutoUndock)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "stopped";
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());
  module.update_latch(true, "bms", "legacy_contact", "dock_main");

  harness.wall_time = 1401.0;
  const auto check = module.snapshot();

  EXPECT_TRUE(check.dock_contact_latch_stale);
  EXPECT_FALSE(check.latch_valid_for_auto_undock);
  EXPECT_EQ(check.dock_occupancy_state, "UNKNOWN");
  EXPECT_EQ(check.dock_occupancy_reason, "stale_bms_latch_ignored");
  EXPECT_FALSE(check.final_auto_undock_required);
}

TEST(DockContactInterlockModuleTest, ActiveNavigationRejectsUnscopedBmsLatchWrite)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.navigation_active = true;
  harness.navigation_goal_running = true;
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());

  module.on_bms_contact_evidence(
    BatteryContactEvaluation{true, "current_above_threshold"}, true, 5.0);

  EXPECT_FALSE(module.read_latch().valid);
}

TEST(DockContactInterlockModuleTest, IdleRuntimeRejectsUnscopedBmsLatchWrite)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_active = false;
  harness.runtime.docking_state = "stopped";
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());

  module.on_bms_contact_evidence(
    BatteryContactEvaluation{true, "power_supply_status=FULL"}, true, 5.0);

  EXPECT_FALSE(module.read_latch().valid);
}

TEST(DockContactInterlockModuleTest, StrongDockLatchSurvivesNoContactUntilExplicitUndock)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "stopped";
  harness.bms.have_state = true;
  harness.bms.fresh = true;
  harness.bms.contact = false;
  harness.bms.no_contact_duration_sec = 30.0;
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());
  module.update_latch(true, "docking_job", "docked_success", "dock_main");

  const auto check = module.snapshot();

  EXPECT_FALSE(check.dock_contact_latch_auto_cleared);
  EXPECT_EQ(check.dock_occupancy_state, "UNCERTAIN_ON_DOCK");
  EXPECT_TRUE(check.final_is_docked_or_charging);
  EXPECT_TRUE(check.final_auto_undock_required);
  EXPECT_TRUE(check.can_auto_undock);
}

TEST(DockContactInterlockModuleTest, LegacyManagerBmsSessionRemainsStrongDockEvidence)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "stopped";
  harness.bms.have_state = true;
  harness.bms.fresh = true;
  harness.bms.contact = false;
  harness.bms.no_contact_duration_sec = 30.0;
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());
  module.update_latch(
    true, "docking_manager_bms", "historical_stable_contact", "dock_main");

  const auto check = module.snapshot();

  EXPECT_EQ(check.dock_contact_latch_source_strength, "strong");
  EXPECT_EQ(check.dock_occupancy_state, "UNCERTAIN_ON_DOCK");
  EXPECT_TRUE(check.final_auto_undock_required);
}

TEST(DockContactInterlockModuleTest, ConfirmedUndockOverridesFullSoc)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "undocked";
  harness.bms.have_state = true;
  harness.bms.have_soc = true;
  harness.bms.fresh = true;
  harness.bms.contact = false;
  harness.bms.no_contact_duration_sec = 3.0;
  harness.bms.soc = 100.0;
  harness.bms.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());
  module.update_latch(true, "docking_job", "docked_success", "dock_main");
  module.update_latch(false, "docking_job", "undocked_success", "dock_main");

  const auto check = module.snapshot();

  EXPECT_EQ(check.dock_occupancy_state, "CONFIRMED_UNDOCKED");
  EXPECT_FALSE(check.final_is_docked_or_charging);
  EXPECT_FALSE(check.final_auto_undock_required);
}

TEST(DockContactInterlockModuleTest, DiagnosticJsonPreservesCommercialFieldNames)
{
  TemporaryLatchFile latch;
  Harness harness;
  harness.runtime.docking_state = "docked";
  harness.runtime.docking_status = "docked contact";
  DockContactInterlockModule module(config_for(latch.path()), harness.ports());
  const auto check = module.snapshot();

  const auto json = module.pre_navigation_check_json(
    check,
    PreNavigationDockCheckContext{"navigation_pre_goal_check", "dock_main", "B10", "F1", "map",
      false});

  EXPECT_NE(json.find("\"schema\":\"njrh.pre_navigation_dock_check.v1\""), std::string::npos);
  EXPECT_NE(json.find("\"dock_contact_snapshot\":"), std::string::npos);
  EXPECT_NE(json.find("\"dock_occupancy_state\":\"CONFIRMED_DOCKED\""), std::string::npos);
  EXPECT_NE(json.find("\"final_auto_undock_required\":true"), std::string::npos);
  EXPECT_NE(json.find("\"status_topic\":\"/docking/status\""), std::string::npos);
}

}  // namespace
}  // namespace robot_api_server::features::docking

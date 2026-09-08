#include "robot_api_server/features/docking/lifecycle/dock_contact_interlock_module.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "sensor_msgs/msg/battery_state.hpp"

#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace fs = std::filesystem;

namespace robot_api_server::features::docking
{
namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::optional<double> parse_utc_iso8601_seconds(const std::string & text)
{
  if (text.empty()) {
    return std::nullopt;
  }
  std::tm tm{};
  std::istringstream input(text);
  input >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  if (input.fail()) {
    return std::nullopt;
  }
#if defined(_WIN32)
  const auto epoch = _mkgmtime(&tm);
#else
  const auto epoch = timegm(&tm);
#endif
  if (epoch < 0) {
    return std::nullopt;
  }
  return static_cast<double>(epoch);
}

bool latch_source_is_bms(const std::string & source)
{
  return lower_copy(source) == "bms";
}

bool latch_source_is_charging_session(const std::string & source)
{
  const auto normalized = lower_copy(source);
  return normalized == "charging_session" || normalized == "docking_manager_bms";
}

bool latch_source_is_docking_evidence(const std::string & source)
{
  const auto normalized = lower_copy(source);
  return normalized == "docking_job" ||
         normalized == "docking_status" ||
         normalized == "docking_manager" ||
         normalized == "charging_session" ||
         normalized == "docking_manager_bms";
}

bool latch_source_is_manual_evidence(const std::string & source)
{
  const auto normalized = lower_copy(source);
  return normalized == "manual" ||
         normalized == "manual_confirm" ||
         normalized == "manual_clear";
}

std::string dock_latch_source_strength(const std::string & source)
{
  if (latch_source_is_charging_session(source) || latch_source_is_docking_evidence(source)) {
    return "strong";
  }
  if (latch_source_is_manual_evidence(source)) {
    return "manual";
  }
  if (latch_source_is_bms(source)) {
    return "weak";
  }
  if (source.empty() || lower_copy(source) == "none" || lower_copy(source) == "unknown") {
    return "none";
  }
  return "unknown";
}

std::string json_nullable_number(const bool valid, const double value)
{
  if (!valid || !std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out << value;
  return out.str();
}

std::string json_string_array_fragment(const std::vector<std::string> & values)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << json_string(values[i]);
  }
  out << "]";
  return out.str();
}

}  // namespace

class DockContactInterlockModule::Impl
{
public:
  Impl(DockContactInterlockConfig config, DockContactInterlockPorts ports)
  : config_(std::move(config)), ports_(std::move(ports))
  {
    if (!ports_.runtime_snapshot || !ports_.bms_snapshot ||
      !ports_.safety_interlock_snapshot || !ports_.dock_zone_snapshot ||
      !ports_.navigation_goal_running || !ports_.wall_time_seconds ||
      !ports_.timestamp_now || !ports_.warn)
    {
      throw std::invalid_argument(
              "dock contact interlock module requires every integration port");
    }
  }

  double latch_age_sec(const DockContactLatchSnapshot & snapshot) const
  {
    const auto stamp = !snapshot.latched_at.empty() ? snapshot.latched_at : snapshot.updated_at;
    try {
      std::size_t parsed = 0U;
      const double seconds = std::stod(stamp, &parsed);
      if (parsed == stamp.size() && std::isfinite(seconds)) {
        return std::max(0.0, ports_.wall_time_seconds() - seconds);
      }
    } catch (const std::exception &) {
    }
    const auto stamp_seconds = parse_utc_iso8601_seconds(stamp);
    if (!stamp_seconds) {
      return -1.0;
    }
    return std::max(0.0, ports_.wall_time_seconds() - *stamp_seconds);
  }

  void refresh_derived_fields(DockContactLatchSnapshot & snapshot) const
  {
    snapshot.source_bms = latch_source_is_bms(snapshot.source);
    snapshot.source_charging_session = latch_source_is_charging_session(snapshot.source);
    snapshot.age_sec = latch_age_sec(snapshot);
    snapshot.stale =
      snapshot.valid &&
      snapshot.latched_docked &&
      snapshot.source_bms &&
      snapshot.age_sec >= 0.0 &&
      config_.bms_ttl_sec > 0.0 &&
      snapshot.age_sec > config_.bms_ttl_sec;
  }

  DockContactLatchSnapshot read_latch() const
  {
    DockContactLatchSnapshot snapshot;
    if (config_.latch_file.empty()) {
      snapshot.reason = "latch_file_not_configured";
      return snapshot;
    }
    try {
      const auto text = read_optional_text_file(config_.latch_file);
      if (text.empty()) {
        snapshot.reason = "latch_file_missing";
        return snapshot;
      }
      snapshot.valid = true;
      snapshot.latched_docked =
        json_bool_value(text, "latched_docked", json_bool_value(text, "docked", false));
      snapshot.docked = snapshot.latched_docked;
      snapshot.source = json_string_value(text, "source").value_or("unknown");
      snapshot.reason = json_string_value(text, "reason").value_or("no_reason");
      snapshot.building_id = json_string_value(text, "building_id").value_or("");
      snapshot.floor_id = json_string_value(text, "floor_id").value_or("");
      snapshot.map_id = json_string_value(text, "map_id").value_or("");
      snapshot.dock_id = json_string_value(text, "dock_id").value_or("");
      snapshot.latched_at = json_string_value(text, "latched_at").value_or("");
      snapshot.last_confirmed_at =
        json_string_value(text, "last_confirmed_at").value_or("");
      snapshot.cleared_at = json_string_value(text, "cleared_at").value_or("");
      snapshot.clear_reason = json_string_value(text, "clear_reason").value_or("");
      snapshot.note = json_string_value(text, "note").value_or("");
      snapshot.updated_at = json_string_value(text, "updated_at").value_or("");
      refresh_derived_fields(snapshot);
      return snapshot;
    } catch (const std::exception & exception) {
      snapshot.reason = std::string("latch_read_failed:") + exception.what();
      return snapshot;
    }
  }

  void update_latch(
    const bool docked,
    const std::string & source,
    const std::string & reason,
    const std::string & dock_id,
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id,
    const std::string & note)
  {
    if (config_.latch_file.empty()) {
      return;
    }
    const auto previous = read_latch();
    const bool current_file_already_matches =
      previous.valid &&
      previous.latched_docked == docked &&
      previous.source == source &&
      previous.reason == reason &&
      previous.dock_id == dock_id &&
      (note.empty() || previous.note == note);
    if (have_last_write_ &&
      docked == last_docked_ &&
      source == last_source_ &&
      reason == last_reason_ &&
      dock_id == last_dock_id_ &&
      note == last_note_ &&
      current_file_already_matches)
    {
      return;
    }
    const auto now_text = ports_.timestamp_now();
    const auto latched_at = docked ?
      (previous.latched_docked && !previous.latched_at.empty() ?
      previous.latched_at : now_text) :
      previous.latched_at;
    std::ostringstream body;
    body << "{\n"
         << "  \"schema\": \"njrh.docking_contact_latch.v1\",\n"
         << "  \"latched_docked\": " << (docked ? "true" : "false") << ",\n"
         << "  \"docked\": " << (docked ? "true" : "false") << ",\n"
         << "  \"source\": " << json_string(source) << ",\n"
         << "  \"reason\": " << json_string(reason) << ",\n"
         << "  \"building_id\": "
         << json_string(building_id.empty() ? previous.building_id : building_id) << ",\n"
         << "  \"floor_id\": "
         << json_string(floor_id.empty() ? previous.floor_id : floor_id) << ",\n"
         << "  \"map_id\": "
         << json_string(map_id.empty() ? previous.map_id : map_id) << ",\n"
         << "  \"dock_id\": " << json_string(dock_id) << ",\n"
         << "  \"latched_at\": " << json_string(latched_at) << ",\n"
         << "  \"last_confirmed_at\": "
         << json_string(docked ? now_text : previous.last_confirmed_at) << ",\n"
         << "  \"cleared_at\": " << json_string(docked ? "" : now_text) << ",\n"
         << "  \"clear_reason\": " << json_string(docked ? "" : reason) << ",\n"
         << "  \"note\": " << json_string(note.empty() ? previous.note : note) << ",\n"
         << "  \"updated_at\": " << json_string(now_text) << "\n"
         << "}\n";
    try {
      fs::create_directories(config_.latch_file.parent_path());
      const auto temporary_path = fs::path(config_.latch_file.string() + ".tmp");
      write_text_file(temporary_path, body.str());
      fs::rename(temporary_path, config_.latch_file);
      have_last_write_ = true;
      last_docked_ = docked;
      last_source_ = source;
      last_reason_ = reason;
      last_dock_id_ = dock_id;
      last_note_ = note;
    } catch (const std::exception & exception) {
      ports_.warn(
        "failed to write docking contact latch " + config_.latch_file.string() + ": " +
        exception.what());
    }
  }

  bool bms_latch_write_allowed_by_runtime() const
  {
    const auto runtime = ports_.runtime_snapshot();
    const auto docking_state = lower_copy(runtime.docking_state);
    const auto docking_status = lower_copy(runtime.docking_status);
    const bool docking_context =
      runtime.docking_active ||
      docking_state == "docked" ||
      docking_state == "charging" ||
      docking_state == "docking" ||
      docking_state == "undocking" ||
      starts_with(docking_status, "docked") ||
      starts_with(docking_status, "charging") ||
      ::robot_api_server::docking_status_is_undocking(docking_status);
    if (docking_context) {
      return true;
    }
    // A BMS sample observed while the robot is merely idle is not enough to
    // create persistent on-dock state.  Strong persistence is established by
    // a docking transaction/status (or an explicit operator confirmation),
    // and is then retained until a proven undock clears it.
    return false;
  }

  void on_bms_contact_evidence(
    const BatteryContactEvaluation & charging_contact,
    const bool contact_stable,
    const double stable_duration_sec)
  {
    if (!charging_contact.contact || !contact_stable) {
      return;
    }
    if (!bms_latch_write_allowed_by_runtime()) {
      return;
    }
    (void)stable_duration_sec;
    update_latch(
      true,
      "charging_session",
      "bms_charging_observed:" + charging_contact.reason,
      "",
      "",
      "",
      "",
      "charging_session_latch_from_stable_bms_contact");
  }

  void copy_latch_fields(PreNavigationDockCheck & check) const
  {
    check.dock_latch_indicates_docked = check.dock_latch.valid && check.dock_latch.docked;
    check.dock_contact_latch_present = check.dock_latch.valid;
    check.dock_contact_latch_latched_docked = check.dock_latch_indicates_docked;
    check.dock_contact_latch_source = check.dock_latch.source;
    check.dock_contact_latch_reason = check.dock_latch.reason;
    check.dock_contact_latch_age_sec = check.dock_latch.age_sec;
    check.dock_contact_latch_stale = check.dock_latch.stale;
    check.dock_contact_latch_source_strength =
      dock_latch_source_strength(check.dock_latch.source);
    check.charging_session_latched =
      check.dock_latch_indicates_docked && check.dock_latch.source_charging_session;
    check.charging_session_age_sec =
      check.charging_session_latched ? check.dock_latch.age_sec : -1.0;
    check.charging_session_last_confirmed_at =
      check.charging_session_latched ? check.dock_latch.last_confirmed_at : "";
  }

  PreNavigationDockCheck snapshot()
  {
    PreNavigationDockCheck check;
    check.runtime = ports_.runtime_snapshot();
    check.bms = ports_.bms_snapshot();
    check.dock_latch = read_latch();
    check.safety_interlock = ports_.safety_interlock_snapshot();
    check.safety_interlock_memory_latched =
      check.safety_interlock.available &&
      check.safety_interlock.fresh &&
      check.safety_interlock.memory_latched;

    const auto docking_state = lower_copy(check.runtime.docking_state);
    const auto docking_status = lower_copy(check.runtime.docking_status);
    check.runtime_state_docked = docking_state == "docked";
    check.runtime_state_charging = docking_state == "charging";
    check.runtime_state_undocking = docking_state == "undocking";
    check.live_docking_state_undocked =
      docking_state == "undocked" ||
      ::robot_api_server::docking_status_is_undocked(docking_status);
    check.docking_status_indicates_docked = starts_with(docking_status, "docked");
    check.docking_status_indicates_charging = starts_with(docking_status, "charging");
    check.docking_status_indicates_undocking =
      ::robot_api_server::docking_status_is_undocking(docking_status);
    copy_latch_fields(check);
    check.live_bms_charging_contact_stable =
      check.bms.have_state && check.bms.fresh && check.bms.contact_stable;

    const bool live_bms_no_contact_stable =
      check.bms.have_state &&
      check.bms.fresh &&
      !check.bms.contact &&
      check.bms.no_contact_duration_sec >= config_.bms_clear_no_contact_sec;
    const bool live_not_docked_or_charging =
      !check.runtime_state_docked &&
      !check.runtime_state_charging &&
      !check.docking_status_indicates_docked &&
      !check.docking_status_indicates_charging &&
      !check.bms.contact;
    const bool live_undocked_or_no_docking_context =
      check.live_docking_state_undocked ||
      (!check.runtime.docking_active &&
      !check.runtime_state_undocking &&
      !check.docking_status_indicates_undocking);
    const bool bms_latch_can_be_cleared_by_live_no_contact =
      check.dock_latch.source_bms && live_undocked_or_no_docking_context;
    const bool charging_session_latch_can_be_cleared_by_live_no_contact =
      check.dock_latch.source_charging_session && check.live_docking_state_undocked;
    const bool latch_source_can_be_cleared_by_live_no_contact =
      bms_latch_can_be_cleared_by_live_no_contact ||
      charging_session_latch_can_be_cleared_by_live_no_contact;
    check.dock_contact_latch_contradicted_by_live_state =
      config_.clear_when_live_undocked_no_contact &&
      check.dock_latch_indicates_docked &&
      latch_source_can_be_cleared_by_live_no_contact &&
      live_not_docked_or_charging &&
      live_bms_no_contact_stable;
    if (check.dock_contact_latch_contradicted_by_live_state) {
      check.dock_contact_latch_auto_cleared = true;
      check.dock_contact_latch_clear_reason = check.dock_latch.source_charging_session ?
        "charging_session_latch_cleared_confirmed_undocked_no_contact" :
        "stale_bms_latch_cleared_live_undocked_no_contact";
      const std::string clear_note = check.dock_latch.source_charging_session ?
        "source=" + check.dock_latch.source + " confirmed_undocked_no_contact" :
        "source=" + check.dock_latch.source +
        " live_undocked_or_no_docking_context_no_contact";
      update_latch(
        false,
        "auto_clear",
        check.dock_contact_latch_clear_reason,
        check.dock_latch.dock_id,
        check.dock_latch.building_id,
        check.dock_latch.floor_id,
        check.dock_latch.map_id,
        clear_note);
      check.dock_latch = read_latch();
      check.dock_latch.contradicted_by_live_state = true;
      copy_latch_fields(check);
    }

    const bool latch_source_valid_for_auto_undock =
      latch_source_is_docking_evidence(check.dock_latch.source) ||
      latch_source_is_manual_evidence(check.dock_latch.source) ||
      (check.dock_latch.source_bms && config_.allow_bms_stale_auto_undock);
    check.latch_valid_for_auto_undock =
      check.dock_latch_indicates_docked &&
      !check.dock_contact_latch_stale &&
      !check.dock_contact_latch_contradicted_by_live_state &&
      latch_source_valid_for_auto_undock;
    check.strong_live_docked =
      check.runtime_state_docked ||
      check.runtime_state_charging ||
      check.docking_status_indicates_docked ||
      check.docking_status_indicates_charging ||
      check.live_bms_charging_contact_stable;

    if (check.runtime_state_docked) {
      check.docked_evidence.push_back("runtime_state:docked");
    }
    if (check.runtime_state_charging) {
      check.docked_evidence.push_back("runtime_state:charging");
    }
    if (check.docking_status_indicates_docked) {
      check.docked_evidence.push_back("docking_status:docked");
    }
    if (check.docking_status_indicates_charging) {
      check.docked_evidence.push_back("docking_status:charging");
    }
    if (check.live_bms_charging_contact_stable) {
      check.docked_evidence.push_back("bms:" + check.bms.reason);
    } else if (check.bms.contact) {
      check.docked_warnings.push_back("bms_contact_unstable:" + check.bms.reason);
    }
    if (check.latch_valid_for_auto_undock) {
      check.docked_evidence.push_back(
        "latch:" + check.dock_latch.source + ":" + check.dock_latch.reason);
    } else if (check.dock_latch_indicates_docked) {
      check.docked_warnings.push_back(
        "dock_latch_ignored:" + check.dock_latch.source + ":" + check.dock_latch.reason);
    }
    if (!check.bms.have_state) {
      check.docked_warnings.push_back("no_bms_state");
    } else if (!check.bms.fresh) {
      check.docked_warnings.push_back("stale_bms_state");
    } else if (!check.bms.contact) {
      check.docked_warnings.push_back("bms_contact_false:" + check.bms.reason);
    }
    if (!check.dock_latch.valid) {
      check.docked_warnings.push_back("dock_latch_unavailable:" + check.dock_latch.reason);
    } else if (check.dock_contact_latch_stale) {
      check.docked_warnings.push_back("stale_bms_dock_latch");
    } else if (
      check.dock_latch.source_bms &&
      check.dock_latch.age_sec >= 0.0 &&
      config_.max_age_warn_sec > 0.0 &&
      check.dock_latch.age_sec > config_.max_age_warn_sec)
    {
      check.docked_warnings.push_back("old_bms_dock_latch");
    }
    if (check.dock_contact_latch_auto_cleared) {
      check.docked_warnings.push_back(check.dock_contact_latch_clear_reason);
    }

    const bool bms_fresh_no_contact =
      check.bms.have_state && check.bms.fresh && !check.bms.contact;
    const bool bms_current_idle =
      !check.bms.have_state ||
      !std::isfinite(check.bms.current) ||
      std::abs(check.bms.current) <= config_.charging_current_min_a;
    const bool bms_soc_full =
      check.bms.have_soc && std::isfinite(check.bms.soc) &&
      check.bms.soc >= config_.full_soc_threshold_pct;
    const bool bms_status_full =
      check.bms.power_supply_status ==
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    const bool strong_session_latch =
      check.dock_latch_indicates_docked &&
      !check.dock_contact_latch_stale &&
      !check.dock_contact_latch_contradicted_by_live_state &&
      (latch_source_is_charging_session(check.dock_latch.source) ||
      latch_source_is_docking_evidence(check.dock_latch.source) ||
      latch_source_is_manual_evidence(check.dock_latch.source));
    check.full_charge_idle_on_dock =
      strong_session_latch &&
      bms_fresh_no_contact &&
      bms_current_idle &&
      (bms_soc_full || bms_status_full);
    if (check.full_charge_idle_on_dock) {
      check.docked_evidence.push_back(
        "full_charge_idle_on_dock:latch=" + check.dock_latch.source + ":" +
        check.dock_latch.reason);
    }
    check.inferred_docked =
      !check.runtime_state_docked &&
      (check.strong_live_docked || check.latch_valid_for_auto_undock);

    if (check.runtime_state_undocking || check.docking_status_indicates_undocking) {
      check.dock_occupancy_state = "UNCERTAIN_ON_DOCK";
      check.dock_occupancy_reason = "undocking_already_active";
      check.dock_occupancy_evidence.push_back("undocking_active");
    } else if (
      check.runtime_state_charging ||
      check.docking_status_indicates_charging ||
      check.live_bms_charging_contact_stable)
    {
      check.dock_occupancy_state = "DOCKED_CHARGING";
      check.dock_occupancy_reason = "live_charging_evidence";
      if (check.runtime_state_charging) {
        check.dock_occupancy_evidence.push_back("runtime_state:charging");
      }
      if (check.docking_status_indicates_charging) {
        check.dock_occupancy_evidence.push_back("docking_status:charging");
      }
      if (check.live_bms_charging_contact_stable) {
        check.dock_occupancy_evidence.push_back("bms:" + check.bms.reason);
      }
    } else if (check.runtime_state_docked || check.docking_status_indicates_docked) {
      check.dock_occupancy_state = "CONFIRMED_DOCKED";
      check.dock_occupancy_reason = "live_docked_evidence";
      if (check.runtime_state_docked) {
        check.dock_occupancy_evidence.push_back("runtime_state:docked");
      }
      if (check.docking_status_indicates_docked) {
        check.dock_occupancy_evidence.push_back("docking_status:docked");
      }
    } else if (check.full_charge_idle_on_dock) {
      check.dock_occupancy_state = "DOCKED_CHARGE_IDLE";
      check.dock_occupancy_reason = "charging_session_or_docking_latch_with_bms_idle";
      check.dock_occupancy_evidence.push_back(
        "latch:" + check.dock_latch.source + ":" + check.dock_latch.reason);
      if (bms_soc_full) {
        check.dock_occupancy_evidence.push_back("bms_soc_full");
      }
      if (bms_current_idle) {
        check.dock_occupancy_evidence.push_back("bms_current_idle");
      }
      if (!check.bms.present) {
        check.dock_occupancy_evidence.push_back("bms_present_false");
      }
    } else if (strong_session_latch) {
      check.dock_occupancy_state = "UNCERTAIN_ON_DOCK";
      check.dock_occupancy_reason = "recent_strong_dock_or_charging_session_latch";
      check.dock_occupancy_evidence.push_back(
        "latch:" + check.dock_latch.source + ":" + check.dock_latch.reason);
    } else if (check.live_docking_state_undocked && !check.dock_latch_indicates_docked) {
      check.dock_occupancy_state = "CONFIRMED_UNDOCKED";
      check.dock_occupancy_reason = "live_docking_state_undocked_without_dock_latch";
      check.dock_occupancy_evidence.push_back("docking_status:undocked");
    } else if (check.dock_contact_latch_stale && check.dock_latch.source_bms) {
      check.dock_occupancy_state = "UNKNOWN";
      check.dock_occupancy_reason = "stale_bms_latch_ignored";
      check.dock_occupancy_evidence.push_back("stale_bms_latch");
    } else {
      check.dock_occupancy_state = "UNKNOWN";
      check.dock_occupancy_reason = "no_strong_dock_evidence";
    }

    check.resolved_dock_id = !check.runtime.docking_dock_id.empty() ?
      check.runtime.docking_dock_id :
      (!check.dock_latch.dock_id.empty() ?
      check.dock_latch.dock_id : check.safety_interlock.dock_id);
    if (check.resolved_dock_id == "none") {
      check.resolved_dock_id.clear();
    }
    check.safety_interlock_state_block =
      config_.require_safety_interlock_state &&
      (!check.safety_interlock.available || !check.safety_interlock.fresh);
    if (check.safety_interlock_state_block) {
      check.pre_navigation_block_reason = !check.safety_interlock.available ?
        "dock safety interlock state has not been received" :
        "dock safety interlock state is stale";
      check.docked_warnings.push_back(
        !check.safety_interlock.available ?
        "safety_interlock_state_unavailable" : "safety_interlock_state_stale");
    }

    if (check.safety_interlock_memory_latched) {
      check.dock_zone = ports_.dock_zone_snapshot(
        check.dock_latch, check.safety_interlock, check.runtime);
      const bool proven_outside =
        check.dock_zone.state == "CLEAR" &&
        !check.safety_interlock.live_bms_contact;
      if (proven_outside) {
        check.clear_stale_safety_interlock_required = true;
        check.dock_occupancy_state = "CONFIRMED_UNDOCKED";
        check.dock_occupancy_reason = "safety_memory_latch_but_robot_proven_outside_dock_zone";
        check.dock_occupancy_evidence.push_back(
          "dock_zone_clear:" + check.dock_zone.reason);
      } else {
        check.dock_occupancy_state = "UNCERTAIN_ON_DOCK";
        check.dock_occupancy_reason = check.safety_interlock.live_bms_contact ?
          "safety_memory_latch_with_live_bms_contact" :
          (check.dock_zone.state == "NEAR" ?
          "safety_memory_latch_near_dock" :
          "safety_memory_latch_position_unknown");
        check.dock_occupancy_evidence.push_back(
          "robot_safety_memory_latch:" + check.safety_interlock.reason);
        check.dock_occupancy_evidence.push_back(
          "dock_zone:" + check.dock_zone.state + ":" + check.dock_zone.reason);
      }
    }

    check.final_is_docked_or_charging =
      check.dock_occupancy_state == "CONFIRMED_DOCKED" ||
      check.dock_occupancy_state == "DOCKED_CHARGING" ||
      check.dock_occupancy_state == "DOCKED_CHARGE_IDLE" ||
      check.dock_occupancy_state == "UNCERTAIN_ON_DOCK";
    check.final_auto_undock_required = check.final_is_docked_or_charging;
    check.docking_active_not_docked_block =
      check.runtime.docking_active &&
      !check.final_is_docked_or_charging &&
      !check.runtime_state_undocking &&
      !check.docking_status_indicates_undocking;
    check.can_auto_undock =
      check.final_auto_undock_required && !check.docking_active_not_docked_block;
    if (check.safety_interlock_state_block) {
      check.pre_navigation_recovery_action = "BLOCK";
      check.pre_navigation_recovery_required = true;
      check.can_auto_undock = false;
    } else if (check.clear_stale_safety_interlock_required) {
      check.pre_navigation_recovery_action = "CLEAR_STALE_INTERLOCK";
      check.pre_navigation_recovery_required = true;
    } else if (check.final_auto_undock_required) {
      check.pre_navigation_recovery_action = "CONTROLLED_UNDOCK";
      check.pre_navigation_recovery_required = true;
    }

    if (check.strong_live_docked) {
      check.docked_state_class = "DOCKED_CONFIRMED";
    } else if (check.latch_valid_for_auto_undock) {
      check.docked_state_class = "DOCKED_LATCHED";
    } else if (check.docking_active_not_docked_block) {
      check.docked_state_class = "UNKNOWN";
    } else if (check.live_docking_state_undocked) {
      check.docked_state_class = "NOT_DOCKED";
    } else {
      check.docked_state_class = "UNKNOWN";
    }

    if (check.safety_interlock_state_block) {
      check.auto_undock_reason = "safety_interlock_state_unavailable_or_stale";
    } else if (check.clear_stale_safety_interlock_required) {
      check.auto_undock_reason = "safety_interlock_stale_outside_dock";
    } else if (check.safety_interlock_memory_latched &&
      check.safety_interlock.live_bms_contact)
    {
      check.auto_undock_reason = "safety_interlock_latched_live_contact";
    } else if (check.safety_interlock_memory_latched && check.dock_zone.state == "NEAR") {
      check.auto_undock_reason = "safety_interlock_latched_near_dock";
    } else if (check.safety_interlock_memory_latched) {
      check.auto_undock_reason = "safety_interlock_latched_position_unknown";
    } else if (check.final_auto_undock_required) {
      check.auto_undock_reason =
        "dock_occupancy_state:" + check.dock_occupancy_state + ":" +
        check.dock_occupancy_reason;
    } else if (check.runtime_state_undocking || check.docking_status_indicates_undocking) {
      check.auto_undock_reason = "undocking_already_active";
    } else if (check.runtime_state_docked) {
      check.auto_undock_reason = "runtime_docking_state_docked";
    } else if (check.runtime_state_charging) {
      check.auto_undock_reason = "runtime_docking_state_charging";
    } else if (check.docking_status_indicates_charging) {
      check.auto_undock_reason = "docking_status_charging";
    } else if (check.docking_status_indicates_docked) {
      check.auto_undock_reason = "docking_status_docked";
    } else if (check.live_bms_charging_contact_stable) {
      check.auto_undock_reason = "bms_charging_contact:" + check.bms.reason;
    } else if (check.latch_valid_for_auto_undock) {
      check.auto_undock_reason =
        "dock_contact_latch:" + check.dock_latch.source + ":" + check.dock_latch.reason;
    } else if (check.dock_contact_latch_auto_cleared) {
      check.auto_undock_reason = check.dock_contact_latch_clear_reason;
    } else if (check.dock_contact_latch_stale) {
      check.auto_undock_reason = "stale_bms_latch_ignored";
    } else if (check.docking_active_not_docked_block) {
      check.auto_undock_reason =
        "docking_active_not_docked:" + check.runtime.docking_state;
    } else if (check.dock_occupancy_state == "CONFIRMED_UNDOCKED") {
      check.auto_undock_reason = "confirmed_undocked";
    } else {
      check.auto_undock_reason = "not_docked";
    }
    return check;
  }

  std::string bms_snapshot_json(const BmsChargingContactSnapshot & bms) const
  {
    std::ostringstream body;
    body << "\"have_state\":" << (bms.have_state ? "true" : "false") << ","
         << "\"fresh\":" << (bms.fresh ? "true" : "false") << ","
         << "\"age_sec\":" << json_nullable_number(bms.have_state, bms.age_sec) << ","
         << "\"contact\":" << (bms.contact ? "true" : "false") << ","
         << "\"contact_stable\":" << (bms.contact_stable ? "true" : "false") << ","
         << "\"contact_stable_duration_sec\":"
         << json_nullable_number(bms.have_state, bms.contact_stable_duration_sec) << ","
         << "\"no_contact_duration_sec\":"
         << json_nullable_number(bms.have_state, bms.no_contact_duration_sec) << ","
         << "\"reason\":" << json_string(bms.reason) << ","
         << "\"soc\":" << json_nullable_number(bms.have_soc, bms.soc) << ","
         << "\"soc_valid\":" << (bms.have_soc && bms.fresh ? "true" : "false") << ","
         << "\"voltage\":" << json_nullable_number(bms.have_state, bms.voltage) << ","
         << "\"current\":" << json_nullable_number(bms.have_state, bms.current) << ","
         << "\"temperature\":"
         << json_nullable_number(bms.have_state, bms.temperature) << ","
         << "\"present\":" << (bms.present ? "true" : "false") << ","
         << "\"power_supply_status\":" << bms.power_supply_status << ","
         << "\"power_supply_health\":" << bms.power_supply_health << ","
         << "\"power_supply_technology\":" << bms.power_supply_technology;
    return body.str();
  }

  std::string latch_snapshot_json(const DockContactLatchSnapshot & latch) const
  {
    std::ostringstream body;
    body << "{"
         << "\"valid\":" << (latch.valid ? "true" : "false") << ","
         << "\"latched_docked\":" << (latch.latched_docked ? "true" : "false") << ","
         << "\"docked\":" << (latch.docked ? "true" : "false") << ","
         << "\"source\":" << json_string(latch.source) << ","
         << "\"reason\":" << json_string(latch.reason) << ","
         << "\"building_id\":" << json_string(latch.building_id) << ","
         << "\"floor_id\":" << json_string(latch.floor_id) << ","
         << "\"map_id\":" << json_string(latch.map_id) << ","
         << "\"dock_id\":" << json_string(latch.dock_id) << ","
         << "\"latched_at\":" << json_string(latch.latched_at) << ","
         << "\"last_confirmed_at\":" << json_string(latch.last_confirmed_at) << ","
         << "\"cleared_at\":" << json_string(latch.cleared_at) << ","
         << "\"clear_reason\":" << json_string(latch.clear_reason) << ","
         << "\"note\":" << json_string(latch.note) << ","
         << "\"age_sec\":" << json_nullable_number(latch.age_sec >= 0.0, latch.age_sec) << ","
         << "\"source_bms\":" << (latch.source_bms ? "true" : "false") << ","
         << "\"source_charging_session\":"
         << (latch.source_charging_session ? "true" : "false") << ","
         << "\"source_strength\":" << json_string(dock_latch_source_strength(latch.source))
         << ","
         << "\"stale\":" << (latch.stale ? "true" : "false") << ","
         << "\"contradicted_by_live_state\":"
         << (latch.contradicted_by_live_state ? "true" : "false") << ","
         << "\"updated_at\":" << json_string(latch.updated_at) << "}";
    return body.str();
  }

  std::string pre_navigation_check_json(
    const PreNavigationDockCheck & check,
    const PreNavigationDockCheckContext & context) const
  {
    std::ostringstream body;
    body << "{"
         << "\"schema\":\"njrh.pre_navigation_dock_check.v1\","
         << "\"request_source\":" << json_string(context.request_source) << ","
         << "\"pose_id\":" << json_string(context.pose_id) << ","
         << "\"building_id\":" << json_string(context.building_id) << ","
         << "\"floor_id\":" << json_string(context.floor_id) << ","
         << "\"frame_id\":" << json_string(context.frame_id) << ","
         << "\"direct_pose\":" << (context.direct_pose ? "true" : "false") << ","
         << "\"api_bms_charging_contact\":" << (check.bms.contact ? "true" : "false")
         << ","
         << "\"api_bms_charging_contact_stable\":"
         << (check.live_bms_charging_contact_stable ? "true" : "false") << ","
         << "\"api_bms_charging_contact_reason\":" << json_string(check.bms.reason) << ","
         << "\"bms\":{" << bms_snapshot_json(check.bms) << "},"
         << "\"safety_interlock\":{"
         << "\"available\":" << (check.safety_interlock.available ? "true" : "false") << ","
         << "\"fresh\":" << (check.safety_interlock.fresh ? "true" : "false") << ","
         << "\"age_sec\":"
         << json_nullable_number(
      check.safety_interlock.age_sec >= 0.0, check.safety_interlock.age_sec) << ","
         << "\"enabled\":" << (check.safety_interlock.enabled ? "true" : "false") << ","
         << "\"memory_latched\":"
         << (check.safety_interlock.memory_latched ? "true" : "false") << ","
         << "\"active\":" << (check.safety_interlock.active ? "true" : "false") << ","
         << "\"battery_sample_fresh\":"
         << (check.safety_interlock.battery_sample_fresh ? "true" : "false") << ","
         << "\"live_bms_contact\":"
         << (check.safety_interlock.live_bms_contact ? "true" : "false") << ","
         << "\"no_contact_duration_sec\":"
         << check.safety_interlock.no_contact_duration_sec << ","
         << "\"reverse_session_seen\":"
         << (check.safety_interlock.reverse_session_seen ? "true" : "false") << ","
         << "\"reverse_permit_active\":"
         << (check.safety_interlock.reverse_permit_active ? "true" : "false") << ","
         << "\"state\":" << json_string(check.safety_interlock.state) << ","
         << "\"reason\":" << json_string(check.safety_interlock.reason) << "},"
         << "\"dock_zone\":{"
         << "\"state\":" << json_string(check.dock_zone.state) << ","
         << "\"reason\":" << json_string(check.dock_zone.reason) << ","
         << "\"dock_id\":" << json_string(check.dock_zone.dock_id) << ","
         << "\"distance_m\":"
         << json_nullable_number(check.dock_zone.distance_m >= 0.0, check.dock_zone.distance_m)
         << "},"
         << "\"dock_contact_snapshot\":" << latch_snapshot_json(check.dock_latch) << ","
         << "\"dock_contact_latch_present\":"
         << (check.dock_contact_latch_present ? "true" : "false") << ","
         << "\"dock_contact_latch_latched_docked\":"
         << (check.dock_contact_latch_latched_docked ? "true" : "false") << ","
         << "\"dock_contact_latch_source\":"
         << json_string(check.dock_contact_latch_source) << ","
         << "\"dock_contact_latch_reason\":"
         << json_string(check.dock_contact_latch_reason) << ","
         << "\"dock_contact_latch_age_sec\":"
         << json_nullable_number(
      check.dock_contact_latch_age_sec >= 0.0, check.dock_contact_latch_age_sec) << ","
         << "\"dock_contact_latch_stale\":"
         << (check.dock_contact_latch_stale ? "true" : "false") << ","
         << "\"dock_contact_latch_contradicted_by_live_state\":"
         << (check.dock_contact_latch_contradicted_by_live_state ? "true" : "false") << ","
         << "\"dock_contact_latch_auto_cleared\":"
         << (check.dock_contact_latch_auto_cleared ? "true" : "false") << ","
         << "\"dock_contact_latch_clear_reason\":"
         << json_string(check.dock_contact_latch_clear_reason) << ","
         << "\"dock_contact_latch_source_strength\":"
         << json_string(check.dock_contact_latch_source_strength) << ","
         << "\"charging_session_latched\":"
         << (check.charging_session_latched ? "true" : "false") << ","
         << "\"charging_session_age_sec\":"
         << json_nullable_number(
      check.charging_session_age_sec >= 0.0,
      check.charging_session_age_sec)
         << ","
         << "\"charging_session_last_confirmed_at\":"
         << json_string(check.charging_session_last_confirmed_at) << ","
         << "\"bms_live_contact\":" << (check.bms.contact ? "true" : "false") << ","
         << "\"bms_live_contact_reason\":" << json_string(check.bms.reason) << ","
         << "\"bms_percentage\":" << json_nullable_number(check.bms.have_soc, check.bms.soc)
         << ","
         << "\"bms_current\":" << json_nullable_number(check.bms.have_state, check.bms.current)
         << ","
         << "\"bms_present\":" << (check.bms.present ? "true" : "false") << ","
         << "\"full_charge_idle_on_dock\":"
         << (check.full_charge_idle_on_dock ? "true" : "false") << ","
         << "\"docking\":{"
         << "\"state\":" << json_string(check.runtime.docking_state) << ","
         << "\"active\":" << (check.runtime.docking_active ? "true" : "false") << ","
         << "\"dock_id\":" << json_string(check.runtime.docking_dock_id) << ","
         << "\"last_status\":" << json_string(check.runtime.docking_status) << ","
         << "\"status_topic\":" << json_string(config_.docking_status_topic) << ","
         << "\"runtime_state_docked\":" << (check.runtime_state_docked ? "true" : "false")
         << ","
         << "\"runtime_state_charging\":"
         << (check.runtime_state_charging ? "true" : "false") << ","
         << "\"runtime_state_undocking\":"
         << (check.runtime_state_undocking ? "true" : "false") << ","
         << "\"status_indicates_docked\":"
         << (check.docking_status_indicates_docked ? "true" : "false") << ","
         << "\"status_indicates_charging\":"
         << (check.docking_status_indicates_charging ? "true" : "false") << ","
         << "\"live_docking_state_undocked\":"
         << (check.live_docking_state_undocked ? "true" : "false") << ","
         << "\"status_indicates_undocking\":"
         << (check.docking_status_indicates_undocking ? "true" : "false") << ","
         << "\"dock_latch_indicates_docked\":"
         << (check.dock_latch_indicates_docked ? "true" : "false") << "},"
         << "\"inferred_docked\":" << (check.inferred_docked ? "true" : "false") << ","
         << "\"latched_docked\":"
         << (check.dock_latch_indicates_docked ? "true" : "false") << ","
         << "\"latched_docked_source\":" << json_string(check.dock_latch.source) << ","
         << "\"latched_docked_age_sec\":"
         << json_nullable_number(check.dock_latch.age_sec >= 0.0, check.dock_latch.age_sec) << ","
         << "\"live_docking_state\":" << json_string(check.runtime.docking_state) << ","
         << "\"live_docking_status_indicates_docked\":"
         << (check.docking_status_indicates_docked ? "true" : "false") << ","
         << "\"live_docking_status_indicates_charging\":"
         << (check.docking_status_indicates_charging ? "true" : "false") << ","
         << "\"live_bms_charging_contact\":" << (check.bms.contact ? "true" : "false")
         << ","
         << "\"live_bms_charging_contact_stable\":"
         << (check.live_bms_charging_contact_stable ? "true" : "false") << ","
         << "\"strong_live_docked\":" << (check.strong_live_docked ? "true" : "false")
         << ","
         << "\"latch_valid_for_auto_undock\":"
         << (check.latch_valid_for_auto_undock ? "true" : "false") << ","
         << "\"docked_state_class\":" << json_string(check.docked_state_class) << ","
         << "\"docked_evidence\":" << json_string_array_fragment(check.docked_evidence) << ","
         << "\"docked_warnings\":" << json_string_array_fragment(check.docked_warnings) << ","
         << "\"dock_occupancy_state\":" << json_string(check.dock_occupancy_state) << ","
         << "\"dock_occupancy_evidence\":"
         << json_string_array_fragment(check.dock_occupancy_evidence) << ","
         << "\"dock_occupancy_reason\":" << json_string(check.dock_occupancy_reason) << ","
         << "\"final_is_docked_or_charging\":"
         << (check.final_is_docked_or_charging ? "true" : "false") << ","
         << "\"final_auto_undock_required\":"
         << (check.final_auto_undock_required ? "true" : "false") << ","
         << "\"clear_stale_safety_interlock_required\":"
         << (check.clear_stale_safety_interlock_required ? "true" : "false") << ","
         << "\"pre_navigation_recovery_required\":"
         << (check.pre_navigation_recovery_required ? "true" : "false") << ","
         << "\"pre_navigation_recovery_action\":"
         << json_string(check.pre_navigation_recovery_action) << ","
         << "\"pre_navigation_block_reason\":"
         << json_string(check.pre_navigation_block_reason) << ","
         << "\"resolved_dock_id\":" << json_string(check.resolved_dock_id) << ","
         << "\"can_auto_undock\":" << (check.can_auto_undock ? "true" : "false") << ","
         << "\"docking_active_not_docked_block\":"
         << (check.docking_active_not_docked_block ? "true" : "false") << ","
         << "\"auto_undock_reason\":" << json_string(check.auto_undock_reason) << ","
         << "\"final_auto_undock_reason\":" << json_string(check.auto_undock_reason)
         << "}";
    return body.str();
  }

  DockContactInterlockConfig config_;
  DockContactInterlockPorts ports_;
  bool have_last_write_{false};
  bool last_docked_{false};
  std::string last_source_;
  std::string last_reason_;
  std::string last_dock_id_;
  std::string last_note_;
};

DockContactInterlockModule::DockContactInterlockModule(
  DockContactInterlockConfig config,
  DockContactInterlockPorts ports)
: impl_(std::make_unique<Impl>(std::move(config), std::move(ports)))
{
}

DockContactInterlockModule::~DockContactInterlockModule() = default;

DockContactLatchSnapshot DockContactInterlockModule::read_latch() const
{
  return impl_->read_latch();
}

void DockContactInterlockModule::update_latch(
  const bool docked,
  const std::string & source,
  const std::string & reason,
  const std::string & dock_id,
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & map_id,
  const std::string & note)
{
  impl_->update_latch(
    docked, source, reason, dock_id, building_id, floor_id, map_id, note);
}

void DockContactInterlockModule::on_bms_contact_evidence(
  const BatteryContactEvaluation & charging_contact,
  const bool contact_stable,
  const double stable_duration_sec)
{
  impl_->on_bms_contact_evidence(charging_contact, contact_stable, stable_duration_sec);
}

PreNavigationDockCheck DockContactInterlockModule::snapshot()
{
  return impl_->snapshot();
}

std::string DockContactInterlockModule::bms_snapshot_json(
  const BmsChargingContactSnapshot & bms) const
{
  return impl_->bms_snapshot_json(bms);
}

std::string DockContactInterlockModule::latch_snapshot_json(
  const DockContactLatchSnapshot & latch) const
{
  return impl_->latch_snapshot_json(latch);
}

std::string DockContactInterlockModule::pre_navigation_check_json(
  const PreNavigationDockCheck & check,
  const PreNavigationDockCheckContext & context) const
{
  return impl_->pre_navigation_check_json(check, context);
}

}  // namespace robot_api_server::features::docking

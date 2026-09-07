#include "robot_global_localization/ros_component_manager_port.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "composition_interfaces/srv/list_nodes.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"

namespace robot_global_localization
{

namespace
{

struct ComponentListing
{
  bool success{false};
  std::string failure_code;
  std::string detail;
  std::vector<std::pair<std::string, std::uint64_t>> exact_matches;
};

constexpr auto kComponentStatePollInterval = std::chrono::milliseconds(50);
constexpr auto kComponentListAttemptTimeout = std::chrono::milliseconds(200);

bool valid_absolute_ros_name(const std::string & value)
{
  return value.size() > 1U && value.front() == '/' && value.back() != '/' &&
         value.find("//") == std::string::npos;
}

std::string container_service(
  const std::string & container_name,
  const std::string & service)
{
  return container_name + "/_container/" + service;
}

std::string node_service(
  const std::string & node_name,
  const std::string & service)
{
  return node_name + "/" + service;
}

LocalizerParameter from_ros_parameter(
  const std::string & name,
  const rcl_interfaces::msg::ParameterValue & value)
{
  LocalizerParameter parameter;
  parameter.name = name;
  using ParameterType = rcl_interfaces::msg::ParameterType;
  switch (value.type) {
    case ParameterType::PARAMETER_BOOL:
      parameter.type = LocalizerParameterType::kBool;
      parameter.bool_value = value.bool_value;
      break;
    case ParameterType::PARAMETER_INTEGER:
      parameter.type = LocalizerParameterType::kInteger;
      parameter.integer_value = value.integer_value;
      break;
    case ParameterType::PARAMETER_DOUBLE:
      parameter.type = LocalizerParameterType::kDouble;
      parameter.double_value = value.double_value;
      break;
    case ParameterType::PARAMETER_STRING:
      parameter.type = LocalizerParameterType::kString;
      parameter.string_value = value.string_value;
      break;
    case ParameterType::PARAMETER_BYTE_ARRAY:
      parameter.type = LocalizerParameterType::kByteArray;
      parameter.byte_array_value = value.byte_array_value;
      break;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      parameter.type = LocalizerParameterType::kBoolArray;
      parameter.bool_array_value = value.bool_array_value;
      break;
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      parameter.type = LocalizerParameterType::kIntegerArray;
      parameter.integer_array_value = value.integer_array_value;
      break;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      parameter.type = LocalizerParameterType::kDoubleArray;
      parameter.double_array_value = value.double_array_value;
      break;
    case ParameterType::PARAMETER_STRING_ARRAY:
      parameter.type = LocalizerParameterType::kStringArray;
      parameter.string_array_value = value.string_array_value;
      break;
    default:
      parameter.type = LocalizerParameterType::kNotSet;
      break;
  }
  return parameter;
}

rcl_interfaces::msg::Parameter to_ros_parameter(const LocalizerParameter & parameter)
{
  rcl_interfaces::msg::Parameter converted;
  converted.name = parameter.name;
  using ParameterType = rcl_interfaces::msg::ParameterType;
  switch (parameter.type) {
    case LocalizerParameterType::kBool:
      converted.value.type = ParameterType::PARAMETER_BOOL;
      converted.value.bool_value = parameter.bool_value;
      break;
    case LocalizerParameterType::kInteger:
      converted.value.type = ParameterType::PARAMETER_INTEGER;
      converted.value.integer_value = parameter.integer_value;
      break;
    case LocalizerParameterType::kDouble:
      converted.value.type = ParameterType::PARAMETER_DOUBLE;
      converted.value.double_value = parameter.double_value;
      break;
    case LocalizerParameterType::kString:
      converted.value.type = ParameterType::PARAMETER_STRING;
      converted.value.string_value = parameter.string_value;
      break;
    case LocalizerParameterType::kByteArray:
      converted.value.type = ParameterType::PARAMETER_BYTE_ARRAY;
      converted.value.byte_array_value = parameter.byte_array_value;
      break;
    case LocalizerParameterType::kBoolArray:
      converted.value.type = ParameterType::PARAMETER_BOOL_ARRAY;
      converted.value.bool_array_value = parameter.bool_array_value;
      break;
    case LocalizerParameterType::kIntegerArray:
      converted.value.type = ParameterType::PARAMETER_INTEGER_ARRAY;
      converted.value.integer_array_value = parameter.integer_array_value;
      break;
    case LocalizerParameterType::kDoubleArray:
      converted.value.type = ParameterType::PARAMETER_DOUBLE_ARRAY;
      converted.value.double_array_value = parameter.double_array_value;
      break;
    case LocalizerParameterType::kStringArray:
      converted.value.type = ParameterType::PARAMETER_STRING_ARRAY;
      converted.value.string_array_value = parameter.string_array_value;
      break;
    case LocalizerParameterType::kNotSet:
      converted.value.type = ParameterType::PARAMETER_NOT_SET;
      break;
  }
  return converted;
}

}  // namespace

class RosComponentManagerPort::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    RosComponentManagerOptions options)
  : options_(std::move(options))
  {
    list_nodes_client_ = node.create_client<composition_interfaces::srv::ListNodes>(
      container_service(options_.container_name, "list_nodes"),
      rmw_qos_profile_services_default, callback_group);
    load_node_client_ = node.create_client<composition_interfaces::srv::LoadNode>(
      container_service(options_.container_name, "load_node"),
      rmw_qos_profile_services_default, callback_group);
    unload_node_client_ = node.create_client<composition_interfaces::srv::UnloadNode>(
      container_service(options_.container_name, "unload_node"),
      rmw_qos_profile_services_default, callback_group);
    list_parameters_client_ = node.create_client<rcl_interfaces::srv::ListParameters>(
      node_service(options_.expected_full_node_name, "list_parameters"),
      rmw_qos_profile_services_default, callback_group);
    get_parameters_client_ = node.create_client<rcl_interfaces::srv::GetParameters>(
      node_service(options_.expected_full_node_name, "get_parameters"),
      rmw_qos_profile_services_default, callback_group);
  }

  ComponentOperationResult preflight()
  {
    if (!valid_absolute_ros_name(options_.container_name) ||
      !valid_absolute_ros_name(options_.expected_full_node_name) ||
      options_.operation_timeout <= std::chrono::nanoseconds::zero())
    {
      return {
        false, "COMPONENT_CONFIG_INVALID",
        "container/localizer names or component timeout are invalid", 0U,
        ComponentPresence::kUnknown};
    }
    if (!list_nodes_client_->wait_for_service(options_.operation_timeout) ||
      !load_node_client_->wait_for_service(options_.operation_timeout) ||
      !unload_node_client_->wait_for_service(options_.operation_timeout) ||
      !list_parameters_client_->wait_for_service(options_.operation_timeout) ||
      !get_parameters_client_->wait_for_service(options_.operation_timeout))
    {
      return {
        false, "COMPONENT_MANAGER_CAPABILITY_MISSING",
        "required composition or parameter service is unavailable", 0U,
        ComponentPresence::kUnknown};
    }
    const ComponentListing listing = list_exact_components();
    if (!listing.success) {
      return {
        false, listing.failure_code, listing.detail, 0U,
        ComponentPresence::kUnknown};
    }
    if (listing.exact_matches.size() != 1U) {
      return {
        false, "LOCALIZER_COMPONENT_AMBIGUOUS",
        "preflight requires exactly one component named " +
        options_.expected_full_node_name, 0U,
        listing.exact_matches.empty() ?
        ComponentPresence::kAbsent : ComponentPresence::kAmbiguous};
    }
    return {
      true, "", "composition replacement capabilities verified",
      listing.exact_matches.front().second, ComponentPresence::kPresent};
  }

  ComponentCaptureResult capture()
  {
    const ComponentListing listing = list_exact_components();
    if (!listing.success) {
      return {false, listing.failure_code, listing.detail, {}};
    }
    if (listing.exact_matches.size() != 1U) {
      return {
        false, "LOCALIZER_COMPONENT_AMBIGUOUS",
        "capture requires exactly one matching localizer component", {}};
    }

    if (!list_parameters_client_->service_is_ready() ||
      !get_parameters_client_->service_is_ready())
    {
      return {
        false, "LOCALIZER_PARAMETER_SERVICE_UNAVAILABLE",
        "localizer parameter services became unavailable", {}};
    }

    auto list_request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    list_request->depth = std::numeric_limits<std::uint64_t>::max();
    auto list_future = list_parameters_client_->async_send_request(list_request);
    if (list_future.wait_for(options_.operation_timeout) != std::future_status::ready) {
      return {
        false, "LOCALIZER_PARAMETER_LIST_TIMEOUT",
        "list_parameters exceeded the configured timeout", {}};
    }

    rcl_interfaces::srv::ListParameters::Response::SharedPtr list_response;
    try {
      list_response = list_future.get();
    } catch (const std::exception & exception) {
      return {
        false, "LOCALIZER_PARAMETER_LIST_FAILED",
        std::string("list_parameters failed: ") + exception.what(), {}};
    }
    if (!list_response || list_response->result.names.empty()) {
      return {
        false, "LOCALIZER_PARAMETER_LIST_EMPTY",
        "localizer returned no declared parameters", {}};
    }

    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = list_response->result.names;
    auto get_future = get_parameters_client_->async_send_request(get_request);
    if (get_future.wait_for(options_.operation_timeout) != std::future_status::ready) {
      return {
        false, "LOCALIZER_PARAMETER_GET_TIMEOUT",
        "get_parameters exceeded the configured timeout", {}};
    }

    rcl_interfaces::srv::GetParameters::Response::SharedPtr get_response;
    try {
      get_response = get_future.get();
    } catch (const std::exception & exception) {
      return {
        false, "LOCALIZER_PARAMETER_GET_FAILED",
        std::string("get_parameters failed: ") + exception.what(), {}};
    }
    if (!get_response ||
      get_response->values.size() != get_request->names.size())
    {
      return {
        false, "LOCALIZER_PARAMETER_CAPTURE_INCOMPLETE",
        "get_parameters did not return one value for every declared parameter", {}};
    }

    ComponentSnapshot snapshot;
    snapshot.full_node_name = listing.exact_matches.front().first;
    snapshot.unique_id = listing.exact_matches.front().second;
    snapshot.parameters.reserve(get_request->names.size());
    for (std::size_t index = 0U; index < get_request->names.size(); ++index) {
      LocalizerParameter parameter =
        from_ros_parameter(get_request->names[index], get_response->values[index]);
      if (parameter.type == LocalizerParameterType::kNotSet) {
        return {
          false, "LOCALIZER_PARAMETER_CAPTURE_INCOMPLETE",
          "declared parameter is unset or has an unsupported type: " +
          parameter.name, {}};
      }
      snapshot.parameters.push_back(std::move(parameter));
    }
    return {true, "", "captured all declared localizer parameters", std::move(snapshot)};
  }

  ComponentOperationResult unload(const std::uint64_t unique_id)
  {
    if (unique_id == 0U) {
      return {
        false, "LOCALIZER_COMPONENT_ID_INVALID",
        "refusing to unload component id zero", 0U, ComponentPresence::kUnknown};
    }
    if (!unload_node_client_->service_is_ready()) {
      return confirm_absence_after_unload_interruption(
        unique_id, "COMPONENT_UNLOAD_SERVICE_UNAVAILABLE",
        "unload_node service is unavailable");
    }
    auto request = std::make_shared<composition_interfaces::srv::UnloadNode::Request>();
    request->unique_id = unique_id;
    auto future = unload_node_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() + options_.operation_timeout;
    bool response_observed = false;
    bool response_success = false;
    std::string response_detail;
    ComponentPresence last_presence = ComponentPresence::kUnknown;
    std::string last_listing_detail = "component graph has not returned a valid listing";
    const auto observe_response = [&]() {
        if (response_observed ||
          future.wait_for(std::chrono::nanoseconds::zero()) != std::future_status::ready)
        {
          return;
        }
        response_observed = true;
        try {
          const auto response = future.get();
          response_success = response && response->success;
          response_detail = response ? response->error_message :
            "unload_node returned no response";
        } catch (const std::exception & exception) {
          response_detail = std::string("unload_node failed: ") + exception.what();
        }
      };

    while (std::chrono::steady_clock::now() < deadline) {
      observe_response();

      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const auto remaining =
        std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
      const ComponentListing listing = list_exact_components(
        std::min<std::chrono::nanoseconds>(remaining, kComponentListAttemptTimeout));
      if (listing.success) {
        last_presence = presence_from_listing(listing, unique_id);
        const bool manager_ready = component_manager_services_ready();
        last_listing_detail = last_presence == ComponentPresence::kPresent ?
          "last valid listing still contains the old component" :
          last_presence == ComponentPresence::kAmbiguous ?
          "last valid listing contains an ambiguous component identity" :
          manager_ready ?
          "last valid listing proves the old component absent" :
          "old component is absent but manager services are not all live";
        if (last_presence == ComponentPresence::kAbsent &&
          manager_ready)
        {
          if (response_observed && !response_success) {
            return {
              false, "LOCALIZER_UNLOAD_FAILED",
              (response_detail.empty() ? "unload_node rejected the request" : response_detail) +
              "; component absence was confirmed",
              0U, ComponentPresence::kAbsent};
          }
          if (response_observed) {
            return {
              true, "", "localizer component unloaded and absence confirmed",
              0U, ComponentPresence::kAbsent};
          }
        }
        if (response_observed && !response_success &&
          last_presence == ComponentPresence::kPresent)
        {
          return {
            false, "LOCALIZER_UNLOAD_FAILED",
            response_detail.empty() ? "unload_node rejected the request" : response_detail,
            0U, last_presence};
        }
      } else {
        last_presence = ComponentPresence::kUnknown;
        last_listing_detail = listing.failure_code + ": " + listing.detail;
      }

      const auto after_listing = std::chrono::steady_clock::now();
      if (after_listing < deadline) {
        std::this_thread::sleep_for(std::min(
          kComponentStatePollInterval,
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - after_listing)));
      }
    }

    observe_response();
    if (response_observed && response_success &&
      last_presence == ComponentPresence::kAbsent && component_manager_services_ready())
    {
      return {
        true, "", "localizer component unloaded and absence confirmed",
        0U, ComponentPresence::kAbsent};
    }
    if (response_observed && !response_success &&
      last_presence == ComponentPresence::kAbsent)
    {
      return {
        false, "LOCALIZER_UNLOAD_FAILED",
        (response_detail.empty() ? "unload_node rejected the request" : response_detail) +
        "; component absence was confirmed",
        0U, ComponentPresence::kAbsent};
    }
    if (!response_observed) {
      (void)unload_node_client_->remove_pending_request(future);
      return {
        false, "LOCALIZER_UNLOAD_TIMEOUT",
        (last_presence == ComponentPresence::kAbsent ?
        "component absence was confirmed after the unload response was lost; "
        "the requested target switch remains unauthorized" :
        "unload_node outcome remained unproven before timeout; " + last_listing_detail),
        0U, last_presence};
    }
    if (!response_success) {
      return {
        false, "LOCALIZER_UNLOAD_FAILED",
        response_detail.empty() ? "unload_node rejected the request" : response_detail,
        0U, last_presence};
    }
    return {
      false, "LOCALIZER_UNLOAD_NOT_CONFIRMED",
      "component remains present or graph state is ambiguous after unload; " +
      last_listing_detail,
      0U, last_presence};
  }

  ComponentOperationResult load(const ComponentLoadRequest & load_request)
  {
    if (load_request.description.package_name.empty() ||
      load_request.description.plugin_name.empty() ||
      load_request.description.node_name.empty())
    {
      return {
        false, "LOCALIZER_COMPONENT_DESCRIPTION_INVALID",
        "package, plugin, and node name are required", 0U,
        ComponentPresence::kAbsent};
    }
    if (!load_node_client_->service_is_ready()) {
      return {
        false, "COMPONENT_LOAD_SERVICE_UNAVAILABLE",
        "load_node service is unavailable", 0U, current_presence(0U)};
    }

    auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
    request->package_name = load_request.description.package_name;
    request->plugin_name = load_request.description.plugin_name;
    request->node_name = load_request.description.node_name;
    request->node_namespace = load_request.description.node_namespace;
    request->log_level = load_request.description.log_level;
    request->remap_rules = load_request.description.remap_rules;
    request->parameters.reserve(load_request.parameters.size());
    for (const auto & parameter : load_request.parameters) {
      if (parameter.name.empty() || parameter.type == LocalizerParameterType::kNotSet) {
        return {
          false, "LOCALIZER_LOAD_PARAMETER_INVALID",
          "refusing to load an unnamed or unset parameter", 0U,
          ComponentPresence::kAbsent};
      }
      request->parameters.push_back(to_ros_parameter(parameter));
    }

    auto future = load_node_client_->async_send_request(request);
    if (future.wait_for(options_.operation_timeout) != std::future_status::ready) {
      if (future.wait_for(options_.operation_timeout) != std::future_status::ready) {
        return {
          false, "LOCALIZER_LOAD_TIMEOUT",
          "load_node exceeded the configured timeout and remains pending; "
          "target absence cannot be proven safely",
          0U, ComponentPresence::kUnknown};
      }

      composition_interfaces::srv::LoadNode::Response::SharedPtr late_response;
      try {
        late_response = future.get();
      } catch (const std::exception & exception) {
        return cleanup_after_failed_load(
          "LOCALIZER_LOAD_TIMEOUT",
          std::string("late load_node response failed: ") + exception.what());
      }
      if (!late_response || !late_response->success) {
        return cleanup_after_failed_load(
          "LOCALIZER_LOAD_TIMEOUT",
          late_response ?
          "load_node returned failure after the configured timeout: " +
          late_response->error_message :
          "load_node returned no response after the configured timeout");
      }
      if (late_response->unique_id == 0U) {
        return cleanup_after_failed_load(
          "LOCALIZER_LOAD_TIMEOUT",
          "load_node returned late success without a usable component id");
      }
      const ComponentOperationResult cleanup = unload(late_response->unique_id);
      return {
        false, "LOCALIZER_LOAD_TIMEOUT",
        cleanup.success ?
        "load_node returned late success; late component was removed" :
        "load_node returned late success; late component cleanup failed: " +
        cleanup.message,
        late_response->unique_id, cleanup.resulting_presence};
    }

    composition_interfaces::srv::LoadNode::Response::SharedPtr response;
    try {
      response = future.get();
    } catch (const std::exception & exception) {
      return cleanup_after_failed_load(
        "LOCALIZER_LOAD_FAILED",
        std::string("load_node failed: ") + exception.what());
    }
    if (!response || !response->success) {
      return cleanup_after_failed_load(
        "LOCALIZER_LOAD_FAILED",
        response ? response->error_message : "load_node returned no response");
    }
    if (response->full_node_name != options_.expected_full_node_name ||
      response->unique_id == 0U)
    {
      if (response->unique_id != 0U) {
        (void)unload(response->unique_id);
      }
      return {
        false, "LOCALIZER_LOAD_IDENTITY_MISMATCH",
        "load_node response did not identify the exact configured localizer",
        response->unique_id, current_presence(response->unique_id)};
    }

    const ComponentListing listing = list_exact_components();
    if (!listing.success || listing.exact_matches.size() != 1U ||
      listing.exact_matches.front().second != response->unique_id)
    {
      (void)unload(response->unique_id);
      return {
        false, "LOCALIZER_LOAD_NOT_CONFIRMED",
        "component graph did not confirm the exact loaded localizer",
        response->unique_id, current_presence(response->unique_id)};
    }
    return {
      true, "", "localizer component loaded and identity confirmed",
      response->unique_id, ComponentPresence::kPresent};
  }

private:
  ComponentListing list_exact_components()
  {
    return list_exact_components(options_.operation_timeout);
  }

  ComponentListing list_exact_components(const std::chrono::nanoseconds timeout)
  {
    if (!list_nodes_client_->service_is_ready()) {
      return {
        false, "COMPONENT_LIST_SERVICE_UNAVAILABLE",
        "list_nodes service is unavailable", {}};
    }
    auto request = std::make_shared<composition_interfaces::srv::ListNodes::Request>();
    auto future = list_nodes_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      (void)list_nodes_client_->remove_pending_request(future);
      return {
        false, "COMPONENT_LIST_TIMEOUT",
        "list_nodes exceeded the configured timeout", {}};
    }

    composition_interfaces::srv::ListNodes::Response::SharedPtr response;
    try {
      response = future.get();
    } catch (const std::exception & exception) {
      return {
        false, "COMPONENT_LIST_FAILED",
        std::string("list_nodes failed: ") + exception.what(), {}};
    }
    if (!response ||
      response->full_node_names.size() != response->unique_ids.size())
    {
      return {
        false, "COMPONENT_LIST_INVALID",
        "list_nodes returned mismatched names and ids", {}};
    }

    ComponentListing result;
    result.success = true;
    for (std::size_t index = 0U;
      index < response->full_node_names.size(); ++index)
    {
      if (response->full_node_names[index] == options_.expected_full_node_name) {
        result.exact_matches.emplace_back(
          response->full_node_names[index], response->unique_ids[index]);
      }
    }
    return result;
  }

  ComponentPresence current_presence(const std::uint64_t expected_unique_id)
  {
    const ComponentListing listing = list_exact_components();
    if (!listing.success) {
      return ComponentPresence::kUnknown;
    }
    return presence_from_listing(listing, expected_unique_id);
  }

  ComponentPresence presence_from_listing(
    const ComponentListing & listing,
    const std::uint64_t expected_unique_id) const
  {
    if (listing.exact_matches.empty()) {
      return ComponentPresence::kAbsent;
    }
    if (listing.exact_matches.size() != 1U) {
      return ComponentPresence::kAmbiguous;
    }
    if (expected_unique_id != 0U &&
      listing.exact_matches.front().second != expected_unique_id)
    {
      return ComponentPresence::kAmbiguous;
    }
    return ComponentPresence::kPresent;
  }

  bool component_manager_services_ready() const
  {
    return list_nodes_client_->service_is_ready() &&
           load_node_client_->service_is_ready() &&
           unload_node_client_->service_is_ready();
  }

  ComponentOperationResult confirm_absence_after_unload_interruption(
    const std::uint64_t unique_id,
    const std::string & failure_code,
    const std::string & failure_detail)
  {
    const auto deadline = std::chrono::steady_clock::now() + options_.operation_timeout;
    ComponentPresence last_presence = ComponentPresence::kUnknown;
    std::string last_listing_detail = "component graph has not returned a valid listing";
    while (std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      const auto remaining =
        std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
      const ComponentListing listing = list_exact_components(
        std::min<std::chrono::nanoseconds>(remaining, kComponentListAttemptTimeout));
      if (listing.success) {
        last_presence = presence_from_listing(listing, unique_id);
        const bool manager_ready = component_manager_services_ready();
        last_listing_detail = last_presence == ComponentPresence::kAbsent ?
          (manager_ready ?
          "last valid listing proves the old component absent" :
          "old component is absent but manager services are not all live") :
          "last valid listing does not prove the old component absent";
        if (last_presence == ComponentPresence::kAbsent &&
          manager_ready)
        {
          return {
            false, failure_code,
            failure_detail + "; component absence was confirmed after manager recovery",
            0U, ComponentPresence::kAbsent};
        }
        if (last_presence == ComponentPresence::kPresent) {
          return {false, failure_code, failure_detail, 0U, last_presence};
        }
      } else {
        last_presence = ComponentPresence::kUnknown;
        last_listing_detail = listing.failure_code + ": " + listing.detail;
      }
      const auto after_listing = std::chrono::steady_clock::now();
      if (after_listing < deadline) {
        std::this_thread::sleep_for(std::min(
          kComponentStatePollInterval,
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - after_listing)));
      }
    }
    return {
      false, failure_code, failure_detail + "; " + last_listing_detail,
      0U, last_presence};
  }

  ComponentOperationResult cleanup_after_failed_load(
    const std::string & failure_code,
    const std::string & detail)
  {
    const ComponentListing listing = list_exact_components();
    if (!listing.success) {
      return {
        false, failure_code,
        detail + "; cleanup could not inspect component graph", 0U,
        ComponentPresence::kUnknown};
    }
    if (listing.exact_matches.empty()) {
      return {
        false, failure_code, detail + "; no target component remains", 0U,
        ComponentPresence::kAbsent};
    }
    if (listing.exact_matches.size() != 1U) {
      return {
        false, failure_code,
        detail + "; cleanup refused ambiguous matching components", 0U,
        ComponentPresence::kAmbiguous};
    }
    const std::uint64_t unique_id = listing.exact_matches.front().second;
    const ComponentOperationResult cleanup = unload(unique_id);
    if (!cleanup.success) {
      return {
        false, failure_code,
        detail + "; cleanup failed: " + cleanup.message, unique_id,
        cleanup.resulting_presence};
    }
    return {
      false, failure_code, detail + "; late/partial target component removed",
      unique_id, ComponentPresence::kAbsent};
  }

  RosComponentManagerOptions options_;
  rclcpp::Client<composition_interfaces::srv::ListNodes>::SharedPtr list_nodes_client_;
  rclcpp::Client<composition_interfaces::srv::LoadNode>::SharedPtr load_node_client_;
  rclcpp::Client<composition_interfaces::srv::UnloadNode>::SharedPtr unload_node_client_;
  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
};

RosComponentManagerPort::RosComponentManagerPort(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  RosComponentManagerOptions options)
: impl_(std::make_unique<Impl>(
      node, std::move(callback_group), std::move(options)))
{
}

RosComponentManagerPort::~RosComponentManagerPort() = default;

ComponentOperationResult RosComponentManagerPort::preflight()
{
  return impl_->preflight();
}

ComponentCaptureResult RosComponentManagerPort::capture()
{
  return impl_->capture();
}

ComponentOperationResult RosComponentManagerPort::unload(const std::uint64_t unique_id)
{
  return impl_->unload(unique_id);
}

ComponentOperationResult RosComponentManagerPort::load(
  const ComponentLoadRequest & request)
{
  return impl_->load(request);
}

}  // namespace robot_global_localization

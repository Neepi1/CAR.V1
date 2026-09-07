#include "robot_floor_manager/floor_transition_reconciliation.hpp"

#include <cctype>

namespace robot_floor_manager
{

namespace
{

std::string extract_failure_code(const std::string & error)
{
  constexpr char marker[] = "failure_code=";
  const auto marker_index = error.find(marker);
  if (marker_index == std::string::npos) {
    return {};
  }

  const auto begin = marker_index + sizeof(marker) - 1U;
  auto end = begin;
  while (end < error.size()) {
    const auto character = static_cast<unsigned char>(error[end]);
    if (!(std::isupper(character) || std::isdigit(character) || character == '_')) {
      break;
    }
    ++end;
  }
  return error.substr(begin, end - begin);
}

bool contains(const std::string & value, const std::string & token)
{
  return value.find(token) != std::string::npos;
}

}  // namespace

ExplicitLocalizationFailureClassification classify_explicit_localization_failure(
  const std::string & trigger_error)
{
  ExplicitLocalizationFailureClassification result;
  result.failure_code = extract_failure_code(trigger_error);

  const bool explicitly_not_dispatched =
    contains(trigger_error, "dispatch_state=not_dispatched");
  if (
    explicitly_not_dispatched &&
    (result.failure_code == "LOCALIZER_POST_RELOAD_NOT_READY" ||
      result.failure_code == "LOCALIZER_OPERATION_BUSY" ||
      result.failure_code == "LOCALIZER_INPUT_NOT_FRESH" ||
      result.failure_code == "ISAAC_SERVICE_UNAVAILABLE" ||
      result.failure_code == "BRIDGE_FORCE_ACCEPT_UNAVAILABLE" ||
      result.failure_code == "BRIDGE_FORCE_ACCEPT_TIMEOUT"))
  {
    result.retryable = true;
  }
  return result;
}

ExplicitLocalizationReconciliation reconcile_explicit_localization(
  const bool trigger_call_succeeded,
  const bool exact_target_evidence_proven,
  const bool canceled,
  const std::string & trigger_error)
{
  ExplicitLocalizationReconciliation result;
  if (exact_target_evidence_proven) {
    result.success = true;
    result.code = trigger_call_succeeded ? "OK" : "OK_RECONCILED";
    result.detail = trigger_call_succeeded ?
      "target explicit localization sequence accepted and proven by exact target evidence" :
      "global localization response was delayed, lost, or rejected, but exact target evidence "
      "proved the new explicit localization sequence";
    return result;
  }

  if (canceled) {
    result.code = "CANCELLED";
    result.detail = "cancelled before exact target localization evidence was proven";
    return result;
  }

  if (!trigger_call_succeeded) {
    result.code = "EXPLICIT_LOCALIZATION_REJECTED";
    result.detail = trigger_error.empty() ?
      "global localization trigger did not succeed and no exact target evidence was observed" :
      trigger_error + "; no exact target evidence was observed";
    return result;
  }

  result.code = "EXPLICIT_LOCALIZATION_UNPROVEN";
  result.detail = "target explicit localization sequence was not proven";
  return result;
}

}  // namespace robot_floor_manager

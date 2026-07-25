#include "robot_floor_manager/floor_switch_preflight.hpp"

#include <algorithm>
#include <cstddef>
#include <cctype>

namespace robot_floor_manager
{
namespace
{

bool safe_identifier(const std::string & value)
{
  if (
    value.empty() || value == "." || value.size() > 128U ||
    value.find("..") != std::string::npos ||
    value.find('/') != std::string::npos ||
    value.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(
    value.begin(), value.end(),
    [](const unsigned char character) {
      return std::isalnum(character) != 0 ||
             character == static_cast<unsigned char>('-') ||
             character == static_cast<unsigned char>('_') ||
             character == static_cast<unsigned char>('.');
    });
}

bool canonical_sha256(const std::string & value)
{
  constexpr char prefix[] = "sha256:";
  constexpr std::size_t prefix_size = sizeof(prefix) - 1U;
  if (value.size() != prefix_size + 64U || value.compare(0U, prefix_size, prefix) != 0) {
    return false;
  }
  return std::all_of(
    value.begin() + static_cast<std::ptrdiff_t>(prefix_size), value.end(),
    [](const unsigned char character) {
      return std::isdigit(character) != 0 ||
             (character >= static_cast<unsigned char>('a') &&
             character <= static_cast<unsigned char>('f'));
    });
}

FloorSwitchPreflightDecision failed(
  const FloorSwitchFailureCode code,
  const std::string & detail)
{
  FloorSwitchPreflightDecision decision;
  decision.failure_code = code;
  decision.code = to_string(code);
  decision.detail = detail;
  return decision;
}

}  // namespace

FloorSwitchPreflightDecision evaluate_floor_switch_preflight(
  const FloorSwitchPreflightInput & input)
{
  const auto & request = input.request;
  if (
    !safe_identifier(request.transaction_id) ||
    !safe_identifier(request.building_id) ||
    !safe_identifier(request.floor_id) ||
    !safe_identifier(request.map_id) ||
    request.expected_asset_epoch == 0U ||
    !canonical_sha256(request.expected_asset_digest))
  {
    return failed(
      FloorSwitchFailureCode::kInvalidGoal,
      "transaction, building, floor, map, nonzero asset epoch, and canonical sha256 "
      "digest are required");
  }
  if (!input.live_floor_switch_enabled) {
    return failed(
      FloorSwitchFailureCode::kLiveSwitchDisabled,
      "production floor switching is disabled until every live safety barrier is proven");
  }
  if (!input.exact_asset_identity_proven) {
    return failed(
      FloorSwitchFailureCode::kAssetBundleInvalid,
      "exact map_id, asset_epoch, and aggregate asset digest are not proven");
  }
  if (!input.localizer_reload_proven) {
    return failed(
      FloorSwitchFailureCode::kLocalizerReloadUnproven,
      "Isaac localizer hot reload and generation evidence are not proven");
  }

  FloorSwitchPreflightDecision decision;
  decision.ready = true;
  decision.mutation_allowed = false;
  decision.failure_code = FloorSwitchFailureCode::kNone;
  decision.code = to_string(decision.failure_code);
  decision.detail =
    "preflight evidence is complete, but this preflight-only adapter has no mutation port";
  return decision;
}

const char * to_string(const FloorSwitchFailureCode code) noexcept
{
  switch (code) {
    case FloorSwitchFailureCode::kNone: return "NONE";
    case FloorSwitchFailureCode::kInvalidGoal: return "INVALID_GOAL";
    case FloorSwitchFailureCode::kTransactionConflict: return "TRANSACTION_CONFLICT";
    case FloorSwitchFailureCode::kAssetNotFound: return "ASSET_NOT_FOUND";
    case FloorSwitchFailureCode::kAssetIdentityMismatch: return "ASSET_IDENTITY_MISMATCH";
    case FloorSwitchFailureCode::kAssetBundleInvalid: return "ASSET_BUNDLE_INVALID";
    case FloorSwitchFailureCode::kAssetDigestMismatch: return "ASSET_DIGEST_MISMATCH";
    case FloorSwitchFailureCode::kMotionHoldUnproven: return "MOTION_HOLD_UNPROVEN";
    case FloorSwitchFailureCode::kNavIdleUnproven: return "NAV_IDLE_UNPROVEN";
    case FloorSwitchFailureCode::kStoppedUnproven: return "STOPPED_UNPROVEN";
    case FloorSwitchFailureCode::kEvidenceStale: return "EVIDENCE_STALE";
    case FloorSwitchFailureCode::kRuntimeContextUnproven: return "RUNTIME_CONTEXT_UNPROVEN";
    case FloorSwitchFailureCode::kLocalizerReloadUnproven:
      return "LOCALIZER_RELOAD_UNPROVEN";
    case FloorSwitchFailureCode::kLiveSwitchDisabled:
      return "LIVE_FLOOR_SWITCH_DISABLED";
    case FloorSwitchFailureCode::kCancelledBeforeMutation:
      return "CANCELLED_BEFORE_MUTATION";
    case FloorSwitchFailureCode::kInternalError: return "INTERNAL_ERROR";
  }
  return "UNKNOWN";
}

}  // namespace robot_floor_manager

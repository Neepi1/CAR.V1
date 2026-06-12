#pragma once

#include <cstdint>
#include <string>

namespace ranger_mini3_mode_controller
{

enum class RangerMotionMode : std::uint8_t
{
  DUAL_ACKERMAN = 0,
  PARALLEL = 1,
  SPINNING = 2,
  SIDE_SLIP = 3,
  UNKNOWN = 255
};

inline RangerMotionMode ranger_motion_mode_from_code(const std::uint8_t code)
{
  switch (code) {
    case 0:
      return RangerMotionMode::DUAL_ACKERMAN;
    case 1:
      return RangerMotionMode::PARALLEL;
    case 2:
      return RangerMotionMode::SPINNING;
    case 3:
      return RangerMotionMode::SIDE_SLIP;
    default:
      return RangerMotionMode::UNKNOWN;
  }
}

inline std::uint8_t ranger_motion_mode_code(const RangerMotionMode mode)
{
  return static_cast<std::uint8_t>(mode);
}

inline const char * ranger_motion_mode_name(const RangerMotionMode mode)
{
  switch (mode) {
    case RangerMotionMode::DUAL_ACKERMAN:
      return "MOTION_MODE_DUAL_ACKERMAN";
    case RangerMotionMode::PARALLEL:
      return "MOTION_MODE_PARALLEL";
    case RangerMotionMode::SPINNING:
      return "MOTION_MODE_SPINNING";
    case RangerMotionMode::SIDE_SLIP:
      return "MOTION_MODE_SIDE_SLIP";
    case RangerMotionMode::UNKNOWN:
      return "MOTION_MODE_UNKNOWN";
  }
  return "MOTION_MODE_UNKNOWN";
}

inline const char * ranger_motion_mode_short_name(const RangerMotionMode mode)
{
  switch (mode) {
    case RangerMotionMode::DUAL_ACKERMAN:
      return "dual_ackerman";
    case RangerMotionMode::PARALLEL:
      return "parallel";
    case RangerMotionMode::SPINNING:
      return "spinning";
    case RangerMotionMode::SIDE_SLIP:
      return "side_slip";
    case RangerMotionMode::UNKNOWN:
      return "unknown";
  }
  return "unknown";
}

inline std::string ranger_motion_mode_json(const RangerMotionMode mode, const std::string & legacy)
{
  return std::string("{\"code\":") + std::to_string(ranger_motion_mode_code(mode)) +
         ",\"name\":\"" + ranger_motion_mode_name(mode) + "\",\"short\":\"" +
         ranger_motion_mode_short_name(mode) + "\",\"legacy\":\"" + legacy + "\"}";
}

}  // namespace ranger_mini3_mode_controller

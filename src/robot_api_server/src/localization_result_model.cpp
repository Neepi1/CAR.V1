#include "robot_api_server/localization_result_model.hpp"

#include <iomanip>
#include <sstream>

namespace robot_api_server
{

std::string localization_result_success_detail(const LocalizationResultSnapshot & snapshot)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "localization_result seq=" << snapshot.seq
      << " age=" << snapshot.age_sec
      << " pose=(" << snapshot.x << "," << snapshot.y << "," << snapshot.yaw << ")";
  return out.str();
}

std::string localization_result_wait_failure_detail(
  const std::string & result_topic,
  const double timeout_sec,
  const LocalizationResultSnapshot & snapshot)
{
  std::ostringstream out;
  if (snapshot.available) {
    out << "no new map-frame " << result_topic
        << " within " << timeout_sec << "s; last frame=" << snapshot.frame_id
        << " age=" << snapshot.age_sec;
  } else {
    out << "no " << result_topic << " observed within " << timeout_sec << "s";
  }
  return out.str();
}

std::string localization_result_recent_fallback_detail(
  const std::string & trigger_message,
  const std::string & result_topic,
  const LocalizationResultSnapshot & snapshot)
{
  std::ostringstream out;
  out << trigger_message << "; using recent " << result_topic
      << " age=" << std::fixed << std::setprecision(3) << snapshot.age_sec
      << "s because no newer result arrived after trigger";
  return out.str();
}

}  // namespace robot_api_server

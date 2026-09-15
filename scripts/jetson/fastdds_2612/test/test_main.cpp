// Harness for the unmodified eProsima Fast-DDS 2.6.12 flow-controller tests.
// Matches the upstream BlackboxEnvironment's UDP / no intra-process settings.
// Run only through run_isolated_regression.sh, never in the robot network.
#include "BlackboxTests.hpp"
#include <fastrtps/rtps/RTPSDomain.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastdds/dds/log/Log.hpp>
#include <gtest/gtest.h>

const char* certs_path = nullptr;
uint16_t global_port = 0;
bool enable_datasharing = false;
bool use_pull_mode = false;
bool use_udpv4 = true;
bool use_ipv6 = false;

class FlowEnvironment : public testing::Environment
{
  void SetUp() override
  {
    global_port = static_cast<uint16_t>(GET_PID());
    if (global_port < 5000) { global_port += 5000; }
    eprosima::fastrtps::LibrarySettingsAttributes settings;
    settings.intraprocess_delivery = eprosima::fastrtps::INTRAPROCESS_OFF;
    eprosima::fastrtps::xmlparser::XMLProfileManager::library_settings(settings);
  }
  void TearDown() override
  {
    eprosima::fastdds::dds::Log::KillThread();
    eprosima::fastrtps::rtps::RTPSDomain::stopAll();
  }
};

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new FlowEnvironment);
  return RUN_ALL_TESTS();
}

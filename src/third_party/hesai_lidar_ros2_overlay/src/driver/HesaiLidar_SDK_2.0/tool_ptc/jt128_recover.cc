#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "logger.h"
#include "ptc_client.h"

using namespace hesai::lidar;

namespace {

void PrintUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0
      << " <device_ip> <ptc_port> [destination_ip] [udp_port] [gps_udp_port] [standby_mode] [apply_dest] [apply_standby]\n"
      << "Defaults:\n"
      << "  destination_ip = 192.168.1.100\n"
      << "  udp_port       = 2368\n"
      << "  gps_udp_port   = 10110\n"
      << "  standby_mode   = 0  (0=operation, 1=standby)\n"
      << "  apply_dest     = 1  (1=apply SetDesIpandPort, 0=skip)\n"
      << "  apply_standby  = 1  (1=apply SetStandbyMode, 0=skip)\n";
}

bool WaitForClientReady(PtcClient& client, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (client.IsOpen()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return client.IsOpen();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    PrintUsage(argv[0]);
    return 2;
  }

  const std::string device_ip = argv[1];
  const int ptc_port = std::atoi(argv[2]);
  const std::string destination_ip = argc >= 4 ? argv[3] : "192.168.1.100";
  const uint16_t udp_port = static_cast<uint16_t>(argc >= 5 ? std::atoi(argv[4]) : 2368);
  const uint16_t gps_udp_port = static_cast<uint16_t>(argc >= 6 ? std::atoi(argv[5]) : 10110);
  const uint32_t standby_mode = static_cast<uint32_t>(argc >= 7 ? std::atoi(argv[6]) : 0);
  const bool apply_dest = argc >= 8 ? (std::atoi(argv[7]) != 0) : true;
  const bool apply_standby = argc >= 9 ? (std::atoi(argv[8]) != 0) : true;

  Logger::GetInstance().setLogTargetRule(LOGTARGET::HESAI_LOG_TARGET_CONSOLE);
  Logger::GetInstance().setLogLevelRule(
      LOGLEVEL::HESAI_LOG_INFO | LOGLEVEL::HESAI_LOG_WARNING |
      LOGLEVEL::HESAI_LOG_ERROR | LOGLEVEL::HESAI_LOG_FATAL);

  std::cout << "Connecting to JT128 PTC at " << device_ip << ":" << ptc_port << '\n';
  PtcClient client(device_ip, static_cast<uint16_t>(ptc_port));

  if (!WaitForClientReady(client, 10000)) {
    std::cerr << "PTC client did not become ready within 10s\n";
    return 1;
  }

  std::cout << "PTC connected\n";
  if (apply_dest) {
    std::cout << "Setting destination IP/port to " << destination_ip << ":" << udp_port
              << " (gps=" << gps_udp_port << ")\n";
    if (!client.SetDesIpandPort(destination_ip, udp_port, gps_udp_port)) {
      std::cerr << "SetDesIpandPort failed, ret_code=" << client.ret_code_ << '\n';
      return 1;
    }
  }

  if (apply_standby) {
    std::cout << "Setting standby_mode=" << standby_mode << '\n';
    if (!client.SetStandbyMode(standby_mode)) {
      std::cerr << "SetStandbyMode failed, ret_code=" << client.ret_code_ << '\n';
      return 1;
    }
  }

  std::cout << "JT128 recover command sequence completed successfully\n";
  return 0;
}

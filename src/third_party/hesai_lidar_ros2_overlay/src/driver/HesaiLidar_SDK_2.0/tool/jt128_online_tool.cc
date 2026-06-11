#include "hesai_lidar_sdk.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {

std::atomic<bool> g_running(true);
std::atomic<bool> g_got_frame(false);
std::atomic<uint64_t> g_last_frame_time_us(0);

void SignalHandler(int) {
  g_running = false;
}

void lidarCallback(const LidarDecodedFrame<LidarPointXYZICRT>& frame) {
  const uint64_t now = GetMicroTimeU64();
  g_last_frame_time_us = now;
  g_got_frame = true;
  std::cout << now
            << " -> frame:" << frame.frame_index
            << " points:" << frame.points_num
            << " packet:" << frame.packet_num
            << " start:" << frame.frame_start_timestamp
            << " end:" << frame.frame_end_timestamp
            << std::endl;
}

void faultMessageCallback(const FaultMessageInfo&) {
}

}  // namespace

int main(int argc, char* argv[]) {
#ifndef _MSC_VER
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
#endif

  std::string correction_path =
      argc > 1 ? argv[1]
               : "/workspaces/isaac_ros-dev/ros2_ws/src/hesai_lidar_ros2/src/driver/"
                 "HesaiLidar_SDK_2.0/correction/angle_correction/"
                 "JT128_Angle Correction File.csv";
  std::string firetime_path =
      argc > 2 ? argv[2]
               : "/workspaces/isaac_ros-dev/ros2_ws/src/hesai_lidar_ros2/src/driver/"
                 "HesaiLidar_SDK_2.0/correction/firetime_correction/"
                 "JT128_Firetime Correction File.csv";
  std::string host_ip = argc > 3 ? argv[3] : "192.168.1.100";
  std::string device_ip = argc > 4 ? argv[4] : "192.168.1.201";
  uint16_t udp_port = static_cast<uint16_t>(argc > 5 ? std::atoi(argv[5]) : 2368);
  uint16_t ptc_port = static_cast<uint16_t>(argc > 6 ? std::atoi(argv[6]) : 9347);
  uint16_t device_udp_src_port =
      static_cast<uint16_t>(argc > 7 ? std::atoi(argv[7]) : 10000);

  std::cout << "JT128 online test config:\n"
            << "  correction_path: " << correction_path << "\n"
            << "  firetime_path: " << firetime_path << "\n"
            << "  host_ip: " << host_ip << "\n"
            << "  device_ip: " << device_ip << "\n"
            << "  udp_port: " << udp_port << "\n"
            << "  ptc_port: " << ptc_port << "\n"
            << "  device_udp_src_port: " << device_udp_src_port << std::endl;

  HesaiLidarSdk<LidarPointXYZICRT> sdk;
  DriverParam param;
  param.input_param.source_type = DATA_FROM_LIDAR;
  param.input_param.device_ip_address = device_ip;
  param.input_param.udp_port = udp_port;
  param.input_param.ptc_port = ptc_port;
  param.input_param.multicast_ip_address = "";
  param.input_param.use_ptc_connected = true;
  param.input_param.correction_file_path = correction_path;
  param.input_param.firetimes_path = firetime_path;
  param.input_param.host_ip_address = host_ip;
  param.input_param.fault_message_port = 0;
  param.input_param.device_udp_src_port = device_udp_src_port;
  param.input_param.recv_point_cloud_timeout = -1;
  param.input_param.ptc_connect_timeout = 3;
  param.input_param.standby_mode = -1;
  param.input_param.speed = -1;
  param.input_param.ptc_mode = PtcMode::tcp;
  param.decoder_param.enable_packet_loss_tool = false;
  param.decoder_param.socket_buffer_size = 262144000;

  sdk.Init(param);
  sdk.RegRecvCallback(lidarCallback);
  sdk.RegRecvCallback(faultMessageCallback);
  sdk.Start();

  if (sdk.lidar_ptr_->GetInitFinish(FailInit)) {
    sdk.Stop();
    std::cerr << "SDK init failed" << std::endl;
    return 1;
  }

  const uint64_t begin_us = GetMicroTimeU64();
  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const uint64_t now_us = GetMicroTimeU64();
    if (!g_got_frame && now_us - begin_us > 8ULL * 1000ULL * 1000ULL) {
      std::cerr << "No valid point-cloud frame received within 8 seconds" << std::endl;
      sdk.Stop();
      return 2;
    }

    if (g_got_frame && now_us - g_last_frame_time_us.load() > 3ULL * 1000ULL * 1000ULL) {
      std::cerr << "Frame stream stopped for more than 3 seconds" << std::endl;
      sdk.Stop();
      return 3;
    }
  }

  sdk.Stop();
  return 0;
}

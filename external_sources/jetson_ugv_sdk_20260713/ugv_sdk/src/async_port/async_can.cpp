/*
 * async_can.cpp
 *
 * Created on: Sep 10, 2020 13:23
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/details/async_port/async_can.hpp"

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>

#include <algorithm>
#include <iostream>
#include <iterator>

namespace westonrobot {
namespace {
constexpr canid_t kMotionCommandCanId = 0x111;
constexpr canid_t kMotionModeCommandCanId = 0x141;
constexpr std::size_t kMaxPendingCanFrames = 32;

canid_t NormalizeCanId(const can_frame &frame) {
  return frame.can_id & CAN_EFF_MASK;
}
}  // namespace

AsyncCAN::AsyncCAN(std::string can_port)
    : port_(can_port), socketcan_stream_(io_context_) {}

AsyncCAN::~AsyncCAN() { Close(); }

bool AsyncCAN::Open() {
  try {
    const size_t iface_name_size = strlen(port_.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ) return false;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, port_.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0) {
      Close();
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int bind_result =
        bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr));
    if (bind_result < 0) {
      Close();
      return false;
    }

    port_opened_ = true;
    std::cout << "Start listening to port: " << port_ << std::endl;
  } catch (std::system_error &e) {
    port_opened_ = false;
    std::cout << e.what() << std::endl;
    return false;
  }

  tx_queue_.clear();
  tx_in_progress_ = false;

  // give some work to io_service to start async io chain
  socketcan_stream_.assign(can_fd_);

#if ASIO_VERSION < 101200L
  io_context_.post(std::bind(&AsyncCAN::ReadFromPort, this,
                             std::ref(rcv_frame_),
                             std::ref(socketcan_stream_)));
#else
  asio::post(io_context_,
             std::bind(&AsyncCAN::ReadFromPort, this, std::ref(rcv_frame_),
                       std::ref(socketcan_stream_)));
#endif

  // start io thread
  io_thread_ = std::thread([this]() { io_context_.run(); });

  return true;
}

void AsyncCAN::Close() {
  port_opened_ = false;
  io_context_.stop();
  if (io_thread_.joinable()) io_thread_.join();
  io_context_.reset();

  tx_queue_.clear();
  tx_in_progress_ = false;

  // release port fd
  if (can_fd_ >= 0) {
    ::close(can_fd_);
  }
  can_fd_ = -1;
}

bool AsyncCAN::IsOpened() const { return port_opened_; }

void AsyncCAN::DefaultReceiveCallback(can_frame *rx_frame) {
  std::cout << std::hex << rx_frame->can_id << "  ";
  for (int i = 0; i < rx_frame->can_dlc; i++)
    std::cout << std::hex << int(rx_frame->data[i]) << " ";
  std::cout << std::dec << std::endl;
}

void AsyncCAN::ReadFromPort(struct can_frame &rec_frame,
                            asio::posix::basic_stream_descriptor<> &stream) {
  auto sthis = shared_from_this();
  stream.async_read_some(
      asio::buffer(&rec_frame, sizeof(rec_frame)),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->Close();
          return;
        }

        if (sthis->rcv_cb_ != nullptr)
          sthis->rcv_cb_(&sthis->rcv_frame_);
        else
          sthis->DefaultReceiveCallback(&sthis->rcv_frame_);

        sthis->ReadFromPort(std::ref(sthis->rcv_frame_),
                            std::ref(sthis->socketcan_stream_));
      });
}

void AsyncCAN::SendFrame(const struct can_frame &frame) {
  if (!port_opened_) return;

  auto sthis = shared_from_this();
#if ASIO_VERSION < 101200L
  io_context_.post([sthis, frame]() { sthis->EnqueueFrame(frame); });
#else
  asio::post(io_context_, [sthis, frame]() { sthis->EnqueueFrame(frame); });
#endif
}

void AsyncCAN::EnqueueFrame(struct can_frame frame) {
  if (!port_opened_) return;

  const auto can_id = NormalizeCanId(frame);
  const bool latest_only =
      can_id == kMotionCommandCanId || can_id == kMotionModeCommandCanId;
  if (latest_only) {
    const auto begin = tx_in_progress_ && !tx_queue_.empty()
                           ? std::next(tx_queue_.begin())
                           : tx_queue_.begin();
    const auto existing = std::find_if(
        begin, tx_queue_.end(), [can_id](const can_frame &queued) {
          return NormalizeCanId(queued) == can_id;
        });
    if (existing != tx_queue_.end()) {
      *existing = frame;
      return;
    }
  }

  if (tx_queue_.size() >= kMaxPendingCanFrames) {
    const auto begin = tx_in_progress_ && !tx_queue_.empty()
                           ? std::next(tx_queue_.begin())
                           : tx_queue_.begin();
    const auto stale_motion = std::find_if(
        begin, tx_queue_.end(), [](const can_frame &queued) {
          return NormalizeCanId(queued) == kMotionCommandCanId;
        });
    if (stale_motion != tx_queue_.end()) {
      tx_queue_.erase(stale_motion);
    } else {
      std::cerr << "CAN TX queue full; rejecting frame 0x" << std::hex
                << can_id << std::dec << std::endl;
      return;
    }
  }

  tx_queue_.push_back(frame);
  StartNextWrite();
}

void AsyncCAN::StartNextWrite() {
  if (tx_in_progress_ || tx_queue_.empty() || !port_opened_) return;

  tx_in_progress_ = true;
  auto sthis = shared_from_this();
  asio::async_write(
      socketcan_stream_,
      asio::buffer(&tx_queue_.front(), sizeof(struct can_frame)),
      [sthis](const asio::error_code &error, size_t bytes_transferred) {
        sthis->HandleWrite(error, bytes_transferred);
      });
}

void AsyncCAN::HandleWrite(const asio::error_code &error,
                           size_t bytes_transferred) {
  if (error) {
    std::cerr << "Failed to send CAN frame: " << error.message() << std::endl;
  } else if (bytes_transferred != sizeof(struct can_frame)) {
    std::cerr << "Short CAN frame write: " << bytes_transferred << "/"
              << sizeof(struct can_frame) << std::endl;
  }

  if (!tx_queue_.empty()) tx_queue_.pop_front();
  tx_in_progress_ = false;
  StartNextWrite();
}

}  // namespace westonrobot

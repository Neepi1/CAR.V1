#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::infrastructure::http
{

bool websocket_headers_valid(const HttpRequest & request);
void set_socket_receive_timeout(int client_fd, double timeout_sec);
bool send_websocket_handshake(int client_fd, const HttpRequest & request);
bool send_websocket_frame(
  int client_fd,
  const std::string & payload,
  std::uint8_t opcode = 0x1U);
std::optional<WebSocketFrame> read_websocket_frame(
  int client_fd,
  const std::atomic_bool & running);

}  // namespace robot_api_server::infrastructure::http

#include "robot_api_server/infrastructure/http/websocket_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <sys/time.h>

namespace robot_api_server::infrastructure::http
{
namespace
{

bool send_all_bytes(const int client_fd, const void * data, const std::size_t length)
{
  const char * cursor = static_cast<const char *>(data);
  std::size_t sent = 0;
  while (sent < length) {
    const ssize_t count = ::send(client_fd, cursor + sent, length - sent, MSG_NOSIGNAL);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(count);
  }
  return true;
}

bool recv_exact(
  const int client_fd,
  void * data,
  const std::size_t length,
  const std::atomic_bool & running)
{
  char * cursor = static_cast<char *>(data);
  std::size_t received = 0;
  while (received < length && running.load()) {
    const ssize_t count = ::recv(client_fd, cursor + received, length - received, 0);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      return false;
    }
    received += static_cast<std::size_t>(count);
  }
  return received == length;
}

}  // namespace

bool websocket_headers_valid(const HttpRequest & request)
{
  const auto upgrade_it = request.headers.find("upgrade");
  const auto connection_it = request.headers.find("connection");
  const auto key_it = request.headers.find("sec-websocket-key");
  if (upgrade_it == request.headers.end() || lower_copy(upgrade_it->second) != "websocket") {
    return false;
  }
  if (connection_it == request.headers.end()) {
    return false;
  }
  if (lower_copy(connection_it->second).find("upgrade") == std::string::npos) {
    return false;
  }
  return key_it != request.headers.end() && !key_it->second.empty();
}

void set_socket_receive_timeout(const int client_fd, const double timeout_sec)
{
  timeval timeout{};
  timeout.tv_sec = static_cast<time_t>(timeout_sec);
  timeout.tv_usec = static_cast<suseconds_t>(
    std::max(0.0, timeout_sec - static_cast<double>(timeout.tv_sec)) * 1000000.0);
  ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

bool send_websocket_handshake(const int client_fd, const HttpRequest & request)
{
  const auto key = request.headers.at("sec-websocket-key");
  std::ostringstream out;
  out << "HTTP/1.1 101 Switching Protocols\r\n";
  out << "Upgrade: websocket\r\n";
  out << "Connection: Upgrade\r\n";
  out << "Sec-WebSocket-Accept: " << websocket_accept_key(key) << "\r\n";
  out << "Access-Control-Allow-Origin: *\r\n";
  out << "\r\n";
  const auto response = out.str();
  return send_all_bytes(client_fd, response.data(), response.size());
}

bool send_websocket_frame(
  const int client_fd,
  const std::string & payload,
  const std::uint8_t opcode)
{
  std::vector<std::uint8_t> header;
  header.push_back(static_cast<std::uint8_t>(0x80U | (opcode & 0x0FU)));
  if (payload.size() <= 125U) {
    header.push_back(static_cast<std::uint8_t>(payload.size()));
  } else if (payload.size() <= 65535U) {
    header.push_back(126U);
    header.push_back(static_cast<std::uint8_t>((payload.size() >> 8U) & 0xFFU));
    header.push_back(static_cast<std::uint8_t>(payload.size() & 0xFFU));
  } else {
    return false;
  }
  if (!send_all_bytes(client_fd, header.data(), header.size())) {
    return false;
  }
  return payload.empty() || send_all_bytes(client_fd, payload.data(), payload.size());
}

std::optional<WebSocketFrame> read_websocket_frame(
  const int client_fd,
  const std::atomic_bool & running)
{
  std::uint8_t header[2]{};
  if (!recv_exact(client_fd, header, sizeof(header), running)) {
    return std::nullopt;
  }

  const std::uint8_t opcode = header[0] & 0x0FU;
  const bool masked = (header[1] & 0x80U) != 0U;
  std::uint64_t payload_length = header[1] & 0x7FU;
  if (payload_length == 126U) {
    std::uint8_t extended[2]{};
    if (!recv_exact(client_fd, extended, sizeof(extended), running)) {
      return std::nullopt;
    }
    payload_length = (static_cast<std::uint64_t>(extended[0]) << 8U) |
      static_cast<std::uint64_t>(extended[1]);
  } else if (payload_length == 127U) {
    std::uint8_t extended[8]{};
    if (!recv_exact(client_fd, extended, sizeof(extended), running)) {
      return std::nullopt;
    }
    payload_length = 0;
    for (const std::uint8_t byte : extended) {
      payload_length = (payload_length << 8U) | static_cast<std::uint64_t>(byte);
    }
  }
  if (payload_length > 4096U) {
    return std::nullopt;
  }

  std::uint8_t mask[4]{};
  if (masked && !recv_exact(client_fd, mask, sizeof(mask), running)) {
    return std::nullopt;
  }

  std::string payload(static_cast<std::size_t>(payload_length), '\0');
  if (payload_length > 0U && !recv_exact(
      client_fd, payload.data(), payload.size(), running))
  {
    return std::nullopt;
  }
  if (masked) {
    for (std::size_t index = 0; index < payload.size(); ++index) {
      payload[index] = static_cast<char>(
        static_cast<std::uint8_t>(payload[index]) ^ mask[index % 4U]);
    }
  }
  return WebSocketFrame{opcode, payload};
}

}  // namespace robot_api_server::infrastructure::http

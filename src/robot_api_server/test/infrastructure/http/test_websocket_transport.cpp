#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <unistd.h>

#include "robot_api_server/infrastructure/http/websocket_transport.hpp"

namespace robot_api_server::infrastructure::http
{
namespace
{

TEST(WebSocketTransport, ValidatesRequiredUpgradeHeaders)
{
  HttpRequest request;
  request.headers["upgrade"] = "WebSocket";
  request.headers["connection"] = "keep-alive, Upgrade";
  request.headers["sec-websocket-key"] = "dGhlIHNhbXBsZSBub25jZQ==";

  EXPECT_TRUE(websocket_headers_valid(request));

  request.headers.erase("sec-websocket-key");
  EXPECT_FALSE(websocket_headers_valid(request));
}

TEST(WebSocketTransport, ReadsMaskedClientFrameAndWritesExtendedServerFrame)
{
  int sockets[2]{-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), 0);

  const std::string client_payload = "hello";
  const std::array<std::uint8_t, 4> mask{0x12U, 0x34U, 0x56U, 0x78U};
  std::vector<std::uint8_t> encoded{0x81U, 0x85U};
  encoded.insert(encoded.end(), mask.begin(), mask.end());
  for (std::size_t index = 0; index < client_payload.size(); ++index) {
    encoded.push_back(
      static_cast<std::uint8_t>(client_payload[index]) ^ mask[index % mask.size()]);
  }
  ASSERT_EQ(
    ::send(sockets[0], encoded.data(), encoded.size(), 0),
    static_cast<ssize_t>(encoded.size()));

  std::atomic_bool running{true};
  const auto received = read_websocket_frame(sockets[1], running);
  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(received->opcode, 0x1U);
  EXPECT_EQ(received->payload, client_payload);

  const std::string server_payload(126U, 'x');
  ASSERT_TRUE(send_websocket_frame(sockets[1], server_payload, 0x1U));
  std::array<std::uint8_t, 4> server_header{};
  ASSERT_EQ(
    ::recv(sockets[0], server_header.data(), server_header.size(), MSG_WAITALL),
    static_cast<ssize_t>(server_header.size()));
  EXPECT_EQ(server_header[0], 0x81U);
  EXPECT_EQ(server_header[1], 126U);
  EXPECT_EQ(server_header[2], 0U);
  EXPECT_EQ(server_header[3], 126U);
  std::string decoded(server_payload.size(), '\0');
  ASSERT_EQ(
    ::recv(sockets[0], decoded.data(), decoded.size(), MSG_WAITALL),
    static_cast<ssize_t>(decoded.size()));
  EXPECT_EQ(decoded, server_payload);

  ::close(sockets[0]);
  ::close(sockets[1]);
}

TEST(WebSocketTransport, RejectsClientPayloadAboveExistingLimit)
{
  int sockets[2]{-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), 0);
  const std::array<std::uint8_t, 10> oversized_header{
    0x81U, 0xFFU,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x10U, 0x01U,
  };
  ASSERT_EQ(
    ::send(sockets[0], oversized_header.data(), oversized_header.size(), 0),
    static_cast<ssize_t>(oversized_header.size()));

  std::atomic_bool running{true};
  EXPECT_FALSE(read_websocket_frame(sockets[1], running).has_value());

  ::close(sockets[0]);
  ::close(sockets[1]);
}

TEST(WebSocketTransport, SendsStandardsCompatibleHandshake)
{
  int sockets[2]{-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), 0);
  HttpRequest request;
  request.headers["sec-websocket-key"] = "dGhlIHNhbXBsZSBub25jZQ==";

  ASSERT_TRUE(send_websocket_handshake(sockets[1], request));
  std::array<char, 512> buffer{};
  const auto count = ::recv(sockets[0], buffer.data(), buffer.size(), 0);
  ASSERT_GT(count, 0);
  const std::string response(buffer.data(), static_cast<std::size_t>(count));
  EXPECT_NE(response.find("HTTP/1.1 101 Switching Protocols\r\n"), std::string::npos);
  EXPECT_NE(
    response.find("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n"),
    std::string::npos);

  ::close(sockets[0]);
  ::close(sockets[1]);
}

TEST(WebSocketTransport, AppliesFractionalReceiveTimeout)
{
  int sockets[2]{-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), 0);

  set_socket_receive_timeout(sockets[1], 1.25);
  timeval timeout{};
  socklen_t timeout_size = sizeof(timeout);
  ASSERT_EQ(
    ::getsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &timeout, &timeout_size),
    0);
  EXPECT_EQ(timeout.tv_sec, 1);
  // Linux may round SO_RCVTIMEO to the socket clock granularity.
  EXPECT_NEAR(timeout.tv_usec, 250000, 5000);

  ::close(sockets[0]);
  ::close(sockets[1]);
}

}  // namespace
}  // namespace robot_api_server::infrastructure::http

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <functional>
#include <fstream>
#include <future>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

bool is_transient_action_client_exception(const std::exception & exc)
{
  const std::string message = exc.what();
  return message.find("Taking data from action client but no ready event") != std::string::npos;
}

struct HttpRequest
{
  std::string method;
  std::string path;
  std::map<std::string, std::string> query;
  std::map<std::string, std::string> headers;
  std::string body;
};

struct HttpResponse
{
  int status{200};
  std::string content_type{"application/json"};
  std::string body{"{}"};
};

struct WebSocketFrame
{
  std::uint8_t opcode{0};
  std::string payload;
};

std::string trim(std::string value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();
  if (first >= last) {
    return "";
  }
  return std::string(first, last);
}

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string strip_query(const std::string & path)
{
  const auto pos = path.find('?');
  if (pos == std::string::npos) {
    return path;
  }
  return path.substr(0, pos);
}

std::map<std::string, std::string> parse_query_params(const std::string & path)
{
  std::map<std::string, std::string> query;
  const auto query_pos = path.find('?');
  if (query_pos == std::string::npos || query_pos + 1 >= path.size()) {
    return query;
  }
  std::string rest = path.substr(query_pos + 1);
  while (!rest.empty()) {
    const auto amp = rest.find('&');
    const auto token = amp == std::string::npos ? rest : rest.substr(0, amp);
    const auto eq = token.find('=');
    if (eq != std::string::npos) {
      query[token.substr(0, eq)] = token.substr(eq + 1);
    } else if (!token.empty()) {
      query[token] = "";
    }
    if (amp == std::string::npos) {
      break;
    }
    rest = rest.substr(amp + 1);
  }
  return query;
}

std::string reason_phrase(const int status)
{
  switch (status) {
    case 200:
      return "OK";
    case 202:
      return "Accepted";
    case 400:
      return "Bad Request";
    case 401:
      return "Unauthorized";
    case 404:
      return "Not Found";
    case 405:
      return "Method Not Allowed";
    case 409:
      return "Conflict";
    case 500:
      return "Internal Server Error";
    case 501:
      return "Not Implemented";
    case 503:
      return "Service Unavailable";
    default:
      return "OK";
  }
}

std::string json_escape(const std::string & value)
{
  std::ostringstream out;
  for (const char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::string json_string(const std::string & value)
{
  return "\"" + json_escape(value) + "\"";
}

std::string error_json(const std::string & error)
{
  return "{\"ok\":false,\"error\":" + json_string(error) + "}";
}

void set_close_on_exec(const int fd)
{
  if (fd < 0) {
    return;
  }
  const int flags = ::fcntl(fd, F_GETFD);
  if (flags < 0) {
    return;
  }
  ::fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
}

void close_inherited_fds()
{
  long max_fd = ::sysconf(_SC_OPEN_MAX);
  if (max_fd < 0) {
    max_fd = 4096;
  }
  max_fd = std::min<long>(max_fd, 65536);
  for (int fd = STDERR_FILENO + 1; fd < max_fd; ++fd) {
    ::close(fd);
  }
}

std::optional<std::string> json_string_value(const std::string & body, const std::string & key)
{
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = body.find(needle);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = body.find(':', key_pos + needle.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  auto value_pos = body.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (value_pos == std::string::npos || body[value_pos] != '"') {
    return std::nullopt;
  }
  ++value_pos;
  std::string value;
  bool escaped = false;
  for (std::size_t i = value_pos; i < body.size(); ++i) {
    const char c = body[i];
    if (escaped) {
      value.push_back(c);
      escaped = false;
      continue;
    }
    if (c == '\\') {
      escaped = true;
      continue;
    }
    if (c == '"') {
      return value;
    }
    value.push_back(c);
  }
  return std::nullopt;
}

std::vector<std::string> json_string_array_value(const std::string & body, const std::string & key)
{
  std::vector<std::string> values;
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = body.find(needle);
  if (key_pos == std::string::npos) {
    return values;
  }
  const auto colon_pos = body.find(':', key_pos + needle.size());
  if (colon_pos == std::string::npos) {
    return values;
  }
  auto pos = body.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (pos == std::string::npos || body[pos] != '[') {
    return values;
  }
  ++pos;
  while (pos < body.size()) {
    pos = body.find_first_not_of(" \t\r\n,", pos);
    if (pos == std::string::npos || body[pos] == ']') {
      break;
    }
    if (body[pos] != '"') {
      break;
    }
    ++pos;
    std::string value;
    bool escaped = false;
    for (; pos < body.size(); ++pos) {
      const char c = body[pos];
      if (escaped) {
        value.push_back(c);
        escaped = false;
        continue;
      }
      if (c == '\\') {
        escaped = true;
        continue;
      }
      if (c == '"') {
        ++pos;
        values.push_back(value);
        break;
      }
      value.push_back(c);
    }
  }
  return values;
}

bool json_bool_value(const std::string & body, const std::string & key, const bool default_value)
{
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = body.find(needle);
  if (key_pos == std::string::npos) {
    return default_value;
  }
  const auto colon_pos = body.find(':', key_pos + needle.size());
  if (colon_pos == std::string::npos) {
    return default_value;
  }
  const auto value_pos = body.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (value_pos == std::string::npos) {
    return default_value;
  }
  if (body.compare(value_pos, 4, "true") == 0) {
    return true;
  }
  if (body.compare(value_pos, 5, "false") == 0) {
    return false;
  }
  return default_value;
}

std::optional<double> json_number_value(const std::string & body, const std::string & key)
{
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = body.find(needle);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = body.find(':', key_pos + needle.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto value_pos = body.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (value_pos == std::string::npos) {
    return std::nullopt;
  }
  std::size_t end_pos = value_pos;
  while (end_pos < body.size()) {
    const char c = body[end_pos];
    if (!(std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E')) {
      break;
    }
    ++end_pos;
  }
  if (end_pos == value_pos) {
    return std::nullopt;
  }
  try {
    return std::stod(body.substr(value_pos, end_pos - value_pos));
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<std::string> json_object_value(const std::string & body, const std::string & key)
{
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = body.find(needle);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto colon_pos = body.find(':', key_pos + needle.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }
  const auto object_start = body.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (object_start == std::string::npos || body[object_start] != '{') {
    return std::nullopt;
  }

  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (std::size_t i = object_start; i < body.size(); ++i) {
    const char c = body[i];
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }
    if (c == '"') {
      in_string = true;
    } else if (c == '{') {
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0) {
        return body.substr(object_start, i - object_start + 1);
      }
    }
  }
  return std::nullopt;
}

std::vector<std::string> json_object_array_value(const std::string & body, const std::string & key)
{
  std::vector<std::string> objects;
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = body.find(needle);
  if (key_pos == std::string::npos) {
    return objects;
  }
  const auto colon_pos = body.find(':', key_pos + needle.size());
  if (colon_pos == std::string::npos) {
    return objects;
  }
  auto pos = body.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (pos == std::string::npos || body[pos] != '[') {
    return objects;
  }
  ++pos;
  while (pos < body.size()) {
    pos = body.find_first_not_of(" \t\r\n,", pos);
    if (pos == std::string::npos || body[pos] == ']') {
      break;
    }
    if (body[pos] != '{') {
      break;
    }

    const auto object_start = pos;
    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (; pos < body.size(); ++pos) {
      const char c = body[pos];
      if (in_string) {
        if (escaped) {
          escaped = false;
        } else if (c == '\\') {
          escaped = true;
        } else if (c == '"') {
          in_string = false;
        }
        continue;
      }
      if (c == '"') {
        in_string = true;
      } else if (c == '{') {
        ++depth;
      } else if (c == '}') {
        --depth;
        if (depth == 0) {
          objects.push_back(body.substr(object_start, pos - object_start + 1));
          ++pos;
          break;
        }
      }
    }
  }
  return objects;
}

std::optional<double> json_nested_number_value(
  const std::string & body,
  const std::string & object_key,
  const std::string & number_key)
{
  const auto object = json_object_value(body, object_key);
  if (!object) {
    return std::nullopt;
  }
  return json_number_value(*object, number_key);
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::string base64_encode(const std::vector<std::uint8_t> & bytes)
{
  static constexpr char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((bytes.size() + 2) / 3) * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const std::uint32_t b0 = bytes[i];
    const std::uint32_t b1 = (i + 1 < bytes.size()) ? bytes[i + 1] : 0;
    const std::uint32_t b2 = (i + 2 < bytes.size()) ? bytes[i + 2] : 0;
    const std::uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
    out.push_back(table[(triple >> 18) & 0x3F]);
    out.push_back(table[(triple >> 12) & 0x3F]);
    out.push_back((i + 1 < bytes.size()) ? table[(triple >> 6) & 0x3F] : '=');
    out.push_back((i + 2 < bytes.size()) ? table[triple & 0x3F] : '=');
  }
  return out;
}

std::uint32_t left_rotate(const std::uint32_t value, const std::uint32_t shift)
{
  return (value << shift) | (value >> (32U - shift));
}

std::array<std::uint8_t, 20> sha1_hash(const std::string & input)
{
  std::vector<std::uint8_t> data(input.begin(), input.end());
  const std::uint64_t bit_length = static_cast<std::uint64_t>(data.size()) * 8U;
  data.push_back(0x80);
  while ((data.size() % 64U) != 56U) {
    data.push_back(0x00);
  }
  for (int i = 7; i >= 0; --i) {
    data.push_back(static_cast<std::uint8_t>((bit_length >> (i * 8)) & 0xFFU));
  }

  std::uint32_t h0 = 0x67452301U;
  std::uint32_t h1 = 0xEFCDAB89U;
  std::uint32_t h2 = 0x98BADCFEU;
  std::uint32_t h3 = 0x10325476U;
  std::uint32_t h4 = 0xC3D2E1F0U;

  for (std::size_t chunk = 0; chunk < data.size(); chunk += 64U) {
    std::uint32_t w[80]{};
    for (std::size_t i = 0; i < 16; ++i) {
      const std::size_t j = chunk + i * 4U;
      w[i] = (static_cast<std::uint32_t>(data[j]) << 24U) |
        (static_cast<std::uint32_t>(data[j + 1]) << 16U) |
        (static_cast<std::uint32_t>(data[j + 2]) << 8U) |
        static_cast<std::uint32_t>(data[j + 3]);
    }
    for (std::size_t i = 16; i < 80; ++i) {
      w[i] = left_rotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1U);
    }

    std::uint32_t a = h0;
    std::uint32_t b = h1;
    std::uint32_t c = h2;
    std::uint32_t d = h3;
    std::uint32_t e = h4;

    for (std::size_t i = 0; i < 80; ++i) {
      std::uint32_t f = 0;
      std::uint32_t k = 0;
      if (i < 20) {
        f = (b & c) | ((~b) & d);
        k = 0x5A827999U;
      } else if (i < 40) {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1U;
      } else if (i < 60) {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDCU;
      } else {
        f = b ^ c ^ d;
        k = 0xCA62C1D6U;
      }
      const std::uint32_t temp = left_rotate(a, 5U) + f + e + k + w[i];
      e = d;
      d = c;
      c = left_rotate(b, 30U);
      b = a;
      a = temp;
    }

    h0 += a;
    h1 += b;
    h2 += c;
    h3 += d;
    h4 += e;
  }

  std::array<std::uint8_t, 20> digest{};
  const std::uint32_t words[5] = {h0, h1, h2, h3, h4};
  for (std::size_t i = 0; i < 5; ++i) {
    digest[i * 4] = static_cast<std::uint8_t>((words[i] >> 24U) & 0xFFU);
    digest[i * 4 + 1] = static_cast<std::uint8_t>((words[i] >> 16U) & 0xFFU);
    digest[i * 4 + 2] = static_cast<std::uint8_t>((words[i] >> 8U) & 0xFFU);
    digest[i * 4 + 3] = static_cast<std::uint8_t>(words[i] & 0xFFU);
  }
  return digest;
}

std::string websocket_accept_key(const std::string & client_key)
{
  static const std::string magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  const auto digest = sha1_hash(client_key + magic);
  return base64_encode(std::vector<std::uint8_t>(digest.begin(), digest.end()));
}

void append_u32_be(std::vector<std::uint8_t> & out, const std::uint32_t value)
{
  out.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

std::uint32_t crc32_bytes(const std::uint8_t * data, const std::size_t size)
{
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint32_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xEDB88320U & static_cast<std::uint32_t>(-(static_cast<int>(crc & 1U))));
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

std::uint32_t adler32_bytes(const std::vector<std::uint8_t> & data)
{
  std::uint32_t a = 1U;
  std::uint32_t b = 0U;
  for (const std::uint8_t byte : data) {
    a = (a + byte) % 65521U;
    b = (b + a) % 65521U;
  }
  return (b << 16U) | a;
}

void append_png_chunk(
  std::vector<std::uint8_t> & png,
  const char type[4],
  const std::vector<std::uint8_t> & data)
{
  append_u32_be(png, static_cast<std::uint32_t>(data.size()));
  const std::size_t crc_start = png.size();
  png.insert(png.end(), type, type + 4);
  png.insert(png.end(), data.begin(), data.end());
  append_u32_be(png, crc32_bytes(png.data() + crc_start, png.size() - crc_start));
}

std::vector<std::uint8_t> zlib_store_blocks(const std::vector<std::uint8_t> & raw)
{
  std::vector<std::uint8_t> out;
  out.reserve(raw.size() + raw.size() / 65535U * 5U + 16U);
  out.push_back(0x78U);
  out.push_back(0x01U);

  std::size_t offset = 0;
  while (offset < raw.size()) {
    const std::size_t block_size = std::min<std::size_t>(65535U, raw.size() - offset);
    const bool final_block = (offset + block_size) == raw.size();
    out.push_back(final_block ? 0x01U : 0x00U);
    const std::uint16_t len = static_cast<std::uint16_t>(block_size);
    const std::uint16_t nlen = static_cast<std::uint16_t>(~len);
    out.push_back(static_cast<std::uint8_t>(len & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((len >> 8U) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>(nlen & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((nlen >> 8U) & 0xFFU));
    out.insert(out.end(), raw.begin() + static_cast<std::ptrdiff_t>(offset),
      raw.begin() + static_cast<std::ptrdiff_t>(offset + block_size));
    offset += block_size;
  }

  append_u32_be(out, adler32_bytes(raw));
  return out;
}

std::string encode_grayscale_png(
  const std::uint32_t width,
  const std::uint32_t height,
  const std::vector<std::uint8_t> & pixels)
{
  if (width == 0U || height == 0U || pixels.size() != static_cast<std::size_t>(width) * height) {
    return {};
  }

  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(height) * (static_cast<std::size_t>(width) + 1U));
  for (std::uint32_t y = 0; y < height; ++y) {
    raw.push_back(0U);  // PNG filter type: None.
    raw.insert(raw.end(),
      pixels.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(y) * width),
      pixels.begin() + static_cast<std::ptrdiff_t>((static_cast<std::size_t>(y) + 1U) * width));
  }

  std::vector<std::uint8_t> png = {0x89U, 'P', 'N', 'G', '\r', '\n', 0x1AU, '\n'};
  std::vector<std::uint8_t> ihdr;
  append_u32_be(ihdr, width);
  append_u32_be(ihdr, height);
  ihdr.push_back(8U);  // bit depth
  ihdr.push_back(0U);  // grayscale
  ihdr.push_back(0U);  // compression
  ihdr.push_back(0U);  // filter
  ihdr.push_back(0U);  // interlace
  append_png_chunk(png, "IHDR", ihdr);
  append_png_chunk(png, "IDAT", zlib_store_blocks(raw));
  append_png_chunk(png, "IEND", {});
  return std::string(reinterpret_cast<const char *>(png.data()), png.size());
}

std::optional<HttpRequest> parse_http_request(const std::string & raw)
{
  const auto header_end = raw.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    return std::nullopt;
  }

  HttpRequest request;
  std::istringstream header_stream(raw.substr(0, header_end));
  std::string line;
  if (!std::getline(header_stream, line)) {
    return std::nullopt;
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::istringstream request_line(line);
  if (!(request_line >> request.method >> request.path)) {
    return std::nullopt;
  }
  request.query = parse_query_params(request.path);
  request.path = strip_query(request.path);

  while (std::getline(header_stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    request.headers[lower_copy(trim(line.substr(0, colon)))] = trim(line.substr(colon + 1));
  }

  request.body = raw.substr(header_end + 4);
  return request;
}

std::size_t content_length_from_headers(const std::string & raw_headers)
{
  std::istringstream stream(raw_headers);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    if (lower_copy(trim(line.substr(0, colon))) == "content-length") {
      try {
        return static_cast<std::size_t>(std::stoul(trim(line.substr(colon + 1))));
      } catch (...) {
        return 0;
      }
    }
  }
  return 0;
}

class SubscriptionManager
{
public:
  using Clock = std::chrono::steady_clock;
  using TransitionCallback = std::function<void(const std::string &, bool)>;

  SubscriptionManager(std::vector<std::string> supported_resources, TransitionCallback transition_callback)
  : supported_resources_(std::move(supported_resources)),
    transition_callback_(std::move(transition_callback))
  {
  }

  bool supported(const std::string & resource) const
  {
    return std::find(supported_resources_.begin(), supported_resources_.end(), resource) !=
           supported_resources_.end();
  }

  std::optional<std::string> validate_resources(const std::vector<std::string> & resources) const
  {
    for (const auto & resource : resources) {
      if (!supported(resource)) {
        return "unsupported subscription resource: " + resource;
      }
    }
    return std::nullopt;
  }

  void acquire(
    const std::string & client_id,
    const std::vector<std::string> & resources,
    const std::chrono::milliseconds ttl)
  {
    std::vector<std::pair<std::string, bool>> transitions;
    const auto expires_at = Clock::now() + ttl;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & resource : resources) {
        auto & clients = leases_[resource];
        const bool was_empty = clients.empty();
        clients[client_id] = expires_at;
        if (was_empty) {
          transitions.emplace_back(resource, true);
        }
      }
    }
    apply_transitions(transitions);
  }

  void release(const std::string & client_id, const std::vector<std::string> & requested_resources)
  {
    std::vector<std::pair<std::string, bool>> transitions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto resources = requested_resources.empty() ? resources_for_client_locked(client_id) : requested_resources;
      for (const auto & resource : resources) {
        auto it = leases_.find(resource);
        if (it == leases_.end()) {
          continue;
        }
        if (it->second.erase(client_id) > 0U && it->second.empty()) {
          transitions.emplace_back(resource, false);
        }
      }
    }
    apply_transitions(transitions);
  }

  std::vector<std::string> resources_for_client(const std::string & client_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return resources_for_client_locked(client_id);
  }

  void expire()
  {
    std::vector<std::pair<std::string, bool>> transitions;
    const auto now = Clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto & [resource, clients] : leases_) {
        const bool was_empty = clients.empty();
        for (auto it = clients.begin(); it != clients.end();) {
          if (it->second <= now) {
            it = clients.erase(it);
          } else {
            ++it;
          }
        }
        if (!was_empty && clients.empty()) {
          transitions.emplace_back(resource, false);
        }
      }
    }
    apply_transitions(transitions);
  }

  bool active(const std::string & resource) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = leases_.find(resource);
    return it != leases_.end() && !it->second.empty();
  }

  std::size_t ref_count(const std::string & resource) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = leases_.find(resource);
    return it == leases_.end() ? 0U : it->second.size();
  }

  std::string snapshot_json() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream out;
    out << "{\"resources\":{";
    for (std::size_t i = 0; i < supported_resources_.size(); ++i) {
      const auto & resource = supported_resources_[i];
      const auto it = leases_.find(resource);
      const std::size_t count = it == leases_.end() ? 0U : it->second.size();
      if (i > 0U) {
        out << ",";
      }
      out << json_string(resource) << ":{\"active\":" << (count > 0U ? "true" : "false")
          << ",\"ref_count\":" << count << "}";
    }
    out << "}}";
    return out.str();
  }

private:
  std::vector<std::string> resources_for_client_locked(const std::string & client_id) const
  {
    std::vector<std::string> resources;
    for (const auto & [resource, clients] : leases_) {
      if (clients.find(client_id) != clients.end()) {
        resources.push_back(resource);
      }
    }
    return resources;
  }

  void apply_transitions(const std::vector<std::pair<std::string, bool>> & transitions)
  {
    for (const auto & transition : transitions) {
      transition_callback_(transition.first, transition.second);
    }
  }

  std::vector<std::string> supported_resources_;
  TransitionCallback transition_callback_;
  mutable std::mutex mutex_;
  std::map<std::string, std::map<std::string, Clock::time_point>> leases_;
};

}  // namespace

class RobotApiServerNode : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

public:
  RobotApiServerNode()
  : Node("robot_api_server")
  {
    configure_runtime_permissions();
    host_ = declare_parameter<std::string>("host", "0.0.0.0");
    port_ = declare_parameter<int>("port", 8080);
    api_token_ = declare_parameter<std::string>("api_token", "");
    max_http_connections_ = std::max(1, static_cast<int>(declare_parameter<int>("max_http_connections", 32)));
    maps_root_ = declare_parameter<std::string>("maps_root", "/workspaces/njrh-v3/workspace1/maps_release");
    runtime_maps_dir_ = declare_parameter<std::string>(
      "runtime_maps_dir", "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps");

    safety_estop_topic_ = declare_parameter<std::string>("safety_estop_topic", "/safety/estop");
    safety_status_topic_ = declare_parameter<std::string>("safety_status_topic", "/safety/status");
    safety_motion_allowed_topic_ =
      declare_parameter<std::string>("safety_motion_allowed_topic", "/safety/motion_allowed");
    floor_status_topic_ = declare_parameter<std::string>("floor_status_topic", "/floor_manager/status");
    bms_state_topic_ = declare_parameter<std::string>("bms_state_topic", "/battery_state");
    bms_state_max_age_sec_ = std::max(0.1, declare_parameter<double>("bms_state_max_age_sec", 3.0));
    teleop_stop_on_charging_ = declare_parameter<bool>("teleop_stop_on_charging", true);
    teleop_charging_current_min_a_ =
      std::max(0.0, declare_parameter<double>("teleop_charging_current_min_a", 0.10));
    floor_switch_service_ = declare_parameter<std::string>("floor_switch_service", "/floor_manager/switch_floor");
    localization_trigger_service_ =
      declare_parameter<std::string>("localization_trigger_service", "/global_localization/trigger");
    navigate_to_pose_action_ = declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    mapping_2d_start_command_ = declare_parameter<std::string>(
      "mapping_2d_start_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_projected_map.sh");
    mapping_2d_log_file_ =
      declare_parameter<std::string>("mapping_2d_log_file", "/tmp/njrh_mapping2d_slam_toolbox.log");
    navigation_resume_command_ = declare_parameter<std::string>(
      "navigation_resume_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_floor_navigation.sh");
    navigation_resume_log_file_ =
      declare_parameter<std::string>("navigation_resume_log_file", "/tmp/njrh_navigation_resume.log");
    runtime_map_context_file_ =
      declare_parameter<std::string>("runtime_map_context_file", "/tmp/njrh_runtime_map_context.json");
    navigation_stop_command_ = declare_parameter<std::string>(
      "navigation_stop_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/stop_floor_navigation.sh");
    navigation_stop_log_file_ =
      declare_parameter<std::string>("navigation_stop_log_file", "/tmp/njrh_navigation_stop.log");
    docking_manager_start_command_ = declare_parameter<std::string>(
      "docking_manager_start_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_docking_manager.sh");
    docking_manager_log_file_ =
      declare_parameter<std::string>("docking_manager_log_file", "/tmp/njrh_docking_manager.log");
    docking_start_service_ = declare_parameter<std::string>("docking_start_service", "/docking/start");
    docking_stop_service_ = declare_parameter<std::string>("docking_stop_service", "/docking/stop");
    docking_status_topic_ = declare_parameter<std::string>("docking_status_topic", "/docking/status");
    docking_pre_dock_distance_m_ =
      std::max(0.05, declare_parameter<double>("docking_pre_dock_distance_m", 0.80));
    mapping_2d_live_map_topic_ = declare_parameter<std::string>("mapping_2d_live_map_topic", "/map");
    mapping_2d_live_map_max_age_sec_ =
      std::max(0.1, declare_parameter<double>("mapping_2d_live_map_max_age_sec", 3.0));
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    scan_max_age_sec_ = std::max(0.1, declare_parameter<double>("scan_max_age_sec", 2.0));
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
    tf_map_frame_ = declare_parameter<std::string>("tf_map_frame", "map");
    tf_odom_frame_ = declare_parameter<std::string>("tf_odom_frame", "odom");
    tf_base_frame_ = declare_parameter<std::string>("tf_base_frame", "base_link");
    tf_map_frame_ = normalized_frame_id(tf_map_frame_);
    tf_odom_frame_ = normalized_frame_id(tf_odom_frame_);
    tf_base_frame_ = normalized_frame_id(tf_base_frame_);
    tf_pose_max_age_sec_ = std::max(0.1, declare_parameter<double>("tf_pose_max_age_sec", 2.0));
    robot_pose_freshness_sec_ =
      std::max(0.05, declare_parameter<double>("robot_pose_freshness_sec", 0.5));
    teleop_cmd_topic_ = declare_parameter<std::string>("teleop_cmd_topic", "/cmd_vel_collision_checked");
    teleop_reverse_enable_topic_ =
      declare_parameter<std::string>("teleop_reverse_enable_topic", "/ranger_mini3/allow_reverse");
    teleop_pose_topic_ = declare_parameter<std::string>("teleop_pose_topic", "/local_state/odometry");
    teleop_max_linear_x_mps_ =
      std::max(0.0, declare_parameter<double>("teleop_max_linear_x_mps", 0.30));
    teleop_max_angular_z_radps_ =
      std::max(0.0, declare_parameter<double>("teleop_max_angular_z_radps", 0.55));
    teleop_allow_reverse_ = declare_parameter<bool>("teleop_allow_reverse", false);
    teleop_require_mapping_active_ = declare_parameter<bool>("teleop_require_mapping_active", true);
    teleop_watchdog_timeout_sec_ =
      std::max(0.1, declare_parameter<double>("teleop_watchdog_timeout_sec", 0.5));
    teleop_socket_idle_timeout_sec_ = std::max(
      teleop_watchdog_timeout_sec_,
      declare_parameter<double>("teleop_socket_idle_timeout_sec", 5.0));
    teleop_repeat_rate_hz_ = std::max(1.0, declare_parameter<double>("teleop_repeat_rate_hz", 20.0));
    subscription_default_ttl_ms_ =
      std::max(1000, static_cast<int>(declare_parameter<int>("subscription_default_ttl_ms", 10000)));
    subscription_max_ttl_ms_ =
      std::max(
        subscription_default_ttl_ms_,
        static_cast<int>(declare_parameter<int>("subscription_max_ttl_ms", 60000)));
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 8.0);
    docking_navigation_start_wait_sec_ =
      std::max(service_timeout_sec_, declare_parameter<double>("docking_navigation_start_wait_sec", 45.0));
    docking_predock_nav_timeout_sec_ =
      std::max(5.0, declare_parameter<double>("docking_predock_nav_timeout_sec", 180.0));

    estop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_estop_topic_, rclcpp::QoS(10).transient_local());
    teleop_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(teleop_cmd_topic_, rclcpp::QoS(10));
    teleop_reverse_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(teleop_reverse_enable_topic_, rclcpp::QoS(1));
    bms_state_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      bms_state_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        handle_bms_state(msg);
      });
    docking_status_sub_ = create_subscription<std_msgs::msg::String>(
      docking_status_topic_, rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_docking_status(msg->data);
      });
    teleop_repeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / teleop_repeat_rate_hz_)),
      [this]() { on_teleop_repeat_timer(); });
    subscription_manager_ = std::make_unique<SubscriptionManager>(
      std::vector<std::string>{"status", "live_map", "scan", "tf", "teleop"},
      [this](const std::string & resource, const bool active) {
        set_subscription_resource_active(resource, active);
      });
    subscription_ttl_timer_ = create_wall_timer(
      1s,
      [this]() {
        if (subscription_manager_) {
          subscription_manager_->expire();
        }
      });

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    floor_switch_client_ = create_client<robot_interfaces::srv::SwitchFloor>(
      floor_switch_service_, rmw_qos_profile_services_default, callback_group_);
    localization_trigger_client_ = create_client<robot_interfaces::srv::TriggerLocalization>(
      localization_trigger_service_, rmw_qos_profile_services_default, callback_group_);
    docking_start_client_ = create_client<std_srvs::srv::Trigger>(
      docking_start_service_, rmw_qos_profile_services_default, callback_group_);
    docking_stop_client_ = create_client<std_srvs::srv::Trigger>(
      docking_stop_service_, rmw_qos_profile_services_default, callback_group_);
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_action_);

    start_server();
  }

  ~RobotApiServerNode() override
  {
    stop_server();
    join_navigation_cancel_worker();
    join_docking_worker();
  }

private:
  void configure_runtime_permissions() const
  {
    ::umask(0002);
  }

  struct StoredPose
  {
    std::string id;
    std::string name;
    std::string type{"delivery_point"};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct RobotPoseSnapshot
  {
    bool available{false};
    std::string frame_id;
    std::string child_frame_id;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double stamp_sec{0.0};
    double age_sec{-1.0};
  };

  struct NavigationCancelJob
  {
    std::uint64_t id{0U};
    std::string state{"idle"};
    std::string phase{"idle"};
    std::string reason;
    std::string detail;
    std::string cancel_all_detail;
    std::string stop_stack_detail;
    std::string started_at;
    std::string finished_at;
    bool stop_stack{true};
    bool ok{true};
    bool action_available{false};
    bool active_goal_cancel_requested{false};
    bool cancel_all_requested{false};
    bool cancel_all_ok{true};
    bool stop_stack_ok{true};
    bool zero_velocity_published{false};
  };

  struct DockingJob
  {
    std::uint64_t id{0U};
    std::string state{"idle"};
    std::string phase{"idle"};
    std::string building_id;
    std::string floor_id;
    std::string map_id;
    std::string dock_id;
    std::string dock_name;
    std::string dock_type;
    std::string detail;
    std::string last_status;
    std::string started_at;
    std::string finished_at;
    double dock_x{0.0};
    double dock_y{0.0};
    double dock_yaw{0.0};
    double approach_x{0.0};
    double approach_y{0.0};
    double approach_yaw{0.0};
    double approach_distance_m{0.0};
    bool ok{true};
    bool resume_navigation{true};
    bool nav_goal_sent{false};
    bool nav_goal_succeeded{false};
    bool docking_service_called{false};
    bool cancel_requested{false};
  };

  struct RuntimeModeSnapshot
  {
    std::string mode{"IDLE"};
    std::string state{"idle"};
    std::string mapping_state{"stopped"};
    std::string navigation_state{"stopped"};
    std::string docking_state{"stopped"};
    std::string docking_status;
    std::string docking_dock_id;
    std::string message;
    bool mapping_active{false};
    bool navigation_active{false};
    bool docking_active{false};
    bool healthy{true};
  };

  std::chrono::nanoseconds service_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  }

  void set_mapping_runtime_state(
    const bool active,
    const std::string & state,
    const std::string & message = "",
    const bool healthy = true)
  {
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    mapping_runtime_active_ = active;
    mapping_runtime_state_ = state;
    runtime_healthy_ = healthy;
    runtime_message_ = message;
    if (active) {
      navigation_runtime_active_ = false;
      navigation_runtime_state_ = "stopped";
      docking_runtime_active_ = false;
      docking_runtime_state_ = "stopped";
    }
  }

  void set_navigation_runtime_state(
    const bool active,
    const std::string & state,
    const std::string & message = "",
    const bool healthy = true)
  {
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    navigation_runtime_active_ = active;
    navigation_runtime_state_ = state;
    runtime_healthy_ = healthy;
    runtime_message_ = message;
    if (active) {
      mapping_runtime_active_ = false;
      mapping_runtime_state_ = "stopped";
    }
  }

  void set_docking_runtime_state(
    const bool active,
    const std::string & state,
    const std::string & message = "",
    const bool healthy = true)
  {
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    docking_runtime_active_ = active;
    docking_runtime_state_ = state;
    runtime_healthy_ = healthy;
    runtime_message_ = message;
    if (active) {
      mapping_runtime_active_ = false;
      mapping_runtime_state_ = "stopped";
    }
  }

  RuntimeModeSnapshot runtime_mode_snapshot() const
  {
    RuntimeModeSnapshot snapshot;
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    snapshot.mapping_active = mapping_runtime_active_;
    snapshot.navigation_active = navigation_runtime_active_;
    snapshot.docking_active = docking_runtime_active_;
    snapshot.mapping_state = mapping_runtime_state_;
    snapshot.navigation_state = navigation_runtime_state_;
    snapshot.docking_state = docking_runtime_state_;
    snapshot.docking_status = docking_runtime_status_;
    snapshot.docking_dock_id = docking_runtime_dock_id_;
    snapshot.healthy = runtime_healthy_;
    snapshot.message = runtime_message_;
    if (!snapshot.healthy) {
      snapshot.mode = "ERROR";
      snapshot.state = "error";
    } else if (snapshot.docking_active) {
      snapshot.mode = "DOCKING";
      snapshot.state = snapshot.docking_state;
    } else if (snapshot.mapping_active) {
      snapshot.mode = "MAPPING_2D";
      snapshot.state = snapshot.mapping_state;
    } else if (snapshot.navigation_active) {
      snapshot.mode = "NAVIGATION";
      snapshot.state = snapshot.navigation_state;
    } else {
      snapshot.mode = "IDLE";
      snapshot.state = "idle";
    }
    return snapshot;
  }

  void set_subscription_resource_active(const std::string & resource, const bool active)
  {
    if (resource == "status") {
      set_status_subscriptions_active(active);
    } else if (resource == "live_map") {
      set_live_map_subscription_active(active);
    } else if (resource == "scan") {
      set_scan_subscription_active(active);
    } else if (resource == "tf") {
      set_tf_subscription_active(active);
    } else if (resource == "teleop" && !active) {
      clear_teleop_command();
    }
  }

  void set_status_subscriptions_active(const bool active)
  {
    std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
    if (active) {
      if (!safety_status_sub_) {
        safety_status_sub_ = create_subscription<std_msgs::msg::String>(
          safety_status_topic_, rclcpp::QoS(10),
          [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            latest_safety_status_ = msg->data;
          });
      }
      if (!motion_allowed_sub_) {
        motion_allowed_sub_ = create_subscription<std_msgs::msg::Bool>(
          safety_motion_allowed_topic_, rclcpp::QoS(10),
          [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            latest_motion_allowed_ = msg->data;
            have_motion_allowed_ = true;
          });
      }
      if (!floor_status_sub_) {
        floor_status_sub_ = create_subscription<std_msgs::msg::String>(
          floor_status_topic_, rclcpp::QoS(10),
          [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            latest_floor_status_ = msg->data;
          });
      }
      return;
    }

    safety_status_sub_.reset();
    motion_allowed_sub_.reset();
    floor_status_sub_.reset();
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    latest_safety_status_ = "UNKNOWN";
    latest_floor_status_ = "UNKNOWN";
    latest_motion_allowed_ = false;
    have_motion_allowed_ = false;
  }

  void set_live_map_subscription_active(const bool active)
  {
    {
      std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
      if (active) {
        if (!live_map_sub_) {
          live_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            mapping_2d_live_map_topic_, rclcpp::QoS(1).reliable(),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
              std::lock_guard<std::mutex> map_lock(live_map_mutex_);
              latest_live_map_ = *msg;
              latest_live_map_received_at_ = std::chrono::steady_clock::now();
              have_live_map_ = true;
            });
        }
        return;
      }
      live_map_sub_.reset();
    }
    std::lock_guard<std::mutex> map_lock(live_map_mutex_);
    latest_live_map_ = nav_msgs::msg::OccupancyGrid{};
    have_live_map_ = false;
  }

  void set_scan_subscription_active(const bool active)
  {
    {
      std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
      if (active) {
        if (!scan_sub_) {
          scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::QoS(10),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
              std::lock_guard<std::mutex> state_lock(state_mutex_);
              latest_scan_frame_ = msg->header.frame_id;
              latest_scan_range_count_ = msg->ranges.size();
              latest_scan_angle_min_ = msg->angle_min;
              latest_scan_angle_max_ = msg->angle_max;
              latest_scan_received_at_ = std::chrono::steady_clock::now();
              have_scan_ = true;
            });
        }
        return;
      }
      scan_sub_.reset();
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    latest_scan_frame_.clear();
    latest_scan_range_count_ = 0U;
    have_scan_ = false;
  }

  void set_tf_subscription_active(const bool active)
  {
    {
      std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
      if (active) {
        if (!tf_sub_) {
          tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic_, rclcpp::QoS(100),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
              handle_tf_message(msg);
            });
        }
        return;
      }
      tf_sub_.reset();
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    have_pose_ = false;
    have_map_to_odom_ = false;
    have_odom_to_base_ = false;
    latest_pose_stamp_sec_ = 0.0;
    latest_map_to_odom_stamp_sec_ = 0.0;
    latest_odom_to_base_stamp_sec_ = 0.0;
  }

  bool tf_subscription_active()
  {
    std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
    return static_cast<bool>(tf_sub_);
  }

  RobotPoseSnapshot current_robot_pose_snapshot()
  {
    RobotPoseSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_pose_ || latest_pose_frame_ != tf_map_frame_) {
      return snapshot;
    }
    snapshot.available = true;
    snapshot.frame_id = latest_pose_frame_;
    snapshot.child_frame_id = tf_base_frame_;
    snapshot.x = latest_pose_x_;
    snapshot.y = latest_pose_y_;
    snapshot.yaw = latest_pose_yaw_;
    snapshot.stamp_sec = latest_pose_stamp_sec_;
    snapshot.age_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_pose_received_at_).count();
    return snapshot;
  }

  RobotPoseSnapshot wait_for_current_robot_pose(
    const bool require_map_frame,
    std::string & error)
  {
    const bool was_active = tf_subscription_active();
    set_tf_subscription_active(true);

    RobotPoseSnapshot snapshot;
    const auto deadline = std::chrono::steady_clock::now() + service_timeout();
    while (std::chrono::steady_clock::now() <= deadline) {
      snapshot = current_robot_pose_snapshot();
      if (snapshot.available && snapshot.age_sec <= robot_pose_freshness_sec_) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }

    if (!was_active && (!subscription_manager_ || !subscription_manager_->active("tf"))) {
      set_tf_subscription_active(false);
    }

    if (!snapshot.available) {
      error = "no fresh map-frame robot pose";
      return snapshot;
    }
    if (snapshot.age_sec > robot_pose_freshness_sec_) {
      error = "no fresh map-frame robot pose";
      snapshot.available = false;
      return snapshot;
    }
    (void)require_map_frame;
    return snapshot;
  }

  std::string no_fresh_map_robot_pose_json() const
  {
    return std::string("{\"ok\":false,") +
      "\"error\":\"no fresh map-frame robot pose\"," +
      "\"frame_id\":" + json_string(tf_map_frame_) + "," +
      "\"child_frame_id\":" + json_string(tf_base_frame_) + "," +
      "\"age_sec\":null}";
  }

  std::string no_fresh_map_robot_pose_json(const std::string & detail) const
  {
    return std::string("{\"ok\":false,") +
      "\"error\":\"no fresh map-frame robot pose\"," +
      "\"detail\":" + json_string(detail) + "," +
      "\"frame_id\":" + json_string(tf_map_frame_) + "," +
      "\"child_frame_id\":" + json_string(tf_base_frame_) + "," +
      "\"age_sec\":null}";
  }

  std::string generated_current_pose_id(const std::string & type, const std::string & name) const
  {
    std::string prefix;
    const std::string raw_prefix = type.empty() ? "pose" : type;
    for (const unsigned char c : raw_prefix) {
      if (std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == ':') {
        prefix.push_back(static_cast<char>(c));
      } else {
        prefix.push_back('_');
      }
      if (prefix.size() >= 40U) {
        break;
      }
    }
    while (!prefix.empty() && (prefix.front() == '_' || prefix.front() == '.')) {
      prefix.erase(prefix.begin());
    }
    while (!prefix.empty() && (prefix.back() == '_' || prefix.back() == '.')) {
      prefix.pop_back();
    }
    if (prefix.empty() || !safe_pose_id(prefix)) {
      prefix = "pose";
    }
    const auto stamp = utc_timestamp_compact();
    const auto seed = prefix + "/" + name + "/" + stamp + "/" + std::to_string(::getpid());
    return prefix + "_" + stamp + "_" + fixed_hex(fnv1a64(seed), 8);
  }

  void handle_bms_state(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    const bool charging = battery_indicates_charging(*msg);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (std::isfinite(msg->percentage)) {
        const double raw_soc = static_cast<double>(msg->percentage);
        latest_bms_soc_ = std::clamp(raw_soc <= 1.0 ? raw_soc * 100.0 : raw_soc, 0.0, 100.0);
        have_bms_soc_ = true;
      } else {
        have_bms_soc_ = false;
      }
      latest_bms_voltage_ = static_cast<double>(msg->voltage);
      latest_bms_current_ = static_cast<double>(msg->current);
      latest_bms_temperature_ = static_cast<double>(msg->temperature);
      latest_bms_power_supply_status_ = static_cast<int>(msg->power_supply_status);
      latest_bms_received_at_ = std::chrono::steady_clock::now();
      have_bms_state_ = true;
    }
    if (charging && teleop_stop_on_charging_) {
      clear_teleop_command();
    }
  }

  std::string normalized_frame_id(std::string frame) const
  {
    while (!frame.empty() && frame.front() == '/') {
      frame.erase(frame.begin());
    }
    return frame;
  }

  double normalize_angle(const double angle) const
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp) const
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  double older_nonzero_stamp(const double lhs, const double rhs) const
  {
    if (lhs > 0.0 && rhs > 0.0) {
      return std::min(lhs, rhs);
    }
    return std::max(lhs, rhs);
  }

  void handle_tf_message(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    bool saw_direct_map_to_base = false;
    for (const auto & transform : msg->transforms) {
      const auto parent = normalized_frame_id(transform.header.frame_id);
      const auto child = normalized_frame_id(transform.child_frame_id);
      const double x = transform.transform.translation.x;
      const double y = transform.transform.translation.y;
      const double yaw = quaternion_yaw(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      const double stamp_sec = stamp_to_seconds(transform.header.stamp);

      if (parent == tf_map_frame_ && child == tf_base_frame_) {
        latest_pose_frame_ = tf_map_frame_;
        latest_pose_x_ = x;
        latest_pose_y_ = y;
        latest_pose_yaw_ = yaw;
        latest_pose_stamp_sec_ = stamp_sec;
        latest_pose_received_at_ = now;
        have_pose_ = true;
        saw_direct_map_to_base = true;
      } else if (parent == tf_map_frame_ && child == tf_odom_frame_) {
        latest_map_to_odom_x_ = x;
        latest_map_to_odom_y_ = y;
        latest_map_to_odom_yaw_ = yaw;
        latest_map_to_odom_stamp_sec_ = stamp_sec;
        latest_map_to_odom_received_at_ = now;
        have_map_to_odom_ = true;
      } else if (parent == tf_odom_frame_ && child == tf_base_frame_) {
        latest_odom_to_base_x_ = x;
        latest_odom_to_base_y_ = y;
        latest_odom_to_base_yaw_ = yaw;
        latest_odom_to_base_stamp_sec_ = stamp_sec;
        latest_odom_to_base_received_at_ = now;
        have_odom_to_base_ = true;
      }
    }

    if (!saw_direct_map_to_base && have_map_to_odom_ && have_odom_to_base_) {
      const double c = std::cos(latest_map_to_odom_yaw_);
      const double s = std::sin(latest_map_to_odom_yaw_);
      latest_pose_frame_ = tf_map_frame_;
      latest_pose_x_ = latest_map_to_odom_x_ + c * latest_odom_to_base_x_ - s * latest_odom_to_base_y_;
      latest_pose_y_ = latest_map_to_odom_y_ + s * latest_odom_to_base_x_ + c * latest_odom_to_base_y_;
      latest_pose_yaw_ = normalize_angle(latest_map_to_odom_yaw_ + latest_odom_to_base_yaw_);
      latest_pose_stamp_sec_ =
        older_nonzero_stamp(latest_map_to_odom_stamp_sec_, latest_odom_to_base_stamp_sec_);
      latest_pose_received_at_ =
        latest_map_to_odom_received_at_ < latest_odom_to_base_received_at_ ?
        latest_map_to_odom_received_at_ : latest_odom_to_base_received_at_;
      have_pose_ = true;
    } else if (have_odom_to_base_ && !have_pose_) {
      latest_pose_frame_ = tf_odom_frame_;
      latest_pose_x_ = latest_odom_to_base_x_;
      latest_pose_y_ = latest_odom_to_base_y_;
      latest_pose_yaw_ = latest_odom_to_base_yaw_;
      latest_pose_stamp_sec_ = latest_odom_to_base_stamp_sec_;
      latest_pose_received_at_ = latest_odom_to_base_received_at_;
      have_pose_ = true;
    }
  }

  void start_server()
  {
    running_.store(true);
    server_thread_ = std::thread([this]() { serve(); });
  }

  void stop_server()
  {
    running_.store(false);
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
      ::close(server_fd_);
      server_fd_ = -1;
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  void serve()
  {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "failed to create API socket: %s", std::strerror(errno));
      return;
    }
    set_close_on_exec(server_fd_);

    int reuse = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (host_ == "0.0.0.0" || host_.empty()) {
      address.sin_addr.s_addr = INADDR_ANY;
    } else if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "invalid API host: %s", host_.c_str());
      return;
    }

    if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      RCLCPP_ERROR(get_logger(), "failed to bind API server on %s:%d: %s", host_.c_str(), port_, std::strerror(errno));
      return;
    }
    if (::listen(server_fd_, 64) < 0) {
      RCLCPP_ERROR(get_logger(), "failed to listen on API socket: %s", std::strerror(errno));
      return;
    }

    RCLCPP_INFO(get_logger(), "robot_api_server listening on %s:%d", host_.c_str(), port_);
    while (running_.load()) {
      sockaddr_in client_address{};
      socklen_t client_length = sizeof(client_address);
      const int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr *>(&client_address), &client_length);
      if (client_fd < 0) {
        if (running_.load()) {
          RCLCPP_WARN(get_logger(), "API accept failed: %s", std::strerror(errno));
        }
        continue;
      }
      set_close_on_exec(client_fd);
      timeval timeout{};
      timeout.tv_sec = 15;
      timeout.tv_usec = 0;
      ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
      if (active_http_connections_.load(std::memory_order_relaxed) >= max_http_connections_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "rejecting HTTP client: active connections reached limit %d",
          max_http_connections_);
        send_response(client_fd, {503, "application/json", error_json("server busy")});
        ::close(client_fd);
        continue;
      }
      active_http_connections_.fetch_add(1, std::memory_order_acq_rel);
      std::thread(
        [this, client_fd]() {
          struct ActiveConnectionGuard
          {
            std::atomic<int> & counter;
            ~ActiveConnectionGuard()
            {
              counter.fetch_sub(1, std::memory_order_acq_rel);
            }
          } guard{active_http_connections_};
          handle_client(client_fd);
        }).detach();
    }
  }

  void handle_client(const int client_fd)
  {
    std::string raw;
    char buffer[4096];
    std::size_t expected_body = 0;

    while (running_.load()) {
      const ssize_t count = ::recv(client_fd, buffer, sizeof(buffer), 0);
      if (count <= 0) {
        break;
      }
      raw.append(buffer, static_cast<std::size_t>(count));
      const auto header_end = raw.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        expected_body = content_length_from_headers(raw.substr(0, header_end));
        const auto current_body = raw.size() - header_end - 4;
        if (current_body >= expected_body) {
          break;
        }
      }
      if (raw.size() > 1024 * 1024) {
        send_response(client_fd, {400, "application/json", error_json("request too large")});
        ::close(client_fd);
        return;
      }
    }

    const auto request = parse_http_request(raw);
    if (!request) {
      send_response(client_fd, {400, "application/json", error_json("invalid HTTP request")});
      ::close(client_fd);
      return;
    }

    if (request->method == "GET" && request->path == "/ws/v1/teleop") {
      handle_teleop_websocket(client_fd, *request);
      ::close(client_fd);
      return;
    }

    const auto started_at = std::chrono::steady_clock::now();
    HttpResponse response;
    try {
      response = route(*request);
    } catch (const std::exception & exc) {
      response = {
        500,
        "application/json",
        error_json(std::string("unhandled API exception: ") + exc.what())
      };
    } catch (...) {
      response = {500, "application/json", error_json("unknown unhandled API exception")};
    }
    const auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at).count();
    log_http_request(*request, response.status, latency_ms);
    send_response(client_fd, response);
    ::close(client_fd);
  }

  void send_response(const int client_fd, const HttpResponse & response)
  {
    std::ostringstream out;
    out << "HTTP/1.1 " << response.status << " " << reason_phrase(response.status) << "\r\n";
    out << "Content-Type: " << response.content_type << "\r\n";
    out << "Content-Length: " << response.body.size() << "\r\n";
    out << "Connection: close\r\n";
    out << "Access-Control-Allow-Origin: *\r\n";
    out << "Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS\r\n";
    out << "Access-Control-Allow-Headers: Content-Type, X-Robot-Token\r\n";
    out << "\r\n";
    out << response.body;
    const auto text = out.str();
    if (!send_all_text(client_fd, text)) {
      RCLCPP_WARN(get_logger(), "failed to send full HTTP response status=%d bytes=%zu", response.status, text.size());
    }
  }

  void log_http_request(const HttpRequest & request, const int status, const long latency_ms)
  {
    if (status >= 500) {
      RCLCPP_ERROR(
        get_logger(),
        "HTTP %s %s -> %d in %ld ms",
        request.method.c_str(),
        request.path.c_str(),
        status,
        latency_ms);
      return;
    }
    if (status >= 400) {
      RCLCPP_WARN(
        get_logger(),
        "HTTP %s %s -> %d in %ld ms",
        request.method.c_str(),
        request.path.c_str(),
        status,
        latency_ms);
      return;
    }
    if (latency_ms > 2000) {
      RCLCPP_WARN(
        get_logger(),
        "slow HTTP %s %s -> %d in %ld ms",
        request.method.c_str(),
        request.path.c_str(),
        status,
        latency_ms);
    }
  }

  bool token_allowed(const HttpRequest & request) const
  {
    if (api_token_.empty()) {
      return true;
    }
    const auto it = request.headers.find("x-robot-token");
    return it != request.headers.end() && it->second == api_token_;
  }

  HttpResponse route(const HttpRequest & request)
  {
    if (request.method == "OPTIONS") {
      return {200, "application/json", "{\"ok\":true}"};
    }
    if (!token_allowed(request)) {
      return {401, "application/json", error_json("missing or invalid X-Robot-Token")};
    }

    if (request.method == "GET" && request.path == "/api/v1/status") {
      return handle_status();
    }
    if (request.method == "GET" && request.path == "/api/v1/robot/pose") {
      return handle_robot_pose(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps") {
      return handle_maps();
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/semantic_layer") {
      return handle_get_semantic_layer(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/poses") {
      return handle_get_poses(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/filters/keepout") {
      return handle_get_keepout_filter(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/mapping/2d/map") {
      return handle_mapping_2d_map_png(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/openapi") {
      return handle_openapi();
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/acquire") {
      return handle_subscription_update(request.body, "acquire");
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/release") {
      return handle_subscription_update(request.body, "release");
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/heartbeat") {
      return handle_subscription_update(request.body, "heartbeat");
    }
    if (request.method == "POST" && request.path == "/api/v1/mapping/2d/start") {
      return handle_start_mapping_2d();
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/mapping/2d/stop" || request.path == "/api/v1/mapping/stop")) {
      return handle_stop_mapping_2d();
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/mapping/2d/save" || request.path == "/api/v1/mapping/save")) {
      return handle_save_mapping_2d(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/delete") {
      return handle_delete_map(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/poses") {
      return handle_save_pose(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/poses/save") {
      return handle_save_pose(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/poses/save_current") {
      return handle_save_current_pose(request.body);
    }
    if (request.method == "PUT" && request.path == "/api/v1/maps/poses/batch") {
      return handle_replace_poses_batch(request.body);
    }
    const std::string pose_item_prefix = "/api/v1/maps/poses/";
    if ((request.method == "PUT" || request.method == "DELETE") && starts_with(request.path, pose_item_prefix)) {
      const auto pose_id = request.path.substr(pose_item_prefix.size());
      if (!safe_pose_id(pose_id)) {
        return {400, "application/json", error_json("valid pose_id path segment is required")};
      }
      if (request.method == "PUT") {
        return handle_save_pose(request.body, std::optional<std::string>(pose_id));
      }
      return handle_delete_pose(request, pose_id);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/filters/keepout/save") {
      return handle_save_keepout_filter(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/safety/stop") {
      return publish_estop(true);
    }
    if (request.method == "POST" && request.path == "/api/v1/safety/resume") {
      return publish_estop(false);
    }
    if (request.method == "POST" && request.path == "/api/v1/floors/switch") {
      return handle_switch_floor(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/localization/trigger") {
      return handle_trigger_localization(request.body);
    }
    if (request.method == "GET" && request.path == "/api/v1/navigation/state") {
      return handle_navigation_state();
    }
    if (request.method == "POST" && request.path == "/api/v1/navigation/goal") {
      return handle_navigation_goal(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/navigation/cancel") {
      return handle_navigation_cancel(request.body);
    }
    if (request.method == "GET" && request.path == "/api/v1/docking/state") {
      return handle_docking_state();
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/start") {
      return handle_docking_start(request.body);
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/docking/cancel" || request.path == "/api/v1/docking/stop")) {
      return handle_docking_cancel(request.body);
    }
    if (starts_with(request.path, "/api/v1/mapping/") || starts_with(request.path, "/api/v1/navigation/")) {
      return not_wired(request.path);
    }
    return {404, "application/json", error_json("endpoint not found: " + request.path)};
  }

  HttpResponse handle_status()
  {
    std::string safety_status;
    std::string floor_status;
    bool motion_allowed = false;
    bool have_motion_allowed = false;
    bool have_bms_state = false;
    bool have_bms_soc = false;
    double bms_soc = 0.0;
    double bms_voltage = 0.0;
    double bms_current = 0.0;
    double bms_temperature = 0.0;
    double bms_age_sec = -1.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      safety_status = latest_safety_status_;
      floor_status = latest_floor_status_;
      motion_allowed = latest_motion_allowed_;
      have_motion_allowed = have_motion_allowed_;
      have_bms_state = have_bms_state_;
      have_bms_soc = have_bms_soc_;
      bms_soc = latest_bms_soc_;
      bms_voltage = latest_bms_voltage_;
      bms_current = latest_bms_current_;
      bms_temperature = latest_bms_temperature_;
      if (have_bms_state_) {
        bms_age_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_bms_received_at_).count();
      }
    }
    const bool bms_valid = have_bms_state && have_bms_soc && bms_age_sec <= bms_state_max_age_sec_;
    const auto runtime = runtime_mode_snapshot();

    std::ostringstream body;
    body << "{";
    body << "\"ok\":true,";
    body << "\"api_version\":\"v1\",";
    body << "\"node\":\"robot_api_server\",";
    body << "\"mode\":" << json_string(runtime.mode) << ",";
    body << "\"state\":" << json_string(runtime.state) << ",";
    body << "\"mapping_active\":" << (runtime.mapping_active ? "true" : "false") << ",";
    body << "\"navigation_active\":" << (runtime.navigation_active ? "true" : "false") << ",";
    body << "\"healthy\":" << (runtime.healthy ? "true" : "false") << ",";
    body << "\"message\":" << json_string(runtime.message) << ",";
    body << "\"mapping\":{";
    body << "\"active\":" << (runtime.mapping_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.mapping_state) << ",";
    body << "\"map_topic\":" << json_string(mapping_2d_live_map_topic_) << ",";
    body << "\"map_endpoint\":\"/api/v1/mapping/2d/map\"";
    body << "},";
    body << "\"navigation\":{";
    body << "\"active\":" << (runtime.navigation_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.navigation_state) << ",";
    body << "\"action\":" << json_string(navigate_to_pose_action_);
    body << "},";
    body << "\"docking_active\":" << (runtime.docking_active ? "true" : "false") << ",";
    body << "\"docking\":{";
    body << "\"active\":" << (runtime.docking_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.docking_state) << ",";
    body << "\"dock_id\":" << json_string(runtime.docking_dock_id) << ",";
    body << "\"status_topic\":" << json_string(docking_status_topic_) << ",";
    body << "\"last_status\":" << json_string(runtime.docking_status);
    body << "},";
    body << "\"safety_status\":" << json_string(safety_status) << ",";
    body << "\"motion_allowed\":" << (motion_allowed ? "true" : "false") << ",";
    body << "\"motion_allowed_valid\":" << (have_motion_allowed ? "true" : "false") << ",";
    body << "\"floor_status\":" << json_string(floor_status) << ",";
    body << "\"bms\":{";
    body << "\"soc\":";
    if (have_bms_soc) {
      body << bms_soc;
    } else {
      body << "null";
    }
    body << ",\"soc_valid\":" << (bms_valid ? "true" : "false") << ",";
    body << "\"source_topic\":" << json_string(bms_state_topic_) << ",";
    body << "\"age_sec\":" << bms_age_sec << ",";
    body << "\"voltage\":";
    if (have_bms_state && std::isfinite(bms_voltage)) {
      body << bms_voltage;
    } else {
      body << "null";
    }
    body << ",\"current\":";
    if (have_bms_state && std::isfinite(bms_current)) {
      body << bms_current;
    } else {
      body << "null";
    }
    body << ",\"temperature\":";
    if (have_bms_state && std::isfinite(bms_temperature)) {
      body << bms_temperature;
    } else {
      body << "null";
    }
    body << "},";
    body << "\"subscriptions\":";
    if (subscription_manager_) {
      body << subscription_manager_->snapshot_json();
    } else {
      body << "{\"resources\":{}}";
    }
    body << ",";
    body << "\"http\":{";
    body << "\"active_connections\":" << active_http_connections_.load(std::memory_order_relaxed) << ",";
    body << "\"max_connections\":" << max_http_connections_;
    body << "},";
    body << "\"maps_root\":" << json_string(maps_root_) << ",";
    body << "\"runtime_maps_dir\":" << json_string(runtime_maps_dir_);
    body << "}";
    return {200, "application/json", body.str()};
  }

  HttpResponse handle_robot_pose(const HttpRequest & request)
  {
    (void)request;
    std::string error;
    const auto pose = wait_for_current_robot_pose(true, error);
    if (!pose.available) {
      (void)error;
      return {503, "application/json", no_fresh_map_robot_pose_json()};
    }

    std::string context_error;
    bool blocked_by_pending_context = false;
    const auto active_map = confirmed_runtime_map_manifest(context_error, blocked_by_pending_context);
    if (blocked_by_pending_context) {
      return {503, "application/json", no_fresh_map_robot_pose_json(context_error)};
    }
    std::ostringstream body;
    body << std::fixed << std::setprecision(6)
         << "{\"ok\":true,"
         << "\"frame_id\":" << json_string(pose.frame_id) << ","
         << "\"child_frame_id\":" << json_string(pose.child_frame_id) << ","
         << "\"x\":" << pose.x << ","
         << "\"y\":" << pose.y << ","
         << "\"yaw\":" << pose.yaw << ","
         << "\"stamp\":" << pose.stamp_sec << ","
         << "\"age_sec\":" << pose.age_sec << ","
         << "\"map_id\":";
    if (active_map) {
      body << json_string(active_map->map_id);
    } else {
      body << "null";
    }
    body << ",\"floor_id\":";
    if (active_map) {
      body << json_string(active_map->floor_id);
    } else {
      body << "null";
    }
    body << ",\"building_id\":";
    if (active_map) {
      body << json_string(active_map->building_id);
    } else {
      body << "null";
    }
    body << "}";
    return {200, "application/json", body.str()};
  }

  HttpResponse handle_openapi()
  {
    const std::string body =
      "{"
      "\"ok\":true,"
      "\"endpoints\":["
      "\"GET /api/v1/status\","
      "\"GET /api/v1/robot/pose\","
      "\"GET /api/v1/maps\","
      "\"GET /api/v1/maps/semantic_layer\","
      "\"GET /api/v1/maps/poses\","
      "\"GET /api/v1/maps/filters/keepout\","
      "\"GET /api/v1/mapping/2d/map\","
      "\"GET /api/v1/openapi\","
      "\"POST /api/v1/maps/poses\","
      "\"PUT /api/v1/maps/poses/{pose_id}\","
      "\"DELETE /api/v1/maps/poses/{pose_id}\","
      "\"PUT /api/v1/maps/poses/batch\","
      "\"POST /api/v1/subscriptions/acquire\","
      "\"POST /api/v1/subscriptions/release\","
      "\"POST /api/v1/subscriptions/heartbeat\","
      "\"POST /api/v1/mapping/2d/start\","
      "\"POST /api/v1/mapping/2d/stop\","
      "\"POST /api/v1/mapping/2d/save\","
      "\"POST /api/v1/mapping/stop\","
      "\"POST /api/v1/mapping/save\","
      "\"POST /api/v1/maps/delete\","
      "\"POST /api/v1/maps/poses/save\","
      "\"POST /api/v1/maps/poses/save_current\","
      "\"POST /api/v1/maps/filters/keepout/save\","
      "\"POST /api/v1/safety/stop\","
      "\"POST /api/v1/safety/resume\","
      "\"POST /api/v1/floors/switch\","
      "\"POST /api/v1/localization/trigger\","
      "\"GET /api/v1/navigation/state\","
      "\"POST /api/v1/navigation/goal\","
      "\"POST /api/v1/navigation/cancel\","
      "\"GET /api/v1/docking/state\","
      "\"POST /api/v1/docking/start\","
      "\"POST /api/v1/docking/cancel\","
      "\"POST /api/v1/docking/stop\","
      "\"WS /ws/v1/teleop\""
      "],"
      "\"not_wired\":["
      "\"POST /api/v1/mapping/3d/start\","
      "\"POST /api/v1/navigation/start\""
      "]"
      "}";
    return {200, "application/json", body};
  }

  bool safe_client_id(const std::string & client_id) const
  {
    if (client_id.empty() || client_id.size() > 128U) {
      return false;
    }
    return std::all_of(client_id.begin(), client_id.end(), [](const unsigned char c) {
      return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == ':';
    });
  }

  int subscription_ttl_ms_from_body(const std::string & body) const
  {
    const auto ttl = json_number_value(body, "ttl_ms");
    if (!ttl || !std::isfinite(*ttl)) {
      return subscription_default_ttl_ms_;
    }
    return std::clamp(static_cast<int>(*ttl), 1000, subscription_max_ttl_ms_);
  }

  std::vector<std::string> subscription_resources_from_body(const std::string & body) const
  {
    auto resources = json_string_array_value(body, "resources");
    if (resources.empty()) {
      const auto resource = json_string_value(body, "resource");
      if (resource) {
        resources.push_back(*resource);
      }
    }
    std::sort(resources.begin(), resources.end());
    resources.erase(std::unique(resources.begin(), resources.end()), resources.end());
    return resources;
  }

  std::pair<std::string, std::string> subscription_client_id_from_body(const std::string & body) const
  {
    const std::array<std::string, 6> keys = {
      "client_id", "clientId", "lease_id", "leaseId", "subscription_id", "subscriptionId"};
    for (const auto & key : keys) {
      const auto value = json_string_value(body, key);
      if (value && safe_client_id(*value)) {
        return {*value, key};
      }
    }
    return {"http:compat-default", "fallback"};
  }

  std::string resource_list_json(const std::vector<std::string> & resources) const
  {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < resources.size(); ++i) {
      if (i > 0U) {
        out << ",";
      }
      out << json_string(resources[i]);
    }
    out << "]";
    return out.str();
  }

  HttpResponse handle_subscription_update(const std::string & body, const std::string & action)
  {
    if (!subscription_manager_) {
      return {503, "application/json", error_json("subscription manager is not initialized")};
    }
    const auto [client_id, client_id_source] = subscription_client_id_from_body(body);
    auto resources = subscription_resources_from_body(body);
    if (action == "heartbeat" && resources.empty()) {
      resources = subscription_manager_->resources_for_client(client_id);
    }
    if (action == "heartbeat" && resources.empty()) {
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"action\":" << json_string(action) << ","
               << "\"client_id\":" << json_string(client_id) << ","
               << "\"lease_id\":" << json_string(client_id) << ","
               << "\"client_id_source\":" << json_string(client_id_source) << ","
               << "\"refreshed\":false,"
               << "\"ttl_ms\":0,"
               << "\"resources\":[],"
               << "\"subscriptions\":" << subscription_manager_->snapshot_json() << "}";
      return {200, "application/json", response.str()};
    }
    if (action != "release" && resources.empty()) {
      return {400, "application/json", error_json("resources array is required")};
    }
    if (const auto error = subscription_manager_->validate_resources(resources)) {
      return {400, "application/json", error_json(*error)};
    }

    int ttl_ms = subscription_ttl_ms_from_body(body);
    if (action == "acquire" || action == "heartbeat") {
      subscription_manager_->acquire(client_id, resources, std::chrono::milliseconds(ttl_ms));
    } else if (action == "release") {
      subscription_manager_->release(client_id, resources);
      ttl_ms = 0;
    } else {
      return {400, "application/json", error_json("unsupported subscription action")};
    }

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"action\":" << json_string(action) << ","
             << "\"client_id\":" << json_string(client_id) << ","
             << "\"lease_id\":" << json_string(client_id) << ","
             << "\"client_id_source\":" << json_string(client_id_source) << ","
             << "\"refreshed\":" << ((action == "heartbeat") ? "true" : "false") << ","
             << "\"ttl_ms\":" << ttl_ms << ","
             << "\"resources\":" << resource_list_json(resources) << ","
             << "\"subscriptions\":" << subscription_manager_->snapshot_json() << "}";
    return {200, "application/json", response.str()};
  }

  struct MapManifest
  {
    std::string map_id;
    std::string display_name;
    std::string safe_map_name;
    std::string building_id;
    std::string floor_id;
    std::string created_at;
    bool active{false};
    fs::path root;
    fs::path manifest_json;
    fs::path nav_map_yaml;
    fs::path nav_map_pgm;
    fs::path localizer_map_png;
    fs::path localizer_params_yaml;
    fs::path keepout_mask_yaml;
    fs::path keepout_mask_pgm;
    fs::path speed_mask_yaml;
    fs::path speed_mask_pgm;
    fs::path binary_mask_yaml;
    fs::path binary_mask_pgm;
    fs::path asset_report_json;
    fs::path poses_yaml;
  };

  struct RuntimeMapContext
  {
    bool confirmed{false};
    std::string state;
    std::string message;
    std::string map_id;
    std::string display_name;
    std::string building_id;
    std::string floor_id;
  };

  struct MapYamlInfo
  {
    std::uint32_t width{0};
    std::uint32_t height{0};
    double resolution{0.0};
    std::array<double, 3> origin{0.0, 0.0, 0.0};
  };

  bool valid_display_map_name(const std::string & name) const
  {
    const auto trimmed = trim(name);
    return !trimmed.empty() && trimmed.size() <= 256U &&
      trimmed.find('/') == std::string::npos && trimmed.find('\\') == std::string::npos &&
      trimmed.find("..") == std::string::npos;
  }

  std::string safe_file_stem_from_display_name(const std::string & display_name) const
  {
    std::string safe;
    bool previous_underscore = false;
    for (const unsigned char c : display_name) {
      char out = '\0';
      if (std::isalnum(c) != 0 || c == '-' || c == '.') {
        out = static_cast<char>(c);
      } else if (c == '_' || std::isspace(c) != 0) {
        out = '_';
      } else {
        out = '_';
      }
      if (out == '_') {
        if (previous_underscore) {
          continue;
        }
        previous_underscore = true;
      } else {
        previous_underscore = false;
      }
      safe.push_back(out);
      if (safe.size() >= 80U) {
        break;
      }
    }
    while (!safe.empty() && (safe.front() == '_' || safe.front() == '.')) {
      safe.erase(safe.begin());
    }
    while (!safe.empty() && (safe.back() == '_' || safe.back() == '.')) {
      safe.pop_back();
    }
    if (safe.empty()) {
      safe = "map";
    }
    return safe;
  }

  std::uint64_t fnv1a64(const std::string & value) const
  {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char c : value) {
      hash ^= static_cast<std::uint64_t>(c);
      hash *= 1099511628211ULL;
    }
    return hash;
  }

  std::string fixed_hex(const std::uint64_t value, const int width) const
  {
    std::ostringstream out;
    out << std::hex << std::nouppercase << std::setw(width) << std::setfill('0') << value;
    auto text = out.str();
    if (static_cast<int>(text.size()) > width) {
      text = text.substr(text.size() - static_cast<std::size_t>(width));
    }
    return text;
  }

  std::string utc_timestamp_compact() const
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    gmtime_r(&time, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%dT%H%M%SZ");
    return out.str();
  }

  std::string utc_timestamp_iso8601() const
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    gmtime_r(&time, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
  }

  double wall_time_seconds() const
  {
    return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  std::string generate_map_id(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & display_name) const
  {
    const auto stamp = utc_timestamp_compact();
    const auto seed = building_id + "/" + floor_id + "/" + display_name + "/" + stamp + "/" +
      std::to_string(::getpid());
    return "map_" + stamp + "_" + fixed_hex(fnv1a64(seed), 10);
  }

  fs::path floor_maps_root_path(const std::string & building_id, const std::string & floor_id) const
  {
    return floor_root_path(building_id, floor_id) / "maps";
  }

  fs::path floor_current_root_path(const std::string & building_id, const std::string & floor_id) const
  {
    return floor_root_path(building_id, floor_id) / "current";
  }

  fs::path map_root_path(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id) const
  {
    return floor_maps_root_path(building_id, floor_id) / map_id;
  }

  void fill_manifest_paths(MapManifest & manifest) const
  {
    manifest.manifest_json = manifest.root / "manifest.json";
    manifest.nav_map_yaml = manifest.root / "nav" / (manifest.safe_map_name + ".yaml");
    manifest.nav_map_pgm = manifest.root / "nav" / (manifest.safe_map_name + ".pgm");
    manifest.localizer_map_png = manifest.root / "localizer" / (manifest.safe_map_name + ".png");
    manifest.localizer_params_yaml = manifest.root / "localizer" / (manifest.safe_map_name + ".yaml");
    manifest.keepout_mask_yaml = manifest.root / "filters" / "keepout_mask.yaml";
    manifest.keepout_mask_pgm = manifest.root / "filters" / "keepout_mask.pgm";
    manifest.speed_mask_yaml = manifest.root / "filters" / "speed_mask.yaml";
    manifest.speed_mask_pgm = manifest.root / "filters" / "speed_mask.pgm";
    manifest.binary_mask_yaml = manifest.root / "filters" / "binary_mask.yaml";
    manifest.binary_mask_pgm = manifest.root / "filters" / "binary_mask.pgm";
    manifest.asset_report_json = manifest.root / "reports" / "asset_report.json";
    manifest.poses_yaml = manifest.root / "poses.yaml";
  }

  MapManifest make_new_manifest(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & display_name) const
  {
    MapManifest manifest;
    manifest.map_id = generate_map_id(building_id, floor_id, display_name);
    manifest.display_name = display_name;
    manifest.safe_map_name = safe_file_stem_from_display_name(display_name);
    manifest.building_id = building_id;
    manifest.floor_id = floor_id;
    manifest.created_at = utc_timestamp_iso8601();
    manifest.active = true;
    manifest.root = map_root_path(building_id, floor_id, manifest.map_id);
    fill_manifest_paths(manifest);
    return manifest;
  }

  std::optional<MapManifest> read_map_manifest(const fs::path & manifest_path) const
  {
    if (!fs::exists(manifest_path) || !fs::is_regular_file(manifest_path)) {
      return std::nullopt;
    }
    const auto text = read_text_file(manifest_path);
    MapManifest manifest;
    const auto map_id = json_string_value(text, "map_id");
    const auto building_id = json_string_value(text, "building_id");
    const auto floor_id = json_string_value(text, "floor_id");
    if (!map_id || !building_id || !floor_id || !safe_asset_id(*map_id) ||
      !safe_asset_id(*building_id) || !safe_asset_id(*floor_id))
    {
      return std::nullopt;
    }
    manifest.map_id = *map_id;
    manifest.display_name = json_string_value(text, "display_name").value_or(
      json_string_value(text, "map_name").value_or(*map_id));
    manifest.safe_map_name = json_string_value(text, "safe_map_name").value_or(
      safe_file_stem_from_display_name(manifest.display_name));
    manifest.building_id = *building_id;
    manifest.floor_id = *floor_id;
    manifest.created_at = json_string_value(text, "created_at").value_or("");
    manifest.active = json_bool_value(text, "active", false);
    manifest.root = manifest_path.parent_path();
    fill_manifest_paths(manifest);
    return manifest;
  }

  std::vector<MapManifest> read_floor_map_manifests(
    const std::string & building_id,
    const std::string & floor_id,
    const bool migrate_legacy = true)
  {
    if (migrate_legacy) {
      ensure_legacy_floor_map_manifest(building_id, floor_id);
    }
    std::vector<MapManifest> manifests;
    const auto maps_root = floor_maps_root_path(building_id, floor_id);
    if (!fs::exists(maps_root) || !fs::is_directory(maps_root)) {
      return manifests;
    }
    for (const auto & entry : fs::directory_iterator(maps_root)) {
      if (!entry.is_directory()) {
        continue;
      }
      const auto manifest = read_map_manifest(entry.path() / "manifest.json");
      if (manifest) {
        manifests.push_back(*manifest);
      }
    }
    std::sort(manifests.begin(), manifests.end(), [](const auto & lhs, const auto & rhs) {
      if (lhs.active != rhs.active) {
        return lhs.active > rhs.active;
      }
      return lhs.created_at > rhs.created_at;
    });
    return manifests;
  }

  std::vector<MapManifest> read_all_map_manifests(const bool migrate_legacy = true)
  {
    std::vector<MapManifest> maps;
    if (!fs::exists(maps_root_) || !fs::is_directory(maps_root_)) {
      return maps;
    }
    for (const auto & building : fs::directory_iterator(maps_root_)) {
      if (!building.is_directory()) {
        continue;
      }
      const auto building_id = building.path().filename().string();
      for (const auto & floor : fs::directory_iterator(building.path())) {
        if (!floor.is_directory()) {
          continue;
        }
        const auto floor_id = floor.path().filename().string();
        auto floor_maps = read_floor_map_manifests(building_id, floor_id, migrate_legacy);
        maps.insert(maps.end(), floor_maps.begin(), floor_maps.end());
      }
    }
    return maps;
  }

  std::optional<MapManifest> find_map_by_id(const std::string & map_id)
  {
    if (!safe_asset_id(map_id)) {
      return std::nullopt;
    }
    for (const auto & manifest : read_all_map_manifests(true)) {
      if (manifest.map_id == map_id) {
        return manifest;
      }
    }
    return std::nullopt;
  }

  std::optional<MapManifest> find_floor_map_by_name(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & display_name,
    std::string & error)
  {
    std::optional<MapManifest> match;
    for (const auto & manifest : read_floor_map_manifests(building_id, floor_id, true)) {
      if (manifest.display_name != display_name) {
        continue;
      }
      if (match) {
        error = "map_name is ambiguous on this floor; use map_id";
        return std::nullopt;
      }
      match = manifest;
    }
    return match;
  }

  std::optional<MapManifest> active_floor_map(
    const std::string & building_id,
    const std::string & floor_id)
  {
    for (const auto & manifest : read_floor_map_manifests(building_id, floor_id, true)) {
      if (manifest.active) {
        return manifest;
      }
    }
    return std::nullopt;
  }

  std::optional<MapManifest> unique_active_map_manifest()
  {
    std::optional<MapManifest> active;
    for (const auto & manifest : read_all_map_manifests(true)) {
      if (!manifest.active) {
        continue;
      }
      if (active) {
        return std::nullopt;
      }
      active = manifest;
    }
    return active;
  }

  void write_runtime_map_context(
    const MapManifest & manifest,
    const std::string & state,
    const bool confirmed,
    const std::string & message) const
  {
    if (runtime_map_context_file_.empty()) {
      return;
    }
    std::ostringstream body;
    body << std::fixed << std::setprecision(6)
         << "{"
         << "\"schema\":\"njrh.runtime_map_context.v1\","
         << "\"state\":" << json_string(state) << ","
         << "\"confirmed\":" << (confirmed ? "true" : "false") << ","
         << "\"message\":" << json_string(message) << ","
         << "\"map_id\":" << json_string(manifest.map_id) << ","
         << "\"display_name\":" << json_string(manifest.display_name) << ","
         << "\"building_id\":" << json_string(manifest.building_id) << ","
         << "\"floor_id\":" << json_string(manifest.floor_id) << ","
         << "\"updated_at\":" << wall_time_seconds()
         << "}\n";
    write_text_file(runtime_map_context_file_, body.str());
  }

  std::optional<RuntimeMapContext> read_runtime_map_context() const
  {
    if (runtime_map_context_file_.empty()) {
      return std::nullopt;
    }
    const fs::path path(runtime_map_context_file_);
    if (!fs::exists(path) || !fs::is_regular_file(path)) {
      return std::nullopt;
    }
    const auto text = read_text_file(path);
    const auto map_id = json_string_value(text, "map_id");
    const auto building_id = json_string_value(text, "building_id");
    const auto floor_id = json_string_value(text, "floor_id");
    if (!map_id || !building_id || !floor_id || !safe_asset_id(*map_id) ||
      !safe_asset_id(*building_id) || !safe_asset_id(*floor_id))
    {
      return std::nullopt;
    }
    RuntimeMapContext context;
    context.confirmed = json_bool_value(text, "confirmed", false);
    context.state = json_string_value(text, "state").value_or("");
    context.message = json_string_value(text, "message").value_or("");
    context.map_id = *map_id;
    context.display_name = json_string_value(text, "display_name").value_or(*map_id);
    context.building_id = *building_id;
    context.floor_id = *floor_id;
    return context;
  }

  std::optional<MapManifest> confirmed_runtime_map_manifest(
    std::string & unavailable_reason,
    bool & blocked_by_pending_context)
  {
    blocked_by_pending_context = false;
    const auto runtime = runtime_mode_snapshot();
    const auto context = read_runtime_map_context();
    if (context) {
      if (!context->confirmed) {
        if (runtime.navigation_active || runtime.docking_active) {
          blocked_by_pending_context = true;
          unavailable_reason = "runtime map context is not confirmed yet: " +
            context->building_id + "/" + context->floor_id + "/" + context->map_id +
            " state=" + context->state;
        }
        return std::nullopt;
      }
      const auto manifest = find_map_by_id(context->map_id);
      if (!manifest || manifest->building_id != context->building_id ||
        manifest->floor_id != context->floor_id)
      {
        if (runtime.navigation_active || runtime.docking_active) {
          blocked_by_pending_context = true;
          unavailable_reason = "confirmed runtime map context does not match a valid manifest: " +
            context->building_id + "/" + context->floor_id + "/" + context->map_id;
        }
        return std::nullopt;
      }
      return manifest;
    }

    if (runtime.navigation_active || runtime.docking_active) {
      blocked_by_pending_context = true;
      unavailable_reason = "navigation or docking is active but no runtime map context is recorded";
      return std::nullopt;
    }

    if (runtime.mapping_active) {
      return std::nullopt;
    }

    return unique_active_map_manifest();
  }

  std::string yaml_with_image_file(const std::string & yaml_text, const std::string & image_file) const
  {
    std::ostringstream out;
    std::istringstream input(yaml_text);
    std::string line;
    bool replaced = false;
    while (std::getline(input, line)) {
      std::string key;
      std::string value;
      if (!replaced && parse_yaml_key_value(line, key, value) && key == "image") {
        out << "image: " << image_file << "\n";
        replaced = true;
      } else {
        out << line << "\n";
      }
    }
    if (!replaced) {
      out << "image: " << image_file << "\n";
    }
    return out.str();
  }

  void copy_file_if_exists(const fs::path & source, const fs::path & target) const
  {
    if (!fs::exists(source) || !fs::is_regular_file(source)) {
      return;
    }
    fs::create_directories(target.parent_path());
    fs::copy_file(source, target, fs::copy_options::overwrite_existing);
  }

  void copy_yaml_with_image_if_exists(
    const fs::path & source,
    const fs::path & target,
    const std::string & image_file) const
  {
    if (!fs::exists(source) || !fs::is_regular_file(source)) {
      return;
    }
    write_text_file(target, yaml_with_image_file(read_text_file(source), image_file));
  }

  std::string map_manifest_json(const MapManifest & manifest) const
  {
    std::ostringstream out;
    out << "{\n"
        << "  \"map_id\": " << json_string(manifest.map_id) << ",\n"
        << "  \"display_name\": " << json_string(manifest.display_name) << ",\n"
        << "  \"map_name\": " << json_string(manifest.display_name) << ",\n"
        << "  \"safe_map_name\": " << json_string(manifest.safe_map_name) << ",\n"
        << "  \"building_id\": " << json_string(manifest.building_id) << ",\n"
        << "  \"floor_id\": " << json_string(manifest.floor_id) << ",\n"
        << "  \"created_at\": " << json_string(manifest.created_at) << ",\n"
        << "  \"active\": " << (manifest.active ? "true" : "false") << ",\n"
        << "  \"assets\": {\n"
        << "    \"nav_map_yaml\": " << json_string(manifest.nav_map_yaml.string()) << ",\n"
        << "    \"nav_map_pgm\": " << json_string(manifest.nav_map_pgm.string()) << ",\n"
        << "    \"localizer_map_png\": " << json_string(manifest.localizer_map_png.string()) << ",\n"
        << "    \"localizer_params_yaml\": " << json_string(manifest.localizer_params_yaml.string()) << ",\n"
        << "    \"keepout_mask_yaml\": " << json_string(manifest.keepout_mask_yaml.string()) << ",\n"
        << "    \"keepout_mask_pgm\": " << json_string(manifest.keepout_mask_pgm.string()) << ",\n"
        << "    \"speed_mask_yaml\": " << json_string(manifest.speed_mask_yaml.string()) << ",\n"
        << "    \"speed_mask_pgm\": " << json_string(manifest.speed_mask_pgm.string()) << ",\n"
        << "    \"binary_mask_yaml\": " << json_string(manifest.binary_mask_yaml.string()) << ",\n"
        << "    \"binary_mask_pgm\": " << json_string(manifest.binary_mask_pgm.string()) << ",\n"
        << "    \"asset_report_json\": " << json_string(manifest.asset_report_json.string()) << ",\n"
        << "    \"poses_yaml\": " << json_string(manifest.poses_yaml.string()) << "\n"
        << "  }\n"
        << "}\n";
    return out.str();
  }

  void write_map_manifest(const MapManifest & manifest) const
  {
    write_text_file(manifest.manifest_json, map_manifest_json(manifest));
  }

  std::optional<std::pair<std::uint32_t, std::uint32_t>> read_pgm_dimensions(
    const fs::path & pgm_path) const
  {
    std::ifstream file(pgm_path, std::ios::binary);
    if (!file) {
      return std::nullopt;
    }

    auto next_token = [&file]() -> std::optional<std::string> {
        std::string token;
        char c = '\0';
        while (file.get(c)) {
          if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            continue;
          }
          if (c == '#') {
            std::string ignored;
            std::getline(file, ignored);
            continue;
          }
          token.push_back(c);
          break;
        }
        while (file.get(c)) {
          if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            break;
          }
          if (c == '#') {
            std::string ignored;
            std::getline(file, ignored);
            break;
          }
          token.push_back(c);
        }
        if (token.empty()) {
          return std::nullopt;
        }
        return token;
      };

    const auto magic = next_token();
    const auto width = next_token();
    const auto height = next_token();
    if (!magic || (*magic != "P5" && *magic != "P2") || !width || !height) {
      return std::nullopt;
    }

    try {
      const auto parsed_width = static_cast<std::uint32_t>(std::stoul(*width));
      const auto parsed_height = static_cast<std::uint32_t>(std::stoul(*height));
      if (parsed_width == 0U || parsed_height == 0U) {
        return std::nullopt;
      }
      return std::make_pair(parsed_width, parsed_height);
    } catch (...) {
      return std::nullopt;
    }
  }

  std::optional<std::array<double, 3>> parse_yaml_origin(std::string value) const
  {
    value = yaml_scalar_unquote(std::move(value));
    const auto open = value.find('[');
    const auto close = value.rfind(']');
    if (open == std::string::npos || close == std::string::npos || close <= open) {
      return std::nullopt;
    }

    std::array<double, 3> origin{0.0, 0.0, 0.0};
    std::string inner = value.substr(open + 1U, close - open - 1U);
    std::istringstream stream(inner);
    std::string token;
    std::size_t index = 0U;
    while (std::getline(stream, token, ',')) {
      if (index >= origin.size()) {
        return std::nullopt;
      }
      const auto parsed = parse_yaml_double(token);
      if (!parsed) {
        return std::nullopt;
      }
      origin[index++] = *parsed;
    }
    if (index != origin.size()) {
      return std::nullopt;
    }
    return origin;
  }

  fs::path resolve_yaml_image_path(const fs::path & yaml_path, std::string image_value) const
  {
    image_value = yaml_scalar_unquote(std::move(image_value));
    const fs::path image_path(image_value);
    if (image_path.is_absolute()) {
      return image_path;
    }
    return yaml_path.parent_path() / image_path;
  }

  std::optional<MapYamlInfo> read_nav_map_info(const fs::path & nav_map_yaml) const
  {
    if (!fs::exists(nav_map_yaml) || !fs::is_regular_file(nav_map_yaml)) {
      return std::nullopt;
    }

    const auto yaml_text = read_text_file(nav_map_yaml);
    std::optional<double> resolution;
    std::optional<std::array<double, 3>> origin;
    std::optional<fs::path> image_path;
    std::optional<std::uint32_t> width_from_yaml;
    std::optional<std::uint32_t> height_from_yaml;

    std::istringstream input(yaml_text);
    std::string line;
    while (std::getline(input, line)) {
      std::string key;
      std::string value;
      if (!parse_yaml_key_value(line, key, value)) {
        continue;
      }
      if (key == "resolution") {
        resolution = parse_yaml_double(value);
      } else if (key == "origin") {
        origin = parse_yaml_origin(value);
      } else if (key == "image") {
        image_path = resolve_yaml_image_path(nav_map_yaml, value);
      } else if (key == "width") {
        if (const auto parsed = parse_yaml_double(value)) {
          width_from_yaml = static_cast<std::uint32_t>(*parsed);
        }
      } else if (key == "height") {
        if (const auto parsed = parse_yaml_double(value)) {
          height_from_yaml = static_cast<std::uint32_t>(*parsed);
        }
      }
    }

    if (!resolution || !origin || *resolution <= 0.0) {
      return std::nullopt;
    }

    std::optional<std::pair<std::uint32_t, std::uint32_t>> dimensions;
    if (image_path) {
      dimensions = read_pgm_dimensions(*image_path);
    }

    MapYamlInfo info;
    info.resolution = *resolution;
    info.origin = *origin;
    if (dimensions) {
      info.width = dimensions->first;
      info.height = dimensions->second;
    } else if (width_from_yaml && height_from_yaml && *width_from_yaml > 0U && *height_from_yaml > 0U) {
      info.width = *width_from_yaml;
      info.height = *height_from_yaml;
    } else {
      return std::nullopt;
    }
    return info;
  }

  std::string map_info_json(const std::optional<MapYamlInfo> & info) const
  {
    if (!info) {
      return "null";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\"width\":" << info->width
        << ",\"height\":" << info->height
        << ",\"resolution\":" << info->resolution
        << ",\"origin\":[" << info->origin[0] << "," << info->origin[1] << "," << info->origin[2]
        << "]}";
    return out.str();
  }

  bool validate_map_manifest_assets(const MapManifest & manifest, std::string & error) const
  {
    const std::vector<fs::path> required = {
      manifest.nav_map_yaml,
      manifest.nav_map_pgm,
      manifest.localizer_map_png,
      manifest.localizer_params_yaml,
      manifest.keepout_mask_yaml,
      manifest.keepout_mask_pgm,
      manifest.speed_mask_yaml,
      manifest.speed_mask_pgm,
      manifest.binary_mask_yaml,
      manifest.binary_mask_pgm,
      manifest.asset_report_json,
      manifest.poses_yaml
    };
    for (const auto & path : required) {
      if (!fs::exists(path)) {
        error = "map asset is incomplete, missing: " + path.string();
        return false;
      }
    }
    return true;
  }

  void sync_manifest_to_fixed_entry(
    const MapManifest & manifest,
    const fs::path & fixed_root,
    const bool include_manifest) const
  {
    copy_file_if_exists(manifest.nav_map_pgm, fixed_root / "nav" / "nav_map.pgm");
    copy_yaml_with_image_if_exists(
      manifest.nav_map_yaml, fixed_root / "nav" / "nav_map.yaml", "nav_map.pgm");
    copy_file_if_exists(manifest.localizer_map_png, fixed_root / "localizer" / "localizer_map.png");
    copy_yaml_with_image_if_exists(
      manifest.localizer_params_yaml,
      fixed_root / "localizer" / "localizer_params.yaml",
      "localizer_map.png");
    copy_file_if_exists(manifest.keepout_mask_pgm, fixed_root / "filters" / "keepout_mask.pgm");
    copy_yaml_with_image_if_exists(
      manifest.keepout_mask_yaml, fixed_root / "filters" / "keepout_mask.yaml", "keepout_mask.pgm");
    copy_file_if_exists(manifest.speed_mask_pgm, fixed_root / "filters" / "speed_mask.pgm");
    copy_yaml_with_image_if_exists(
      manifest.speed_mask_yaml, fixed_root / "filters" / "speed_mask.yaml", "speed_mask.pgm");
    copy_file_if_exists(manifest.binary_mask_pgm, fixed_root / "filters" / "binary_mask.pgm");
    copy_yaml_with_image_if_exists(
      manifest.binary_mask_yaml, fixed_root / "filters" / "binary_mask.yaml", "binary_mask.pgm");
    copy_file_if_exists(manifest.asset_report_json, fixed_root / "reports" / "asset_report.json");
    copy_file_if_exists(manifest.poses_yaml, fixed_root / "poses.yaml");
    if (include_manifest) {
      write_text_file(fixed_root / "manifest.json", map_manifest_json(manifest));
    }
  }

  void remove_current_map_entry(const std::string & building_id, const std::string & floor_id) const
  {
    const auto floor_root = floor_root_path(building_id, floor_id);
    const auto current_root = floor_current_root_path(building_id, floor_id);
    if (current_root.filename() != "current" || current_root.parent_path() != floor_root) {
      throw std::runtime_error("refusing unsafe current map reset path: " + current_root.string());
    }

    std::error_code ec;
    fs::remove_all(current_root, ec);
    if (!ec) {
      return;
    }

    const auto remove_error = ec.message();
    std::error_code status_ec;
    const auto status = fs::symlink_status(current_root, status_ec);
    if (status_ec) {
      throw std::runtime_error(
              "failed to reset current map entry: " + current_root.string() +
              " remove_all=" + remove_error + " status=" + status_ec.message());
    }
    if (!fs::exists(status)) {
      return;
    }

    // Old dashboard/test runs may leave current/ as a non-empty root-owned directory.
    // Renaming only needs write permission on the floor directory, then the new
    // current/ can be created by the API runtime user.
    fs::path stale_root;
    for (int index = 0; index < 100; ++index) {
      stale_root = floor_root /
        (".stale_current_" + utc_timestamp_compact() + "_" + std::to_string(::getpid()) + "_" +
        std::to_string(index));
      if (!fs::exists(stale_root)) {
        break;
      }
    }

    std::error_code rename_ec;
    fs::rename(current_root, stale_root, rename_ec);
    if (rename_ec) {
      throw std::runtime_error(
              "failed to reset current map entry: " + current_root.string() +
              " remove_all=" + remove_error + " rename=" + rename_ec.message());
    }

    std::error_code cleanup_ec;
    fs::remove_all(stale_root, cleanup_ec);
    if (cleanup_ec) {
      RCLCPP_WARN(
        get_logger(),
        "quarantined stale current map entry at %s after reset; cleanup skipped: %s",
        stale_root.string().c_str(), cleanup_ec.message().c_str());
    }
  }

  void activate_map_manifest(MapManifest manifest)
  {
    std::string error;
    if (!validate_map_manifest_assets(manifest, error)) {
      throw std::runtime_error(error);
    }
    for (auto other : read_floor_map_manifests(manifest.building_id, manifest.floor_id, false)) {
      if (other.map_id == manifest.map_id) {
        continue;
      }
      if (other.active) {
        other.active = false;
        write_map_manifest(other);
      }
    }

    manifest.active = true;
    write_map_manifest(manifest);

    const auto current_root = floor_current_root_path(manifest.building_id, manifest.floor_id);
    remove_current_map_entry(manifest.building_id, manifest.floor_id);
    sync_manifest_to_fixed_entry(manifest, current_root, true);

    // Keep the historical fixed role files in the floor root as a compatibility shim for older tools.
    sync_manifest_to_fixed_entry(manifest, floor_root_path(manifest.building_id, manifest.floor_id), false);
  }

  void clear_fixed_floor_entries(const std::string & building_id, const std::string & floor_id) const
  {
    const auto floor_root = floor_root_path(building_id, floor_id);
    std::error_code ec;
    remove_current_map_entry(building_id, floor_id);
    fs::remove_all(floor_root / "nav", ec);
    fs::remove_all(floor_root / "localizer", ec);
    fs::remove_all(floor_root / "filters", ec);
    fs::remove_all(floor_root / "reports", ec);
    fs::remove(floor_root / "poses.yaml", ec);
  }

  void ensure_legacy_floor_map_manifest(
    const std::string & building_id,
    const std::string & floor_id)
  {
    if (!safe_asset_id(building_id) || !safe_asset_id(floor_id)) {
      return;
    }
    const auto maps_root = floor_maps_root_path(building_id, floor_id);
    if (fs::exists(maps_root) && fs::is_directory(maps_root)) {
      for (const auto & entry : fs::directory_iterator(maps_root)) {
        if (entry.is_directory() && fs::exists(entry.path() / "manifest.json")) {
          return;
        }
      }
    }

    const auto floor_root = floor_root_path(building_id, floor_id);
    const auto legacy_nav_yaml = floor_root / "nav" / "nav_map.yaml";
    const auto legacy_nav_pgm = floor_root / "nav" / "nav_map.pgm";
    const auto legacy_localizer_png = floor_root / "localizer" / "localizer_map.png";
    const auto legacy_localizer_params = floor_root / "localizer" / "localizer_params.yaml";
    const auto legacy_report = floor_root / "reports" / "asset_report.json";
    const auto legacy_poses = floor_root / "poses.yaml";
    if (!fs::exists(legacy_nav_yaml) || !fs::exists(legacy_nav_pgm) ||
      !fs::exists(legacy_localizer_png) || !fs::exists(legacy_localizer_params))
    {
      return;
    }

    MapManifest manifest;
    manifest.map_id = "legacy_" + fixed_hex(fnv1a64(floor_root.string()), 12);
    manifest.display_name = "legacy_" + floor_id;
    manifest.safe_map_name = safe_file_stem_from_display_name(manifest.display_name);
    manifest.building_id = building_id;
    manifest.floor_id = floor_id;
    manifest.created_at = utc_timestamp_iso8601();
    manifest.active = true;
    manifest.root = map_root_path(building_id, floor_id, manifest.map_id);
    fill_manifest_paths(manifest);

    copy_file_if_exists(legacy_nav_pgm, manifest.nav_map_pgm);
    copy_yaml_with_image_if_exists(
      legacy_nav_yaml, manifest.nav_map_yaml, manifest.nav_map_pgm.filename().string());
    copy_file_if_exists(legacy_localizer_png, manifest.localizer_map_png);
    copy_yaml_with_image_if_exists(
      legacy_localizer_params,
      manifest.localizer_params_yaml,
      manifest.localizer_map_png.filename().string());
    copy_file_if_exists(floor_root / "filters" / "keepout_mask.pgm", manifest.keepout_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "keepout_mask.yaml",
      manifest.keepout_mask_yaml,
      "keepout_mask.pgm");
    copy_file_if_exists(floor_root / "filters" / "speed_mask.pgm", manifest.speed_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "speed_mask.yaml",
      manifest.speed_mask_yaml,
      "speed_mask.pgm");
    copy_file_if_exists(floor_root / "filters" / "binary_mask.pgm", manifest.binary_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "binary_mask.yaml",
      manifest.binary_mask_yaml,
      "binary_mask.pgm");
    copy_file_if_exists(legacy_report, manifest.asset_report_json);
    if (!fs::exists(manifest.asset_report_json)) {
      write_text_file(manifest.asset_report_json, "{}\n");
    }
    copy_file_if_exists(legacy_poses, manifest.poses_yaml);
    if (!fs::exists(manifest.poses_yaml)) {
      write_text_file(manifest.poses_yaml, "poses: []\n");
    }
    std::string error;
    if (validate_map_manifest_assets(manifest, error)) {
      write_map_manifest(manifest);
      activate_map_manifest(manifest);
    } else {
      manifest.active = false;
      write_map_manifest(manifest);
      RCLCPP_WARN(get_logger(), "legacy map manifest created but not activated: %s", error.c_str());
    }
  }

  HttpResponse handle_maps()
  {
    std::ostringstream body;
    body << "{\"ok\":true,\"runtime_maps\":[";
    bool first = true;
    if (fs::exists(runtime_maps_dir_)) {
      for (const auto & entry : fs::directory_iterator(runtime_maps_dir_)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".yaml") {
          continue;
        }
        const auto stem = entry.path().stem().string();
        if (stem.size() >= 10 && stem.substr(stem.size() - 10) == ".localizer") {
          continue;
        }
        const auto pgm = entry.path().parent_path() / (stem + ".pgm");
        if (!fs::exists(pgm)) {
          continue;
        }
        if (!first) {
          body << ",";
        }
        first = false;
        const auto map_info = read_nav_map_info(entry.path());
        body << "{\"name\":" << json_string(stem)
             << ",\"yaml\":" << json_string(entry.path().string())
             << ",\"pgm\":" << json_string(pgm.string())
             << ",\"map_info\":" << map_info_json(map_info) << "}";
      }
    }

    const auto manifests = read_all_map_manifests(true);
    body << "],\"floor_maps\":[";
    first = true;
    for (const auto & manifest : manifests) {
      if (!first) {
        body << ",";
      }
      first = false;
      body << "{\"map_id\":" << json_string(manifest.map_id)
           << ",\"display_name\":" << json_string(manifest.display_name)
           << ",\"map_name\":" << json_string(manifest.display_name)
           << ",\"building_id\":" << json_string(manifest.building_id)
           << ",\"floor_id\":" << json_string(manifest.floor_id)
           << ",\"active\":" << (manifest.active ? "true" : "false")
           << ",\"nav_map_yaml\":" << json_string(manifest.nav_map_yaml.string())
           << ",\"localizer_map_png\":" << json_string(manifest.localizer_map_png.string())
           << ",\"map_info\":" << map_info_json(read_nav_map_info(manifest.nav_map_yaml))
           << ",\"manifest_json\":" << json_string(manifest.manifest_json.string()) << "}";
    }

    body << "],\"floors\":[";
    first = true;
    if (fs::exists(maps_root_)) {
      for (const auto & building : fs::directory_iterator(maps_root_)) {
        if (!building.is_directory()) {
          continue;
        }
        for (const auto & floor : fs::directory_iterator(building.path())) {
          if (!floor.is_directory()) {
            continue;
          }
          const auto building_id = building.path().filename().string();
          const auto floor_id = floor.path().filename().string();
          const auto active = active_floor_map(building_id, floor_id);
          const auto current_root = floor.path() / "current";
          const auto root = fs::exists(current_root / "nav" / "nav_map.yaml") ? current_root : floor.path();
          const auto nav_yaml = root / "nav" / "nav_map.yaml";
          const auto nav_pgm = root / "nav" / "nav_map.pgm";
          const auto localizer_png = root / "localizer" / "localizer_map.png";
          const auto localizer_params = root / "localizer" / "localizer_params.yaml";
          if (!fs::exists(nav_yaml) || !fs::exists(nav_pgm) || !fs::exists(localizer_png) ||
            !fs::exists(localizer_params))
          {
            continue;
          }
          if (!first) {
            body << ",";
          }
          first = false;
          body << "{\"building_id\":" << json_string(building_id)
               << ",\"floor_id\":" << json_string(floor_id)
               << ",\"active_map_id\":" << json_string(active ? active->map_id : "")
               << ",\"active_display_name\":" << json_string(active ? active->display_name : "")
               << ",\"nav_map_yaml\":" << json_string(nav_yaml.string())
               << ",\"nav_map_pgm\":" << json_string(nav_pgm.string())
               << ",\"localizer_map_png\":" << json_string(localizer_png.string())
               << ",\"localizer_params_yaml\":" << json_string(localizer_params.string())
               << ",\"map_info\":" << map_info_json(read_nav_map_info(nav_yaml)) << "}";
        }
      }
    }
    body << "]}";
    return {200, "application/json", body.str()};
  }

  HttpResponse handle_delete_map(const std::string & body)
  {
    const auto map_id = json_string_value(body, "map_id");
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");

    if (!map_id) {
      if (building_id || floor_id) {
        return {
          400,
          "application/json",
          error_json("delete by map_id only; refusing to delete non-empty building/floor assets")
        };
      }
      return {400, "application/json", error_json("map_id is required")};
    }
    if (!safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }

    const auto manifest = find_map_by_id(*map_id);
    if (!manifest) {
      return {404, "application/json", error_json("map_id not found: " + *map_id)};
    }

    std::uintmax_t entries_deleted = 0;
    try {
      std::error_code ec;
      entries_deleted = fs::remove_all(manifest->root, ec);
      if (ec) {
        return {500, "application/json", error_json("failed to delete map asset: " + manifest->root.string())};
      }
      if (manifest->active) {
        auto remaining = read_floor_map_manifests(manifest->building_id, manifest->floor_id, false);
        if (!remaining.empty()) {
          activate_map_manifest(remaining.front());
        } else {
          clear_fixed_floor_entries(manifest->building_id, manifest->floor_id);
        }
      }
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"deleted\":" << (entries_deleted > 0U ? "true" : "false") << ","
             << "\"map_id\":" << json_string(manifest->map_id) << ","
             << "\"display_name\":" << json_string(manifest->display_name) << ","
             << "\"building_id\":" << json_string(manifest->building_id) << ","
             << "\"floor_id\":" << json_string(manifest->floor_id) << ","
             << "\"active_deleted\":" << (manifest->active ? "true" : "false") << ","
             << "\"entries_deleted\":" << entries_deleted << "}";
    return {200, "application/json", response.str()};
  }

  struct ManifestLookupResult
  {
    bool ok{false};
    MapManifest manifest;
    HttpResponse error{400, "application/json", "{}"};
  };

  std::optional<std::string> query_value(const HttpRequest & request, const std::string & key) const
  {
    const auto it = request.query.find(key);
    if (it == request.query.end() || it->second.empty()) {
      return std::nullopt;
    }
    return it->second;
  }

  ManifestLookupResult resolve_map_manifest_from_query(const HttpRequest & request)
  {
    const auto requested_building_id = query_value(request, "building_id");
    const auto requested_floor_id = query_value(request, "floor_id");
    const auto requested_map_id = query_value(request, "map_id");
    const auto requested_map_name = query_value(request, "map_name").value_or(
      query_value(request, "display_name").value_or(""));

    ManifestLookupResult result;
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      result.error = {400, "application/json", error_json("valid building_id is required")};
      return result;
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      result.error = {400, "application/json", error_json("valid floor_id is required")};
      return result;
    }

    std::optional<MapManifest> manifest;
    if (requested_map_id) {
      if (!safe_asset_id(*requested_map_id)) {
        result.error = {400, "application/json", error_json("valid map_id is required")};
        return result;
      }
      manifest = find_map_by_id(*requested_map_id);
      if (!manifest) {
        result.error = {404, "application/json", error_json("map_id not found: " + *requested_map_id)};
        return result;
      }
      if (requested_building_id && *requested_building_id != manifest->building_id) {
        result.error = {400, "application/json", error_json("map_id does not belong to requested building")};
        return result;
      }
      if (requested_floor_id && *requested_floor_id != manifest->floor_id) {
        result.error = {400, "application/json", error_json("map_id does not belong to requested floor")};
        return result;
      }
    } else {
      if (!requested_building_id) {
        result.error = {400, "application/json", error_json("valid building_id is required")};
        return result;
      }
      if (!requested_floor_id) {
        result.error = {400, "application/json", error_json("valid floor_id is required")};
        return result;
      }
      if (!requested_map_name.empty()) {
        if (!valid_display_map_name(requested_map_name)) {
          result.error = {400, "application/json", error_json("valid map_name is required")};
          return result;
        }
        std::string error;
        manifest = find_floor_map_by_name(*requested_building_id, *requested_floor_id, requested_map_name, error);
        if (!manifest) {
          result.error = {
            error.empty() ? 404 : 400,
            "application/json",
            error_json(error.empty() ? "map_name not found: " + requested_map_name : error)
          };
          return result;
        }
      } else {
        manifest = active_floor_map(*requested_building_id, *requested_floor_id);
        if (!manifest) {
          result.error = {
            404,
            "application/json",
            error_json("active map not found for floor: " + *requested_building_id + "/" + *requested_floor_id)
          };
          return result;
        }
      }
    }

    result.ok = true;
    result.manifest = *manifest;
    return result;
  }

  fs::path keepout_semantic_json_path(const MapManifest & manifest) const
  {
    return manifest.root / "filters" / "keepout_semantic_layer.json";
  }

  std::string read_optional_text_file(const fs::path & path) const
  {
    if (!fs::exists(path) || !fs::is_regular_file(path)) {
      return "";
    }
    return read_text_file(path);
  }

  std::string json_raw_or_null(const std::string & text) const
  {
    const auto stripped = trim(text);
    if (stripped.empty()) {
      return "null";
    }
    if (stripped.front() == '{' || stripped.front() == '[') {
      return stripped;
    }
    return "null";
  }

  std::string keepout_semantic_payload_json(const std::string & semantic_json) const
  {
    const auto keepout = json_object_value(semantic_json, "keepout");
    if (keepout) {
      return *keepout;
    }
    return json_raw_or_null(semantic_json);
  }

  std::string poses_json_array(const std::vector<StoredPose> & poses) const
  {
    std::ostringstream response;
    response << std::fixed << std::setprecision(6);
    response << "[";
    bool first = true;
    for (const auto & pose : poses) {
      if (!first) {
        response << ",";
      }
      first = false;
      response << "{\"pose_id\":" << json_string(pose.id)
               << ",\"id\":" << json_string(pose.id)
               << ",\"type\":" << json_string(pose.type.empty() ? "delivery_point" : pose.type)
               << ",\"name\":" << json_string(pose.name.empty() ? pose.id : pose.name)
               << ",\"x\":" << pose.x
               << ",\"y\":" << pose.y
               << ",\"yaw\":" << normalize_angle(pose.yaw) << "}";
    }
    response << "]";
    return response.str();
  }

  std::string keepout_filter_json(const MapManifest & manifest) const
  {
    const auto semantic_path = keepout_semantic_json_path(manifest);
    const auto mask_yaml = read_optional_text_file(manifest.keepout_mask_yaml);
    const auto semantic_json = read_optional_text_file(semantic_path);
    const auto keepout_payload = keepout_semantic_payload_json(semantic_json);
    std::ostringstream response;
    response << "{"
             << "\"filter\":\"keepout\","
             << "\"keepout_mask_yaml\":" << json_string(manifest.keepout_mask_yaml.string()) << ","
             << "\"keepout_mask_pgm\":" << json_string(manifest.keepout_mask_pgm.string()) << ","
             << "\"mask_yaml_exists\":" << (fs::exists(manifest.keepout_mask_yaml) ? "true" : "false") << ","
             << "\"mask_pgm_exists\":" << (fs::exists(manifest.keepout_mask_pgm) ? "true" : "false") << ","
             << "\"yaml\":" << json_string(mask_yaml) << ","
             << "\"mask_yaml\":" << json_string(mask_yaml) << ","
             << "\"semantic_json_path\":" << json_string(semantic_path.string()) << ","
             << "\"semantic_json_exists\":" << (fs::exists(semantic_path) ? "true" : "false") << ","
             << "\"semantic_json\":" << json_string(semantic_json) << ","
             << "\"raw_json\":" << json_string(semantic_json) << ","
             << "\"payload\":" << json_raw_or_null(semantic_json) << ","
             << "\"keepout_payload\":" << keepout_payload << ","
             << "\"keepout\":" << keepout_payload
             << "}";
    return response.str();
  }

  HttpResponse handle_get_keepout_filter(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }
    try {
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
               << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
               << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
               << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
               << "\"filter\":\"keepout\","
               << "\"keepout\":" << keepout_filter_json(lookup.manifest) << ","
               << "\"filters\":{\"keepout\":" << keepout_filter_json(lookup.manifest) << "}"
               << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_get_semantic_layer(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }

    try {
      const auto poses = read_floor_poses(lookup.manifest.poses_yaml);
      const auto keepout_filter = keepout_filter_json(lookup.manifest);
      const auto keepout_payload = keepout_semantic_payload_json(
        read_optional_text_file(keepout_semantic_json_path(lookup.manifest)));
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"schema\":\"njrh.semantic_layer.v1\","
               << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
               << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
               << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
               << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(lookup.manifest.poses_yaml.string()) << ","
               << "\"poses\":" << poses_json_array(poses) << ","
               << "\"filters\":{\"keepout\":" << keepout_filter << "},"
               << "\"keepout_filter\":" << keepout_filter << ","
               << "\"keepout\":" << keepout_payload
               << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_get_poses(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }

    std::vector<StoredPose> poses;
    try {
      poses = read_floor_poses(lookup.manifest.poses_yaml);
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    std::ostringstream response;
    response << std::fixed << std::setprecision(6);
    response << "{\"ok\":true,"
             << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
             << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
             << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
             << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
             << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
             << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
             << "\"poses_yaml\":" << json_string(lookup.manifest.poses_yaml.string()) << ","
             << "\"poses\":" << poses_json_array(poses) << "}";
    return {200, "application/json", response.str()};
  }

  bool safe_map_name(const std::string & name) const
  {
    if (name.empty() || name == "." || name.size() > 128U) {
      return false;
    }
    if (name.find("..") != std::string::npos || name.find('/') != std::string::npos ||
      name.find('\\') != std::string::npos)
    {
      return false;
    }
    return std::all_of(name.begin(), name.end(), [](const unsigned char c) {
      return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.';
    });
  }

  std::vector<fs::path> runtime_map_asset_paths(const std::string & map_name) const
  {
    const fs::path runtime_dir(runtime_maps_dir_);
    return {
      runtime_dir / (map_name + ".yaml"),
      runtime_dir / (map_name + ".pgm"),
      runtime_dir / (map_name + ".png"),
      runtime_dir / (map_name + ".localizer.yaml"),
      runtime_dir / (map_name + ".localizer.png"),
      runtime_dir / (map_name + ".localizer.pgm"),
      runtime_dir / (map_name + ".meta.json"),
      runtime_dir / (map_name + ".metadata.json")
    };
  }

  bool safe_pose_id(const std::string & pose_id) const
  {
    if (pose_id.empty() || pose_id.size() > 128U || pose_id.find("..") != std::string::npos ||
      pose_id.find('/') != std::string::npos || pose_id.find('\\') != std::string::npos)
    {
      return false;
    }
    return std::all_of(pose_id.begin(), pose_id.end(), [](const unsigned char c) {
      return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == ':';
    });
  }

  bool safe_asset_id(const std::string & id) const
  {
    if (id.empty() || id.size() > 128U || id.find("..") != std::string::npos ||
      id.find('/') != std::string::npos || id.find('\\') != std::string::npos)
    {
      return false;
    }
    return std::all_of(id.begin(), id.end(), [](const unsigned char c) {
      return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.';
    });
  }

  fs::path floor_root_path(const std::string & building_id, const std::string & floor_id) const
  {
    return fs::path(maps_root_) / building_id / floor_id;
  }

  struct FloorAssetPaths
  {
    fs::path root;
    fs::path nav_map_yaml;
    fs::path localizer_map_png;
    fs::path localizer_params_yaml;
  };

  bool resolve_floor_asset_paths(
    const std::string & building_id,
    const std::string & floor_id,
    FloorAssetPaths & assets,
    std::string & error) const
  {
    if (!safe_asset_id(building_id) || !safe_asset_id(floor_id)) {
      error = "building_id and floor_id must be safe asset ids";
      return false;
    }

    const auto floor_root = floor_root_path(building_id, floor_id);
    const auto current_root = floor_current_root_path(building_id, floor_id);
    assets.root = fs::exists(current_root / "nav" / "nav_map.yaml") ? current_root : floor_root;
    assets.nav_map_yaml = assets.root / "nav" / "nav_map.yaml";
    assets.localizer_map_png = assets.root / "localizer" / "localizer_map.png";
    assets.localizer_params_yaml = assets.root / "localizer" / "localizer_params.yaml";

    const std::vector<fs::path> required = {
      assets.nav_map_yaml,
      assets.root / "nav" / "nav_map.pgm",
      assets.localizer_map_png,
      assets.localizer_params_yaml,
      assets.root / "filters" / "keepout_mask.yaml",
      assets.root / "filters" / "keepout_mask.pgm",
      assets.root / "filters" / "speed_mask.yaml",
      assets.root / "filters" / "speed_mask.pgm",
      assets.root / "filters" / "binary_mask.yaml",
      assets.root / "filters" / "binary_mask.pgm",
      assets.root / "reports" / "asset_report.json",
      assets.root / "poses.yaml"
    };
    for (const auto & path : required) {
      if (!fs::exists(path)) {
        error = "floor asset is incomplete, missing: " + path.string();
        return false;
      }
    }
    return true;
  }

  fs::path poses_yaml_path(const std::string & building_id, const std::string & floor_id) const
  {
    const auto current_poses = floor_current_root_path(building_id, floor_id) / "poses.yaml";
    if (fs::exists(current_poses)) {
      return current_poses;
    }
    return floor_root_path(building_id, floor_id) / "poses.yaml";
  }

  std::string read_text_file(const fs::path & path) const
  {
    std::ifstream file(path);
    if (!file) {
      throw std::runtime_error("failed to open file for reading: " + path.string());
    }
    std::ostringstream data;
    data << file.rdbuf();
    return data.str();
  }

  std::string yaml_scalar_unquote(std::string value) const
  {
    value = trim(value);
    const auto comment = value.find(" #");
    if (comment != std::string::npos) {
      value = trim(value.substr(0, comment));
    }
    if (value.size() >= 2U &&
      ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\'')))
    {
      value = value.substr(1, value.size() - 2U);
    }
    return value;
  }

  std::optional<double> parse_yaml_double(std::string value) const
  {
    value = yaml_scalar_unquote(std::move(value));
    if (value.empty()) {
      return std::nullopt;
    }
    try {
      std::size_t idx = 0;
      const double parsed = std::stod(value, &idx);
      if (idx == 0U || !std::isfinite(parsed)) {
        return std::nullopt;
      }
      return parsed;
    } catch (...) {
      return std::nullopt;
    }
  }

  bool parse_yaml_key_value(
    const std::string & line,
    std::string & key,
    std::string & value) const
  {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      return false;
    }
    key = trim(line.substr(0, colon));
    value = trim(line.substr(colon + 1));
    return !key.empty();
  }

  void apply_pose_yaml_field(
    StoredPose & pose,
    bool & have_x,
    bool & have_y,
    bool & have_yaw,
    const std::string & key,
    const std::string & value) const
  {
    if (key == "id" || key == "pose_id") {
      pose.id = yaml_scalar_unquote(value);
    } else if (key == "name") {
      pose.name = yaml_scalar_unquote(value);
    } else if (key == "type") {
      pose.type = yaml_scalar_unquote(value);
    } else if (key == "x") {
      const auto parsed = parse_yaml_double(value);
      if (parsed) {
        pose.x = *parsed;
        have_x = true;
      }
    } else if (key == "y") {
      const auto parsed = parse_yaml_double(value);
      if (parsed) {
        pose.y = *parsed;
        have_y = true;
      }
    } else if (key == "yaw" || key == "theta" || key == "heading") {
      const auto parsed = parse_yaml_double(value);
      if (parsed) {
        pose.yaw = *parsed;
        have_yaw = true;
      }
    }
  }

  std::vector<StoredPose> read_floor_poses(const fs::path & path) const
  {
    std::vector<StoredPose> poses;
    if (!fs::exists(path)) {
      return poses;
    }
    const auto text = read_text_file(path);
    std::istringstream input(text);
    std::string line;
    StoredPose current;
    bool in_pose = false;
    bool have_x = false;
    bool have_y = false;
    bool have_yaw = false;

    const auto flush_pose = [&]() {
      if (in_pose && !current.id.empty() && have_x && have_y && have_yaw) {
        if (current.name.empty()) {
          current.name = current.id;
        }
        if (current.type.empty()) {
          current.type = "delivery_point";
        }
        poses.push_back(current);
      }
      current = StoredPose{};
      in_pose = false;
      have_x = false;
      have_y = false;
      have_yaw = false;
    };

    while (std::getline(input, line)) {
      auto stripped = trim(line);
      if (stripped.empty() || stripped.front() == '#') {
        continue;
      }
      if (stripped.rfind("- ", 0) == 0) {
        flush_pose();
        in_pose = true;
        stripped = trim(stripped.substr(2));
        if (!stripped.empty()) {
          std::string key;
          std::string value;
          if (parse_yaml_key_value(stripped, key, value)) {
            apply_pose_yaml_field(current, have_x, have_y, have_yaw, key, value);
          }
        }
        continue;
      }
      if (!in_pose) {
        continue;
      }
      std::string key;
      std::string value;
      if (parse_yaml_key_value(stripped, key, value)) {
        apply_pose_yaml_field(current, have_x, have_y, have_yaw, key, value);
      }
    }
    flush_pose();
    return poses;
  }

  void write_floor_poses(const fs::path & path, const std::vector<StoredPose> & poses) const
  {
    std::ostringstream yaml;
    yaml << std::fixed << std::setprecision(6);
    yaml << "poses:\n";
    for (const auto & pose : poses) {
      yaml << "  - id: " << json_string(pose.id) << "\n";
      yaml << "    type: " << json_string(pose.type.empty() ? "delivery_point" : pose.type) << "\n";
      yaml << "    name: " << json_string(pose.name.empty() ? pose.id : pose.name) << "\n";
      yaml << "    x: " << pose.x << "\n";
      yaml << "    y: " << pose.y << "\n";
      yaml << "    yaw: " << normalize_angle(pose.yaw) << "\n";
    }
    write_text_file(path, yaml.str());
  }

  std::optional<StoredPose> find_floor_pose(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & pose_id) const
  {
    const auto path = poses_yaml_path(building_id, floor_id);
    if (!fs::exists(path)) {
      return std::nullopt;
    }
    for (const auto & pose : read_floor_poses(path)) {
      if (pose.id == pose_id) {
        return pose;
      }
    }
    return std::nullopt;
  }

  std::optional<fs::path> newest_png_in_directory(const fs::path & directory) const
  {
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
      return std::nullopt;
    }

    std::optional<fs::path> newest;
    fs::file_time_type newest_time{};
    for (const auto & entry : fs::directory_iterator(directory)) {
      if (!entry.is_regular_file() || entry.path().extension() != ".png") {
        continue;
      }
      const auto stem = entry.path().stem().string();
      if (stem.size() >= 10U && stem.substr(stem.size() - 10U) == ".localizer") {
        continue;
      }
      const auto write_time = entry.last_write_time();
      if (!newest || write_time > newest_time) {
        newest = entry.path();
        newest_time = write_time;
      }
    }
    return newest;
  }

  std::optional<fs::path> newest_floor_localizer_png() const
  {
    if (!fs::exists(maps_root_) || !fs::is_directory(maps_root_)) {
      return std::nullopt;
    }

    std::optional<fs::path> newest;
    fs::file_time_type newest_time{};
    for (const auto & entry : fs::recursive_directory_iterator(maps_root_)) {
      if (!entry.is_regular_file() || entry.path().filename() != "localizer_map.png") {
        continue;
      }
      const auto write_time = entry.last_write_time();
      if (!newest || write_time > newest_time) {
        newest = entry.path();
        newest_time = write_time;
      }
    }
    return newest;
  }

  std::optional<fs::path> resolve_mapping_2d_png(const HttpRequest & request) const
  {
    const auto name_it = request.query.find("name");
    if (name_it != request.query.end()) {
      if (!safe_map_name(name_it->second)) {
        return std::nullopt;
      }
      const auto candidate = fs::path(runtime_maps_dir_) / (name_it->second + ".png");
      if (fs::exists(candidate) && fs::is_regular_file(candidate)) {
        return candidate;
      }
      return std::nullopt;
    }

    const auto latest_runtime = newest_png_in_directory(runtime_maps_dir_);
    if (latest_runtime) {
      return latest_runtime;
    }
    return newest_floor_localizer_png();
  }

  HttpResponse handle_saved_mapping_2d_map_png(const HttpRequest & request)
  {
    const auto png_path = resolve_mapping_2d_png(request);
    if (!png_path) {
      return {
        404,
        "application/json",
        error_json("no saved 2D PNG map is available; save a slam_toolbox 2D map first")
      };
    }

    std::ifstream file(*png_path, std::ios::binary);
    if (!file) {
      return {404, "application/json", error_json("failed to open 2D PNG map: " + png_path->string())};
    }
    std::ostringstream data;
    data << file.rdbuf();
    return {200, "image/png", data.str()};
  }

  double map_origin_yaw(const nav_msgs::msg::OccupancyGrid & map) const
  {
    const auto & q = map.info.origin.orientation;
    return quaternion_yaw(q.x, q.y, q.z, q.w);
  }

  double quaternion_yaw(const double x, const double y, const double z, const double w) const
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::vector<std::uint8_t> occupancy_grid_to_image_pixels(const nav_msgs::msg::OccupancyGrid & map) const
  {
    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);
    for (std::uint32_t y = 0; y < height; ++y) {
      const std::uint32_t src_y = height - 1U - y;
      for (std::uint32_t x = 0; x < width; ++x) {
        const std::size_t src = static_cast<std::size_t>(src_y) * width + x;
        const std::size_t dst = static_cast<std::size_t>(y) * width + x;
        pixels[dst] = occupancy_to_gray(static_cast<int>(map.data[src]));
      }
    }
    return pixels;
  }

  std::string map_yaml_text(
    const std::string & image_name,
    const nav_msgs::msg::OccupancyGrid & map) const
  {
    std::ostringstream yaml;
    yaml << std::fixed << std::setprecision(6);
    yaml << "image: " << image_name << "\n";
    yaml << "resolution: " << map.info.resolution << "\n";
    yaml << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", "
         << map_origin_yaw(map) << "]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml << "mode: trinary\n";
    return yaml.str();
  }

  void write_text_file(const fs::path & path, const std::string & text) const
  {
    fs::create_directories(path.parent_path());
    std::ofstream file(path);
    if (!file) {
      throw std::runtime_error("failed to open file for writing: " + path.string());
    }
    file << text;
  }

  void write_binary_file(const fs::path & path, const std::vector<std::uint8_t> & data) const
  {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("failed to open file for writing: " + path.string());
    }
    file.write(reinterpret_cast<const char *>(data.data()), static_cast<std::streamsize>(data.size()));
  }

  void write_binary_file(const fs::path & path, const std::string & data) const
  {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("failed to open file for writing: " + path.string());
    }
    file.write(data.data(), static_cast<std::streamsize>(data.size()));
  }

  void write_pgm_file(
    const fs::path & path,
    const std::uint32_t width,
    const std::uint32_t height,
    const std::vector<std::uint8_t> & pixels) const
  {
    std::vector<std::uint8_t> payload;
    const auto header = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
    payload.insert(payload.end(), header.begin(), header.end());
    payload.insert(payload.end(), pixels.begin(), pixels.end());
    write_binary_file(path, payload);
  }

  void write_neutral_filter_assets(
    const fs::path & filters_dir,
    const nav_msgs::msg::OccupancyGrid & map) const
  {
    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    // map_server trinary mode loads white/free PGM pixels as OccupancyGrid 0, the no-effect
    // value for keepout/speed/binary costmap filters.
    const std::vector<std::uint8_t> neutral(static_cast<std::size_t>(width) * height, 254U);
    for (const auto & stem : {"keepout_mask", "speed_mask", "binary_mask"}) {
      write_pgm_file(filters_dir / (std::string(stem) + ".pgm"), width, height, neutral);
      write_text_file(filters_dir / (std::string(stem) + ".yaml"), map_yaml_text(std::string(stem) + ".pgm", map));
    }
  }

  void write_asset_report(
    const MapManifest & manifest,
    const nav_msgs::msg::OccupancyGrid & map) const
  {
    std::ostringstream report;
    report << "{\n";
    report << "  \"producer\": \"robot_api_server_slam_toolbox_save\",\n";
    report << "  \"map_id\": " << json_string(manifest.map_id) << ",\n";
    report << "  \"display_name\": " << json_string(manifest.display_name) << ",\n";
    report << "  \"map_name\": " << json_string(manifest.display_name) << ",\n";
    report << "  \"building_id\": " << json_string(manifest.building_id) << ",\n";
    report << "  \"floor_id\": " << json_string(manifest.floor_id) << ",\n";
    report << "  \"resolution\": " << map.info.resolution << ",\n";
    report << "  \"width\": " << map.info.width << ",\n";
    report << "  \"height\": " << map.info.height << ",\n";
    report << "  \"nav_map\": " << json_string(manifest.nav_map_yaml.string()) << ",\n";
    report << "  \"localizer_map\": " << json_string(manifest.localizer_params_yaml.string()) << "\n";
    report << "}\n";
    write_text_file(manifest.asset_report_json, report.str());
  }

  nav_msgs::msg::OccupancyGrid latest_mapping_map_for_save(double & age_sec)
  {
    std::lock_guard<std::mutex> map_lock(live_map_mutex_);
    if (!have_live_map_) {
      throw std::runtime_error("no live slam_toolbox /map has been received; start 2D mapping before saving");
    }
    age_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_live_map_received_at_).count();
    return latest_live_map_;
  }

  HttpResponse handle_save_mapping_2d(const std::string & body)
  {
    const auto map_name = json_string_value(body, "map_name");
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");
    if (!map_name || !valid_display_map_name(*map_name)) {
      return {400, "application/json", error_json("valid map_name is required")};
    }
    if (!building_id || !safe_asset_id(*building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (!floor_id || !safe_asset_id(*floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }

    double map_age_sec = 0.0;
    nav_msgs::msg::OccupancyGrid map;
    bool mapping_was_active = false;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      mapping_2d_process_running_locked();
      mapping_was_active = mapping_2d_active_;
    }
    try {
      map = latest_mapping_map_for_save(map_age_sec);
    } catch (const std::exception & exc) {
      return {404, "application/json", error_json(exc.what())};
    }

    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    if (width == 0U || height == 0U || map.data.size() != static_cast<std::size_t>(width) * height) {
      return {503, "application/json", error_json("live slam_toolbox /map has invalid dimensions")};
    }
    set_mapping_runtime_state(true, "saving", "saving live 2D mapping assets");

    const auto pixels = occupancy_grid_to_image_pixels(map);
    const auto png = encode_grayscale_png(width, height, pixels);
    if (png.empty()) {
      set_mapping_runtime_state(true, "running", "map save failed: PNG encode failed", false);
      return {500, "application/json", error_json("failed to encode live slam_toolbox map as PNG")};
    }

    auto manifest = make_new_manifest(*building_id, *floor_id, *map_name);

    const fs::path runtime_dir(runtime_maps_dir_);
    const fs::path runtime_base = runtime_dir / manifest.safe_map_name;
    const fs::path runtime_yaml = runtime_base.string() + ".yaml";
    const fs::path runtime_pgm = runtime_base.string() + ".pgm";
    const fs::path runtime_png = runtime_base.string() + ".png";
    const fs::path runtime_localizer_yaml = runtime_dir / (manifest.safe_map_name + ".localizer.yaml");
    const fs::path runtime_localizer_png = runtime_dir / (manifest.safe_map_name + ".localizer.png");

    try {
      write_pgm_file(runtime_pgm, width, height, pixels);
      write_binary_file(runtime_png, png);
      write_text_file(runtime_yaml, map_yaml_text(runtime_pgm.filename().string(), map));
      write_binary_file(runtime_localizer_png, png);
      write_text_file(runtime_localizer_yaml, map_yaml_text(runtime_localizer_png.filename().string(), map));

      write_pgm_file(manifest.nav_map_pgm, width, height, pixels);
      write_text_file(
        manifest.nav_map_yaml, map_yaml_text(manifest.nav_map_pgm.filename().string(), map));
      write_binary_file(manifest.localizer_map_png, png);
      write_text_file(
        manifest.localizer_params_yaml,
        map_yaml_text(manifest.localizer_map_png.filename().string(), map));
      write_neutral_filter_assets(manifest.root / "filters", map);
      if (!fs::exists(manifest.poses_yaml)) {
        write_text_file(manifest.poses_yaml, "poses: []\n");
      }
      write_asset_report(manifest, map);
      write_map_manifest(manifest);
      activate_map_manifest(manifest);
      write_runtime_map_context(manifest, "ready", true, "2D map saved and activated");
    } catch (const std::exception & exc) {
      set_mapping_runtime_state(true, "running", std::string("map save failed: ") + exc.what(), false);
      return {500, "application/json", error_json(exc.what())};
    }

    std::size_t stopped_groups = 0U;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      stopped_groups = terminate_mapping_2d_process_groups_locked();
    }
    set_mapping_runtime_state(false, "stopped", "2D map saved and mapping chain stopped");

    std::ostringstream response;
    response << std::fixed << std::setprecision(3);
    response << "{\"ok\":true,"
             << "\"mapping_active\":false,"
             << "\"mapping_was_active\":" << (mapping_was_active ? "true" : "false") << ","
             << "\"stopped\":" << (stopped_groups > 0U ? "true" : "false") << ","
             << "\"stopped_groups\":" << stopped_groups << ","
             << "\"map_age_sec\":" << map_age_sec << ","
             << "\"map_id\":" << json_string(manifest.map_id) << ","
             << "\"display_name\":" << json_string(manifest.display_name) << ","
             << "\"map_name\":" << json_string(manifest.display_name) << ","
             << "\"safe_map_name\":" << json_string(manifest.safe_map_name) << ","
             << "\"building_id\":" << json_string(*building_id) << ","
             << "\"floor_id\":" << json_string(*floor_id) << ","
             << "\"runtime_map\":{"
             << "\"yaml\":" << json_string(runtime_yaml.string()) << ","
             << "\"pgm\":" << json_string(runtime_pgm.string()) << ","
             << "\"png\":" << json_string(runtime_png.string()) << ","
             << "\"localizer_yaml\":" << json_string(runtime_localizer_yaml.string()) << ","
             << "\"localizer_png\":" << json_string(runtime_localizer_png.string()) << "},"
             << "\"floor_assets\":{"
             << "\"root\":" << json_string(manifest.root.string()) << ","
             << "\"current_root\":" << json_string(floor_current_root_path(*building_id, *floor_id).string()) << ","
             << "\"manifest_json\":" << json_string(manifest.manifest_json.string()) << ","
             << "\"nav_map_yaml\":" << json_string(manifest.nav_map_yaml.string()) << ","
             << "\"nav_map_pgm\":" << json_string(manifest.nav_map_pgm.string()) << ","
             << "\"localizer_map_png\":" << json_string(manifest.localizer_map_png.string()) << ","
             << "\"localizer_params_yaml\":" << json_string(manifest.localizer_params_yaml.string()) << ","
             << "\"asset_report_json\":" << json_string(manifest.asset_report_json.string()) << "}}";
    return {200, "application/json", response.str()};
  }

  bool mapping_2d_process_running_locked()
  {
    if (mapping_2d_pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(mapping_2d_pid_, &status, WNOHANG);
    if (wait_result == mapping_2d_pid_) {
      mapping_2d_pid_ = -1;
      mapping_2d_active_ = false;
      set_mapping_runtime_state(false, "stopped", "2D mapping process exited");
      return false;
    }
    if (::kill(mapping_2d_pid_, 0) == 0) {
      return true;
    }
    if (errno == ESRCH) {
      mapping_2d_pid_ = -1;
      mapping_2d_active_ = false;
      set_mapping_runtime_state(false, "stopped", "2D mapping process is not alive");
      return false;
    }
    return true;
  }

  bool is_pid_directory(const fs::path & path) const
  {
    const auto name = path.filename().string();
    return !name.empty() && std::all_of(name.begin(), name.end(), [](const unsigned char c) {
      return std::isdigit(c) != 0;
    });
  }

  std::string read_proc_cmdline(const pid_t pid) const
  {
    std::ifstream file(fs::path("/proc") / std::to_string(pid) / "cmdline", std::ios::binary);
    if (!file) {
      return {};
    }
    std::string cmdline((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
    return trim(cmdline);
  }

  bool is_mapping_2d_process_command(const std::string & cmdline) const
  {
    static constexpr std::array<const char *, 4> kMapping2dProcessPatterns{
      "run_projected_map.sh",
      "jt128_slam_toolbox_mapping.launch.py",
      "jt128_2d_mapping.launch.py",
      "run_jt128_2d_mapping.sh"
    };
    return std::any_of(
      kMapping2dProcessPatterns.begin(), kMapping2dProcessPatterns.end(),
      [&cmdline](const char * pattern) {
        return cmdline.find(pattern) != std::string::npos;
      });
  }

  bool is_mapping_2d_residual_process_command(const std::string & cmdline) const
  {
    static constexpr std::array<const char *, 8> kMapping2dResidualProcessPatterns{
      "slam_toolbox",
      "nav_cloud_preprocessor",
      "pointcloud_to_laserscan_node",
      "scan_republisher_node",
      "fastlio_mapping",
      "laser_mapping",
      "ros2 run fast_lio fastlio_mapping",
      "run_fastlio_tf.sh"
    };
    return std::any_of(
      kMapping2dResidualProcessPatterns.begin(), kMapping2dResidualProcessPatterns.end(),
      [&cmdline](const char * pattern) {
        return cmdline.find(pattern) != std::string::npos;
      });
  }

  std::set<pid_t> discover_mapping_2d_process_groups() const
  {
    std::set<pid_t> groups;
    const pid_t self_pid = ::getpid();
    if (!fs::exists("/proc")) {
      return groups;
    }
    for (const auto & entry : fs::directory_iterator("/proc")) {
      if (!entry.is_directory() || !is_pid_directory(entry.path())) {
        continue;
      }
      const pid_t pid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
      if (pid <= 1 || pid == self_pid) {
        continue;
      }
      const auto cmdline = read_proc_cmdline(pid);
      if (!is_mapping_2d_process_command(cmdline)) {
        continue;
      }
      const pid_t pgid = ::getpgid(pid);
      groups.insert(pgid > 0 ? pgid : pid);
    }
    return groups;
  }

  std::set<pid_t> discover_mapping_2d_residual_processes() const
  {
    std::set<pid_t> pids;
    const pid_t self_pid = ::getpid();
    if (!fs::exists("/proc")) {
      return pids;
    }
    for (const auto & entry : fs::directory_iterator("/proc")) {
      if (!entry.is_directory() || !is_pid_directory(entry.path())) {
        continue;
      }
      const pid_t pid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
      if (pid <= 1 || pid == self_pid) {
        continue;
      }
      const auto cmdline = read_proc_cmdline(pid);
      if (is_mapping_2d_residual_process_command(cmdline)) {
        pids.insert(pid);
      }
    }
    return pids;
  }

  bool process_group_has_live_process(const pid_t pgid) const
  {
    if (pgid <= 0 || !fs::exists("/proc")) {
      return false;
    }
    const pid_t self_pid = ::getpid();
    for (const auto & entry : fs::directory_iterator("/proc")) {
      if (!entry.is_directory() || !is_pid_directory(entry.path())) {
        continue;
      }
      const pid_t pid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
      if (pid <= 1 || pid == self_pid) {
        continue;
      }
      if (::getpgid(pid) == pgid) {
        return true;
      }
    }
    return false;
  }

  bool process_pid_is_live(const pid_t pid) const
  {
    if (pid <= 0) {
      return false;
    }
    if (::kill(pid, 0) == 0) {
      return true;
    }
    return errno != ESRCH;
  }

  bool signal_process_group(const pid_t pgid, const int signal) const
  {
    if (pgid <= 0) {
      return false;
    }
    if (::kill(-pgid, signal) == 0) {
      return true;
    }
    return errno == ESRCH;
  }

  std::size_t terminate_mapping_2d_residual_processes() const
  {
    std::set<pid_t> pids = discover_mapping_2d_residual_processes();
    if (pids.empty()) {
      return 0U;
    }
    const std::size_t requested_pids = pids.size();
    for (const int signal : {SIGINT, SIGTERM, SIGKILL}) {
      for (const auto pid : pids) {
        ::kill(pid, signal);
      }
      std::this_thread::sleep_for(signal == SIGKILL ? 200ms : 800ms);
      for (auto it = pids.begin(); it != pids.end();) {
        if (!process_pid_is_live(*it)) {
          it = pids.erase(it);
        } else {
          ++it;
        }
      }
      if (pids.empty()) {
        break;
      }
    }
    return requested_pids;
  }

  std::size_t terminate_mapping_2d_process_groups_locked()
  {
    std::set<pid_t> groups = discover_mapping_2d_process_groups();
    if (mapping_2d_pid_ > 0) {
      const pid_t pgid = ::getpgid(mapping_2d_pid_);
      groups.insert(pgid > 0 ? pgid : mapping_2d_pid_);
    }

    const std::size_t requested_groups = groups.size();
    if (!groups.empty()) {
      for (const int signal : {SIGINT, SIGTERM, SIGKILL}) {
        for (const auto pgid : groups) {
          signal_process_group(pgid, signal);
        }
        std::this_thread::sleep_for(signal == SIGKILL ? 200ms : 800ms);

        for (auto it = groups.begin(); it != groups.end();) {
          if (!process_group_has_live_process(*it)) {
            it = groups.erase(it);
          } else {
            ++it;
          }
        }
        if (groups.empty()) {
          break;
        }
      }
    }

    if (mapping_2d_pid_ > 0) {
      int status = 0;
      while (::waitpid(mapping_2d_pid_, &status, WNOHANG) == mapping_2d_pid_) {
      }
    }
    mapping_2d_pid_ = -1;
    mapping_2d_active_ = false;
    set_mapping_runtime_state(false, "stopped", "2D mapping runtime stopped");
    return requested_groups + terminate_mapping_2d_residual_processes();
  }

  HttpResponse handle_start_mapping_2d()
  {
    if (mapping_2d_start_command_.empty() || !fs::exists(mapping_2d_start_command_)) {
      return {
        503,
        "application/json",
        error_json("2D slam_toolbox start command is not available: " + mapping_2d_start_command_)
      };
    }

    std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
    if (mapping_2d_process_running_locked()) {
      set_mapping_runtime_state(true, "running", "2D mapping chain is already running");
      return {
        202,
        "application/json",
        "{\"ok\":true,\"state\":\"already_running\",\"map_topic\":" + json_string(mapping_2d_live_map_topic_) +
        ",\"map_endpoint\":\"/api/v1/mapping/2d/map\"}"
      };
    }

    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      have_live_map_ = false;
      latest_live_map_ = nav_msgs::msg::OccupancyGrid{};
    }

    const pid_t pid = ::fork();
    if (pid < 0) {
      return {500, "application/json", error_json("failed to fork 2D slam_toolbox mapping process")};
    }
    if (pid == 0) {
      ::setsid();
      const int log_fd = ::open(mapping_2d_log_file_.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
      if (log_fd >= 0) {
        ::dup2(log_fd, STDOUT_FILENO);
        ::dup2(log_fd, STDERR_FILENO);
        ::close(log_fd);
      }
      close_inherited_fds();
      ::execl("/bin/bash", "bash", mapping_2d_start_command_.c_str(), static_cast<char *>(nullptr));
      ::_exit(127);
    }

    mapping_2d_pid_ = pid;
    mapping_2d_active_ = true;
    mapping_2d_started_at_ = std::chrono::steady_clock::now();
    set_mapping_runtime_state(true, "starting", "2D mapping chain start accepted");

    std::ostringstream body;
    body << "{\"ok\":true,\"state\":\"starting\","
         << "\"pid\":" << pid << ","
         << "\"map_topic\":" << json_string(mapping_2d_live_map_topic_) << ","
         << "\"map_endpoint\":\"/api/v1/mapping/2d/map\","
         << "\"log_file\":" << json_string(mapping_2d_log_file_) << "}";
    return {202, "application/json", body.str()};
  }

  HttpResponse handle_stop_mapping_2d()
  {
    set_mapping_runtime_state(true, "stopping", "stopping 2D mapping chain");
    std::size_t requested_groups = 0U;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      requested_groups = terminate_mapping_2d_process_groups_locked();
    }
    set_mapping_runtime_state(false, "stopped", "2D mapping chain stopped");

    std::ostringstream body;
    body << "{\"ok\":true,\"mapping_active\":false,\"stopped\":"
         << (requested_groups > 0U ? "true" : "false")
         << ",\"stopped_groups\":" << requested_groups << "}";
    return {200, "application/json", body.str()};
  }

  bool navigation_resume_process_running_locked()
  {
    if (navigation_resume_pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(navigation_resume_pid_, &status, WNOHANG);
    if (wait_result == navigation_resume_pid_) {
      navigation_resume_pid_ = -1;
      return false;
    }
    if (::kill(navigation_resume_pid_, 0) == 0) {
      return true;
    }
    if (errno == ESRCH) {
      navigation_resume_pid_ = -1;
      return false;
    }
    return true;
  }

  void terminate_navigation_resume_process_locked()
  {
    if (!navigation_resume_process_running_locked()) {
      navigation_resume_pid_ = -1;
      return;
    }
    const pid_t pgid = ::getpgid(navigation_resume_pid_);
    if (pgid > 0) {
      signal_process_group(pgid, SIGINT);
      std::this_thread::sleep_for(800ms);
      if (process_group_has_live_process(pgid)) {
        signal_process_group(pgid, SIGTERM);
        std::this_thread::sleep_for(800ms);
      }
      if (process_group_has_live_process(pgid)) {
        signal_process_group(pgid, SIGKILL);
      }
    } else {
      ::kill(navigation_resume_pid_, SIGINT);
    }
    int status = 0;
    while (::waitpid(navigation_resume_pid_, &status, WNOHANG) == navigation_resume_pid_) {
    }
    navigation_resume_pid_ = -1;
  }

  HttpResponse handle_resume_floor_navigation(
    const std::string & building_id,
    const std::string & floor_id,
    const std::optional<MapManifest> & selected_map = std::nullopt)
  {
    FloorAssetPaths assets;
    std::string error;
    if (!resolve_floor_asset_paths(building_id, floor_id, assets, error)) {
      return {404, "application/json", error_json(error)};
    }
    if (navigation_resume_command_.empty() || !fs::exists(navigation_resume_command_)) {
      return {
        503,
        "application/json",
        error_json("navigation resume command is not available: " + navigation_resume_command_)
      };
    }

    std::lock_guard<std::mutex> process_lock(navigation_process_mutex_);
    terminate_navigation_resume_process_locked();
    if (selected_map) {
      write_runtime_map_context(
        *selected_map, "starting", false, "navigation runtime start accepted");
    }

    const pid_t pid = ::fork();
    if (pid < 0) {
      return {500, "application/json", error_json("failed to fork navigation resume process")};
    }
    if (pid == 0) {
      ::setsid();
      const int log_fd = ::open(navigation_resume_log_file_.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
      if (log_fd >= 0) {
        ::dup2(log_fd, STDOUT_FILENO);
        ::dup2(log_fd, STDERR_FILENO);
        ::close(log_fd);
      }
      close_inherited_fds();
      if (selected_map) {
        ::setenv("NJRH_RUNTIME_MAP_CONTEXT_FILE", runtime_map_context_file_.c_str(), 1);
        ::setenv("NJRH_MAP_ID", selected_map->map_id.c_str(), 1);
        ::setenv("NJRH_MAP_DISPLAY_NAME", selected_map->display_name.c_str(), 1);
        ::setenv("NJRH_MAP_CONTEXT_BUILDING_ID", selected_map->building_id.c_str(), 1);
        ::setenv("NJRH_MAP_CONTEXT_FLOOR_ID", selected_map->floor_id.c_str(), 1);
      }
      ::execl(
        "/bin/bash",
        "bash",
        navigation_resume_command_.c_str(),
        building_id.c_str(),
        floor_id.c_str(),
        static_cast<char *>(nullptr));
      ::_exit(127);
    }

    navigation_resume_pid_ = pid;
    set_navigation_runtime_state(true, "starting", "navigation runtime start accepted");

    std::ostringstream body;
    body << "{\"ok\":true,"
         << "\"state\":\"navigation_resume_starting\","
         << "\"pid\":" << pid << ","
         << "\"building_id\":" << json_string(building_id) << ","
         << "\"floor_id\":" << json_string(floor_id) << ","
         << "\"map_id\":" << json_string(selected_map ? selected_map->map_id : "") << ","
         << "\"display_name\":" << json_string(selected_map ? selected_map->display_name : "") << ","
         << "\"resume_navigation\":true,"
         << "\"nav_map_yaml\":" << json_string(assets.nav_map_yaml.string()) << ","
         << "\"localizer_map_png\":" << json_string(assets.localizer_map_png.string()) << ","
         << "\"localizer_params_yaml\":" << json_string(assets.localizer_params_yaml.string()) << ","
         << "\"log_file\":" << json_string(navigation_resume_log_file_) << "}";
    return {202, "application/json", body.str()};
  }

  std::uint8_t occupancy_to_gray(const int value) const
  {
    if (value < 0) {
      return 205U;
    }
    if (value == 0) {
      return 254U;
    }
    if (value >= 100) {
      return 0U;
    }
    const double occupied_ratio = static_cast<double>(value) / 100.0;
    return static_cast<std::uint8_t>(std::clamp(254.0 - occupied_ratio * 254.0, 0.0, 254.0));
  }

  HttpResponse handle_live_mapping_2d_map_png()
  {
    if (!subscription_manager_ || !subscription_manager_->active("live_map")) {
      return {
        409,
        "application/json",
        error_json("live_map resource is not acquired; call POST /api/v1/subscriptions/acquire first")
      };
    }

    std::chrono::steady_clock::time_point started_at;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      mapping_2d_process_running_locked();
      if (!mapping_2d_active_) {
        return {
          409,
          "application/json",
          error_json("2D slam_toolbox mapping is not active; call POST /api/v1/mapping/2d/start first")
        };
      }
      started_at = mapping_2d_started_at_;
    }

    nav_msgs::msg::OccupancyGrid map;
    std::chrono::steady_clock::time_point received_at;
    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      if (!have_live_map_ || latest_live_map_received_at_ < started_at) {
        return {
          404,
          "application/json",
          error_json("waiting for live slam_toolbox /map data")
        };
      }
      map = latest_live_map_;
      received_at = latest_live_map_received_at_;
    }

    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - received_at).count();
    if (age > mapping_2d_live_map_max_age_sec_) {
      return {
        503,
        "application/json",
        error_json("live slam_toolbox /map is stale")
      };
    }

    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    if (width == 0U || height == 0U || map.data.size() != static_cast<std::size_t>(width) * height) {
      return {503, "application/json", error_json("live slam_toolbox /map has invalid dimensions")};
    }

    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);
    for (std::uint32_t y = 0; y < height; ++y) {
      const std::uint32_t src_y = height - 1U - y;
      for (std::uint32_t x = 0; x < width; ++x) {
        const std::size_t src = static_cast<std::size_t>(src_y) * width + x;
        const std::size_t dst = static_cast<std::size_t>(y) * width + x;
        pixels[dst] = occupancy_to_gray(static_cast<int>(map.data[src]));
      }
    }

    const auto png = encode_grayscale_png(width, height, pixels);
    if (png.empty()) {
      return {500, "application/json", error_json("failed to encode live slam_toolbox map as PNG")};
    }
    return {200, "image/png", png};
  }

  HttpResponse handle_mapping_2d_map_png(const HttpRequest & request)
  {
    const auto source_it = request.query.find("source");
    const bool explicit_saved =
      request.query.find("name") != request.query.end() ||
      (source_it != request.query.end() && source_it->second == "saved");
    if (explicit_saved) {
      return handle_saved_mapping_2d_map_png(request);
    }
    return handle_live_mapping_2d_map_png();
  }

  HttpResponse handle_save_pose(
    const std::string & body,
    const std::optional<std::string> & forced_pose_id = std::nullopt)
  {
    const auto requested_building_id = json_string_value(body, "building_id");
    const auto requested_floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    const auto body_pose_id = json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or(""));
    const auto pose_id = forced_pose_id.value_or(body_pose_id);
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }
    if (forced_pose_id && !body_pose_id.empty() && body_pose_id != *forced_pose_id) {
      return {400, "application/json", error_json("pose_id in body does not match path pose_id")};
    }

    const auto x = json_number_value(body, "x").value_or(
      json_nested_number_value(body, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto y = json_number_value(body, "y").value_or(
      json_nested_number_value(body, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto yaw = json_number_value(body, "yaw").value_or(
      json_number_value(body, "theta").value_or(
        json_nested_number_value(body, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      return {400, "application/json", error_json("finite x, y, and yaw are required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    if (map_id && !map_id->empty()) {
      manifest = find_map_by_id(*map_id);
      if (!manifest) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
    } else {
      if (!requested_building_id) {
        return {400, "application/json", error_json("valid building_id is required")};
      }
      if (!requested_floor_id) {
        return {400, "application/json", error_json("valid floor_id is required")};
      }
      resolved_building_id = *requested_building_id;
      resolved_floor_id = *requested_floor_id;
      manifest = active_floor_map(resolved_building_id, resolved_floor_id);
    }

    const auto floor_root = floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = json_string_value(body, "name").value_or(pose_id);
    pose.type = json_string_value(body, "type").value_or("delivery_point");
    pose.x = x;
    pose.y = y;
    pose.yaw = normalize_angle(yaw);

    const auto path = manifest ? manifest->poses_yaml : poses_yaml_path(resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      bool updated = false;
      for (auto & existing : poses) {
        if (existing.id == pose.id) {
          existing = pose;
          updated = true;
          break;
        }
      }
      if (!updated) {
        poses.push_back(pose);
      }
      write_floor_poses(path, poses);
      if (manifest && manifest->active) {
        copy_file_if_exists(path, floor_current_root_path(resolved_building_id, resolved_floor_id) / "poses.yaml");
        copy_file_if_exists(path, floor_root_path(resolved_building_id, resolved_floor_id) / "poses.yaml");
      }

      std::ostringstream response;
      response << std::fixed << std::setprecision(6)
               << "{\"ok\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose.id) << ","
               << "\"updated\":" << (updated ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(path.string()) << ","
               << "\"pose\":{\"x\":" << pose.x << ",\"y\":" << pose.y << ",\"yaw\":" << pose.yaw << "}}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & ex) {
      return {500, "application/json", error_json(ex.what())};
    }
  }

  HttpResponse handle_save_current_pose(const std::string & body)
  {
    const auto stripped_body = trim(body);
    if (stripped_body.empty() || stripped_body.front() != '{') {
      return {400, "application/json", error_json("JSON request body is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        json_string_value(body, "building_id"),
        json_string_value(body, "floor_id"),
        json_string_value(body, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }
    if (!manifest) {
      return {
        404,
        "application/json",
        error_json("active map not found for floor: " + resolved_building_id + "/" + resolved_floor_id)
      };
    }

    const auto floor_root = floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    const auto body_pose_id = json_string_value(body, "pose_id").value_or(
      json_string_value(body, "id").value_or(""));
    const std::string pose_type = json_string_value(body, "type").value_or("delivery_point");
    const std::string pose_name = json_string_value(body, "name").value_or(
      body_pose_id.empty() ? pose_type : body_pose_id);
    const std::string pose_id =
      body_pose_id.empty() ? generated_current_pose_id(pose_type, pose_name) : body_pose_id;
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }

    std::string pose_error;
    const auto current_pose = wait_for_current_robot_pose(true, pose_error);
    if (!current_pose.available) {
      (void)pose_error;
      return {503, "application/json", no_fresh_map_robot_pose_json()};
    }
    std::string context_error;
    bool blocked_by_pending_context = false;
    const auto current_context = confirmed_runtime_map_manifest(context_error, blocked_by_pending_context);
    if (blocked_by_pending_context) {
      return {503, "application/json", no_fresh_map_robot_pose_json(context_error)};
    }
    if (current_context &&
      (current_context->map_id != manifest->map_id ||
      current_context->building_id != resolved_building_id ||
      current_context->floor_id != resolved_floor_id))
    {
      return {
        503,
        "application/json",
        no_fresh_map_robot_pose_json(
          "requested pose target does not match confirmed runtime map context: " +
          current_context->building_id + "/" + current_context->floor_id + "/" +
          current_context->map_id)
      };
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = pose_name.empty() ? pose_id : pose_name;
    pose.type = pose_type.empty() ? "delivery_point" : pose_type;
    pose.x = current_pose.x;
    pose.y = current_pose.y;
    pose.yaw = current_pose.yaw;

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      bool updated = false;
      for (auto & existing : poses) {
        if (existing.id == pose.id) {
          existing = pose;
          updated = true;
          break;
        }
      }
      if (!updated) {
        poses.push_back(pose);
      }
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << std::fixed << std::setprecision(6)
               << "{\"ok\":true,"
               << "\"source\":\"current_robot_pose\","
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose.id) << ","
               << "\"updated\":" << (updated ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(path.string()) << ","
               << "\"source_pose\":{"
               << "\"frame_id\":" << json_string(current_pose.frame_id) << ","
               << "\"child_frame_id\":" << json_string(current_pose.child_frame_id) << ","
               << "\"x\":" << current_pose.x << ","
               << "\"y\":" << current_pose.y << ","
               << "\"yaw\":" << current_pose.yaw << ","
               << "\"stamp\":" << current_pose.stamp_sec << ","
               << "\"age_sec\":" << current_pose.age_sec << "},"
               << "\"pose\":{"
               << "\"id\":" << json_string(pose.id) << ","
               << "\"name\":" << json_string(pose.name) << ","
               << "\"type\":" << json_string(pose.type) << ","
               << "\"x\":" << pose.x << ","
               << "\"y\":" << pose.y << ","
               << "\"yaw\":" << pose.yaw << "}}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  std::optional<std::string> request_value(const HttpRequest & request, const std::string & key) const
  {
    const auto query = query_value(request, key);
    if (query) {
      return query;
    }
    return json_string_value(request.body, key);
  }

  bool resolve_pose_target_manifest(
    const std::optional<std::string> & requested_building_id,
    const std::optional<std::string> & requested_floor_id,
    const std::optional<std::string> & map_id,
    std::optional<MapManifest> & manifest,
    std::string & resolved_building_id,
    std::string & resolved_floor_id,
    HttpResponse & error)
  {
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      error = {400, "application/json", error_json("valid building_id is required")};
      return false;
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      error = {400, "application/json", error_json("valid floor_id is required")};
      return false;
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      error = {400, "application/json", error_json("valid map_id is required")};
      return false;
    }

    if (map_id && !map_id->empty()) {
      manifest = find_map_by_id(*map_id);
      if (!manifest) {
        error = {404, "application/json", error_json("map_id not found: " + *map_id)};
        return false;
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        error = {400, "application/json", error_json("map_id does not belong to requested building")};
        return false;
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        error = {400, "application/json", error_json("map_id does not belong to requested floor")};
        return false;
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
      return true;
    }

    if (!requested_building_id) {
      error = {400, "application/json", error_json("valid building_id is required")};
      return false;
    }
    if (!requested_floor_id) {
      error = {400, "application/json", error_json("valid floor_id is required")};
      return false;
    }
    resolved_building_id = *requested_building_id;
    resolved_floor_id = *requested_floor_id;
    manifest = active_floor_map(resolved_building_id, resolved_floor_id);
    return true;
  }

  fs::path pose_target_path(
    const std::optional<MapManifest> & manifest,
    const std::string & building_id,
    const std::string & floor_id) const
  {
    return manifest ? manifest->poses_yaml : poses_yaml_path(building_id, floor_id);
  }

  void sync_active_poses_if_needed(
    const std::optional<MapManifest> & manifest,
    const std::string & building_id,
    const std::string & floor_id,
    const fs::path & path) const
  {
    if (!manifest || !manifest->active) {
      return;
    }
    copy_file_if_exists(path, floor_current_root_path(building_id, floor_id) / "poses.yaml");
    copy_file_if_exists(path, floor_root_path(building_id, floor_id) / "poses.yaml");
  }

  std::optional<StoredPose> parse_pose_payload(
    const std::string & payload,
    const std::optional<std::string> & forced_pose_id,
    std::string & error) const
  {
    const auto body_pose_id = json_string_value(payload, "pose_id").value_or(
      json_string_value(payload, "id").value_or(""));
    const auto pose_id = forced_pose_id.value_or(body_pose_id);
    if (!safe_pose_id(pose_id)) {
      error = "valid pose_id is required";
      return std::nullopt;
    }
    if (forced_pose_id && !body_pose_id.empty() && body_pose_id != *forced_pose_id) {
      error = "pose_id in body does not match path pose_id";
      return std::nullopt;
    }

    const auto x = json_number_value(payload, "x").value_or(
      json_nested_number_value(payload, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto y = json_number_value(payload, "y").value_or(
      json_nested_number_value(payload, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto yaw = json_number_value(payload, "yaw").value_or(
      json_number_value(payload, "theta").value_or(
        json_nested_number_value(payload, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      error = "finite x, y, and yaw are required";
      return std::nullopt;
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = json_string_value(payload, "name").value_or(pose_id);
    pose.type = json_string_value(payload, "type").value_or("delivery_point");
    pose.x = x;
    pose.y = y;
    pose.yaw = normalize_angle(yaw);
    return pose;
  }

  HttpResponse handle_delete_pose(const HttpRequest & request, const std::string & pose_id)
  {
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        request_value(request, "building_id"),
        request_value(request, "floor_id"),
        request_value(request, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }

    const auto floor_root = floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      const auto before = poses.size();
      poses.erase(
        std::remove_if(poses.begin(), poses.end(), [&pose_id](const StoredPose & pose) {
          return pose.id == pose_id;
        }),
        poses.end());
      if (poses.size() == before) {
        return {404, "application/json", error_json("pose_id not found in poses.yaml: " + pose_id)};
      }
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"deleted\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose_id) << ","
               << "\"remaining\":" << poses.size() << ","
               << "\"poses_yaml\":" << json_string(path.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_replace_poses_batch(const std::string & body)
  {
    if (body.find("\"poses\"") == std::string::npos) {
      return {400, "application/json", error_json("poses array is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        json_string_value(body, "building_id"),
        json_string_value(body, "floor_id"),
        json_string_value(body, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }

    const auto floor_root = floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    std::vector<StoredPose> poses;
    std::set<std::string> pose_ids;
    for (const auto & object : json_object_array_value(body, "poses")) {
      std::string parse_error;
      auto pose = parse_pose_payload(object, std::nullopt, parse_error);
      if (!pose) {
        return {400, "application/json", error_json(parse_error)};
      }
      if (!pose_ids.insert(pose->id).second) {
        return {400, "application/json", error_json("duplicate pose_id in batch: " + pose->id)};
      }
      poses.push_back(*pose);
    }

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"replaced\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"count\":" << poses.size() << ","
               << "\"poses_yaml\":" << json_string(path.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_save_keepout_filter(const std::string & body)
  {
    const auto stripped_body = trim(body);
    if (stripped_body.empty() || stripped_body.front() != '{') {
      return {400, "application/json", error_json("JSON request body is required")};
    }

    const auto requested_building_id = json_string_value(body, "building_id");
    const auto requested_floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    if (map_id && !map_id->empty()) {
      manifest = find_map_by_id(*map_id);
      if (!manifest) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
    } else {
      if (!requested_building_id) {
        return {400, "application/json", error_json("valid building_id is required")};
      }
      if (!requested_floor_id) {
        return {400, "application/json", error_json("valid floor_id is required")};
      }
      resolved_building_id = *requested_building_id;
      resolved_floor_id = *requested_floor_id;
      manifest = active_floor_map(resolved_building_id, resolved_floor_id);
      if (!manifest) {
        return {
          404,
          "application/json",
          error_json("active map not found for floor: " + resolved_building_id + "/" + resolved_floor_id)
        };
      }
    }

    const auto path = keepout_semantic_json_path(*manifest);
    try {
      write_text_file(path, stripped_body + "\n");
      if (manifest->active) {
        copy_file_if_exists(
          path,
          floor_current_root_path(resolved_building_id, resolved_floor_id) /
          "filters" / "keepout_semantic_layer.json");
        copy_file_if_exists(
          path,
          floor_root_path(resolved_building_id, resolved_floor_id) /
          "filters" / "keepout_semantic_layer.json");
      }

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest->map_id) << ","
               << "\"display_name\":" << json_string(manifest->display_name) << ","
               << "\"map_name\":" << json_string(manifest->display_name) << ","
               << "\"active\":" << (manifest->active ? "true" : "false") << ","
               << "\"semantic_json_path\":" << json_string(path.string()) << ","
               << "\"keepout_mask_yaml\":" << json_string(manifest->keepout_mask_yaml.string()) << ","
               << "\"keepout_mask_pgm\":" << json_string(manifest->keepout_mask_pgm.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_navigation_goal(const std::string & body)
  {
    const auto pose_id = json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or(""));
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");
    const bool by_pose_id = !pose_id.empty();

    StoredPose target;
    std::string target_source = "direct_pose";
    std::string frame_id = normalized_frame_id(json_string_value(body, "frame_id").value_or("map"));

    if (by_pose_id) {
      if (!building_id || !safe_map_name(*building_id)) {
        return {400, "application/json", error_json("valid building_id is required for pose_id navigation")};
      }
      if (!floor_id || !safe_map_name(*floor_id)) {
        return {400, "application/json", error_json("valid floor_id is required for pose_id navigation")};
      }
      if (!safe_pose_id(pose_id)) {
        return {400, "application/json", error_json("valid pose_id is required")};
      }
      std::optional<StoredPose> pose;
      try {
        pose = find_floor_pose(*building_id, *floor_id, pose_id);
      } catch (const std::exception & ex) {
        return {500, "application/json", error_json(ex.what())};
      }
      if (!pose) {
        return {404, "application/json", error_json("pose_id not found in poses.yaml: " + pose_id)};
      }
      target = *pose;
      target_source = "poses_yaml";
      frame_id = "map";
    } else {
      const auto x = json_number_value(body, "x").value_or(
        json_nested_number_value(body, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
      const auto y = json_number_value(body, "y").value_or(
        json_nested_number_value(body, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
      const auto yaw = json_number_value(body, "yaw").value_or(
        json_number_value(body, "theta").value_or(
          json_nested_number_value(body, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        return {400, "application/json", error_json("pose_id or finite x, y, and yaw are required")};
      }
      target.id = "direct";
      target.name = "direct";
      target.type = "direct_goal";
      target.x = x;
      target.y = y;
      target.yaw = normalize_angle(yaw);
    }

    if (frame_id != "map") {
      return {400, "application/json", error_json("navigation goals must be in map frame")};
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame_id;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.z = std::sin(target.yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(target.yaw * 0.5);

    NavigateGoalHandle::SharedPtr goal_handle;
    try {
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      if (!navigate_to_pose_client_->wait_for_action_server(service_timeout())) {
        return {503, "application/json", error_json("action unavailable: " + navigate_to_pose_action_)};
      }
      auto future = navigate_to_pose_client_->async_send_goal(goal);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        return {503, "application/json", error_json("timed out sending navigation goal")};
      }
      goal_handle = future.get();
    } catch (const std::exception & exc) {
      return {
        503,
        "application/json",
        error_json(std::string("exception sending navigation goal: ") + exc.what())};
    } catch (...) {
      return {503, "application/json", error_json("unknown exception sending navigation goal")};
    }

    if (!goal_handle) {
      return {500, "application/json", error_json("navigation goal was rejected by Nav2")};
    }

    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_nav_goal_handle_ = goal_handle;
      active_nav_goal_pose_id_ = by_pose_id ? pose_id : "";
      active_nav_goal_building_id_ = building_id.value_or("");
      active_nav_goal_floor_id_ = floor_id.value_or("");
    }
    set_navigation_runtime_state(true, "navigating", "navigation goal accepted");

    std::ostringstream response;
    response << std::fixed << std::setprecision(6)
             << "{\"ok\":true,"
             << "\"accepted\":true,"
             << "\"action\":" << json_string(navigate_to_pose_action_) << ","
             << "\"source\":" << json_string(target_source) << ","
             << "\"frame_id\":" << json_string(frame_id) << ","
             << "\"pose_id\":" << json_string(by_pose_id ? pose_id : "") << ",";
    if (building_id) {
      response << "\"building_id\":" << json_string(*building_id) << ",";
    }
    if (floor_id) {
      response << "\"floor_id\":" << json_string(*floor_id) << ",";
    }
    response << "\"goal\":{\"x\":" << target.x << ",\"y\":" << target.y << ",\"yaw\":" << target.yaw << "}}";
    return {202, "application/json", response.str()};
  }

  bool cancel_active_navigation_goal(std::string & detail)
  {
    NavigateGoalHandle::SharedPtr active_goal;
    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_goal = active_nav_goal_handle_;
    }

    if (!active_goal) {
      detail = "no active API goal handle cached";
      return false;
    }

    try {
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      auto future = navigate_to_pose_client_->async_cancel_goal(active_goal);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        detail = "timed out canceling cached goal handle";
        return false;
      }
    } catch (const std::exception & exc) {
      detail = std::string("exception canceling cached goal handle: ") + exc.what();
      return false;
    } catch (...) {
      detail = "unknown exception canceling cached goal handle";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_nav_goal_handle_.reset();
      active_nav_goal_pose_id_.clear();
      active_nav_goal_building_id_.clear();
      active_nav_goal_floor_id_.clear();
    }
    detail = "cached goal handle cancel requested";
    return true;
  }

  std::string navigation_cancel_job_json_locked() const
  {
    std::ostringstream response;
    response << "{\"id\":" << navigation_cancel_job_.id << ","
             << "\"state\":" << json_string(navigation_cancel_job_.state) << ","
             << "\"phase\":" << json_string(navigation_cancel_job_.phase) << ","
             << "\"reason\":" << json_string(navigation_cancel_job_.reason) << ","
             << "\"stop_stack\":" << (navigation_cancel_job_.stop_stack ? "true" : "false") << ","
             << "\"ok\":" << (navigation_cancel_job_.ok ? "true" : "false") << ","
             << "\"action_available\":" << (navigation_cancel_job_.action_available ? "true" : "false") << ","
             << "\"active_goal_cancel_requested\":"
             << (navigation_cancel_job_.active_goal_cancel_requested ? "true" : "false") << ","
             << "\"cancel_all_requested\":"
             << (navigation_cancel_job_.cancel_all_requested ? "true" : "false") << ","
             << "\"cancel_all_ok\":" << (navigation_cancel_job_.cancel_all_ok ? "true" : "false") << ","
             << "\"navigation_stack_stopped\":"
             << (navigation_cancel_job_.stop_stack_ok ? "true" : "false") << ","
             << "\"zero_velocity_published\":"
             << (navigation_cancel_job_.zero_velocity_published ? "true" : "false") << ","
             << "\"detail\":" << json_string(navigation_cancel_job_.detail) << ","
             << "\"cancel_all_detail\":" << json_string(navigation_cancel_job_.cancel_all_detail) << ","
             << "\"stop_stack_detail\":" << json_string(navigation_cancel_job_.stop_stack_detail) << ","
             << "\"started_at\":" << json_string(navigation_cancel_job_.started_at) << ","
             << "\"finished_at\":" << json_string(navigation_cancel_job_.finished_at) << "}";
    return response.str();
  }

  void set_navigation_cancel_job_phase(const std::uint64_t job_id, const std::string & phase)
  {
    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    if (navigation_cancel_job_.id == job_id && navigation_cancel_job_.state == "running") {
      navigation_cancel_job_.phase = phase;
    }
  }

  void finish_navigation_cancel_job(
    const std::uint64_t job_id,
    const bool ok,
    const bool action_available,
    const bool active_goal_cancel_requested,
    const bool cancel_all_requested,
    const bool cancel_all_ok,
    const bool stop_stack_ok,
    const std::string & detail,
    const std::string & cancel_all_detail,
    const std::string & stop_stack_detail)
  {
    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    if (navigation_cancel_job_.id != job_id) {
      return;
    }
    navigation_cancel_job_.state = ok ? "succeeded" : "failed";
    navigation_cancel_job_.phase = "finished";
    navigation_cancel_job_.ok = ok;
    navigation_cancel_job_.action_available = action_available;
    navigation_cancel_job_.active_goal_cancel_requested = active_goal_cancel_requested;
    navigation_cancel_job_.cancel_all_requested = cancel_all_requested;
    navigation_cancel_job_.cancel_all_ok = cancel_all_ok;
    navigation_cancel_job_.stop_stack_ok = stop_stack_ok;
    navigation_cancel_job_.detail = detail;
    navigation_cancel_job_.cancel_all_detail = cancel_all_detail;
    navigation_cancel_job_.stop_stack_detail = stop_stack_detail;
    navigation_cancel_job_.finished_at = utc_timestamp_iso8601();
    if (ok && navigation_cancel_job_.stop_stack) {
      set_navigation_runtime_state(false, "stopped", "navigation runtime stopped");
    } else if (ok) {
      set_navigation_runtime_state(true, "ready", "navigation goal canceled; navigation stack remains active");
    } else {
      const auto message = stop_stack_detail.empty() ? detail : stop_stack_detail;
      set_navigation_runtime_state(true, "error", message, false);
    }
  }

  void run_navigation_cancel_job(const std::uint64_t job_id, const bool stop_stack)
  {
    bool action_available = false;
    std::string action_availability_detail;
    set_navigation_cancel_job_phase(job_id, "wait_for_nav2_action");
    try {
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      action_available = navigate_to_pose_client_->wait_for_action_server(service_timeout());
    } catch (const std::exception & exc) {
      action_availability_detail = std::string("exception waiting for navigation action server: ") + exc.what();
    } catch (...) {
      action_availability_detail = "unknown exception waiting for navigation action server";
    }
    std::string active_goal_detail;
    bool active_goal_cancel_requested = false;
    bool cancel_all_requested = false;
    bool cancel_all_ok = true;
    std::string cancel_all_detail = "not requested";

    if (action_available) {
      set_navigation_cancel_job_phase(job_id, "cancel_cached_goal");
      active_goal_cancel_requested = cancel_active_navigation_goal(active_goal_detail);
      cancel_all_requested = true;
      set_navigation_cancel_job_phase(job_id, "cancel_all_goals");
      try {
        std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
        auto cancel_all_future = navigate_to_pose_client_->async_cancel_all_goals();
        if (cancel_all_future.wait_for(service_timeout()) == std::future_status::ready) {
          cancel_all_detail = "cancel-all requested";
        } else {
          cancel_all_ok = false;
          cancel_all_detail = "timed out canceling navigation goals";
        }
      } catch (const std::exception & exc) {
        cancel_all_ok = false;
        cancel_all_detail = std::string("exception canceling navigation goals: ") + exc.what();
      } catch (...) {
        cancel_all_ok = false;
        cancel_all_detail = "unknown exception canceling navigation goals";
      }
    } else {
      active_goal_detail = action_availability_detail.empty()
        ? "action server unavailable: " + navigate_to_pose_action_
        : action_availability_detail;
      cancel_all_ok = false;
      cancel_all_detail = active_goal_detail;
    }

    set_navigation_cancel_job_phase(job_id, "publish_zero_velocity");
    clear_teleop_command();
    publish_teleop_zero_burst();

    std::string stop_stack_detail = "not requested";
    bool stop_stack_ok = true;
    if (stop_stack) {
      set_navigation_cancel_job_phase(job_id, "stop_navigation_stack");
      stop_stack_ok = stop_navigation_runtime_stack(stop_stack_detail);
      publish_teleop_zero_burst();
    }

    const bool ok = stop_stack ? stop_stack_ok : cancel_all_ok;
    finish_navigation_cancel_job(
      job_id,
      ok,
      action_available,
      active_goal_cancel_requested,
      cancel_all_requested,
      cancel_all_ok,
      stop_stack_ok,
      active_goal_detail,
      cancel_all_detail,
      stop_stack_detail);
  }

  HttpResponse handle_navigation_state()
  {
    const auto runtime = runtime_mode_snapshot();
    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"mode\":" << json_string(runtime.mode) << ","
             << "\"state\":" << json_string(runtime.navigation_state) << ","
             << "\"navigation_active\":" << (runtime.navigation_active ? "true" : "false") << ","
             << "\"healthy\":" << (runtime.healthy ? "true" : "false") << ","
             << "\"message\":" << json_string(runtime.message) << ","
             << "\"navigation_cancel\":" << navigation_cancel_job_json_locked() << "}";
    return {200, "application/json", response.str()};
  }

  void join_navigation_cancel_worker()
  {
    if (navigation_cancel_worker_.joinable()) {
      navigation_cancel_worker_.join();
    }
  }

  HttpResponse handle_navigation_cancel(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("");
    const bool stop_stack = json_bool_value(body, "stop_stack", true);

    clear_teleop_command();
    publish_teleop_zero_burst();
    set_navigation_runtime_state(true, "canceling", "navigation cancel accepted");

    std::lock_guard<std::mutex> start_lock(navigation_cancel_start_mutex_);

    {
      std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
      if (navigation_cancel_job_.state == "running") {
        std::ostringstream response;
        response << "{\"ok\":true,"
                 << "\"accepted\":true,"
                 << "\"already_running\":true,"
                 << "\"navigation_cancel\":" << navigation_cancel_job_json_locked() << "}";
        return {202, "application/json", response.str()};
      }
    }

    join_navigation_cancel_worker();

    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
      job_id = ++navigation_cancel_job_seq_;
      navigation_cancel_job_ = NavigationCancelJob{};
      navigation_cancel_job_.id = job_id;
      navigation_cancel_job_.state = "running";
      navigation_cancel_job_.phase = "accepted";
      navigation_cancel_job_.reason = reason;
      navigation_cancel_job_.stop_stack = stop_stack;
      navigation_cancel_job_.started_at = utc_timestamp_iso8601();
      navigation_cancel_job_.zero_velocity_published = true;
    }

    try {
      navigation_cancel_worker_ = std::thread(
        [this, job_id, stop_stack]() {
          run_navigation_cancel_job(job_id, stop_stack);
        });
    } catch (const std::exception & exc) {
      finish_navigation_cancel_job(
        job_id,
        false,
        false,
        false,
        false,
        false,
        false,
        std::string("failed to start navigation cancel worker: ") + exc.what(),
        "not requested",
        "not requested");
      return {
        500,
        "application/json",
        error_json(std::string("failed to start navigation cancel worker: ") + exc.what())};
    }

    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"accepted\":true,"
             << "\"cancel_requested\":true,"
             << "\"action\":" << json_string(navigate_to_pose_action_) << ","
             << "\"navigation_cancel\":" << navigation_cancel_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  bool stop_navigation_runtime_stack(std::string & detail)
  {
    if (navigation_stop_command_.empty() || !fs::exists(navigation_stop_command_)) {
      detail = "navigation stop command is not available: " + navigation_stop_command_;
      return false;
    }

    {
      std::lock_guard<std::mutex> process_lock(navigation_process_mutex_);
      terminate_navigation_resume_process_locked();
    }

    const pid_t pid = ::fork();
    if (pid < 0) {
      detail = "failed to fork navigation stop process";
      return false;
    }
    if (pid == 0) {
      ::setsid();
      const int log_fd = ::open(navigation_stop_log_file_.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
      if (log_fd >= 0) {
        ::dup2(log_fd, STDOUT_FILENO);
        ::dup2(log_fd, STDERR_FILENO);
        ::close(log_fd);
      }
      close_inherited_fds();
      ::execl("/bin/bash", "bash", navigation_stop_command_.c_str(), static_cast<char *>(nullptr));
      ::_exit(127);
    }

    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    bool exited = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const pid_t wait_result = ::waitpid(pid, &status, WNOHANG);
      if (wait_result == pid) {
        exited = true;
        break;
      }
      if (wait_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        detail = "failed waiting for navigation stop process";
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (!exited) {
      ::kill(-pid, SIGKILL);
      ::waitpid(pid, &status, 0);
      detail = "timed out waiting for navigation stop command; log_file=" + navigation_stop_log_file_;
      return false;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      std::ostringstream out;
      out << "navigation stop command failed";
      if (WIFEXITED(status)) {
        out << " with exit code " << WEXITSTATUS(status);
      }
      out << "; log_file=" << navigation_stop_log_file_;
      detail = out.str();
      return false;
    }
    detail = "navigation runtime stack stopped; log_file=" + navigation_stop_log_file_;
    return true;
  }

  HttpResponse publish_estop(const bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    estop_pub_->publish(msg);
    return {202, "application/json", std::string("{\"ok\":true,\"estop\":") + (active ? "true" : "false") + "}"};
  }

  HttpResponse handle_switch_floor(const std::string & body)
  {
    auto floor_id = json_string_value(body, "floor_id");
    auto building_id = json_string_value(body, "building_id").value_or("building_1");
    const auto map_id = json_string_value(body, "map_id");
    const auto map_name = json_string_value(body, "map_name");
    const bool resume_navigation = json_bool_value(body, "resume_navigation", false);

    std::optional<MapManifest> selected_map;
    if (map_id && !map_id->empty()) {
      selected_map = find_map_by_id(*map_id);
      if (!selected_map) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (!safe_asset_id(building_id) || (floor_id && !safe_asset_id(*floor_id))) {
        return {400, "application/json", error_json("building_id/floor_id must be safe asset ids")};
      }
      if (json_string_value(body, "building_id") && building_id != selected_map->building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (floor_id && *floor_id != selected_map->floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      building_id = selected_map->building_id;
      floor_id = selected_map->floor_id;
    } else {
      if (!floor_id || floor_id->empty()) {
        return {400, "application/json", error_json("floor_id is required")};
      }
      if (!safe_asset_id(building_id) || !safe_asset_id(*floor_id)) {
        return {400, "application/json", error_json("building_id/floor_id must be safe asset ids")};
      }
      if (map_name && !map_name->empty()) {
        std::string error;
        selected_map = find_floor_map_by_name(building_id, *floor_id, *map_name, error);
        if (!error.empty()) {
          return {409, "application/json", error_json(error)};
        }
        if (!selected_map) {
          return {404, "application/json", error_json("map_name not found on requested floor: " + *map_name)};
        }
      } else {
        selected_map = active_floor_map(building_id, *floor_id);
      }
    }

    if (selected_map) {
      try {
        activate_map_manifest(*selected_map);
      } catch (const std::exception & exc) {
        return {500, "application/json", error_json(exc.what())};
      }
    }

    if (resume_navigation) {
      return handle_resume_floor_navigation(building_id, *floor_id, selected_map);
    }

    if (!floor_switch_client_->wait_for_service(service_timeout())) {
      return {503, "application/json", error_json("service unavailable: " + floor_switch_service_)};
    }
    auto request = std::make_shared<robot_interfaces::srv::SwitchFloor::Request>();
    request->building_id = building_id;
    request->floor_id = *floor_id;
    request->resume_navigation = resume_navigation;

    auto future = floor_switch_client_->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      return {503, "application/json", error_json("timed out waiting for floor switch")};
    }
    const auto response = future.get();
    std::ostringstream out;
    out << "{\"ok\":" << (response->success ? "true" : "false")
        << ",\"message\":" << json_string(response->message)
        << ",\"map_id\":" << json_string(selected_map ? selected_map->map_id : "")
        << ",\"display_name\":" << json_string(selected_map ? selected_map->display_name : "")
        << ",\"nav_map_yaml\":" << json_string(response->nav_map_yaml)
        << ",\"localizer_map_png\":" << json_string(response->localizer_map_png)
        << ",\"localizer_params_yaml\":" << json_string(response->localizer_params_yaml) << "}";
    return {response->success ? 200 : 500, "application/json", out.str()};
  }

  HttpResponse handle_trigger_localization(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("robot_api_server");
    if (!localization_trigger_client_->wait_for_service(service_timeout())) {
      return {503, "application/json", error_json("service unavailable: " + localization_trigger_service_)};
    }
    auto request = std::make_shared<robot_interfaces::srv::TriggerLocalization::Request>();
    request->reason = reason;

    auto future = localization_trigger_client_->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      return {503, "application/json", error_json("timed out waiting for localization trigger")};
    }
    const auto response = future.get();
    std::ostringstream out;
    out << "{\"ok\":" << (response->accepted ? "true" : "false")
        << ",\"message\":" << json_string(response->message) << "}";
    return {response->accepted ? 202 : 500, "application/json", out.str()};
  }

  std::chrono::nanoseconds docking_navigation_start_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(docking_navigation_start_wait_sec_));
  }

  bool docking_manager_process_running_locked()
  {
    if (docking_manager_pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(docking_manager_pid_, &status, WNOHANG);
    if (wait_result == docking_manager_pid_) {
      docking_manager_pid_ = -1;
      return false;
    }
    if (::kill(docking_manager_pid_, 0) == 0 || errno == EPERM) {
      return true;
    }
    docking_manager_pid_ = -1;
    return false;
  }

  bool ensure_docking_manager_running(std::string & detail)
  {
    if (docking_start_client_->wait_for_service(500ms)) {
      detail = "docking service already available";
      return true;
    }
    if (docking_manager_start_command_.empty() || !fs::exists(docking_manager_start_command_)) {
      detail = "docking manager start command is not available: " + docking_manager_start_command_;
      return false;
    }

    {
      std::lock_guard<std::mutex> process_lock(docking_manager_process_mutex_);
      if (!docking_manager_process_running_locked()) {
        const pid_t pid = ::fork();
        if (pid < 0) {
          detail = "failed to fork docking manager process";
          return false;
        }
        if (pid == 0) {
          ::setsid();
          const int log_fd = ::open(docking_manager_log_file_.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
          if (log_fd >= 0) {
            ::dup2(log_fd, STDOUT_FILENO);
            ::dup2(log_fd, STDERR_FILENO);
            ::close(log_fd);
          }
          close_inherited_fds();
          ::execl("/bin/bash", "bash", docking_manager_start_command_.c_str(), static_cast<char *>(nullptr));
          ::_exit(127);
        }
        docking_manager_pid_ = pid;
      }
    }

    const auto deadline = std::chrono::steady_clock::now() + service_timeout();
    while (std::chrono::steady_clock::now() < deadline) {
      if (docking_start_client_->wait_for_service(500ms)) {
        detail = "docking manager ready; log_file=" + docking_manager_log_file_;
        return true;
      }
    }
    detail = "timed out waiting for docking service; log_file=" + docking_manager_log_file_;
    return false;
  }

  bool call_docking_trigger_service(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & service_name,
    std::string & detail)
  {
    if (!client->wait_for_service(service_timeout())) {
      detail = "service unavailable: " + service_name;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      detail = "timed out waiting for service: " + service_name;
      return false;
    }
    const auto response = future.get();
    detail = response->message;
    return response->success;
  }

  bool docking_status_is_success(const std::string & status) const
  {
    return status.find("docked") != std::string::npos || status.find("charging") != std::string::npos;
  }

  bool docking_status_is_failure(const std::string & status) const
  {
    return status.find("failed") != std::string::npos || status.find("timeout") != std::string::npos ||
      status.find("outside hard limit") != std::string::npos;
  }

  bool docking_status_is_stopped(const std::string & status) const
  {
    return status.find("stopped") != std::string::npos;
  }

  void handle_docking_status(const std::string & status)
  {
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_status_ = status;
    }

    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    docking_job_.last_status = status;
    if (docking_job_.state != "running") {
      return;
    }
    if (docking_status_is_success(status)) {
      finish_docking_job_locked(true, "docked", status);
    } else if (docking_status_is_failure(status)) {
      finish_docking_job_locked(false, "failed", status);
    } else if (docking_status_is_stopped(status)) {
      finish_docking_job_locked(true, docking_job_.cancel_requested ? "canceled" : "stopped", status);
    } else {
      set_docking_runtime_state(true, "fine_docking", status);
    }
  }

  std::string docking_job_json_locked() const
  {
    std::ostringstream response;
    response << std::fixed << std::setprecision(6);
    response << "{\"id\":" << docking_job_.id << ","
             << "\"state\":" << json_string(docking_job_.state) << ","
             << "\"phase\":" << json_string(docking_job_.phase) << ","
             << "\"building_id\":" << json_string(docking_job_.building_id) << ","
             << "\"floor_id\":" << json_string(docking_job_.floor_id) << ","
             << "\"map_id\":" << json_string(docking_job_.map_id) << ","
             << "\"dock_id\":" << json_string(docking_job_.dock_id) << ","
             << "\"dock_name\":" << json_string(docking_job_.dock_name) << ","
             << "\"dock_type\":" << json_string(docking_job_.dock_type) << ","
             << "\"ok\":" << (docking_job_.ok ? "true" : "false") << ","
             << "\"resume_navigation\":" << (docking_job_.resume_navigation ? "true" : "false") << ","
             << "\"nav_goal_sent\":" << (docking_job_.nav_goal_sent ? "true" : "false") << ","
             << "\"nav_goal_succeeded\":" << (docking_job_.nav_goal_succeeded ? "true" : "false") << ","
             << "\"docking_service_called\":" << (docking_job_.docking_service_called ? "true" : "false") << ","
             << "\"cancel_requested\":" << (docking_job_.cancel_requested ? "true" : "false") << ","
             << "\"dock_pose\":{\"x\":" << docking_job_.dock_x << ",\"y\":" << docking_job_.dock_y
             << ",\"yaw\":" << docking_job_.dock_yaw << "},"
             << "\"approach_pose\":{\"x\":" << docking_job_.approach_x << ",\"y\":" << docking_job_.approach_y
             << ",\"yaw\":" << docking_job_.approach_yaw << "},"
             << "\"approach_distance_m\":" << docking_job_.approach_distance_m << ","
             << "\"detail\":" << json_string(docking_job_.detail) << ","
             << "\"last_status\":" << json_string(docking_job_.last_status) << ","
             << "\"started_at\":" << json_string(docking_job_.started_at) << ","
             << "\"finished_at\":" << json_string(docking_job_.finished_at) << "}";
    return response.str();
  }

  void set_docking_job_phase(const std::uint64_t job_id, const std::string & phase)
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id == job_id && docking_job_.state == "running") {
      docking_job_.phase = phase;
    }
  }

  bool docking_cancel_requested(const std::uint64_t job_id)
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    return docking_job_.id == job_id && docking_job_.cancel_requested;
  }

  void finish_docking_job_locked(const bool ok, const std::string & final_state, const std::string & detail)
  {
    docking_job_.state = final_state;
    docking_job_.phase = "finished";
    docking_job_.ok = ok;
    docking_job_.detail = detail;
    docking_job_.finished_at = utc_timestamp_iso8601();
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_active_ = false;
      docking_runtime_state_ = final_state;
      docking_runtime_status_ = detail;
      runtime_healthy_ = true;
      runtime_message_ = detail;
      if (final_state == "docked") {
        navigation_runtime_active_ = false;
        navigation_runtime_state_ = "stopped";
      }
    }
  }

  void finish_docking_job(const std::uint64_t job_id, const bool ok, const std::string & final_state, const std::string & detail)
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id != job_id) {
      return;
    }
    finish_docking_job_locked(ok, final_state, detail);
  }

  void mark_docking_nav_goal_sent(
    const std::uint64_t job_id,
    const NavigateGoalHandle::SharedPtr & goal_handle,
    const std::string & building_id,
    const std::string & floor_id)
  {
    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_nav_goal_handle_ = goal_handle;
      active_nav_goal_pose_id_ = "";
      active_nav_goal_building_id_ = building_id;
      active_nav_goal_floor_id_ = floor_id;
    }
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.nav_goal_sent = true;
      }
    }
  }

  void run_docking_job(const std::uint64_t job_id)
  {
    DockingJob job;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id != job_id) {
        return;
      }
      job = docking_job_;
    }

    if (job.resume_navigation) {
      bool action_available = false;
      {
        std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
        action_available = navigate_to_pose_client_->wait_for_action_server(500ms);
      }
      if (!action_available) {
        set_docking_job_phase(job_id, "resume_navigation_stack");
        std::optional<MapManifest> selected_map;
        if (!job.map_id.empty()) {
          selected_map = find_map_by_id(job.map_id);
        } else {
          selected_map = active_floor_map(job.building_id, job.floor_id);
        }
        const auto resume_response = handle_resume_floor_navigation(job.building_id, job.floor_id, selected_map);
        if (resume_response.status >= 400) {
          finish_docking_job(job_id, false, "failed", "failed to start navigation runtime: " + resume_response.body);
          return;
        }
      }
    }

    set_docking_job_phase(job_id, "wait_for_nav2_action");
    const auto action_deadline = std::chrono::steady_clock::now() + docking_navigation_start_timeout();
    bool action_available = false;
    while (std::chrono::steady_clock::now() < action_deadline) {
      if (docking_cancel_requested(job_id)) {
        finish_docking_job(job_id, true, "canceled", "docking canceled before approach navigation");
        return;
      }
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      if (navigate_to_pose_client_->wait_for_action_server(500ms)) {
        action_available = true;
        break;
      }
    }
    if (!action_available) {
      finish_docking_job(job_id, false, "failed", "action unavailable: " + navigate_to_pose_action_);
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = job.approach_x;
    goal.pose.pose.position.y = job.approach_y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.z = std::sin(job.approach_yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(job.approach_yaw * 0.5);

    NavigateGoalHandle::SharedPtr goal_handle;
    try {
      set_docking_job_phase(job_id, "send_approach_goal");
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      auto future = navigate_to_pose_client_->async_send_goal(goal);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        finish_docking_job(job_id, false, "failed", "timed out sending predock navigation goal");
        return;
      }
      goal_handle = future.get();
    } catch (const std::exception & exc) {
      finish_docking_job(job_id, false, "failed", std::string("exception sending predock goal: ") + exc.what());
      return;
    } catch (...) {
      finish_docking_job(job_id, false, "failed", "unknown exception sending predock goal");
      return;
    }
    if (!goal_handle) {
      finish_docking_job(job_id, false, "failed", "predock navigation goal rejected by Nav2");
      return;
    }
    mark_docking_nav_goal_sent(job_id, goal_handle, job.building_id, job.floor_id);
    set_navigation_runtime_state(true, "navigating", "docking predock navigation accepted");
    set_docking_runtime_state(true, "nav_to_predock", "navigating to docking approach pose");

    auto result_future = navigate_to_pose_client_->async_get_result(goal_handle);
    set_docking_job_phase(job_id, "nav_to_predock");
    const auto predock_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(docking_predock_nav_timeout_sec_));
    while (result_future.wait_for(200ms) != std::future_status::ready) {
      if (docking_cancel_requested(job_id)) {
        std::string cancel_detail;
        cancel_active_navigation_goal(cancel_detail);
        finish_docking_job(job_id, true, "canceled", cancel_detail);
        return;
      }
      if (std::chrono::steady_clock::now() > predock_deadline) {
        std::string cancel_detail;
        cancel_active_navigation_goal(cancel_detail);
        finish_docking_job(job_id, false, "failed", "timed out navigating to predock pose; " + cancel_detail);
        return;
      }
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      std::ostringstream detail;
      detail << "predock navigation failed with result code " << static_cast<int>(result.code);
      finish_docking_job(job_id, false, "failed", detail.str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.nav_goal_succeeded = true;
      }
    }
    set_navigation_runtime_state(true, "ready", "predock navigation reached");

    set_docking_job_phase(job_id, "start_fine_docking");
    if (docking_cancel_requested(job_id)) {
      finish_docking_job(job_id, true, "canceled", "docking canceled before GS2 fine docking");
      return;
    }
    std::string ensure_detail;
    if (!ensure_docking_manager_running(ensure_detail)) {
      finish_docking_job(job_id, false, "failed", ensure_detail);
      return;
    }
    std::string service_detail;
    if (!call_docking_trigger_service(docking_start_client_, docking_start_service_, service_detail)) {
      finish_docking_job(job_id, false, "failed", service_detail);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.docking_service_called = true;
        docking_job_.phase = "fine_docking_active";
        docking_job_.detail = service_detail;
      }
    }
    set_docking_runtime_state(true, "fine_docking", service_detail);
  }

  HttpResponse handle_docking_start(const std::string & body)
  {
    auto building_id = json_string_value(body, "building_id").value_or("building_1");
    auto floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    const auto map_name = json_string_value(body, "map_name");
    const auto dock_id = json_string_value(body, "dock_id").value_or(
      json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or("")));
    const bool resume_navigation = json_bool_value(body, "resume_navigation", true);
    const double approach_distance = std::clamp(
      json_number_value(body, "approach_distance_m").value_or(docking_pre_dock_distance_m_),
      0.10,
      2.00);

    if (!floor_id || floor_id->empty()) {
      return {400, "application/json", error_json("floor_id is required")};
    }
    if (!safe_asset_id(building_id) || !safe_asset_id(*floor_id)) {
      return {400, "application/json", error_json("building_id/floor_id must be safe asset ids")};
    }
    if (!safe_pose_id(dock_id)) {
      return {400, "application/json", error_json("valid dock_id is required")};
    }

    std::optional<MapManifest> selected_map;
    try {
      if (map_id && !map_id->empty()) {
        selected_map = find_map_by_id(*map_id);
        if (!selected_map) {
          return {404, "application/json", error_json("map_id not found: " + *map_id)};
        }
        building_id = selected_map->building_id;
        floor_id = selected_map->floor_id;
        activate_map_manifest(*selected_map);
      } else if (map_name && !map_name->empty()) {
        std::string error;
        selected_map = find_floor_map_by_name(building_id, *floor_id, *map_name, error);
        if (!error.empty()) {
          return {409, "application/json", error_json(error)};
        }
        if (!selected_map) {
          return {404, "application/json", error_json("map_name not found on requested floor: " + *map_name)};
        }
        activate_map_manifest(*selected_map);
      } else {
        selected_map = active_floor_map(building_id, *floor_id);
      }
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    const auto pose = find_floor_pose(building_id, *floor_id, dock_id);
    if (!pose) {
      return {404, "application/json", error_json("dock_id not found in poses.yaml: " + dock_id)};
    }

    DockingJob next_job;
    next_job.id = 0U;
    next_job.state = "running";
    next_job.phase = "accepted";
    next_job.building_id = building_id;
    next_job.floor_id = *floor_id;
    next_job.map_id = selected_map ? selected_map->map_id : "";
    next_job.dock_id = dock_id;
    next_job.dock_name = pose->name;
    next_job.dock_type = pose->type;
    next_job.started_at = utc_timestamp_iso8601();
    next_job.resume_navigation = resume_navigation;
    next_job.dock_x = pose->x;
    next_job.dock_y = pose->y;
    next_job.dock_yaw = normalize_angle(pose->yaw);
    next_job.approach_distance_m = approach_distance;
    next_job.approach_x = next_job.dock_x - std::cos(next_job.dock_yaw) * approach_distance;
    next_job.approach_y = next_job.dock_y - std::sin(next_job.dock_yaw) * approach_distance;
    next_job.approach_yaw = next_job.dock_yaw;

    std::lock_guard<std::mutex> start_lock(docking_start_mutex_);
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.state == "running") {
        std::ostringstream response;
        response << "{\"ok\":true,\"accepted\":true,\"already_running\":true,"
                 << "\"docking\":" << docking_job_json_locked() << "}";
        return {202, "application/json", response.str()};
      }
    }
    join_docking_worker();
    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      job_id = ++docking_job_seq_;
      next_job.id = job_id;
      docking_job_ = next_job;
    }
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_active_ = true;
      docking_runtime_state_ = "accepted";
      docking_runtime_dock_id_ = dock_id;
      docking_runtime_status_.clear();
      runtime_healthy_ = true;
      runtime_message_ = "docking accepted";
      mapping_runtime_active_ = false;
      mapping_runtime_state_ = "stopped";
    }

    try {
      docking_worker_ = std::thread([this, job_id]() { run_docking_job(job_id); });
    } catch (const std::exception & exc) {
      finish_docking_job(job_id, false, "failed", std::string("failed to start docking worker: ") + exc.what());
      return {
        500,
        "application/json",
        error_json(std::string("failed to start docking worker: ") + exc.what())};
    }

    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,\"docking\":" << docking_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  HttpResponse handle_docking_cancel(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("app_docking_cancel");
    clear_teleop_command();
    publish_teleop_zero_burst();
    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      docking_job_.cancel_requested = true;
      docking_job_.detail = reason;
      job_id = docking_job_.id;
    }
    std::string nav_detail;
    cancel_active_navigation_goal(nav_detail);
    std::string stop_detail;
    if (docking_stop_client_->wait_for_service(500ms)) {
      call_docking_trigger_service(docking_stop_client_, docking_stop_service_, stop_detail);
    } else {
      stop_detail = "docking stop service not available";
    }
    publish_teleop_zero_burst();
    if (job_id != 0U) {
      finish_docking_job(job_id, true, "canceled", reason + "; " + nav_detail + "; " + stop_detail);
    } else {
      set_docking_runtime_state(false, "canceled", reason + "; " + stop_detail);
    }
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,\"navigation_cancel_detail\":"
             << json_string(nav_detail) << ",\"docking_stop_detail\":" << json_string(stop_detail)
             << ",\"docking\":" << docking_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  HttpResponse handle_docking_state()
  {
    const auto runtime = runtime_mode_snapshot();
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"mode\":" << json_string(runtime.mode) << ","
             << "\"state\":" << json_string(runtime.docking_state) << ","
             << "\"docking_active\":" << (runtime.docking_active ? "true" : "false") << ","
             << "\"last_status\":" << json_string(runtime.docking_status) << ","
             << "\"docking\":" << docking_job_json_locked() << "}";
    return {200, "application/json", response.str()};
  }

  void join_docking_worker()
  {
    if (docking_worker_.joinable()) {
      docking_worker_.join();
    }
  }

  HttpResponse not_wired(const std::string & endpoint)
  {
    return {
      501,
      "application/json",
      "{\"ok\":false,\"error\":\"endpoint is reserved but not wired to a ROS-native service/action yet\","
      "\"endpoint\":" + json_string(endpoint) + "}"
    };
  }

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

  bool send_all_text(const int client_fd, const std::string & text)
  {
    return send_all_bytes(client_fd, text.data(), text.size());
  }

  bool recv_exact(const int client_fd, void * data, const std::size_t length)
  {
    char * cursor = static_cast<char *>(data);
    std::size_t received = 0;
    while (received < length && running_.load()) {
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

  void set_socket_receive_timeout(const int client_fd, const double timeout_sec) const
  {
    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(timeout_sec);
    timeout.tv_usec = static_cast<suseconds_t>(
      std::max(0.0, timeout_sec - static_cast<double>(timeout.tv_sec)) * 1000000.0);
    ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  bool websocket_headers_valid(const HttpRequest & request) const
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

  void send_websocket_handshake(const int client_fd, const HttpRequest & request)
  {
    const auto key = request.headers.at("sec-websocket-key");
    std::ostringstream out;
    out << "HTTP/1.1 101 Switching Protocols\r\n";
    out << "Upgrade: websocket\r\n";
    out << "Connection: Upgrade\r\n";
    out << "Sec-WebSocket-Accept: " << websocket_accept_key(key) << "\r\n";
    out << "Access-Control-Allow-Origin: *\r\n";
    out << "\r\n";
    send_all_text(client_fd, out.str());
  }

  bool send_websocket_frame(
    const int client_fd,
    const std::string & payload,
    const std::uint8_t opcode = 0x1)
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
    if (!payload.empty() && !send_all_bytes(client_fd, payload.data(), payload.size())) {
      return false;
    }
    return true;
  }

  std::optional<WebSocketFrame> read_websocket_frame(const int client_fd)
  {
    std::uint8_t header[2]{};
    if (!recv_exact(client_fd, header, sizeof(header))) {
      return std::nullopt;
    }

    const std::uint8_t opcode = header[0] & 0x0FU;
    const bool masked = (header[1] & 0x80U) != 0U;
    std::uint64_t payload_length = header[1] & 0x7FU;
    if (payload_length == 126U) {
      std::uint8_t extended[2]{};
      if (!recv_exact(client_fd, extended, sizeof(extended))) {
        return std::nullopt;
      }
      payload_length = (static_cast<std::uint64_t>(extended[0]) << 8U) |
        static_cast<std::uint64_t>(extended[1]);
    } else if (payload_length == 127U) {
      std::uint8_t extended[8]{};
      if (!recv_exact(client_fd, extended, sizeof(extended))) {
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
    if (masked && !recv_exact(client_fd, mask, sizeof(mask))) {
      return std::nullopt;
    }

    std::string payload;
    payload.resize(static_cast<std::size_t>(payload_length));
    if (payload_length > 0U && !recv_exact(client_fd, payload.data(), payload.size())) {
      return std::nullopt;
    }
    if (masked) {
      for (std::size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<char>(static_cast<std::uint8_t>(payload[i]) ^ mask[i % 4U]);
      }
    }
    return WebSocketFrame{opcode, payload};
  }

  void publish_teleop_zero()
  {
    geometry_msgs::msg::Twist twist;
    teleop_cmd_pub_->publish(twist);
  }

  void publish_teleop_zero_burst()
  {
    for (int i = 0; i < 8; ++i) {
      publish_teleop_zero();
      std::this_thread::sleep_for(50ms);
    }
  }

  void publish_teleop_reverse_enable(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled && teleop_allow_reverse_;
    teleop_reverse_enable_pub_->publish(msg);
  }

  bool battery_indicates_charging(const sensor_msgs::msg::BatteryState & msg) const
  {
    return msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING ||
      msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL ||
      (std::isfinite(msg.current) && msg.current > teleop_charging_current_min_a_);
  }

  bool teleop_charging_guard_active()
  {
    if (!teleop_stop_on_charging_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_bms_state_) {
      return false;
    }
    const double age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_bms_received_at_).count();
    if (age_sec > bms_state_max_age_sec_) {
      return false;
    }
    return latest_bms_power_supply_status_ ==
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING ||
      latest_bms_power_supply_status_ == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL ||
      (std::isfinite(latest_bms_current_) && latest_bms_current_ > teleop_charging_current_min_a_);
  }

  void mark_teleop_session_started()
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    ++teleop_session_count_;
  }

  void mark_teleop_session_stopped()
  {
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      if (teleop_session_count_ > 0) {
        --teleop_session_count_;
      }
      if (teleop_session_count_ > 0) {
        return;
      }
      teleop_command_active_ = false;
      teleop_zero_sent_ = true;
      latest_teleop_cmd_ = geometry_msgs::msg::Twist{};
    }
    publish_teleop_zero();
    publish_teleop_reverse_enable(false);
  }

  void store_teleop_command(const geometry_msgs::msg::Twist & twist)
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    latest_teleop_cmd_ = twist;
    latest_teleop_cmd_at_ = std::chrono::steady_clock::now();
    teleop_command_active_ = true;
    teleop_zero_sent_ = false;
  }

  void clear_teleop_command()
  {
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      latest_teleop_cmd_ = geometry_msgs::msg::Twist{};
      teleop_command_active_ = false;
      teleop_zero_sent_ = true;
    }
    publish_teleop_zero();
    publish_teleop_reverse_enable(false);
  }

  void on_teleop_repeat_timer()
  {
    if (teleop_charging_guard_active()) {
      clear_teleop_command();
      return;
    }

    geometry_msgs::msg::Twist twist;
    bool should_publish_cmd = false;
    bool should_publish_zero = false;
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      if (teleop_session_count_ == 0) {
        return;
      }

      const auto now = std::chrono::steady_clock::now();
      const bool command_fresh = teleop_command_active_ &&
        latest_teleop_cmd_at_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - latest_teleop_cmd_at_).count() <= teleop_watchdog_timeout_sec_;
      if (command_fresh) {
        twist = latest_teleop_cmd_;
        should_publish_cmd = true;
      } else if (!teleop_zero_sent_) {
        teleop_command_active_ = false;
        teleop_zero_sent_ = true;
        latest_teleop_cmd_ = geometry_msgs::msg::Twist{};
        should_publish_zero = true;
      }
    }

    if (should_publish_cmd) {
      publish_teleop_reverse_enable(teleop_allow_reverse_);
      teleop_cmd_pub_->publish(twist);
    } else if (should_publish_zero) {
      publish_teleop_zero();
      publish_teleop_reverse_enable(false);
    }
  }

  bool mapping_2d_active_now()
  {
    std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
    if (mapping_2d_process_running_locked()) {
      return true;
    }
    if (mapping_2d_pid_ <= 0 && !discover_mapping_2d_process_groups().empty()) {
      mapping_2d_active_ = true;
      return true;
    }
    return mapping_2d_active_;
  }

  bool teleop_session_allowed(std::string & reason)
  {
    if (!teleop_require_mapping_active_) {
      return true;
    }
    if (mapping_2d_active_now()) {
      return true;
    }
    reason = "WebSocket teleop is only allowed while 2D mapping is active";
    return false;
  }

  std::string teleop_state_json()
  {
    const bool mapping_active = mapping_2d_active_now();

    bool map_available = false;
    double map_age_sec = 0.0;
    double area_m2 = 0.0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    double resolution = 0.0;
    {
      std::lock_guard<std::mutex> lock(live_map_mutex_);
      if (have_live_map_) {
        map_available = true;
        const auto now = std::chrono::steady_clock::now();
        map_age_sec = std::chrono::duration<double>(now - latest_live_map_received_at_).count();
        width = latest_live_map_.info.width;
        height = latest_live_map_.info.height;
        resolution = latest_live_map_.info.resolution;
        const auto known_cells =
          std::count_if(latest_live_map_.data.begin(), latest_live_map_.data.end(), [](const int8_t value) {
            return value >= 0;
          });
        area_m2 = static_cast<double>(known_cells) * resolution * resolution;
      }
    }

    bool pose_available = false;
    std::string pose_frame;
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_yaw = 0.0;
    double pose_age_sec = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (have_pose_) {
        pose_frame = latest_pose_frame_;
        pose_x = latest_pose_x_;
        pose_y = latest_pose_y_;
        pose_yaw = latest_pose_yaw_;
        pose_age_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_pose_received_at_).count();
        pose_available = pose_age_sec <= tf_pose_max_age_sec_;
      }
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\"ok\":true,\"type\":\"mapping_state\","
        << "\"state\":" << json_string(mapping_active ? "running" : "stopped") << ","
        << "\"teleop_allowed\":" << (mapping_active || !teleop_require_mapping_active_ ? "true" : "false") << ","
        << "\"allow_reverse\":" << (teleop_allow_reverse_ ? "true" : "false") << ","
        << "\"map_available\":" << (map_available ? "true" : "false");
    if (map_available) {
      out << ",\"map_age_sec\":" << map_age_sec
          << ",\"area_m2\":" << area_m2
          << ",\"map\":{\"width\":" << width
          << ",\"height\":" << height
          << ",\"resolution\":" << resolution << "}";
    }
    if (pose_available) {
      out << ",\"pose\":{\"frame_id\":" << json_string(pose_frame)
          << ",\"x\":" << pose_x
          << ",\"y\":" << pose_y
          << ",\"yaw\":" << pose_yaw
          << ",\"age_sec\":" << pose_age_sec << "}";
    }
    out << "}";
    return out.str();
  }

  bool publish_teleop_command(const std::string & payload, std::string & ack_json)
  {
    const auto message_type = json_string_value(payload, "type").value_or("cmd_vel");
    if (message_type == "stop") {
      clear_teleop_command();
      ack_json = "{\"ok\":true,\"type\":\"teleop_stopped\"}";
      return true;
    }
    if (message_type != "cmd_vel") {
      ack_json = error_json("unsupported teleop message type: " + message_type);
      return false;
    }

    const double raw_linear_x = json_number_value(payload, "linear_x")
                                  .value_or(json_number_value(payload, "linearX")
                                              .value_or(json_number_value(payload, "vx")
                                                          .value_or(json_nested_number_value(payload, "linear", "x")
                                                                      .value_or(0.0))));
    const double raw_angular_z = json_number_value(payload, "angular_z")
                                  .value_or(json_number_value(payload, "angularZ")
                                              .value_or(json_number_value(payload, "wz")
                                                          .value_or(json_nested_number_value(payload, "angular", "z")
                                                                      .value_or(0.0))));
    if (!std::isfinite(raw_linear_x) || !std::isfinite(raw_angular_z)) {
      ack_json = error_json("linear_x/angular_z must be finite numbers");
      return false;
    }

    std::string reject_reason;
    if (!teleop_session_allowed(reject_reason)) {
      clear_teleop_command();
      ack_json = error_json(reject_reason);
      return false;
    }
    if (teleop_charging_guard_active()) {
      clear_teleop_command();
      ack_json = error_json("charging detected; teleop command stopped");
      return false;
    }

    const double min_linear_x = teleop_allow_reverse_ ? -teleop_max_linear_x_mps_ : 0.0;
    const double linear_x = std::clamp(raw_linear_x, min_linear_x, teleop_max_linear_x_mps_);
    const double angular_z =
      std::clamp(raw_angular_z, -teleop_max_angular_z_radps_, teleop_max_angular_z_radps_);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    store_teleop_command(twist);
    publish_teleop_reverse_enable(teleop_allow_reverse_);
    teleop_cmd_pub_->publish(twist);

    std::ostringstream out;
    out << "{\"ok\":true,\"type\":\"cmd_vel_ack\","
        << "\"linear_x\":" << linear_x << ","
        << "\"angular_z\":" << angular_z << ","
        << "\"allow_reverse\":" << (teleop_allow_reverse_ ? "true" : "false") << ","
        << "\"cmd_topic\":" << json_string(teleop_cmd_topic_) << "}";
    ack_json = out.str();
    return true;
  }

  void handle_teleop_websocket(const int client_fd, const HttpRequest & request)
  {
    if (!token_allowed(request)) {
      send_response(client_fd, {401, "application/json", error_json("missing or invalid X-Robot-Token")});
      return;
    }
    if (!websocket_headers_valid(request)) {
      send_response(client_fd, {400, "application/json", error_json("invalid websocket upgrade request")});
      return;
    }
    std::string reject_reason;
    if (!teleop_session_allowed(reject_reason)) {
      send_response(client_fd, {409, "application/json", error_json(reject_reason)});
      return;
    }

    send_websocket_handshake(client_fd, request);
    const std::string websocket_client_id = "websocket:" + std::to_string(client_fd);
    const int websocket_ttl_ms = std::clamp(
      static_cast<int>((teleop_socket_idle_timeout_sec_ + 1.0) * 1000.0),
      1000,
      subscription_max_ttl_ms_);
    if (subscription_manager_) {
      subscription_manager_->acquire(
        websocket_client_id,
        {"teleop", "tf"},
        std::chrono::milliseconds(websocket_ttl_ms));
    }
    mark_teleop_session_started();
    set_socket_receive_timeout(client_fd, teleop_socket_idle_timeout_sec_);
    publish_teleop_reverse_enable(teleop_allow_reverse_);
    {
      std::ostringstream ready;
      ready << "{\"ok\":true,\"type\":\"teleop_ready\","
            << "\"cmd_topic\":" << json_string(teleop_cmd_topic_) << ","
            << "\"reverse_enable_topic\":" << json_string(teleop_reverse_enable_topic_) << ","
            << "\"max_linear_x_mps\":" << teleop_max_linear_x_mps_ << ","
            << "\"max_angular_z_radps\":" << teleop_max_angular_z_radps_ << ","
            << "\"watchdog_timeout_sec\":" << teleop_watchdog_timeout_sec_ << ","
            << "\"socket_idle_timeout_sec\":" << teleop_socket_idle_timeout_sec_ << ","
            << "\"repeat_rate_hz\":" << teleop_repeat_rate_hz_ << ","
            << "\"require_mapping_active\":" << (teleop_require_mapping_active_ ? "true" : "false") << ","
            << "\"allow_reverse\":" << (teleop_allow_reverse_ ? "true" : "false") << "}";
      send_websocket_frame(client_fd, ready.str());
      send_websocket_frame(client_fd, teleop_state_json());
    }

    while (running_.load()) {
      const auto frame = read_websocket_frame(client_fd);
      if (!frame) {
        break;
      }
      if (frame->opcode == 0x8U) {
        send_websocket_frame(client_fd, "", 0x8U);
        break;
      }
      if (frame->opcode == 0x9U) {
        send_websocket_frame(client_fd, frame->payload, 0xAU);
        continue;
      }
      if (frame->opcode != 0x1U) {
        send_websocket_frame(client_fd, error_json("only text websocket frames are accepted"));
        continue;
      }

      std::string ack;
      if (subscription_manager_) {
        subscription_manager_->acquire(
          websocket_client_id,
          {"teleop", "tf"},
          std::chrono::milliseconds(websocket_ttl_ms));
      }
      publish_teleop_command(frame->payload, ack);
      if (!send_websocket_frame(client_fd, ack)) {
        break;
      }
      if (!send_websocket_frame(client_fd, teleop_state_json())) {
        break;
      }
    }

    mark_teleop_session_stopped();
    if (subscription_manager_) {
      subscription_manager_->release(websocket_client_id, {"teleop", "tf"});
    }
  }

  std::string host_;
  int port_{8080};
  std::string api_token_;
  int max_http_connections_{32};
  std::string maps_root_;
  std::string runtime_maps_dir_;
  std::string safety_estop_topic_;
  std::string safety_status_topic_;
  std::string safety_motion_allowed_topic_;
  std::string floor_status_topic_;
  std::string bms_state_topic_;
  double bms_state_max_age_sec_{3.0};
  std::string floor_switch_service_;
  std::string localization_trigger_service_;
  std::string navigate_to_pose_action_;
  std::string mapping_2d_start_command_;
  std::string mapping_2d_log_file_;
  std::string navigation_resume_command_;
  std::string navigation_resume_log_file_;
  std::string runtime_map_context_file_;
  std::string navigation_stop_command_;
  std::string navigation_stop_log_file_;
  std::string docking_manager_start_command_;
  std::string docking_manager_log_file_;
  std::string docking_start_service_;
  std::string docking_stop_service_;
  std::string docking_status_topic_;
  double docking_pre_dock_distance_m_{0.80};
  double docking_navigation_start_wait_sec_{45.0};
  double docking_predock_nav_timeout_sec_{180.0};
  std::string mapping_2d_live_map_topic_;
  double mapping_2d_live_map_max_age_sec_{3.0};
  std::string scan_topic_;
  double scan_max_age_sec_{2.0};
  std::string tf_topic_;
  std::string tf_map_frame_{"map"};
  std::string tf_odom_frame_{"odom"};
  std::string tf_base_frame_{"base_link"};
  double tf_pose_max_age_sec_{2.0};
  double robot_pose_freshness_sec_{0.5};
  std::string teleop_cmd_topic_;
  std::string teleop_reverse_enable_topic_;
  std::string teleop_pose_topic_;
  double teleop_max_linear_x_mps_{0.30};
  double teleop_max_angular_z_radps_{0.55};
  bool teleop_allow_reverse_{false};
  bool teleop_require_mapping_active_{true};
  bool teleop_stop_on_charging_{true};
  double teleop_charging_current_min_a_{0.10};
  double teleop_watchdog_timeout_sec_{0.5};
  double teleop_socket_idle_timeout_sec_{5.0};
  double teleop_repeat_rate_hz_{20.0};
  int subscription_default_ttl_ms_{10000};
  int subscription_max_ttl_ms_{60000};
  double service_timeout_sec_{8.0};

  std::atomic<bool> running_{false};
  std::atomic<int> active_http_connections_{0};
  int server_fd_{-1};
  std::thread server_thread_;
  std::unique_ptr<SubscriptionManager> subscription_manager_;
  rclcpp::TimerBase::SharedPtr subscription_ttl_timer_;
  std::mutex subscription_lifecycle_mutex_;

  std::mutex state_mutex_;
  std::string latest_safety_status_{"UNKNOWN"};
  std::string latest_floor_status_{"UNKNOWN"};
  bool latest_motion_allowed_{false};
  bool have_motion_allowed_{false};
  bool have_bms_state_{false};
  bool have_bms_soc_{false};
  double latest_bms_soc_{0.0};
  double latest_bms_voltage_{0.0};
  double latest_bms_current_{0.0};
  double latest_bms_temperature_{0.0};
  int latest_bms_power_supply_status_{sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN};
  std::chrono::steady_clock::time_point latest_bms_received_at_{};
  std::string latest_pose_frame_;
  double latest_pose_x_{0.0};
  double latest_pose_y_{0.0};
  double latest_pose_yaw_{0.0};
  double latest_pose_stamp_sec_{0.0};
  bool have_pose_{false};
  std::chrono::steady_clock::time_point latest_pose_received_at_{};
  bool have_map_to_odom_{false};
  double latest_map_to_odom_x_{0.0};
  double latest_map_to_odom_y_{0.0};
  double latest_map_to_odom_yaw_{0.0};
  double latest_map_to_odom_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_map_to_odom_received_at_{};
  bool have_odom_to_base_{false};
  double latest_odom_to_base_x_{0.0};
  double latest_odom_to_base_y_{0.0};
  double latest_odom_to_base_yaw_{0.0};
  double latest_odom_to_base_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_odom_to_base_received_at_{};
  bool have_scan_{false};
  std::string latest_scan_frame_;
  std::size_t latest_scan_range_count_{0U};
  double latest_scan_angle_min_{0.0};
  double latest_scan_angle_max_{0.0};
  std::chrono::steady_clock::time_point latest_scan_received_at_{};

  mutable std::mutex runtime_mode_mutex_;
  bool mapping_runtime_active_{false};
  bool navigation_runtime_active_{false};
  bool docking_runtime_active_{false};
  bool runtime_healthy_{true};
  std::string mapping_runtime_state_{"stopped"};
  std::string navigation_runtime_state_{"stopped"};
  std::string docking_runtime_state_{"stopped"};
  std::string docking_runtime_status_;
  std::string docking_runtime_dock_id_;
  std::string runtime_message_;

  std::mutex teleop_mutex_;
  int teleop_session_count_{0};
  geometry_msgs::msg::Twist latest_teleop_cmd_;
  bool teleop_command_active_{false};
  bool teleop_zero_sent_{true};
  std::chrono::steady_clock::time_point latest_teleop_cmd_at_{};

  std::mutex mapping_process_mutex_;
  pid_t mapping_2d_pid_{-1};
  bool mapping_2d_active_{false};
  std::chrono::steady_clock::time_point mapping_2d_started_at_{};

  std::mutex navigation_process_mutex_;
  pid_t navigation_resume_pid_{-1};
  std::mutex docking_manager_process_mutex_;
  pid_t docking_manager_pid_{-1};
  std::mutex navigation_cancel_start_mutex_;
  std::mutex navigation_cancel_job_mutex_;
  NavigationCancelJob navigation_cancel_job_;
  std::uint64_t navigation_cancel_job_seq_{0U};
  std::thread navigation_cancel_worker_;
  std::mutex docking_start_mutex_;
  std::mutex docking_job_mutex_;
  DockingJob docking_job_;
  std::uint64_t docking_job_seq_{0U};
  std::thread docking_worker_;

  std::mutex navigate_action_mutex_;
  std::mutex active_nav_goal_mutex_;
  NavigateGoalHandle::SharedPtr active_nav_goal_handle_;
  std::string active_nav_goal_pose_id_;
  std::string active_nav_goal_building_id_;
  std::string active_nav_goal_floor_id_;

  std::mutex live_map_mutex_;
  nav_msgs::msg::OccupancyGrid latest_live_map_;
  bool have_live_map_{false};
  std::chrono::steady_clock::time_point latest_live_map_received_at_{};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_reverse_enable_pub_;
  rclcpp::TimerBase::SharedPtr teleop_repeat_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_allowed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr floor_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bms_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr docking_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr live_map_sub_;
  rclcpp::Client<robot_interfaces::srv::SwitchFloor>::SharedPtr floor_switch_client_;
  rclcpp::Client<robot_interfaces::srv::TriggerLocalization>::SharedPtr localization_trigger_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_stop_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<RobotApiServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok()) {
      try {
        executor.spin();
        break;
      } catch (const std::runtime_error & exc) {
        if (!is_transient_action_client_exception(exc)) {
          throw;
        }
        RCLCPP_ERROR(
          node->get_logger(),
          "continuing after transient action client executor exception: %s",
          exc.what());
        std::this_thread::sleep_for(100ms);
      }
    }
  } catch (const std::exception & exc) {
    std::cerr << "robot_api_server fatal exception: " << exc.what() << std::endl;
    exit_code = 1;
  } catch (...) {
    std::cerr << "robot_api_server unknown fatal exception" << std::endl;
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}

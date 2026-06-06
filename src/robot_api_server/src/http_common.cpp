#include "robot_api_server/http_common.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <sstream>

namespace robot_api_server
{

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

namespace
{

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

}  // namespace

std::string websocket_accept_key(const std::string & client_key)
{
  static const std::string magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  const auto digest = sha1_hash(client_key + magic);
  return base64_encode(std::vector<std::uint8_t>(digest.begin(), digest.end()));
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

}  // namespace robot_api_server

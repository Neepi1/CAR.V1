#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace robot_api_server
{

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

std::string trim(std::string value);
std::string lower_copy(std::string value);
std::string strip_query(const std::string & path);
std::map<std::string, std::string> parse_query_params(const std::string & path);
std::string reason_phrase(int status);

std::string json_escape(const std::string & value);
std::string json_string(const std::string & value);
std::string error_json(const std::string & error);
std::optional<std::string> json_string_value(const std::string & body, const std::string & key);
std::vector<std::string> json_string_array_value(const std::string & body, const std::string & key);
bool json_bool_value(const std::string & body, const std::string & key, bool default_value);
std::optional<double> json_number_value(const std::string & body, const std::string & key);
std::optional<std::string> json_object_value(const std::string & body, const std::string & key);
std::vector<std::string> json_object_array_value(const std::string & body, const std::string & key);
std::optional<double> json_nested_number_value(
  const std::string & body,
  const std::string & object_key,
  const std::string & number_key);

std::string websocket_accept_key(const std::string & client_key);
std::optional<HttpRequest> parse_http_request(const std::string & raw);
std::size_t content_length_from_headers(const std::string & raw_headers);

}  // namespace robot_api_server

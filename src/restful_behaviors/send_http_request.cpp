// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/restful_behaviors/send_http_request.hpp>

#include <curl/curl.h>
#include <behaviortree_cpp/contrib/json.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace
{
inline constexpr auto kDescriptionSendHttpRequest = R"(
                <p>
                    Sends one HTTP request to a web service (a REST API) and outputs the response. Set
                    <code>method</code> to <code>GET</code>, <code>POST</code>, <code>PUT</code>,
                    <code>PATCH</code> or <code>DELETE</code>. Use it to read from or write to a system that
                    does not use ROS, for example a fleet manager, a PLC gateway or a dashboard.
                </p>
                <p>
                    <code>query_parameters</code> and <code>headers</code> are JSON objects written as text,
                    for example <code>{"id": "42"}</code>. <code>body</code> is sent as it is, with
                    <code>Content-Type</code> set from <code>content_type</code>. The response body is written
                    to <code>response_body</code>, and the response headers to <code>response_headers</code>
                    as a JSON object with lower-case names.
                </p>
                <p>
                    A 2xx status code is SUCCESS. Any other status, a connection error or a
                    <code>timeout</code> is FAILURE, and <code>error_message</code> says why. All outputs are
                    written on every run; with no response, <code>status_code</code> is 0.
                </p>
                <p>
                    The request runs in the background, so the rest of the tree keeps running, and halting
                    the Behavior cancels it. Only <code>http://</code> and <code>https://</code> URLs are
                    allowed. Use https when the request crosses an untrusted network, and keep
                    <code>verify_tls</code> true unless the server is a trusted device with a self-signed
                    certificate. Proxy settings come from the standard environment variables.
                </p>
            )";

constexpr auto kPortIDUrl = "url";
constexpr auto kPortIDMethod = "method";
constexpr auto kPortIDQueryParameters = "query_parameters";
constexpr auto kPortIDHeaders = "headers";
constexpr auto kPortIDBody = "body";
constexpr auto kPortIDContentType = "content_type";
constexpr auto kPortIDTimeout = "timeout";
constexpr auto kPortIDVerifyTls = "verify_tls";
constexpr auto kPortIDFollowRedirects = "follow_redirects";
constexpr auto kPortIDStatusCode = "status_code";
constexpr auto kPortIDResponseBody = "response_body";
constexpr auto kPortIDResponseHeaders = "response_headers";
constexpr auto kPortIDErrorMessage = "error_message";

/// Longest part of an error response body copied into the failure message.
constexpr std::size_t kMaxBodyInError = 200;
/// Most redirects followed when follow_redirects is true.
constexpr long kMaxRedirects = 10;
/// How often, in milliseconds, the transfer loop checks for a halt.
constexpr int kHaltPollIntervalMs = 50;
/// Longest timeout passed to curl, in milliseconds; it must fit a 32-bit long.
constexpr double kMaxTimeoutMs = 2.0e9;

// ordered_json keeps keys in the order they were written, so query parameters keep their order.
using Json = nlohmann::ordered_json;
using HeaderList = std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)>;

std::string toUpper(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return text;
}

std::string toLower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

std::string trim(const std::string& text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
  {
    return "";
  }
  return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

/// The URL up to its query string, for messages, so a token in a query parameter is not logged.
std::string urlForMessages(const std::string& url)
{
  return url.substr(0, url.find_first_of("?#"));
}

/// Reads a string port. BT.CPP would read JSON text typed into the port, such as {"a": 1}, as a blackboard key.
tl::expected<std::string, std::string> getTextInput(const BT::TreeNode& node, const std::string& port)
{
  const auto& ports = node.config().input_ports;
  if (const auto it = ports.find(port);
      it != ports.end() && BT::TreeNode::isBlackboardPointer(it->second) && Json::accept(it->second))
  {
    return it->second;
  }
  auto text = node.getInput<std::string>(port);
  if (!text)
  {
    return tl::make_unexpected("Failed to get input [" + port + "]: " + text.error());
  }
  return text.value();
}

/// Parses an optional JSON object port. Empty text is an empty object.
tl::expected<Json, std::string> parseObject(const std::string& text, const std::string& port)
{
  if (trim(text).empty())
  {
    return Json::object();
  }
  Json value;
  try
  {
    value = Json::parse(text);
  }
  catch (const Json::parse_error& e)
  {
    return tl::make_unexpected("[" + port + "] is not valid JSON: " + e.what());
  }
  if (!value.is_object())
  {
    return tl::make_unexpected("[" + port + "] must be a JSON object, not " + std::string(value.type_name()) + ".");
  }
  return value;
}

/// Text of a scalar in a query string or header: a string as it is, a number or boolean as JSON.
std::optional<std::string> scalarText(const Json& value)
{
  if (value.is_string())
  {
    return value.get<std::string>();
  }
  if (value.is_number() || value.is_boolean())
  {
    return value.dump();
  }
  return std::nullopt;
}

std::string escape(CURL* curl, const std::string& text)
{
  const std::unique_ptr<char, decltype(&curl_free)> escaped(
      curl_easy_escape(curl, text.data(), static_cast<int>(text.size())), &curl_free);
  return escaped ? std::string(escaped.get()) : std::string();
}

/// Percent-encodes @p query_text (a JSON object) and appends it to the query string of @p url.
tl::expected<std::string, std::string> appendQuery(CURL* curl, const std::string& url, const std::string& query_text)
{
  const auto query = parseObject(query_text, kPortIDQueryParameters);
  if (!query)
  {
    return tl::make_unexpected(query.error());
  }
  std::string encoded;
  const auto add = [&](const std::string& key, const std::optional<std::string>& value) {
    encoded += (encoded.empty() ? "" : "&") + escape(curl, key);
    if (value)
    {
      encoded += "=" + escape(curl, *value);
    }
  };
  for (const auto& [key, value] : query->items())
  {
    if (value.is_null())
    {
      add(key, std::nullopt);
      continue;
    }
    // An array repeats the key once per element, for example ?tag=a&tag=b.
    const Json elements = value.is_array() ? value : Json::array({ value });
    for (const auto& element : elements)
    {
      const auto text = scalarText(element);
      if (!text)
      {
        return tl::make_unexpected("[query_parameters] value of '" + key + "' is " + std::string(element.type_name()) +
                                   "; use a string, number, boolean, null or an array of those.");
      }
      add(key, text);
    }
  }
  if (encoded.empty())
  {
    return url;
  }
  const auto fragment = url.find('#');
  std::string result = url.substr(0, fragment);
  if (result.find('?') == std::string::npos)
  {
    result += '?';
  }
  else if (result.back() != '?' && result.back() != '&')
  {
    result += '&';
  }
  return result + encoded + (fragment == std::string::npos ? "" : url.substr(fragment));
}

/// True if @p text holds CR, LF or NUL, which would end a header line early.
bool hasLineBreak(const std::string& text)
{
  return text.find_first_of(std::string("\r\n\0", 3)) != std::string::npos;
}

/// True for the characters RFC 9110 allows in a header name.
bool isTokenChar(unsigned char c)
{
  return std::isalnum(c) != 0 || (c != '\0' && std::strchr("!#$%&'*+-.^_`|~", c) != nullptr);
}

/// Parses @p headers_text (a JSON object) into "Name: value" lines, refusing names or values that could
/// inject another header line.
tl::expected<std::vector<std::pair<std::string, std::string>>, std::string> parseHeaders(const std::string& headers_text)
{
  const auto headers = parseObject(headers_text, kPortIDHeaders);
  if (!headers)
  {
    return tl::make_unexpected(headers.error());
  }
  std::vector<std::pair<std::string, std::string>> result;
  for (const auto& [name, value] : headers->items())
  {
    if (name.empty() || !std::all_of(name.begin(), name.end(), [](unsigned char c) { return isTokenChar(c); }))
    {
      return tl::make_unexpected("[headers] '" + name + "' is not a valid header name.");
    }
    const auto text = value.is_null() ? std::optional<std::string>("") : scalarText(value);
    if (!text)
    {
      return tl::make_unexpected("[headers] value of '" + name + "' is " + std::string(value.type_name()) +
                                 "; use a string, number, boolean or null.");
    }
    if (hasLineBreak(*text))
    {
      return tl::make_unexpected("[headers] value of '" + name + "' contains a line break or NUL character.");
    }
    result.emplace_back(name, *text);
  }
  return result;
}

void appendHeader(HeaderList& list, const std::string& line)
{
  // curl_slist_append returns the head of the list, or nullptr (leaving the list unchanged) if it fails.
  if (curl_slist* head = curl_slist_append(list.get(), line.c_str()))
  {
    list.release();
    list.reset(head);
  }
}

size_t appendToString(char* data, size_t size, size_t count, void* user_data)
{
  static_cast<std::string*>(user_data)->append(data, size * count);
  return size * count;
}

/// Collects response headers into a JSON object with lower-case names; a repeated name is joined with ", ".
size_t collectHeader(char* data, size_t size, size_t count, void* user_data)
{
  auto& headers = *static_cast<Json*>(user_data);
  const std::string line = trim(std::string(data, size * count));
  // A status line starts a new response (after a redirect or "100 Continue"); keep the last response only.
  if (line.rfind("HTTP/", 0) == 0)
  {
    headers = Json::object();
    return size * count;
  }
  const auto colon = line.find(':');
  if (colon == std::string::npos || colon == 0)
  {
    return size * count;
  }
  const std::string name = toLower(trim(line.substr(0, colon)));
  const std::string value = trim(line.substr(colon + 1));
  if (const auto it = headers.find(name); it != headers.end())
  {
    *it = it->get<std::string>() + ", " + value;
  }
  else
  {
    headers[name] = value;
  }
  return size * count;
}

void initCurlOnce()
{
  static std::once_flag once;
  std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

/// Runs the transfer on a multi handle so a halt is seen within kHaltPollIntervalMs. Empty means cancelled.
std::optional<CURLcode> performUnlessHalted(CURL* handle, const std::atomic<bool>& halt_requested)
{
  const std::unique_ptr<CURLM, decltype(&curl_multi_cleanup)> multi(curl_multi_init(), &curl_multi_cleanup);
  if (!multi || curl_multi_add_handle(multi.get(), handle) != CURLM_OK)
  {
    return CURLE_FAILED_INIT;
  }
  std::optional<CURLcode> result = CURLE_FAILED_INIT;
  int running = 1;
  while (running > 0)
  {
    if (halt_requested)
    {
      result = std::nullopt;
      break;
    }
    if (curl_multi_perform(multi.get(), &running) != CURLM_OK)
    {
      break;
    }
    if (running > 0)
    {
      curl_multi_poll(multi.get(), nullptr, 0, kHaltPollIntervalMs, nullptr);
    }
  }
  if (result.has_value())
  {
    int queued = 0;
    while (const CURLMsg* message = curl_multi_info_read(multi.get(), &queued))
    {
      if (message->msg == CURLMSG_DONE && message->easy_handle == handle)
      {
        result = message->data.result;
      }
    }
  }
  curl_multi_remove_handle(multi.get(), handle);
  return result;
}
}  // namespace

namespace experimental_behaviors
{
SendHttpRequest::SendHttpRequest(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList SendHttpRequest::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDUrl, "URL of the request, for example http://192.168.1.10:8080/api/jobs."),
    BT::InputPort<std::string>(kPortIDMethod, "GET", "HTTP method: GET, POST, PUT, PATCH or DELETE."),
    BT::InputPort<std::string>(kPortIDQueryParameters, "",
                               "Query parameters as a JSON object, for example {\"id\": \"42\"}. Empty for none."),
    BT::InputPort<std::string>(kPortIDHeaders, "",
                               "Request headers as a JSON object, for example {\"Authorization\": \"Bearer abc\"}."),
    BT::InputPort<std::string>(kPortIDBody, "", "Request body, sent as it is. Must be empty for GET."),
    BT::InputPort<std::string>(kPortIDContentType, "application/json",
                               "Content-Type of the body. Empty sends no Content-Type."),
    BT::InputPort<double>(kPortIDTimeout, 10.0, "Seconds to wait for the whole request before failing."),
    BT::InputPort<bool>(kPortIDVerifyTls, true,
                        "Check the server's https certificate. Keep true on untrusted networks."),
    BT::InputPort<bool>(kPortIDFollowRedirects, false, "Follow 3xx redirects (at most 10)."),
    BT::OutputPort<int>(kPortIDStatusCode, "{status_code}", "HTTP status code of the response; 0 if none."),
    BT::OutputPort<std::string>(kPortIDResponseBody, "{response_body}", "Body of the response."),
    BT::OutputPort<std::string>(kPortIDResponseHeaders, "{response_headers}",
                                "Response headers as a JSON object with lower-case names."),
    BT::OutputPort<std::string>(kPortIDErrorMessage, "{error_message}", "Why the request failed; empty on SUCCESS."),
  };
}

BT::KeyValueVector SendHttpRequest::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "REST" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSendHttpRequest } };
}

tl::expected<bool, std::string> SendHttpRequest::failWithoutResponse(const std::string& error)
{
  setOutput(kPortIDStatusCode, 0);
  setOutput(kPortIDResponseBody, std::string());
  setOutput(kPortIDResponseHeaders, std::string("{}"));
  setOutput(kPortIDErrorMessage, error);
  return tl::make_unexpected(error);
}

tl::expected<bool, std::string> SendHttpRequest::doWork()
{
  // A halt may come at any time; the transfer loop picks it up.
  halt_requested_ = false;
  notifyCanHalt();

  const auto url = getInput<std::string>(kPortIDUrl);
  const auto method_input = getInput<std::string>(kPortIDMethod);
  const auto query_text = getTextInput(*this, kPortIDQueryParameters);
  const auto headers_text = getTextInput(*this, kPortIDHeaders);
  const auto body = getTextInput(*this, kPortIDBody);
  const auto content_type = getInput<std::string>(kPortIDContentType);
  const auto timeout = getInput<double>(kPortIDTimeout);
  const auto verify_tls = getInput<bool>(kPortIDVerifyTls);
  const auto follow_redirects = getInput<bool>(kPortIDFollowRedirects);
  std::string input_error;
  const auto check = [&input_error](const auto& input) {
    if (!input && input_error.empty())
    {
      input_error = input.error();
    }
  };
  check(url);
  check(method_input);
  check(query_text);
  check(headers_text);
  check(body);
  check(content_type);
  check(timeout);
  check(verify_tls);
  check(follow_redirects);
  if (!input_error.empty())
  {
    return failWithoutResponse("Failed to get required input: " + input_error);
  }

  const std::string method = toUpper(method_input.value());
  if (method != "GET" && method != "POST" && method != "PUT" && method != "PATCH" && method != "DELETE")
  {
    return failWithoutResponse("[method] must be GET, POST, PUT, PATCH or DELETE, not '" + method_input.value() + "'.");
  }
  const std::string scheme = toLower(url->substr(0, url->find("://") + 3));
  if (scheme != "http://" && scheme != "https://")
  {
    return failWithoutResponse("[url] must start with http:// or https://, not '" + urlForMessages(url.value()) + "'.");
  }
  if (!std::isfinite(timeout.value()) || timeout.value() <= 0.0)
  {
    return failWithoutResponse("[timeout] must be a finite number of seconds > 0.");
  }
  if (hasLineBreak(content_type.value()))
  {
    return failWithoutResponse("[content_type] contains a line break or NUL character.");
  }
  if (method == "GET" && !body->empty())
  {
    return failWithoutResponse("[body] must be empty for a GET request.");
  }
  // POST, PUT and PATCH always carry a body (it may be empty); DELETE only when one is given.
  const bool sends_body = method != "GET" && (method != "DELETE" || !body->empty());
  const std::string request = method + " " + urlForMessages(url.value());

  initCurlOnce();
  const std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(curl_easy_init(), &curl_easy_cleanup);
  if (!curl)
  {
    return failWithoutResponse("curl_easy_init() failed.");
  }
  CURL* handle = curl.get();
  const auto full_url = appendQuery(handle, url.value(), query_text.value());
  const auto request_headers = parseHeaders(headers_text.value());
  if (!full_url || !request_headers)
  {
    return failWithoutResponse(!full_url ? full_url.error() : request_headers.error());
  }

  HeaderList headers(nullptr, &curl_slist_free_all);
  bool has_content_type = false;
  for (const auto& [name, value] : request_headers.value())
  {
    has_content_type = has_content_type || toLower(name) == "content-type";
    // curl drops a header written as "Name:"; "Name;" sends it with an empty value.
    appendHeader(headers, value.empty() ? name + ";" : name + ": " + value);
  }
  if (sends_body && !has_content_type)
  {
    // "Content-Type:" with no value stops curl from adding its default form content type.
    appendHeader(headers, content_type->empty() ? "Content-Type:" : "Content-Type: " + content_type.value());
  }
  // Without this, curl waits for a "100 Continue" reply before sending a large body.
  appendHeader(headers, "Expect:");

  std::string response_body;
  Json response_headers = Json::object();
  char error_buffer[CURL_ERROR_SIZE] = { 0 };
  curl_easy_setopt(handle, CURLOPT_URL, full_url->c_str());
  if (method == "GET")
  {
    curl_easy_setopt(handle, CURLOPT_HTTPGET, 1L);
  }
  else if (method != "POST")
  {
    curl_easy_setopt(handle, CURLOPT_CUSTOMREQUEST, method.c_str());
  }
  if (sends_body)
  {
    curl_easy_setopt(handle, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(body->size()));
    curl_easy_setopt(handle, CURLOPT_POSTFIELDS, body->data());
  }
  curl_easy_setopt(handle, CURLOPT_HTTPHEADER, headers.get());
  curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS,
                   static_cast<long>(std::lround(std::clamp(timeout.value() * 1000.0, 1.0, kMaxTimeoutMs))));
  curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(handle, CURLOPT_FOLLOWLOCATION, follow_redirects.value() ? 1L : 0L);
  curl_easy_setopt(handle, CURLOPT_MAXREDIRS, kMaxRedirects);
#if LIBCURL_VERSION_NUM >= 0x075500  // CURLOPT_PROTOCOLS_STR arrived in curl 7.85.0.
  curl_easy_setopt(handle, CURLOPT_PROTOCOLS_STR, "http,https");
  curl_easy_setopt(handle, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
#else
  curl_easy_setopt(handle, CURLOPT_PROTOCOLS, static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
  curl_easy_setopt(handle, CURLOPT_REDIR_PROTOCOLS, static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
#endif
  if (!verify_tls.value())
  {
    curl_easy_setopt(handle, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(handle, CURLOPT_SSL_VERIFYHOST, 0L);
  }
  curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, &appendToString);
  curl_easy_setopt(handle, CURLOPT_WRITEDATA, &response_body);
  curl_easy_setopt(handle, CURLOPT_HEADERFUNCTION, &collectHeader);
  curl_easy_setopt(handle, CURLOPT_HEADERDATA, &response_headers);
  curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, error_buffer);

  const auto result = performUnlessHalted(handle, halt_requested_);
  if (!result.has_value())
  {
    return failWithoutResponse(request + " was cancelled because the Behavior was halted.");
  }
  if (result.value() != CURLE_OK)
  {
    const std::string detail =
        error_buffer[0] != '\0' ? std::string(error_buffer) : std::string(curl_easy_strerror(result.value()));
    return failWithoutResponse(request + " failed: " + detail);
  }

  long status_code = 0;
  curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status_code);
  setOutput(kPortIDStatusCode, static_cast<int>(status_code));
  setOutput(kPortIDResponseBody, response_body);
  setOutput(kPortIDResponseHeaders, response_headers.dump());
  if (status_code < 200 || status_code > 299)
  {
    const std::string error = request + " returned HTTP status " + std::to_string(status_code) + ": " +
                              response_body.substr(0, kMaxBodyInError);
    setOutput(kPortIDErrorMessage, error);
    return tl::make_unexpected(error);
  }
  setOutput(kPortIDErrorMessage, std::string());
  return true;
}

tl::expected<void, std::string> SendHttpRequest::doHalt()
{
  halt_requested_ = true;
  return {};
}
}  // namespace experimental_behaviors

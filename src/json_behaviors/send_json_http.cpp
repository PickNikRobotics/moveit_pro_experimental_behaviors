// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/send_json_http.hpp>

#include <curl/curl.h>
#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <mutex>

namespace
{
inline constexpr auto kDescriptionSendJsonHttp = R"(
                <p>
                    Sends a JSON document to a web service with an HTTP <code>POST</code> or
                    <code>PUT</code> request, and outputs the response body and status code. Use it when
                    the other system must confirm that it received the message, for example a fleet manager
                    or a supervisor with a REST API.
                </p>
                <p>
                    The request runs in the background, so the rest of the tree keeps running, and halting
                    the Behavior cancels the request. The request carries
                    <code>Content-Type: application/json</code>. A 2xx status code is SUCCESS. Any other
                    status, a connection error or <code>timeout</code> is FAILURE; on a non-2xx status the
                    response and status code are still written, so the tree can inspect them.
                </p>
                <p>
                    Only <code>http://</code> and <code>https://</code> URLs are allowed, and redirects are
                    not followed. Use https when the request crosses an untrusted network. Proxy settings
                    come from the standard environment variables (<code>http_proxy</code>,
                    <code>no_proxy</code>).
                </p>
            )";

constexpr auto kPortIDUrl = "url";
constexpr auto kPortIDMethod = "method";
constexpr auto kPortIDPayload = "payload";
constexpr auto kPortIDTimeout = "timeout";
constexpr auto kPortIDResponse = "response";
constexpr auto kPortIDStatusCode = "status_code";

/// Longest part of an error response body copied into the failure message.
constexpr std::size_t kMaxBodyInError = 200;

size_t appendToString(char* data, size_t size, size_t count, void* user_data)
{
  static_cast<std::string*>(user_data)->append(data, size * count);
  return size * count;
}

int abortIfHalted(void* user_data, curl_off_t /*dltotal*/, curl_off_t /*dlnow*/, curl_off_t /*ultotal*/,
                  curl_off_t /*ulnow*/)
{
  return static_cast<const std::atomic<bool>*>(user_data)->load() ? 1 : 0;
}

void initCurlOnce()
{
  static std::once_flag once;
  std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}
}  // namespace

namespace experimental_behaviors
{
SendJsonHttp::SendJsonHttp(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList SendJsonHttp::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDUrl, "URL to send the request to, for example http://192.168.1.10:8080/status."),
    BT::InputPort<std::string>(kPortIDMethod, "POST", "HTTP method: POST or PUT."),
    BT::InputPort<std::string>(kPortIDPayload, "{json}", "JSON document to send as the request body, as JSON text."),
    BT::InputPort<double>(kPortIDTimeout, 10.0, "Seconds to wait for the whole request before failing."),
    BT::OutputPort<std::string>(kPortIDResponse, "{response}", "Body of the response."),
    BT::OutputPort<int>(kPortIDStatusCode, "{status_code}", "HTTP status code of the response."),
  };
}

BT::KeyValueVector SendJsonHttp::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSendJsonHttp } };
}

tl::expected<bool, std::string> SendJsonHttp::doWork()
{
  // A halt may come at any time; the transfer callback picks it up.
  halt_requested_ = false;
  notifyCanHalt();

  const auto url = getInput<std::string>(kPortIDUrl);
  auto method = getInput<std::string>(kPortIDMethod);
  const auto payload = json_utils::getJsonText(*this, kPortIDPayload);
  const auto timeout = getInput<double>(kPortIDTimeout);
  if (!url || !method || !payload || !timeout)
  {
    const std::string error = !url    ? url.error() :
                              !method ? method.error() :
                                        (!payload ? payload.error() : timeout.error());
    return tl::make_unexpected("Failed to get required input: " + error);
  }
  std::transform(method->begin(), method->end(), method->begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  if (method.value() != "POST" && method.value() != "PUT")
  {
    return tl::make_unexpected("[method] must be POST or PUT, not '" + method.value() + "'.");
  }
  if (!std::isfinite(timeout.value()) || timeout.value() <= 0.0)
  {
    return tl::make_unexpected("[timeout] must be a finite number of seconds > 0.");
  }
  if (const auto doc = json_utils::parse(payload.value(), "[payload]"); !doc)
  {
    return tl::make_unexpected(doc.error());
  }

  initCurlOnce();
  const std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(curl_easy_init(), &curl_easy_cleanup);
  if (!curl)
  {
    return tl::make_unexpected("curl_easy_init() failed.");
  }
  curl_slist* raw_headers = nullptr;
  raw_headers = curl_slist_append(raw_headers, "Content-Type: application/json");
  raw_headers = curl_slist_append(raw_headers, "Accept: application/json");
  // Without this, curl waits for a "100 Continue" reply before sending a large body.
  raw_headers = curl_slist_append(raw_headers, "Expect:");
  const std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)> headers(raw_headers, &curl_slist_free_all);

  std::string response;
  char error_buffer[CURL_ERROR_SIZE] = { 0 };
  CURL* handle = curl.get();
  curl_easy_setopt(handle, CURLOPT_URL, url->c_str());
  curl_easy_setopt(handle, CURLOPT_CUSTOMREQUEST, method->c_str());
  curl_easy_setopt(handle, CURLOPT_POSTFIELDS, payload->data());
  curl_easy_setopt(handle, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(payload->size()));
  curl_easy_setopt(handle, CURLOPT_HTTPHEADER, headers.get());
  curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, static_cast<long>(std::lround(timeout.value() * 1000.0)));
  curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(handle, CURLOPT_FOLLOWLOCATION, 0L);
#if LIBCURL_VERSION_NUM >= 0x075500  // CURLOPT_PROTOCOLS_STR arrived in curl 7.85.0.
  curl_easy_setopt(handle, CURLOPT_PROTOCOLS_STR, "http,https");
#else
  curl_easy_setopt(handle, CURLOPT_PROTOCOLS, static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
#endif
  curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, &appendToString);
  curl_easy_setopt(handle, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(handle, CURLOPT_NOPROGRESS, 0L);
  curl_easy_setopt(handle, CURLOPT_XFERINFOFUNCTION, &abortIfHalted);
  curl_easy_setopt(handle, CURLOPT_XFERINFODATA, &halt_requested_);
  curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, error_buffer);

  const CURLcode result = curl_easy_perform(handle);
  const std::string request = method.value() + " " + url.value();
  if (result == CURLE_ABORTED_BY_CALLBACK && halt_requested_)
  {
    return tl::make_unexpected(request + " was cancelled because the Behavior was halted.");
  }
  if (result != CURLE_OK)
  {
    const std::string detail = error_buffer[0] != '\0' ? std::string(error_buffer) : curl_easy_strerror(result);
    return tl::make_unexpected(request + " failed: " + detail);
  }

  long status_code = 0;
  curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status_code);
  setOutput(kPortIDResponse, response);
  setOutput(kPortIDStatusCode, static_cast<int>(status_code));
  if (status_code < 200 || status_code > 299)
  {
    return tl::make_unexpected(request + " returned HTTP status " + std::to_string(status_code) + ": " +
                               response.substr(0, kMaxBodyInError));
  }
  return true;
}

tl::expected<void, std::string> SendJsonHttp::doHalt()
{
  halt_requested_ = true;
  return {};
}
}  // namespace experimental_behaviors

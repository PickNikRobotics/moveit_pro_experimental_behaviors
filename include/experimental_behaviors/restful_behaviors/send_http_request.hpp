// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>

#include <atomic>
#include <future>
#include <string>

namespace experimental_behaviors
{
/**
 * @brief Sends one HTTP request (GET, POST, PUT, PATCH or DELETE) to a REST API and outputs the response.
 *
 * | Data Port Name   | Port Type | Object Type | Default              |
 * | ---------------- | --------- | ----------- | -------------------- |
 * | url              | input     | std::string |                      |
 * | method           | input     | std::string | `GET`                |
 * | query_parameters | input     | std::string | `""`                 |
 * | headers          | input     | std::string | `""`                 |
 * | body             | input     | std::string | `""`                 |
 * | content_type     | input     | std::string | `application/json`   |
 * | timeout          | input     | double      | `10.0`               |
 * | verify_tls       | input     | bool        | `true`               |
 * | follow_redirects | input     | bool        | `false`              |
 * | status_code      | output    | int         | `{status_code}`      |
 * | response_body    | output    | std::string | `{response_body}`    |
 * | response_headers | output    | std::string | `{response_headers}` |
 * | error_message    | output    | std::string | `{error_message}`    |
 *
 * @details
 * query_parameters, headers and response_headers are JSON objects as text. A 2xx status is SUCCESS; any other
 * status, or no response, is FAILURE. Every run writes all four outputs (see docs/restful_behaviors.md).
 *
 * The request runs in the background, so the tree keeps ticking, and a halt cancels it. Only http and https
 * URLs are allowed.
 */
class SendHttpRequest final : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  SendHttpRequest(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  tl::expected<bool, std::string> doWork() override;
  tl::expected<void, std::string> doHalt() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /// Writes the four outputs for a run that got no HTTP response, and returns @p error for doWork().
  tl::expected<bool, std::string> failWithoutResponse(const std::string& error);

  /// Set by doHalt(); the transfer loop reads it and aborts the request.
  std::atomic<bool> halt_requested_{ false };
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace experimental_behaviors

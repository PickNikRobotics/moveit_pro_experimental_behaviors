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
 * @brief Sends a JSON document in an HTTP POST or PUT request and outputs the response.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | url            | input     | std::string |
 * | method         | input     | std::string |
 * | payload        | input     | std::string |
 * | timeout        | input     | double      |
 * | response       | output    | std::string |
 * | status_code    | output    | int         |
 *
 * @details
 * The request runs in the background, so the tree keeps ticking, and a halt cancels it. The request has
 * `Content-Type: application/json`. SUCCESS needs a 2xx status; any other status sets both outputs and fails.
 * Only http and https URLs are allowed, and redirects are not followed.
 */
class SendJsonHttp final : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  SendJsonHttp(const std::string& name, const BT::NodeConfiguration& config,
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

  /// Set by doHalt(); the transfer callback reads it and aborts the request.
  std::atomic<bool> halt_requested_{ false };
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace experimental_behaviors

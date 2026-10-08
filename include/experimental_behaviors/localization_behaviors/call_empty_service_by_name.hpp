// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <std_srvs/srv/empty.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <string>

namespace experimental_behaviors
{
using Empty = std_srvs::srv::Empty;

/**
 * @brief Calls a std_srvs/srv/Empty service by name and succeeds when the server responds.
 *
 * | Data Port Name                    | Port Type | Object Type |
 * | --------------------------------- | --------- | ----------- |
 * | service_name                      | input     | std::string |
 * | response_timeout                  | input     | double      |
 * | wait_for_server_available_timeout | input     | double      |
 *
 * @details
 * An Empty response carries no success field, so any response counts as success. It fails if no server
 * appears in time or the response does not arrive in time.
 *
 * Newer MoveIt Pro releases ship the same Behavior as CallEmptyService; this ID differs so both can load.
 */
class CallEmptyServiceByName final : public moveit_pro::behaviors::ServiceClientBehaviorBase<Empty>
{
public:
  CallEmptyServiceByName(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /** @brief Test constructor: injects the client, so no service server is needed. */
  CallEmptyServiceByName(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
                         std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Empty>> client_interface);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;
  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;
  tl::expected<std::chrono::duration<double>, std::string> getWaitForServerAvailableTimeout() override;
  tl::expected<Empty::Request, std::string> createRequest() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace experimental_behaviors

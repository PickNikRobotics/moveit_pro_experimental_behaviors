// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Sends a JSON document as one UDP datagram.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | host           | input     | std::string |
 * | port           | input     | int         |
 * | payload        | input     | std::string |
 *
 * @details
 * `host` is an IPv4 or IPv6 address or a host name. The payload must be valid JSON and fit in one datagram.
 * UDP gives no delivery guarantee: SUCCESS means the datagram left this host, not that anyone received it.
 */
class SendJsonUdp final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SendJsonUdp(const std::string& name, const BT::NodeConfiguration& config,
              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors

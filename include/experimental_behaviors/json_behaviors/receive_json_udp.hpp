// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <experimental_behaviors/json_behaviors/udp_socket.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <chrono>
#include <string>

namespace experimental_behaviors
{
/**
 * @brief Waits for a JSON document to arrive as a UDP datagram.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | port           | input     | int         |
 * | bind_address   | input     | std::string |
 * | timeout        | input     | double      |
 * | payload        | output    | std::string |
 *
 * @details
 * The socket opens on the first tick and stays open until the tree is destroyed, so datagrams that arrive
 * between runs are kept by the operating system. Each run outputs the newest queued datagram and drops older ones.
 * If none is queued, it returns RUNNING until one arrives, and FAILURE after `timeout` seconds.
 *
 * Security: `bind_address` defaults to the loopback interface. Bind to a specific interface rather than
 * all of them, and treat the payload as untrusted: UDP is not an authenticated channel.
 */
class ReceiveJsonUdp final : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  ReceiveJsonUdp(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Outputs the newest queued datagram. Returns RUNNING if none is queued yet.
  BT::NodeStatus pollSocket();

  json_utils::UdpSocket socket_;
  std::string bound_address_;
  int bound_port_ = 0;
  double timeout_ = 0.0;
  std::chrono::steady_clock::time_point deadline_;
};
}  // namespace experimental_behaviors

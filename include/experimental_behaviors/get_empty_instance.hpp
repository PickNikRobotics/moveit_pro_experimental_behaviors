// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <shared_mutex>

#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/empty.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Snapshot-style `std_msgs/Empty` subscriber. Blocks until the first message arrives,
 *        then returns SUCCESS and tears down its subscription. Use for one-shot event
 *        triggers where downstream logic must not run until the trigger fires.
 *
 * | Data Port Name    | Port Type | Object Type |
 * | ----------------- | --------- | ----------- |
 * | empty_topic_name  | input     | std::string |
 * | message_received  | output    | bool        |
 * | message_count     | output    | uint64_t    |
 */
class GetEmptyInstance : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetEmptyInstance(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> empty_subscriber_;
  std::shared_mutex empty_mutex_;
  uint64_t message_count_;
};
}  // namespace experimental_behaviors

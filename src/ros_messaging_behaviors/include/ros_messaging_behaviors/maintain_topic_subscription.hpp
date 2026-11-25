// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros_messaging_behaviors
{
/**
 * @brief Hold subscriptions to one or more topics, of any message type, open while this Behavior is RUNNING.
 *
 * @details Some drivers publish only while a topic has a subscriber. Run this Behavior in a Parallel
 * next to the steps that read the stream; halting it drops the subscriptions.
 *
 * The message type of each topic comes from the ROS graph, so a topic with no publisher yet is
 * retried on every tick.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | topic_names    | Input     | std::string |
 */
class MaintainTopicSubscription final : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  MaintainTopicSubscription(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  /** @brief Subscribes to every topic that has a publisher now. Returns RUNNING. */
  BT::NodeStatus onStart() override;

  /** @brief Retries the topics that had no publisher yet. Returns RUNNING until halted. */
  BT::NodeStatus onRunning() override;

  /** @brief Drops every subscription, so the driver can stop the stream. */
  void onHalted() override;

private:
  /** @brief True when the topic is done with: subscribed, or skipped after an error. False to retry later. */
  bool trySubscribe(const std::string& topic);

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::vector<std::string> pending_topics_;
};
}  // namespace ros_messaging_behaviors

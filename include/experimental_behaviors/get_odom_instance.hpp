// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <optional>
#include <shared_mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Snapshot-style `nav_msgs/Odometry` subscriber. Subscribes on start, blocks until
 *        the first message arrives, returns SUCCESS, and tears down its subscription. Use
 *        for one-shot pose snapshots where downstream math must not race against
 *        subscription cold-start.
 *
 * | Data Port Name           | Port Type | Object Type             |
 * | ------------------------ | --------- | ----------------------- |
 * | odom_topic_name          | input     | std::string             |
 * | subscribed_odom_instance | output    | nav_msgs::msg::Odometry |
 */
class GetOdomInstance : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetOdomInstance(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  std::shared_mutex odom_mutex_;
  std::optional<nav_msgs::msg::Odometry> current_odometry_;
  std::string odom_topic_name_;
};
}  // namespace experimental_behaviors

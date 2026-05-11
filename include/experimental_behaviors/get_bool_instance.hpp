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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Snapshot-style `std_msgs/Bool` subscriber. Blocks until the first message arrives,
 *        then returns SUCCESS and tears down its subscription. Outputs the full Bool
 *        message (not the primitive bool) — useful when downstream consumers need the
 *        ROS message wrapper. For streaming the primitive value, use `GetBool` instead.
 *
 * | Data Port Name           | Port Type | Object Type         |
 * | ------------------------ | --------- | ------------------- |
 * | bool_topic_name          | input     | std::string         |
 * | subscribed_bool_instance | output    | std_msgs::msg::Bool |
 */
class GetBoolInstance : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetBoolInstance(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;
  std::shared_mutex bool_mutex_;
  std::optional<std_msgs::msg::Bool> current_bool_;
  std::string bool_topic_name_;
};
}  // namespace experimental_behaviors

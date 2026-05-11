// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <optional>
#include <shared_mutex>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Long-lived `std_msgs/Bool` subscriber that writes the latest received value to the
 *        blackboard every tick. Returns RUNNING forever; halted by the parent on parallel
 *        completion.
 *
 * @details Uses `std::optional<bool>` for the cached value and skips `setOutput` until the
 *          first real message arrives. Without this, a default-constructed `false` would be
 *          written on the first onRunning tick, clobbering any pre-seeded blackboard value
 *          before the publisher has delivered a real sample.
 *
 * | Data Port Name   | Port Type | Object Type |
 * | ---------------- | --------- | ----------- |
 * | bool_topic_name  | input     | std::string |
 * | subscribed_bool  | output    | bool        |
 */
class GetBool : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetBool(const std::string& name, const BT::NodeConfiguration& config,
          const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;
  std::shared_mutex bool_mutex_;
  std::optional<bool> current_bool_value_;
};
}  // namespace experimental_behaviors

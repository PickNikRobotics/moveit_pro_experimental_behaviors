// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Publishes a `std_msgs/Bool` message to the given topic.
 *
 * @details Mirror of the existing core `PublishString` and `PublishEmpty` behaviors, for
 *          the `Bool` message type. The publisher is created lazily on the first tick that
 *          uses a given topic, and re-created if the topic name changes between ticks.
 *
 * | Data Port Name | Port Type | Object Type   |
 * | -------------- | --------- | ------------- |
 * | topic_name     | input     | std::string   |
 * | value          | input     | bool          |
 */
class PublishBool final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  PublishBool(const std::string& name, const BT::NodeConfiguration& config,
              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> publisher_;
  std::string current_topic_name_;
};
}  // namespace experimental_behaviors

// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Converts a `nav_msgs/Odometry` message into a `geometry_msgs/PoseStamped`,
 *        preserving header and pose; the twist field is dropped.
 *
 * | Data Port Name | Port Type | Object Type                     |
 * | -------------- | --------- | ------------------------------- |
 * | odometry       | input     | nav_msgs::msg::Odometry         |
 * | pose_stamped   | output    | geometry_msgs::msg::PoseStamped |
 */
class ConvertOdomToPoseStamped final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ConvertOdomToPoseStamped(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors

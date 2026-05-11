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
 * @brief Converts a `geometry_msgs/PoseStamped` into a `nav_msgs/Odometry`,
 *        preserving header and pose; twist is left zero-initialized.
 *
 * | Data Port Name | Port Type | Object Type                     |
 * | -------------- | --------- | ------------------------------- |
 * | pose_stamped   | input     | geometry_msgs::msg::PoseStamped |
 * | child_frame    | input     | std::string (optional)          |
 * | odom           | output    | nav_msgs::msg::Odometry         |
 */
class ConvertPoseStampedToOdom final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ConvertPoseStampedToOdom(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors

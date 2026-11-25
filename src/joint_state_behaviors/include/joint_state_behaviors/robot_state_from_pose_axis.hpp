// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <memory>
#include <string>

namespace joint_state_behaviors
{

/**
 * @brief A one-joint RobotState goal whose value is a coordinate of a pose.
 *
 * For driving a prismatic axis to line up with something: "put linear_x_joint at
 * the x of the shelf slot we are about to place in". The tree can pass the pose
 * around but cannot reach inside it, and CreateRobotState wants its positions up
 * front, so this closes the gap.
 *
 * It outputs a moveit_msgs::RobotState, the type `Move to Joint State` consumes, not a
 * sensor_msgs::JointState. See docs/AGENT_COMMENTS.md.
 *
 * Naming exactly one joint pairs with allow_partial_joints_goal on the controller:
 * the other axes keep whatever they had, which is what you want when only one of
 * them should move.
 */
class RobotStateFromPoseAxis final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  RobotStateFromPoseAxis(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace joint_state_behaviors

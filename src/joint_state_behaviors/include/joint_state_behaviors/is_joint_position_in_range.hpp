// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tl_expected/expected.hpp>

#include <memory>
#include <string>

namespace joint_state_behaviors
{
namespace internal
{
/** The position of @p joint_name in @p state, or why it has none. */
tl::expected<double, std::string> jointPosition(const sensor_msgs::msg::JointState& state,
                                                const std::string& joint_name);
}  // namespace internal

/**
 * @brief SUCCESS if one joint's current position is within [min_position, max_position].
 *
 * Reads the joint from a planning scene's robot state (GetCurrentPlanningScene), so it
 * needs no subscription of its own, and outputs the value for logging or scripts.
 *
 * The use it was written for is a grasp check on a gripper with no success signal: the
 * Robotiq 2F-85's GripperActionController allows stalling, so closing on a part and
 * closing on nothing both SUCCEED. The knuckle joint tells them apart - it stalls part way
 * on the part and runs to its end stop on nothing.
 */
class IsJointPositionInRange final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  IsJointPositionInRange(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace joint_state_behaviors

// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Reads a single-DOF joint's position limits from the loaded robot model.
 *
 * @details
 * Exposes the position bounds recorded in `robot_description` (the URDF) so behavior trees can read
 * a joint's travel range at runtime instead of duplicating it in a static YAML. This keeps clamps
 * correct when limits are configured per deployment in the URDF rather than globally.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | joint_name     | input     | std::string |
 * | min_position   | output    | double      |
 * | max_position   | output    | double      |
 *
 * Returns FAILURE if the robot model is unavailable, the joint is not found, the joint is not
 * single-DOF, or the joint has no position limits.
 */
class GetJointLimits final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetJointLimits(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace experimental_behaviors

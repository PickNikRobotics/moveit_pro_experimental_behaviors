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
 * @brief Keep only the named joints of a RobotState.
 *
 * Exists to bridge two behaviors with opposite policies about the same waypoint:
 * PlanToJointGoal FILTERS a stored state to its planning group, while
 * GeneratePointToPointTrajectory REFUSES one carrying anything outside it
 * ("'target_state' must contain a subset of the planning group joints").
 *
 * The UI saves waypoints with every joint on the robot, so without this a pose recorded
 * for one arm cannot reach the point-to-point planner. See docs/AGENT_COMMENTS.md.
 *
 * Filtering rather than re-recording is deliberate: the waypoint stays the one the
 * operator saved and verified, and nothing has to be kept in sync by hand.
 */
class FilterRobotStateToJoints final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  FilterRobotStateToJoints(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace joint_state_behaviors

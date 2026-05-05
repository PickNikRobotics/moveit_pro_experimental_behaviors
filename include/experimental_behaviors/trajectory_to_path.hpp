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
 * @brief Converts a JointTrajectory into a list of stamped Cartesian poses
 *        using forward kinematics on a user-chosen tip link.
 *
 * @details
 * Designed to sit upstream of `VisualizePath`: the output `path` port plugs
 * directly into `VisualizePath`'s `path` input, allowing visualization of a
 * joint-space trajectory (e.g. one about to be sent to `ExecuteTrajectory`).
 *
 * The base FK state is the planning scene's current state; joints named in
 * `joint_trajectory.joint_names` overwrite that base for each point so joints
 * absent from the trajectory (e.g. a rail or the other arm) keep their
 * current values.
 *
 * | Data Port Name     | Port Type | Object Type                                    |
 * | ------------------ | --------- | ---------------------------------------------- |
 * | joint_trajectory   | input     | trajectory_msgs::msg::JointTrajectory          |
 * | tip_link           | input     | std::string                                    |
 * | path               | output    | std::vector<geometry_msgs::msg::PoseStamped>   |
 *
 * Output poses are stamped with the robot model's root frame
 * (`robot_model->getModelFrame()`) and the current node clock.
 */
class TrajectoryToPath final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  static constexpr auto kPortIDJointTrajectory = "joint_trajectory";
  static constexpr auto kPortIDTipLink = "tip_link";
  static constexpr auto kPortIDPath = "path";

  TrajectoryToPath(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors

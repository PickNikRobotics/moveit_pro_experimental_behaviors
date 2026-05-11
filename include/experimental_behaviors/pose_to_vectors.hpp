// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <utility>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
namespace detail
{
/// @brief Decomposes a Pose into translation [x, y, z] and quaternion [x, y, z, w] vectors.
/// Pure function — no I/O, no failure modes. Companion to composePose() in vectors_to_pose.hpp.
[[nodiscard]] std::pair<std::vector<double>, std::vector<double>> decomposePose(const geometry_msgs::msg::Pose& pose);
}  // namespace detail

/**
 * @brief Decomposes a PoseStamped into translation_xyz and quaternion_xyzw vectors compatible
 *        with the std::vector<double> ports used by core behaviors like TransformPose.
 *
 * @details Companion to VectorsToPose.
 *
 * | Data Port Name   | Port Type | Object Type                     |
 * | ---------------- | --------- | ------------------------------- |
 * | pose_stamped     | input     | geometry_msgs::msg::PoseStamped |
 * | translation_xyz  | output    | std::vector<double> (size 3)    |
 * | quaternion_xyzw  | output    | std::vector<double> (size 4)    |
 */
class PoseToVectors final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  PoseToVectors(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors

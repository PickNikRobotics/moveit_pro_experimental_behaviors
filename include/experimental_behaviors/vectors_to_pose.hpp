// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <tl_expected/expected.hpp>

namespace experimental_behaviors
{
namespace detail
{
/// @brief Composes a Pose from translation [x, y, z] and quaternion [x, y, z, w] vectors.
/// Validates sizes (3 and 4 respectively); returns an error string if either is wrong.
/// Pure function — no I/O. Companion to decomposePose() in pose_to_vectors.hpp.
[[nodiscard]] tl::expected<geometry_msgs::msg::Pose, std::string>
composePose(const std::vector<double>& translation_xyz, const std::vector<double>& quaternion_xyzw);
}  // namespace detail

/**
 * @brief Assembles a PoseStamped from translation_xyz and quaternion_xyzw vectors plus a frame_id.
 *
 * @details Inverse of PoseToVectors. The output PoseStamped's header.stamp is set to the node's
 *          current ROS time; if you need a specific stamp, override it downstream.
 *
 * | Data Port Name   | Port Type | Object Type                     |
 * | ---------------- | --------- | ------------------------------- |
 * | translation_xyz  | input     | std::vector<double> (size 3)    |
 * | quaternion_xyzw  | input     | std::vector<double> (size 4)    |
 * | frame_id         | input     | std::string                     |
 * | pose_stamped     | output    | geometry_msgs::msg::PoseStamped |
 */
class VectorsToPose final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  VectorsToPose(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace experimental_behaviors

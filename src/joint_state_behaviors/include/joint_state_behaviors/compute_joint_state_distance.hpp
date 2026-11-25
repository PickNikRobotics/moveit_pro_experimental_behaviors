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
#include <vector>

namespace joint_state_behaviors
{
namespace internal
{
/** @brief Joint-space distance between two joint states. */
struct JointStateDistance
{
  /** @brief Euclidean norm of the per-joint position differences. */
  double distance = 0.0;
  /** @brief Largest absolute position difference of one joint. */
  double max_joint_distance = 0.0;
};

/**
 * @brief Distance between @p current and @p target over @p joint_names.
 * @param joint_names Joints to compare. Empty means every joint named in @p target.
 * @return The distance, or why it cannot be computed (a joint without a position in either state).
 */
tl::expected<JointStateDistance, std::string> jointStateDistance(const sensor_msgs::msg::JointState& current,
                                                                 const sensor_msgs::msg::JointState& target,
                                                                 const std::vector<std::string>& joint_names);
}  // namespace internal

/**
 * @brief Compute the joint-space distance between two joint states.
 *
 * @details Outputs the Euclidean norm of the position differences and the largest difference of
 * one joint, over the joints named in joint_names.
 *
 * The norm adds radians and meters as they are, so compare arm joints and a linear axis in two
 * separate calls.
 *
 * | Data Port Name      | Port Type | Object Type                  |
 * | ------------------- | --------- | ---------------------------- |
 * | joint_state_current | Input     | sensor_msgs::msg::JointState |
 * | joint_state_target  | Input     | sensor_msgs::msg::JointState |
 * | joint_names         | Input     | std::vector<std::string>     |
 * | distance            | Output    | double                       |
 * | max_joint_distance  | Output    | double                       |
 */
class ComputeJointStateDistance final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ComputeJointStateDistance(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace joint_state_behaviors

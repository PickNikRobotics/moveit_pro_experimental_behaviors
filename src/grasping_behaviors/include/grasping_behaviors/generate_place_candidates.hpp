// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tl_expected/expected.hpp>

#include <memory>
#include <string>
#include <vector>

namespace grasping_behaviors
{
namespace internal
{
/** @brief Inputs of the place candidate math, all poses already in one frame. */
struct PlaceCandidateRequest
{
  Eigen::Isometry3d object_at_pick = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d grasp_at_pick = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::Vector3d target_offset = Eigen::Vector3d::Zero();
  double yaw_range_deg = 0.0;
  double yaw_increment_deg = 0.0;
  bool include_upright = true;
  bool include_flipped = false;
};

/** @brief Grasp-link poses that place the object, and where the object lands for the first one. */
struct PlaceCandidates
{
  std::vector<Eigen::Isometry3d> grasp_poses;
  Eigen::Isometry3d predicted_object_pose = Eigen::Isometry3d::Identity();
};

/** @brief Computes the place candidates, or why there are none. */
tl::expected<PlaceCandidates, std::string> generatePlaceCandidates(const PlaceCandidateRequest& request);
}  // namespace internal

/**
 * @brief Compute grasp-link poses that place a held object at a target pose.
 *
 * @details The grasp-to-object transform at pick time stays fixed, so the grasp link follows the object
 * to target_pose plus target_offset. The math is in docs/AGENT_COMMENTS.md.
 *
 * Optional extra candidates turn about the target Z axis inside a yaw window, or place the object
 * upside down. Candidates come out nominal first.
 *
 * | Data Port Name        | Port Type | Object Type                                  |
 * | --------------------- | --------- | -------------------------------------------- |
 * | object_pose           | Input     | geometry_msgs::msg::PoseStamped              |
 * | pick_pose             | Input     | geometry_msgs::msg::PoseStamped              |
 * | target_pose           | Input     | geometry_msgs::msg::PoseStamped              |
 * | target_offset_xyz     | Input     | std::vector<double>                          |
 * | yaw_range_deg         | Input     | double                                       |
 * | yaw_increment_deg     | Input     | double                                       |
 * | include_upright       | Input     | bool                                         |
 * | include_flipped       | Input     | bool                                         |
 * | place_poses           | Output    | std::vector<geometry_msgs::msg::PoseStamped> |
 * | predicted_object_pose | Output    | geometry_msgs::msg::PoseStamped              |
 */
class GeneratePlaceCandidates final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GeneratePlaceCandidates(const std::string& name, const BT::NodeConfiguration& config,
                          const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace grasping_behaviors

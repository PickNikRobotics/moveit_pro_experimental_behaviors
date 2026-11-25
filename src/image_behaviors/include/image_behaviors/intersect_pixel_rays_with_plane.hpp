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

namespace image_behaviors
{
/**
 * @brief Project pixels onto a plane by intersecting their camera rays with it.
 *
 * @details Use it instead of a depth reading where the depth is wrong or missing, for example on a
 * handle or a shiny feature. See docs/AGENT_COMMENTS.md.
 *
 * Pixels are fractions of the image size, as GetCenterFromMask2D outputs them. Rays that miss the
 * plane are dropped, so the output is not index-aligned with the input.
 *
 * reference_pixel projects one extra pixel the same way, to get a reference point on the plane.
 *
 * | Data Port Name  | Port Type | Object Type                                  |
 * | --------------- | --------- | -------------------------------------------- |
 * | camera_info     | Input     | sensor_msgs::msg::CameraInfo                 |
 * | pixel_coords    | Input     | std::vector<geometry_msgs::msg::PointStamped>|
 * | plane_pose      | Input     | geometry_msgs::msg::PoseStamped              |
 * | reference_pixel | Input     | std::vector<double>                          |
 * | output_poses    | Output    | std::vector<geometry_msgs::msg::PoseStamped> |
 * | hit_count       | Output    | int                                          |
 * | reference_pose  | Output    | geometry_msgs::msg::PoseStamped              |
 */
class IntersectPixelRaysWithPlane final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  IntersectPixelRaysWithPlane(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace image_behaviors

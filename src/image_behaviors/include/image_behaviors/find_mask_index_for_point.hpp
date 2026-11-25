// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <memory>
#include <string>

namespace image_behaviors
{

/**
 * @brief Finds the index of the first 2D mask that contains a given point (in normalized image UV coordinates).
 *
 * @details The input PointStamped has x,y in [0,1] representing normalized image UV coordinates.
 * The behavior converts these to pixel coordinates using the image dimensions, then walks the
 * masks in input order and returns the index of the first mask whose pixel value at the point
 * is non-zero. Fails if the point is outside every mask.
 *
 * | Data Port Name  | Port Type | Object Type                                            |
 * | --------------- | --------- | ------------------------------------------------------ |
 * | point           | input     | geometry_msgs::msg::PointStamped                       |
 * | image           | input     | sensor_msgs::msg::Image                                |
 * | masks2d         | input     | std::vector<moveit_studio_vision_msgs::msg::Mask2D>    |
 * | mask_index      | output    | int                                                    |
 */
class FindMaskIndexForPoint final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  FindMaskIndexForPoint(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace image_behaviors

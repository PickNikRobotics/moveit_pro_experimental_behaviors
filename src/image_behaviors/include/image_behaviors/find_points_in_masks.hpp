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
 * @brief Finds which points (in normalized image UV coordinates) fall inside any of the given 2D masks.
 *
 * @details Each input PointStamped has x,y in [0,1] representing normalized image UV coordinates.
 * The behavior converts these to pixel coordinates using the image dimensions, then checks
 * each point against every Mask2D. Points that fall inside at least one mask have their index
 * included in the output vector.
 *
 * | Data Port Name  | Port Type | Object Type                                            |
 * | --------------- | --------- | ------------------------------------------------------ |
 * | points          | input     | std::vector<geometry_msgs::msg::PointStamped>          |
 * | image           | input     | sensor_msgs::msg::Image                                |
 * | masks2d         | input     | std::vector<moveit_studio_vision_msgs::msg::Mask2D>    |
 * | point_indices   | output    | std::vector<int>                                       |
 */
class FindPointsInMasks final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  FindPointsInMasks(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace image_behaviors

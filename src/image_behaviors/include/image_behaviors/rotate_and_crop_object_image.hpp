// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace image_behaviors
{
/**
 * @brief Crop one object out of an image by its mask, and rotate the crop so the +Z axis of the object frame points up.
 *
 * @details The result is a view of the object in a known orientation, for example as input to a
 * vision-language model. How the angle and the crop are found is in docs/AGENT_COMMENTS.md.
 *
 * Pixels outside the mask are black. padding_factor enlarges the square crop so the rotation does
 * not cut the corners of the object.
 *
 * | Data Port Name | Port Type | Object Type                            |
 * | -------------- | --------- | -------------------------------------- |
 * | image          | Input     | sensor_msgs::msg::Image                |
 * | mask           | Input     | moveit_studio_vision_msgs::msg::Mask2D |
 * | object_pose    | Input     | geometry_msgs::msg::PoseStamped        |
 * | camera_info    | Input     | sensor_msgs::msg::CameraInfo           |
 * | padding_factor | Input     | double                                 |
 * | save_image_dir | Input     | std::string                            |
 * | rotated_image  | Output    | sensor_msgs::msg::Image                |
 */
class RotateAndCropObjectImage final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  RotateAndCropObjectImage(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace image_behaviors

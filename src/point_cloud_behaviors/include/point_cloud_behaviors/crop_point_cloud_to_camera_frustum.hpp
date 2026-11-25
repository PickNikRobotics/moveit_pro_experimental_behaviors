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

namespace point_cloud_behaviors
{
/**
 * @brief Keep only the points inside the depth-limited view volume of a camera.
 *
 * @details Drops every point outside the image bounds or outside the range limits along the optical
 * Z axis. Why the limits are planes is in docs/AGENT_COMMENTS.md.
 *
 * Every field of a kept point is kept. The output is unordered, so a consumer that indexes by pixel
 * needs the original cloud instead.
 *
 * | Data Port Name      | Port Type | Object Type                   |
 * | ------------------- | --------- | ----------------------------- |
 * | point_cloud         | Input     | sensor_msgs::msg::PointCloud2 |
 * | camera_info         | Input     | sensor_msgs::msg::CameraInfo  |
 * | min_range           | Input     | double                        |
 * | max_range           | Input     | double                        |
 * | pixel_margin        | Input     | double                        |
 * | point_cloud_cropped | Output    | sensor_msgs::msg::PointCloud2 |
 * | kept_count          | Output    | int                           |
 */
class CropPointCloudToCameraFrustum final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  CropPointCloudToCameraFrustum(const std::string& name, const BT::NodeConfiguration& config,
                                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

}  // namespace point_cloud_behaviors
